/*
 * F280025C_modbus_slave.c — TI TMS320F280025C(LAUNCHXL-F280025C) Modbus RTU 从站
 *
 * 将可移植核心 modbus_rtu.c 挂接到 C2000Ware driverlib 的 SCI-A(UART)上:
 *   - SCI-A 经板上 XDS110 USB 回传通道(虚拟串口)与上位机通讯
 *   - RX FIFO 中断逐字节喂入 modbus_rtu 核心,主循环取出响应并发送
 *   - 预留控制环挂点:更新 ACTUAL_SPEED / ACTUAL_CURRENT,响应线圈开关
 *
 * 依赖: C2000Ware (driverlib, f28002x 器件支持库), 目标为默认 100 MHz SYSCLK。
 *
 * ⚠️ 使用前请在 README.md 核对以下两处(因编译器/板级仓库差异而异):
 *    1) SCI-A 引脚映射(TX=GPIO29 / RX=GPIO28,见板卡原理图;C2000Ware 6.x
 *       宏名为 GPIO_29_SCIA_TX('GPIO_29_SCITXDA') / GPIO_28_SCIA_RX('GPIO_28_SCIRXDA'))
 *    2) 波特率默认 1500000;若 XDS110 回传不稳,改 MODBUS_BAUD 为 115200
 *
 * 兼容性: 面向 C2000Ware 6.x driverlib(SCI_setConfig / SCI_writeCharBlockingFIFO 等),
 *         毫秒计时由 CPUTimer0 提供(SysCtl_getTick 在新版已移除)。
 */
#include "driverlib.h"
#include "device.h"
#include "sci.h"
#include "gpio.h"
#include "interrupt.h"
#include "sysctl.h"
#include "cputimer.h"
#include "math.h"

#include "modbus_rtu.h"

/* ── 配置(唯一需要按项目调整的地方)────────────────────────────── */
#define MODBUS_SLAVE_ADDR   0x01u        /* 从站地址(默认 0x01) */
/* 波特率 = LSPCLK(25MHz)/(8*(BRR+1)),只能取整分频。
 * 注意 driverlib 先整除后减一: divider = floor(25e6/(8*baud)) - 1。
 *   baud=1041667 -> floor(2.99999)=2 -> div=1 -> 实际 1,562,500(错!已踩坑)
 *   baud=1041666 -> floor(3.00000)=3 -> div=2 -> 实际 1,041,666 (精确;实测此处大帧≈93%,略边缘)
 *   baud=781250  -> floor(4.000)=4 -> div=3 -> 实际 781,250 (精确;实测稳定 100%,本次采用)★
 * 921600 无整数解(最近真值 781250 / 1041666,偏差≥13% 必然断链)。
 * 采用 781250: 仍是 115200 的 6.8 倍,5kHz×4B 波形(20KB/s)余量充足。 */
#define MODBUS_BAUD         781250u

/* ── 波形生成参数 ──────────────────────────────────────────────── */
#define SAMPLE_RATE_HZ       5000u       /* 上传采样率(0x2000 波形区,每 200µs 1 点) */
#define GEN_RATE_HZ          30000u      /* 内部生成速率: 6 点取 1 上传(cos(600π t) 正弦) */

/* ── 全局从站实例(静态放置,~2.6KB RAM,勿放栈上)─────────────────── */
static ModbusSlave g_rtu;

/* ── 响应发送双缓冲(修复:旧版单缓冲会在发送期间被新请求覆盖,
 *        导致大帧尾段变乱(NaN/乱数/丢帧)——已定位并固定)────── */
static uint8_t  g_txBuf[2][MB_MAX_RESP];       /* ISR 生产、主循环发送,双槽乒乓 */
static uint8_t  g_txSendSlot   = 0u;           /* 当前发送槽(主循环/ISR共用) */
static volatile uint16_t g_txLen = 0u;         /* 待发送长度;0=无已发布响应 */

/* 单位毫秒的单调计时(由 CPUTimer0 1ms 中断提供;用于帧空闲超时) */
static volatile uint32_t g_tickMs = 0u;
static volatile uint32_t g_lastRxMs = 0u;

/* ── 1ms 系统节拍(CPUTimer0) ──────────────────────────────────── */
__interrupt void CPUTIMER0_ISR(void)
{
    g_tickMs += 1u;
    /* 组1 中断(INT_TIMER0)需应答 */
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

static void tick_init(void)
{
    CPUTimer_setPeriod(CPUTIMER0_BASE, (DEVICE_SYSCLK_FREQ / 1000u) - 1u);
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0u);      /* ÷1(SYSCLK) */
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);

    Interrupt_register(INT_TIMER0, &CPUTIMER0_ISR);
    Interrupt_enable(INT_TIMER0);

    CPUTimer_startTimer(CPUTIMER0_BASE);
}

/* ── 波形生成(CPUTimer1 @30kHz): cos(600π t) 正弦,每 6 点抽 1 上传 ── */
static volatile uint16_t g_genPhase = 0u;   /* 30kHz 采样相角(0..99 周期回绕) */

__interrupt void TIMER1_30K_ISR(void)
{
    if ((g_genPhase % 6u) == 0u) {           /* 每 200µs 一个点 = 5kHz 上传率 */
        /* cos(600π t), t = phase/30000 → 相角 = π·phase/50 */
        float v = cosf(3.14159265f * (float)g_genPhase / 50.0f);
        ModbusSlave_pushWaveSample(&g_rtu, v);
    }
    g_genPhase = (uint16_t)((g_genPhase + 1u) % 100u);
    /* INT_TIMER1 = PIE 组2 */
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2);
}

static void gen30000_init(void)
{
    CPUTimer_setPeriod(CPUTIMER1_BASE, (DEVICE_SYSCLK_FREQ / GEN_RATE_HZ) - 1u);
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0u);
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    CPUTimer_setEmulationMode(CPUTIMER1_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_enableInterrupt(CPUTIMER1_BASE);

    Interrupt_register(INT_TIMER1, &TIMER1_30K_ISR);
    Interrupt_enable(INT_TIMER1);

    CPUTimer_startTimer(CPUTIMER1_BASE);
}

/* ── SCI-A 引脚配置 ────────────────────────────────────────────── */
/* LAUNCHXL-F280025C 板上 XDS110 回传接 SCI-A:
 *   SCITXDA = GPIO29(GPIO_29_SCIA_TX,旧名 GPIO_29_SCITXDA)
 *   SCIRXDA = GPIO28(GPIO_28_SCIA_RX,旧名 GPIO_28_SCIRXDA)
 * 与 C2000Ware f28002x pin_map.h 及官方 launchxl demo 一致。 */
static void sci_gpio_config(void)
{
    /* 引脚复用到 SCI-A */
    GPIO_setPinConfig(GPIO_29_SCIA_TX);
    GPIO_setPinConfig(GPIO_28_SCIA_RX);

    GPIO_setDirectionMode(29u, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(28u, GPIO_DIR_MODE_IN);

    GPIO_setPadConfig(29u, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(28u, GPIO_PIN_TYPE_STD);

    /* RX 输入无需同步滤波(串口信号本身已整形) */
    GPIO_setQualificationMode(28u, GPIO_QUAL_ASYNC);
}

/* ── SCI-A 模块初始化(8N1, MODBUS_BAUD)────────────────────────── */
static void sci_a_init(void)
{
    SCI_performSoftwareReset(SCIA_BASE);

    /* 数据格式 8N1;波特率按 LSPCLK(=SYSCLK/4)计算,须与实际时钟一致 */
    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, MODBUS_BAUD,
                  (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE));
    SCI_resetChannels(SCIA_BASE);

    /* FIFO: RX 收到 1 字节即触发中断(低延迟);TX 断开空中断(主循环阻塞写) */
    SCI_enableFIFO(SCIA_BASE);
    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX0, SCI_FIFO_RX1);

    SCI_resetTxFIFO(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);
    SCI_enableInterrupt(SCIA_BASE, SCI_INT_RXFF);

    SCI_enableModule(SCIA_BASE);
}

/* ── RX FIFO 中断服务 ──────────────────────────────────────────── */
__interrupt void SCIA_RX_ISR(void)
{
    /* 排空 RX FIFO */
    while (SCI_getRxFIFOStatus(SCIA_BASE) != SCI_FIFO_RX0) {
        uint16_t ch = SCI_readCharNonBlocking(SCIA_BASE);
        uint8_t byte = (uint8_t)(ch & 0xFFu);
        g_lastRxMs = g_tickMs;   /* ms;仅供空闲帧超时参考 */

        if (ModbusSlave_feedByte(&g_rtu, byte) == MB_RESULT_RESPONSE) {
            /* 始终拷入"非发送槽",绝不动 pump 正在发的槽;拷完后发布长度+换槽 */
            uint8_t slot = (uint8_t)(g_txSendSlot ^ 1u);
            uint16_t len = g_rtu.respLen;
            for (uint16_t i = 0u; i < len; i++) g_txBuf[slot][i] = g_rtu.resp[i];
            g_txLen = len;          /* 全部拷贝完成后再发布 */
            g_txSendSlot = slot;    /* 新槽成为下轮发送目标 */
        }
    }

    /* 清 RX 溢出与中断标志 */
    SCI_clearOverflowStatus(SCIA_BASE);
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);

    /* PIE 组 9 应答: INT_SCIA_RX(9.1);不清 ACK 该组后续中断被屏蔽 */
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

/* ── 主循环发送响应(只发 g_txSendSlot 指向槽;期间新响应由 ISR 落另一槽) ── */
static void pump_tx(void)
{
    if (g_txLen == 0u) return;
    uint16_t len = g_txLen;
    uint8_t  s   = g_txSendSlot;
    for (uint16_t i = 0u; i < len; i++) {
        SCI_writeCharBlockingFIFO(SCIA_BASE, g_txBuf[s][i]);
    }
    /* 若发送期间没有新响应发布,标记空闲;否则保留(新响应已换槽,下轮自会发送) */
    if (g_txLen == len) g_txLen = 0u;
}

/* ── 控制环挂点(占位)───────────────────────────────────────────── */
/* 在这里读取线圈/寄存器驱动电机,并把实际转速/电流回写至状态区,供上位机 0x03 轮询 */
static void control_loop_tick(void)
{
    /* 示例:把"转速设定"当作"实际转速"回显(便于先验证通讯链路,无电机也能看到非零)
     * 正式控制环请改由电流采样 / 编码器反馈更新 ACTUAL_SPEED / ACTUAL_CURRENT */
    uint16_t set = ModbusSlave_getReg(&g_rtu, MB_ADDR_SPEED_SETPOINT);
    ModbusSlave_setFloat(&g_rtu, MB_ADDR_ACTUAL_SPEED, (float)set);
    /* ModbusSlave_setFloat(&g_rtu, MB_ADDR_ACTUAL_CURRENT, ia_sample); */

    /* 线圈示例:急停时清转速设定,防止上电即转 */
    if (ModbusSlave_getCoil(&g_rtu, MB_COIL_EMERGENCY_STOP)) {
        ModbusSlave_setReg(&g_rtu, MB_ADDR_SPEED_SETPOINT, 0u);
    }
}

/* ── 主函数 ────────────────────────────────────────────────────── */
int main(void)
{
    /* 器件/中断/向量表基础初始化(C2000Ware 标准) */
    Device_init();
    Device_initGPIO();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    /* 时钟: 由你的工程 Board_init/_MBOARD 或下面 SysCtl 设置。
     * 在此使用默认 SYSCLK;若需 100 MHz 且 DEVICE_SYSCLK_FREQ 相符,可保持默认。
     * 若要显式设置:PLL 配置请参照 C2000Ware f28002x 模板的 syscfg/board 配置,
     * 勿写死导致 SCI 波特率(PLL 依赖)算错。 */
    /* SysCtl_setClock( 依你的 syscfg ); */

    sci_gpio_config();
    sci_a_init();
    tick_init();
    gen30000_init();

    ModbusSlave_init(&g_rtu, MODBUS_SLAVE_ADDR);

    /* 上电默认(协议文档 §8: 所有参数由下位机在初始化时赋值) */
    ModbusSlave_setReg(&g_rtu, MB_ADDR_SPEED_SETPOINT, 0u);
    ModbusSlave_setReg(&g_rtu, MB_ADDR_CONTROL_MODE, 0u);
    ModbusSlave_setFloat(&g_rtu, MB_ADDR_ACTUAL_SPEED, 0.0f);
    ModbusSlave_setFloat(&g_rtu, MB_ADDR_ACTUAL_CURRENT, 0.0f);

    /* 注册 SCI-A RX 中断 */
    Interrupt_register(INT_SCIA_RX, &SCIA_RX_ISR);
    Interrupt_enable(INT_SCIA_RX);

    /* 使能全局中断 */
    EINT;

    for (;;) {
        pump_tx();                 /* 发送待发响应 */

        /* 帧空闲看门狗: 若距最后收到的字节超过 ~10ms,视为残缺帧结束,复位接收 */
        if (g_tickMs - g_lastRxMs > 10u) {
            ModbusSlave_resetFrame(&g_rtu);
        }

        control_loop_tick();       /* 控制环挂点 */

        /* 可加 MCU 节拍延时,避免空转 */
    }
}
