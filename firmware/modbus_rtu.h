/*
 * modbus_rtu.h — 可移植 Modbus RTU 从站核心(与上位机 lib/serial/modbus.ts 严格对齐)
 *
 * 目标: TI F280025C(LAUNCHXL-F280025C,SCI-A 经板上 XDS110 USB 回传通道)
 * 通讯协议: ai_motor_control/docs/modbus_rtu_protocol.md
 *           宿主上位机实现: ai_motor_control/lib/serial/modbus.ts
 *
 * 本模块【不依赖任何硬件头文件】,仅使用 stdint/stdbool/string,
 * 可以在 x86 上用 gcc 编译做主机侧单元测试,再无缝移植到 C2000(CCS/C2000Ware)。
 *
 * 协议要点(与上位机必须一致,否则无法通讯):
 *   - 帧: [slave, fc, ...PDU, CRC_lo, CRC_hi]; CRC-16/MODBUS(0xA001, init 0xFFFF, 低字节在前)
 *   - 功能码: 01 读线圈 / 03 读保持寄存器 / 05 写单线圈 / 06 写单寄存器 /
 *             0F(15) 写多线圈 / 10(16) 写多寄存器
 *   - 线圈: 独立位地址空间,uint8_t 数组 LSB 优先(coil addr -> bit(addr&7) 于 byte(addr>>3))
 *   - 保持寄存器: u16 数组;float32 占 2 寄存器,big-endian(高字存低地址)
 *   - 正常响应: [slave, fc, ...] + CRC;读 0x03 响应 [slave, 0x03, byteCount, hi, lo, ...] + CRC
 *   - 异常响应: [slave, fc|0x80, code] + CRC;code: 1=非法功能 2=非法数据地址 3=非法数据值
 */
#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdint.h>
#include <stdbool.h>

/* TI C28x 目标: CPU/char 为 16 位,标准头不提供 uint8_t。
 * driverlib(hw_types.h)已 typedef uint16_t uint8_t;若其未包含,
 * 则以 16 位无符号 char 作为"字节"容器(取值 0..255),两层尺寸一致。 */
#if defined(__TI_COMPILER_VERSION__) && !defined(HW_TYPES_H)
typedef unsigned char uint8_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ── 地址空间(与 lib/serial/modbus.ts 的 ADDR / 协议文档对齐) ─────────── */
/* 保持寄存器区(Cfg) 0x0000..0x03FF:全局控制 + PID 速度环 + PID 电流环 + ALGO2 预留 */
#define MB_HOLDING_CFG_SIZE          0x0400u        /* 0x0000..0x03FF */

/* 保持寄存器区(状态/遥测,只读) 0x1000..0x10FF */
#define MB_HOLDING_ST_BASE           0x1000u
#define MB_HOLDING_ST_SIZE           0x0100u        /* 0x1000..0x10FF */

/* 线圈区(独立地址空间,共 32 个线圈,即 4 字节) */
#define MB_COIL_COUNT                0x0020u        /* 32 coils */
#define MB_COIL_BYTE_COUNT           4u             /* 32/8 */

/* 寄存器地址常量(镜像 lib/serial/modbus.ts ADDR) */
#define MB_ADDR_SPEED_SETPOINT       0x0000u
#define MB_ADDR_CONTROL_MODE         0x0001u
#define MB_ADDR_PID_SPD_KP           0x0100u
#define MB_ADDR_PID_SPD_KI           0x0102u
#define MB_ADDR_PID_SPD_KD           0x0104u
#define MB_ADDR_PID_SPD_KD_N         0x0106u
#define MB_ADDR_PID_SPD_KI_UPLIM     0x0108u
#define MB_ADDR_PID_SPD_KI_LOWLIM    0x010Au
#define MB_ADDR_PID_SPD_KI_OUT_LIM   0x010Cu
#define MB_ADDR_PID_SPD_OUT_LIM      0x010Eu
#define MB_ADDR_PID_CUR_KP           0x0110u
#define MB_ADDR_PID_CUR_KI           0x0112u
#define MB_ADDR_PID_CUR_KD           0x0114u
#define MB_ADDR_PID_CUR_KD_N         0x0116u
#define MB_ADDR_PID_CUR_KI_UPLIM     0x0118u
#define MB_ADDR_PID_CUR_KI_LOWLIM    0x011Au
#define MB_ADDR_PID_CUR_KI_OUT_LIM   0x011Cu
#define MB_ADDR_PID_CUR_OUT_LIM      0x011Eu
#define MB_ADDR_ALGO2_P0             0x0300u
#define MB_ADDR_ACTUAL_SPEED         0x1000u        /* f32 */
#define MB_ADDR_ACTUAL_CURRENT       0x1002u        /* f32 */
#define MB_ADDR_FAULT_CODE           0x1004u        /* u16 */
#define MB_ADDR_STATUS_FLAGS         0x1005u        /* u16 */

/* 线圈地址常量(镜像 modbus.ts ADDR.COIL_*) */
#define MB_COIL_MOTOR_EN             0x0000u
#define MB_COIL_FAULT_RESET          0x0001u
#define MB_COIL_EMERGENCY_STOP       0x0002u

/* ── 波形区(电流采样缓冲,5kHz×100 点)───────────────────────────── */
/* 缓冲数据: 0x2000..0x20C7 共 200 寄存器 = 100 个 float32(big-endian,高字低地址)。
 * 注意: 0x03 单次最多读 125 寄存器,200 寄存器需分两次(0x2000×125 + 0x207D×75)。 */
#define MB_WAVE_BASE                 0x2000u
#define MB_WAVE_COUNT                100u         /* 每批样本数 */
#define MB_WAVE_BATCH_REG            0x2200u      /* 批次号(只读,发布一批 +1) */
#define MB_WAVE_STATUS_REG           0x2201u      /* bit0=新批次就绪;0x03 读到后自动清位,也可 0x06 写 0 清 */

/* ── 帧长边界 ─────────────────────────────────────────────────────── */
#define MB_MAX_FRAME                 256u           /* 最长请求帧(0x10/0x0F 多写) */
#define MB_MAX_RESP                  264u           /* 最长响应帧(0x03 读 125 寄存器) */

/* ── 从站数据结构 ──────────────────────────────────────────────────── */
typedef struct {
    uint8_t  slaveAddr;                             /* 从站地址(默认 0x01) */

    /* 线圈:每字节 8 个,LSB 优先;coil addr -> bit(addr&7) of byte(addr>>3) */
    uint8_t  coils[MB_COIL_BYTE_COUNT];

    /* 保持寄存器(两段独立 u16 区,地址不相邻:0x0000..0x03FF / 0x1000..0x10FF) */
    uint16_t cfg[MB_HOLDING_CFG_SIZE];
    uint16_t st[MB_HOLDING_ST_SIZE];

    /* 接收帧累积缓冲 */
    uint8_t  rx[MB_MAX_FRAME];
    uint16_t rxLen;

    /* 响应帧缓冲(一次只暂存一条) */
    uint8_t  resp[MB_MAX_RESP];
    uint16_t respLen;

    /* ── 波形双缓冲: waveWrite 正在写,waveFreeze 冻结对总线发布 ──
     * 攒满 100 点先冻结(整体发布)再切换,主站任何时刻读到的是完整一批。 */
    float    waveBuf[2][MB_WAVE_COUNT];
    uint16_t waveCount;            /* 当前写缓冲已攒点数 */
    uint8_t  waveWrite;            /* 正在写的缓冲 0/1 */
    uint8_t  waveFreeze;           /* 冻结对总线发布 0/1 */
    uint16_t waveBatch;            /* 批次号(每冻结一批 +1,循环回绕) */
    uint16_t waveStatus;           /* bit0=新批次就绪(未读) */
} ModbusSlave;

/* 喂字节后的返回状态 */
typedef enum {
    MB_RESULT_NONE = 0,     /* 帧未完整 / 无需回复(待继续喂字节或该帧不属于本站) */
    MB_RESULT_RESPONSE      /* 一条完整合法帧已处理,resp[0..respLen-1] 待发送 */
} ModbusResult;

/* ── 主 API ───────────────────────────────────────────────────────── */
void         ModbusSlave_init(ModbusSlave *s, uint8_t slaveAddr);

/* 喂入单个接收字节。返回 MB_RESULT_RESPONSE 时,s->resp / s->respLen 即待发送数据。 */
ModbusResult ModbusSlave_feedByte(ModbusSlave *s, uint8_t byte);

/* 清空接收累积(用于帧空闲超时,防止残缺帧卡死)。 */
void         ModbusSlave_resetFrame(ModbusSlave *s);

/* CRC-16/MODBUS(与上位机 crc16() 一致):init 0xFFFF, poly 0xA001, 返回 0..0xFFFF */
uint16_t     ModbusCrc16(const uint8_t *data, uint16_t len);

/* ── 数据访问辅助(供控制环 / 监控用)────────────────────────────── */
void         ModbusSlave_setCoil(ModbusSlave *s, uint16_t addr, bool on);
bool         ModbusSlave_getCoil(const ModbusSlave *s, uint16_t addr);

uint16_t     ModbusSlave_getReg(const ModbusSlave *s, uint16_t addr);
void         ModbusSlave_setReg(ModbusSlave *s, uint16_t addr, uint16_t val);

/* float32 big-endian:高字存 addr,低字存 addr+1(与上位机 float32ToRegs 一致) */
void         ModbusSlave_setFloat(ModbusSlave *s, uint16_t addr, float f);
float        ModbusSlave_getFloat(const ModbusSlave *s, uint16_t addr);

/* ── 波形 API(数据生产者=采样/生成 ISR,消费者=0x03 总线读)────────── */
/* 推入一个采样点(期望 5kHz 调用率);攒满 MB_WAVE_COUNT 点自动冻结切换并发布批次 */
void         ModbusSlave_pushWaveSample(ModbusSlave *s, float sample);
/* 读冻结批次第 idx 个寄存器(0..199 = 100 个 float32,高字在前),越界返回 0 */
uint16_t     ModbusSlave_getWaveReg(const ModbusSlave *s, uint16_t idx);
/* 读冻结批次号(0x2200 数据源) */
uint16_t     ModbusSlave_getWaveBatch(const ModbusSlave *s);
/* 读 0x2201 bit0 状态(总线读到后核心自动清位;上位机也可 0x06 写 0 清) */
uint16_t     ModbusSlave_getWaveStatus(const ModbusSlave *s);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_H */
