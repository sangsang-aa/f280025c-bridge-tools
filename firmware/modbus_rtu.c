/*
 * modbus_rtu.c — Modbus RTU 从站核心实现(与上位机 lib/serial/modbus.ts 严格对齐)
 *
 * 实现要点:
 *   - 按字节喂入(ModbusSlave_feedByte),由硬件层(SCI RX 中断)逐个喂字节
 *   - 帧长判定: 0x01/0x03/0x05/0x06 固定 8 字节; 0x0F/0x10 = 9 + byteCount 字节
 *   - 收到整帧后校验 CRC,再按功能码分发,生成正常/异常响应
 *   - 寄存器/线圈采用分段映射,非法地址(跨段/越界)返回 0x02 异常
 */
#include "modbus_rtu.h"
#include <string.h>

/* ── 内部工具: 判断地址属于哪个读/写区 ───────────────────────────── */
/* 返回区编号: 0=无(非法)  1=cfg 段(0x0000..0x03FF)  2=status 段(0x1000..0x10FF)
 *             3=波形缓冲(0x2000..0x20C7)  4=波形批次/状态(0x2200..0x2201) */
static int region_of(uint16_t addr)
{
    if (addr < MB_HOLDING_CFG_SIZE)
        return 1;                                  /* 0x0000 .. 0x03FF */
    if (addr >= MB_HOLDING_ST_BASE &&
        addr < (MB_HOLDING_ST_BASE + MB_HOLDING_ST_SIZE))
        return 2;                                  /* 0x1000 .. 0x10FF */
    if (addr >= MB_WAVE_BASE &&
        addr < (MB_WAVE_BASE + MB_WAVE_COUNT * 2u))
        return 3;                                  /* 0x2000 .. 0x20C7 */
    if (addr == MB_WAVE_BATCH_REG || addr == MB_WAVE_STATUS_REG)
        return 4;                                  /* 0x2200 .. 0x2201 */
    return 0;
}

/* 取可变寄存器指针(可写);3/4 区只读返回 NULL(状态位写入做特判) */
static uint16_t *reg_ptr(ModbusSlave *s, uint16_t addr)
{
    int r = region_of(addr);
    if (r == 1) return &s->cfg[addr];
    if (r == 2) return &s->st[addr - MB_HOLDING_ST_BASE];
    (void)s;
    return NULL;
}

/* 只读寄存器逐口求值: 波形/批次/状态区无连续指针,按值返回(高字在前 big-endian) */
static uint16_t reg_read16(const ModbusSlave *s, uint16_t addr)
{
    int r = region_of(addr);
    if (r == 1) return s->cfg[addr];
    if (r == 2) return s->st[addr - MB_HOLDING_ST_BASE];
    if (r == 3) return ModbusSlave_getWaveReg(s, (uint16_t)(addr - MB_WAVE_BASE));
    if (addr == MB_WAVE_BATCH_REG)  return s->waveBatch;
    if (addr == MB_WAVE_STATUS_REG) return s->waveStatus;
    return 0u;
}

/* 校验连续 count 个寄存器地址是否完全落在【同一段】内(不允许跨段) */
static bool range_valid(const ModbusSlave *s, uint16_t addr, uint16_t count)
{
    (void)s;
    if (count == 0u) return false;
    int r0 = region_of(addr);
    if (r0 == 0) return false;
    int rN = region_of((uint16_t)(addr + count - 1u));
    return (rN == r0);
}

/* ── CRC-16/MODBUS ──────────────────────────────────────────────── */
uint16_t ModbusCrc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 1u)
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ── 请求帧长度判定 ─────────────────────────────────────────────── */
/* 0x0F/0x10 长度由第 7 字节(byteCount)决定;其余功能码(01/03/05/06 及未知码)按固定 8 字节端。
 * 未知功能码也按 8 字节端,以便触发"非法功能"异常响应。 */
static uint16_t expected_len(const ModbusSlave *s)
{
    if (s->rxLen < 6u) return 0u;                  /* 至少要有 slave+fc+4 字节地址/数量 */

    uint8_t fc = s->rx[1];
    if (fc == 0x0Fu || fc == 0x10u) {
        if (s->rxLen >= 7u) {
            uint16_t bc = s->rx[6];
            return (uint16_t)(9u + bc);            /* 7 + byteCount + 2(CRC) */
        }
        return 0u;
    }

    return 8u;
}

/* ── 响应构建辅助 ────────────────────────────────────────────────── */
static void append_crc(ModbusSlave *s)
{
    uint16_t crc = ModbusCrc16(s->resp, s->respLen);
    s->resp[s->respLen++] = (uint8_t)(crc & 0xFFu);
    s->resp[s->respLen++] = (uint8_t)((crc >> 8) & 0xFFu);
}

/* 单写请求回显(0x05/0x06): 直接回显请求前 6 字节 [slave,fc,addrH,addrL,valH,valL] */
static ModbusResult echo_request(ModbusSlave *s)
{
    memcpy(s->resp, s->rx, 6u);
    s->respLen = 6u;
    append_crc(s);
    return MB_RESULT_RESPONSE;
}

/* 异常响应 [slave, fc|0x80, code] + CRC */
static ModbusResult resp_exception(ModbusSlave *s, uint8_t fc, uint8_t code)
{
    s->resp[0] = s->slaveAddr;
    s->resp[1] = (uint8_t)(fc | 0x80u);
    s->resp[2] = code;
    s->respLen = 3u;
    append_crc(s);
    return MB_RESULT_RESPONSE;
}

/* ── 各功能码处理(请求已通过 CRC 校验,且地址匹配本站) ───────────── */

/* 0x01 读线圈 */
static ModbusResult fc_read_coils(ModbusSlave *s)
{
    uint16_t start = (uint16_t)((s->rx[2] << 8) | s->rx[3]);
    uint16_t count = (uint16_t)((s->rx[4] << 8) | s->rx[5]);

    if (count == 0u || count > MB_COIL_COUNT)
        return resp_exception(s, 0x01u, 0x03u);    /* 非法数据值 */
    if (start >= MB_COIL_COUNT ||
        (uint16_t)(start + count) > MB_COIL_COUNT)
        return resp_exception(s, 0x01u, 0x02u);    /* 非法数据地址 */

    uint16_t byteCount = (uint16_t)((count + 7u) / 8u);
    s->resp[0] = s->slaveAddr;
    s->resp[1] = 0x01u;
    s->resp[2] = (uint8_t)byteCount;
    memset(&s->resp[3], 0u, byteCount);

    for (uint16_t k = 0u; k < count; k++) {
        uint16_t a = start + k;
        if (s->coils[a >> 3] & (uint8_t)(1u << (a & 7u)))
            s->resp[3 + (k >> 3)] |= (uint8_t)(1u << (k & 7u)); /* LSB 优先 */
    }
    s->respLen = (uint16_t)(3u + byteCount);
    append_crc(s);
    return MB_RESULT_RESPONSE;
}

/* 0x03 读保持寄存器 */
static ModbusResult fc_read_holding(ModbusSlave *s)
{
    uint16_t start = (uint16_t)((s->rx[2] << 8) | s->rx[3]);
    uint16_t count = (uint16_t)((s->rx[4] << 8) | s->rx[5]);

    if (count == 0u || count > 125u)
        return resp_exception(s, 0x03u, 0x03u);    /* 非法数据值 */
    if (!range_valid(s, start, count))
        return resp_exception(s, 0x03u, 0x02u);    /* 非法数据地址 */

    uint16_t byteCount = (uint16_t)(count * 2u);
    s->resp[0] = s->slaveAddr;
    s->resp[1] = 0x03u;
    s->resp[2] = (uint8_t)byteCount;

    /* 波形区(0x2000..0x20C7)整体快照期间关中断,防止 30kHz 采样 ISR
     * 在拷贝中途切换冻结缓冲导致撕裂(仅 TI 目标需要;x86 单测为 no-op)。 */
#if defined(__TI_COMPILER_VERSION__)
    asm(" DINT");
#endif
    for (uint16_t i = 0u; i < count; i++) {
        uint16_t v = reg_read16(s, (uint16_t)(start + i));
        s->resp[3u + i * 2u]             = (uint8_t)(v >> 8);
        s->resp[3u + i * 2u + 1u]        = (uint8_t)(v & 0xFFu);
    }
#if defined(__TI_COMPILER_VERSION__)
    asm(" EINT");
#endif

    /* 读 0x2201 状态寄存器:读完即清 bit0(新批次信号是"读后清"语义) */
    if (start <= MB_WAVE_STATUS_REG &&
        (uint16_t)(start + count) > MB_WAVE_STATUS_REG) {
        if (s->waveStatus & 0x0001u) s->waveStatus &= (uint16_t)~0x0001u;
    }

    s->respLen = (uint16_t)(3u + byteCount);
    append_crc(s);
    return MB_RESULT_RESPONSE;
}

/* 0x05 写单线圈 */
static ModbusResult fc_write_coil(ModbusSlave *s)
{
    uint16_t start = (uint16_t)((s->rx[2] << 8) | s->rx[3]);
    uint16_t val   = (uint16_t)((s->rx[4] << 8) | s->rx[5]);

    if (start >= MB_COIL_COUNT)
        return resp_exception(s, 0x05u, 0x02u);
    if (val != 0xFF00u && val != 0x0000u)
        return resp_exception(s, 0x05u, 0x03u);    /* 非 0xFF00/0x0000 */

    ModbusSlave_setCoil(s, start, (val == 0xFF00u));
    return echo_request(s);
}

/* 0x06 写单寄存器 */
static ModbusResult fc_write_reg(ModbusSlave *s)
{
    uint16_t start = (uint16_t)((s->rx[2] << 8) | s->rx[3]);
    uint16_t val   = (uint16_t)((s->rx[4] << 8) | s->rx[5]);

    uint16_t *p = reg_ptr(s, start);
    if (!p) {
        /* 0x2201 状态位: 上位机可写 0 清除(与"读后清位"等价) */
        if (start == MB_WAVE_STATUS_REG && val == 0u) {
            s->waveStatus = 0u;
            return echo_request(s);
        }
        return resp_exception(s, 0x06u, 0x02u);    /* 非法数据地址 */
    }
    *p = val;
    return echo_request(s);
}

/* 0x0F 写多线圈 */
static ModbusResult fc_write_multi_coils(ModbusSlave *s)
{
    uint16_t start     = (uint16_t)((s->rx[2] << 8) | s->rx[3]);
    uint16_t count     = (uint16_t)((s->rx[4] << 8) | s->rx[5]);
    uint8_t  byteCount = s->rx[6];

    if (count == 0u || count > MB_COIL_COUNT)
        return resp_exception(s, 0x0Fu, 0x03u);
    if (start >= MB_COIL_COUNT ||
        (uint16_t)(start + count) > MB_COIL_COUNT)
        return resp_exception(s, 0x0Fu, 0x02u);

    if (byteCount != (uint8_t)((count + 7u) / 8u))
        return resp_exception(s, 0x0Fu, 0x03u);

    for (uint16_t k = 0u; k < count; k++) {
        uint8_t b = s->rx[7u + (k >> 3)];
        ModbusSlave_setCoil(s, (uint16_t)(start + k),
                            ((b >> (uint8_t)(k & 7u)) & 1u) != 0u);
    }

    /* 响应 [slave, 0x0F, addrH, addrL, qtyH, qtyL] + CRC */
    s->resp[0] = s->slaveAddr;
    s->resp[1] = 0x0Fu;
    s->resp[2] = s->rx[2];
    s->resp[3] = s->rx[3];
    s->resp[4] = s->rx[4];
    s->resp[5] = s->rx[5];
    s->respLen = 6u;
    append_crc(s);
    return MB_RESULT_RESPONSE;
}

/* 0x10 写多寄存器 */
static ModbusResult fc_write_multi_regs(ModbusSlave *s)
{
    uint16_t start     = (uint16_t)((s->rx[2] << 8) | s->rx[3]);
    uint16_t count     = (uint16_t)((s->rx[4] << 8) | s->rx[5]);
    uint8_t  byteCount = s->rx[6];

    if (count == 0u || count > 123u)
        return resp_exception(s, 0x10u, 0x03u);
    if (byteCount != (uint8_t)(count * 2u))
        return resp_exception(s, 0x10u, 0x03u);
    if (!range_valid(s, start, count))
        return resp_exception(s, 0x10u, 0x02u);

    for (uint16_t i = 0u; i < count; i++) {
        uint16_t v = (uint16_t)((s->rx[7u + i * 2u] << 8) | s->rx[7u + i * 2u + 1u]);
        *reg_ptr(s, (uint16_t)(start + i)) = v;
    }

    /* 响应 [slave, 0x10, addrH, addrL, qtyH, qtyL] + CRC */
    s->resp[0] = s->slaveAddr;
    s->resp[1] = 0x10u;
    s->resp[2] = s->rx[2];
    s->resp[3] = s->rx[3];
    s->resp[4] = s->rx[4];
    s->resp[5] = s->rx[5];
    s->respLen = 6u;
    append_crc(s);
    return MB_RESULT_RESPONSE;
}

/* ── 帧处理分发 ──────────────────────────────────────────────────── */
static ModbusResult process_frame(ModbusSlave *s)
{
    uint8_t addr = s->rx[0];
    uint8_t fc   = s->rx[1];

    /* 仅响应本站地址;广播地址 0 或其它地址一律静默(不回包) */
    if (addr != s->slaveAddr)
        return MB_RESULT_NONE;

    switch (fc) {
        case 0x01u: return fc_read_coils(s);
        case 0x03u: return fc_read_holding(s);
        case 0x05u: return fc_write_coil(s);
        case 0x06u: return fc_write_reg(s);
        case 0x0Fu: return fc_write_multi_coils(s);
        case 0x10u: return fc_write_multi_regs(s);
        default:    return resp_exception(s, fc, 0x01u);  /* 非法功能码 */
    }
}

/* ── 主 API 实现 ────────────────────────────────────────────────── */
void ModbusSlave_init(ModbusSlave *s, uint8_t slaveAddr)
{
    memset(s, 0, sizeof(*s));
    s->slaveAddr = slaveAddr;
}

ModbusResult ModbusSlave_feedByte(ModbusSlave *s, uint8_t byte)
{
    if (s->rxLen >= MB_MAX_FRAME) {
        ModbusSlave_resetFrame(s);
        return MB_RESULT_NONE;
    }

    s->rx[s->rxLen++] = byte;

    uint16_t exp = expected_len(s);
    if (exp == 0u) return MB_RESULT_NONE;                 /* 字节还不够 */
    if (exp > MB_MAX_FRAME) {
        ModbusSlave_resetFrame(s);
        return MB_RESULT_NONE;
    }
    if (s->rxLen < exp) return MB_RESULT_NONE;            /* 继续等待补齐 */

    /* 帧已完整(s->rxLen == exp):校验 CRC(最后 2 字节,低字节在前) */
    uint16_t crcCalc = ModbusCrc16(s->rx, (uint16_t)(s->rxLen - 2u));
    uint16_t crcRecv = (uint16_t)s->rx[s->rxLen - 2u] |
                       ((uint16_t)s->rx[s->rxLen - 1u] << 8);

    ModbusResult result = MB_RESULT_NONE;
    if (crcCalc == crcRecv)
        result = process_frame(s);

    ModbusSlave_resetFrame(s);          /* 清空累积,准备下一帧 */
    return result;
}

void ModbusSlave_resetFrame(ModbusSlave *s)
{
    s->rxLen = 0u;
}

/* ── 数据访问辅助 ────────────────────────────────────────────────── */
void ModbusSlave_setCoil(ModbusSlave *s, uint16_t addr, bool on)
{
    if (addr >= MB_COIL_COUNT) return;
    if (on)
        s->coils[addr >> 3] |= (uint8_t)(1u << (addr & 7u));
    else
        s->coils[addr >> 3] &= (uint8_t)~(1u << (addr & 7u));
}

bool ModbusSlave_getCoil(const ModbusSlave *s, uint16_t addr)
{
    if (addr >= MB_COIL_COUNT) return false;
    return ((s->coils[addr >> 3] >> (uint8_t)(addr & 7u)) & 1u) != 0u;
}

uint16_t ModbusSlave_getReg(const ModbusSlave *s, uint16_t addr)
{
    return reg_read16(s, addr);
}

void ModbusSlave_setReg(ModbusSlave *s, uint16_t addr, uint16_t val)
{
    uint16_t *p = reg_ptr(s, addr);
    if (p) *p = val;
}

/* float32 big-endian:高字存 addr,低字存 addr+1 */
void ModbusSlave_setFloat(ModbusSlave *s, uint16_t addr, float f)
{
    uint32_t u;
    memcpy(&u, &f, 4u);
    ModbusSlave_setReg(s, addr, (uint16_t)(u >> 16));
    ModbusSlave_setReg(s, (uint16_t)(addr + 1u), (uint16_t)(u & 0xFFFFu));
}

float ModbusSlave_getFloat(const ModbusSlave *s, uint16_t addr)
{
    uint32_t u = ((uint32_t)ModbusSlave_getReg(s, addr) << 16) |
                 (uint32_t)ModbusSlave_getReg(s, (uint16_t)(addr + 1u));
    float f;
    memcpy(&f, &u, 4u);
    return f;
}

/* ── 波形 API ────────────────────────────────────────────────────── */
void ModbusSlave_pushWaveSample(ModbusSlave *s, float sample)
{
    s->waveBuf[s->waveWrite][s->waveCount++] = sample;
    if (s->waveCount >= MB_WAVE_COUNT) {
        /* 先冻结整批,再公布批次号/状态位(读者任何时刻看到完整一批) */
        s->waveFreeze = s->waveWrite;
        s->waveWrite  = (uint8_t)(s->waveWrite ^ 1u);
        s->waveCount  = 0u;
        s->waveBatch++;
        s->waveStatus = (uint16_t)(s->waveStatus | 0x0001u);
    }
}

uint16_t ModbusSlave_getWaveReg(const ModbusSlave *s, uint16_t idx)
{
    uint16_t w = s->waveFreeze;
    uint32_t u;
    memcpy(&u, &s->waveBuf[w][idx >> 1u], 4u);
    /* big-endian: 高字在低地址(idx 偶数为高 16 位) */
    return (idx & 1u) ? (uint16_t)(u & 0xFFFFu)
                      : (uint16_t)(u >> 16);
}

uint16_t ModbusSlave_getWaveBatch(const ModbusSlave *s)
{
    return s->waveBatch;
}

uint16_t ModbusSlave_getWaveStatus(const ModbusSlave *s)
{
    return s->waveStatus;
}
