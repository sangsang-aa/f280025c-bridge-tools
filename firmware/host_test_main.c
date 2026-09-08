/*
 * host_test_main.c — 主机侧(gcc)单元测试,验证 modbus_rtu.c 与上位机协议的字节精确对齐。
 *
 * 编译/运行:
 *   gcc -std=c11 -I. -o host_test host_test_main.c modbus_rtu.c && ./host_test
 *
 * 覆盖: CRC-16/MODBUS、0x03 读保持寄存器(13 字节响应)、float32 big-endian、
 *       0x01 读线圈、0x05/0x06 写单回显、0x0F/0x10 写多响应、异常码、越界、地址/CRC 过滤。
 */
#include <stdio.h>
#include <string.h>
#include "modbus_rtu.h"

static int g_pass = 0;
static int g_fail = 0;

static void check(const char *name, int ok)
{
    if (ok) { g_pass++; printf("  [PASS] %s\n", name); }
    else    { g_fail++; printf("  [FAIL] %s\n", name); }
}

static uint16_t make_fixed_req(uint8_t *frame, uint8_t slave, uint8_t fc, uint16_t a, uint16_t b)
{
    frame[0] = slave;
    frame[1] = fc;
    frame[2] = (uint8_t)(a >> 8); frame[3] = (uint8_t)(a & 0xFF);
    frame[4] = (uint8_t)(b >> 8); frame[5] = (uint8_t)(b & 0xFF);
    uint16_t crc = ModbusCrc16(frame, 6u);
    frame[6] = (uint8_t)(crc & 0xFF);
    frame[7] = (uint8_t)((crc >> 8) & 0xFF);
    return 8u;
}

/* 通用多写请求构建: payload 放 frame[7..], byteCount=b */
static uint16_t make_multi(uint8_t *frame, uint8_t slave, uint8_t fc,
                           uint16_t addr, uint16_t qty, uint8_t byteCount,
                           const uint8_t *payload)
{
    frame[0] = slave; frame[1] = fc;
    frame[2] = (uint8_t)(addr >> 8); frame[3] = (uint8_t)(addr & 0xFF);
    frame[4] = (uint8_t)(qty >> 8); frame[5] = (uint8_t)(qty & 0xFF);
    frame[6] = byteCount;
    if (payload) memcpy(&frame[7], payload, byteCount);
    uint16_t len = (uint16_t)(7u + byteCount + 2u);
    uint16_t crc = ModbusCrc16(frame, (uint16_t)(len - 2u));
    frame[len - 2] = (uint8_t)(crc & 0xFF);
    frame[len - 1] = (uint8_t)((crc >> 8) & 0xFF);
    return len;
}

/* 整帧喂入,返回响应是否 ready 及响应长度 */
static int feed(ModbusSlave *s, const uint8_t *req, uint16_t len, uint16_t *respLen)
{
    ModbusResult r = MB_RESULT_NONE;
    for (uint16_t i = 0; i < len; i++) {
        r = ModbusSlave_feedByte(s, req[i]);
    }
    if (r == MB_RESULT_RESPONSE) { *respLen = s->respLen; return 1; }
    return 0;
}

static void hex(const char *tag, const uint8_t *b, uint16_t n)
{
    printf("    %s: ", tag);
    for (uint16_t i = 0; i < n; i++) printf("%02X ", b[i]);
    printf("\n");
}

int main(void)
{
    ModbusSlave s;
    uint8_t req[64];
    uint8_t payload[64];
    uint16_t respLen;

    ModbusSlave_init(&s, 0x01);

    printf("=== CRC-16/MODBUS ===\n");
    /* 标准向量: 01 03 00 00 00 0a -> CRC 0xCDC5 */
    {
        uint8_t v[] = {0x01,0x03,0x00,0x00,0x00,0x0A};
        uint16_t c = ModbusCrc16(v, 6);
        check("crc16(01 03 00 00 00 0A)==0xCDC5", c == 0xCDC5u);
    }

    printf("=== 0x03 读保持寄存器与 float32 big-endian ===\n");
    {
        /* 预设 ACTUAL_SPEED=2000.0f, ACTUAL_CURRENT=3.0f, PID_SPD_KP=1.5f */
        ModbusSlave_setFloat(&s, MB_ADDR_ACTUAL_SPEED, 2000.0f);
        ModbusSlave_setFloat(&s, MB_ADDR_ACTUAL_CURRENT, 3.0f);
        ModbusSlave_setFloat(&s, MB_ADDR_PID_SPD_KP, 1.5f);
        check("2000.0f -> 0x44FA/0x0000", ModbusSlave_getReg(&s, MB_ADDR_ACTUAL_SPEED)==0x44FAu
                                          && ModbusSlave_getReg(&s, MB_ADDR_ACTUAL_SPEED+1)==0x0000u);
        check("1.5f -> 0x3FC0/0x0000", ModbusSlave_getReg(&s, MB_ADDR_PID_SPD_KP)==0x3FC0u
                                        && ModbusSlave_getReg(&s, MB_ADDR_PID_SPD_KP+1)==0x0000u);

        /* 读遥测 4 寄存器(0x1000 x4):上位机期望 13 字节 */
        uint16_t len = make_fixed_req(req, 0x01, 0x03, 0x1000, 4);
        check("feed read 0x1000 x4 ready", feed(&s, req, len, &respLen) && respLen == 13);
        hex("req", req, len);
        hex("resp", s.resp, respLen);
        /* slave, 0x03, 0x08, [44][FA][00][00], [40][40][00][00], crc => 13*/
        uint8_t expect[] = {0x01,0x03,0x08, 0x44,0xFA,0x00,0x00, 0x40,0x40,0x00,0x00, 0x00,0x00};
        memcpy(&expect[11], &s.resp[11], 2); /* 末 2 字节 CRC 动态 */
        check("resp body == expected telemetry", memcmp(s.resp, expect, 11)==0);
    }

    printf("=== 0x01 读线圈(LSB 优先) ===\n");
    {
        ModbusSlave_setCoil(&s, MB_COIL_MOTOR_EN, true);
        ModbusSlave_setCoil(&s, MB_COIL_FAULT_RESET, true);
        uint16_t len = make_fixed_req(req, 0x01, 0x01, 0x0000, 0x0003);
        check("feed read coils ready", feed(&s, req, len, &respLen) && respLen == 6);
        /* byteCount=1, data bit0|bit1 = 0x03 */
        check("coils byte == 0x03", s.resp[2]==1 && s.resp[3]==0x03);
    }

    printf("=== 0x05 写单线圈 ===\n");
    {
        uint16_t len = make_fixed_req(req, 0x01, 0x05, 0x0000, 0xFF00);
        check("write coil echo 8B", feed(&s, req, len, &respLen) && respLen == 8);
        check("coil set on", ModbusSlave_getCoil(&s, MB_COIL_MOTOR_EN));
        /* 非法值(非 0xFF00/0x0000) -> 异常 0x03 */
        uint16_t len2 = make_fixed_req(req, 0x01, 0x05, 0x0000, 0x1234);
        check("start next frame write bad value -> exception 0x03",
              feed(&s, req, len2, &respLen) && respLen==5 && s.resp[1]==0x85u && s.resp[2]==0x03u);
    }

    printf("=== 0x06 写单寄存器 ===\n");
    {
        uint16_t len = make_fixed_req(req, 0x01, 0x06, MB_ADDR_SPEED_SETPOINT, 0x0BB8);
        check("write reg echo 8B", feed(&s, req, len, &respLen) && respLen == 8);
        check("reg == 3000", ModbusSlave_getReg(&s, MB_ADDR_SPEED_SETPOINT)==0x0BB8u);
    }

    printf("=== 0x10 写多寄存器(float32,big-endian) ===\n");
    {
        /* 上个例子: KP=1.5 -> 01 10 01 00 00 02 04 3F C0 00 00 + CRC */
        payload[0]=0x3F; payload[1]=0xC0; payload[2]=0x00; payload[3]=0x00;
        uint16_t len = make_multi(req, 0x01, 0x10, MB_ADDR_PID_SPD_KP, 2, 4, payload);
        check("write multi reg echo 8B", feed(&s, req, len, &respLen) && respLen == 8);
        check("KP float == 1.5", ModbusSlave_getFloat(&s, MB_ADDR_PID_SPD_KP)==1.5f);
    }

    printf("=== 0x0F 写多线圈 ===\n");
    {
        /* 写 3 个线圈: bit0=MOTOR_EN(1) bit1=FAULT(1) bit2=ESTOP(0) -> 0x03 */
        payload[0] = 0x03;
        uint16_t len = make_multi(req, 0x01, 0x0F, 0x0000, 3, 1, payload);
        check("write multi coil echo 8B", feed(&s, req, len, &respLen) && respLen == 8);
        check("coil2(ESTOP)==off", !ModbusSlave_getCoil(&s, MB_COIL_EMERGENCY_STOP));
        check("coil0==on", ModbusSlave_getCoil(&s, MB_COIL_MOTOR_EN));
    }

    printf("=== 越界 / 非法地址 ===\n");
    {
        /* 读 0x1100 -> 异常 0x02 */
        uint16_t len = make_fixed_req(req, 0x01, 0x03, 0x1100, 1);
        check("read 0x1100 -> exception 0x02", feed(&s, req, len, &respLen) && respLen==5
                                                && s.resp[1]==0x83u && s.resp[2]==0x02u);
        /* 跨段读 0x03FF x2(会进入 0x1000 段) -> 异常 0x02 */
        uint16_t len2 = make_fixed_req(req, 0x01, 0x03, 0x03FF, 2);
        check("cross-region read -> exception 0x02", feed(&s, req, len2, &respLen) && respLen==5
                                                       && s.resp[2]==0x02u);
        /* 未知功能码 -> 异常 0x01 */
        uint16_t len3 = make_fixed_req(req, 0x01, 0x08, 0x0000, 1);
        check("illegal function -> exception 0x01", feed(&s, req, len3, &respLen) && respLen==5
                                                      && s.resp[1]==0x88u && s.resp[2]==0x01u);
    }

    printf("=== 地址过滤 / CRC 过滤 ===\n");
    {
        /* 从站地址不符 -> 无响应 */
        uint16_t len = make_fixed_req(req, 0x02, 0x03, 0x1000, 1);
        uint16_t rl;
        check("wrong slave -> no response", !feed(&s, req, len, &rl));
        /* CRC 错误 -> 无响应 */
        uint16_t len2 = make_fixed_req(req, 0x01, 0x03, 0x1000, 1);
        req[len2-1] ^= 0xFF;                 /* 破坏 CRC 高字节 */
        check("bad CRC -> no response", !feed(&s, req, len2, &rl));
    }

    printf("\n=== RESULT: %d passed, %d failed ===\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
