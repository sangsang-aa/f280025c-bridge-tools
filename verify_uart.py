# verify_uart.py — 验证固件已运行: 发一帧读转速设定(0x0000)请求
import sys
import time

import serial

BAUD = 781250  # 与固件 MODBUS_BAUD 一致(921600 无法整除 25MHz LSPCLK)


def main() -> int:
    try:
        s = serial.Serial("COM4", BAUD, timeout=0.5)
        time.sleep(0.3)
        s.reset_input_buffer()
        s.write(bytes.fromhex("01 03 00 00 00 01 84 0A"))
        d = s.read(64)
        s.close()
    except Exception as e:
        print("  [X] 串口错误:", e)
        return 1
    if d and d[:3] == bytes.fromhex("01 03 02"):
        print("  RX:", d.hex())
        print("  [OK] 固件已运行,可以启动桥接/上位机")
        return 0
    print("  RX:", d.hex() if d else "(empty)")
    print("  [X] 无应答,请确认板子已插好/串口号")
    return 1


if __name__ == "__main__":
    sys.exit(main())
