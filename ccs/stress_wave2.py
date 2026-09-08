# -*- coding: utf-8 -*-
"""拟真节奏压测: 30ms 间隔轮询状态+波形(接近页面 50ms 轮询 + 波形读取)。"""
import struct
import time

import serial


def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = ((c >> 1) ^ 0xA001) if (c & 1) else (c >> 1)
    return c & 0xFFFF


def req(addr, count):
    body = bytes([1, 3, addr >> 8, addr & 0xFF, count >> 8, count & 0xFF])
    c = crc16(body)
    return body + bytes([c & 0xFF, c >> 8])


def main():
    s = serial.Serial("COM4", 781250, timeout=0.5)
    ok = 0
    tot = 0
    bad = 0
    for i in range(15):
        s.reset_input_buffer()
        s.write(req(0x2200, 2))
        d = s.read(9)
        tot += 1
        if len(d) == 9 and d[:3] == bytes([1, 3, 4]):
            ok += 1
        else:
            print("status short", i, len(d))

        time.sleep(0.03)

        s.reset_input_buffer()
        s.write(req(0x2000, 125))
        d = s.read(255)
        tot += 1
        if len(d) == 255:
            ok += 1
            for j in range(62):
                f = struct.unpack(">f", d[3 + j * 2:3 + j * 2 + 4])[0]
                if f != f or abs(f) > 1.5:
                    bad += 1
        else:
            print("wave short", i, len(d))

        time.sleep(0.03)
    print("transactions ok", ok, "/", tot, "| invalid floats:", bad)
    s.close()


if __name__ == "__main__":
    main()
