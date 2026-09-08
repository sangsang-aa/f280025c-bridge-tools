# -*- coding: utf-8 -*-
"""波形压测: 大帧/短帧交替(模拟页面 50ms 轮询 + 波形读取),检验响应双缓冲。"""
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
    s = serial.Serial("COM4", 781250, timeout=0.8)
    ok = 0
    tot = 0
    for i in range(12):
        for (addr, count, need) in [(0x2000, 125, 255), (0x2200, 2, 9), (0x207D, 75, 157)]:
            s.reset_input_buffer()
            s.write(req(addr, count))
            d = s.read(need)
            tot += 1
            if addr == 0x2000 and len(d) == 255:
                ok += 1
            elif addr == 0x2200 and len(d) == 9 and d[:3] == bytes([1, 3, 4]):
                ok += 1
            elif addr == 0x207D and len(d) == 157:
                ok += 1
    print("transactions ok", ok, "/", tot)

    s.reset_input_buffer()
    s.write(req(0x2000, 125))
    a = s.read(255)
    s.reset_input_buffer()
    s.write(req(0x207D, 75))
    b = s.read(157)
    floats = [struct.unpack(">f", a[3 + j * 2:3 + j * 2 + 4])[0] for j in range(62)]
    floats += [struct.unpack(">f", b[3 + j * 2:3 + j * 2 + 4])[0] for j in range(38)]
    bad = [(i, "%.4g" % f) for i, f in enumerate(floats) if f != f or abs(f) > 1.5]
    print("100 点样本异常数:", len(bad), bad[:5])
    print("样例:", ["%.3f" % v for v in floats[:8]])
    s.close()


if __name__ == "__main__":
    main()
