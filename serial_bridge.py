#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_bridge.py — COM <-> WebSocket 透明桥 + Modbus RTU 帧日志(MOTOTUNE 桥接模式)

用途: Web 上位机(浏览器)不再直接开 COM 口,而是连接本桥(ws://127.0.0.1:8765);
      桥独占 COM 并把每一帧通讯 TX/RX 16 进制 + 解释 实时打印 —— 上位机测试与监控并行。

用法:
  python serial_bridge.py                          # --port auto: 自动识别 XDS110 回传(COM n)
  python serial_bridge.py --port COM4 --baud 115200

依赖: pip install pyserial websockets
"""
import argparse
import asyncio
import subprocess
import sys
from pathlib import Path

import serial
from serial.tools import list_ports

import websockets

from serial_monitor import Monitor

FRAME_MAX = 300


def crc16_of(data: bytes) -> int:
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c >> 1) ^ 0xA001) if (c & 1) else (c >> 1)
    return c & 0xFFFF


def parse_request(buf: bytes):
    """解析主站->从站请求帧(请求与响应格式不同,_parse 只认响应)。"""
    if len(buf) < 8:
        return None
    fc = buf[1]
    if fc in (0x01, 0x03, 0x05, 0x06):
        total = 8
    elif fc == 0x0F:
        if len(buf) < 9:
            return None
        total = 7 + buf[6]
    elif fc == 0x10:
        if len(buf) < 9:
            return None
        total = 9 + buf[6]
    else:
        return None
    if len(buf) < total:
        return None
    frame = bytes(buf[:total])
    calc = crc16_of(frame[:-2])
    recv = frame[total - 2] | (frame[total - 1] << 8)
    return frame if calc == recv else None


# 板子断电重启后 BROM 会停在 WAIT;桥启动时自动"释放启动"(fix_boot 流程)
CCS_RUN_BAT = r"C:\ti\ccs2040\ccs\scripting\run.bat"
BOOT_RUN_MJS = Path(r"E:\f280025c_modbus_slave\ccs\boot_run.mjs")


def auto_fix_boot() -> bool:
    try:
        r = subprocess.run(
            f'"{CCS_RUN_BAT}" "{BOOT_RUN_MJS}"',
            shell=True, capture_output=True, timeout=150, cwd=str(BOOT_RUN_MJS.parent)
        )
        raw = (r.stdout or b"") + (r.stderr or b"")
        if b"BOOT RELEASED" in raw:
            print("[i] 启动释放完成(固件从 Flash 运行)")
            return True
        print("[!] 启动释放未确认(板子插好了吗?仍可尝试后续连接)")
        for line in raw.decode("utf-8", errors="replace").splitlines()[-6:]:
            print("   ", line.strip())
        return False
    except Exception as e:
        print("[!] 自动释放启动失败(可手动运行 fix_boot.ps1):", e)
        return False


def uart_verify(port: str, baud: int) -> bool:
    """发一帧读转速设定,验证从站已应答。"""
    import time

    try:
        s = serial.Serial(port, baud, timeout=0.5)
        time.sleep(0.2)
        s.reset_input_buffer()
        s.write(bytes.fromhex("01 03 00 00 00 01 84 0A"))
        d = s.read(64)
        s.close()
    except Exception as e:
        print("  [X] 串口自检失败:", e)
        return False
    if d and d[:3] == bytes.fromhex("01 03 02"):
        print("  [OK] 固件已运行(自检 RX:", d.hex() + ")")
        return True
    print("  [X] 自检无应答:", (d or b"").hex() or "(empty)")
    return False


def wait_for_xds110_port(exclude_bluetooth: bool = True) -> str | None:
    """等 XDS110 应用串口出现(拔插后枚举需几秒)。"""
    import time

    for _ in range(30):
        for p in list_ports.comports():
            if "xds110" in (p.description or "").lower() and p.device.lower().startswith("com"):
                if "uart" in (p.description or "").lower() or "user" in (p.description or "").lower() or "app" in (p.description or "").lower():
                    return p.device
        time.sleep(1)
    return None

# 自动识别: XDS110 的应用串口描述里通常含 "Application/User UART" 或 "XDS110"
def find_xds110_port() -> str | None:
    for p in list_ports.comports():
        blob = f"{p.device} {p.description or ''} {p.product or ''}".lower()
        if "xds110" in blob and ("uart" in blob or "app" in blob or "user" in blob):
            return p.device
    return None


class FrameLog:
    """按帧识别与打印:复用 serial_monitor 的 CRC/长度解析与解释,风格一致。"""

    def __init__(self, baud: int, slave: int = 1):
        self.mon = Monitor(baud=baud, slave=slave)
        self.rx = bytearray()
        self.tx = bytearray()

    def feed(self, buf: bytearray, chunk: bytes, tag: str) -> None:
        buf.extend(chunk)
        if len(buf) > FRAME_MAX:
            del buf[: len(buf) - FRAME_MAX]
        for _ in range(32):
            while buf and buf[0] != self.mon.slave:
                del buf[0]
            if tag == "TX":
                frame = parse_request(bytes(buf))
            else:
                frame = self.mon._parse(bytes(buf))
            if frame is None:
                break
            note = self.mon._decode(frame, frame) if tag == "RX" else ""
            self.mon._log(tag, frame, note)
            del buf[: len(frame)]

    def done(self) -> None:
        if self.rx:
            self.mon._log("RX", bytes(self.rx), "(残留未解析字节)")
            self.rx.clear()
        if self.tx:
            self.mon._log("TX", bytes(self.tx), "(残留未解析字节)")
            self.tx.clear()


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="MOTOTUNE COM<->WebSocket 串口桥 + Modbus 帧日志")
    ap.add_argument("--port", default="auto", help="串口(默认 auto=自动识别 XDS110 回传;可写 COM4 等)")
    ap.add_argument("--baud", type=int, default=115200, help="波特率(默认 115200,需与固件一致)")
    ap.add_argument("--listen", default="127.0.0.1", help="监听地址(默认 127.0.0.1)")
    ap.add_argument("--tcp", type=int, default=8765, help="监听端口(默认 8765)")
    ap.add_argument("--no-fix-boot", action="store_true", help="启动时不自动释放板子启动(默认自动)")
    return ap.parse_args()


async def main() -> None:
    args = parse_args()

    import sys
    sys.stdout.reconfigure(encoding="utf-8")

    # 等待板子/探针枚举完成(拔插后 COM 需要几秒才出现)
    print("[i] 等待 XDS110 应用串口出现...")
    waited = wait_for_xds110_port()
    if not waited:
        print("[!] 未等到 XDS110 串口(确认板子 USB 已插好);仍按原配置启动,可稍后用 --port COMx")
    else:
        print(f"[i] 检测到 XDS110 回传串口: {waited}")
        if args.port == "auto":
            args.port = waited

    if not args.no_fix_boot:
        print("[i] 释放板子启动(解除 BROM WAIT)...")
        auto_fix_boot()
        print(f"[i] 串口自检({args.port} @ {args.baud}):")
        uart_verify(args.port, args.baud)

    print("[i] 已检测到串口:")
    for p in list_ports.comports():
        print(f"    {p.device}  {p.description}")
    print(f"[i] MOTOTUNE 串口桥: {args.port} @ {args.baud} baud, 8N1")
    print(f"[i] 等待上位机连接 ws://{args.listen}:{args.tcp} (页面点「桥接」); Ctrl+C 退出")

    session = {"ser": None, "task": None}

    async def handler(ws):
        # 新客户端接管: 结束旧的会话/串口,避免"已连接"卡死
        if session["ser"] is not None:
            if session["task"] is not None:
                session["task"].cancel()
            old_ser = session["ser"]
            session["ser"] = None
            try:
                old_ser.close()
                print("[*] 替换旧会话,旧串口已关闭")
            except Exception:
                pass
        try:
            ser = serial.Serial(args.port, args.baud, timeout=0.02, write_timeout=0.05)
        except Exception as e:
            hint = "串口可能被占用(serial_monitor.py/其他程序占用,先关掉)" if ("PermissionError" in str(e) or "denied" in str(e).lower()) else "板子没插好或 COM 号不对(看上面检测列表,或用 --port COMx 指定)"
            await ws.send("ERROR: 无法打开 %s(%s)。%s" % (args.port, e, hint))
            print("[-] 打开串口失败:", e, "|", hint)
            return
        session["ser"] = ser
        log = FrameLog(args.baud)
        print(f"\n[+] 浏览器已连接 → 串口 {args.port} @ {args.baud} 8N1,开始记录通讯帧...")
        await ws.send("READY OK")
        loop = asyncio.get_running_loop()

        async def reader():
            while True:
                try:
                    data = await loop.run_in_executor(None, ser.read, 256)
                except Exception as e:
                    print("[-] 读串口异常:", e)
                    break
                if not data:
                    await asyncio.sleep(0.001)
                    continue
                try:
                    log.feed(log.rx, data, "RX")
                    await ws.send(bytes(data))
                except Exception:
                    break

        session["task"] = asyncio.create_task(reader())
        try:
            async for msg in ws:
                if isinstance(msg, bytes):
                    log.feed(log.tx, msg, "TX")
                    ser.write(msg)
        except Exception:
            pass
        finally:
            if session["ser"] is ser:
                session["ser"] = None
            session["task"] = None
            try:
                session["task"] and session["task"].cancel()
            except Exception:
                pass
            log.done()
            try:
                ser.close()
            except Exception:
                pass
            print(f"[−] 浏览器断开,串口 {args.port} 已关闭.\n")

    async with websockets.serve(handler, args.listen, args.tcp):
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[i] 桥已退出(Ctrl+C)。板子会自动从 Flash 继续运行,无需再释放。")
