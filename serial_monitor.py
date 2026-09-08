#!/usr/bin/env python3
"""
serial_monitor.py — F280025C Modbus RTU 从站上位机监控/测试工具

用途: 代替 Web 版上位机,作为 Modbus 主站与 DSP 从站(F280025C modbus_rtu 固件)通讯,
      实时打印每一帧的 TX(/dev 定向)/RX 16 进制 + 可读解释,验证串口收发过程。

用法示例:
  python3 serial_monitor.py                          # 自动检测串口 + 进入交互 shell
  python3 serial_monitor.py --tests                  # 跑一遍完整读写验证然后退出
  python3 serial_monitor.py --poll 200               # 每 200ms 轮询遥测(转速/电流),Ctrl-C 退出
  python3 serial_monitor.py --baud 115200 --port /dev/ttyACM0

协议与 ai_motor_control/docs/modbus_rtu_protocol.md 及 lib/serial/modbus.ts 严格一致:
  CRC-16/MODBUS(0xA001, init 0xFFFF), 功能码 01/03/05/06/0F/10,
  线圈 LSB 优先, float32 big-endian(高字低地址), 默认从站 0x01, 8N1。
"""
import argparse
import sys
import time
from datetime import datetime

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("[!] 需要 pyserial: pip install pyserial", file=sys.stderr)
    raise SystemExit(1)

# ── 地址常量(与 modbus_rtu.h / lib/serial/modbus.ts ADDR 一致)────────────────
ADDR = {
    "SPEED_SETPOINT": 0x0000,
    "CONTROL_MODE": 0x0001,
    "PID_SPD_KP": 0x0100, "PID_SPD_KI": 0x0102, "PID_SPD_KD": 0x0104,
    "PID_SPD_KD_N": 0x0106, "PID_SPD_KI_UPLIM": 0x0108, "PID_SPD_KI_LOWLIM": 0x010A,
    "PID_SPD_KI_OUT_LIM": 0x010C, "PID_SPD_OUT_LIM": 0x010E,
    "PID_CUR_KP": 0x0110, "PID_CUR_KI": 0x0112, "PID_CUR_KD": 0x0114,
    "PID_CUR_KD_N": 0x0116, "PID_CUR_KI_UPLIM": 0x0118, "PID_CUR_KI_LOWLIM": 0x011A,
    "PID_CUR_KI_OUT_LIM": 0x011C, "PID_CUR_OUT_LIM": 0x011E,
    "ACTUAL_SPEED": 0x1000, "ACTUAL_CURRENT": 0x1002,
    "FAULT_CODE": 0x1004, "STATUS_FLAGS": 0x1005,
    "COIL_MOTOR_EN": 0x0000, "COIL_FAULT_RESET": 0x0001, "COIL_EMERGENCY_STOP": 0x0002,
}
FC = {"READ_COILS": 0x01, "READ_HOLDING": 0x03, "WRITE_SINGLE_COIL": 0x05,
      "WRITE_SINGLE_REG": 0x06, "WRITE_MULTI_COILS": 0x0F, "WRITE_MULTI_REGS": 0x10}


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def pack(pdu: tuple) -> bytes:
    body = bytes(pdu)
    crc = crc16(body)
    return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def u16(b: bytes, i: int) -> int:
    return (b[i] << 8) | b[i + 1]


def f32_regs(hi: int, lo: int) -> float:
    import struct
    return struct.unpack(">f", bytes([(hi >> 8) & 0xFF, hi & 0xFF,
                                      (lo >> 8) & 0xFF, lo & 0xFF]))[0]


# ── 请求帧构建(PDU 去 CRC,由 pack 加 CRC)────────────────────────────────
def req_read_coils(slave, addr, count):
    return pack((slave, FC["READ_COILS"], addr >> 8, addr & 0xFF, count >> 8, count & 0xFF))


def req_read_holding(slave, addr, count):
    return pack((slave, FC["READ_HOLDING"], addr >> 8, addr & 0xFF, count >> 8, count & 0xFF))


def req_write_coil(slave, addr, on):
    v = 0xFF00 if on else 0x0000
    return pack((slave, FC["WRITE_SINGLE_COIL"], addr >> 8, addr & 0xFF, v >> 8, v & 0xFF))


def req_write_reg(slave, addr, value):
    return pack((slave, FC["WRITE_SINGLE_REG"], addr >> 8, addr & 0xFF, value >> 8, value & 0xFF))


def req_write_multi_coils(slave, addr, states):
    nbytes = (len(states) + 7) // 8
    data = bytearray(nbytes)
    for i, st in enumerate(states):
        if st:
            data[i >> 3] |= 1 << (i & 7)          # LSB 优先
    pdu = [slave, FC["WRITE_MULTI_COILS"], addr >> 8, addr & 0xFF,
           len(states) >> 8, len(states) & 0xFF, nbytes]
    return pack(tuple(pdu + list(data)))


def req_write_multi_regs(slave, addr, values):
    bytecount = len(values) * 2
    body = []
    for v in values:
        body.append((v >> 8) & 0xFF)
        body.append(v & 0xFF)
    pdu = [slave, FC["WRITE_MULTI_REGS"], addr >> 8, addr & 0xFF,
           len(values) >> 8, len(values) & 0xFF, bytecount]
    return pack(tuple(pdu + body))


def f32_regs_bytes(f: float):
    import struct
    b = struct.pack(">f", f)
    return b[0], b[1], b[2], b[3]


class Monitor:
    def __init__(self, port=None, baud=781250, slave=1, timeout=0.25):
        self.baud = baud
        self.slave = slave
        self.timeout = timeout
        self.ser = None
        self.port = port

    def detect_ports(self):
        ports = list_ports.comports()
        if not ports:
            return []
        return [(p.device, p.description or "") for p in ports]

    def connect(self, port):
        self.ser = serial.Serial(port, self.baud, timeout=self.timeout)
        self.ser.reset_input_buffer()
        self.port = port
        print(f"[OK] 已打开 {port} @ {self.baud} baud, 8N1")

    def close(self):
        if self.ser:
            self.ser.close()
            self.ser = None

    # ── 帧日志 ──────────────────────────────────────────────────────────
    @staticmethod
    def _log(tag, data: bytes, note=""):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        hexs = " ".join(f"{b:02X}" for b in data)
        ascii_ = "".join(chr(b) if 32 <= b < 127 else "." for b in data)
        print(f"[{ts}] [{tag:>3}] [{len(data):>3}B] {hexs:<52} |{ascii_}| {note}")

    # ── 应答解析: 从 buf 中识别一条完整合法帧 ───────────────────────────
    def _parse(self, buf: bytearray):
        if len(buf) < 5:
            return None
        fc = buf[1]
        if fc & 0x80:                               # 异常响应
            total = 5
        elif fc in (FC["READ_COILS"], FC["READ_HOLDING"]):
            if len(buf) < 3:
                return None
            total = 3 + buf[2] + 2
        elif fc in (FC["WRITE_SINGLE_COIL"], FC["WRITE_SINGLE_REG"],
                    FC["WRITE_MULTI_COILS"], FC["WRITE_MULTI_REGS"]):
            total = 8
        else:
            return None
        if len(buf) < total:
            return None
        frame = bytes(buf[:total])
        body = frame[:-2]
        calc = crc16(body)
        recv = frame[total - 2] | (frame[total - 1] << 8)
        return frame if calc == recv else None

    def transact(self, frame: bytes, note=""):
        if not self.ser:
            raise RuntimeError("未连接串口")
        self._log("TX", frame, note)
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        buf = bytearray()
        end = time.time() + self.timeout
        parsed = None
        while time.time() < end:
            if self.ser.in_waiting:
                chunk = self.ser.read(self.ser.in_waiting)
                buf += chunk
                parsed = self._parse(buf)
                if parsed is not None:
                    break
            else:
                time.sleep(0.002)
        if parsed is None:
            self._log("RX", bytes(buf), "(超时/无有效应答)")
            return None
        self._log("RX", parsed, self._decode(parsed, frame))
        return parsed

    # ── 应答解释 ──────────────────────────────────────────────────────
    def _decode(self, resp: bytes, req: bytes):
        fc = resp[1]
        try:
            if fc & 0x80:
                codes = {1: "非法功能", 2: "非法数据地址", 3: "非法数据值", 4: "从站故障"}
                return f"异常! code=0x{resp[2]:02X} {codes.get(resp[2],'')}"
            if fc == FC["READ_HOLDING"]:
                return self._decode_read_holding(resp, req)
            if fc == FC["READ_COILS"]:
                return self._decode_read_coils(resp, req)
            if fc in (FC["WRITE_SINGLE_COIL"], FC["WRITE_SINGLE_REG"]):
                return "写单 回显确认"
            if fc in (FC["WRITE_MULTI_COILS"], FC["WRITE_MULTI_REGS"]):
                return "写多 回显确认"
        except Exception:
            return ""
        return ""

    def _decode_read_holding(self, resp, req):
        reqaddr = u16(req, 2)
        count = u16(req, 4)
        bc = resp[2]
        regs = []
        for i in range(count):
            regs.append(u16(resp, 3 + i * 2))
        words = " ".join(f"R{reqaddr + i:04X}=0x{r:04X}({r})" for i, r in enumerate(regs))
        try:
            if reqaddr == ADDR["ACTUAL_SPEED"] and count >= 4:
                rpm = f32_regs(regs[0], regs[1])
                cur = f32_regs(regs[2], regs[3])
                return f"遥测 RPM={rpm:.1f} 电流={cur:.2f}A  [{words}]"
            if count == 2 and reqaddr in (ADDR["PID_SPD_KP"], ADDR["PID_SPD_KI"],
                                          ADDR["PID_SPD_KD"], ADDR["PID_CUR_KP"]):
                return f"float={f32_regs(regs[0], regs[1]):.4f}  [{words}]"
        except Exception:
            pass
        return words

    def _decode_read_coils(self, resp, req):
        count = u16(req, 4)
        bits = []
        for i in range(count):
            byte = resp[3 + (i >> 3)]
            bits.append("1" if (byte >> (i & 7)) & 1 else "0")
        return "线圈[" + ",".join(bits) + "]"


# ── 交互 shell 命令 ───────────────────────────────────────────────────
HELP = """
可用命令:
  help                         显示本帮助
  ports                        列出检测到的串口
  open <port>                  打开串口(如 /dev/ttyACM0 或 COM3)
  close                        关闭串口
  read   <addr> <count>        读保持寄存器(0x03)
  rcoil  <addr> <count>        读线圈(0x01)
  write  <addr> <val>          写单寄存器(0x06)
  wcoil  <addr> on|off         写单线圈(0x05)
  wm     <addr> v1,v2,...      写多寄存器(0x10)
  wmcoil <addr> b'0101'        写多线圈(0x0F),b'...' 字符串位序
  setp   <rpm>                 设转速设定(SPEED_SETPOINT)
  pid    <REG> <f>             写 float32 PID 参数(REG e.g. PID_SPD_KP)
  poll   <ms>                  周期性轮询遥测(0x1000 x4),Ctrl-C 停止
  raw    <hex>                 发送任意原始帧(如 '01 03 10 00 00 04')
  tests                        跑一遍验证
  quit | exit                  退出
""".strip()


def run_tests(mon: Monitor):
    print("== 运行验证 ==\n")
    s = mon.slave

    def chk(name, ok):
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")

    def tr(frame, note):
        r = mon.transact(frame, note)
        return r

    # 0x06 写单寄存器
    r = tr(req_write_reg(s, ADDR["SPEED_SETPOINT"], 3000), "设转速=3000")
    chk("写单寄存器 -> 8B 回显", r is not None and len(r) == 8)
    # 0x03 读回
    r = tr(req_read_holding(s, ADDR["SPEED_SETPOINT"], 1), "读回转速设定")
    chk("读保持寄存器 -> 应答", r is not None)
    # 0x05 写线圈(使能)
    r = tr(req_write_coil(s, ADDR["COIL_MOTOR_EN"], True), "电机使能=1")
    chk("写单线圈 -> 8B 回显", r is not None and len(r) == 8)
    # 0x01 读线圈
    r = tr(req_read_coils(s, 0x0000, 3), "读 3 个线圈")
    chk("读线圈 -> 应答", r is not None)
    # 0x10 写多寄存器(float32 PI 参数 1.5)
    b = f32_regs_bytes(1.5)
    r = tr(req_write_multi_regs(s, ADDR["PID_SPD_KP"], [u16(b, 0), u16(b, 2)]),
           "写 PID_SPD_KP=1.5")
    chk("写多寄存器(float32) -> 8B 回显", r is not None and len(r) == 8)
    # 0x03 读回 float
    r = tr(req_read_holding(s, ADDR["PID_SPD_KP"], 2), "读回 KP")
    chk("读回 float -> 应答", r is not None)
    # 0x0F 写多线圈
    r = tr(req_write_multi_coils(s, 0x0000, [True, False, True]), "写多线圈 101")
    chk("写多线圈 -> 8B 回显", r is not None and len(r) == 8)


def run_poll(mon: Monitor, interval_ms):
    print(f"== 轮询遥测(每 {interval_ms}ms), Ctrl-C 停止 ==\n")
    try:
        while True:
            r = mon.transact(req_read_holding(mon.slave, ADDR["ACTUAL_SPEED"], 4),
                             "轮询遥测 ACTUAL_SPEED x4")
            time.sleep(interval_ms / 1000.0)
    except KeyboardInterrupt:
        print("\n== 停止轮询 ==")


def interactive(mon: Monitor, auto_port):
    if auto_port:
        mon.connect(auto_port)
    elif not mon.ser:
        ports = mon.detect_ports()
        if len(ports) == 1:
            mon.connect(ports[0][0])
        elif len(ports) > 1:
            print("检测到多个串口, 请用 'open <port>' 选择:", ", ".join(p[0] for p in ports))
        else:
            print("未检测到串口, 请用 'open <port>' 指定, 或 'ports' 查看")

    print("MOTOTUNE Modbus RTU 监控 (%s baud, slave 0x%02X)" % (mon.baud, mon.slave))
    print(HELP)
    while True:
        try:
            line = input("modbus> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()
        try:
            if cmd in ("quit", "exit"):
                break
            elif cmd == "help":
                print(HELP)
            elif cmd == "ports":
                for p in mon.detect_ports():
                    print(" ", p[0], p[1])
            elif cmd == "open":
                mon.connect(parts[1])
            elif cmd == "close":
                mon.close()
            elif cmd == "read":
                mon.transact(req_read_holding(mon.slave, int(parts[1], 0), int(parts[2], 0)), "读保持寄存器")
            elif cmd == "rcoil":
                mon.transact(req_read_coils(mon.slave, int(parts[1], 0), int(parts[2], 0)), "读线圈")
            elif cmd == "write":
                mon.transact(req_write_reg(mon.slave, int(parts[1], 0), int(parts[2], 0)), "写单寄存器")
            elif cmd == "wcoil":
                mon.transact(req_write_coil(mon.slave, int(parts[1], 0), parts[2].lower() == "on"), "写单线圈")
            elif cmd == "wm":
                vals = [int(x, 0) for x in parts[2].split(",")]
                mon.transact(req_write_multi_regs(mon.slave, int(parts[1], 0), vals), "写多寄存器")
            elif cmd == "wmcoil":
                bits = [c == "1" for c in parts[2] if c in "01"]
                mon.transact(req_write_multi_coils(mon.slave, int(parts[1], 0), bits), "写多线圈")
            elif cmd == "setp":
                mon.transact(req_write_reg(mon.slave, ADDR["SPEED_SETPOINT"], int(parts[1])), "设转速")
            elif cmd == "pid":
                name = parts[1].upper()
                if name not in ADDR:
                    print("未知 REG:", name)
                    continue
                f = float(parts[2])
                b = f32_regs_bytes(f)
                vals = [u16(b, 0), u16(b, 2)]
                mon.transact(req_write_multi_regs(mon.slave, ADDR[name], vals),
                             f"写 {name}={f}")
            elif cmd == "poll":
                ms = int(parts[1] or 200)
                run_poll(mon, ms)
            elif cmd == "raw":
                raw = bytes(int(x, 16) for x in parts[1].split())
                mon.transact(raw, "原始帧")
            elif cmd == "tests":
                run_tests(mon)
            else:
                print("未知命令, 'help' 查看。")
        except Exception as e:
            print(f"[!] {e}")


def main():
    ap = argparse.ArgumentParser(description="F280025C Modbus RTU 从站监控/测试")
    ap.add_argument("--port", help="串口设备(默认自动检测)")
    ap.add_argument("--baud", type=int, default=781250, help="波特率(默认 781250,与固件一致)")
    ap.add_argument("--slave", type=int, default=1, help="从站地址(默认 1)")
    ap.add_argument("--tests", action="store_true", help="跑一遍验证后退出")
    ap.add_argument("--poll", nargs="?", const=200, type=int, default=None, help="轮询遥测 [间隔 ms]")
    args = ap.parse_args()

    mon = Monitor(port=args.port, baud=args.baud, slave=args.slave)
    try:
        if args.tests:
            if not args.port:
                ports = mon.detect_ports()
                if not ports:
                    print("未检测到串口, 用 --port 指定。")
                    return 1
                args.port = ports[0][0]
            mon.connect(args.port)
            run_tests(mon)
            return 0
        if args.poll is not None:
            if not args.port:
                ports = mon.detect_ports()
                if not ports:
                    print("未检测到串口, 用 --port 指定。")
                    return 1
                args.port = ports[0][0]
            mon.connect(args.port)
            run_poll(mon, args.poll)
            return 0
        interactive(mon, args.port)
    finally:
        mon.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
