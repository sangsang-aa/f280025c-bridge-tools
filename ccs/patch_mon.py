import re
import py_compile

p = "/mnt/e/f280025c_modbus_slave/tools/serial_monitor.py"
s = open(p, encoding="utf-8").read()
if "frame[total - 2] |" not in s:
    s = s.replace("recv = u16(frame, total - 2)",
                  "recv = frame[total - 2] | (frame[total - 1] << 8)")
s = re.sub(r'default=\d+, help="[^"]*"',
           'default=781250, help="波特率(默认 781250,与固件一致)"', s, count=1)
s = s.replace("baud=1500000, slave", "baud=781250, slave")
open(p, "w", encoding="utf-8", newline="\n").write(s)
py_compile.compile(p, doraise=True)
print("monitor OK; CRC-fix present:", "frame[total - 2] |" in s)
