# F280025C 调试工具(桥接/监控/固件)— 与 ai_motor_control 主程序无关

> ⚠️ **仅限本地调试联调**。本仓库是 F280025C(LAUNCHXL-F280025C)Modbus RTU 从站
> 的辅助调试/监控/固件工具,与 MOTOTUNE 主程序(ai_motor_control)无关,
> **打包/发布 ai_motor_control 时请排除,不要进入构建产物**。
>
> Windows 侧部署参考目录:`E:\f280025c_modbus_slave`(与仓库同构,仅 Windows 路径)。

## 目录

```
├── firmware/                         固件源码(driverlib 宿主 + 可移植协议核心)
│   ├── F280025C_modbus_slave.c         C2000Ware 6.x 宿主:SCI-A / 30kHz 生成 / 双缓冲发送
│   ├── modbus_rtu.h / modbus_rtu.c     可移植核心:CRC / 01/03/05/06/0F/10 / float32 BE / 波形区
│   └── host_test_main.c                x86 gcc 单测(22 项)
├── tools/
│   ├── serial_bridge.py                主调试后端:COM<->WebSocket 桥 + 每帧 TX/RX 日志,
│   │                                  启动自动:等 XDS110 串口→释放 BROM WAIT→自检
│   └── serial_monitor.py               直连 COM Modbus 测试/监控(--tests/--poll/shell)
├── ccs/
│   ├── build_fw.ps1                    C2000 CGT 命令行编译(build_fw 输出的 .out)
│   ├── fix_boot.ps1 / boot_run.mjs     板子断电重启后的启动释放 + 自检
│   ├── verify_uart.py                  串口自检脚本
│   └── stress_wave.py / probe.mjs      波形压测 / 调试读取脚本
└── README.md                           本文档
```

## 关键事实

| 项 | 值 | 说明 |
|---|---|---|
| 固件 | F280025C Modbus 从站,8N1,从站 0x01 | 已烧入开发板(旧烧录:781250 波形版) |
| 波特率 | **781250** | =25MHz/32 精确;**921600 无整数解**(最近真值 781250/1041666,偏差≥13% 必断) |
| SCI 引脚 | GPIO29=TX / GPIO28=RX | C2000Ware 6.x:`GPIO_29_SCIA_TX` / `GPIO_28_SCIA_RX`(28 为 RX,注意与旧文档相反) |
| 串口 | XDS110 Application/User UART(自动识别) | `--port auto` 默认;板子复用 USB 回传 |
| 烧写 | DSLite + TMS320F280025C_LaunchPad.ccxml | XDS110 固件提示升级 3.0.0.41(可选) |

## 寄存器(波形 + 常规)

- 常规:转速设定 0x0000、PID 参数 0x0100 起、遥测 ACTUAL_SPEED/ACTUAL_CURRENT 0x1000/0x1002,线圈 MOTOR_EN 0、FAULT_RESET 1、**EMERGENCY_STOP 2**
- 波形:`0x2000..0x20C7`=100×float32(big-endian 高字在前);`0x2200` 批次号(只读,发布+1);`0x2201` bit0=新批次(0x03 读到自动清位,0x06 写 0 也可清)
- 0x03 单次≤125 寄存器 → 波形 200 寄存器拆 `0x2000×125 + 0x207D×75`;**拼接:两帧数据字节直接串接后按 4 字节一组解 float**(0x207D 起点在 float62 中间,不可按帧独立对齐)

## 固件行为(重要)

- **30kHz 生成 cos(600πt) → 每 6 点抽 1 → 5kHz 采集;双缓冲 100 点攒满冻结发布(批次号+状态位)**
- **响应发送双缓冲(乒乓)**:ISR 生成后拷入非发送槽再发布,修复"发送期间被新请求覆盖"问题
- **急停联动**:`EMERGENCY_STOP(coil2)=1` 时控制环持续把转速设定清 0(防上电即转,by 设计)。
  测试过"写多线圈 101"会置位,需发 `01 05 00 02 00 00 6C 0A` 解除,否则转速永远为 0
- ACTUAL_SPEED 回显=设定值(无电机时便于链路验证);ACTUAL_CURRENT 恒 0(未接采样)

## 已知边界

- XDS110 回传在 781250 **满负荷压测**(连续 30ms 级请求)时偶发丢帧/丢 2 字节;真实页面节奏(≤10Hz 波形 + 50ms 遥测)实测**连续正确**。前端应:CRC 校验 + 失败重试 2 次 + 批次前后一致性
- 波特率 1041666(.67,BRR=2)也行但大帧稳定率略低(≈93%),未采用
- `serial_monitor.py` 与桥不能同时开同一 COM

## 用法(Windows)

```powershell
# 桥(一条命令:自动唤醒板子 + 日志 + 桥接)
python -X utf8 -u tools\serial_bridge.py
# 纯直连监控(与桥二选一)
python -X utf8 tools\serial_monitor.py --port COM4 --baud 781250 --tests
# 重烧固件
powershell -ExecutionPolicy Bypass -File ccs\build_fw.ps1
& C:\ti\ccs2040\ccs\ccs_base\DebugServer\bin\DSLite.exe flash --config C:\ti\c2000\C2000Ware_6_00_01_00\device_support\f28002x\common\targetConfigs\TMS320F280025C_LaunchPad.ccxml -e -u -f E:\f280025c_modbus_slave\build\f280025c_modbus_slave.out
```

上位机配合:网页「桥接」→ `ws://127.0.0.1:8765`,其 `DEFAULT_BAUD` 应为 **781250**。
