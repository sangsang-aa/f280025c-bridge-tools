# f280025c_modbus_slave — 调试工具(不入打包)

> ⚠️ 本目录是 F280025C Modbus RTU 从站固件的**调试/联调工具**,仅用于本地开发,打包 ai_motor_control 应用时要排除。
> 源码主仓在 WSL:`~/f280025c_modbus_slave`,本目录与源码仓是副本关系(修改后请自行同步)。

## 目录

```
E:\f280025c_modbus_slave\
├── firmware\                 固件源码 + host 侧 gcc 单测
│   ├── F280025C_modbus_slave.c   C2000Ware 6.x 宿主层(SCI-A, GPIO29=TX/28=RX)
│   ├── modbus_rtu.h/.c           可移植协议核心(CRC/01/03/05/06/0F/10, float32 BE)
│   └── README.md                固件细节(引脚/API/波特率说明)
├── tools\
│   ├── serial_monitor.py        直连 COM 的 Modbus 测试/监控(--tests / --poll / shell)
│   └── serial_bridge.py         本地串口桥(Web 上位机桥接模式 + 实时帧日志)
└── ccs\
    ├── build_fw.ps1             CGT 命令行编译(输出 build\f280025c_modbus_slave.out)
    ├── fix_boot.ps1             USB 插拔后必跑:解除 BROM WAIT + 放行 Flash 运行 + 自检
    ├── boot_run.mjs             fix_boot 内部脚本(EMU 启动注册写入 + PC=0x80000 + run)
    ├── verify_uart.py           fix_boot 内部脚本(读 0x0000 寄存器验证固件已运行)
    └── probe.mjs                调试诊断脚本(读 PC/内存,排查用)
```

## 关键事实

| 项 | 值 | 说明 |
|---|---|---|
| 固件 | F280025C Modbus RTU 从站,8N1,从站地址 0x01 | 已烧入开发板 |
| 波特率 | **1041667** | =25MHz/24 精确分频;921600 在此 LSPCLK 下无整数解(最近真值 781250/1041667,偏差≥13%),**上位机需用 1041667** |
| 波形区 | 0x2000..0x20C7 = 100×float32(big-endian 高字低地址),0x2200=批次号(只读),0x2201 bit0=新批次就绪(0x03 读后自动清,0x06 写 0 也可清) | 5kHz 上传采集;100 点攒满冻结再发布,主站读到的一定是完整一批。0x03 单次最多 125 寄存器,200 寄存器分两次读(0x2000×125 + 0x207D×75) |
| SCI 引脚 | GPIO29=TX, GPIO28=RX | C2000Ware 6.x 宏名 `GPIO_29_SCIA_TX` / `GPIO_28_SCIA_RX` |
| 串口 | XDS110 **Application/User UART**,默认 COM4 | --port auto 自动识别;怕 COM 漂移 |
| 烧写 | DSLite(UniFlash CLI)+ TMS320F280025C_LaunchPad.ccxml | XDS110 固件提示升级到 3.0.0.41(可选) |

## 日常流程(调试)

```powershell
# 1) 板子插上 USB(断电重启后必做)
powershell -ExecutionPolicy Bypass -File E:\f280025c_modbus_slave\ccs\fix_boot.ps1
#    → 输出 "[OK] 固件已运行" 后再继续(约 10 秒)

# 2) 起桥(独占 COM4,窗口实时打印每帧 TX/RX)
python -X utf8 E:\f280025c_modbus_slave\tools\serial_bridge.py

# 2.5) 注意:Web 上位机(WSL)需把 DEFAULT_BAUD 改为 1041667(与固件一致)
#       (在 ~/ai_motor_control/lib/config.ts,改完强刷页面)

# 3) 启动 Web 上位机(WSL)
cd ~/ai_motor_control && npm run dev

# 4) 浏览器打开 localhost:300x → 顶栏点蓝色「桥接」→ 网页收发命令,桥窗口看帧
```

不拔 USB 时第 2~4 步即插即用;只做纯直连调试时可用 `serial_monitor.py --tests`(不能与桥同时开 COM4)。

## 改固件→重烧(必要时)

```powershell
powershell -ExecutionPolicy Bypass -File E:\f280025c_modbus_slave\ccs\build_fw.ps1
& C:\ti\ccs2040\ccs\ccs_base\DebugServer\bin\DSLite.exe flash --config C:\ti\c2000\C2000Ware_6_00_01_00\device_support\f28002x\common\targetConfigs\TMS320F280025C_LaunchPad.ccxml -e -u -f E:\f280025c_modbus_slave\build\f280025c_modbus_slave.out
```

## 排查清单(网页连上但无帧/无应答)

1. 看桥启动信息:检测到的串口要含 XDS110 应用 UART;`--port` 不对时用 `--port COMx`
2. 桥窗口无 `[+] 浏览器已连接` → 页面不是最新 JS(强刷 Ctrl+Shift+R / 重启 next dev)
3. 桥窗口有连接但无帧 → 板子未运行:重跑 fix_boot.ps1
4. 页面弹 `无法打开 COM...` → 先关掉 serial_monitor.py 等占串口的程序
5. `serial_bridge.py` 报端口占用 → 不要开第二个桥
