# fix_boot.ps1 — 板子 USB 插拔(断电重启)后必须执行一次(约 5~15 秒)
# 作用: 解除 F280025C BROM 的 WAIT 启动状态,让已烧录的 Modbus 固件从 Flash 自动运行。
# 之后主控即独立运行(无需持续连接调试器),可直接启动桥接 + Web 上位机。
# 用法: powershell -ExecutionPolicy Bypass -File E:\f280025c_modbus_slave\ccs\fix_boot.ps1

$ErrorActionPreference = 'Stop'
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path

Write-Host "[1/2] 释放启动(BROM WAIT -> Flash)..."
& "C:\ti\ccs2040\ccs\scripting\run.bat" (Join-Path $scriptRoot 'boot_run.mjs') | Out-Host

Write-Host "[2/2] 验证串口应答..."
python -X utf8 (Join-Path $scriptRoot 'verify_uart.py')
