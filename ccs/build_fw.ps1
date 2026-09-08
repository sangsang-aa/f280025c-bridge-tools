# build_fw.ps1 - 用 C2000 CGT 命令行编译 F280025C Modbus RTU 从站(LAUNCHXL-F280025C, FLASH)
# 用法: powershell -ExecutionPolicy Bypass -File .\build_fw.ps1
$ErrorActionPreference = 'Stop'

$CGT     = 'C:\ti\ccs2040\ccs\tools\compiler\ti-cgt-c2000_22.6.3.LTS'
$DLIB    = 'C:\ti\c2000\C2000Ware_6_00_01_00\driverlib\f28002x\driverlib'
$WARE    = 'C:\ti\c2000\C2000Ware_6_00_01_00'
$DEVSRC  = 'C:\ti\c2000\C2000Ware_6_00_01_00\device_support\f28002x\common'
$ROOT    = 'E:\f280025c_modbus_slave'
$BUILD   = Join-Path $ROOT 'build'
$OBJ     = Join-Path $BUILD 'obj'
$DEV     = Join-Path $OBJ 'device'

New-Item -ItemType Directory -Force -Path $DEV | Out-Null

Copy-Item "$DEVSRC\include\driverlib.h" $DEV -Force
Copy-Item "$DEVSRC\include\device.h"    $DEV -Force
Copy-Item "$DEVSRC\source\device.c"     $DEV -Force
Copy-Item "$DEVSRC\source\f28002x_codestartbranch.asm" $DEV -Force

$common = @('-v28','-ml','-mt','--abi=eabi','--float_support=fpu32','--c11',
             '--define=_FLASH','--define=_LAUNCHXL_F280025C','-diag_suppress=10063',
             "-I$CGT\include","-I$DEV","-I$DLIB","-I$WARE")

$cl2000 = Join-Path $CGT 'bin\cl2000.exe'

Write-Host "== Compiling device.c..."
& $cl2000 @common -c "$DEV\device.c" --output_file="$OBJ\device.obj"
if ($LASTEXITCODE -ne 0) { throw "device.c failed ($LASTEXITCODE)" }

Write-Host "== Assembling f28002x_codestartbranch.asm..."
& $cl2000 @common -c "$DEV\f28002x_codestartbranch.asm" --output_file="$OBJ\codestart.obj"
if ($LASTEXITCODE -ne 0) { throw "codestart failed ($LASTEXITCODE)" }

Write-Host "== Compiling modbus_rtu.c..."
& $cl2000 @common -c "$ROOT\firmware\modbus_rtu.c" --output_file="$OBJ\modbus_rtu.obj"
if ($LASTEXITCODE -ne 0) { throw "modbus_rtu.c failed ($LASTEXITCODE)" }

Write-Host "== Compiling F280025C_modbus_slave.c..."
& $cl2000 @common -c "$ROOT\firmware\F280025C_modbus_slave.c" --output_file="$OBJ\F280025C_modbus_slave.obj"
if ($LASTEXITCODE -ne 0) { throw "F280025C_modbus_slave.c failed ($LASTEXITCODE)" }

Write-Host "== Linking..."
$out = Join-Path $BUILD 'f280025c_modbus_slave.out'
& $cl2000 '--abi=eabi' '-z' "-o$out" "-m$(Join-Path $BUILD 'f280025c_modbus_slave.map')" -stack 0x200 -heap 0x100 --entry_point code_start "$DEVSRC\cmd\28002x_generic_flash_lnk.cmd" "$OBJ\device.obj" "$OBJ\codestart.obj" "$OBJ\modbus_rtu.obj" "$OBJ\F280025C_modbus_slave.obj" "$DLIB\ccs\Debug\driverlib.lib" "$CGT\lib\rts2800_fpu32_eabi.lib"
if ($LASTEXITCODE -ne 0) { throw "link failed ($LASTEXITCODE)" }

Write-Host "BUILD OK: $out"
