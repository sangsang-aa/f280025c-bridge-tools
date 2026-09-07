// boot_run.mjs — USB 插拔(断电重启)后执行:解除 F280025C BROM WAIT,使固件从 Flash 自动运行
// 原理: 注入 EMU_BOOTPIN_CONFIG=0x5A + EMU_BOOTDEF=FLASH(0x03),设 PC=code_start 并运行。
// 用法: ccs-server 脚本环境 run.bat boot_run.mjs(由 fix_boot.ps1 调用,约 5 秒)
const ds = initScripting();

ds.configure("C:/ti/c2000/C2000Ware_6_00_01_00/device_support/f28002x/common/targetConfigs/TMS320F280025C_LaunchPad.ccxml");
const session = ds.openSession(/c28/i);
session.target.connect();
session.target.halt();

session.memory.write(0xd00, [0x5a000000, 0x03030303, 0x03030303, 0x03030303], 32);
session.registers.write("PC", 0x80000);
session.target.run(false);
await sleep(500);

console.log("BOOT RELEASED: firmware running from flash (tick=" +
  session.memory.read(0xa712, 1, 16)[0].toString(16) + ").");
ds.shutdown();
