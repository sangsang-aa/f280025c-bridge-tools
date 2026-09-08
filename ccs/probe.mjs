const ds = initScripting();

ds.configure("C:/ti/c2000/C2000Ware_6_00_01_00/device_support/f28002x/common/targetConfigs/TMS320F280025C_LaunchPad.ccxml");
const session = ds.openSession(/c28/i);
session.target.connect();
session.target.halt();
console.log("EMU KEY check: @0xD00 =", (session.memory.read(0xd00, 1, 32)[0]).toString(16));

let w = session.memory.read(0xa710, 4, 16);
console.log("pre-run g_txLen/g_tickMs:", w.map(x => x.toString(16)).join(" "));

session.registers.write("PC", 0x80000);
session.target.run(false);
await sleep(2500);
session.target.halt();
let w2 = session.memory.read(0xa710, 4, 16);
console.log("post-run g_tickMs:", w2.map(x => x.toString(16)).join(" "));
console.log("PC = 0x" + session.registers.read("PC").toString(16));

ds.shutdown();
