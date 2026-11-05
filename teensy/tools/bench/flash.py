"""Flash the Teensy from its running console. usage: flash.py PORT [hex]
mode off, soft reboot into HalfKay, then hkflash.py (keeps offering a block while HalfKay stalls it with error 31;
teensy_loader_cli gives up and can leave the bootloader refusing everything until a button press / replug).
Tested so far with the extender OUT only (2026-11-04). If it reports "refused": press the Teensy button and run
hkflash.py <hex> directly."""
import os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from sdlink import Link
TOOLS = r"<PLATFORMIO_CORE_DIR>\packages\tool-teensy"
port = sys.argv[1]
hexf = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "..", "..", ".pio", "build", "teensy41", "firmware.hex")
try:
    with Link(port) as L:
        L.write_line("mode off")
        time.sleep(0.3)
except Exception as e:
    print("console not reachable (already in bootloader?):", e)
subprocess.run([os.path.join(TOOLS, "teensy_reboot.exe"), "-s"], capture_output=True)
time.sleep(2)
r = subprocess.run([sys.executable, os.path.join(HERE, "hkflash.py"), hexf])
if r.returncode:
    sys.exit(r.returncode)
time.sleep(3)
with Link(port) as L:
    for ln in L.cmd("ver"):
        print(ln)
