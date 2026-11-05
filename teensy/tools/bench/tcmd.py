"""Send console commands to the Teensy and print replies. usage: tcmd.py PORT "cmd1" "cmd2" ..."""
import sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from sdlink import Link
port, cmds = sys.argv[1], sys.argv[2:]
with Link(port) as L:
    for c in cmds:
        if c.startswith("sleep "):
            time.sleep(float(c[6:])); continue
        L.write_line(c)
        t = time.time()
        while time.time() - t < 1.5:
            s = L.read_line(timeout=0.3)
            if s:
                print(s)
