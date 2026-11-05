"""Listen-only: for each RX phase preset, count decoded host commands and CRC failures."""
import sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from sdlink import Link, get_counters
port = sys.argv[1]; secs = float(sys.argv[2]) if len(sys.argv) > 2 else 6
phases = [int(x) for x in (sys.argv[3] if len(sys.argv) > 3 else "0,1,2,3").split(",")]
with Link(port) as L:
    L.cmd("mode off"); L.cmd("cfg log 1")
    for p in phases:
        L.cmd("mode off"); L.cmd("phase %d" % p); L.cmd("mode mon"); L.cmd("clr")
        good = {}; t0 = time.time()
        while time.time() - t0 < secs:
            s = L.read_line(0.3)
            if s and s.startswith("C ") and "crc=ok" in s:
                k = s.split()[2]; good[k] = good.get(k, 0) + 1
        c = get_counters(L)
        print("phase %d: cmds=%s crc_bad=%s good=%s" % (p, c.get("host_cmds"), c.get("host_crc_bad"),
              " ".join("%s:%d" % kv for kv in sorted(good.items()))))
    L.cmd("mode off"); L.cmd("phase 0")
