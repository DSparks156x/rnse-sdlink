"""Live knob sweep: for each setting run card mode N s and print how far the RNS-E got.
usage: knobtest.py PORT SECS "launch 0" "launch 1" "launch 1,dse 6" ..."""
import sys, time, collections
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from sdlink import Link
port, secs, sets = sys.argv[1], float(sys.argv[2]), sys.argv[3:]
with Link(port) as L:
    L.cmd("mode card"); 
    for st in sets:
        for kv in st.split(","):
            L.write_line("cfg " + kv.strip())
        time.sleep(0.2); L.write_line("clr")
        cnt = collections.Counter(); bad = []
        t = time.time(); ctr = ""
        while time.time() - t < secs:
            s = L.read_line(timeout=0.2)
            if s and s[:2] == "C ":
                cnt[s.split()[2]] += 1
            elif s and s[:2] in ("R ", "D "):
                if s[:2] == "R " and len(bad) < 4: bad.append(s)
        L.write_line("ctr"); t = time.time(); lines = []
        while time.time() - t < 0.6:
            s = L.read_line(timeout=0.2)
            if s and s.startswith("CTR"): lines.append(s)
        print("==", st, dict(cnt)); [print("  ", x) for x in lines[:1]]; [print("  ", x) for x in bad]
    L.write_line("mode off")
