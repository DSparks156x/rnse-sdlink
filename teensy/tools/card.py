"""card.py -- stages S2/S3: Teensy answers as the SD card (FAT16 1 GiB test volume).

    <python> -I card.py --seconds 300 --log s2_card.txt
    <python> -I card.py --blockaddr 1        # if the CMDLOG shows block (not byte) addresses
    <python> -I card.py --busy 5             # answer ACMD41 'busy' 5 times before 'ready'

Order: start this script FIRST (Teensy armed, VDD-gated), THEN power the RNS-E / insert the extender.
Summary explains where the init stopped and what each counter means (README S2/S3).
"""
import argparse, time
from sdlink import Link, parse_kv

INIT_ORDER = ["CMD0", "ACMD41", "CMD2", "CMD3", "CMD9", "CMD7", "CMD16", "ACMD6", "ACMD13", "ACMD51", "CMD17/18"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--seconds", type=float, default=120)
    ap.add_argument("--log", default="card_log.txt")
    ap.add_argument("--blockaddr", type=int, default=0)
    ap.add_argument("--busy", type=int, default=2)
    ap.add_argument("--illegal", type=int, default=1)
    ap.add_argument("--phase", type=int, default=1)   # 1 = bench-proven on the RNS-E
    ap.add_argument("--launch", type=int, default=1)   # 1 = falling, what the RNS-E needs at 25 MHz
    ap.add_argument("--stay", action="store_true")     # leave card mode on at exit (the RNS-E keeps its card)
    ap.add_argument("--no-lbk", action="store_true")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()
    seen, answered = set(), set()
    reads, widths = 0, set()
    last = {}
    with Link(a.port) as t, open(a.log, "w", buffering=1) as lf:
        t.cmd("mode off")
        for c in (f"cfg blockaddr {a.blockaddr}", f"cfg acmd41busy {a.busy}", f"cfg illegal {a.illegal}",
                  f"cfg lbk {0 if a.no_lbk else 1}", "cfg log 1", f"phase {a.phase}", f"cfg launch {a.launch}", "perstat 1", "clr", "trace arm"):
            t.cmd(c)
        t.write_line("mode card")             # no t.cmd(): it would discard the first C/R lines of a running host
        print(f"armed for {a.seconds:.0f} s. Now power the RNS-E / insert the card. Log -> {a.log}")
        end = time.time() + a.seconds
        while time.time() < end:
            ln = t.read_line(0.5)
            if not ln:
                continue
            lf.write(ln + "\n")
            if ln.startswith("C "):
                kv = parse_kv(ln)
                name = ln.split()[2]
                key = "CMD17/18" if name in ("CMD17", "CMD18") else name
                seen.add(key)
                if kv.get("resp") == 1:
                    answered.add(key)
                if key == "CMD17/18":
                    reads += 1
            elif ln.startswith("E ") and "ACMD6" in ln:
                widths.add(ln.split("width=")[1])
            elif ln.startswith("STAT"):
                last = parse_kv(ln)
            if not a.quiet:
                print(ln)
        t.write_line("trace")                 # CMD7 word trace (dcyc = CPU cycles per 16 SDCLK)
        for _ in range(200):
            ln = t.read_line(1.0)
            if ln is None:
                break
            lf.write(ln + "\n")
            if ln.startswith("TR end") or ln.startswith("TR not"):
                break
        ctr = {}
        t.write_line("ctr")
        for _ in range(4):
            x = t.read_line(1)
            if x and x.startswith("CTR"):
                ctr.update(parse_kv(x))
        if not a.stay:
            t.cmd("mode off")

    print("\n================ S2/S3 summary ================")
    print("init progress:", " ".join(f"{s}{'+' if s in answered else ('?' if s in seen else '-')}" for s in INIT_ORDER),
          "  (+ answered, ? seen not answered, - never seen)")
    print(f"bus width(s) selected: {sorted(widths) or ['1bit (no ACMD6)']}; final state {last.get('state')}")
    print(f"reads {reads}, sectors served {ctr.get('sectors_served', 0)}, retries {ctr.get('retries', 0)}, "
          f"CMD12 {ctr.get('cmd12', 0)}, CMD0 (re-inits) {ctr.get('cmd0', 0)}")
    print(f"host CRC7 bad {ctr.get('host_crc_bad', 0)}/{ctr.get('host_cmds', 0)}; illegal {ctr.get('illegal', 0)}; "
          f"suppressed (no VDD) {ctr.get('suppressed_novdd', 0)}")
    print(f"CMD loopback: responses sent {ctr.get('responded', 0)}, echoed {ctr.get('resp_seen', 0)}, "
          f"echo CRC bad {ctr.get('resp_crc_bad', 0)}, late TX feeds {ctr.get('tx_underrun_resp', 0)}, "
          f"CMD RX overruns {ctr.get('rx_overrun', 0)}")
    print(f"DAT loopback: blocks ok {ctr.get('dat_ok', 0)} bad {ctr.get('dat_bad', 0)}")
    hints = []
    if not seen:
        hints.append("no commands: VDD not sensed (see STAT vdd=), CLK/CMD wiring, or RNS-E not polling the slot")
    elif "ACMD41" in seen and "CMD2" not in seen:
        hints.append("stuck at ACMD41: host does not see our R1/R3 -> CMD drive (dse), launch edge, CMD wiring")
    elif "CMD9" in seen and "CMD7" not in seen:
        hints.append("host rejected CSD? check R2 timing (CMD loopback echo CRC), try 'cfg launch 1' at low clock")
    elif "CMD17/18" in seen and ctr.get("retries", 0) > reads // 10:
        hints.append("many read retries: host CRC16 errors on DAT -> eye sweep (sweep.py --mode card)")
    if ctr.get("suppressed_novdd", 0):
        hints.append("commands arrived while VDD was not sensed: check the divider on pin 14")
    for h in hints:
        print("HINT:", h)


if __name__ == "__main__":
    main()
