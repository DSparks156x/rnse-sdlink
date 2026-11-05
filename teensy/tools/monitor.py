"""monitor.py -- stage S1: passive monitor of the RNS-E SD slot (Teensy never drives a pin).

    <python> -I monitor.py --seconds 120 --log s1_realcard.txt
    <python> -I monitor.py --phase 1          # try the other word-timer phase if CRC errors appear
    <python> -I monitor.py --no-dat           # CMD only (lighter CPU at 25 MHz)

Prints every decoded command/response and a 1 s STAT line, then a summary:
  * SDCLK frequency (ID mode ~390 kHz? transfer ~25 MHz? patched ~3.1 MHz?) and duty cycle
  * the init sequence the RNS-E sends (CMD0, CMD8, CMD55/ACMD41 ..., CMD2, CMD3, CMD9, CMD7 ...)
  * CRC7 errors on host commands and card responses, CRC16 results of data blocks (1-bit / 4-bit)
  * slot VDD transitions and idle line levels (host pull-ups present?)
"""
import argparse, collections, time
from sdlink import Link, parse_kv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--log", default="monitor_log.txt")
    ap.add_argument("--phase", type=int, default=0, help="0 rising-enable, 1 falling-enable, 2 free, 3 wrong edge")
    ap.add_argument("--no-dat", action="store_true")
    ap.add_argument("--verbose-blocks", action="store_true", help="log every data block (log 2)")
    ap.add_argument("--quiet", action="store_true", help="do not echo C/R/D lines")
    a = ap.parse_args()

    hist = collections.Counter()
    init_seq, seen_read = [], False
    freqs, vdd_events = [], []
    last_stat = {}
    with Link(a.port) as t, open(a.log, "w") as lf:
        t.cmd("mode off")
        for c in (f"cfg dat {0 if a.no_dat else 1}", f"cfg log {2 if a.verbose_blocks else 1}",
                  f"phase {a.phase}", "perstat 1", "clr"):
            t.cmd(c)
        print(" ".join(t.cmd("pins")), "(mode off: plain GPIO inputs, no pulls)")
        t.cmd("mode mon")
        print(f"monitoring {a.seconds:.0f} s, log -> {a.log}")
        end = time.time() + a.seconds
        while time.time() < end:
            ln = t.read_line(0.5)
            if not ln:
                continue
            lf.write(ln + "\n")
            if ln.startswith("C "):
                kv = parse_kv(ln)
                name = ln.split()[2]
                hist[name] += 1
                if not seen_read:
                    init_seq.append(name + ("" if kv.get("crc") == "ok" else "(CRC!)"))
                    if name in ("CMD17", "CMD18"):
                        seen_read = True
            elif ln.startswith("E ") and "VDD" in ln:
                vdd_events.append(ln)
            elif ln.startswith("STAT"):
                last_stat = parse_kv(ln)
                for k in ("clk_avg", "clk_peak", "clk_min"):
                    if isinstance(last_stat.get(k), int) and last_stat[k] > 0:
                        freqs.append(last_stat[k])
                if not a.quiet:
                    print(ln)
                continue
            if not a.quiet:
                print(ln)
        duty = t.cmd("duty 2000000", until="DUTY", timeout=10)
        ctr = {}
        t.write_line("ctr")
        for _ in range(4):
            x = t.read_line(1)
            if x and x.startswith("CTR"):
                ctr.update(parse_kv(x))
        t.cmd("mode off")

    print("\n================ S1 summary ================")
    if freqs:
        print(f"SDCLK seen: min {min(freqs) / 1e3:.1f} kHz, max {max(freqs) / 1e6:.3f} MHz "
              f"(ID mode should be ~390 kHz, stock transfer ~25 MHz, /16 patch ~3.1 MHz)")
    else:
        print("SDCLK: no edges counted (clock not running, CLK wire, or slot not powered)")
    print("duty:", " ".join(duty))
    print("VDD events:", vdd_events or "none", "| last vdd", last_stat.get("vddv"), "V")
    print("init sequence (until first read):", " ".join(init_seq[:60]) or "none")
    print("command histogram:", dict(hist.most_common(20)))
    hc, hb = ctr.get("host_cmds", 0), ctr.get("host_crc_bad", 0)
    rs, rb = ctr.get("resp_seen", 0), ctr.get("resp_crc_bad", 0)
    print(f"host commands {hc}, CRC7 bad {hb} | card responses {rs}, CRC bad {rb}")
    print(f"data blocks ok {ctr.get('dat_ok', 0)} bad {ctr.get('dat_bad', 0)} "
          f"(1-bit {ctr.get('dat_w1', 0)}, 4-bit {ctr.get('dat_w4', 0)}); ring overruns "
          f"cmd {ctr.get('ring_overrun_cmd', 0)} dat {ctr.get('ring_overrun_dat', 0)}")
    if hc and hb == 0 and rb == 0:
        print("VERDICT: CMD sampling clean at this phase.")
    elif hc:
        print("VERDICT: CRC errors -> try --phase 1 / 2, check CLK wiring/ground, shorten leads (README S1).")
    else:
        print("VERDICT: no commands decoded -> check CMD/CLK wiring, VDD, that the RNS-E accesses the slot.")


if __name__ == "__main__":
    main()
