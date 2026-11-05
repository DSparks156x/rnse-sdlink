"""sweep.py -- software "eye" sweeps (no scope needed): step the timing/pad knobs, count errors per point.

Monitor mode (passive, safe, real card in the slot):  sweeps the RX word phase presets and reports
CRC7 / CRC16 error rates of what the real card and host exchange. Finds the receive setting to use.
    <python> -I sweep.py --mode mon --seconds 20

Card mode (Teensy is the card, RNS-E must be reading continuously, e.g. a raw-read loop of the test
pattern or a media scan): sweeps TX launch edge x pad drive (DSE) x slew LIVE (no card re-init) and
reports host retries (a re-read of the same address = host saw a CRC error), DAT/CMD loopback errors.
    <python> -I sweep.py --mode card --seconds 30
    <python> -I sweep.py --mode card --dse 3,4,5,6 --launch 0,1 --sre 0

Prints a table; the best point is the one with zero errors in the middle of a zero-error region.
"""
import argparse, itertools, time
from sdlink import Link, get_counters


def run_point(t, settings, seconds):
    for c in settings:
        t.cmd(c)
    t.cmd("clr")
    t0 = time.time()
    while time.time() - t0 < seconds:
        t.read_line(0.5)            # drain logs while the point runs
    return get_counters(t)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--mode", choices=("mon", "card"), default="mon")
    ap.add_argument("--seconds", type=float, default=20)
    ap.add_argument("--dse", default="2,3,4,5,6,7")
    ap.add_argument("--launch", default="0,1")
    ap.add_argument("--sre", default="0,1")
    ap.add_argument("--phases", default="0,1,2,3")
    a = ap.parse_args()
    rows = []
    with Link(a.port) as t:
        t.cmd("mode off")
        t.cmd("perstat 0")
        t.cmd("cfg log 0")
        if a.mode == "mon":
            for p in [int(x) for x in a.phases.split(",")]:
                t.cmd(f"phase {p}")
                t.cmd("mode mon")
                c = run_point(t, [], a.seconds)
                rows.append((f"phase {p}", c))
                t.cmd("mode off")
        else:
            t.cmd("phase 0")
            t.cmd("mode card")
            print("card armed; make sure the RNS-E has initialised the card and is reading continuously")
            time.sleep(2)
            for launch, dse, sre in itertools.product([int(x) for x in a.launch.split(",")],
                                                     [int(x) for x in a.dse.split(",")],
                                                     [int(x) for x in a.sre.split(",")]):
                c = run_point(t, [f"cfg launch {launch}", f"cfg dse {dse}", f"cfg sre {sre}"], a.seconds)
                rows.append((f"launch {launch} dse {dse} sre {sre}", c))
        t.cmd("cfg log 1")
        t.cmd("perstat 1")
        t.cmd("mode off")

    print("\n" + "=" * 100)
    if a.mode == "mon":
        print(f"{'point':12} {'cmds':>7} {'crc7bad':>8} {'resp':>7} {'rcrcbad':>8} {'blk_ok':>8} {'blk_bad':>8}")
        for name, c in rows:
            print(f"{name:12} {c.get('host_cmds', 0):7} {c.get('host_crc_bad', 0):8} {c.get('resp_seen', 0):7} "
                  f"{c.get('resp_crc_bad', 0):8} {c.get('dat_ok', 0):8} {c.get('dat_bad', 0):8}")
        print("expect: phases 0-2 clean, phase 3 (falling-edge sample) full of errors = the sweep can see errors.")
    else:
        print(f"{'point':24} {'reads':>7} {'sectors':>8} {'retries':>8} {'cmd0':>5} {'crc7bad':>8} "
              f"{'echo_bad':>8} {'late_tx':>8} {'lbk_ok':>7} {'lbk_bad':>7}")
        for name, c in rows:
            print(f"{name:24} {c.get('cmd17', 0) + c.get('cmd18', 0):7} {c.get('sectors_served', 0):8} "
                  f"{c.get('retries', 0):8} {c.get('cmd0', 0):5} {c.get('host_crc_bad', 0):8} "
                  f"{c.get('resp_crc_bad', 0):8} {c.get('tx_underrun_resp', 0):8} {c.get('dat_ok', 0):7} "
                  f"{c.get('dat_bad', 0):7}")
        print("retries/cmd0 = the HOST rejected something (its CRC16/CRC7 or a timeout). lbk_bad/echo_bad = our own "
              "pins read back wrong (contention/short). late_tx = firmware fed a response word too late.")


if __name__ == "__main__":
    main()
