"""usb_speed.py -- stage S0: USB High-Speed throughput between the PC/Pi and the Teensy (USB serial).

    <python> -I usb_speed.py                 # 64 MB Teensy->host, then 64 MB host->Teensy
    <python> -I usb_speed.py --mb 256 --dir rx
    <python> -I usb_speed.py --soak 600      # SPEC stage 0: host->Teensy for 10 min, report min/avg MB/s

Pattern: little-endian u32 counter, verified on the receiving side (errors = discontinuities).
SPEC 5.1 need: ~5.6 MB/s OUT (host->Teensy); production target >= 15 MB/s sustained.
"""
import argparse, array, sys, time
from sdlink import Link


def pattern(start_word, nbytes):
    a = array.array("I", range(start_word, start_word + nbytes // 4))
    if sys.byteorder != "little":
        a.byteswap()
    return a.tobytes()


def tx_test(t, mb, verify=True):
    t.write_line(f"usbtx {mb}")
    n = None
    end = time.time() + 5
    while time.time() < end:
        ln = t.read_line(1)
        if ln and ln.startswith("USBTX BEGIN"):
            n = int(ln.split()[2])
            break
        if ln and ln.startswith("ERR"):
            sys.exit(ln)
    if n is None:
        sys.exit("no USBTX BEGIN (firmware not in 'mode off'?)")
    t0 = time.time()
    data = t.read_exact(n, timeout=10)
    dt = time.time() - t0
    tail = [t.read_line(2), t.read_line(2)]
    fw_us = next((int(x.split("us=")[1]) for x in tail if x and "USBTX END" in x), None)
    errs = 0
    if verify and len(data) == n:
        got = array.array("I"); got.frombytes(data[: n - n % 4])
        step = max(1, len(got) // 4096)            # spot-check 4096 words + exact length
        errs = sum(1 for i in range(0, len(got), step) if got[i] != (i & 0xFFFFFFFF))
    mbps = len(data) / dt / 1e6
    print(f"Teensy->host: {len(data)}/{n} bytes in {dt:.3f} s = {mbps:.1f} MB/s (host clock)"
          + (f", firmware {n / fw_us:.1f} MB/s" if fw_us else "") + f", pattern errors {errs}")
    return mbps if len(data) == n and errs == 0 else 0.0


def rx_test(t, mb, chunks=None):
    n = mb * 1048576
    chunk = 1 << 20
    if chunks is None:
        chunks = [pattern(i * chunk // 4, chunk) for i in range(mb)]
    t.write_line(f"usbrx {mb}")
    end = time.time() + 5
    while time.time() < end:
        ln = t.read_line(1)
        if ln and ln.startswith("USBRX READY"):
            break
        if ln and ln.startswith("ERR"):
            sys.exit(ln)
    else:
        sys.exit("no USBRX READY (firmware not in 'mode off'?)")
    t0 = time.time()
    for c in chunks:
        t.ser.write(c)
    t.ser.flush()
    done = None
    end = time.time() + 15
    while time.time() < end:
        ln = t.read_line(1)
        if ln and ln.startswith("USBRX DONE"):
            done = ln
            break
    dt = time.time() - t0
    if not done:
        print("host->Teensy: no USBRX DONE")
        return 0.0
    kv = dict(x.split("=") for x in done.split()[2:])
    got, us, errs = int(kv["bytes"]), int(kv["us"]), int(kv["errors"])
    mbps_fw = got / us if us else 0
    print(f"host->Teensy: {got}/{n} bytes, firmware {mbps_fw:.1f} MB/s, host {n / dt / 1e6:.1f} MB/s, "
          f"pattern errors {errs}")
    return mbps_fw if got == n and errs == 0 else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--mb", type=int, default=64)
    ap.add_argument("--dir", choices=("both", "tx", "rx"), default="both")
    ap.add_argument("--soak", type=int, default=0, help="seconds of repeated host->Teensy runs")
    a = ap.parse_args()
    with Link(a.port) as t:
        t.cmd("mode off")
        t.cmd("perstat 0")
        if a.soak:
            chunks = [pattern(i * (1 << 18), 1 << 20) for i in range(a.mb)]
            rates, t_end = [], time.time() + a.soak
            while time.time() < t_end:
                r = rx_test(t, a.mb, chunks)
                rates.append(r)
                if r == 0.0:
                    print("FAIL: data loss or error during soak")
                    break
            ok = rates and min(rates) > 0
            print(f"soak: {len(rates)} runs, min {min(rates):.1f} avg {sum(rates) / len(rates):.1f} MB/s"
                  f" -> {'PASS' if ok and min(rates) >= 15 else 'below the 15 MB/s target' if ok else 'FAIL'}")
            return
        if a.dir in ("both", "tx"):
            tx_test(t, a.mb)
        if a.dir in ("both", "rx"):
            rx_test(t, a.mb)


if __name__ == "__main__":
    main()
