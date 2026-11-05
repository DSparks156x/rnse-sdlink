"""selftest.py -- compare the firmware's CRC / response / wire-image / volume values with sd_wire.py.

    <python> -I selftest.py            # Teensy on USB: sends 'selftest', compares every ST line
    <python> -I selftest.py --host     # no hardware: builds tools/host_selftest natively (zig c++ from
                                       #   the teensy/.venv) from the SAME sdproto.cpp/selftest.cpp and compares
    <python> -I selftest.py --port COM7

Also checks a few full sectors / wire images over the console ('sec', 'wire') on the device.
Exit code 0 = all match.
"""
import argparse, os, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sdemu_ref as REF  # noqa: E402
import sd_wire as W      # noqa: E402  (path set up by sdemu_ref)


def parse_st(lines):
    got = {}
    for ln in lines:
        if ln.startswith("ST ") and not ln.startswith(("ST begin", "ST end")):
            parts = ln.split(" ", 2)
            if len(parts) == 3:
                got[parts[1]] = parts[2].strip()
    return got


def compare(got, exp):
    bad = 0
    for k, v in exp.items():
        g = got.get(k)
        if g is None:
            print(f"  MISSING {k}")
            bad += 1
        elif g.upper() != v.upper():
            print(f"  MISMATCH {k}\n     firmware {g[:96]}\n     expected {v[:96]}")
            bad += 1
    for k in ("parsers", "avstream", "internal"):
        v = got.get(k, "missing")
        ok = v.startswith("PASS")
        print(f"  {k:9} {v}")
        bad += 0 if ok else 1
    return bad


def host_lines():
    root = os.path.normpath(os.path.join(HERE, ".."))
    exe = os.path.join(HERE, "host_selftest", "host_selftest.exe" if os.name == "nt" else "host_selftest")
    py = os.path.join(root, ".venv", "Scripts" if os.name == "nt" else "bin", "python.exe" if os.name == "nt" else "python")
    src = [os.path.join(HERE, "host_selftest", "main.cpp"), os.path.join(root, "sdemu", "sdproto.cpp"),
           os.path.join(root, "sdemu", "selftest.cpp"), os.path.join(root, "sdemu", "avstream.cpp")]
    cmd = [py, "-m", "ziglang", "c++", "-O2", "-std=c++17", "-w", "-DAVS_HOST", "-o", exe] + src
    print("building host_selftest:", " ".join(os.path.basename(c) for c in cmd[3:]))
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        print(r.stderr[-3000:])
        sys.exit("host build failed (pip install ziglang into teensy/.venv)")
    out = subprocess.run([exe], capture_output=True, text=True).stdout
    return out.splitlines()


def device_checks(link):
    """full sector and wire-image spot checks over the console"""
    bad = 0
    for lba in (0, 2048, REF.LBA_FAT1, REF.LBA_ROOT, REF.LBA_DATA, 0x40100):
        ln = [x for x in link.cmd(f"sec {lba}", until="SEC", timeout=3) if x.startswith("SEC")]
        data = bytes.fromhex(ln[0].split()[2]) if ln else b""
        ok = data == REF.gen_sector(lba)
        bad += not ok
        print(f"  sec {lba:7} {'ok' if ok else 'MISMATCH'}")
    for lba, width in ((0x40100, 1), (0x40100, 4), (REF.LBA_ROOT, 4)):
        ln = [x for x in link.cmd(f"wire {lba} {width}", until="WIRE", timeout=3) if x.startswith("WIRE")]
        got = ln[0].split()[2] if ln else ""
        f = W.wire4 if width == 4 else W.wire1
        exp = "".join(f"{w:08X}" for w in f(REF.gen_sector(lba)))
        ok = got.upper() == exp
        bad += not ok
        print(f"  wire {lba} {width}-bit {'ok' if ok else 'MISMATCH'}")
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", action="store_true", help="native build of the firmware code, no Teensy")
    ap.add_argument("--port")
    a = ap.parse_args()
    exp = REF.expected_selftest()
    if a.host:
        got = parse_st(host_lines())
        bad = compare(got, exp)
    else:
        from sdlink import Link
        with Link(a.port) as t:
            t.cmd("mode off")
            print("device:", " ".join(t.cmd("ver")))
            got = parse_st(t.cmd("selftest", until="ST end", timeout=5))
            bad = compare(got, exp)
            bad += device_checks(t)
    print(f"{len(exp)} reference values compared: {'ALL MATCH' if not bad else f'{bad} PROBLEM(S)'}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
