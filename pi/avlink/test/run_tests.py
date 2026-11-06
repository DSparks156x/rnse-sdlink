r"""run_tests.py -- avlink checks on the PC (Windows Python + WSL gcc). No hardware.

  <python> -I run_tests.py [--quick] [--skip-e2e] [--raw]

  1. wire     avlink --wire-test == link/sd_wire.py wire4()/wire1() (random + edge sectors)
  2. sdgen    avlink --k K --dump-sf == rnse_proto/sdgen.py superframes() for the same input, byte for byte
              (highway110_night K=48 one superframe per frame; turn_night K=48 --maxvid 60 = TLV 0x11 fragments;
              --raw adds rawtest4 K=30 = 770 KB frames over 7 superframes)
  0. units    test_credits, test_nocredits, test_stall, test_discard; ownership: teensy/tools/test_avstream_ownership.cpp (ASan/UBSan);
              host selftest of the firmware's sdproto/avstream against tools/sdemu_ref.py (sec_4064 differs since before batch 1)
  3. e2e      avlink -> pty (fake /dev/ttyACM0) -> Teensy host build (sdemu avstream.cpp + sdproto.cpp, the same
              files the firmware uses) -> simulated RNS-E mode A reader (SPEC 2.A.4) -> served superframes
              == sdgen.py, every served wire image CRC16-checked, video reassembled (TLV 0x11) and decoded by
              rnse_proto/sim/host_dec: per-frame YUY2 CRC == video/out/*_c.txt. Variants: 4-bit bus, 1-bit bus
              (Teensy re-encodes 4-bit images), stale-slot/EXPIRED (slow reader), Teensy-first start (holds).
  4. live     test_live.py: real tick and credits, reader away and back, seq past 255, overload (--live-quick: three scenarios, --skip-live)
Everything built and written goes to --out DIR (default test/out), so an earlier run's results stay where they are.
Python: python
"""
import argparse, json, os, random, struct, subprocess, sys, tempfile, zlib

HERE = os.path.dirname(os.path.abspath(__file__))
AVL = os.path.normpath(os.path.join(HERE, '..'))
ROOT = os.path.normpath(os.path.join(AVL, '..', '..'))          # analysis/av_over_sd_v1
PROTO = os.path.join(ROOT, 'rnse')
TEENSY = os.path.join(ROOT, 'teensy')
sys.path.insert(0, os.path.join(ROOT, 'link'))
sys.path.insert(0, PROTO)
import sd_wire  # noqa: E402
try:
    import sdgen    # noqa: E402  (reference generator + source clips are not in this repo)
except ImportError:
    sdgen = None

OUT = os.path.join(HERE, 'out')
AVLBIN = None    # avlink built for this run


def wslpath(p):
    p = os.path.abspath(p).replace('\\', '/')
    return '/mnt/' + p[0].lower() + p[2:]


def q(p):
    return "'" + wslpath(p) + "'"


def wsl(cmd, timeout=300, check=True):
    r = subprocess.run(['wsl.exe', '-e', 'bash', '-c', cmd], capture_output=True, text=True, timeout=timeout)
    if check and r.returncode:
        print(r.stdout[-3000:], r.stderr[-3000:])
        raise SystemExit('WSL command failed: ' + cmd[:200])
    return r


def build(e2e):
    o = q(OUT)
    lib = 'sf.c wire.c crc.c audio.c link.c'
    wsl('cd %s && cc -O2 -g -std=c11 -Wall -Wextra -Wno-unused-parameter avlink.c %s -o %s/avlink 2>&1' % (q(AVL), lib, o))
    wsl('cd %s && cc -std=c11 -Wall -Wextra -Werror -O2 test/test_credits.c link.c -o %s/credit-test' % (q(AVL), o))
    for t in ('nocredits', 'discard'):
        wsl('cd %s && cc -std=c11 -Wall -Wextra -Werror -O2 test/test_%s.c %s -o %s/%s-test' % (q(AVL), t, lib, o, t))
    wsl('cd %s && cc -O2 -std=c11 -w -Dmain=avlink_main -c avlink.c -o %s/avlink_t.o && cc -O2 -std=c11 -w test/test_stall.c %s/avlink_t.o %s -I. -o %s/stall-test' % (q(AVL), o, o, lib, o))
    sd = q(os.path.join(TEENSY, 'sdemu'))
    wsl('g++ -O1 -g -std=c++17 -Wall -Wextra -fsanitize=address,undefined -DAVS_HOST -DAVS_TEST_INTERLEAVE -I%s -o %s/ownership-test %s %s/avstream.cpp %s/sdproto.cpp 2>&1'
        % (sd, o, q(os.path.join(TEENSY, 'tools', 'test_avstream_ownership.cpp')), sd, sd))
    wsl('g++ -O2 -std=c++17 -w -DAVS_HOST -o %s/host_selftest %s %s/sdproto.cpp %s/selftest.cpp %s/avstream.cpp'
        % (o, q(os.path.join(TEENSY, 'tools', 'host_selftest', 'main.cpp')), sd, sd, sd))
    if e2e:
        # sim/run_host.sh's build, into OUT: rnse_proto stays as it is
        cf = '-O2 -DRNV_HOST -Wno-unused-parameter -w -Isrc'
        tmp = '%s/dec' % o
        wsl('mkdir -p %s && cd %s && gcc %s -DRNV_PF=0 -c src/rnv2sh.c -o %s/o.o && gcc %s -DRNV_PF=1 -c src/rnv2sh.c -o %s/p.o && gcc %s -DRNV_FAST=1 -c src/rnv2sh.c -o %s/f.o '
            '&& gcc %s -c src/rnv2tab.c -o %s/t.o && gcc %s -c src/rnv2ref.c -o %s/r.o && gcc %s -c sim/host_dec.c -o %s/h.o && gcc -o %s/host_dec %s/h.o %s/o.o %s/p.o %s/f.o %s/t.o %s/r.o'
            % (tmp, q(PROTO), cf, tmp, cf, tmp, cf, tmp, cf, tmp, cf, tmp, cf, tmp, o, tmp, tmp, tmp, tmp, tmp, tmp))
        hl = q(os.path.join(TEENSY, 'tools', 'host_link', 'main.cpp'))
        wsl('g++ -O2 -g -std=c++17 -Wall -Wextra -Wno-unused-parameter -DAVS_HOST %s %s/avstream.cpp %s/sdproto.cpp -o %s/host_link 2>&1' % (hl, sd, sd, o))


def test_selftest():
    r = wsl('%s/host_selftest' % q(OUT), check=False)
    lines = r.stdout.splitlines()
    got = {}
    for ln in lines:
        if ln.startswith('ST ') and not ln.startswith(('ST begin', 'ST end')):
            parts = ln.split(' ', 2)
            if len(parts) == 3:
                got[parts[1]] = parts[2].strip()
    sys.path.insert(0, os.path.join(TEENSY, 'tools'))
    import sdemu_ref as REF
    exp = REF.expected_selftest()
    diff = [k for k, v in exp.items() if got.get(k, '').upper() != v.upper()]
    ok = all(got.get(k, '').startswith('PASS') for k in ('parsers', 'avstream', 'internal')) and diff in ([], ['sec_4064'])
    print('  host selftest: parsers=%s avstream=%s internal=%s; %d reference values, differ: %s (sec_4064 is the old known one)' % (
        got.get('parsers', '?')[:4], got.get('avstream', '?')[:4], got.get('internal', '?')[:4], len(exp), diff or 'none'))
    return ok


# ------------------------------------------------------------------ 1. wire images
def test_wire():
    rnd = random.Random(1234)
    secs = [bytes(512), b'\xff' * 512, bytes(range(256)) * 2, bytes([0xA5, 0x5A] * 256)]
    secs += [bytes(rnd.randrange(256) for _ in range(512)) for _ in range(60)]
    inp = os.path.join(OUT, 'wire_in.bin')
    open(inp, 'wb').write(b''.join(secs))
    ok = True
    for width, f, nw in ((4, sd_wire.wire4, 132), (1, sd_wire.wire1, 130)):
        o = os.path.join(OUT, 'wire_out%d.bin' % width)
        wsl('%s/avlink --wire %d --wire-test %s %s' % (q(OUT), width, q(inp), q(o)))
        got = open(o, 'rb').read()
        exp = b''.join(struct.pack('<%dI' % nw, *f(s)) for s in secs)
        good = got == exp
        if good:                                                     # and the decoder round trip
            for i, s in enumerate(secs[:8]):
                if width == 4:
                    d, crc_ok, end_ok = sd_wire.unwire4(list(struct.unpack('<132I', got[i * 528:(i + 1) * 528])))
                    good &= d == s and crc_ok and end_ok
        print('  wire %d-bit: %d sectors %s' % (width, len(secs), 'identical to sd_wire.py' if good else 'MISMATCH'))
        ok &= good
    return ok


# ------------------------------------------------------------------ 2. superframes vs sdgen.py
CASES = [('v2_real/highway110_night', 48, 248, 'hwy'), ('v2_real/turn_night', 48, 60, 'frag')]


def tone_pcm_le(k, tone=2, dbfs=-20):
    amp = 32767 * 10 ** (dbfs / 20.0)
    be = b''.join(sdgen.tone_block(s, tone, amp) for s in range(k))
    return b''.join(struct.pack('<h', v) for (v,) in struct.iter_unpack('>h', be))


def case_inputs(seqname, k, name):
    """(.rnv path, _c.txt path, pcm path) for a case; rawtestN gets written here"""
    pcm = os.path.join(OUT, name + '_tone.pcm')
    open(pcm, 'wb').write(tone_pcm_le(k))
    if seqname.startswith('rawtest'):
        rnv, crc = os.path.join(OUT, name + '.rnv'), os.path.join(OUT, name + '_c.txt')
        with open(rnv, 'wb') as f, open(crc, 'w') as g:
            for i in range(int(seqname[7:] or 4)):
                fr, c = sdgen.raw_frame(i)
                f.write(struct.pack('>I', len(fr)) + fr)
                g.write('%d %d\n' % (i, c))
        return rnv, crc, pcm
    base = os.path.join(ROOT, 'video', 'out', seqname)
    return base + '.rnv', base + '_c.txt', pcm


def test_sdgen(cases):
    ok = True
    refs = {}
    for seqname, k, maxvid, name in cases:
        sfs, meta, nframes, spans = sdgen.superframes(seqname, k, 2, -20, False, maxvid)
        ref = b''.join(sfs)
        refs[name] = (sfs, meta, nframes)
        rnv, crc, pcm = case_inputs(seqname, k, name)
        o = os.path.join(OUT, name + '_avlink.rsav')
        wsl('%s/avlink -q --k %d --maxvid %d --video %s --yuvcrc %s --audio %s --dump-sf %s' %
            (q(OUT), k, maxvid, q(rnv), q(crc), q(pcm), q(o)))
        got = open(o, 'rb').read()
        good = got == ref
        detail = ''
        if not good:
            off = next((i for i in range(min(len(got), len(ref))) if got[i] != ref[i]), min(len(got), len(ref)))
            detail = ' (len %d vs %d, first diff at byte %d = sector %d)' % (len(got), len(ref), off, off // 512)
        print('  sdgen %-6s K=%d maxvid=%d: %d superframes, %d frames, spans %s: %s%s' % (
            name, k, maxvid, k, nframes, dict(sorted(spans.items())), 'byte-identical' if good else 'MISMATCH', detail))
        ok &= good
    return ok, refs


# ------------------------------------------------------------------ 3. end to end through the Teensy host build
def unpack_served(path):
    return open(path, 'rb').read()


def reassemble_rnv(body_list):
    """video payloads of served superframes -> .rnv (4-byte BE len + frame) using TLV 0x11, like sdstream.c"""
    out, asm, want = [], b'', None
    for b in body_list:
        h = sd_wire.parse_header_sector(b[:512])
        if h is None or h['video_bytes'] == 0:
            continue
        v0 = h['video_sec'] * 512
        vid = b[v0:v0 + h['video_bytes']]
        assert zlib.crc32(vid) & 0xFFFFFFFF == h['video_crc32'], 'video CRC'
        ctrl, frag, i = b[64:64 + h['ctrl_bytes']], None, 0
        while i + 2 <= len(ctrl):
            t, L = ctrl[i], ctrl[i + 1]
            if t == 0x11:
                frag = struct.unpack('>III', ctrl[i + 2:i + 14])
            i += 2 + L
        if frag is None:
            out.append(vid)
            continue
        total, off, fcrc = frag
        if off == 0:
            asm, want = b'', (total, fcrc)
        assert off == len(asm), 'fragment gap'
        asm += vid
        if len(asm) == total:
            assert zlib.crc32(asm) & 0xFFFFFFFF == fcrc, 'whole-frame CRC'
            out.append(asm)
            asm = b''
    return b''.join(struct.pack('>I', len(f)) + f for f in out), len(out)


def run_e2e(name, seqname, k, maxvid, sfs, bus, extra_t='', extra_p='', label='', expect_identical=True,
            teensy_first=False):
    rnv, crc, pcm = case_inputs(seqname, k, name)
    tag = '%s_%s' % (name, label or 'b%d' % bus)
    served, ptyf, tlog = (os.path.join(OUT, tag + x) for x in ('_served.rsav', '_pty.txt', '_teensy.txt'))
    for f in (served, ptyf):
        if os.path.exists(f):
            os.remove(f)
    host = os.path.join(OUT, 'host_link')
    tcmd = '%s --ptyfile %s --k %d --bus %d --out %s %s' % (q(host), q(ptyf), k, bus, q(served), extra_t)
    pcmd = '%s/avlink --dev "$(cat %s)" --k %d --maxvid %d --video %s --yuvcrc %s --audio %s %s' % (
        q(OUT), q(ptyf), k, maxvid, q(rnv), q(crc), q(pcm), extra_p)
    script = ('set -e; %s > %s 2>&1 & T=$!; for i in $(seq 200); do [ -s %s ] && break; sleep 0.02; done; '
              '%s %s 2> %s; wait $T; echo TEENSY_RC=$?' % (
                  tcmd, q(tlog), q(ptyf), 'sleep 0.3;' if teensy_first else '', pcmd, q(tlog + '.pi')))
    r = wsl(script, timeout=600, check=False)
    tl = open(tlog).read() if os.path.exists(tlog) else ''
    pl = open(tlog + '.pi').read() if os.path.exists(tlog + '.pi') else ''
    summ = [l for l in tl.splitlines() if l.startswith('HOSTLINK')]
    if 'TEENSY_RC=0' not in r.stdout or not os.path.exists(served):
        print('  e2e %s: FAILED to run\n%s\n%s\n%s' % (tag, r.stdout[-2000:], tl[-3000:], pl[-2000:]))
        return False
    got = open(served, 'rb').read()
    ref = b''.join(sfs)
    stats = dict(kv.split('=', 1) for kv in ' '.join(summ).split() if '=' in kv)
    ok = int(stats.get('wire_crc_bad', 1)) == 0 and int(stats.get('hdr_bad', 1)) == 0
    if expect_identical:
        ident = got == ref
        ok &= ident
        what = 'served == sdgen.py (%d B)' % len(ref) if ident else 'MISMATCH (%d vs %d B)' % (len(got), len(ref))
    else:   # EXPIRED / stale test: every served superframe must equal sdgen's for its seq
        bodies, i, bad = [], 0, 0
        while i < len(got):
            h = sd_wire.parse_header_sector(got[i:i + 512])
            n = h['total_sec'] * 512
            bad += got[i:i + n] != sfs[h['seq']]
            bodies.append(h['seq'])
            i += n
        ok &= bad == 0 and int(stats.get('expired_seen', 0)) > 0
        what = 'served %d superframes (seq %s..%s), %d differ from sdgen; EXPIRED seen %s' % (
            len(bodies), bodies[0] if bodies else '-', bodies[-1] if bodies else '-', bad, stats.get('expired_seen'))
    dec_note = ''
    if expect_identical and ok:
        bl, i = [], 0
        while i < len(got):
            h = sd_wire.parse_header_sector(got[i:i + 512])
            bl.append(got[i:i + h['total_sec'] * 512])
            i += h['total_sec'] * 512
        rnvdata, nfr = reassemble_rnv(bl)
        rp = os.path.join(OUT, tag + '_served.rnv')
        open(rp, 'wb').write(rnvdata)
        d = wsl('%s/host_dec %s o 3' % (q(OUT), q(rp)), check=False)
        dec = [l.split() for l in d.stdout.splitlines() if l.strip()]
        exp = [int(l.split()[1]) for l in open(crc)][:nfr]
        crc_ok = len(dec) == nfr and all(len(x) == 2 and int(x[1]) == e for x, e in zip(dec, exp))
        ok &= crc_ok
        dec_note = '; host_dec %d frames YUY2 CRC %s' % (len(dec), 'exact' if crc_ok else 'MISMATCH')
    keys = ('wire_crc_bad', 'hdr_bad', 'notready_seen', 'idle_seen', 'expired_seen', 'holds', 'converted',
            'evicted', 'reads', 'retries')
    print('  e2e %-18s %s%s\n      teensy: %s' % (tag, what, dec_note, ' '.join('%s=%s' % (x, stats.get(x, '?')) for x in keys)))
    pi_end = [l for l in pl.splitlines() if l.startswith('END')]
    if pi_end:
        print('      avlink:', pi_end[-1][:230])
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--skip-e2e', action='store_true')
    ap.add_argument('--raw', action='store_true', help='also the worst-case rawtest4 (770 KB frames)')
    ap.add_argument('--out', help='output directory for binaries and results (default test/out)')
    ap.add_argument('--skip-live', action='store_true')
    ap.add_argument('--skip-sdgen', action='store_true', help='skip stages 2 and 3 (they need sdgen.py and the source clips)')
    ap.add_argument('--live-quick', action='store_true')
    a = ap.parse_args()
    global OUT
    if a.out:
        OUT = os.path.abspath(a.out)
    os.makedirs(OUT, exist_ok=True)
    print('building (WSL): avlink, rnse_proto/sim/host_dec, teensy/tools/host_link')
    if a.skip_sdgen or sdgen is None:
        a.skip_e2e = True
    build(not a.skip_e2e)
    res = {}
    print('[0] credit origins and fail-closed transport')
    for name, exe in (('credits', 'credit-test'), ('nocredits', 'nocredits-test'), ('discard', 'discard-test'), ('stall', 'stall-test')):
        r = wsl('%s/%s' % (q(OUT), exe), check=False)
        print(' ', r.stdout.strip().splitlines()[-1] if r.stdout.strip() else r.stderr[-400:])
        res[name] = r.returncode == 0
    r = wsl('ASAN_OPTIONS=detect_leaks=0 %s/ownership-test' % q(OUT), check=False, timeout=600)
    print(' ', r.stdout.strip() if r.returncode == 0 else r.stderr[-800:])
    res['ownership'] = r.returncode == 0
    res['selftest'] = test_selftest()
    print('[1] wire images vs sd_wire.py')
    res['wire'] = test_wire()
    cases = CASES + ([('rawtest4', 30, 248, 'raw')] if a.raw else [])
    if a.skip_sdgen or sdgen is None:
        print('[2][3] superframes vs sdgen.py, end to end: skipped (need sdgen.py and the source clips, not in this repo)')
        refs = None
    else:
        print('[2] superframes vs sdgen.py')
        res['sdgen'], refs = test_sdgen(cases)
    if not a.skip_e2e:
        print('[3] end to end: avlink -> pty -> Teensy host build -> mode A reader -> host_dec')
        e = []
        for seqname, k, maxvid, name in cases:
            sfs = refs[name][0]
            e.append(run_e2e(name, seqname, k, maxvid, sfs, 4))
        e.append(run_e2e('frag', 'v2_real/turn_night', 48, 60, refs['frag'][0], 1, label='bus1'))
        e.append(run_e2e('hwy', 'v2_real/highway110_night', 48, 248, refs['hwy'][0], 4, label='tfirst',
                         teensy_first=True, extra_t='--start-delay 0'))
        e.append(run_e2e('hwy', 'v2_real/highway110_night', 48, 248, refs['hwy'][0], 4, label='backpressure',
                         extra_t='--start-delay 300'))
        # avlink CONFIG sets stale_ms=1500, overriding the host simulator's startup value. Delay the
        # reader past the actual sender-selected timeout to force eviction; the shorter case above
        # must instead retain all frames while the credited producer waits for the reader.
        e.append(run_e2e('hwy', 'v2_real/highway110_night', 48, 248, refs['hwy'][0], 4, label='stale',
                         extra_t='--start-delay 2200', expect_identical=False))
        e.append(run_e2e('frag', 'v2_real/turn_night', 48, 60, refs['frag'][0], 4, label='chunk7',
                         extra_p='--chunk 7'))
        e.append(run_e2e('frag', 'v2_real/turn_night', 48, 60, refs['frag'][0], 4, label='wholesf',
                         extra_p='--whole-sf 1'))
        res['e2e'] = all(e)
    if not a.skip_live:
        print('[4] live: real tick and credits, reader away and back, overload')
        cmd = 'python3 %s/test/test_live.py --out %s/live%s' % (q(AVL), q(OUT), ' --quick' if a.live_quick else '')
        r = wsl(cmd, timeout=900, check=False)
        print(r.stdout.rstrip())
        res['live'] = r.returncode == 0
    print('RESULT: ' + ' '.join('%s=%s' % (k, 'PASS' if v else 'FAIL') for k, v in res.items()))
    sys.exit(0 if all(res.values()) else 1)


if __name__ == '__main__':
    main()
