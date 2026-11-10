#!/usr/bin/env python3
r"""test_handshake.py -- the RNS-E version handshake end to end, no hardware (docs/PROTOCOL.md 4.5):
real avlink (live tick) -> pty -> teensy/tools/host_link (the firmware's avstream.cpp + sdproto.cpp) whose unit model announces like
rnse/module/tvauto.c hs_gate. Runs inside WSL or Linux:

  wsl.exe -e python3 pi/avlink/test/test_handshake.py [--out DIR]

Cases: same format (accept, stream read), unit older than the chain (reject, avlink stops sending, unit stays stock),
no announcement (old module / old Teensy: avlink assumes today's format and says so once the unit reads).
"""
import argparse, os, shutil, subprocess, sys, tempfile, time

HERE = os.path.dirname(os.path.abspath(__file__))
AVL = os.path.normpath(os.path.join(HERE, '..'))
ROOT = os.path.normpath(os.path.join(AVL, '..', '..'))
SD = os.path.join(ROOT, 'teensy', 'sdemu')
HL = os.path.join(ROOT, 'teensy', 'tools', 'host_link', 'main.cpp')


def build(out):
    av, hl = os.path.join(out, 'avlink_hs'), os.path.join(out, 'host_link_hs')
    subprocess.run(['gcc', '-O2', '-std=c11', '-Wall', '-Wextra', '-Wno-unused-parameter', '-o', av] +
                   [os.path.join(AVL, f) for f in ('avlink.c', 'sf.c', 'wire.c', 'crc.c', 'audio.c', 'link.c')], check=True)
    subprocess.run(['g++', '-O2', '-std=c++17', '-Wall', '-Wextra', '-Wno-unused-parameter', '-DAVS_HOST', '-o', hl, HL,
                    os.path.join(SD, 'avstream.cpp'), os.path.join(SD, 'sdproto.cpp')], check=True)
    return av, hl


def run(av, hl, hl_args, avl_args, out, name):
    w = tempfile.mkdtemp(prefix='avl_hs_')
    pty, vid, rf = (os.path.join(w, x) for x in ('pty', 'vid', 'rnse.txt'))
    os.mkfifo(vid)
    keep = os.open(vid, os.O_RDWR | os.O_NONBLOCK)         # a video source that stays open and sends nothing: REPEAT superframes
    r = subprocess.Popen([hl, '--ptyfile', pty, '--out', os.path.join(w, 'served.rsav'), '--start-delay', '500', '--timeout', '20'] + hl_args,
                         stdout=subprocess.PIPE, text=True)
    for _ in range(200):
        if os.path.exists(pty) and os.path.getsize(pty):
            break
        time.sleep(0.01)
    dev = open(pty).read().strip()
    al = subprocess.Popen([av, '--video', vid, '--no-audio', '--dev', dev, '--rnse-file', rf, '--stats', '0'] + avl_args,
                          stderr=subprocess.PIPE, text=True)
    hout = r.communicate(timeout=60)[0]
    al.terminate()
    err = al.communicate(timeout=10)[1]
    os.close(keep)
    rnse = open(rf).read() if os.path.exists(rf) else ''
    shutil.rmtree(w, ignore_errors=True)
    with open(os.path.join(out, name + '.log'), 'w') as f:
        f.write(hout + '\n--- avlink\n' + err + '\n--- rnse file\n' + rnse)
    return r.returncode, hout, err, dict(l.split('=', 1) for l in rnse.splitlines() if '=' in l and not l.startswith('#'))


# the host_link reader is a batch reader (stops at seq K-1); live, it reads until its timeout (rc 3) and we count what it served
LIVE = ['--k', '1000000000', '--timeout', '3']


def served(h):
    return next((int(x.split('=')[1]) for l in h.splitlines() if l.startswith('HOSTLINK') for x in l.split() if x.startswith('served_sf=')), 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(HERE, 'out', 'handshake'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    av, hl = build(a.out)
    fails = 0

    def check(c, m):
        nonlocal fails
        print('  %s %s' % ('ok  ' if c else 'FAIL', m))
        fails += not c

    print('[hs] accept: unit fmt 1, chain fmt 1')
    rc, h, e, f = run(av, hl, LIVE + ['--announce', '1:1'], [], a.out, 'accept')
    check('HS accept fmt 1' in h, 'unit sees accept, chosen fmt 1 (%s)' % next((l for l in h.splitlines() if l.startswith('HS ')), '-'))
    check(served(h) >= 60, 'unit then reads the stream (%d superframes in 3 s)' % served(h))
    check('avlink: RNS-E module build F46C419D fmt 1 table 54B/1: streaming fmt 1' in e, 'avlink log line')
    check(f.get('RNSE_ANNOUNCED') == '1' and f.get('RNSE_BUILD') == 'F46C419D' and f.get('STREAM_FMT') == '1', 'rnse file: announced, STREAM_FMT=1')

    print('[hs] reject: unit fmt 1, chain fmt 2')
    rc, h, e, f = run(av, hl, ['--k', '30', '--announce', '1:1'], ['--fmts', '2:2'], a.out, 'reject')
    check('HS reject fmt 0' in h, 'unit sees reject and stays stock')
    nb = next((int(l.split('=')[1]) for l in h.splitlines() if l.startswith('HS begun_after_reject=')), -1)
    check(nb == 0, 'avlink sends no superframe after the reject (%d)' % nb)
    check('avlink: RNS-E module too old: build F46C419D fmt 1 table 54B/1; this chain sends fmt 2-2: not streaming' in e, 'avlink "too old" line')
    check(f.get('STREAM_FMT') == '0', 'rnse file: STREAM_FMT=0')

    print('[hs] no announcement (old module or old Teensy)')
    rc, h, e, f = run(av, hl, LIVE, [], a.out, 'none')
    check(served(h) >= 60, 'stream read as before (%d superframes in 3 s)' % served(h))
    check('no version announced' in e and 'assuming stream fmt 1' in e, 'avlink says no version announced, assumes fmt 1')
    check(f.get('RNSE_ANNOUNCED') == '0' and f.get('STREAM_FMT') == '1', 'rnse file: not announced, STREAM_FMT=1')

    print('%s: handshake end to end, %d failure(s)' % ('PASS' if not fails else 'FAIL', fails))
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
