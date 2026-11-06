#!/usr/bin/env python3
r"""test_live.py -- live-mode checks: the real avlink (real tick, credits, key FIFO, audio) over a pty to the real AvStream, read by the
reader model in teensy/tools/host_link/live_reader.cpp. Unlike the batch (--k) end-to-end runs this has encoder back-pressure, an audio stage under stall,
a reader that leaves and returns, and seq past 255. Runs inside WSL or Linux, real time, no hardware:

  wsl.exe -e python3 pi/avlink/test/test_live.py [--out DIR] [--quick] [--pi DIR --sd DIR] [--no-check] [--only NAME]

--pi/--sd build another source tree (A/B against an older one). Latencies: encode time or audio sample time -> superframe accepted by the reader.
Limits: host timing, a pty for USB, a time model for reads, no decoder or sink. See review_batch1_fixes.md.
"""
import argparse, json, os, shutil, subprocess, sys, tempfile, time

HERE = os.path.dirname(os.path.abspath(__file__))
AVL = os.path.normpath(os.path.join(HERE, '..'))
ROOT = os.path.normpath(os.path.join(AVL, '..', '..'))
TOOLS = os.path.join(ROOT, 'teensy', 'tools', 'host_link')


def build(pi, sd, out, tag):
    os.makedirs(out, exist_ok=True)
    av, rd = os.path.join(out, 'avlink_' + tag), os.path.join(out, 'live_reader_' + tag)
    src = [os.path.join(pi, f) for f in ('avlink.c', 'sf.c', 'wire.c', 'crc.c', 'audio.c', 'link.c')]
    subprocess.run(['gcc', '-O2', '-g', '-std=c11', '-w', '-o', av] + src, check=True)
    inc = os.path.join(out, 'inc_' + tag)                 # the reader's own includes resolve against this tree
    shutil.rmtree(inc, ignore_errors=True)
    for d in ('sdemu', os.path.join('tools', 'host_link')):
        os.makedirs(os.path.join(inc, d))
    for f in ('avstream.h', 'avstream.cpp', 'sdproto.h', 'sdproto.cpp', 'tonemp3.h', 'config.h'):
        shutil.copy(os.path.join(sd, f), os.path.join(inc, 'sdemu', f))
    shutil.copy(os.path.join(TOOLS, 'live_reader.cpp'), os.path.join(inc, 'tools', 'host_link', 'live_reader.cpp'))
    defs = []
    if 'AVS_POOL_OCRAM' not in open(os.path.join(sd, 'config.h')).read():
        defs = ['-DPOOL_A=880', '-DPOOL_B=192']           # older tree: its pool
    subprocess.run(['g++', '-O2', '-std=c++17', '-w', '-DAVS_HOST'] + defs + ['-o', rd, os.path.join(inc, 'tools/host_link/live_reader.cpp'),
                    os.path.join(inc, 'sdemu/avstream.cpp'), os.path.join(inc, 'sdemu/sdproto.cpp')], check=True)
    return av, rd


def run(av, rd, size, reader_args, maxvid=140, keysize=200000, fps=60, log=None, keyevery=0, avl_args=()):
    w = tempfile.mkdtemp(prefix='avl_live_')
    pty, pcm, key = (os.path.join(w, x) for x in ('pty', 'pcm', 'key'))
    os.mkfifo(pcm); os.mkfifo(key)
    r = subprocess.Popen([rd, '--ptyfile', pty] + reader_args, stdout=subprocess.PIPE, text=True)
    for _ in range(200):
        if os.path.exists(pty) and os.path.getsize(pty):
            break
        time.sleep(0.01)
    dev = open(pty).read().strip()
    keep = os.open(key, os.O_RDWR | os.O_NONBLOCK)         # as hud2.sh: the FIFO exists before the encoder and avlink
    gen = os.path.join(HERE, 'live_gen.py')
    au = subprocess.Popen([sys.executable, gen, 'audio', pcm])
    vg = subprocess.Popen([sys.executable, gen, 'video', str(size), str(fps), str(keysize), key] + ([str(keyevery)] if keyevery else []), stdout=subprocess.PIPE)
    al = subprocess.Popen([av, '--video', '-', '--audio', pcm, '--dev', dev, '--keyctl', key, '--maxvid', str(maxvid), '--stats', '0'] + list(avl_args),
                          stdin=vg.stdout, stderr=subprocess.PIPE, text=True)
    vg.stdout.close()
    out = r.communicate()[0]
    for p in (vg, au, al):
        p.terminate()
    err = al.communicate(timeout=10)[1]
    for p in (vg, au):
        p.wait(timeout=10)
    os.close(keep)
    shutil.rmtree(w, ignore_errors=True)
    if log:
        with open(log, 'w') as f:
            f.write(out + '\n' + err)
    j = {}
    for line in out.splitlines():
        if line.startswith('JSON '):
            j = json.loads(line[5:])
    end = [x for x in err.splitlines() if x.startswith('END')]
    # The reader exits first and closes the pty, so avlink can see the hangup before our SIGTERM: rc 1 with exactly that message is a clean end of test.
    j['avlink_rc'] = 0 if al.returncode == 1 and 'device hung up' in err and 'credit accounting' not in err else al.returncode
    if end:
        for kv in end[-1].split():
            if '=' in kv:
                k, v = kv.split('=', 1)
                if v.lstrip('-').isdigit():
                    j['al_' + k] = int(v)
    return j


# name, frame bytes, reader args, maxvid, checks
LONG = ['--absent-at', '3', '--absent-for', '10', '--total', '18', '--preset', '1000000']
SCEN = [
    ('away10_3k',  3000,  LONG, 140, 'settle'),
    ('away10_20k', 20000, LONG, 140, 'settle'),
    ('away10_60k', 60000, LONG, 140, 'settle'),
    ('away10_75k', 75000, LONG, 150, 'settle'),        # 155 sectors per superframe, the 4.75 MB/s target
    ('away03_3k',  3000,  ['--absent-at', '3', '--absent-for', '0.3', '--total', '9'], 140, 'settle'),    # below the stall limit, the old standing backlog
    ('away06_3k',  3000,  ['--absent-at', '3', '--absent-for', '0.6', '--total', '9'], 140, 'settle'),
    ('away1_20k',  20000, ['--absent-at', '3', '--absent-for', '1', '--total', '9'], 140, 'settle'),
    ('twice_3k',   3000,  ['--absent-at', '3', '--absent-for', '2', '--absent-at', '8', '--absent-for', '0.5', '--total', '13'], 140, 'settle'),
    ('late_start', 20000, ['--absent-at', '0', '--absent-for', '4', '--total', '9'], 140, 'settle'),      # chain up first, then the reader
    ('steady_75k', 75000, ['--total', '10'], 150, 'steady'),
    ('over_60k',   60000, ['--total', '15', '--us-per-sector', '160', '--settle', '5'], 140, 'overload'),  # reader about 50 superframes/s
    ('over_3k',    3000,  ['--total', '15', '--sleep-us', '20000', '--settle', '5'], 140, 'overload'),     # reader limited, small frames
    ('peak_170',   81000, ['--total', '12', '--settle', '5'], 162, 'overload'),                            # 167-170 sectors: 17.1 ms per read, over the tick
]
# extra scenarios: name, frame bytes, reader args, maxvid, checks, options. Whole-superframe hold on (--whole-sf 1), and what a real source looks like: below 60 fps, periodic keys.
WSF = ['--whole-sf', '1']
SCEN2 = [
    ('wsf_steady', 75000, ['--total', '10'], 150, 'steady', {'avl_args': WSF}),
    ('wsf_away10', 20000, LONG, 140, 'settle', {'avl_args': WSF}),
    ('wsf_over',   60000, ['--total', '15', '--us-per-sector', '160', '--settle', '5'], 140, 'overload', {'avl_args': WSF}),
    ('steady_55',  75000, ['--total', '10', '--settle', '3'], 150, 'steady', {'fps': 55}),                 # the standing queue behind the start key drains
    ('away03_55',  3000,  ['--absent-at', '3', '--absent-for', '0.3', '--total', '9'], 140, 'settle', {'fps': 55}),
    ('keys4s_60',  60000, ['--total', '14', '--settle', '3'], 140, 'none', {'keyevery': 4}),               # periodic key, exactly 60 fps: compare the older tree
    ('keys4s_55',  60000, ['--total', '14', '--settle', '3'], 140, 'none', {'keyevery': 4, 'fps': 55}),
]
QUICK = {'away10_3k', 'away03_3k', 'over_60k', 'wsf_steady'}


def check(kind, j):
    bad = []
    if j.get('avlink_rc') not in (0, -15, 143):
        bad.append('avlink rc %s' % j.get('avlink_rc'))
    if j.get('chain_breaks', 1):
        bad.append('delta decoded on a wrong reference: %s' % j.get('chain_breaks'))
    if j.get('rx_errors', 1):
        bad.append('rx_errors %s' % j.get('rx_errors'))
    if kind == 'settle':
        if not j.get('nv'):
            bad.append('no video after the settle time')
        if j.get('v_mean', 9e9) >= 150:
            bad.append('video mean %.0f ms, limit 150, a second after the return' % j.get('v_mean', 0))
        if j.get('v_p95', 9e9) >= 200:
            bad.append('video p95 %.0f ms, limit 200' % j.get('v_p95', 0))
        if abs(j.get('v_mean', 0) - j.get('a_mean', 0)) >= 100:
            bad.append('A/V skew %.0f ms, limit 100' % (j.get('v_mean', 0) - j.get('a_mean', 0)))
        if not 0 <= j.get('first_vid_ms', -1) < 1000:
            bad.append('first picture after return %.0f ms, limit 1000' % j.get('first_vid_ms', -1))
    elif kind == 'steady':
        if j.get('v_mean', 9e9) >= 100 or j.get('evicted', 1) or j.get('expired', 1):
            bad.append('steady: video %.0f ms evicted %s expired %s' % (j.get('v_mean', 0), j.get('evicted'), j.get('expired')))
        if j.get('al_discards', 0):
            bad.append('steady: %s discards' % j.get('al_discards'))
    elif kind == 'overload':
        if not j.get('nv'):
            bad.append('no video')
        if j.get('v_max', 9e9) >= 500:
            bad.append('overload: video max %.0f ms, limit 500' % j.get('v_max', 0))
        if j.get('a_max', 9e9) >= 500:
            bad.append('overload: audio max %.0f ms, limit 500' % j.get('a_max', 0))
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(HERE, 'out_live'))
    ap.add_argument('--pi', default=AVL)
    ap.add_argument('--sd', default=os.path.join(ROOT, 'teensy', 'sdemu'))
    ap.add_argument('--tag', default='new')
    ap.add_argument('--quick', action='store_true')
    ap.add_argument('--no-check', action='store_true', help='print only: for runs of an older tree')
    ap.add_argument('--only')
    a = ap.parse_args()
    av, rd = build(a.pi, a.sd, a.out, a.tag)
    fails = 0
    print('%-11s %7s %7s %7s %7s %7s %8s %8s %6s %6s %5s %5s %5s %5s %6s' % ('scenario', 'v_mean', 'v_p95', 'v_max', 'a_mean', 'a_max', '1st_sf', '1st_pic', 'keys', 'breaks', 'disc', 'evict', 'exp', 'seq', 'midhld'))
    res = {}
    for name, size, args, maxvid, kind, *opt in SCEN + SCEN2:
        if (a.quick and name not in QUICK) or (a.only and a.only not in name):
            continue
        opt = opt[0] if opt else {}
        if a.no_check and 'avl_args' in opt:
            continue                                   # an older tree has no such option
        j = run(av, rd, size, args, maxvid, log=os.path.join(a.out, '%s_%s.log' % (a.tag, name)), **opt)
        res[name] = j
        print('%-11s %7.0f %7.0f %7.0f %7.0f %7.0f %8.0f %8.0f %6s %6s %5s %5s %5s %5s %6s' % (name, j.get('v_mean', -1), j.get('v_p95', -1), j.get('v_max', -1), j.get('a_mean', -1), j.get('a_max', -1),
              j.get('first_sf_ms', -1), j.get('first_vid_ms', -1), j.get('keys', '?'), j.get('chain_breaks', '?'), j.get('al_discards', '-'), j.get('evicted', '?'), j.get('expired', '?'), j.get('seq_max', '?'), j.get('mid_holds', '?')), flush=True)
        if not a.no_check:
            for b in check(kind, j):
                print('   FAIL %s' % b, flush=True)
                fails += 1
    with open(os.path.join(a.out, a.tag + '_results.json'), 'w') as f:
        json.dump(res, f, indent=1)
    if not a.no_check:
        print('RESULT live: %s' % ('PASS' if not fails else 'FAIL %d' % fails))
    sys.exit(1 if fails and not a.no_check else 0)


if __name__ == '__main__':
    main()
