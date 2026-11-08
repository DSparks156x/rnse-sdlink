#!/usr/bin/env python3
r"""test_backoff.py -- audio under reader overload, old avlink against new, on the harness of test_live.py (real avlink, real AvStream, modelled reader).
  wsl.exe -e python3 pi/avlink/test/test_backoff.py OLD_TREE [--out DIR]
OLD_TREE: a directory with the older avlink sources. Trees without --linkcap run without it. Numbers: audio lost on the Pi, audio lateness at the reader
(the unit's FIFO runs down by the rise over the steady value), what the link did. Host timing and a time model for reads: no decoder, no sink.
"""
import argparse, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_live as T

SLOW = ['--us-per-sector', '119']    # 86 KB superframe in 20 ms: 4.3 MB/s, the ceiling in the car log of 2026-11-07
STALLS = []
for at in (6, 10, 14, 18):
    STALLS += ['--absent-at', str(at), '--absent-for', '0.2']
LONG = []
for at in (6, 11, 16, 21):
    LONG += ['--absent-at', str(at), '--absent-for', '0.35']
CASES = [
    ('stalls_350ms', 60000, ['--total', '28', '--settle', '5'] + LONG, 162, 4.5e6),
    ('overload_4.3', 80000, ['--total', '25', '--settle', '5'] + SLOW, 162, 4.8e6),
    ('stalls_200ms', 20000, ['--total', '25', '--settle', '5'] + STALLS, 162, 4.5e6),
    ('healthy_4.5',  75000, ['--total', '20', '--settle', '5'], 162, 4.5e6),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('old')
    ap.add_argument('--out', default='/tmp/backoff_out')
    ap.add_argument('--only')
    ap.add_argument('--sd', default=os.path.join(T.ROOT, 'teensy', 'sdemu'))
    a = ap.parse_args()
    print('%-13s %-4s %9s %8s %6s %7s %7s %7s %6s %7s %6s %6s %6s %5s' % ('case', 'tree', 'adrop_ms', 'sil_ms', 'late', 'a_mean', 'a_max', 'v_mean', 'keys', 'discard', 'back', 'catch', 'capkb', 'break'))
    for name, size, args, maxvid, cap in CASES:
        if a.only and a.only not in name:
            continue
        for tag, tree, extra in (('old', a.old, []), ('new', T.AVL, ['--linkcap', '%.0f' % cap])):
            av, rd = T.build(tree, a.sd, a.out, tag)
            j = T.run(av, rd, size, args, maxvid, log=os.path.join(a.out, '%s_%s.log' % (tag, name)), avl_args=extra)
            print('%-13s %-4s %9.0f %8.0f %6s %7.0f %7.0f %7.0f %6s %7s %6s %6s %6s %5s' % (name, tag, j.get('al_adrop', 0) / 48, j.get('al_silence', 0) / 48, j.get('al_late', '?'), j.get('a_mean', -1), j.get('a_max', -1),
                  j.get('v_mean', -1), j.get('keys', '?'), j.get('al_discards', '?'), j.get('al_backoffs', '-'), j.get('al_catchups', '-'), j.get('al_cap_kb', '-'), j.get('chain_breaks', '?')), flush=True)


if __name__ == '__main__':
    main()
