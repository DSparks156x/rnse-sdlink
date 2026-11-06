r"""benchd.py sdload N [secs]: N SD loads of sdstream (verify mode, ends by itself after secs), with timing. RAM only."""
import sys, time
import layout as L, rnse_bench, sdfast, sdstream


def main(argv):
    n = int(argv[0]) if argv else 20
    secs = int(argv[1]) if len(argv) > 1 else 1
    b = rnse_bench.open_bench()
    times, bad = [], 0
    for i in range(n):
        params = sdstream.make_ctl(0, 0, sdstream.MODES['verify'], secs, 0)
        try:
            t = b.start_fast('sdstream', params, 'loadtest')
            times.append(t)
        except sdfast.FastFail as e:
            bad += 1
            print('load %d FAILED: %s' % (i, e))
            continue
        end = time.monotonic() + secs + 4
        while time.monotonic() < end:
            h = L.header(b.read(L.STAT_BASE, 32))
            if h['valid'] and h['prog'] == L.PROG_ID['sdstream'] and h['done']:
                break
            time.sleep(0.2)
        else:
            print('load %d: program never finished: %s' % (i, h))
            bad += 1
            break
        st, ent = b.task_state()
        if st != 0x10 or ent != L.DOOM_ENTRY:
            print('load %d: task state %04X entry %08X' % (i, st, ent))
            bad += 1
            break
    if times:
        s = sorted(times)
        print('%d loads ok, %d bad; seconds min %.2f median %.2f max %.2f' % (len(s), bad, s[0], s[len(s) // 2], s[-1]))
