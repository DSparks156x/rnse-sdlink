r"""sdbench host script: timed raw SD reads (firmware int 80653598(slot, lba, buf, n)) from a REAL card, read-only.

  sdbench.py run --slot N [--lba 0] [--sectors 4096] [--chunks 8,64,512,4096] [--passes 1] [--gate high|low|none]
  sdbench.py status

--gate: card-detect gate on GPIO FF93E00C (slot0 bit 0x400, slot1 bit 0x800). 'high' (default) = run only if the
        slot bit is 1, 'low' = only if 0 (use whatever sdprobe showed with the card IN), 'none' = no gate (the driver
        then returns -1 by itself when there is no card; the program stops after the first failed call).
Run: python -I sdbench.py ...
"""
import sys, argparse
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # -I does not add the script dir
import layout as L

RW = 10
ERRS = {0: 'ok', 1: 'no CTL block', 2: 'bad parameters', 3: 'card-detect gate: no card in that slot (not called)',
        4: 'stopped by host', 5: 'a raw read returned -1 (see the failing pass)'}
HF = ['cd_raw', 'cd_bit', 'clk0', 'info1_0', 'info2_0', 'dmdl', 'nres', 'sum_last', 's0_0', 's0_1', 's0_2', 's0_3',
      's0_tail', 'sum_mismatch', 'clk1', 'opt1', 'info1_1', 'info2_1', 'busy0', 'busy1']


def make_ctl(slot=1, lba=0, total=4096, gate=1, chunks=(8, 64, 512, 4096), passes=1):
    ch = list(chunks)[:6]
    return [slot, lba, total, gate, len(ch)] + ch + [0] * (6 - len(ch)) + [passes]


def decode(st):
    h = L.header(st)
    w = L.words(st)
    d = dict(h)
    for i, k in enumerate(HF):
        d[k] = w[8 + i]
    d['passes'] = []
    for i in range(min(d['nres'], 24)):
        r = w[64 + RW * i:64 + RW * i + RW]
        if len(r) < RW:
            break
        p = dict(chunk=r[0] & 0xFFFF, rep=r[0] >> 16, sect_ok=r[1], calls=r[2], errs=r[3], counts=r[4], ms=r[5],
                 sum=r[6], untouched=r[7], first_err=r[8], clk=r[9] & 0xFFFF, info2=r[9] >> 16)
        sec = p['counts'] / L.TMU_HZ
        p['secs'] = sec
        p['kBps'] = (p['sect_ok'] * 512 / 1024.0 / sec) if sec > 0 else 0
        p['ms_per_call'] = sec * 1000 / p['calls'] if p['calls'] else 0
        d['passes'].append(p)
    return d


def report(d):
    print('sdbench: %s err %d = %s' % ('DONE' if d['done'] else 'NOT DONE (stage %X)' % d['stage'], d['err'],
                                      ERRS.get(d['err'], '?')))
    print('  CD GPIO %08X (slot bit = %d)  CLK_CTRL before %04X after %04X  OPTION after %04X  INFO1 %04X->%04X  '
          'INFO2 %04X->%04X' % (d['cd_raw'], d['cd_bit'], d['clk0'], d['clk1'], d['opt1'], d['info1_0'], d['info1_1'],
                                d['info2_0'], d['info2_1']))
    print('  driver busy flags 8EAAC970/96C at start %08X/%08X  MP3 sink handle %08X' % (d['busy0'], d['busy1'], d['dmdl']))
    print('  %-6s %-3s %7s %6s %4s %9s %8s %9s %8s %5s %6s' % ('chunk', 'rep', 'sectors', 'calls', 'err', 'ms(TMU)',
                                                           'KB/s', 'MB/s', 'ms/call', 'untch', 'CLK'))
    for p in d['passes']:
        print('  %-6d %-3d %7d %6d %4d %9.2f %8.0f %9.3f %8.3f %5d   %04X' % (
            p['chunk'], p['rep'], p['sect_ok'], p['calls'], p['errs'], p['secs'] * 1000, p['kBps'],
            p['kBps'] / 1024.0, p['ms_per_call'], p['untouched'], p['clk']))
        if p['errs']:
            print('         first failing call at sector offset %d (LBA +%d); INFO2 after = %04X' % (
                p['first_err'], p['first_err'], p['info2']))
    print('  data: sum of last full pass %08X, passes with a different sum: %d; sector 0 starts %08X %08X %08X %08X, '
          'ends ..%08X (MBR/boot sector ends 55AA)' % (d['sum_last'], d['sum_mismatch'], d['s0_0'], d['s0_1'], d['s0_2'],
                                                       d['s0_3'], d['s0_tail']))
    if any(p['untouched'] for p in d['passes']):
        print('  WARNING: some sectors still hold the pre-fill marker after a "successful" read (DMA did not land?)')
    if d['passes']:
        best = max(d['passes'], key=lambda p: p['kBps'])
        print('  BEST: %.2f MB/s with %d-sector calls (%.2f ms per call)' % (best['kBps'] / 1024, best['chunk'],
                                                                           best['ms_per_call']))


def read_all(b):
    st = b.final(0x100)
    n = min(L.words(st)[8 + HF.index('nres')], 24)
    return st + (b.read(L.STAT_BASE + 0x100, RW * 4 * n) if n else b'')


def show(st):
    d = decode(st)
    return 'stage %X  passes done %d' % (d['stage'], d['nres'])


def main(argv):
    ap = argparse.ArgumentParser(description='sdbench bench script (read-only on the card)')
    ap.add_argument('cmd', choices=['run', 'status'])
    ap.add_argument('--slot', type=int, choices=(0, 1))
    ap.add_argument('--lba', type=int, default=0)
    ap.add_argument('--sectors', type=int, default=4096)
    ap.add_argument('--chunks', default='8,64,512,4096')
    ap.add_argument('--passes', type=int, default=1)
    ap.add_argument('--gate', choices=('high', 'low', 'none'), default='high')
    a = ap.parse_args(argv)
    import rnse_bench
    b = rnse_bench.open_bench()
    try:
        if a.cmd == 'status':
            report(decode(read_all(b)))
            return 0
        if a.slot is None:
            ap.error('--slot is required for run (0 or 1; see README)')
        chunks = [int(x) for x in a.chunks.split(',')]
        gate = {'none': 0, 'high': 1, 'low': 2}[a.gate]
        params = make_ctl(a.slot, a.lba, a.sectors, gate, chunks, a.passes)
        rnse_bench.log('PLAN sdbench slot %d lba %d sectors %d chunks %s passes %d gate %s; MMIO reads FF93E00C '
                       'FF90A01C/1E/24/28 (16-bit); calls 80653598' % (a.slot, a.lba, a.sectors, chunks, a.passes, a.gate))
        b.start('sdbench', params, tag='slot %d' % a.slot)
        worst = len(chunks) * a.passes * (a.sectors * 512 / 300e3 + a.sectors / min(chunks) * 0.01 + 1) + 20
        b.poll('sdbench', timeout=worst, every=1.0, show=show, nstat=0x200)
        report(decode(read_all(b)))
    finally:
        b.close()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]) or 0)
