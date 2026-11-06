r"""sdtime host script: where the 92 us/sector SD read time goes. See src/sdtime.c. RAM only; flash is not touched.

  benchd.py sdtime run --timing --mem --dump
      CPU timing of RAM and registers, RAM bandwidth, register dump. No SD traffic.
  benchd.py sdtime run --read [--lba 262400] [--sectors 512] [--chunk 64] [--clock 0] [--dcr A,B,..]
      Timed reads: flash driver first, then a RAM copy of the driver for each DMA control value.

--port and --extra read registers the firmware does not read (see src/sdtime.c): opt-in.
Use `cfg fastpat 1` on the Teensy for --read, so its per-sector work and data are the same for every pass.
An error stops the run. A bad DMA value can leave the SD host stuck: `benchd.py reboot` recovers it (RAM only).
"""
import sys, os, argparse
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L

L.PROG_BASE.setdefault('sdtime', 0x8F7C8000)        # after uiprobe's window; same exclusive area
L.PROG_ID.setdefault('sdtime', 7)

R_N = 16
ST_TIME, ST_MEM, ST_PASS, ST_DUMP = 16, 56, 64, 300
NSTAT = 0xA00
ST_TR = 520
ST_RATE = 430
RATE_NAMES = ['ch8', 'ch0', 'ch1', 'ch2', 'ch3', 'ch7', 'ch10', 'ch11', 'ch13', 'ch14', 'ch0 burst', 'ch1 burst', 'ch2 burst', 'ch3 burst', 'ch8+9 together', 'ch12+13 together']
RATE_N = 4096
# INFO2 meanings from Linux tmio_mmc.h (same offsets as this host; SH7773's own documentation is under NDA)
INFO2_BITS = ((0, 'CMD_IDX'), (1, 'CRC'), (2, 'STOPBIT'), (3, 'DATA_TIMEOUT'), (4, 'RX_OVERFLOW'), (5, 'TX_UNDERRUN'),
              (6, 'CMD_TIMEOUT'), (15, 'ILL_ACCESS'))
ERRS = {0: 'ok', 1: 'no CTL block', 2: 'bad parameters', 3: 'firmware differs from stock at a patched address: nothing run',
        4: 'stopped by host', 5: 'a read failed or the guard was hit (see the last pass)'}
TN = 1024
TIMED = ['RAM cached hit (loop cost)', 'RAM uncached R32', 'RAM uncached R16', 'RAM uncached W32', 'SD CLK_CTRL R16',
         'SD INFO2 R16', 'SD CC_EXT_MODE R16', 'DMA DSTSR R32', 'DMA DINTSR R32', 'TMU TCNT R32', 'SD data port R16', 'SSI CR R32', 'SSI CLKW R32', 'GPIO R32', 'DU DSSR R32',
         'FF903024 R32', 'FF900864 R32', 'FF80001C R32', 'SD CLK_CTRL R32', 'SD data port R32']
MEM = [('write-back 64 KiB dirty', 65536), ('cached read, misses', 65536), ('cached write', 65536),
       ('cached copy, word loop', 65536), ('uncached read', 8192), ('uncached write', 8192)]
STOCK = 0x00402112
# stock | PKMD 4-byte | PKMD at the channel's size | channel's size, no packing | same, with the destination fields the
# manual asks for when memory is the destination | burst (documented only for some channels)
DEFAULT_DCR = [0x00402112, 0x00502112, 0x00102112, 0x00002112, 0x00102100, 0x00602112]
DCR_BITS = ((26, 'DTAMD'), (25, 'DTAC'), (24, 'DTAU'), (23, 'DTAU1'), (22, 'SWMD'), (21, 'BTMD'), (20, 'PKMD'),
            (18, 'CT'), (17, 'ACMD'), (16, 'DIP'), (13, 'SMDL'), (12, 'SPDAM'), (5, 'DMDL'), (4, 'DPDAM'))


def dcr_name(v):
    n = [name for bit, name in DCR_BITS if v >> bit & 1]
    n.append('S%d' % (8 << (v >> 8 & 3)))
    n.append('D%d' % (8 << (v & 3)))
    return ' '.join(n)


def make_ctl(mode, slot=0, lba=262400, total=512, chunk=64, tmo=200, clock=0, dcr=(), once=False, flags=0):
    dcr = list(dcr)[:6]
    return [mode, slot, lba, total, chunk, tmo, clock & 0xFFFFFFFF, len(dcr) | (0x100 if once else 0) | flags] + dcr + [0] * (6 - len(dcr))


def decode(st):
    d = dict(L.header(st))
    w = L.words(st)
    d['detail'], d['ntime'], d['npass'], d['ndump'] = w[8:12]
    d['clk0'], d['info2_0'], d['info2_1'], d['clk1'] = w[12:16]
    d['time'] = [(w[ST_TIME + 2 * i], w[ST_TIME + 2 * i + 1]) for i in range(20)]
    d['mem'] = w[ST_MEM:ST_MEM + len(MEM)]
    d['passes'] = []
    for i in range(min(d['npass'], 14)):
        r = w[ST_PASS + R_N * i:ST_PASS + R_N * i + R_N]
        if len(r) < R_N:
            break
        p = dict(zip(('dcr', 'path', 'chunk', 'sect', 'calls', 'errs', 'counts', 'sum', 'w0', 'w1', 'untouched', 'guard',
                      'clk', 'drvinfo', 'errsts', 'live'), r))
        p['us_sector'] = p['counts'] / L.TMU_HZ * 1e6 / p['sect'] if p['sect'] else 0
        p['mb_s'] = p['sect'] * 512 / (p['counts'] / L.TMU_HZ) / 1e6 if p['counts'] else 0
        d['passes'].append(p)
    d['tr'] = w[ST_TR:ST_TR + 98] if len(w) >= ST_TR + 98 else []
    d['rate'] = [w[ST_RATE + 4 * i:ST_RATE + 4 * i + 4] for i in range(16)] if len(w) > ST_RATE + 64 else []
    d['rate_lock'] = w[ST_RATE - 1] if len(w) > ST_RATE else 0
    d['dump'] = [(w[ST_DUMP + 2 * i], w[ST_DUMP + 2 * i + 1]) for i in range(min(d['ndump'], 60))]
    return d


def report(d):
    print('sdtime: %s err %d = %s (detail %08X)' % ('DONE' if d['done'] else 'NOT DONE (stage %X)' % d['stage'], d['err'],
                                                 ERRS.get(d['err'], '?'), d['detail']))
    print('  word 8EAD1254 %08X -> %08X   INFO2 %04X -> %04X' % (d['clk0'], d['clk1'], d['info2_0'], d['info2_1']))
    if d['time']:
        print('  %-28s %9s %9s   (per access, loop cost removed)' % ('timing', 'best ns', 'worst ns'))
        base = d['time'][0][0] / L.TMU_HZ * 1e9 / TN
        for name, (lo, hi) in zip(TIMED, d['time']):
            if not lo:
                continue
            print('  %-28s %9.1f %9.1f' % (name, lo / L.TMU_HZ * 1e9 / TN - base, hi / L.TMU_HZ * 1e9 / TN - base))
    if any(d['mem']):
        print('  RAM seen by the CPU:')
        for (name, n), c in zip(MEM, d['mem']):
            if c:
                print('    %-26s %8.1f MB/s' % (name, n / (c / L.TMU_HZ) / 1e6))
    if d['tr'] and d['tr'][32]:
        tr = d['tr']; tick = 1e6 / L.TMU_HZ; total = sum(tr[:32]); big = tr[33] & 0xFFFFFF; ng = tr[33] >> 24
        print('  DMA trace, last call: %d transfers in %.1f us; pauses over 1 us: %d, total %.1f us (%.2f us per sector)' % (
            total, tr[32] * tick, ng, big * tick, big * tick / max(total / 256.0, 1)))
        print('  ticks per transfer (1 tick = 80 ns): ' + ' '.join('%d:%d' % (i, c) for i, c in enumerate(tr[:32]) if c))
        print('  first pauses, position in sector and length us: ' + ' '.join('%d:%.1f' % (tr[34 + 2 * i] % 256, tr[35 + 2 * i] * tick) for i in range(min(ng, 32))))
    if any(r[1] for r in d['rate']):
        print('  DMA alone, auto-request, %d transfers (lock ret %X):' % (RATE_N, d['rate_lock']))
        for name, r in zip(RATE_NAMES, d['rate']):
            print('    %-18s %8.1f ns each  (%d moved)  done %d  status/2nd moved %d  DCR read %08X' % (name, r[0] / L.TMU_HZ * 1e9 / max(r[2], 1), r[2], r[1] & 1, r[1] >> 8, r[3]))
    for a, v in d['dump']:
        print('  %08X = %08X' % (a, v))
    if d['passes']:
        print('  %-5s %-8s %-8s %-3s %5s %7s %8s %8s %7s %8s %8s %3s %3s %4s %3s %s' % (
            'path', 'DCR', 'live', 'rep', 'chunk', 'sectors', 'us/sect', 'MB/s', 'err', 'sum', 'word0', 'unt', 'grd', 'clk', 'div', 'bits'))
        base = d['passes'][0]['sum']
        for p in d['passes']:
            note = '' if p['sum'] == base and p['sect'] == d['passes'][0]['sect'] else '  DATA DIFFERS'
            print('  %-5s %08X %08X %-3d %5d %7d %8.2f %8.2f %7s %08X %08X %3d %3d %04X %3d %s%s' % (
                'copy' if p['path'] & 1 else 'flash', p['dcr'], p['live'], p['path'] >> 8, p['chunk'], p['sect'], p['us_sector'], p['mb_s'],
                '%X' % p['errs'], p['sum'], p['w0'], p['untouched'], p['guard'], p['clk'] & 0xFFFF, p['clk'] >> 24,
                dcr_name(p['dcr']), note))
            if p['drvinfo'] & 0x807F or p['errs']:
                bits = [n for b, n in INFO2_BITS if p['drvinfo'] >> b & 1]
                print('        driver error image %08X %s  ERR_STS %08X  after %d of %d sectors' % (
                    p['drvinfo'], ' '.join(bits) or '(no error bits)', p['errsts'], p['sect'], p['chunk']))
        print('  div = driver divider byte 8EAD1256 after the pass: if it rose, the driver hit an error and slowed down')


def read_all(b):
    return b.final(NSTAT)


def main(argv):
    ap = argparse.ArgumentParser(description='sdtime: where the SD read time goes')
    ap.add_argument('cmd', choices=['run', 'status'])
    for f in ('timing', 'mem', 'dump', 'port', 'extra', 'read', 'periph', 'wide', 'dma'):
        ap.add_argument('--' + f, action='store_true')
    ap.add_argument('--slot', type=int, default=0, choices=(0, 1))
    ap.add_argument('--lba', type=int, default=262400)
    ap.add_argument('--sectors', type=int, default=512)
    ap.add_argument('--chunk', type=int, default=64)
    ap.add_argument('--timeout', type=int, default=500, help='driver wait in ms (stock is 50000)')
    ap.add_argument('--clock', type=lambda s: int(s, 0), default=0, help='word 8EAD1254 before each pass: 0 = /2, 0x100 = /4, -1 = leave')
    ap.add_argument('--dcr', default=','.join('%08X' % v for v in DEFAULT_DCR))
    ap.add_argument('--w32', action='store_true', help='copy programs the DMA count in 32-bit units; use with a DCR with SPDS=32')
    ap.add_argument('--hostmode', action='store_true', help='also set SD +E4=0101 during the copy passes')
    ap.add_argument('--delayed', action='store_true', help='copy starts the DMA only after first data; for auto-request DCR values')
    ap.add_argument('--pio', action='store_true', help='with --delayed: CPU copy instead of DMA (measurement)')
    ap.add_argument('--trace', action='store_true', help='with --delayed: time each DMA count change during the read')
    ap.add_argument('--once', action='store_true', help='copy only, no retries: the first error stops it and its bits survive')
    a = ap.parse_args(argv)
    import rnse_bench
    b = rnse_bench.open_bench()
    try:
        if a.cmd == 'status':
            report(decode(read_all(b)))
            return 0
        mode = (a.timing and 1) | (a.mem and 2) | (a.dump and 4) | (a.port and 9) | (a.extra and 20) | (a.read and 32) | (a.periph and 65) | (a.wide and 129) | (a.dma and 256)
        if not mode:
            ap.error('choose at least one of --timing --mem --dump --port --extra --read --periph --wide --dma')
        dcr = [int(x, 16) for x in a.dcr.split(',') if x] if a.read else []
        if len(dcr) > 6:
            ap.error('at most 6 --dcr values')
        params = make_ctl(mode, a.slot, a.lba, a.sectors, a.chunk, a.timeout, a.clock, dcr, a.once, (0x200 if a.w32 else 0) | (0x400 if a.hostmode else 0) | (0x800 if a.delayed else 0) | (0x1000 if a.pio else 0) | (0x2000 if a.trace else 0))
        rnse_bench.log('PLAN sdtime mode %X slot %d lba %d sectors %d chunk %d timeout %d clock %X dcr %s' % (
            mode, a.slot, a.lba, a.sectors, a.chunk, a.timeout, a.clock & 0xFFFFFFFF, ' '.join('%08X' % v for v in dcr)))
        b.start('sdtime', params, tag='mode %X' % mode)
        worst = 30 + (len(dcr) + 1) * 2 * (a.sectors * 0.001 + a.timeout / 1000.0 + 1)
        b.poll('sdtime', timeout=worst, every=1.0, show=lambda st: 'stage %X passes %d' % (L.header(st)['stage'], decode(st.ljust(NSTAT, b'\0'))['npass']), nstat=NSTAT)
        report(decode(read_all(b)))
    finally:
        b.close()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]) or 0)
