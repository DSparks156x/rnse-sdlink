r"""sdstream host script: the full SD -> decode -> display + DSP audio loop (RAM program out/sdstream.bin @8F760000).

  sdstream.py run --slot N [--gate high|low|none] [--mode verify|decode|audio|av] [--secs 20] [--wrap 240]
                  [--p1c 1] [--lrswap 0] [--block 960] [--target 2400] [--nbuf 3] [--pref] [--nopace]
                  [--info out/sdgen/highway110_night_240.json]
  sdstream.py status [--info ...]

--mode  verify : read + header CRC + payload CRC only (no decode, no display, no audio)      <- FIRST bench run
        decode : verify + RNV2 decode + per-frame YUY2 CRC vs the TLV in the header (no display, no audio)
        audio  : decode + PCM into the DSP sink (MP3 source selected by hand, sink free; as tone48)
        av     : audio + display on DU array 0 (YUY2 takeover, restore on exit)
--wrap  number of superframes on a card made by sdgen.py (slots 0..K-1, played in a loop; DISCONT on seq 0).
        0 = live stream (Teensy/Pi). Card images are paced at 60 Hz by the program (--nopace = as fast as possible).
--p1c   DSP_DEC param +1C (tone48 A2/A5 decides; 1 = no swap in the worker).
Run: python -I sdstream.py ...
"""
import os, sys, json, time, argparse
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # -I does not add the script dir
import layout as L

HERE = os.path.dirname(os.path.abspath(__file__))
F_DECODE, F_AUDIO, F_DISPLAY, F_TLVCRC, F_PAYCRC, F_PACE, F_PREF = 1, 2, 4, 8, 16, 32, 64
F_MEM = 128
F_STANDBY = 512
MODES = {'verify': F_PAYCRC, 'decode': F_PAYCRC | F_DECODE | F_TLVCRC,
         'audio': F_PAYCRC | F_DECODE | F_AUDIO, 'av': F_PAYCRC | F_DECODE | F_AUDIO | F_DISPLAY}
ERRS = {0: 'ok', 1: 'no CTL block', 2: 'bad parameters', 3: 'card-detect gate: no card', 4: 'PCM sink busy (MP3 player '
        'holds it)', 5: 'Snd_opn failed', 6: 'no initial credits within 300 ms', 7: '5 raw reads in a row failed',
        8: 'stopped by host', 9: 'Snd_dyD returned -1', 10: 'source changed by the user (clean exit)'}
F = ['seq', 'sf_ok', 'hdr_bad', 'retry', 'expired', 'seq_mism', 'rd_calls', 'rd_err', 'rd_sect', 'rd_cnt_lo',
     'rd_cnt_max', 'v_dec', 'v_dec_err', 'v_crc_bad', 'v_drop', 'v_repeat', 'v_late', 'dec_max', 'dec_lo', 'dec_hi',
     'v_shown', 'tlv_ok', 'tlv_bad', 'a_crc_bad', 'a_in', 'a_under', 'a_blocks', 'a_credits', 'a_fill', 'a_min',
     'a_max', 'a_over', 'a_handle', 'a_state', 'a_played', 'discont', 'eos', 'skipped', 'du', 'loop_max',
     'last_flags', 'last_fid', 'sink0', 'close', 'eos_ret', 'drain_ms', 'rd_cnt_hi', 'last_rd_ret', 'pace_waits',
     'first_err_seq', 'v_err_code', 'cd_raw', 'a_fmt_bad', 'wall_ms', 'sf_total', 'reuse_waits', 'a_ev_under',
     'a_ev_other', 'frag_in', 'frag_err', 'span_max', 'vframe_max', 'span1', 'span2', 'span3', 'span4', 'span5',
     'span6', 'span7', 'span8p', 'log_n', 'mem_cmd', 'mem_bad', 'mem_tx', 'mem_retry', 'mem_io',
     'iis_ret', 'iis_saved', 'iis_ms', 'iis_reissue', 'iis_end', 'ssi_cr', 'ssi_sr', 'e219a', 'e219b', 'p904a', 'p904b', 'stuck_n', 'stuck_ms', 'stuck_lba', 'stuck_cnt', 'sdr0', 'sdr1', 'sdr2', 'sdr3', 'sdr4', 'sdr5', 'sdr6', 'sdr7', 'fl_n', 'fl_wait', 'fl_f14', 'fl_f20', 'fl_f34', 'fl_f50', 'fl_max', 'fl_blk',
     'sess', 'r_min', 'r_zero', 'rate_mhz', 'ctx_chg', 'ctx_a', 'ctx_b', 'du_fix', 'a_resync', 'mute_a', 'keys', 'restarts']
LOGN = 1024
CPM = L.TMU_HZ / 1000.0
EV = ['hdr_bad', 'retry', 'vdec', 'verr', 'vcrc', 'vdrop', 'late', 'acrc', 'under', 'discont', 'tlv_bad', 'shown']


def make_ctl(slot=1, gate=1, flags=F_PAYCRC, secs=20, wrap=240, p1c=1, lrswap=4 << 8, block=480, target=3000, nbuf=3,
             hint=64, start=0, maxsf=0, mem_arm=0, mem_lo=0, mem_hi=0, totest=0):
    params = [slot, gate | (totest << 8), flags, secs, wrap, p1c, lrswap, block, target, nbuf, hint, start, maxsf]
    return params + [mem_arm, mem_lo, mem_hi] if flags & F_MEM else params


def decode(st, logb=b''):
    h = L.header(st)
    w = L.words(st)
    d = dict(h)
    for i, k in enumerate(F):
        d[k] = w[8 + i]
    lw = L.words(logb)
    d['log'] = [dict(seq=lw[8 * i], flags=lw[8 * i + 1], ev=lw[8 * i + 2], rd=lw[8 * i + 3], dec=lw[8 * i + 4],
                     fill=lw[8 * i + 5], total=lw[8 * i + 6], tick=lw[8 * i + 7]) for i in range(len(lw) // 8)]
    return d


def report(d, info=None):
    print('sdstream: %s err %d = %s' % ('DONE' if d['done'] else 'NOT DONE (stage %X)' % d['stage'], d['err'],
                                       ERRS.get(d['err'], '?')))
    n = max(d['sf_ok'], 1)
    rd_ms = (d['rd_cnt_lo'] + (d['rd_cnt_hi'] << 32)) / CPM
    print('  superframes ok %d (of %d processed), header bad %d, retries %d, seq mismatches %d, skipped %d, expired %d,'
          ' DISCONT %d, wall %.1f s (%.1f sf/s)' % (d['sf_ok'], d['sf_total'], d['hdr_bad'], d['retry'], d['seq_mism'],
                                                  d['skipped'], d['expired'], d['discont'], d['wall_ms'] / 1000.0,
                                                  d['sf_ok'] * 1000.0 / max(d['wall_ms'], 1)))
    print('  SD: %d calls, %d errors (last ret %d), %d sectors = %.2f MB, read time total %.0f ms, mean %.2f ms/sf, '
          'max call %.2f ms -> %.2f MB/s while reading' % (
              d['rd_calls'], d['rd_err'], L.s32(d['last_rd_ret']), d['rd_sect'], d['rd_sect'] * 512 / 1e6, rd_ms,
              rd_ms / n, d['rd_cnt_max'] / CPM, d['rd_sect'] * 512 / 1e6 / max(rd_ms / 1000.0, 1e-9)))
    dec_ms = (d['dec_lo'] + (d['dec_hi'] << 32)) / CPM
    print('  video: decoded %d, decode errors %d (last code %d at block %d), payload CRC bad %d, dropped (need key) %d,'
          ' repeats %d, shown %d, late %d; decode mean %.2f ms max %.2f ms; loop max %.2f ms' % (
              d['v_dec'], d['v_dec_err'], d['v_err_code'] & 0xFF, d['v_err_code'] >> 8, d['v_crc_bad'], d['v_drop'],
              d['v_repeat'], d['v_shown'], d['v_late'], dec_ms / max(d['v_dec'], 1), d['dec_max'] / CPM,
              d['loop_max'] / CPM))
    spans = [d['span%d' % i] for i in range(1, 8)] + [d['span8p']]
    if any(spans[1:]) or d['frag_err']:
        print('  video frames spanning N superframes: %s (max span %d, largest frame %d B); fragments in %d, '
              'fragment errors %d' % (' '.join('%d:%d' % (i + 1, c) for i, c in enumerate(spans) if c) +
                                      ('+' if spans[7] else ''), d['span_max'], d['vframe_max'], d['frag_in'],
                                      d['frag_err']))
    else:
        print('  video frames spanning N superframes: 1:%d (none split), largest frame %d B' % (spans[0], d['vframe_max']))
    if d['tlv_ok'] or d['tlv_bad']:
        print('  decoded-frame CRC vs header TLV (PC decoder): %d ok, %d BAD%s' % (
            d['tlv_ok'], d['tlv_bad'], ' -> bit-exact through SD' if not d['tlv_bad'] else ''))
    print('  audio: frames in %d, payload CRC bad %d, fmt bad %d, FIFO overflow %d; sink handle %08X state %d, blocks %d,'
          ' credits %d, underruns %d (sink events: underrun %d other %d)' % (
              d['a_in'], d['a_crc_bad'], d['a_fmt_bad'], d['a_over'], d['a_handle'], d['a_state'], d['a_blocks'],
              d['a_credits'], d['a_under'], d['a_ev_under'], d['a_ev_other']))
    if d['a_blocks']:
        print('  jitter FIFO fill: now %d, min %d, max %d frames (%.1f / %.1f / %.1f ms); played %d frames' % (
            d['a_fill'], d['a_min'] if d['a_min'] != 0xFFFFFFFF else 0, d['a_max'], d['a_fill'] / 48.0,
            (d['a_min'] if d['a_min'] != 0xFFFFFFFF else 0) / 48.0, d['a_max'] / 48.0, d['a_played']))
        print('  close: EOS dyD %d, drain %d ms, Snd_stp %d, Snd_cls %d' % (
            L.s32(d['eos_ret']), d['drain_ms'], L.s32(d['close'] >> 16) if d['close'] >> 31 else d['close'] >> 16,
            L.s32(d['close'] & 0xFFFF) if d['close'] & 0x8000 else d['close'] & 0xFFFF))
    if d.get('mem_tx') or d.get('mem_cmd') or d.get('mem_bad'):
        print('  memory: %d commands, %d malformed/busy/stale, %d sectors sent, %d retries, %d I/O errors' %
              tuple(d[k] for k in ('mem_cmd', 'mem_bad', 'mem_tx', 'mem_retry', 'mem_io')))
    if d['iis_saved'] or d['iis_ret']:
        print('  DSP input: was %02X, 805F3A94(3) returned %d, status reached 04 after %d ms, re-issued %d, restore %d ms (status %02X)' % (
            d['iis_saved'], L.s32(d['iis_ret']), d['iis_ms'], d['iis_reissue'], d['iis_end'] & 0xFFFFFF, d['iis_end'] >> 24))
        print('  SSI CR %08X SR %08X; 8E9E0219 %02X -> %02X; FF909004 %08X -> %08X' % (d['ssi_cr'], d['ssi_sr'], d['e219a'], d['e219b'], d['p904a'], d['p904b']))
    if d.get('stuck_n'):
        print('  STUCK READS %d, last LBA %X n %d: CMD %04X INFO1 %04X INFO2 %04X M1 %04X M2 %04X CLK %04X SIZE %04X OPT %04X ERR1 %04X ERR2 %04X STOP %04X SEC %04X RSP %08X lock %08X' % (
            d['stuck_n'], d['stuck_lba'], d['stuck_cnt'], d['sdr0'] >> 16, d['sdr0'] & 0xFFFF, d['sdr1'] >> 16, d['sdr1'] & 0xFFFF, d['sdr2'] >> 16, d['sdr2'] & 0xFFFF,
            d['sdr3'] >> 16, d['sdr3'] & 0xFFFF, d['sdr4'] >> 16, d['sdr4'] & 0xFFFF, d['sdr5'] >> 16, d['sdr5'] & 0xFFFF, d['sdr6'], d['sdr7']))
    if d.get('fl_n'):
        print('  flips: %d intervals, paced %d; <14 ms %d, 20-34 ms %d, 34-50 ms %d, >50 ms %d; max %.1f ms' % (
            d['fl_n'], d['fl_wait'], d['fl_f14'], d['fl_f20'], d['fl_f34'], d['fl_f50'], d['fl_max'] / CPM))
    if d.get('a_blocks'):
        print('  audio rate match: step offset %+.0f ppm; sink rate %.2f Hz (TMU clock); sink ring min %d, ring empty %d; backlog cuts %d' % (
            L.s32(d['fl_blk']) * 15.26, d['rate_mhz'] / 1000.0, d['r_min'] if d['r_min'] != 0xFFFFFFFF else -1, d['r_zero'], d['a_resync']))
    print('  first frame shown %d ms after session start' % d['stuck_ms'])
    print('  sessions %d, source left %d, ctx %08X -> %08X, DU re-asserted %d, mute at route %04X, keys %d, restarts %d' % (d['sess'], d['ctx_chg'], d['ctx_a'], d['ctx_b'], d['du_fix'], d['mute_a'], d['keys'], d['restarts']))
    if d['du']:
        print('  display: state %X (1 = took array 0, |100 = restored), buffer reuse waits %d, pace waits %d' % (
            d['du'], d['reuse_waits'], d['pace_waits']))
    if d['log']:
        lg = sorted(d['log'], key=lambda e: e['tick'])[-12:]
        print('  last superframes:  seq  flags  events                   read ms  dec ms  fifo  sect')
        for e in lg:
            evs = ','.join(n for i, n in enumerate(EV) if e['ev'] >> i & 1)
            print('              %6d  %04X  %-24s %7.2f %7.2f %5d %5d' % (e['seq'], e['flags'] & 0xFFFF, evs,
                                                                         e['rd'] / CPM, e['dec'] / CPM, e['fill'],
                                                                         e['total']))
    ok = d['done'] and d['err'] == 0 and not (d['hdr_bad'] or d['rd_err'] or d['v_dec_err'] or d['v_crc_bad'] or d['frag_err'] or
                                               d['tlv_bad'] or d['a_crc_bad'] or d['a_under'] or d.get('mem_bad') or d.get('mem_io'))
    print('  VERDICT: %s' % ('clean' if ok else 'see counters above'))


def read_all(b, full_log=False):
    st = b.final(0x200)
    if not full_log:                            # the ring is 32 KB = ~80 s over CAN
        return st, b''
    n = min(L.words(st)[8 + F.index('log_n')], LOGN)
    return st, (b.read(L.RES_BASE, 32 * n) if n else b'')


def show(st):
    d = decode(st.ljust(0x200, bytes(1)))               # the poll reads only what this line needs: CAN reads stall the unit
    return 'stage %X seq %d ok %d dec %d err %d tlvbad %d fifo %d under %d' % (
        d['stage'], d['seq'], d['sf_ok'], d['v_dec'], d['v_dec_err'], d['tlv_bad'], d['a_fill'], d['a_under'])


def main(argv):
    ap = argparse.ArgumentParser(description='sdstream bench script')
    ap.add_argument('cmd', choices=['run', 'status'])
    ap.add_argument('--slot', type=int, choices=(0, 1))
    ap.add_argument('--gate', choices=('high', 'low', 'none'), default='high')
    ap.add_argument('--mode', choices=list(MODES), default='verify')
    ap.add_argument('--secs', type=int, default=20)
    ap.add_argument('--wrap', type=int, default=240)
    ap.add_argument('--p1c', type=int, choices=(0, 1), default=1)
    ap.add_argument('--lrswap', type=int, choices=(0, 1), default=0)
    ap.add_argument('--block', type=int, default=480)
    ap.add_argument('--target', type=int, default=3000)
    ap.add_argument('--nbuf', type=int, choices=(2, 3), default=3)
    ap.add_argument('--pref', action='store_true', help='bench: the old rnvo decoder instead of rnvf, for A/B')
    ap.add_argument('--nopace', action='store_true')
    ap.add_argument('--iis', action='store_true', help='switch the DSP input to IIS via 805F3A94(3) once the SSI runs; restore on exit')
    ap.add_argument('--poll', type=float, default=5.0, help='seconds between status reads over CAN (the reads stall the unit)')
    ap.add_argument('--totest', type=int, default=0, help='bench: every Nth read gets a 1 ms timeout, to test recovery from a hung read')
    ap.add_argument('--novcrc', action='store_true', help='skip the video payload CRC (SD CRC16 covers the wire; the decoder is bounds-checked)')
    ap.add_argument('--steal', action='store_true', help='stop + close the (paused) MP3 player handle, then open the sink')
    ap.add_argument('--scan', action='store_true', help='on a stall, scan all tasks for SD/FS code on the stack (~60 s)')
    ap.add_argument('--tlv', action='store_true', help='also check decoded CRCs in audio/av modes (+~5 ms per frame)')
    ap.add_argument('--info', default='')
    ap.add_argument('--ring', action='store_true', help='bench: print the DSP_DEC ring fill + route state at each poll')
    ap.add_argument('--sink', type=int, default=4, help='blocks allowed in the DSP_DEC ring (latency = this x block)')
    ap.add_argument('--sinktone', type=int, default=0, help='bench: 1 = steady 1 kHz at the sink instead of the stream audio')
    ap.add_argument('--scan2', dest='scan', type=int, default=0, help='bench: 1 = check a steady tone for jumps at FIFO input and in the FIFO at exit, words at 8F72FFD0')
    ap.add_argument('--herectx', type=int, default=0, help='bench: --standby waits for the source it started on, not TV')
    ap.add_argument('--tcbscan', type=float, default=0, help='bench: at T s list tasks in ACON code (slow)')
    ap.add_argument('--nohop', action='store_true', help='switch straight to IIS, no hop through the FM input')
    ap.add_argument('--standby', action='store_true', help='after the user leaves TV: idle, resume on TV re-entry')
    ap.add_argument('--press', action='append', default=[], metavar='T:CMD', help='bench: at T s, CMD = radio|media|tv (panel key via queue 6 / direct select)')
    ap.add_argument('--full-log', action='store_true', help='also read the 32 KB per-superframe log (~80 s)')
    ap.add_argument('--memory', action='store_true', help='enable channel-3 memory service (emulator only, --wrap 0)')
    ap.add_argument('--memory-write-window', metavar='START:END', help='arm RAM patches inside this explicit window (end exclusive)')
    a = ap.parse_args(argv)
    mem_lo = mem_hi = mem_arm = 0
    if a.memory_write_window:
        if not a.memory:
            ap.error('--memory-write-window requires --memory')
        try:
            mem_lo, mem_hi = (int(x, 0) for x in a.memory_write_window.split(':'))
        except ValueError:
            ap.error('write window is START:END, e.g. 0x8F9E0000:0x8FA00000')
        if not (mem_lo < mem_hi and mem_lo >> 26 == (mem_hi - 1) >> 26 and mem_lo >> 26 in (0x23, 0x2B)):
            ap.error('write window must lie in one RAM alias (8C..90 or AC..B0)')
        mem_arm = 0x5A
    if a.memory and a.wrap:
        ap.error('--memory requires --wrap 0 (live emulator; writes reply sectors)')
    info = json.load(open(a.info)) if a.info else None
    import rnse_bench
    b = rnse_bench.open_bench()
    try:
        if a.cmd == 'status':
            report(decode(*read_all(b, a.full_log)), info)
            return 0
        if a.slot is None:
            ap.error('--slot is required (sdprobe S2 tells which)')
        fl = MODES[a.mode] | (0 if a.nopace or not a.wrap else F_PACE) | (F_PREF if a.pref else 0) | \
            (F_TLVCRC if a.tlv else 0) | (F_MEM if a.memory else 0) | (256 if a.novcrc else 0) | (F_STANDBY if a.standby else 0)
        params = make_ctl(a.slot, {'none': 0, 'high': 1, 'low': 2}[a.gate] | (16 if a.steal else 0) | (32 if a.iis else 0) | (64 if a.nohop else 0), fl, a.secs, a.wrap, a.p1c, a.lrswap | (a.sink << 8) | (a.sinktone << 16) | (a.scan << 17) | (a.herectx << 18),
                          a.block, a.target, a.nbuf, mem_arm=mem_arm, mem_lo=mem_lo, mem_hi=mem_hi, totest=a.totest)
        rnse_bench.log('PLAN sdstream mode %s flags %X slot %d wrap %d secs %d p1c %d; calls 80653598%s%s' % (
            a.mode, fl, a.slot, a.wrap, a.secs, a.p1c, ', Snd_*' if fl & F_AUDIO else '',
            ', DU array-0 takeover' if fl & F_DISPLAY else ''))
        if fl & F_AUDIO:
            h = b.u32(L.DMDL_HANDLE)
            busy = h >> 24 == 2                 # handles are 02xxxxxx; other values = uninitialised since power-up
            print('PCM sink handle @%08X = %08X -> %s' % (L.DMDL_HANDLE, h, 'BUSY' if busy else 'FREE'))
            if busy and not a.steal:
                print('refusing: stop the MP3 track first (tone48.py sinkcheck explains)')
                return 1
        if a.memory:
            # Raw-write entry point is not used by the ordinary read-only bench programs.
            # Include slot/LBA/buffer/count assignment and guards, beyond the shared read/write prologue.
            expected = bytes.fromhex('2f862f962fa62fb62fc62fd62fe64f224f127fcc6a4d6d73e2ff60a3e90088016c636b536e431f268d041f932aa88901a0a500094b158901a0a100092cc88b01a09d00094d158901')
            if b.read(0x80653776, len(expected)) != expected:
                raise RuntimeError('sd_raw_write entry bytes differ - refusing memory service')
            rnse_bench.log('PLAN memory service: SD reply writes 80653776, RAM arm %02X window %08X:%08X' %
                           (mem_arm, mem_lo, mem_hi))
        if a.iis:                               # bytes per AUDIO_ROUTE_RE T1; refuse on any other firmware
            if b.read(0x805F3A94, 8).hex() != '2fc62fd62fe64f22' or b.read(0x8047C8E4, 8).hex() != '0102030405060809':
                raise RuntimeError('805F3A94 or table 8047C8E4 differ - refusing --iis')
            print('DSP input now: cache %s status %s mute %s' % (b.read(0x8E9F08C4, 1).hex(), b.read(0x8E9F08CC, 1).hex(), b.read(0x8E9E0216, 2).hex()))
        KEYS = {'radio': 0xAC000180, 'media': 0xAC000140, 'tv': 0xAC000200, 'pradio': 0xAC000380, 'pmedia': 0xAC000340, 'ctx1': 0xAC000401, 'ctx7': 0xAC000407, 'mr': 0xAC000500}
        presses = sorted((float(x.split(':')[0]), KEYS[x.split(':')[1]]) for x in a.press)
        b.start('sdstream', params, tag=a.mode)
        t_run = time.monotonic()
        stall = {'seq': None, 'hb': None, 't': time.monotonic(), 'n': 0}

        def show2(st):                         # stall detector: dump task 159's TCB when the heartbeat stops
            d = decode(st.ljust(0x200, bytes(1)))
            if d['hb'] != stall['hb']:
                stall.update(hb=d['hb'], t=time.monotonic())
            elif time.monotonic() - stall['t'] > 2 + a.poll and stall['n'] < 3:
                stall['n'] += 1
                d = decode(b.read(L.STAT_BASE, 0x200))
                print('  STALL %.1f s: stage %X seq %d hb %X last_rd %d rd_calls %d TCB159 %s' % (
                    time.monotonic() - stall['t'], d['stage'], d['seq'], d['hb'], L.s32(d['last_rd_ret']), d['rd_calls'],
                    b.read(L.TCB159, 0x40).hex()))
                if a.scan and stall['n'] == 1:      # one full scan: who is inside the SD/FS code?
                    base = L.TCB159 - 0x9F * 0x40
                    for tid in range(1, 0xA0):
                        tc = b.read(base + tid * 0x40, 0x30)
                        stw = int.from_bytes(tc[4:6], 'big')
                        tsp = int.from_bytes(tc[0x2C:0x30], 'big')
                        if stw == 0x10 or not stw or not tsp:
                            continue
                        fr = b.read((tsp & 0x1FFFFFFF) | 0x80000000, 0xC0)
                        w = [fr[i:i + 4].hex() for i in range(0, len(fr), 4) if fr[i] == 0x80 and 0x40 <= fr[i + 1] <= 0x7F]
                        if any(x[:4] in ('8065', '8066', '8067', '8068', '8063') for x in w) or tid == 0x9F:
                            print('    id %02X prio %02X state %s wait %s sp %08X code %s' % (
                                tid, int.from_bytes(tc[2:4], 'big'), tc[4:6].hex(), tc[6:8].hex(), tsp, ' '.join(w[:10])))
            extra = ''
            if a.tcbscan and time.monotonic() - t_run >= a.tcbscan and not stall.get('scanned'):
                stall['scanned'] = 1
                base = L.TCB159 - 0x9F * 0x40
                for tid in range(1, 0xA0):
                    tc = b.read(base + tid * 0x40, 0x30)
                    stw = int.from_bytes(tc[4:6], 'big')
                    tsp = int.from_bytes(tc[0x2C:0x30], 'big')
                    if stw == 0x10 or not stw or not tsp:
                        continue
                    fr = b.read((tsp & 0x1FFFFFFF) | 0x80000000, 0x140) if tc[6:8] in (b'',) or int.from_bytes(tc[2:4], 'big') >= 0x20 else b''
                    w = [fr[i:i + 4].hex() for i in range(0, len(fr), 4) if fr[i] == 0x80 and 0x40 <= fr[i + 1] <= 0x7F]
                    print('    id %02X prio %02X state %s wait %s sp %08X code %s' % (tid, int.from_bytes(tc[2:4], 'big'), tc[4:6].hex(), tc[6:8].hex(), tsp, ' '.join(w[:14])))
            while presses and time.monotonic() - t_run >= presses[0][0]:
                b.poke(0x8F72FFF0, presses.pop(0)[1])
                extra = ' [press]'
            if a.ring:                         # DSP_DEC ring: elem size, n, read count, write count (8062E98A = fill); +in-flight DMAs; a_blocks vs PC time
                r = L.words(b.read(0x8EA19874, 16)) ; fl = b.u32(0x8EA18C98)
                now = time.monotonic()
                if 'rb' in stall:
                    extra += ' | ring el %d n %d fill %d dma %d blk/s %.2f (%.1f Hz)' % (r[0], r[1], r[3] - r[2], fl, (d['a_blocks'] - stall['rb']) / (now - stall['rt']), (d['a_blocks'] - stall['rb']) * a.block / (now - stall['rt']))
                else:
                    extra += ' | ring el %d n %d fill %d dma %d' % (r[0], r[1], r[3] - r[2], fl)
                extra += ' ctx %s cache %s status %s mute %s e219 %s kslot %s kreq %s' % (b.read(0x8E9DFF50, 4).hex(), b.read(0x8E9F08C4, 1).hex(), b.read(0x8E9F08CC, 1).hex(), b.read(0x8E9E0216, 2).hex(), b.read(0x8E9E0219, 1).hex(), b.read(0x8F72FFF4, 8).hex(), b.read(0x8F72FFF0, 4).hex())
                stall['rb'], stall['rt'] = d['a_blocks'], now
            return show(st) + ' rd %d err %d retry %d' % (d['rd_calls'], d['rd_err'], d['retry']) + extra

        b.poll('sdstream', timeout=a.secs + 30, every=a.poll, show=show2, nstat=0xB0)
        d = decode(*read_all(b, a.full_log))
        report(d, info)
        out = os.path.join(HERE, 'sdstream_%s.json' % time.strftime('%Y%m%d_%H%M%S'))
        json.dump({k: v for k, v in d.items()}, open(out, 'w'), indent=1)
        print('  saved %s' % out)
    finally:
        b.close()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]) or 0)
