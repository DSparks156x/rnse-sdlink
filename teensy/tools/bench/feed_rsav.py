"""feed_rsav.py -- Windows stand-in for pi/avlink: stream an sdgen .rsav to the Teensy live window over USB (TLNK).

    feed_rsav.py COM7 ..\\..\\..\\rnse_proto\\out\\sdgen\\highway110_night_240.rsav --secs 60 [--fps 60]

Superframes loop with advancing seq and pts (header patched and re-CRCed per superframe; payload images cached).
Credits follow link/SPEC.md 5.2. Prints one stats line per second. Teensy must already be in card mode.
"""
import argparse, ctypes, json, os, struct, sys, threading, time, zlib
import serial

if sys.platform == 'win32':                     # 1 ms sleeps; time.monotonic ticks at 15.6 ms, so use perf_counter
    ctypes.WinDLL('winmm').timeBeginPeriod(1)
time.monotonic = time.perf_counter

MAGIC = 0x4B4E4C54
T_SF_BEGIN, T_WIRE, T_SF_END, T_CONFIG, T_FLUSH, T_STATUS, T_SERVED = 1, 2, 3, 5, 7, 0x83, 0x85
SFF_KEY, SFF_REPEAT, SFF_DISCONT = 1, 2, 8
ST_FMT = '<HH17I4BI'
ST_NAMES = ('ver size free_sectors rx_sectors free_slots begun newest_seq served_seq sectors_served holds hold_timeouts '
            'notready expired idle evicted converted rx_errors crc7_rejects write_crc_errors card_state bus_width mode '
            'flags pool_sectors').split()

CRC_TAB = []
for i in range(256):
    d = i << 8
    for _ in range(8):
        d = ((d << 1) ^ 0x1021) & 0xFFFF if d & 0x8000 else (d << 1) & 0xFFFF
    CRC_TAB.append(d)
NIB = bytes(((b & 15) << 4) | (b >> 4) for b in range(256))
SPREAD = []
for b in range(256):
    SPREAD.append(tuple(((b >> (4 + k)) & 1) << 1 | ((b >> k) & 1) for k in range(4)))


def wire4(blk):
    """528-B 4-bit wire image of one sector (= pi/avlink/wire.c wire4_sector, = link/sd_wire.py wire4)."""
    c = [0, 0, 0, 0]
    for i in range(0, 512, 4):
        s0, s1, s2, s3 = SPREAD[blk[i]], SPREAD[blk[i + 1]], SPREAD[blk[i + 2]], SPREAD[blk[i + 3]]
        for k in range(4):
            v = s0[k] << 6 | s1[k] << 4 | s2[k] << 2 | s3[k]
            c[k] = ((c[k] << 8) & 0xFFFF) ^ CRC_TAB[((c[k] >> 8) ^ v) & 0xFF]
    a = b_ = 0
    for j in range(16):
        bit = 15 - j
        nib = sum(((c[k] >> bit) & 1) << k for k in range(4))
        if j < 8:
            a |= nib << (4 * j)
        else:
            b_ |= nib << (4 * (j - 8))
    return struct.pack('<I', 0x0FFFFFFF) + blk.translate(NIB) + struct.pack('<III', a, b_, 0xFFFFFFFF)


class Link:
    def __init__(self, port):
        self.s = serial.Serial()
        self.s.port, self.s.baudrate, self.s.timeout, self.s.write_timeout = port, 115200, 0.05, 5
        self.s.dtr = True; self.s.rts = True      # the firmware drops the link when DTR is low
        self.s.open()
        time.sleep(0.3)
        self.seq = 0
        self.st = None
        self.st_t = 0
        self.served = 0
        self.lat = None
        self.sent_sect = 0
        self.begins = 0
        self.stop = False
        self.console = []
        self.cv = threading.Condition()
        self.t = threading.Thread(target=self._rx, daemon=True)
        self.t.start()

    def pack(self, typ, pl):
        self.seq += 1
        return struct.pack('<IHHII', MAGIC, typ, 0, len(pl), self.seq) + pl

    def send(self, typ, pl):
        self.s.write(self.pack(typ, pl))

    def _rx(self):
        buf = bytearray()
        mg = struct.pack('<I', MAGIC)
        while not self.stop:
            buf += self.s.read(4096)
            while True:
                i = buf.find(mg)
                if i < 0:
                    if len(buf) > 3:
                        self.console.append(bytes(buf[:-3]))
                        del buf[:-3]
                    break
                if i:
                    self.console.append(bytes(buf[:i]))
                    del buf[:i]
                if len(buf) < 16:
                    break
                _, typ, _, ln, _ = struct.unpack('<IHHII', buf[:16])
                if ln > 4096:
                    del buf[:4]
                    continue
                if len(buf) < 16 + ln:
                    break
                pl = bytes(buf[16:16 + ln])
                del buf[:16 + ln]
                if typ == T_STATUS and ln >= struct.calcsize(ST_FMT):
                    with self.cv:
                        self.st = dict(zip(ST_NAMES, struct.unpack(ST_FMT, pl[:struct.calcsize(ST_FMT)])))
                        self.st_t = time.monotonic()
                        self.cv.notify_all()
                elif typ == T_SERVED:
                    self.served += 1

    def wait_credit(self, sect, timeout=2.0):
        end = time.monotonic() + timeout
        with self.cv:
            while True:
                st = self.st
                if st:
                    fs = st['free_sectors'] - ((self.sent_sect - st['rx_sectors']) & 0xFFFFFFFF)
                    fb = st['free_slots'] - ((self.begins - st['begun']) & 0xFFFFFFFF)
                    if fs >= sect and fb > 0:
                        return True
                left = end - time.monotonic()
                if left <= 0:
                    return False
                self.cv.wait(left)

    def close(self):
        self.stop = True
        self.t.join(1)
        self.s.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('port')
    ap.add_argument('rsav', nargs='+', help='one or more .rsav clips, played in a chain')
    ap.add_argument('--secs', type=float, default=60)
    ap.add_argument('--fps', type=float, default=60)
    ap.add_argument('--chunk', type=int, default=32)
    ap.add_argument('--quiet', action='store_true')
    ap.add_argument('--audio', default='', help='any audio file (ffmpeg decodes it): replaces the test tone; 800 frames per superframe')
    ap.add_argument('--audio-start', type=float, default=0, help='seconds into the file')
    ap.add_argument('--gain-db', type=float, default=0)
    ap.add_argument('--console', default='', help='append Teensy console text to this file')
    ap.add_argument('--cmd', action='append', default=[], help='console command to send first, e.g. "cfg log 0"')
    a = ap.parse_args()
    t0 = time.time()
    sfs = []
    for clip in a.rsav:
        data = open(clip, 'rb').read()
        idx = json.load(open(clip.rsplit('.', 1)[0] + '_index.json'))
        for e in idx:
            secs = [data[e['offset'] + 512 * i:e['offset'] + 512 * (i + 1)] for i in range(e['sectors'])]
            sfs.append((secs[0], b''.join(wire4(s) for s in secs[1:]), e['sectors']))
    total = sum(s[2] for s in sfs)
    print('cached %d superframes, %d sectors (%.2f MB/s at %g fps) in %.1f s' % (
        len(sfs), total, total * 512 / len(sfs) * a.fps / 1e6, a.fps, time.time() - t0), flush=True)

    music = None
    if a.audio:
        import hashlib, shutil, subprocess, tempfile
        ff = shutil.which('ffmpeg') or os.path.join(os.path.expandvars(r'%LOCALAPPDATA%'), 'Microsoft', 'WinGet', 'Packages',
                                                    'Gyan.FFmpeg_Microsoft.Winget.Source_8wekyb3d8bbwe', 'ffmpeg-8.1.1-full_build', 'bin', 'ffmpeg.exe')
        key = hashlib.sha256(('%s|%d|%g|%g|%g' % (os.path.abspath(a.audio), os.path.getmtime(a.audio), a.audio_start, a.secs, a.gain_db)).encode()).hexdigest()[:16]
        cache = os.path.join(tempfile.gettempdir(), 'feed_audio_%s.bin' % key)
        need = int((a.secs + 2) * 60)
        if not os.path.exists(cache):               # 7 sectors of 4-bit wire image + the raw block, per superframe
            pcm = subprocess.run([ff, '-v', 'error', '-ss', str(a.audio_start), '-i', a.audio, '-t', str(a.secs + 2), '-vn', '-ac', '2', '-ar', '48000',
                                  '-af', 'volume=%gdB' % a.gain_db, '-f', 's16be', '-'], stdout=subprocess.PIPE, check=True).stdout
            t1 = time.time()
            with open(cache + '.tmp', 'wb') as f:
                for i in range(len(pcm) // 3200):
                    blk = pcm[i * 3200:(i + 1) * 3200]
                    pad = blk + bytes(3584 - 3200)
                    f.write(struct.pack('>I', zlib.crc32(blk)) + b''.join(wire4(pad[j * 512:(j + 1) * 512]) for j in range(7)))
            os.replace(cache + '.tmp', cache)
            print('encoded %d superframes of audio in %.1f s' % (len(pcm) // 3200, time.time() - t1), flush=True)
        music = open(cache, 'rb').read()
        RECSZ = 4 + 7 * 528
        nmus = len(music) // RECSZ
        if not nmus:
            sys.exit('no audio decoded from %s' % a.audio)
        print('audio: %s, %.1f s' % (a.audio, nmus / 60), flush=True)

    L = Link(a.port)
    L.send(T_CONFIG, struct.pack('<HHHHB7x', 0, 0, 0, 0, 20))   # first TLNK message: turns on STATUS
    time.sleep(0.2)
    for c in a.cmd:
        L.s.write(c.encode() + bytes([10]))
    if not L.st:
        print('no STATUS from the Teensy yet', flush=True)
    else:                                                # credit totals are cumulative across links
        L.sent_sect, L.begins = L.st['rx_sectors'], L.st['begun']
    seq = 0
    skipped = 0
    nxt = t_start = time.monotonic()
    end = nxt + a.secs
    t_stat = nxt + 1
    sent0 = 0
    frames = len(sfs)
    wr_tot = wr_max = 0.0
    try:
        while time.monotonic() < end:
            now = time.monotonic()
            if now < nxt:
                time.sleep(min(nxt - now, 0.001))
                continue
            hdr, body, n = sfs[seq % frames]
            if not L.wait_credit(n, 0.5):
                skipped += 1
                nxt = time.monotonic() + 1 / a.fps
                continue
            h = bytearray(hdr)
            base = seq - seq % frames
            ap_, vp = struct.unpack_from('>II', h, 16)
            struct.pack_into('>I', h, 8, seq & 0xFFFFFFFF)
            struct.pack_into('>II', h, 16, (ap_ + base * 800) & 0xFFFFFFFF, (vp + base * 800) & 0xFFFFFFFF)
            nh = sfs[(seq + 1) % frames][2]
            struct.pack_into('>H', h, 42, nh)
            if music:                                   # audio occupies wire sectors 1..7 of every superframe
                rec = (seq % nmus) * RECSZ
                h[48:52] = music[rec:rec + 4]
                body = music[rec + 4:rec + RECSZ] + body[7 * 528:]
            struct.pack_into('>I', h, 60, zlib.crc32(bytes(h[:60]) + bytes(h[64:])))
            w = wire4(bytes(h)) + body
            flags = struct.unpack_from('>I', h, 12)[0]
            out = [L.pack(T_SF_BEGIN, struct.pack('<IHH', seq, n, flags & 0xFFFF))]
            L.begins += 1
            for i in range(0, n, a.chunk):
                c = min(a.chunk, n - i)
                out.append(L.pack(T_WIRE, struct.pack('<IHHI', seq, i, c, 4) + w[i * 528:(i + c) * 528]))
                L.sent_sect += c
            out.append(L.pack(T_SF_END, struct.pack('<I', seq)))
            tw = time.monotonic()
            L.s.write(b''.join(out))                    # one write per superframe (Windows per-write latency)
            tw = time.monotonic() - tw
            wr_tot += tw; wr_max = max(wr_max, tw)
            seq += 1
            nxt += 1 / a.fps
            if time.monotonic() - nxt > 0.25:           # fell far behind: don't burst
                nxt = time.monotonic()
            if time.monotonic() >= t_stat and not a.quiet:
                t_stat += 1
                s = L.st or {}
                print('sent seq %d (+%d) skipped %d | served_seq %s srv_sect %s holds %s timeouts %s notready %s expired %s idle %s '
                      'evicted %s rxerr %s free %s state %s bus %s' % (
                          seq, seq - sent0, skipped, s.get('served_seq'), s.get('sectors_served'), s.get('holds'),
                          s.get('hold_timeouts'), s.get('notready'), s.get('expired'), s.get('idle'), s.get('evicted'),
                          s.get('rx_errors'), s.get('free_sectors'), s.get('card_state'), s.get('bus_width')), flush=True)
                sent0 = seq
    finally:
        print('END sent %d superframes in %.1f s (%.1f/s), skipped %d, write mean %.2f ms max %.1f ms, last status %s' % (seq, time.monotonic() - t_start, seq / max(time.monotonic() - t_start, .001), skipped, wr_tot * 1e3 / max(seq, 1), wr_max * 1e3, L.st), flush=True)
        L.s.write(b'ctr' + bytes([10]) + b'status' + bytes([10]))
        time.sleep(0.4)
        tail = b''.join(L.console)[-1400:].decode('latin1').splitlines()
        for ln in tail:
            if ln.startswith('CTR'):
                print(ln, flush=True)
        if a.console:
            with open(a.console, 'ab') as f:
                f.write(b''.join(L.console))
        L.close()


if __name__ == '__main__':
    sys.exit(main() or 0)
