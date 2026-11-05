#!/usr/bin/env python3
r"""capdec -- Pi side of the RNS-E screen capture (uplink channel 2, link/SPEC.md 4.1; encoder rnse_proto/src/capenc.c,
format in capenc.h, design in rnse_proto/CAPTURE.txt).

Reassembles RSUP sectors into messages, applies the capture records (HDR / PAL / TILES / END) to one picture per DU
layer, verifies the END CRC, and writes PNG / raw / an RGB24 stream / a v4l2loopback device.

  capdec.py SECTORS [--hostwrite] [--png DIR] [--raw DIR] [--all] [--composite] [--rgb-out FILE|-]
                    [--v4l2 /dev/videoN] [--quiet]

  SECTORS      file of concatenated 512-B sectors (e.g. the uplink_stub ring), or '-' for stdin (streaming);
               with --hostwrite: a log of Teensy TLNK messages (SPEC 5.2; type 0x81 HOST_WRITE {u32 lba, u8 crc_ok,
               512 B}), only LBAs in the channel-2 window 0x28000..0x2BFFF are used.
  --png DIR    one PNG per converged frame (END with pending == 0) per layer; --all = every END
  --composite  also write/stream the composite of all layers (HDR z: 1 = top, colour key honoured)
  --rgb-out    raw RGB24 frames of the composite (or the single layer) to a file / stdout, e.g.
                 capdec.py - --rgb-out - < pipe | ffmpeg -f rawvideo -pix_fmt rgb24 -s 800x480 -i - -f v4l2 /dev/video10
  --v4l2 DEV   write RGB24 frames straight into a v4l2loopback device (VIDIOC_S_FMT + write(); Linux only, untested
               on hardware: modprobe v4l2loopback video_nr=10 exclusive_caps=1 card_label=RNS-E)

Library use (the Pi link daemon): d = CapDecoder(on_frame=cb); d.feed_sector(sector512) for every channel-2 sector in
LBA order; d.need_refresh tells the daemon to send the capture-refresh request on the down-channel.
Needs numpy. Python: the Pi's python3 (on the PC: the codex runtime python with numpy).
"""
import sys, os, struct, zlib, argparse
import numpy as np

RSUP = 0x52535550
CHAN = 2
LBA_BASE, LBA_WIN = 0x28000, 0x4000
R_HDR, R_PAL, R_TILES, R_END = 1, 2, 3, 4
M_SOLID, M_RAW, M_OPS = 0, 1, 2
FMT_NAMES = {0: 'PAL8', 1: 'RGB565', 2: 'ARGB1555', 3: 'YUY2', 4: 'XRGB8888'}
DT = {1: np.dtype('u1'), 2: np.dtype('>u2'), 4: np.dtype('>u4')}


class CapError(Exception):
    pass


class Layer:
    def __init__(self, idx):
        self.idx = idx
        self.epoch = 0
        self.w = self.h = self.bpp = self.fmt = 0
        self.tw, self.th = 64, 16
        self.pix = None
        self.known = None
        self.pal = np.zeros(256, dtype=np.uint32)
        self.x = self.y = 0
        self.z = 0
        self.key = None
        self.mr = 0
        self.damaged = False

    def setup(self, epoch, w, h, fmt, bpp, tw, th):
        if epoch < self.epoch:
            return False
        if epoch > self.epoch or self.pix is None or (w, h, bpp) != (self.w, self.h, self.bpp):
            self.epoch, self.w, self.h, self.fmt, self.bpp, self.tw, self.th = epoch, w, h, fmt, bpp, tw, th
            self.pix = np.zeros((h, w), dtype=DT[bpp])
            self.ntx = (w + tw - 1) // tw
            self.nty = (h + th - 1) // th
            self.known = np.zeros(self.ntx * self.nty, dtype=bool)
            self.damaged = False
        return True

    def crc(self):
        return zlib.crc32(self.pix.tobytes()) & 0xFFFFFFFF

    def rgb(self):
        """-> HxWx3 uint8 (display interpretation of the DU format)"""
        v = self.pix.astype(np.uint32)
        if self.fmt == 1:                                             # RGB565
            r, g, b = v >> 11 & 31, v >> 5 & 63, v & 31
            return np.stack([r << 3 | r >> 2, g << 2 | g >> 4, b << 3 | b >> 2], -1).astype(np.uint8)
        if self.fmt == 2:                                             # ARGB1555
            r, g, b = v >> 10 & 31, v >> 5 & 31, v & 31
            return np.stack([r << 3 | r >> 2, g << 3 | g >> 2, b << 3 | b >> 2], -1).astype(np.uint8)
        if self.fmt == 0:                                             # PAL8, CLUT 0x00RRGGBB
            p = self.pal[v]
            return np.stack([p >> 16 & 255, p >> 8 & 255, p & 255], -1).astype(np.uint8)
        if self.fmt == 4:
            return np.stack([v >> 16 & 255, v >> 8 & 255, v & 255], -1).astype(np.uint8)
        if self.fmt == 3:                                             # YUY2 Y0 Cb Y1 Cr (BT.601 studio)
            b = self.pix.view(np.uint8).reshape(self.h, self.w * 2).astype(np.float32)
            y = b[:, 0::2]
            cb = np.repeat(b[:, 1::4], 2, axis=1)[:, :self.w] - 128
            cr = np.repeat(b[:, 3::4], 2, axis=1)[:, :self.w] - 128
            yy = (y - 16) * 1.164
            out = np.stack([yy + 1.596 * cr, yy - 0.392 * cb - 0.813 * cr, yy + 2.017 * cb], -1)
            return np.clip(out + 0.5, 0, 255).astype(np.uint8)
        raise CapError('unknown fmt %d' % self.fmt)

    def mask(self):
        """True where the layer is opaque (colour key)"""
        if self.key is None:
            return np.ones((self.h, self.w), dtype=bool)
        return self.pix != self.key


def decode_ops(ops, old, n, tw, B, dt):
    out = [0] * n
    i = pos = prev = 0
    L = len(ops)
    while i < n:
        if pos >= L:
            raise CapError('ops underrun')
        b = ops[pos]; pos += 1
        k, c = b >> 6, (b & 63) + 1
        if i + c > n:
            raise CapError('op past tile end')
        if k == 0:
            out[i:i + c] = [prev] * c
        elif k == 1:
            if i < tw:
                raise CapError('UP in row 0')
            if c <= tw:
                out[i:i + c] = out[i - tw:i - tw + c]
            else:
                for j in range(i, i + c):
                    out[j] = out[j - tw]
        elif k == 2:
            e = pos + c * B
            if e > L:
                raise CapError('literal past end')
            out[i:i + c] = np.frombuffer(ops[pos:e], dtype=dt).tolist()
            pos = e
        else:
            if old is None:
                raise CapError('OLD op on an unknown tile')
            out[i:i + c] = old[i:i + c]
        i += c
        prev = out[i - 1]
    if pos != L:
        raise CapError('ops trailing bytes')
    return out


class CapDecoder:
    def __init__(self, on_frame=None, verbose=False):
        self.layers = {}
        self.on_frame = on_frame
        self.verbose = verbose
        self.expect_seq = None
        self.cur = None             # (seq, [payloads])
        self.need_refresh = False
        self.stats = dict(sectors=0, bad_magic=0, bad_crc=0, other_chan=0, msgs=0, gaps=0, lost_msgs=0,
                          dropped_partial=0, records=0, tiles=0, ends=0, crc_ok=0, crc_bad=0, errors=0, stale=0)
        self.events = []            # END events: dict(layer, epoch, frame, pending, dirty, crc_valid, crc_ok)
        self.errors = []

    # ---------------------------------------------------------------- RSUP layer
    def feed_sector(self, s):
        st = self.stats
        st['sectors'] += 1
        if len(s) < 512 or struct.unpack_from('>I', s, 0)[0] != RSUP:
            st['bad_magic'] += 1
            return
        chan, flags, plen, seq, crc = struct.unpack_from('>BBHII', s, 4)
        if chan != CHAN:
            st['other_chan'] += 1
            return
        if plen > 496:
            st['bad_magic'] += 1
            return
        pay = bytes(s[16:16 + plen])
        if zlib.crc32(pay) & 0xFFFFFFFF != crc:
            st['bad_crc'] += 1
            self._lost('sector CRC')
            self.cur = None
            return
        if flags & 1:
            if self.cur is not None:
                st['dropped_partial'] += 1
            self.cur = (seq, [pay])
        elif self.cur is None or self.cur[0] != seq:
            st['dropped_partial'] += 1
            self.cur = None
            self._lost('continuation without first')
            return
        else:
            self.cur[1].append(pay)
        if flags & 2:
            seq0, parts = self.cur
            self.cur = None
            if self.expect_seq is not None and seq0 != self.expect_seq:
                st['gaps'] += 1
                st['lost_msgs'] += (seq0 - self.expect_seq) & 0xFFFFFFFF
                self._lost('message seq gap %d -> %d' % (self.expect_seq, seq0))
            self.expect_seq = (seq0 + 1) & 0xFFFFFFFF
            st['msgs'] += 1
            self.feed_message(b''.join(parts))

    def _lost(self, why):
        if not self.need_refresh and self.verbose:
            print('capdec: loss (%s) -> refresh needed' % why)
        self.need_refresh = True
        for l in self.layers.values():
            l.damaged = True

    # ---------------------------------------------------------------- capture records
    def layer(self, i):
        if i not in self.layers:
            self.layers[i] = Layer(i)
        return self.layers[i]

    def feed_message(self, m):
        pos = 0
        while pos + 4 <= len(m):
            typ, li, rl = struct.unpack_from('>BBH', m, pos)
            if rl < 4 or pos + rl > len(m):
                self._err('bad record length %d at %d' % (rl, pos))
                return
            try:
                self.record(typ, self.layer(li), m[pos:pos + rl])
            except CapError as e:
                self._err('layer %d: %s' % (li, e))
                self.layer(li).damaged = True
                self.need_refresh = True
            pos += rl
            self.stats['records'] += 1

    def _err(self, s):
        self.stats['errors'] += 1
        if len(self.errors) < 100:
            self.errors.append(s)
        if self.verbose:
            print('capdec: ERROR ' + s)

    def _epoch(self, L, epoch):
        if epoch < L.epoch:
            self.stats['stale'] += 1
            return False
        if epoch > L.epoch and L.pix is not None:
            L.epoch = epoch
            L.known[:] = False
            L.damaged = False
            if all(l.epoch >= epoch or l.pix is None for l in self.layers.values()):
                self.need_refresh = False
        return True

    def record(self, typ, L, r):
        if typ == R_HDR:
            epoch, w, h, fmt, bpp, tw, th, x, y, z, fl, _, key, mr = struct.unpack_from('>IHHBBBBhhBBHII', r, 4)
            if self._epoch(L, epoch) and L.setup(epoch, w, h, fmt, bpp, tw, th):
                L.x, L.y, L.z, L.mr = x, y, z, mr
                L.key = key if fl & 1 else None
        elif typ == R_PAL:
            epoch, n = struct.unpack_from('>IH', r, 4)
            if self._epoch(L, epoch):
                L.pal[:n] = np.frombuffer(r, dtype='>u4', count=n, offset=12)
        elif typ == R_TILES:
            epoch, fid, w, h, fmt, bpp, tw, th, nt = struct.unpack_from('>IIHHBBBBH', r, 4)
            if not self._epoch(L, epoch):
                return
            if not L.setup(epoch, w, h, fmt, bpp, tw, th):
                return
            if L.epoch < epoch:
                L.epoch = epoch
            self.tiles(L, r, 24, nt)
        elif typ == R_END:
            epoch, fid, pend, dirty, fl, crc = struct.unpack_from('>IIIIII', r, 4)
            if not self._epoch(L, epoch) or L.pix is None:
                return
            ev = dict(layer=L.idx, epoch=epoch, frame=fid, pending=pend, dirty=dirty, crc_valid=bool(fl & 1),
                      crc_ok=None, damaged=L.damaged, complete=bool(L.known.all()))
            if fl & 1:
                ev['crc_ok'] = L.crc() == crc
                self.stats['crc_ok' if ev['crc_ok'] else 'crc_bad'] += 1
                if not ev['crc_ok']:
                    L.damaged = True
                    self.need_refresh = True
            self.stats['ends'] += 1
            self.events.append(ev)
            if self.on_frame:
                self.on_frame(self, L, ev)
        # unknown record types are skipped (forward compatibility)

    def tiles(self, L, r, pos, nt):
        B, dt, P = L.bpp, DT[L.bpp], L.pix
        for _ in range(nt):
            t, mode = struct.unpack_from('>HB', r, pos)
            pos += 3
            if t >= L.ntx * L.nty:
                raise CapError('tile index %d out of range' % t)
            x0, y0 = t % L.ntx * L.tw, t // L.ntx * L.th
            tw, th = min(L.tw, L.w - x0), min(L.th, L.h - y0)
            n = tw * th
            if mode == M_SOLID:
                v = np.frombuffer(r, dtype=dt, count=1, offset=pos)[0]
                P[y0:y0 + th, x0:x0 + tw] = v
                pos += B
            elif mode == M_RAW:
                P[y0:y0 + th, x0:x0 + tw] = np.frombuffer(r, dtype=dt, count=n, offset=pos).reshape(th, tw)
                pos += n * B
            elif mode == M_OPS:
                ln = struct.unpack_from('>H', r, pos)[0]
                pos += 2
                old = P[y0:y0 + th, x0:x0 + tw].ravel().tolist() if L.known[t] else None
                if old is None and L.damaged and any(b >> 6 == 3 for b in self._opbytes(r[pos:pos + ln], B)):
                    # after a detected loss the encoder may reference pixels we never got: skip until the refresh
                    self.stats['skipped_damaged'] = self.stats.get('skipped_damaged', 0) + 1
                    pos += ln
                    continue
                out = decode_ops(r[pos:pos + ln], old, n, tw, B, dt)
                P[y0:y0 + th, x0:x0 + tw] = np.array(out, dtype=dt).reshape(th, tw)
                pos += ln
            else:
                raise CapError('tile mode %d' % mode)
            L.known[t] = True
            self.stats['tiles'] += 1
        if pos != len(r):
            raise CapError('TILES record length mismatch (%d vs %d)' % (pos, len(r)))

    @staticmethod
    def _opbytes(ops, B):
        pos = 0
        while pos < len(ops):
            b = ops[pos]
            yield b
            pos += 1 + ((b & 63) + 1) * B if b >> 6 == 2 else 1

    # ---------------------------------------------------------------- output helpers
    def composite(self):
        ls = [l for l in self.layers.values() if l.pix is not None]
        if not ls:
            return None
        W = max(l.x + l.w for l in ls); H = max(l.y + l.h for l in ls)
        out = np.zeros((H, W, 3), dtype=np.uint8)
        for l in sorted(ls, key=lambda l: -l.z):          # z 1 = top: paint the highest z number first
            m = l.mask()
            out[l.y:l.y + l.h, l.x:l.x + l.w][m] = l.rgb()[m]
        return out


def write_png(path, rgb):
    h, w = rgb.shape[:2]
    raw = b''.join(b'\x00' + rgb[y].tobytes() for y in range(h))

    def chunk(t, d):
        c = struct.pack('>I', len(d)) + t + d
        return c + struct.pack('>I', zlib.crc32(t + d) & 0xFFFFFFFF)
    open(path, 'wb').write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)) +
                           chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


class V4L2Out:
    """write RGB24 frames into a v4l2loopback device (Linux). VIDIOC_S_FMT on struct v4l2_format (208 B, 64-bit)."""
    def __init__(self, dev, w, h):
        import fcntl
        self.fd = os.open(dev, os.O_RDWR)
        fmt = bytearray(208)
        V4L2_BUF_TYPE_VIDEO_OUTPUT, RGB3 = 2, 0x33424752     # fourcc 'RGB3' = RGB24
        struct.pack_into('=I', fmt, 0, V4L2_BUF_TYPE_VIDEO_OUTPUT)
        struct.pack_into('=IIIIII', fmt, 8, w, h, RGB3, 1, w * 3, w * h * 3)   # field NONE = 1
        fcntl.ioctl(self.fd, 0xC0D05605, fmt)                   # VIDIOC_S_FMT
        self.size = (w, h)

    def write(self, rgb):
        os.write(self.fd, rgb.tobytes())


def sectors_from(src, hostwrite):
    f = sys.stdin.buffer if src == '-' else open(src, 'rb')
    if not hostwrite:
        while True:
            s = f.read(512)
            if len(s) < 512:
                return
            yield s
    while True:
        h = f.read(16)
        if len(h) < 16:
            return
        magic, typ, fl, ln, seq = struct.unpack('<IHHII', h)
        if magic != 0x4B4E4C54:
            raise SystemExit('hostwrite log: lost TLNK framing')
        p = f.read(ln)
        if typ == 0x81 and ln >= 517:
            lba, ok = struct.unpack_from('<IB', p, 0)
            if ok and LBA_BASE <= lba < LBA_BASE + LBA_WIN:
                yield p[5:517]


def main(argv):
    ap = argparse.ArgumentParser(description='RNS-E capture decoder (uplink channel 2)')
    ap.add_argument('src')
    ap.add_argument('--hostwrite', action='store_true')
    ap.add_argument('--png'); ap.add_argument('--raw'); ap.add_argument('--all', action='store_true')
    ap.add_argument('--composite', action='store_true')
    ap.add_argument('--rgb-out'); ap.add_argument('--v4l2')
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args(argv)
    for d in (a.png, a.raw):
        if d:
            os.makedirs(d, exist_ok=True)
    rgb_f = None
    if a.rgb_out:
        rgb_f = sys.stdout.buffer if a.rgb_out == '-' else open(a.rgb_out, 'wb')
    v4 = [None]

    def on_frame(dec, L, ev):
        if ev['pending'] and not a.all:
            return
        tag = 'L%d_e%d_f%06d' % (L.idx, ev['epoch'], ev['frame'])
        if a.png:
            write_png(os.path.join(a.png, tag + '.png'), L.rgb())
        if a.raw:
            open(os.path.join(a.raw, tag + '.raw'), 'wb').write(L.pix.tobytes())
        if a.composite or rgb_f or a.v4l2:
            img = dec.composite() if (a.composite or len(dec.layers) > 1) else L.rgb()
            if a.composite and a.png:
                write_png(os.path.join(a.png, 'comp_f%06d.png' % ev['frame']), img)
            if rgb_f:
                rgb_f.write(img.tobytes()); rgb_f.flush()
            if a.v4l2:
                if v4[0] is None:
                    v4[0] = V4L2Out(a.v4l2, img.shape[1], img.shape[0])
                v4[0].write(img)
        if not a.quiet:
            print('frame %6d layer %d epoch %d pending %d dirty %d crc %s%s' % (
                ev['frame'], L.idx, ev['epoch'], ev['pending'], ev['dirty'],
                '-' if ev['crc_ok'] is None else ('ok' if ev['crc_ok'] else 'BAD'),
                ' DAMAGED(refresh needed)' if ev['damaged'] else ''), file=sys.stderr)
    d = CapDecoder(on_frame=on_frame, verbose=not a.quiet)
    for s in sectors_from(a.src, a.hostwrite):
        d.feed_sector(s)
    print('capdec: ' + ' '.join('%s=%d' % kv for kv in d.stats.items()), file=sys.stderr)
    for l in d.layers.values():
        print('  layer %d: %dx%d %s epoch %d %s' % (l.idx, l.w, l.h, FMT_NAMES.get(l.fmt, l.fmt), l.epoch,
                                                     'damaged' if l.damaged else 'ok'), file=sys.stderr)
    return 1 if d.stats['errors'] or d.stats['crc_bad'] else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
