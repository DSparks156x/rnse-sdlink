r"""sdfast.py -- fast RAM-program load over SD (SDBOOT.txt). PC side, used by rnse_bench.Bench.start.

Image -> wire images -> ssh -> Pi helper (sdloadd.py) -> Teensy superframe window; CAN then starts the resident bootstrap,
which reads, checks and starts the image. Any failure raises FastFail and Bench.start falls back to the CAN upload.
"""
import base64, hashlib, os, struct, subprocess, threading, time, zlib
import layout as L

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'out')
SSH_HOST = 'hudiy'

HDR_MAGIC = 0x5253424C
RES_MAGIC = 0x52534252
CODES = {1: 'bootstrap running or stuck', 2: 'bad header', 3: 'range check', 4: 'SD read failed', 5: 'image CRC bad', 6: 'firmware bytes differ',
         7: 'loaded (not started)', 8: 'jumped to the program'}
CODE_LOADED, CODE_JUMP = 7, 8


class FastFail(Exception):
    pass


# ---------------------------------------------------------------- wire images (= sdproto.cpp wire4, 528 B little-endian words)
_T2 = []
for _i in range(4):                                     # crc = (crc << 2) ^ T[(crc >> 14) ^ bits]
    _c = _i << 14
    for _ in range(2):
        _c = ((_c << 1) ^ 0x1021) & 0xFFFF if _c & 0x8000 else (_c << 1) & 0xFFFF
    _T2.append(_c)
_BITS = [[((_b >> (4 + _k)) & 1) << 1 | ((_b >> _k) & 1) for _b in range(256)] for _k in range(4)]
_SWAP = bytes(((_b >> 4) | (_b << 4)) & 255 for _b in range(256))


def wire4(sec):
    """One 512-B sector -> its 528-B wire image: start word, nibble-swapped data, 4 line CRCs, end word."""
    assert len(sec) == 512
    crcs = []
    t = _T2
    for k in range(4):
        c = 0
        bk = _BITS[k]
        for b in sec:
            c = ((c << 2) & 0xFFFF) ^ t[(c >> 14) ^ bk[b]]
        crcs.append(c)
    w0 = w1 = 0
    for j in range(16):
        nib = 0
        for k in range(4):
            nib |= ((crcs[k] >> (15 - j)) & 1) << k
        if j < 8:
            w0 |= nib << (4 * j)
        else:
            w1 |= nib << (4 * (j - 8))
    return struct.pack('<I', 0x0FFFFFFF) + sec.translate(_SWAP) + struct.pack('<III', w0, w1, 0xFFFFFFFF)


def wire_image(data):
    """data padded to sectors -> concatenated wire images, cached on disk by content."""
    data = data + bytes(-len(data) % 512)
    sha = hashlib.sha256(data).hexdigest()
    path = os.path.join(OUT, '.wire_%s.bin' % sha[:16])
    if os.path.exists(path):
        w = open(path, 'rb').read()
        if len(w) == len(data) // 512 * 528:
            return w, sha
    w = b''.join(wire4(data[i:i + 512]) for i in range(0, len(data), 512))
    for f in os.listdir(OUT):
        if f.startswith('.wire_'):
            try:
                os.remove(os.path.join(OUT, f))
            except OSError:
                pass
    open(path, 'wb').write(w)
    return w, sha


def make_header(load_id, base, img, entry, win, ctl, start=True, checks=None):
    nsect = (len(img) + 511) // 512
    checks = L.FW_CHECKS if checks is None else checks
    if len(ctl) % 4 or len(ctl) > 256 or len(checks) > 15 or nsect > 255:
        raise FastFail('header limits')
    h = bytearray(512)
    struct.pack_into('>11I', h, 0, HDR_MAGIC, load_id, base, len(img), entry, win, zlib.crc32(img) & 0xFFFFFFFF, nsect,
                     len(ctl) // 4, 1 if start else 0, len(checks))
    h[64:64 + len(ctl)] = ctl
    for i, (a, exp) in enumerate(checks):
        e = bytes.fromhex(exp)
        assert len(e) == 8
        struct.pack_into('>I', h, 320 + 12 * i, a)
        h[320 + 12 * i + 4:320 + 12 * i + 12] = e
    struct.pack_into('>I', h, 508, zlib.crc32(bytes(h[:508])) & 0xFFFFFFFF)
    return bytes(h)


# ---------------------------------------------------------------- Pi helper over one long-lived ssh
class Pi:
    def __init__(self):
        self.p = None
        self.sent = None
        self.lines = []
        self.cv = threading.Condition()

    def _reader(self, p):
        for line in p.stdout:
            with self.cv:
                self.lines.append(line.decode('utf-8', 'replace').strip())
                self.cv.notify_all()
        with self.cv:
            self.lines.append('EOF')
            self.cv.notify_all()

    def ensure(self):
        if self.p and self.p.poll() is None:
            return
        src = open(os.path.join(HERE, 'sdloadd.py'), 'rb').read()
        code = "import sys;i=sys.stdin.buffer;exec(compile(i.read(int(i.readline())),chr(115),chr(101)+chr(120)+chr(101)+chr(99)))"    # ssh on Windows cuts long command lines: the script goes over stdin
        self.p = subprocess.Popen(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=3', '-o', 'ServerAliveInterval=10', SSH_HOST,
                                   "python3 -u -c '%s'" % code], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=open(os.path.join(OUT, '.sdloadd.err'), 'ab'), creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        self.p.stdin.write(b'%d\n' % len(src) + src)
        self.p.stdin.flush()
        self.lines = []
        self.sent = None
        threading.Thread(target=self._reader, args=(self.p,), daemon=True).start()

    def next_line(self, timeout):
        end = time.monotonic() + timeout
        with self.cv:
            while not self.lines:
                left = end - time.monotonic()
                if left <= 0:
                    raise FastFail('Pi helper: no reply')
                self.cv.wait(left)
            return self.lines.pop(0)

    def send_load(self, seq, hdr_wire, img_wire, sha):
        self.ensure()
        self.lines = []
        use_cache = self.sent == sha
        body = img_wire if not use_cache else b''
        try:
            self.p.stdin.write(b'L %d %d %d %s\n' % (seq, len(hdr_wire), len(body), sha.encode()) + hdr_wire + body)
            self.p.stdin.flush()
        except OSError as e:
            self.p = None
            raise FastFail('ssh to the Pi: %s' % e)
        self.sent = sha

    def close(self):
        if self.p and self.p.poll() is None:
            try:
                self.p.stdin.write(b'Q\n')
                self.p.stdin.flush()
            except OSError:
                pass
            self.p.terminate()
        self.p = None


_pi = Pi()


def send_image(load_id, base, img, entry, win, ctl, start=True):
    """Puts header + image on the Teensy. Returns the ms it took. Then the Pi holds the port until the unit has read it."""
    hdr = make_header(load_id, base, img, entry, win, ctl, start)
    w, sha = wire_image(img)
    t0 = time.monotonic()
    first = _pi.p is None or _pi.p.poll() is not None
    # Sector 0 is a dummy the bootstrap never reads: reading a slot's sector 0 marks it served, and with avlink streaming a served
    # slot is the first one the Teensy evicts at the next superframe (new_slot), in the middle of the image read (bench: 8 of 20 CRC bad).
    _pi.send_load(L.SDBOOT_SEQ, wire4(bytes(512)) + wire4(hdr), w, sha)
    line = _pi.next_line(8.0 if first else 3.0)
    while line[:1] in ('S', 'T'):                       # late end of an earlier load
        line = _pi.next_line(3.0)
    if line.startswith('R'):
        f = line.split()
        lba = 1 + (L.SDBOOT_LBA if len(f) < 3 else 0x10000 + (int(f[2]) & 255) * 256)    # seq given: avlink queued it in its stream
        return (time.monotonic() - t0) * 1000, lba
    raise FastFail('Pi: ' + line)


def stop_stream(timeout=1.0):
    """STOP TLV via avlink's control FIFO: the reader exits on the next superframe. Raises FastFail when there is no avlink."""
    _pi.ensure()
    _pi.lines = []
    _pi.p.stdin.write(b'K\n')
    _pi.p.stdin.flush()
    line = _pi.next_line(timeout)
    if line != 'K ok':
        raise FastFail('Pi: ' + line)


def wait_served(timeout=4.0):
    return _pi.next_line(timeout)


def boot_image():
    """Bootstrap bytes + stamp (magic, CRC, length, 0)."""
    code = open(os.path.join(OUT, 'sdboot.bin'), 'rb').read()
    if len(code) > L.SDBOOT_STAMP - L.SDBOOT_BASE:
        raise FastFail('sdboot too large')
    code += bytes(-len(code) % 4)
    return code, struct.pack('>IIII', L.SDBOOT_MAGIC, zlib.crc32(code) & 0xFFFFFFFF, len(code), 0)
