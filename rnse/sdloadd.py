#!/usr/bin/env python3
"""sdloadd.py -- runs on the Pi, driven over one long-lived ssh by sdfast.py (sent inline, nothing to deploy). See SDBOOT.txt.

stdin:  'L <seq> <nhdr> <nimg> <sha>\n' + nhdr bytes (wire images) + nimg bytes (wire images; 0 = use the cached image <sha>)
        'Q\n' quit
stdout: 'BUSY <who>\n' | 'MISS\n' | 'ERR <text>\n' | 'R <ms>\n' = the superframe is complete on the Teensy, then
        'S <sectors> <ms>\n' once the unit has read it, or 'T <sectors>\n' on timeout; the port closes after either.
Only opens /dev/ttyACM0 when no other process holds it (avlink owns it while streaming). stdlib only.
"""
import fcntl, os, select, struct, sys, termios, time

DEV = '/dev/ttyACM0'
FIFO = '/tmp/avlink_sdctl'
SWAP = bytes(((b >> 4) | (b << 4)) & 255 for b in range(256))
MAGIC = 0x4B4E4C54
SF_BEGIN, WIRE, SF_END, CONFIG, STATUS, SERVED = 0x01, 0x02, 0x03, 0x05, 0x83, 0x85
CHUNK = 32
inp = sys.stdin.buffer
out = sys.stdout


def say(s):
    out.write(s + '\n')
    out.flush()


def holders():
    target = os.path.realpath(DEV)
    who = []
    for p in os.listdir('/proc'):
        if not p.isdigit() or int(p) == os.getpid():
            continue
        try:
            for fd in os.listdir('/proc/%s/fd' % p):
                if os.path.realpath('/proc/%s/fd/%s' % (p, fd)) == target:
                    who.append(open('/proc/%s/comm' % p).read().strip())
                    break
        except OSError:
            pass
    return who


class Link:
    def __init__(self):
        self.fd = os.open(DEV, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        t = termios.tcgetattr(self.fd)
        t[0] = t[1] = t[3] = 0
        t[2] |= termios.CLOCAL | termios.CREAD
        t[6][termios.VMIN] = 0
        t[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, t)
        termios.tcflush(self.fd, termios.TCIFLUSH)
        self.seq = 0
        self.rx = b''
        self.status = None
        self.served = None

    def close(self):
        os.close(self.fd)

    def write(self, data):
        m = memoryview(data)
        end = time.monotonic() + 2
        while m:
            try:
                n = os.write(self.fd, m)
                m = m[n:]
            except BlockingIOError:
                if time.monotonic() > end:
                    raise IOError('USB write stuck')
                select.select([], [self.fd], [], 0.01)

    def msg(self, typ, payload=b''):
        self.write(struct.pack('<IHHII', MAGIC, typ, 0, len(payload), self.seq) + payload)
        self.seq += 1

    def pump(self, wait):
        r, _, _ = select.select([self.fd], [], [], wait)
        if r:
            try:
                self.rx += os.read(self.fd, 65536)
            except BlockingIOError:
                pass
        while True:
            i = self.rx.find(b'TLNK')
            if i < 0:
                self.rx = self.rx[-3:]
                return
            self.rx = self.rx[i:]
            if len(self.rx) < 16:
                return
            mag, typ, fl, ln, sq = struct.unpack('<IHHII', self.rx[:16])
            if ln > 4096:
                self.rx = self.rx[4:]
                continue
            if len(self.rx) < 16 + ln:
                return
            p = self.rx[16:16 + ln]
            self.rx = self.rx[16 + ln:]
            if typ == STATUS and ln >= 76:
                self.status = dict(zip(('free', 'rx', 'slots', 'begun', 'newest', 'served', 'srv_sect'), struct.unpack('<7I', p[4:32])),
                                   flags=p[75])
            elif typ == SERVED and ln >= 8:
                self.served = struct.unpack('<I', p[:4])[0]


def fifo_write(data, timeout=2.0):
    fd = os.open(FIFO, os.O_WRONLY | os.O_NONBLOCK)    # ENXIO/ENOENT: no avlink with the control FIFO
    try:
        m = memoryview(data)
        end = time.monotonic() + timeout
        while m:
            try:
                m = m[os.write(fd, m):]
            except BlockingIOError:
                if time.monotonic() > end:
                    raise IOError('control FIFO stuck')
                select.select([], [fd], [], 0.005)
    finally:
        os.close(fd)


def inject(hdr, img):
    """avlink owns the port: it queues the image as a superframe and tells which seq it got."""
    t0 = time.monotonic()
    wire = hdr + img
    raw = b''.join(wire[i + 4:i + 516].translate(SWAP) for i in range(0, len(wire), 528))
    try:
        os.remove(FIFO + '.seq')
    except OSError:
        pass
    fifo_write(b'IMG1' + struct.pack('<I', len(raw)) + raw)
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        try:
            v = open(FIFO + '.seq').read()
        except OSError:
            v = ''
        if v.endswith('\n'):
            time.sleep(0.005)                           # USB: bytes left the kernel; the bootstrap's holds cover the rest
            say('R %.1f %d' % ((time.monotonic() - t0) * 1000, int(v)))
            say('S 0 0')
            return
        time.sleep(0.002)
    say('ERR avlink did not take the image')


def load(seq, hdr, img, wait_s):
    who = holders()
    if who:
        if all(w.startswith('avlink') for w in who) and os.path.exists(FIFO):
            inject(hdr, img)
        else:
            say('BUSY ' + ','.join(who))
        return
    t0 = time.monotonic()
    l = Link()
    try:
        time.sleep(0.03)                                # a message before the Teensy's DTR debounce (15 ms) resets the link
        cfg = struct.pack('<HHHHB7x', 0, 0, 0, 5000, 200)
        end = time.monotonic() + 1.0
        n = 0
        while not l.status:
            if time.monotonic() > end:
                say('ERR no STATUS from the Teensy')
                return
            if n % 5 == 0:
                l.msg(CONFIG, cfg)
            n += 1
            l.pump(0.01)
        base = l.status['srv_sect']
        wire = hdr + img
        total = len(wire) // 528
        l.msg(SF_BEGIN, struct.pack('<IHH', seq, total, 0))
        for k in range(0, total, CHUNK):
            c = wire[k * 528:(k + CHUNK) * 528]
            l.msg(WIRE, struct.pack('<IHHI', seq, k, len(c) // 528, 4) + c)
        l.msg(SF_END, struct.pack('<I', seq))
        end = time.monotonic() + 1.0
        while not (l.status['flags'] & 1 and l.status['newest'] == seq):
            if time.monotonic() > end:
                say('ERR superframe not complete on the Teensy: %s' % (l.status,))
                return
            l.pump(0.005)
        say('R %.1f' % ((time.monotonic() - t0) * 1000))
        t1 = time.monotonic()
        end = t1 + wait_s
        while time.monotonic() < end:
            l.pump(0.005)
            if l.status['srv_sect'] - base >= total - 1:      # sector 0 is never read
                time.sleep(0.01)
                say('S %d %.1f' % (l.status['srv_sect'] - base, (time.monotonic() - t1) * 1000))
                return
        say('T %d' % (l.status['srv_sect'] - base))
    finally:
        l.close()


def main():
    cache = {}
    while True:
        line = inp.readline()
        if not line or line.startswith(b'Q'):
            return
        if line.startswith(b'K'):                       # instant stop of the running sdstream, via the next superframes
            try:
                fifo_write(b'STOP')
                say('K ok')
            except OSError as e:
                say('K %s' % e)
            continue
        f = line.split()
        if f[0] != b'L':
            say('ERR bad command')
            continue
        seq, nh, ni, sha = int(f[1]), int(f[2]), int(f[3]), f[4]
        hdr = inp.read(nh)
        img = inp.read(ni) if ni else cache.get(sha)
        if ni:
            cache = {sha: img}                          # one image is enough
        if img is None:
            say('MISS')
            continue
        try:
            load(seq, hdr, img, 3.0)
        except Exception as e:
            say('ERR %s: %s' % (type(e).__name__, e))


if __name__ == '__main__':
    main()
