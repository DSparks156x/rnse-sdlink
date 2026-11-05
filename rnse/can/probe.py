r"""soc_caps_v1 bench probe runner (r56 bench unit, CowCAN-FT, 0x7B0 -> 0x7B6).

Only existing r56 commands are used:
  B1 a3 a2 a1 a0 LEN  -> read RAM/flash (and the fixed idread P4 allow-list)
  B9 i2 i1 i0 d3..d0  -> u32 RAM write at 0x8C000000 + idx*4 (P1, write-through: CCR=0x80008981, CB=0)
  B8 5A               -> sta_tsk(159)
RAM code exec = write a probe program to fresh RAM, point TCB159+0x20 (task entry, read by the sta_tsk frame
builder 0x804CF486) at it, B8 5A, read results, restore TCB159+0x20 = 0x80B06000 (Doom entry).

Every P4 address is appended to probe_log.txt BEFORE the request that touches it is sent.
Run: python -I probe.py CMD ...
"""
import os, sys, time, struct
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cowcan_memread as cm
import sh4asm

TCB159 = 0x8E8D7B6C
ENTRY_OFF = 0x20
DOOM_ENTRY = 0x80B06000
TRIG = 0x8EF803E0
TAB = 0x8F5E0000
PROG = 0x8F5E0FF0
RES = 0x8F5E1000
CODE_BASE = 0x8F500000
LOG = os.path.join(HERE, 'probe_log.txt')
STATE = os.path.join(HERE, 'code_slot.txt')     # next fresh code slot (never re-execute an address: I-cache)


def log(msg):
    with open(LOG, 'a') as f:
        f.write(time.strftime('%Y-%m-%d %H:%M:%S ') + msg + '\n')
        f.flush(); os.fsync(f.fileno())


class Bus:
    def __init__(self):
        self.b = cm.CowBus(100000, reply_id=0x7B6, req_id=0x7B0)

    def close(self):
        self.b.close()

    def send(self, data):
        self.b.send(bytes(data))
        time.sleep(0.004)

    def read(self, addr, n):
        out = b''
        while n:
            k = min(n, 48)
            out += self._read1(addr, k)
            addr += k; n -= k
        return out

    def _read1(self, addr, n):
        req = [0xB1, (addr >> 24) & 255, (addr >> 16) & 255, (addr >> 8) & 255, addr & 255, n, 0, 0]
        nf = (n + 5) // 6
        for attempt in range(5):
            self.b.drain(); self.b.send(bytes(req))
            hdr = None; fr = {}
            end = time.monotonic() + 1.0 + nf * 0.05
            while time.monotonic() < end:
                m = self.b.recv(max(0.01, end - time.monotonic()))
                if not m:
                    continue
                d = m[1]
                if len(d) != 8:
                    continue
                if d[0] == 0x80:
                    hdr = d
                    if d[1] != 0:
                        raise IOError('B1 %08X status %d' % (addr, d[1]))
                elif 0x81 <= d[0] <= 0x80 + nf:
                    if (sum(d[1:7]) & 255) == d[7]:
                        fr[d[0] & 0x7F] = d[1:7]
                if hdr and len(fr) == nf:
                    return b''.join(fr[i] for i in range(1, nf + 1))[:n]
        raise IOError('B1 %08X len %d: no complete reply' % (addr, n))

    def u32(self, addr):
        return struct.unpack('>I', self.read(addr, 4))[0]

    def poke(self, addr, val):
        assert addr % 4 == 0 and 0x8C000000 <= addr <= 0x8FFFFFFC
        i = (addr - 0x8C000000) >> 2
        self.send([0xB9, (i >> 16) & 255, (i >> 8) & 255, i & 255] + list(struct.pack('>I', val & 0xFFFFFFFF)))

    def write_block(self, addr, data, verify=True):
        data = data + bytes(-len(data) % 4)
        for k in range(0, len(data), 4):
            self.poke(addr + k, struct.unpack('>I', data[k:k + 4])[0])
        if verify:
            back = self.read(addr, len(data))
            if back != data:
                raise IOError('RAM write verify failed at %08X' % addr)


def next_slot():
    s = int(open(STATE).read().strip(), 16) if os.path.exists(STATE) else CODE_BASE
    open(STATE, 'w').write('%08X\n' % (s + 0x400))
    return s


def run(bus, src, wait=1.5, extra_check=None, tag='', table=None, res_words=0, done_magic=0xD0DE0000):
    """Assemble src at a fresh slot, write it, run as task 159, return result words."""
    slot = next_slot()
    code = sh4asm.assemble(src, slot)
    assert len(code) <= 0x400
    st = struct.unpack('>H', bus.read(TCB159 + 4, 2))[0]
    ent = bus.u32(TCB159 + ENTRY_OFF)
    if st != 0x10:
        raise RuntimeError('task 159 not dormant (state %04X) - refusing' % st)
    if ent not in (DOOM_ENTRY,) and not (CODE_BASE <= ent < CODE_BASE + 0x80000):
        raise RuntimeError('unexpected TCB159 entry %08X' % ent)
    if table is not None:
        bus.write_block(TAB, struct.pack('>%dI' % (len(table) + 1), len(table), *table))
    bus.poke(PROG, 0xFFFFFFFF)
    if res_words:
        bus.write_block(RES, bytes(4 * res_words), verify=False)
    bus.write_block(slot, code)
    trig_before = bus.read(TRIG, 8)
    log('RUN %s slot %08X len %d table=%s' % (tag, slot, len(code),
                                                 ' '.join('%08X' % a for a in (table or []))))
    bus.poke(TCB159 + ENTRY_OFF, slot)
    bus.send([0xB8, 0x5A, 0, 0, 0, 0, 0, 0])
    time.sleep(wait)
    try:
        prog = bus.u32(PROG)
        trig = bus.read(TRIG, 8)
    finally:
        bus.poke(TCB159 + ENTRY_OFF, DOOM_ENTRY)
    restored = bus.u32(TCB159 + ENTRY_OFF)
    st2 = struct.unpack('>H', bus.read(TCB159 + 4, 2))[0]
    res = list(struct.unpack('>%dI' % res_words, bus.read(RES, 4 * res_words))) if res_words else []
    log('  -> sta_tsk r0=%08X count %08X->%08X prog=%08X state=%04X entry restored %08X res=%s' % (
        struct.unpack('>I', trig[:4])[0], struct.unpack('>I', trig_before[4:8])[0], struct.unpack('>I', trig[4:8])[0],
        prog, st2, restored, ' '.join('%08X' % r for r in res)))
    return prog, res


P4READ = open(os.path.join(HERE, 'p4read.s')).read()


def p4read(bus, addrs, tag='p4read'):
    for a in addrs:
        log('PLAN P4 read %08X (%s)' % (a, tag))
    prog, res = run(bus, P4READ, table=list(addrs), res_words=len(addrs), tag=tag)
    if prog != (0xD0DE0000 | len(addrs)):
        print('INCOMPLETE: progress word %08X' % prog)
    for a, v in zip(addrs, res):
        print('%08X = %08X' % (a, v))
    return prog, res


def main(a):
    bus = Bus()
    try:
        if a[0] == 'r':
            addr = int(a[1], 16); n = int(a[2], 0) if len(a) > 2 else 4
            print(bus.read(addr, n).hex(' ', 4))
        elif a[0] == 'p4':
            p4read(bus, [int(x, 16) for x in a[1:]], tag='cli')
        else:
            print(__doc__)
    finally:
        bus.close()


if __name__ == '__main__':
    main(sys.argv[1:] or ['help'])
