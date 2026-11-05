r"""rnse_proto bench runner: loads a RAM program over CAN and runs it as task 159 (the soc_caps_v1 mechanism),
with the ignition keep-alive running the whole time. Used by tone48.py / sdprobe.py / sdbench.py.

Mechanism (unchanged from analysis/soc_caps_v1/probe.py, which is reused for the B1/B9 transport):
  B9 writes the image to its fixed window (0x8F700000/04000/08000), a 48-byte launcher (out/tramp.bin, patched with
  image start/end/entry) to a FRESH 0x40 slot at 0x8F710000+, CTL params to 0x8F720000;
  TCB159+0x20 := launcher; B8 5A (sta_tsk 159); TCB159+0x20 := 0x80B06000 (Doom entry) restored right away;
  then the status block 0x8F720100 is polled with B1.
Why not probe.next_slot(): its slot counter (analysis/soc_caps_v1/code_slot.txt) has walked past 0x8FAC0000 into
OSM-owned buffers (it reads 0x8FCEA800); this runner keeps to 0x8F700000..0x8FA00000 inside the documented free block.

Safety gates before any write: task 159 dormant (state 0x0010) with entry = Doom entry or one of our launchers;
the live firmware bytes at every called entry point match (layout.FW_CHECKS); VNMC literal identifies r57/r56b.
Every run is logged to bench_log.txt BEFORE it is sent.
Python: python -I  (64-bit, has gs_usb/python-can)
One adapter user at a time: stop vin888.py serve / doomd.py / adc_slider_ui first.
"""
import hashlib
import os, sys, time, struct, threading
import layout as L

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
OUT = os.path.join(HERE, 'out')
LOG = os.path.join(HERE, 'bench_log.txt')
SLOT_STATE = os.path.join(HERE, 'tramp_slot.txt')


def log(msg):
    with open(LOG, 'a') as f:
        f.write(time.strftime('%Y-%m-%d %H:%M:%S ') + msg + '\n')
        f.flush(); os.fsync(f.fileno())


def _probe():
    sys.path.insert(0, os.path.join(HERE, 'can'))
    import probe
    probe.LOG = LOG
    return probe


class Bench:
    """probe.Bus + background keep-alive 0x2C3 [07] @100 ms and 0x575 [03 00 00 00] @200 ms (screen/audio stay on)."""
    def __init__(self, keepalive=True):
        probe = _probe()
        self.bus = probe.Bus()
        self.lock = threading.RLock()
        self._stop = False
        self._t = None
        orig_send, orig_read1 = self.bus.send, self.bus._read1

        def send(data):
            with self.lock:
                orig_send(data)

        def read1(addr, n):
            with self.lock:
                return orig_read1(addr, n)
        self.bus.send, self.bus._read1 = send, read1
        if keepalive:
            self._t = threading.Thread(target=self._ka, daemon=True)
            self._t.start()
            time.sleep(0.3)

    def _id(self, cid, data):
        with self.lock:
            b = self.bus.b
            old = b.req_id
            b.req_id = cid
            try:
                b.send(bytes(data))
            finally:
                b.req_id = old

    def _ka(self):
        n = 0
        while not self._stop:
            try:
                self._id(0x2C3, [0x07])
                if n % 2 == 0:
                    self._id(0x575, [0x03, 0, 0, 0])
            except Exception as e:          # keep going; the main thread reports real failures
                print('keep-alive send failed: %s' % e)
            n += 1
            time.sleep(0.1)

    def close(self):
        self._stop = True
        if self._t:
            self._t.join()
        self.bus.close()

    # ---- helpers ----
    def read(self, a, n):
        return self.bus.read(a, n)

    def u32(self, a):
        return struct.unpack('>I', self.bus.read(a, 4))[0]

    def poke(self, a, v):
        assert L.REGION[0] <= a < L.REGION[1] or a == L.TCB159 + L.ENTRY_OFF, 'poke outside our region: %08X' % a
        self.bus.poke(a, v)

    def write_block(self, a, data, verify=True):
        assert L.REGION[0] <= a and a + len(data) <= L.REGION[1], 'write outside our region'
        self.bus.write_block(a, data, verify=verify)

    def firmware_check(self):
        bad = []
        for a, exp in L.FW_CHECKS:
            got = self.read(a, len(exp) // 2).hex()
            if got != exp:
                bad.append('%08X: %s (expected %s)' % (a, got, exp))
        vn = self.u32(L.VNMC_LIT)
        fw = {0x00010749: 'r57 (r56b + TV cal)', 0x00010449: 'r56b / r56 lineage'}.get(vn, 'UNKNOWN (VNMC %08X)' % vn)
        return fw, bad

    def image_resident(self, base, img, samples=24, size=32):
        """RAM persists until power-off: skip the slow upload when spread-out samples of the image already match."""
        step = max(1, (len(img) - size) // (samples - 1))
        for off in range(0, len(img) - size + 1, step):
            if self.read(base + off, size) != img[off:off + size]:
                return False
        return True

    def task_state(self):
        st = struct.unpack('>H', self.read(L.TCB159 + 4, 2))[0]
        ent = self.u32(L.TCB159 + L.ENTRY_OFF)
        return st, ent

    def next_slot(self):
        k = int(open(SLOT_STATE).read().strip()) if os.path.exists(SLOT_STATE) else 0
        open(SLOT_STATE, 'w').write('%d\n' % ((k + 1) % L.TRAMP_SLOTS))
        return L.TRAMP_BASE + L.TRAMP_SIZE * k

    # ---- fast load over SD (SDBOOT.txt) ----
    def _boot_resident(self):
        """Bootstrap present (its stamp matches this build)? Uploads it over CAN if not (once per power-on). True = uploaded now."""
        import sdfast
        code, stamp = sdfast.boot_image()
        if self.read(L.SDBOOT_STAMP, 16) == stamp:
            return False
        for a, exp in L.FW_SDBOOT:
            if self.read(a, len(exp) // 2).hex() != exp:
                raise sdfast.FastFail('firmware bytes differ at %08X' % a)
        t = time.monotonic()
        print('uploading the SD bootstrap (%d B) over CAN, once per power-on ...' % len(code))
        self.bus.poke(L.SDBOOT_STAMP, 0)
        self.write_block(L.SDBOOT_BASE, code)
        self.write_block(L.SDBOOT_STAMP, stamp)
        print('  done in %.1f s' % (time.monotonic() - t))
        log('SDBOOT uploaded %d B' % len(code))
        return True

    def start_fast(self, prog, params, tag=''):
        """SD load + start. Raises sdfast.FastFail with nothing started; the CAN path then takes over."""
        import sdfast
        t0 = time.monotonic()
        img = open(os.path.join(OUT, prog + '.bin'), 'rb').read()
        base, win = L.PROG_BASE[prog], L.window(prog)
        st, ent = self.task_state()
        if st != 0x10:
            raise RuntimeError('task 159 not dormant (state %04X): Doom or a previous program is running - refusing' % st)
        ours = L.TRAMP_BASE <= ent < L.TRAMP_BASE + L.TRAMP_SIZE * L.TRAMP_SLOTS or ent == L.SDBOOT_BASE
        if ent != L.DOOM_ENTRY and not ours:
            raise RuntimeError('unexpected TCB159 entry %08X - refusing' % ent)
        uploaded = self._boot_resident()
        load_id = int(time.time() * 1000) & 0x7FFFFFFF
        ctl = L.ctl_words(params)
        ms_usb = sdfast.send_image(load_id, base, img, base, win, ctl)
        stamp = os.path.join(OUT, '.%s.uploaded' % prog)
        try:
            os.remove(stamp)                                        # the image in RAM is about to change
        except OSError:
            pass
        log('PLAN %s %s SD load %08X+%d id %08X params %s' % (prog, tag, base, len(img), load_id, ' '.join('%X' % p for p in params)))
        entry = L.SDBOOT_BASE
        if uploaded:                                                # first run of a new bootstrap: through a fresh launcher (icache)
            tr = bytearray(open(os.path.join(OUT, 'tramp.bin'), 'rb').read())
            struct.pack_into('>III', tr, 0x20, L.SDBOOT_BASE, L.SDBOOT_BASE + 0x1000, L.SDBOOT_BASE)
            entry = self.next_slot()
            self.write_block(entry, bytes(tr))
        self.poke(L.STAT_BASE, 0)
        self.poke(L.SDBOOT_ARG, L.SDBOOT_LBA)
        self.bus.poke(L.TCB159 + L.ENTRY_OFF, entry)
        try:
            self.bus.send([0xB8, 0x5A, 0, 0, 0, 0, 0, 0])
            line = sdfast.wait_served(1.5)                          # the Pi saw the unit read every sector
        except sdfast.FastFail as e:
            line = 'no reply (%s)' % e
        finally:
            self.bus.poke(L.TCB159 + L.ENTRY_OFF, L.DOOM_ENTRY)
        res = None
        end = time.monotonic() + 1.0
        while time.monotonic() < end:                               # the result word is written before the jump
            res = struct.unpack('>8I', self.read(L.SDBOOT_RES, 32))
            if res[0] == sdfast.RES_MAGIC and res[1] == load_id and res[2] >= 2:
                break
            time.sleep(0.02)
        if not (res and res[0] == sdfast.RES_MAGIC and res[1] == load_id and res[2] == sdfast.CODE_JUMP):
            code = res[2] if res else 0
            msg = 'bootstrap: %s (code %d, id %08X want %08X, tries %d, Pi said: %s)' % (
                sdfast.CODES.get(code, '?'), code, res[1] if res else 0, load_id, res[4] if res else 0, line)
            log('  SD load failed: ' + msg)
            raise sdfast.FastFail(msg)
        open(stamp, 'w').write(hashlib.sha256(img).hexdigest())
        rest = self.u32(L.TCB159 + L.ENTRY_OFF)
        total = time.monotonic() - t0
        print('SD load: %s %d B, %.2f s total (to Teensy %.0f ms, bootstrap read+CRC %d ms, %d read calls%s)' % (
            prog, len(img), total, ms_usb, res[3], res[4], ', bootstrap uploaded' if uploaded else ''))
        log('RUN %s via SD id %08X in %.2f s; entry restored %08X; %s' % (prog, load_id, total, rest, line))
        if rest != L.DOOM_ENTRY:
            print('WARNING: TCB159 entry reads %08X after restore (expected %08X)' % (rest, L.DOOM_ENTRY))
        return total

    def start(self, prog, params, tag='', fast=True):
        """Load + launch. SD first if the program allows it (bootstrap, CRC, all gates in the bootstrap); CAN on any failure.
        Returns once task 159 has been started and the entry restored."""
        if fast and params and params[0] == 0 and not os.environ.get('RNSE_NO_SD_LOAD') and prog == 'sdstream':
            import sdfast
            try:
                self.start_fast(prog, params, tag)
                return
            except sdfast.FastFail as e:
                print('SD load not used: %s; falling back to CAN' % e)
        fw, bad = self.firmware_check()
        print('firmware: %s' % fw)
        if bad:
            raise RuntimeError('firmware bytes differ at called entry points - refusing:\n  ' + '\n  '.join(bad))
        st, ent = self.task_state()
        ours = L.TRAMP_BASE <= ent < L.TRAMP_BASE + L.TRAMP_SIZE * L.TRAMP_SLOTS or ent == L.SDBOOT_BASE
        if st != 0x10:
            raise RuntimeError('task 159 not dormant (state %04X): Doom or a previous program is running - refusing' % st)
        if ent != L.DOOM_ENTRY and not ours:
            raise RuntimeError('unexpected TCB159 entry %08X - refusing' % ent)
        img = open(os.path.join(OUT, prog + '.bin'), 'rb').read()
        base = L.PROG_BASE[prog]
        win = L.window(prog)
        assert len(img) <= win
        tr = bytearray(open(os.path.join(OUT, 'tramp.bin'), 'rb').read())
        assert len(tr) == L.TRAMP_SIZE
        struct.pack_into('>III', tr, 0x20, base, base + win, base)
        slot = self.next_slot()
        ctl = L.ctl_words(params)
        log('PLAN %s %s image %08X+%d launcher %08X params %s' % (
            prog, tag, base, len(img), slot, ' '.join('%X' % p for p in params)))
        t = time.monotonic()
        print('writing %s (%d B) + launcher + params over CAN (about %.0f s incl. verify) ...' % (
            prog, len(img), len(img) / 160 + 3))
        # the spot check alone can't see a small rebuild, so also require the same file as the last upload
        stamp = os.path.join(OUT, '.%s.uploaded' % prog)
        sha = hashlib.sha256(img).hexdigest()
        same = os.path.exists(stamp) and open(stamp).read().strip() == sha
        if same and self.image_resident(base, img):
            print('  image already resident (same file as last upload, spot check passed), not re-sending it')
        else:
            self.write_block(base, img)
            open(stamp, 'w').write(sha)
        self.write_block(L.CTL_BASE, ctl)
        self.poke(L.STAT_BASE, 0)                                   # invalidate the old status magic
        self.poke(L.STAT_BASE + 28, 0)                              # and the old done word
        self.write_block(slot, bytes(tr))
        print('  loaded in %.1f s' % (time.monotonic() - t))
        log('RUN %s launcher %08X' % (prog, slot))
        self.poke(L.TCB159 + L.ENTRY_OFF, slot)
        try:
            self.bus.send([0xB8, 0x5A, 0, 0, 0, 0, 0, 0])
            time.sleep(0.3)
        finally:
            self.poke(L.TCB159 + L.ENTRY_OFF, L.DOOM_ENTRY)
        rest = self.u32(L.TCB159 + L.ENTRY_OFF)
        log('  started; entry restored %08X' % rest)
        if rest != L.DOOM_ENTRY:
            print('WARNING: TCB159 entry reads %08X after restore (expected %08X)' % (rest, L.DOOM_ENTRY))

    def stop(self):
        self.poke(L.CTL_BASE + 4, 1)
        log('STOP requested (CTL stop word)')

    def poll(self, prog, timeout, every=1.0, show=None, nstat=0x40):
        """Poll the status block until the done word appears. Ctrl-C asks the program to stop and keeps polling."""
        end = time.monotonic() + timeout
        stopped = False
        last = None
        while True:
            try:
                st = self.read(L.STAT_BASE, nstat)
                h = L.header(st)
                if h['valid'] and h['prog'] == L.PROG_ID[prog]:
                    line = show(st) if show else 'stage %X err %d hb %d' % (h['stage'], h['err'], h['hb'])
                    if line != last:
                        print('  ' + line)
                        last = line
                    if h['done']:
                        return True
                elif not h['valid']:
                    print('  (status block not written yet)')
                if time.monotonic() > end:
                    print('TIMEOUT waiting for %s; asking it to stop' % prog)
                    if not stopped:
                        self.stop(); stopped = True; end = time.monotonic() + 10
                    else:
                        return False
                time.sleep(every)
            except KeyboardInterrupt:
                if stopped:
                    raise
                print('Ctrl-C: asking the program to stop cleanly (Ctrl-C again to abandon polling)')
                self.stop(); stopped = True; end = time.monotonic() + 10

    def final(self, nbytes):
        st = self.read(L.STAT_BASE, nbytes)
        s, ent = self.task_state()
        log('  done: status %s task159 state %04X entry %08X' % (st[:32].hex(), s, ent))
        if s != 0x10:
            print('NOTE: task 159 state %04X (expected 0010 dormant after ext_tsk)' % s)
        return st


def open_bench():
    return Bench()
