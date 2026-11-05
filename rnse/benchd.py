r"""benchd.py -- hold the CowCAN and keep the RNS-E awake between bench commands.

  Server:  python -I benchd.py serve            (one adapter user: this process owns the CowCAN)
  Client:  python -I benchd.py sdstream run --slot 0 --gate none --mode verify --secs 20 --wrap 0
           python -I benchd.py srcsel 1         (r59 BA opcode: media select; polls the sink handle)
           python -I benchd.py sink | status
           python -I benchd.py quit

Commands run inside the server on one shared Bench (0x2C3/0x575 keep-alive never stops). Output comes back to the
client. Localhost TCP only. Runs one command at a time.
"""
import contextlib, importlib, io, os, socket, socketserver, struct, sys, threading, time, traceback
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
PORT = 47159


class _Writer(io.TextIOBase):
    def __init__(self, sock):
        self.sock = sock

    def write(self, s):
        try:
            self.sock.sendall(s.encode('utf-8', 'replace'))
        except OSError:
            pass
        return len(s)

    def flush(self):
        pass


def serve():
    import rnse_bench, layout as L
    bench = rnse_bench.Bench()
    real_close = bench.close
    bench.close = lambda: None                 # bench scripts must not drop the shared keep-alive
    rnse_bench.open_bench = lambda: bench
    import sdstream
    lock = threading.Lock()
    stop = threading.Event()

    def srcsel(args, out):
        kind = int(args[0])
        bench.bus.send([0xBA, kind, 0, 0, 0, 0, 0, 0])
        t = time.time()
        last = None
        while time.time() - t < float(args[1] if len(args) > 1 else 15):
            h = bench.u32(L.DMDL_HANDLE)
            if h != last:
                print('%.1f s sink %08X' % (time.time() - t, h), flush=True)
                last = h
            time.sleep(0.5)

    class H(socketserver.StreamRequestHandler):
        def handle(self):
            argv = self.rfile.readline().decode().strip().split('\x1f')
            w = _Writer(self.connection)
            if argv[0] == 'abort':             # no lock: a running program holds it. Asks the running RAM program to stop (CTL stop word)
                bench.stop()
                print('stop requested', file=w)
                return
            with lock, contextlib.redirect_stdout(w), contextlib.redirect_stderr(w):
                try:
                    if argv[0] == 'quit':
                        stop.set()
                    elif argv[0] == 'sdstream':
                        importlib.reload(sdstream)     # script edits apply without dropping the keep-alive
                        sdstream.main(argv[1:])
                    elif argv[0] == 'tasks':           # READY/RUN tasks (prio/id/state) + saved code words
                        base = L.TCB159 - 0x9F * 0x40
                        for tid in range(1, 0xA0):
                            tc = bench.read(base + tid * 0x40, 0x30)
                            stw = int.from_bytes(tc[4:6], 'big')
                            if stw & 3 or tid in (0x32, 0x50, 0x53, 0x14, 0x63):
                                tsp = int.from_bytes(tc[0x2C:0x30], 'big')
                                fr = bench.read((tsp & 0x1FFFFFFF) | 0x80000000, 0x100) if tsp else b''
                                w = [fr[i:i + 4].hex() for i in range(0, len(fr), 4) if fr[i] == 0x80 and 0x40 <= fr[i + 1] <= 0x7F]
                                print('prio %02X id %02X state %s sp %08X code %s' % (int.from_bytes(tc[2:4], 'big'), tid, tc[4:6].hex(), tsp, ' '.join(w[:12])))
                    elif argv[0] == 'sdbench':
                        import sdbench
                        importlib.reload(sdbench)
                        sdbench.main(argv[1:])
                    elif argv[0] == 'sdtime':
                        import sdtime
                        importlib.reload(sdtime)
                        sdtime.main(argv[1:])
                    elif argv[0] == 'reboot':          # RAM only: cold reset as proven in r52 (VBR=0, jmp A0000000); no flash
                        code = bytes.fromhex('0002' 'cbf0' '400e' 'e000' '402e' 'd101' '412b' '0009' 'a0000000')
                        st, ent = bench.task_state()
                        if st != 0x10:
                            print('task 159 busy, refusing')
                        else:
                            slot = bench.next_slot()
                            bench.write_block(slot, code)
                            bench.poke(L.TCB159 + L.ENTRY_OFF, slot)
                            bench.bus.send([0xB8, 0x5A, 0, 0, 0, 0, 0, 0])
                            print('reset sent from %08X' % slot)
                    elif argv[0] == 'poke':            # poke ADDR VALUE: one RAM word, then read it back
                        if int(argv[1], 16) == 0x8EAD1254:  # SD clock variant word: the one driver word allowed outside our region
                            bench.bus.poke(0x8EAD1254, int(argv[2], 16))
                        else:
                            bench.poke(int(argv[1], 16), int(argv[2], 16))
                        print('%s = %08X' % (argv[1], bench.u32(int(argv[1], 16))))
                    elif argv[0] == 'peek':            # peek ADDR LEN: hex dump via B1
                        d = bench.read(int(argv[1], 16), int(argv[2], 0))
                        print(d.hex())
                    elif argv[0] == 'srcsel':
                        srcsel(argv[1:], w)
                    elif argv[0] == 'sink':
                        print('sink %08X task %s' % (bench.u32(L.DMDL_HANDLE), [hex(x) for x in bench.task_state()]))
                    elif argv[0] == 'status':
                        print('alive; firmware %s' % (bench.firmware_check(),))
                    else:
                        print('unknown command %s' % argv[0])
                except SystemExit:
                    pass
                except BaseException:
                    traceback.print_exc(file=w)

    class S(socketserver.ThreadingTCPServer):
        allow_reuse_address = True
        daemon_threads = True

    srv = S(('127.0.0.1', PORT), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print('benchd: keep-alive running, listening on %d' % PORT, flush=True)
    try:
        while not stop.wait(1):
            pass
    finally:
        srv.shutdown()
        real_close()


def client(argv):
    s = socket.create_connection(('127.0.0.1', PORT), timeout=5)
    s.settimeout(None)
    s.sendall(('\x1f'.join(argv) + '\n').encode())
    while True:
        d = s.recv(4096)
        if not d:
            return
        sys.stdout.write(d.decode('utf-8', 'replace'))
        sys.stdout.flush()


if __name__ == '__main__':
    if sys.argv[1:] == ['serve']:
        serve()
    else:
        client(sys.argv[1:])
