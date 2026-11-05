#!/usr/bin/env python3
"""Dump a whole address range from the RNS-E over CAN using the r43 B1 read-memory command.

Standalone: Python 3 + Linux SocketCAN only, no other project files needed. Run it on the Pi.

Request  0x7B0: B1 a3 a2 a1 a0 LEN 00 00            (addr big-endian, LEN 1..64)
Reply    0x7B6: header (80) st a3 a2 a1 a0 LEN NF   st: 00 ok / 01 range / 02 length ; NF = ceil(LEN/6)
                data   (80|seq) d0 d1 d2 d3 d4 d5 CK  seq 1..NF, 6 bytes each, last zero-padded, CK=(d0+..+d5)&FF
                (r44+ moved every reply from 0x7B1 to 0x7B6; 0x7B1 collided with stock 0x461 in the CAN parity hash and was dropped)

Readable ranges (the whole span must sit inside ONE, and not cross a 64 MB boundary):
    80000000-83FFFFFF  flash
    8C000000-8FFFFFFF  RAM
    AC000000-AFFFFFFF  RAM (uncached alias)
    B8000000-BBFFFFFF  runtime/SRAM (r51+ firmware only: restart count B800001A, reset cause B800001C, mode word B8000000)

Examples (bring up the CAN interface first, e.g.  sudo ip link set can0 up type can bitrate 500000):
    # the 512-byte block just below the app image
    python3 pi_memread_block.py 0x80400000 --length 0x200 --out desc.bin

    # the whole low flash region below the app (4 MB) -- slow, resumable
    python3 pi_memread_block.py 0x80000000 --end 0x80400000 --out lowflash.bin

    # re-run the same command to resume an interrupted dump
    python3 pi_memread_block.py 0x80000000 --end 0x80400000 --out lowflash.bin --resume
"""
import argparse
import os
import signal
import socket
import struct
import sys
import time

REQ_ID = 0x7B0
REPLY_ID = 0x7B6
OPCODE = 0xB1
MAX_LEN = 64
PER_FRAME = 6
CAN_FMT = '=IB3x8s'           # SocketCAN frame: id(u32) dlc(u8) pad(3) data(8)
REGIONS = ('80000000-83FFFFFF flash', '8C000000-8FFFFFFF RAM', 'AC000000-AFFFFFFF RAM uncached',
           'B8000000-BBFFFFFF runtime/SRAM (r51+: restart count B800001A, reset cause B800001C, mode word B8000000)')
# region index (addr>>26) of each readable 64 MB window the firmware B1 cave allows.
# r51+ adds 0x2E (0xB8000000..0xBBFFFFFF); older firmware rejects it with a range-error status (no hang).
READABLE_REGIONS = (0x20, 0x23, 0x2B, 0x2E)


def check_span(addr, length):
    """The firmware's allow-list, mirrored client-side. Returns an error string or None."""
    if length < 1:
        return 'length must be at least 1'
    last = addr + length - 1
    if addr < 0 or last > 0xFFFFFFFF:
        return '0x%08X + %d wraps past FFFFFFFF' % (addr, length)
    # one 64 MB region (bits >>26 equal), and that region must be readable
    if addr >> 26 != last >> 26 or (addr >> 26) not in READABLE_REGIONS:
        return '0x%08X..0x%08X is not inside one readable region: %s' % (addr, last, ', '.join(REGIONS))
    return None


def request(addr, length):
    return bytes((OPCODE, addr >> 24 & 255, addr >> 16 & 255, addr >> 8 & 255, addr & 255, length, 0, 0))


class Bus:
    def __init__(self, interface):
        if not hasattr(socket, 'AF_CAN'):
            raise RuntimeError('SocketCAN needs Linux; run this on the Pi')
        self.s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, struct.pack('=II', REPLY_ID, 0x7FF))
        self.s.bind((interface,))

    def send(self, payload):
        frame = struct.pack(CAN_FMT, REQ_ID, 8, payload)
        if self.s.send(frame) != len(frame):
            raise OSError('SocketCAN did not accept the whole frame')

    def recv(self, timeout):
        self.s.settimeout(max(timeout, 0.001))
        try:
            can_id, dlc, data = struct.unpack(CAN_FMT, self.s.recv(16))
        except socket.timeout:
            return None
        return can_id & 0x7FF, data[:dlc]

    def drain(self):
        while self.recv(0.001) is not None:
            pass

    def close(self):
        self.s.close()


def _hex(b):
    return ' '.join('%02X' % x for x in b)


def read_chunk(bus, addr, length, timeout, verbose=False):
    """One B1 request. Returns the bytes, or raises IOError (range/length) / ValueError (incomplete)."""
    bus.drain()
    req = request(addr, length)
    bus.send(req)
    if verbose:
        sys.stderr.write('\n  TX 7B0: %s\n' % _hex(req))
    nf_expected = -(-length // PER_FRAME)
    status = None
    nf = 0
    frames = {}
    saw_any = False
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        m = bus.recv(deadline - time.monotonic())
        if m is None:
            break
        rid, data = m
        if verbose:
            sys.stderr.write('  RX %03X: %s\n' % (rid, _hex(data)))
        saw_any = True
        if rid != REPLY_ID or len(data) != 8 or data[0] & 0xF0 != 0x80:
            continue
        seq = data[0] & 0x0F
        if seq == 0:                                   # header
            if struct.unpack('>I', data[2:6])[0] != addr or data[6] != length:
                if verbose:
                    sys.stderr.write('     (header for another request, ignored)\n')
                continue                               # stale reply to an earlier request
            status, nf = data[1], (data[7] if data[1] == 0 else 0)
            if status == 1:
                raise IOError('range error (address not readable)')
            if status == 2:
                raise IOError('length error')
        elif status == 0 and 1 <= seq <= nf and seq not in frames:
            if sum(data[1:7]) & 0xFF == data[7]:       # checksum ok
                frames[seq] = data[1:7]
            elif verbose:
                sys.stderr.write('     (bad checksum, dropped)\n')
        if status == 0 and nf == nf_expected and len(frames) == nf:
            return b''.join(frames[s] for s in range(1, nf + 1))[:length]
    if verbose and not saw_any:
        sys.stderr.write('  (no frames at all on 7B6 within %.1fs)\n' % timeout)
    raise ValueError('incomplete (status %s, have %d/%d frames)' % (status, len(frames), nf_expected))


def human(n):
    for unit in ('B', 'KB', 'MB'):
        if n < 1024 or unit == 'MB':
            return '%.1f %s' % (n, unit) if unit != 'B' else '%d B' % n
        n /= 1024.0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('addr', help='start address, e.g. 0x80000000')
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument('--length', help='number of bytes to read, e.g. 0x200 or 4096')
    g.add_argument('--end', help='exclusive end address, e.g. 0x80400000')
    p.add_argument('--out', required=True, help='output file')
    p.add_argument('--interface', default='can0')
    p.add_argument('--chunk', type=lambda v: int(v, 0), default=MAX_LEN, help='bytes per request, 1..64 (default 64)')
    p.add_argument('--timeout', type=float, default=1.0, help='seconds to wait per request (default 1.0)')
    p.add_argument('--retries', type=int, default=3, help='retries per chunk before giving up (default 3)')
    p.add_argument('--resume', action='store_true', help='continue a previous dump: skip the bytes already in --out')
    p.add_argument('--skip-errors', action='store_true', help='on a chunk that keeps failing, write 00 fill and carry on')
    p.add_argument('-v', '--verbose', action='store_true', help='print every frame sent and received (to stderr) for debugging')
    args = p.parse_args(argv)

    addr = int(args.addr, 0)
    total = int(args.length, 0) if args.length is not None else int(args.end, 0) - addr
    if not 1 <= args.chunk <= MAX_LEN:
        p.error('--chunk must be 1..64')
    if total < 1:
        p.error('nothing to read (length/end <= start)')
    err = check_span(addr, total)
    if err:
        p.error(err)

    done = 0
    mode = 'r+b'
    if args.resume and os.path.exists(args.out):
        done = (os.path.getsize(args.out) // args.chunk) * args.chunk     # restart at a chunk boundary
        print('resuming: %s already holds %s, continuing from 0x%08X' % (args.out, human(done), addr + done))
    elif not os.path.exists(args.out):
        mode = 'wb'

    bus = Bus(args.interface)
    stop = {'now': False}
    signal.signal(signal.SIGINT, lambda *a: stop.update(now=True))

    started = time.monotonic()
    failures = []
    with open(args.out, mode) as f:
        f.seek(done)
        off = done
        while off < total and not stop['now']:
            n = min(args.chunk, total - off)
            a = addr + off
            data = None
            for attempt in range(args.retries + 1):
                try:
                    data = read_chunk(bus, a, n, args.timeout, args.verbose)
                    break
                except IOError as e:                        # range/length: a hard no from the firmware
                    print('\n0x%08X+%d: %s' % (a, n, e))
                    data = None
                    break
                except ValueError:
                    continue                                 # dropped frames: retry
            if data is None:
                failures.append(a)
                if not args.skip_errors:
                    print('\naborting at 0x%08X after %d tries; re-run with --resume to retry, or --skip-errors to fill.' %
                          (a, args.retries + 1))
                    break
                data = bytes(n)
            f.write(data)
            f.flush()
            off += n
            if off % (64 * 1024) < args.chunk or off >= total:
                rate = off - done
                secs = time.monotonic() - started
                speed = (rate / secs) if secs > 0 else 0
                eta = ((total - off) / speed) if speed > 0 else 0
                sys.stdout.write('\r0x%08X  %s / %s  %5.1f%%  %6.0f B/s  ETA %4.0fs ' %
                                 (addr + off, human(off), human(total), 100.0 * off / total, speed, eta))
                sys.stdout.flush()
    bus.close()
    print()
    if stop['now']:
        print('stopped by Ctrl-C at 0x%08X; %s saved. Re-run with --resume.' % (addr + off, human(off)))
    if failures:
        print('%d chunk(s) failed, first at 0x%08X.' % (len(failures), failures[0]) +
              ('' if args.skip_errors else ' Re-run with --resume to continue.'))
    print('wrote %s to %s' % (human(off), args.out))
    return 1 if (failures and not args.skip_errors) else 0


if __name__ == '__main__':
    sys.exit(main())
