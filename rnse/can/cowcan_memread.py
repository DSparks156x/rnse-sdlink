"""Dump an RNS-E address range through the CowCAN-FT USB adapter (gs_usb/candleLight protocol, USB 1D50:606F)
using the r43+ B1 read-memory command. The RNS-E bus is 100 kbit/s fault-tolerant CAN, which J2534 high-speed
channels (Scanmatik) cannot see.

Same protocol, options and resume behaviour as pi_memread_block.py (which must sit next to this file); only
the CAN transport differs. Needs python-can's companion packages in the 64-bit Python:
    gs_usb, pyusb, libusb-package

    python -I tools\\cowcan_memread.py ...

Extra options (everything else is passed through to pi_memread_block):
    --baud N         CAN bit rate (default 100000)
    --sniff SECS     just print all bus traffic for SECS seconds

Examples:
    ... cowcan_memread.py --sniff 3
    ... cowcan_memread.py 0x80400000 --length 1 --out t.bin -v
    ... cowcan_memread.py 0x80000000 --end 0x80400200 --out below_app.bin --resume
"""
import argparse
import os
import sys
import time

import libusb_package
import usb.core
from gs_usb import gs_usb as gsmod
from gs_usb.constants import CAN_ERR_FLAG, GS_CAN_MODE_NORMAL
from gs_usb.gs_usb_frame import GS_USB_NONE_ECHO_ID, GsUsbFrame

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # -I drops the script dir
import pi_memread_block as pm                                        # noqa: E402

COWCAN = (0x1D50, 0x606F)

# gs_usb looks for libusb on PATH and has a fixed VID/PID allow-list without the CowCAN
_backend = libusb_package.get_libusb1_backend()
gsmod.libusb1.get_backend = lambda *a, **k: _backend
_stock_match = gsmod.GsUsb.is_gs_usb_device
gsmod.GsUsb.is_gs_usb_device = staticmethod(lambda d: (d.idVendor, d.idProduct) == COWCAN or _stock_match(d))


def set_bitrate(dev, baud):
    if dev.set_bitrate(baud):
        return
    fclk = dev.device_capability.fclk_can            # clock gs_usb has no table for: 87.5 % sample point
    for tq in (16, 20, 10, 8, 24, 25, 12):
        brp, rem = divmod(fclk, baud * tq)
        if rem == 0 and 1 <= brp <= 1024:
            ph2 = max(1, round(tq * 0.125))
            dev.set_timing(1, tq - 2 - ph2, ph2, 1, brp)     # sync(1) + prop(1) + ph1 + ph2 = tq
            return
    raise OSError('no bit timing for %d bit/s at fclk %d Hz' % (baud, fclk))


class CowBus:
    def __init__(self, baud, reply_id=pm.REPLY_ID, req_id=pm.REQ_ID):
        devs = gsmod.GsUsb.scan()
        if not devs:
            raise OSError('no CowCAN / gs_usb adapter found (USB 1D50:606F)')
        self.dev = devs[0]
        set_bitrate(self.dev, baud)
        self.dev.start(GS_CAN_MODE_NORMAL)           # no HW timestamps: 20-byte frames
        self.reply_id, self.req_id = reply_id, req_id
        self.rx = GsUsbFrame()

    def send(self, payload):
        # candleLight echoes every TX frame on the IN endpoint once it has actually hit the CAN wire.
        # Waiting for that echo after each send is exact flow control: it paces us to bus throughput
        # (so a sustained B3 flood never overruns the adapter TX FIFO) and frees the device's frame pool.
        frame = GsUsbFrame(can_id=self.req_id, data=bytes(payload))
        for _ in range(8):
            try:
                self.dev.send(frame)
                break
            except usb.core.USBError:            # TX FIFO full / stall: drain and retry
                self._pump()
                time.sleep(0.002)
        else:
            self.dev.send(frame)                 # last try: let a genuine failure raise
        end = time.monotonic() + 0.05
        while time.monotonic() < end:            # consume our TX echo (discard any RX seen meanwhile)
            if self.dev.read(self.rx, 2) and self.rx.echo_id != GS_USB_NONE_ECHO_ID:
                return

    def _pump(self):
        """Discard any immediately-available IN frames (TX echoes / unsolicited RX) to free device buffers."""
        for _ in range(64):
            if not self.dev.read(self.rx, 1):
                return

    def recv(self, timeout):
        end = time.monotonic() + timeout
        while True:
            ms = max(1, int((end - time.monotonic()) * 1000))
            if not self.dev.read(self.rx, ms):
                return None
            f = self.rx
            if f.echo_id == GS_USB_NONE_ECHO_ID and not f.can_id & CAN_ERR_FLAG:
                cid = f.can_id & 0x7FF
                if self.reply_id is None or cid == self.reply_id:
                    return cid, bytes(f.data[:f.can_dlc])
            if time.monotonic() >= end:               # echoes, error frames, other IDs
                return None

    def drain(self):
        while self.dev.read(self.rx, 1):
            pass

    def close(self):
        self.dev.stop()


def sniff(baud, secs):
    bus = CowBus(baud, reply_id=None)
    end, seen = time.monotonic() + secs, {}
    while time.monotonic() < end:
        m = bus.recv(end - time.monotonic())
        if m:
            seen[m[0]] = seen.get(m[0], 0) + 1
            print('%03X  %s' % (m[0], pm._hex(m[1])))
    bus.close()
    print('%d frame(s), ids: %s' % (sum(seen.values()),
                                   ' '.join('%03X x%d' % kv for kv in sorted(seen.items())) or 'none'))
    return 0 if seen else 1


def main(argv=None):
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument('--baud', type=int, default=100000)
    p.add_argument('--sniff', type=float, metavar='SECS')
    p.add_argument('-h', '--help', action='store_true')
    args, rest = p.parse_known_args(argv)
    if args.help:
        print(__doc__)
        rest = ['--help']
    if args.sniff:
        return sniff(args.baud, args.sniff)
    pm.Bus = lambda _interface: CowBus(args.baud)
    return pm.main(rest)


if __name__ == '__main__':
    sys.exit(main())
