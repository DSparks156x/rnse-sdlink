#!/usr/bin/env python3
"""Read the installed SDLink module sector (107) CRC32 as a firmware fingerprint.

Uses only the existing B1 read-memory command (r44+ replies on 0x7B6). No
upload, flash write, reset, or stream-control command is sent. Reading all
128 KiB takes longer than the RAM-stub CRC operation in RNS-E-Hudiy.

Pi / SocketCAN (bring up can0 first):
    python3 rnse/can/decoder_check.py
Windows / CowCAN (same USB dependencies as cowcan_memread.py):
    python -I rnse/can/decoder_check.py --transport cowcan
Machine-readable result:
    ... decoder_check.py --json

Exit status: 0 = read succeeded, 2 = error,
130 = interrupted. A CRC identifies the whole sector, including unused tail
bytes; it does not by itself identify a source revision or compatibility.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import zlib

# Isolated Python (-I) omits the script directory from the import path.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import pi_memread_block as pm  # noqa: E402

DECODER_SECTOR = 107   # SDLink module (was 96 on r61..r64 units with the old module)
SECTOR_SIZE = 0x20000
DECODER_ADDRESS = 0x80000000 + DECODER_SECTOR * SECTOR_SIZE


class DecoderReadError(Exception):
    pass


def read_decoder_crc(bus, timeout=1.0, retries=3):
    """CRC32 of every byte of the module sector; incomplete reads never produce a CRC."""
    crc = 0
    for offset in range(0, SECTOR_SIZE, pm.MAX_LEN):
        address = DECODER_ADDRESS + offset
        for attempt in range(retries + 1):
            try:
                data = pm.read_chunk(bus, address, pm.MAX_LEN, timeout)
                if len(data) != pm.MAX_LEN:
                    raise ValueError('short read (%d/%d bytes)' % (len(data), pm.MAX_LEN))
                break
            except OSError as error:
                # Firmware range/length rejection is definitive, not a lost reply.
                raise DecoderReadError('0x%08X: %s' % (address, error)) from error
            except ValueError as error:
                if attempt == retries:
                    raise DecoderReadError('0x%08X after %d attempt(s): %s' %
                                           (address, retries + 1, error)) from error
        crc = zlib.crc32(data, crc)
    return crc


def open_bus(args):
    if args.transport == 'cowcan':
        # Keep USB packages optional on the Pi and for offline help/tests.
        from cowcan_memread import CowBus
        return CowBus(args.baud)
    return pm.Bus(args.interface)


def main(argv=None, bus_factory=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--transport', choices=('socketcan', 'cowcan'), default='socketcan')
    parser.add_argument('--interface', default='can0', help='SocketCAN interface (default can0)')
    parser.add_argument('--baud', type=int, default=100000, help='CowCAN bit rate (default 100000)')
    parser.add_argument('--timeout', type=float, default=1.0, help='seconds per B1 request (default 1)')
    parser.add_argument('--retries', type=int, default=3, help='retries per incomplete request (default 3)')
    parser.add_argument('--json', action='store_true', help='emit a JSON result for callers')
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and greater than zero')
    if args.retries < 0:
        parser.error('--retries must be at least zero')
    if args.baud <= 0:
        parser.error('--baud must be greater than zero')

    try:
        bus = (bus_factory or open_bus)(args)
        try:
            crc = read_decoder_crc(bus, args.timeout, args.retries)
        finally:
            bus.close()
    except KeyboardInterrupt:
        print('Decoder check interrupted; no complete sector CRC available.', file=sys.stderr)
        return 130
    except (DecoderReadError, OSError, RuntimeError, ImportError, ValueError) as error:
        message = str(error)
        print(json.dumps({'error': message}) if args.json else 'Decoder check failed: ' + message,
              file=sys.stderr)
        return 2

    result = {'sector': DECODER_SECTOR, 'address': '0x%08X' % DECODER_ADDRESS,
              'size': SECTOR_SIZE, 'crc32': '%08X' % crc}
    if args.json:
        print(json.dumps(result))
    else:
        print('SDLink decoder sector %d: CRC32 %08X (%d bytes).' % (DECODER_SECTOR, crc, SECTOR_SIZE))
    return 0


if __name__ == '__main__':
    sys.exit(main())
