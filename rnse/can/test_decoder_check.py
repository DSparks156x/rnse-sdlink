"""Offline B1/CRC/version-fingerprint checks. Run: python -I rnse/can/test_decoder_check.py"""
import contextlib
import io
import json
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parent))
import decoder_check as dc


class MemoryBus:
    """Replies to real B1 requests using deterministic emulated flash contents."""
    def __init__(self, image):
        self.image = image
        self.requests = []
        self.frames = []
        self.closed = False

    def drain(self):
        self.frames.clear()

    def send(self, payload):
        self.assert_request(payload)
        address = struct.unpack('>I', payload[1:5])[0]
        length = payload[5]
        self.requests.append((address, length))
        offset = address - dc.DECODER_ADDRESS
        data = self.image[offset:offset + length]
        count = (length + 5) // 6
        self.frames.append(bytes((0x80, 0)) + payload[1:6] + bytes((count,)))
        for n in range(count):
            block = data[n * 6:(n + 1) * 6].ljust(6, b'\x00')
            self.frames.append(bytes((0x81 + n,)) + block + bytes((sum(block) & 255,)))

    @staticmethod
    def assert_request(payload):
        if len(payload) != 8 or payload[0] != 0xB1 or payload[6:] != b'\x00\x00':
            raise AssertionError('only B1 read-memory requests are allowed')

    def recv(self, timeout):
        return (dc.pm.REPLY_ID, self.frames.pop(0)) if self.frames else None

    def close(self):
        self.closed = True


class DecoderCheckTests(unittest.TestCase):
    image = bytes(range(256)) * (dc.SECTOR_SIZE // 256)

    def run_cli(self, args, bus):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            status = dc.main(args, bus_factory=lambda _: bus)
        return status, out.getvalue(), err.getvalue()

    def test_complete_sector_through_real_b1_parser(self):
        bus = MemoryBus(self.image)
        status, out, err = self.run_cli(['--json'], bus)
        self.assertEqual(status, 0, err)
        result = json.loads(out)
        self.assertEqual(result['crc32'], '%08X' % zlib.crc32(self.image))
        self.assertEqual((result['sector'], result['size']), (107, 131072))
        self.assertNotIn('match', result)
        self.assertEqual(bus.requests, [(dc.DECODER_ADDRESS + n, 64)
                                        for n in range(0, dc.SECTOR_SIZE, 64)])
        self.assertTrue(bus.closed)

    def test_incomplete_read_retries_then_fails_without_crc(self):
        bus = MemoryBus(self.image)
        with patch.object(dc.pm, 'read_chunk', side_effect=ValueError('incomplete')) as read:
            status, out, err = self.run_cli(['--retries', '2', '--json'], bus)
        self.assertEqual(status, 2)
        self.assertEqual(read.call_count, 3)
        self.assertEqual(out, '')
        self.assertIn('0x80D60000', json.loads(err)['error'])
        self.assertTrue(bus.closed)

    def test_short_read_never_produces_a_fingerprint(self):
        bus = MemoryBus(self.image)
        with patch.object(dc.pm, 'read_chunk', return_value=b'\x00' * 63) as read:
            status, out, err = self.run_cli(['--retries', '1', '--json'], bus)
        self.assertEqual(status, 2)
        self.assertEqual(read.call_count, 2)
        self.assertEqual(out, '')
        self.assertIn('short read (63/64 bytes)', json.loads(err)['error'])
        self.assertTrue(bus.closed)

    def test_retry_recovers_without_skipping_or_double_counting(self):
        bus = MemoryBus(self.image)
        original = dc.pm.read_chunk
        calls = []

        def read(*args):
            calls.append(args[1])
            if len(calls) == 1:
                raise ValueError('dropped reply')
            return original(*args)

        with patch.object(dc.pm, 'read_chunk', side_effect=read):
            self.assertEqual(dc.read_decoder_crc(bus), zlib.crc32(self.image))
        self.assertEqual(calls[:2], [dc.DECODER_ADDRESS] * 2)

    def test_hard_rejection_and_interrupt_close_bus(self):
        for exception, expected in ((OSError('range error'), 2), (KeyboardInterrupt(), 130)):
            with self.subTest(exception=type(exception).__name__):
                bus = MemoryBus(self.image)
                with patch.object(dc.pm, 'read_chunk', side_effect=exception) as read:
                    status, out, err = self.run_cli([], bus)
                self.assertEqual(status, expected)
                self.assertEqual(read.call_count, 1)
                self.assertEqual(out, '')
                self.assertTrue(bus.closed)


if __name__ == '__main__':
    unittest.main()
