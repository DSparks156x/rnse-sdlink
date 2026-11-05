"""sdemu_ref.py -- Python reference for everything the Teensy firmware generates.

* expected selftest values (built from ../../link/sd_wire.py, the spec's reference encoder)
* the synthetic FAT16 1 GiB volume (mirror of sdproto.cpp gen_sector) and the per-LBA test pattern
* pattern checker for sectors read back on the RNS-E / Pi side

No third-party deps. Run directly for an offline consistency check:  <python> -I sdemu_ref.py
"""
import os, struct, sys, zlib

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "..", "link")))
import sd_wire as W  # noqa: E402

RCA = 0x5A01
VOL_TOTAL = 2097152
PART_LBA = 2048
PART_SECTORS = VOL_TOTAL - PART_LBA
RESERVED = 1760
FATSZ = 128
SPC = 64
LBA_FAT1 = PART_LBA + RESERVED
LBA_FAT2 = LBA_FAT1 + FATSZ
LBA_ROOT = LBA_FAT2 + FATSZ
LBA_DATA = LBA_ROOT + 32
LBA_BIN = LBA_DATA + SPC
CLUSTERS = 32704
LBA_INFO = 0x40000
MP3_CLUSTERS = 32
MP3_CLUS1 = CLUSTERS + 2 - MP3_CLUSTERS
LBA_MP3 = LBA_DATA + (MP3_CLUS1 - 2) * SPC
MP3_FRAME = 384
MP3_SIZE = (MP3_CLUSTERS * SPC * 512 // MP3_FRAME) * MP3_FRAME

TEST_TXT = (
    b"RNS-E SD emulator test volume (Teensy 4.1 sdemu bring-up firmware).\r\n"
    b"Every data sector from LBA 4160 up (file RSAVLINK.BIN) carries a per-LBA pattern:\r\n"
    b"big-endian u32 words: 'RSPT', LBA, ~LBA, 1, then 124 xorshift32 words seeded with\r\n"
    b"(LBA * 0x9E3779B9) ^ 0xA5A5A5A5 (x ^= x<<13; x ^= x>>17; x ^= x<<5). LBA 0x40000 = info.\r\n")

M32 = 0xFFFFFFFF


def pattern_sector(lba):
    out = [0x52535054, lba & M32, (~lba) & M32, 1]
    x = ((lba * 0x9E3779B9) & M32) ^ 0xA5A5A5A5
    if x == 0:
        x = 1
    for _ in range(124):
        x ^= (x << 13) & M32
        x ^= x >> 17
        x ^= (x << 5) & M32
        out.append(x)
    return struct.pack(">128I", *out)


def check_sector(lba, data):
    """True if data (512 B) is the firmware's pattern for this LBA (data area only)."""
    return data == pattern_sector(lba)


def _dirent(name11, attr, clus, size):
    t = 12 << 11
    d = ((2026 - 1980) << 9) | (11 << 5) | 2
    e = bytearray(32)
    e[0:11] = name11
    e[11] = attr
    struct.pack_into("<HHH", e, 14, t, d, d)
    struct.pack_into("<HHHI", e, 22, t, d, clus, size)
    return bytes(e)


def gen_sector(lba, pattern=True):
    s = bytearray(512)
    if lba == 0:
        struct.pack_into("<I", s, 440, 0x52534156)
        s[446:454] = bytes([0x00, 0xFE, 0xFF, 0xFF, 0x06, 0xFE, 0xFF, 0xFF])
        struct.pack_into("<II", s, 454, PART_LBA, PART_SECTORS)
        s[510:512] = b"\x55\xAA"
    elif lba == PART_LBA:
        s[0:3] = b"\xEB\x3C\x90"
        s[3:11] = b"RNSAVLNK"
        struct.pack_into("<HBHBHHBHHHII", s, 11, 512, SPC, RESERVED, 2, 512, 0, 0xF8, FATSZ, 63, 255,
                         PART_LBA, PART_SECTORS)
        s[36] = 0x80
        s[38] = 0x29
        struct.pack_into("<I", s, 39, 0x20261102)
        s[43:54] = b"RNSAVLINK  "
        s[54:62] = b"FAT16   "
        s[62:65] = b"\xF4\xEB\xFD"
        s[510:512] = b"\x55\xAA"
    elif LBA_FAT1 <= lba < LBA_ROOT:
        base = ((lba - LBA_FAT1) % FATSZ) * 256
        last = CLUSTERS + 1
        for i in range(256):
            e = base + i
            if e == 0:
                v = 0xFFF8
            elif e in (1, 2):
                v = 0xFFFF
            elif e == MP3_CLUS1 - 1:
                v = 0xFFFF
            elif e < last:
                v = e + 1
            elif e == last:
                v = 0xFFFF
            else:
                v = 0
            struct.pack_into("<H", s, 2 * i, v)
    elif lba == LBA_ROOT:
        s[0:32] = _dirent(b"RNSAVLINK  ", 0x08, 0, 0)
        s[32:64] = _dirent(b"TEST    TXT", 0x01, 2, len(TEST_TXT))
        s[64:96] = _dirent(b"RSAVLINKBIN", 0x07, 3, (CLUSTERS - 1 - MP3_CLUSTERS) * SPC * 512)
        s[96:128] = _dirent(b"SILENCE MP3", 0x20, MP3_CLUS1, MP3_SIZE)
    elif lba == LBA_DATA:
        s[:len(TEST_TXT)] = TEST_TXT
    elif lba == LBA_INFO:
        pass  # dynamic on the Teensy (info sector); not compared
    elif LBA_MP3 <= lba < VOL_TOTAL:
        off = (lba - LBA_MP3) * 512
        for i in range(512):
            p = off + i
            if p < MP3_SIZE and p % MP3_FRAME < 4:
                s[i] = (0xFF, 0xFB, 0x94, 0x00)[p % MP3_FRAME]
    elif LBA_BIN <= lba < VOL_TOTAL and pattern:
        s[:] = pattern_sector(lba)
    return bytes(s)


SELFTEST_LBAS = [0, 2048, 2049, LBA_FAT1, LBA_FAT1 + 1, LBA_FAT1 + 127, LBA_FAT2, LBA_FAT2 + 127,
                 LBA_ROOT, LBA_ROOT + 1, LBA_DATA, LBA_DATA + 1, LBA_DATA + 64, 0x40100, 0x7FFFF, 2097151]


def _hx(b):
    return b.hex().upper()


def _hw(words):
    return "".join(f"{w:08X}" for w in words)


def expected_selftest():
    """dict key -> expected hex string, exactly as the firmware prints 'ST <key> <value>'."""
    st = W.status
    e = {
        "cmd0": _hx(W.cmd_frame(0, 0)), "cmd17": _hx(W.cmd_frame(17, 0)),
        "cmd8": _hx(W.cmd_frame(8, 0x1AA)), "r1_17_900": _hx(W.r1(17, 0x900)),
        "crc16_ff512": "%08X" % W.crc16_bytes(bytes([0xFF]) * 512),
        "resp_cmd55_idle": _hx(W.r1(55, st(W.ST_IDLE, app=True))),
        "resp_acmd41_busy": _hx(W.r3(0x00FF8000)), "resp_acmd41_ready": _hx(W.r3(0x80FF8000)),
        "resp_cmd2": _hx(W.r2(W.cid())), "resp_cmd3": _hx(W.r6(RCA, W.ST_IDENT)),
        "resp_cmd9": _hx(W.r2(W.csd_v1())), "resp_cmd7": _hx(W.r1(7, st(W.ST_STBY))),
        "resp_cmd16": _hx(W.r1(16, st(W.ST_TRAN))), "resp_cmd55_tran": _hx(W.r1(55, st(W.ST_TRAN, app=True))),
        "resp_acmd6": _hx(W.r1(6, st(W.ST_TRAN, app=True))), "resp_acmd13": _hx(W.r1(13, st(W.ST_TRAN, app=True))),
        "resp_acmd51": _hx(W.r1(51, st(W.ST_TRAN, app=True))), "resp_cmd13": _hx(W.r1(13, st(W.ST_TRAN))),
        "resp_cmd17": _hx(W.r1(17, st(W.ST_TRAN))), "resp_cmd18": _hx(W.r1(18, st(W.ST_TRAN))),
        "resp_cmd12_data": _hx(W.r1(12, st(W.ST_DATA))), "resp_cmd12_rcv": _hx(W.r1(12, st(W.ST_RCV))),
        "resp_cmd24": _hx(W.r1(24, st(W.ST_TRAN))), "resp_cmd25": _hx(W.r1(25, st(W.ST_TRAN))),
        "scr": _hx(W.scr()), "sdstat4": _hx(W.sd_status(True)), "sdstat1": _hx(W.sd_status(False)),
        "crc16lines_scr": "".join(f"{c:08X}" for c in W.crc16_4bit_lines(W.scr())),
    }
    blk = bytes(range(256)) * 2
    e["wire1_blk"] = _hw(W.wire1(blk))
    e["wire4_blk"] = _hw(W.wire4(blk))
    e["wire4_scr"] = _hw(W.wire4(W.scr()))
    e["wire1_scr"] = _hw(W.wire1(W.scr()))
    for lba in SELFTEST_LBAS:
        e[f"sec_{lba}"] = f"{zlib.crc32(gen_sector(lba)) & M32:08X}"
    return e


# ---- mirror of the firmware's bit tricks (sdproto.cpp) to prove them against sd_wire offline
def _rev8(b):
    return int(f"{b:08b}"[::-1], 2)


def _gather4(x):
    x &= 0x11111111
    x = (x | (x >> 3)) & 0x03030303
    x = (x | (x >> 6)) & 0x000F000F
    x = (x | (x >> 12)) & 0xFF
    return x


def fw_crc_lines4(block):
    tab = []
    for i in range(256):
        d = i << 8
        for _ in range(8):
            d = ((d << 1) ^ 0x1021) & 0xFFFF if d & 0x8000 else (d << 1) & 0xFFFF
        tab.append(d)
    c = [0, 0, 0, 0]
    for i in range(0, len(block), 4):
        w = int.from_bytes(bytes(W.nibswap(x) for x in block[i:i + 4]), "little")
        for k in range(4):
            b = _rev8(_gather4(w >> k))
            c[k] = ((c[k] << 8) ^ tab[((c[k] >> 8) ^ b) & 0xFF]) & 0xFFFF
    return c


def _offline_check():
    import random
    rnd = random.Random(1)
    for _ in range(50):
        blk = bytes(rnd.randrange(256) for _ in range(rnd.choice((8, 64, 512))))
        assert fw_crc_lines4(blk) == W.crc16_4bit_lines(blk)
    # FAT sanity: chain covers the BIN file, sizes add up
    assert LBA_DATA == 4096 and LBA_BIN == 4160
    assert LBA_DATA + CLUSTERS * SPC == VOL_TOTAL
    assert (CLUSTERS + 2) * 2 <= FATSZ * 512
    p = pattern_sector(0x40100)
    assert check_sector(0x40100, p) and not check_sector(0x40101, p)
    e = expected_selftest()
    # cross-check a few SPEC 2.A.2 table rows
    assert e["resp_cmd55_idle"] == "370000012083" and e["resp_cmd17"] == "110000090067"
    assert e["resp_cmd2"] == "3F5A5253524E534156100000000101ABED"
    assert e["resp_cmd9"] == "3F000E00325B5983FFFEFBFF800A4000AF"
    return e


if __name__ == "__main__":
    e = _offline_check()
    print("offline reference check OK (firmware CRC-line trick == sd_wire, volume geometry, SPEC rows)")
    for k in ("resp_cmd3", "resp_cmd12_data", "scr", "crc16lines_scr", "sec_0", "sec_2048", "sec_4064", "sec_262400"):
        print(f"  {k:18} {e[k]}")
