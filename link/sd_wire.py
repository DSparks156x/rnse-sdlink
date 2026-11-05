"""sd_wire.py -- reference encoder for the Pi -> Teensy -> RNS-E SD link (SPEC.md).

Single source of truth for:
  * CRC7 / CRC16 (SD physical layer), self-tested against published vectors
  * every fixed card response of the stock-driver profile (mode A), with CRC7
  * CID / CSD v1 / SCR / SD-status contents
  * 4-bit and 1-bit DAT "wire images" (exact 32-bit words the Teensy FlexIO
    transmit shifter consumes, LSB-first on the wire)
  * superframe header pack/unpack (big-endian, CRC32)

Run:  <python> -I sd_wire.py          -> self-test + prints the tables used in SPEC.md
No third-party deps.  Python 3.8+.
"""
import struct, zlib

# --------------------------------------------------------------------------- CRCs
def crc7(data: bytes) -> int:
    """SD CRC7, poly x^7+x^3+1, init 0, MSB first. Returns 7-bit value."""
    c = 0
    for b in data:
        for i in range(7, -1, -1):
            fb = ((c >> 6) & 1) ^ ((b >> i) & 1)
            c = (c << 1) & 0x7F
            if fb:
                c ^= 0x09
    return c

def crc16_bits(bits) -> int:
    """SD CRC16-CCITT, poly 0x1021, init 0, over an iterable of bits (MSB first)."""
    c = 0
    for bit in bits:
        fb = ((c >> 15) & 1) ^ bit
        c = (c << 1) & 0xFFFF
        if fb:
            c ^= 0x1021
    return c

def crc16_bytes(data: bytes) -> int:
    return crc16_bits(((b >> i) & 1) for b in data for i in range(7, -1, -1))

def crc16_4bit_lines(block: bytes):
    """Per-DAT-line CRC16 for 4-bit mode. Line k carries bit (4+k) then bit k of every byte."""
    out = []
    for k in range(4):
        out.append(crc16_bits(bit for b in block for bit in (((b >> (4 + k)) & 1), ((b >> k) & 1))))
    return out  # [crc_DAT0, crc_DAT1, crc_DAT2, crc_DAT3]

# --------------------------------------------------------------------------- commands / responses
def cmd_frame(index: int, arg: int, host=True) -> bytes:
    """48-bit token (start 0, dir bit, index, 32-bit field, CRC7, end 1)."""
    b0 = (0x40 if host else 0x00) | (index & 0x3F)
    body = bytes([b0]) + struct.pack(">I", arg & 0xFFFFFFFF)
    return body + bytes([(crc7(body) << 1) | 1])

def r1(index: int, status: int) -> bytes:
    return cmd_frame(index, status, host=False)

def r3(ocr: int) -> bytes:
    # R3: index field and CRC field are all ones (no CRC)
    return bytes([0x3F]) + struct.pack(">I", ocr) + b"\xFF"

def r2(reg16: bytes) -> bytes:
    """136-bit R2: '0','0','111111', then register bits 127..1 (incl. internal CRC7), end '1'.
    reg16 = 15 content bytes; the 16th byte (CRC7<<1|1) is appended here."""
    assert len(reg16) == 15
    return b"\x3F" + reg16 + bytes([(crc7(reg16) << 1) | 1])

def r6(rca: int, state: int) -> bytes:
    # R6 field: RCA[31:16] | status bits 23,22,19 -> 15..13 (0 here) | status[12:0]
    return cmd_frame(3, (rca << 16) | (state << 9) | 0x100, host=False)

# card status (R1) helpers
ST_IDLE, ST_READY, ST_IDENT, ST_STBY, ST_TRAN, ST_DATA, ST_RCV, ST_PRG = range(8)
READY_FOR_DATA = 1 << 8
APP_CMD = 1 << 5
def status(state, app=False, extra=0):
    return (state << 9) | READY_FOR_DATA | (APP_CMD if app else 0) | extra

# --------------------------------------------------------------------------- registers
def pack_bits(fields, nbits):
    """fields: list of (msb, lsb, value). Returns big-endian bytes of nbits."""
    v = 0
    for msb, lsb, val in fields:
        w = msb - lsb + 1
        assert 0 <= val < (1 << w), (msb, lsb, val)
        v |= val << lsb
    return v.to_bytes(nbits // 8, "big")

def csd_v1(c_size=4095, c_size_mult=7, read_bl_len=9):
    f = [(127, 126, 0), (119, 112, 0x0E), (111, 104, 0x00), (103, 96, 0x32),
         (95, 84, 0x5B5), (83, 80, read_bl_len), (79, 79, 1), (73, 62, c_size),
         (61, 59, 7), (58, 56, 6), (55, 53, 7), (52, 50, 6), (49, 47, c_size_mult),
         (46, 46, 1), (45, 39, 0x7F), (28, 26, 2), (25, 22, 9)]
    full = pack_bits(f, 128)
    return full[:15]

def cid():
    f = [(127, 120, 0x5A), (119, 104, int.from_bytes(b"RS", "big")),
         (103, 64, int.from_bytes(b"RNSAV", "big")), (63, 56, 0x10),
         (55, 24, 0x00000001), (19, 8, (26 << 4) | 11)]
    return pack_bits(f, 128)[:15]

def scr():
    # SCR_STRUCTURE 0, SD_SPEC 0 (v1.0/1.01 -> no CMD6, no CMD8 expected), DATA_STAT_AFTER_ERASE 0,
    # SD_SECURITY 0 (no CPRM), SD_BUS_WIDTHS 0b0101 (1 and 4 bit)
    return pack_bits([(63, 60, 0), (59, 56, 0), (55, 55, 0), (54, 52, 0), (51, 48, 0b0101)], 64)

def sd_status(four_bit=True):
    return pack_bits([(511, 510, 0b10 if four_bit else 0b00), (447, 440, 0x02), (431, 428, 0x9)], 512)

def capacity_bytes(c_size, mult, bl_len):
    return (c_size + 1) * (1 << (mult + 2)) * (1 << bl_len)

# --------------------------------------------------------------------------- wire images
def nibswap(b):
    return ((b & 0x0F) << 4) | (b >> 4)

def wire4(block: bytes):
    """4-bit wire image for one data block of any length (512 for sectors, 64 SD status, 8 SCR).
    Words are LSB-nibble-first; nibble bit k drives DAT k (FlexIO PINSEL = DAT0, PWIDTH=3).
    Layout: [0x0FFFFFFF start word] [data words] [CRC words] [0xFFFFFFFF end+idle]."""
    assert len(block) % 4 == 0
    words = [0x0FFFFFFF]                        # 7 idle '1' nibbles, then start nibble 0
    for i in range(0, len(block), 4):
        words.append(int.from_bytes(bytes(nibswap(x) for x in block[i:i + 4]), "little"))
    crcs = crc16_4bit_lines(block)
    nibs = []
    for j in range(15, -1, -1):                 # CRC MSB first on every line simultaneously
        nibs.append(sum(((crcs[k] >> j) & 1) << k for k in range(4)))
    for i in range(0, 16, 8):
        words.append(sum(n << (4 * p) for p, n in enumerate(nibs[i:i + 8])))
    words.append(0xFFFFFFFF)                    # end-bit nibble F + idle
    return words

def bitrev8(b):
    return int(f"{b:08b}"[::-1], 2)

def bitrev16(v):
    return int(f"{v:016b}"[::-1], 2)

def wire1(block: bytes):
    """1-bit wire image (DAT0 only), 32-bit words LSB first."""
    words = [0x7FFFFFFF]                        # 31 idle ones, then start bit 0
    for i in range(0, len(block), 4):
        words.append(int.from_bytes(bytes(bitrev8(x) for x in block[i:i + 4]), "little"))
    words.append(bitrev16(crc16_bytes(block)) | 0xFFFF0000)   # CRC16 MSB-first, end bit, idle
    return words

def unwire4(words):
    """Decode a 4-bit wire image (or a captured RX stream already nibble-aligned) back to bytes
    and check CRC. Used by the Teensy/Pi for host WRITE blocks."""
    nibs = [(w >> (4 * p)) & 0xF for w in words for p in range(8)]
    s = nibs.index(0)                           # first all-zero nibble = start bit on all lines
    n = nibs[s + 1:]
    nbytes = (len(words) - 4) * 4
    data = bytes((n[2 * i] << 4) | n[2 * i + 1] for i in range(nbytes))
    crcn = n[2 * nbytes:2 * nbytes + 16]
    got = [sum(((crcn[15 - j] >> k) & 1) << j for j in range(16)) for k in range(4)]
    return data, got == crc16_4bit_lines(data), n[2 * nbytes + 16] == 0xF

# --------------------------------------------------------------------------- superframe header
SF_MAGIC = 0x52534156  # 'RSAV'
HDR_FMT = ">IHHIIIIIHBBIHHHHIIHHI"   # 60 bytes before hdr_crc32
HDR_FIELDS = ("magic version hdr_len seq flags audio_pts video_pts video_bytes audio_frames "
              "audio_fmt video_fmt audio_bytes audio_sec video_sec total_sec next_total_hint "
              "video_crc32 audio_crc32 ctrl_bytes frame_id pi_time_us").split()
assert struct.calcsize(HDR_FMT) == 60

def pack_header_sector(ctrl=b"", **kw):
    d = dict(magic=SF_MAGIC, version=1, hdr_len=64, ctrl_bytes=len(ctrl))
    d.update(kw)
    body = struct.pack(HDR_FMT, *[d.get(k, 0) for k in HDR_FIELDS])
    assert len(ctrl) <= 448
    rest = ctrl + b"\x00" * (448 - len(ctrl))
    crc = zlib.crc32(body + rest) & 0xFFFFFFFF
    return body + struct.pack(">I", crc) + rest

def parse_header_sector(sec: bytes):
    if len(sec) != 512 or struct.unpack(">I", sec[:4])[0] != SF_MAGIC:
        return None
    if (zlib.crc32(sec[:60] + sec[64:]) & 0xFFFFFFFF) != struct.unpack(">I", sec[60:64])[0]:
        return None
    return dict(zip(HDR_FIELDS, struct.unpack(HDR_FMT, sec[:60])))

# --------------------------------------------------------------------------- self-test + tables
def _selftest():
    # published vectors (SD physical layer simplified spec, CRC section examples)
    assert cmd_frame(0, 0)[-1] == 0x95, "CMD0"
    assert cmd_frame(17, 0)[-1] == 0x55, "CMD17"
    assert r1(17, 0x00000900)[-1] == 0x67, "R1 CMD17 0x900"
    assert cmd_frame(8, 0x1AA)[-1] == 0x87, "CMD8"
    assert crc16_bytes(b"\xFF" * 512) == 0x7FA1, "CRC16 FF*512"
    blk = bytes(range(256)) * 2
    for wf, nwords in ((wire4, 132), (wire1, 130)):
        assert len(wf(blk)) == nwords
    d, ok, end = unwire4(wire4(blk))
    assert d == blk and ok and end
    assert capacity_bytes(4095, 7, 9) == 1 << 30
    h = pack_header_sector(seq=7, video_bytes=1234, ctrl=b"hi")
    assert parse_header_sector(h)["seq"] == 7
    return True

def _hex(b):
    return " ".join(f"{x:02X}" for x in b)

if __name__ == "__main__":
    _selftest()
    print("self-test OK (CMD0=95, CMD17=55, R1(17,0x900)=67, CMD8=87, CRC16(FF*512)=7FA1)\n")
    RCA = 0x5A01
    rows = [
        ("CMD0  GO_IDLE", "-", "no response"),
        ("CMD8  SEND_IF_COND 0x1AA", "-", "no response (SD 1.x profile)"),
        ("CMD55 arg 0 (idle)", "R1", _hex(r1(55, status(ST_IDLE, app=True)))),
        ("ACMD41 (busy polls)", "R3", _hex(r3(0x00FF8000))),
        ("ACMD41 (ready)", "R3", _hex(r3(0x80FF8000))),
        ("CMD2  ALL_SEND_CID", "R2", _hex(r2(cid()))),
        ("CMD3  SEND_RCA", "R6", _hex(r6(RCA, ST_IDENT))),
        ("CMD9  SEND_CSD", "R2", _hex(r2(csd_v1()))),
        ("CMD7  SELECT (stby->tran)", "R1b", _hex(r1(7, status(ST_STBY)))),
        ("CMD16 SET_BLOCKLEN 512", "R1", _hex(r1(16, status(ST_TRAN)))),
        ("CMD55 arg RCA (tran)", "R1", _hex(r1(55, status(ST_TRAN, app=True)))),
        ("ACMD6 SET_BUS_WIDTH", "R1", _hex(r1(6, status(ST_TRAN, app=True)))),
        ("ACMD13 SD_STATUS", "R1+64B", _hex(r1(13, status(ST_TRAN, app=True)))),
        ("ACMD51 SEND_SCR", "R1+8B", _hex(r1(51, status(ST_TRAN, app=True)))),
        ("CMD13 SEND_STATUS (tran)", "R1", _hex(r1(13, status(ST_TRAN)))),
        ("CMD17 READ_SINGLE", "R1", _hex(r1(17, status(ST_TRAN)))),
        ("CMD18 READ_MULTIPLE", "R1", _hex(r1(18, status(ST_TRAN)))),
        ("CMD12 STOP (in data)", "R1b", _hex(r1(12, status(ST_DATA)))),
        ("CMD24 WRITE_BLOCK", "R1", _hex(r1(24, status(ST_TRAN)))),
        ("CMD25 WRITE_MULTIPLE", "R1", _hex(r1(25, status(ST_TRAN)))),
        ("CMD12 STOP (in rcv)", "R1b", _hex(r1(12, status(ST_RCV)))),
        ("CMD60 W32 uplink (custom)", "R1", "R1 index 60, status field = 32-bit downlink word"),
    ]
    print(f"{'command':30} {'type':7} response bytes on CMD (MSB first)")
    for a, b, c in rows:
        print(f"{a:30} {b:7} {c}")
    print("\nCID   :", _hex(cid() + bytes([(crc7(cid()) << 1) | 1])))
    print("CSD v1:", _hex(csd_v1() + bytes([(crc7(csd_v1()) << 1) | 1])), " capacity", capacity_bytes(4095, 7, 9))
    print("SCR   :", _hex(scr()), " CRC16(1-bit)", hex(crc16_bytes(scr())),
          " CRC16 lines(4-bit)", [hex(x) for x in crc16_4bit_lines(scr())])
    ss = sd_status(True)
    print("SDSTAT(4-bit) first 12 B:", _hex(ss[:12]), " CRC16 lines", [hex(x) for x in crc16_4bit_lines(ss)])
    z = bytes(512)
    print("\nwire4(512 x 00) words:", len(wire4(z)), " first/last:", [hex(w) for w in wire4(z)[:2]], [hex(w) for w in wire4(z)[-3:]])
    print("wire1(512 x 00) words:", len(wire1(z)), " last:", hex(wire1(z)[-1]))
    print("SCR wire4 :", [hex(w) for w in wire4(scr())])
