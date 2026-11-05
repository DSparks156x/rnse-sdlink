"""Shared constants of the rnse_proto RAM programs (mirror of src/rnse.h). No I/O here."""
import struct

PROG_BASE = {'tone48': 0x8F700000, 'sdprobe': 0x8F704000, 'sdbench': 0x8F708000, 'duprobe': 0x8F70C000,
             'decbench': 0x8F740000, 'sdstream': 0x8F760000}
PROG_ID = {'tone48': 1, 'sdprobe': 2, 'sdbench': 3, 'duprobe': 4, 'decbench': 5, 'sdstream': 6}
WINDOW = 0x4000                 # per-program window (image + .bss), icbi range (probes)
PROG_WINDOW = {'decbench': 0x20000, 'sdstream': 0x20000}    # A/V prototypes (src/avsd.h RAM map)
# capture probe (CAPTURE.txt; build_cap.sh). 8F7C0000 lies in decbench's blob area (exclusive, same task 159)
PROG_BASE['uiprobe'] = 0x8F7C0000
PROG_ID['uiprobe'] = 0x20
PROG_WINDOW['uiprobe'] = 0x8000


def window(prog):
    return PROG_WINDOW.get(prog, WINDOW)
TRAMP_BASE = 0x8F710000         # 0x40-byte launcher slots, fresh one per run
TRAMP_SIZE = 0x40
TRAMP_SLOTS = 960               # 8F710000..8F71F000; the last 4 KiB hold the SD bootstrap
SDBOOT_BASE = 0x8F71F000        # resident SD bootstrap (src/sdboot.c, SDBOOT.txt)
SDBOOT_STAMP = 0x8F71FDF0       # 16 B written last: magic, CRC-32 of the code, length, 0
SDBOOT_MAGIC = 0x53444231       # 'SDB1'
SDBOOT_ARG = 0x8F720FD0         # host pokes the header LBA here before every start
SDBOOT_RES = 0x8F720FE0         # 8 words: 'RSBR', load id, code, ms, read tries, CRC, sectors, entry
SDBOOT_SEQ = 0xBE               # Teensy window slot of the header + image (LBA 0x10000 + 0xBE * 256)
SDBOOT_LBA = 0x10000 + SDBOOT_SEQ * 256
FW_SDBOOT = [(0x8067D108, '4f22626366537ffc')]   # read core sdboot calls: live bytes checked over CAN when it is uploaded
CTL_BASE = 0x8F720000
STAT_BASE = 0x8F720100
TONE_BUF = 0x8F730000           # 64 KiB
SD_BUF = 0x8F800000             # 2 MiB
RES_BASE = 0x8F721000           # decbench/sdstream per-frame result arrays (to 0x8F730000)
DEC_STREAM = 0x8F780000         # decbench RNVS blob (2.5 MiB max)
FB = (0x8F4C0000, 0x8F57B800, 0x8F637000)   # YUY2 frame buffers (DU_DESIGN.txt 2)
REGION = (0x8F700000, 0x8FA00000)

TCB159 = 0x8E8D7B6C             # task 159 control block (r53+ lineage, r56b/r57)
ENTRY_OFF = 0x20                # task entry word read by sta_tsk
DOOM_ENTRY = 0x80B06000
TICK_MS = 0x8E8DA8B0
DMDL_HANDLE = 0x8FCE6294
DSP_EFFECT = 0x8EA18CA8

CTL_MAGIC = 0x43544C31
STAT_MAGIC = 0x41565344
DONE_MAGIC = 0xD0DE0000
TMU_HZ = 12487500.0             # Pck/4, Pck = 49.95 MHz (confirmed soc_caps_v1)

# live firmware identity checks (address, expected bytes hex) - all must match before anything runs
FW_CHECKS = [
    (0x8062D654, '2fd62fe64f22d186'),   # Snd_opn
    (0x8062D6CC, '2fb62fc62fd62fe6'),   # Snd_dyD
    (0x8062D71A, '2fe64f22d1576e43'),   # Snd_stp
    (0x8062D742, '2fe64f22d14d6e43'),   # Snd_cls
    (0x80653598, '2f862f962fa62fb6'),   # sd_raw_read
    (0x804D04D8, '0002c8f08bd7e226'),   # dly_tsk
    (0x804D027C, '0002c8f08b74e201'),   # ext_tsk
]
VNMC_LIT = 0x80628CD8           # 00010749 on r57, 00010449 on r56b


def ctl_words(params):
    if len(params) > 16:
        raise ValueError('at most 16 CTL parameters')
    p = list(params) + [0] * max(0, 14 - len(params))
    return struct.pack('>%dI' % (2 + len(p)), CTL_MAGIC, 0, *[x & 0xFFFFFFFF for x in p])


def words(b):
    return list(struct.unpack('>%dI' % (len(b) // 4), b[:len(b) // 4 * 4]))


def header(st):
    w = words(st[:32])
    return dict(magic=w[0], prog=w[1], stage=w[2], err=w[3], hb=w[4], t0=w[5], t1=w[6], done_word=w[7],
                valid=w[0] == STAT_MAGIC, done=(w[7] & 0xFFFF0000) == DONE_MAGIC)


def s32(x):
    return x - (1 << 32) if x & 0x80000000 else x
