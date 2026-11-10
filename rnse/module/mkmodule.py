#!/usr/bin/env python3
"""Build the SDLink flash module (SDST) and its sector image: rnse/out/module/sector_107.bin (+ .sha256, module.json).

Run through rnse/module/build.sh (WSL / Linux, sh-elf toolchain at ~/sh-elf/bin, Python 3 stdlib only). It expects rnse/build.sh to
have built out/sdstream.bin/.sym with the shipping config (XCF default -DSD_LEAVEFIX, sdstream built -DFWTAB).

Layout of the 128 KiB sector at NOR 0x00D60000 (sector 107) -- the firmware contract, docs/FIRMWARE_CONTRACT.md:
  +0x00  0x70 bytes header text, starts with the tag '.SDS' (NOT a boot-ROM / OSM module tag, so nothing at boot walks it)
  +0x70  BE32 destination 0x00D60000      +0x74  BE32 declared size (0x80 + payload)
  +0x78  BE32 word sum of the payload     +0x7C  0
  +0x80  payload: hdr.S block ('TVA2', tva_poll, tva_task, image, image length, build id, required table version,
         stream formats (sdl_ver.h: SDL_STREAM_FMT_MIN << 8 | SDL_STREAM_FMT)),
         tvauto code, the sdstream RAM image (copied to RAM by tvauto at start)
  rest   zero up to 0x20000 (the firmware's TOC allocation is the whole sector, so a disc install and a field write give the same bytes)
"""
import hashlib, json, os, re, struct, subprocess, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent            # rnse/module
RNSE = HERE.parent
OUT = RNSE / 'out'
MOD = OUT / 'module'
X = os.environ.get('SH_ELF') or os.path.expanduser('~/sh-elf/bin/sh-elf-')   # toolchain prefix; CI: tools/build-sh-elf.sh
HISTORY = HERE / 'history.txt'

SECTOR = 107
DEST = SECTOR * 0x20000                           # 0x00D60000
XIP = 0x80000000 + DEST + 0x80                    # 0x80D60080
SECT = 0x20000
FWT_VERSION = 1                                   # fwtab.h FWT_VERSION (checked against the header below)
TAG = b'.SDS'

# The run parameters tvauto writes into sdstream's CTL words (rnse/sdstream.py make_ctl with: slot 0, gate 0, IIS route, no hop, mode av,
# secs 0 (forever), wrap 0 (live), target 7800, block 960, sink 3, standby off). Word 1 (stop word) is never written by tvauto.
CTL_WORDS = [0x43544C31, 0x00000000, 0x00000000, 0x00000060, 0x00000017, 0x00000000, 0x00000000, 0x00000001,
             0x00000300, 0x000003C0, 0x00001E78, 0x00000003, 0x00000040, 0x00000000, 0x00000000, 0x00000000]
CTL_FLAGS = 0x17
USE_SCREEN = 1                                    # TV screen word as well as the source context
STANDBY = 0


def die(m):
    sys.exit('mkmodule: ' + m)


def run(cmd):
    r = subprocess.run(cmd, cwd=RNSE, capture_output=True, text=True)
    if r.returncode or 'warning' in (r.stdout + r.stderr).lower():
        die('%s\n%s%s' % (' '.join(cmd), r.stdout, r.stderr))
    return r.stdout


def syms(path):
    out = {}
    for ln in Path(path).read_text().splitlines():
        p = ln.split()
        if len(p) == 3:
            out[p[2]] = int(p[0], 16)
    return out


def wsum(b):
    return sum(x[0] for x in struct.iter_unpack('>I', b)) & 0xFFFFFFFF


def stream_fmt():
    """(newest, oldest) stream format from src/sdl_ver.h, the one definition (docs/FIRMWARE_CONTRACT.md, format-version rule)."""
    t = (RNSE / 'src' / 'sdl_ver.h').read_text()
    v = {k: int(m.group(1), 0) for k in ('SDL_STREAM_FMT', 'SDL_STREAM_FMT_MIN')
         for m in [re.search(r'#define\s+%s\s+(\w+)' % k, t)] if m}
    if len(v) != 2 or not 1 <= v['SDL_STREAM_FMT_MIN'] <= v['SDL_STREAM_FMT'] <= 255:
        die('sdl_ver.h SDL_STREAM_FMT / SDL_STREAM_FMT_MIN missing or out of range')
    return v['SDL_STREAM_FMT'], v['SDL_STREAM_FMT_MIN']


def header_text(build_id):
    """0x70 bytes, same CR/LF field shape as the firmware's own module headers; only the tag is ever compared (and '.SDS' is
    deliberately not an accepted boot/OSM tag)."""
    t = TAG + b'\r\nSDLINK%02d\r\n' % FWT_VERSION
    t += b'BUILD %08X\r\n' % build_id
    t += b'sdlink SDST sector %d\r\n' % SECTOR
    if len(t) > 0x70:
        die('header text too long')
    return t + bytes(0x70 - len(t))


def history(build):
    """Build ids oldest first, ending with this one (module.json 'history'). Build ids are hashes with no order of their own, so
    an updater orders an installed module against the bundled one by this list. A build not listed yet is appended to
    module/history.txt: commit that line with the sources it was built from."""
    ids = []
    if HISTORY.exists():
        ids = [ln.split()[0].upper() for ln in HISTORY.read_text().splitlines() if ln.strip() and not ln.startswith('#')]
    if build not in ids:
        import datetime
        with HISTORY.open('a', newline='\n') as f:
            f.write('%s %s\n' % (build, datetime.date.today().isoformat()))
        ids.append(build)
    return ids


def main():
    img_p, sym_p = OUT / 'sdstream.bin', OUT / 'sdstream.sym'
    if not img_p.exists() or not sym_p.exists():
        die('run rnse/build.sh first (out/sdstream.bin/.sym missing)')
    img = img_p.read_bytes()
    img += bytes(-len(img) % 4)
    s = syms(sym_p)
    for k in ('_start', '_main', '__bss_start', '__bss_end', '_g_fwt', '_fwt_exit'):
        if k not in s:
            die('sdstream symbol %s missing: not the -DFWTAB table build?' % k)
    fwt = (RNSE / 'src' / 'fwtab.h').read_text()
    if '#define FWT_VERSION  %du' % FWT_VERSION not in fwt:
        die('fwtab.h FWT_VERSION != %d' % FWT_VERSION)
    build_id = int(hashlib.sha256(img).hexdigest()[:8], 16)

    MOD.mkdir(parents=True, exist_ok=True)
    (MOD / 'sdstream_pin.bin').write_bytes(img)
    (MOD / 'gen_cfg.h').write_text(
        '/* generated by rnse/module/mkmodule.py */\n#define IMG_BASE 0x%08Xu\n#define IMG_MAIN 0x%08Xu\n#define IMG_BSS_S 0x%08Xu\n'
        '#define IMG_BSS_E 0x%08Xu\n#define TVA_CTL_WORDS { %s }\n#define TVA_CTL_FLAGS 0x%Xu\n#define TVA_USE_SCREEN %d\n'
        '#define TVA_BUILD_ID 0x%08Xu\n#define TVA_STANDBY %d\n' % (
            s['_start'], s['_main'], s['__bss_start'], s['__bss_end'], ', '.join('0x%08Xu' % w for w in CTL_WORDS), CTL_FLAGS,
            USE_SCREEN, build_id, STANDBY))
    cf = ['-mb', '-m4-single', '-O2', '-ffreestanding', '-fno-builtin', '-nostdlib', '-fno-common', '-Wall', '-Wextra',
          '-Wno-unused-parameter', '-Imodule', '-Isrc', '-Iout/module']
    libgcc = run([X + 'gcc', '-mb', '-m4-single', '-print-libgcc-file-name']).strip()
    run([X + 'gcc'] + cf + ['-c', 'module/tvauto.c', '-o', 'out/module/tvauto.o'])
    run([X + 'gcc'] + cf + ['-DIMAGE_BIN="out/module/sdstream_pin.bin"', '-DHDR_BUILD_ID=0x%08X' % build_id,
                            '-DHDR_FWT_VER=%d' % FWT_VERSION, '-Wa,-I.', '-c', 'module/hdr.S', '-o', 'out/module/hdr.o'])
    run([X + 'ld', '--no-warn-rwx-segments', '--orphan-handling=warn', '-T', 'module/tvauto.ld', '--defsym', 'BASE=0x%08X' % XIP,
         '-o', 'out/module/sdst.elf', 'out/module/hdr.o', 'out/module/tvauto.o', libgcc])
    run([X + 'objcopy', '-O', 'binary', 'out/module/sdst.elf', 'out/module/sdst.bin'])
    (MOD / 'sdst.dis').write_text(run([X + 'objdump', '-d', 'out/module/sdst.elf']))
    (MOD / 'sdst.sym').write_text(run([X + 'nm', '-n', 'out/module/sdst.elf']))

    payload = (MOD / 'sdst.bin').read_bytes()
    payload += bytes(-len(payload) % 4)
    ss = syms(MOD / 'sdst.sym')
    if payload[:4] != b'TVA2' or struct.unpack_from('>II', payload, 4) != (ss['_tva_poll'], ss['_tva_task']):
        die('module head block')
    fmt, fmt_min = stream_fmt()
    if struct.unpack_from('>III', payload, 0x14) != (build_id, FWT_VERSION, fmt_min << 8 | fmt):
        die('build id / table version / stream format words')
    i0 = ss['_tva_image'] - XIP
    if payload[i0:i0 + len(img)] != img:
        die('embedded sdstream image differs')
    module = header_text(build_id) + struct.pack('>IIII', DEST, 0x80 + len(payload), wsum(payload), 0) + payload
    if len(module) > SECT:
        die('module 0x%X B does not fit one sector' % len(module))
    sector = module + bytes(SECT - len(module))
    (OUT / 'sector_107.bin').write_bytes(sector)
    h = hashlib.sha256(sector).hexdigest()
    (OUT / 'sector_107.bin.sha256').write_text('%s  sector_107.bin\n' % h)
    info = dict(sector=SECTOR, nor=hex(DEST), xip=hex(XIP), sha256=h, bytes=SECT, declared=hex(0x80 + len(payload)),
                payload_sum='%08X' % wsum(payload), build_id='%08X' % build_id, table_version=FWT_VERSION,
                stream_fmt=fmt, stream_fmt_min=fmt_min,
                sdstream=dict(sha256=hashlib.sha256(img).hexdigest(), bytes=len(img), base='%08X' % s['_start'],
                              main='%08X' % s['_main'], bss=['%08X' % s['__bss_start'], '%08X' % s['__bss_end']]),
                ctl_words=['%08X' % w for w in CTL_WORDS], ctl_flags=CTL_FLAGS, use_screen=USE_SCREEN, standby=STANDBY,
                tva_symbols={k: '%08X' % v for k, v in ss.items() if k.startswith('_tva')},
                history=history('%08X' % build_id))
    (OUT / 'module.json').write_text(json.dumps(info, indent=2) + '\n')
    print('sector_107.bin  %s  build %08X  fmt %d (min %d)  payload %d B  sum %08X' % (h, build_id, fmt, fmt_min, len(payload), wsum(payload)))


if __name__ == '__main__':
    main()
