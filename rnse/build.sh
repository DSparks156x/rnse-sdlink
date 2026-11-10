#!/bin/bash
# Build the rnse_proto RAM programs (WSL): wsl.exe -e bash -lc "cd '<this dir as /mnt/c/...>' && bash build.sh"
# Output: $OUT/<prog>.bin (flat image, load at its base), .elf, .map, .dis, .sym; $OUT/tramp.bin (PIC, 48 bytes).
# Probes (tone48 sdprobe sdbench duprobe): one .c each, window 0x4000 (unchanged).
# A/V prototypes (decbench sdstream): several objects + libgcc, window 0x20000 (see src/avsd.h for the RAM map).
# Env: SDLRAM (-D SDL_FB_BASE/SDL_CTL_BASE/SDL_WORK_BASE, src/sdl_ram.h) + SDL_IMG_BASE (= SDL_WORK_BASE + 0x8000), OUT (output dir, default out), XCF (extra sdstream defines, default -DSD_LEAVEFIX = the shipping config; XCF= for none).
# sdstream is the SDLink v2 table build (-DFWTAB, src/fwtab.h): it needs a firmware table at run time (CTL words 62/63).
set -e
cd "$(dirname "$0")"
OUT=${OUT:-out}
XCF=${XCF--DSD_LEAVEFIX}
X=~/sh-elf/bin/sh-elf-
CF="-mb -m4-single -O2 -ffreestanding -fno-builtin -nostdlib -fno-common -Wall -Wextra -Wno-unused-parameter -Isrc ${SDLRAM}"
LIBGCC=$(${X}gcc -mb -m4-single -print-libgcc-file-name)
mkdir -p $OUT
declare -A BASE=( [tone48]=0x8F700000 [sdprobe]=0x8F704000 [sdbench]=0x8F708000 [duprobe]=0x8F70C000
                  [decbench]=0x8F740000 [sdstream]=${SDL_IMG_BASE:-0x8F760000} )
link() {   # prog window objs...
  local p=$1 win=$2; shift 2
  ${X}ld --no-warn-rwx-segments -T src/prog.ld --defsym BASE=${BASE[$p]} --defsym WIN=$win -Map $OUT/$p.map \
      -o $OUT/$p.elf ${CRT0:-$OUT/crt0.o} "$@" $LIBGCC
  ${X}objcopy -O binary -R .bss $OUT/$p.elf $OUT/$p.bin
  ${X}objdump -d $OUT/$p.elf > $OUT/$p.dis
  ${X}nm -n $OUT/$p.elf > $OUT/$p.sym
  ${X}size $OUT/$p.elf | tail -1
}
${X}gcc $CF -c src/crt0.S -o $OUT/crt0.o
${X}gcc $CF -DFWTAB -c src/crt0.S -o $OUT/crt0_fwt.o
for p in tone48 sdprobe sdbench duprobe; do
  ${X}gcc $CF -c src/$p.c -o $OUT/$p.o
  link $p 0x4000 $OUT/$p.o
done
# decoder objects (shared): rnvo = plain, rnvp = with PREF, ref = video/dec/rnv2dec.c logic (builtins allowed)
${X}gcc $CF -DRNV_PF=0 -c src/rnv2sh.c -o $OUT/rnvo.o
${X}gcc $CF -DRNV_PF=1 -c src/rnv2sh.c -o $OUT/rnvp.o
${X}gcc $CF -DRNV_FAST=1 -c src/rnv2sh.c -o $OUT/rnvf.o
${X}gcc $CF -c src/rnv2tab.c -o $OUT/rnvtab.o
${X}gcc ${CF/-fno-builtin/} -Wno-sign-compare -c src/rnv2ref.c -o $OUT/rnvref.o
${X}gcc $CF -DFWTAB -c src/avlib.c -o $OUT/avlib.o
${X}gcc $CF -c src/decbench.c -o $OUT/decbench.o
# RNV3: decbench no longer fits its window with the larger decoder; not needed for sdstream
# link decbench 0x20000 $OUT/decbench.o $OUT/avlib.o $OUT/rnvtab.o $OUT/rnvo.o $OUT/rnvp.o $OUT/rnvf.o $OUT/rnvref.o
if [ -f src/sdstream.c ]; then
  echo "sdstream XCF: ${XCF}"
  ${X}gcc $CF -DFWTAB ${XCF} -c src/sdstream.c -o $OUT/sdstream.o
  ${X}gcc $CF -DFWTAB -c src/sdmem.c -o $OUT/sdmem.o
  ${X}gcc $CF -DFWTAB -c src/sdmem_target.c -o $OUT/sdmem_target.o
  CRT0=$OUT/crt0_fwt.o link sdstream 0x20000 $OUT/sdstream.o $OUT/sdmem.o $OUT/sdmem_target.o $OUT/avlib.o $OUT/rnvtab.o $OUT/rnvo.o $OUT/rnvf.o
fi
# resident SD bootstrap (SDBOOT.txt): no crt0, no .data/.bss, entry first
${X}gcc ${CF/-O2/-Os} -c src/sdboot.c -o $OUT/sdboot.o
${X}ld --no-warn-rwx-segments -T src/prog.ld --defsym BASE=0x8F71F000 --defsym WIN=0xDF0 -Map $OUT/sdboot.map -o $OUT/sdboot.elf $OUT/sdboot.o
${X}objcopy -O binary -R .bss $OUT/sdboot.elf $OUT/sdboot.bin
${X}objdump -d $OUT/sdboot.elf > $OUT/sdboot.dis
${X}size $OUT/sdboot.elf | tail -1
# per-function sizes of the decoder (static cost check, see DECBENCH.txt)
${X}objdump -d $OUT/rnvo.o > $OUT/rnvo.dis
${X}objdump -d $OUT/rnvp.o > $OUT/rnvp.dis
${X}gcc $CF -c src/tramp.S -o $OUT/tramp.o
${X}ld -Ttext=0 -e _tramp -o $OUT/tramp.elf $OUT/tramp.o
${X}objcopy -O binary -j .text $OUT/tramp.elf $OUT/tramp.bin
${X}objdump -d $OUT/tramp.elf > $OUT/tramp.dis
rm -f $OUT/*.o
ls -l $OUT/*.bin
