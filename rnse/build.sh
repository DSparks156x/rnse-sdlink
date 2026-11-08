#!/bin/bash
# Build the rnse_proto RAM programs (WSL): wsl.exe -e bash -lc "cd '<this dir as /mnt/c/...>' && bash build.sh"
# Output: out/<prog>.bin (flat image, load at its base), .elf, .map, .dis, .sym; out/tramp.bin (PIC, 48 bytes).
# Probes (tone48 sdprobe sdbench duprobe): one .c each, window 0x4000 (unchanged).
# A/V prototypes (decbench sdstream): several objects + libgcc, window 0x20000 (see src/avsd.h for the RAM map).
set -e
cd "$(dirname "$0")"
X=~/sh-elf/bin/sh-elf-
CF="-mb -m4-single -O2 -ffreestanding -fno-builtin -nostdlib -fno-common -Wall -Wextra -Wno-unused-parameter -Isrc"
LIBGCC=$(${X}gcc -mb -m4-single -print-libgcc-file-name)
mkdir -p out
declare -A BASE=( [tone48]=0x8F700000 [sdprobe]=0x8F704000 [sdbench]=0x8F708000 [duprobe]=0x8F70C000
                  [decbench]=0x8F740000 [sdstream]=0x8F760000 )
link() {   # prog window objs...
  local p=$1 win=$2; shift 2
  ${X}ld --no-warn-rwx-segments -T src/prog.ld --defsym BASE=${BASE[$p]} --defsym WIN=$win -Map out/$p.map \
      -o out/$p.elf out/crt0.o "$@" $LIBGCC
  ${X}objcopy -O binary -R .bss out/$p.elf out/$p.bin
  ${X}objdump -d out/$p.elf > out/$p.dis
  ${X}nm -n out/$p.elf > out/$p.sym
  ${X}size out/$p.elf | tail -1
}
${X}gcc $CF -c src/crt0.S -o out/crt0.o
for p in tone48 sdprobe sdbench duprobe; do
  ${X}gcc $CF -c src/$p.c -o out/$p.o
  link $p 0x4000 out/$p.o
done
# decoder objects (shared): rnvo = plain, rnvp = with PREF, ref = video/dec/rnv2dec.c logic (builtins allowed)
${X}gcc $CF -DRNV_PF=0 -c src/rnv2sh.c -o out/rnvo.o
${X}gcc $CF -DRNV_PF=1 -c src/rnv2sh.c -o out/rnvp.o
${X}gcc $CF -DRNV_FAST=1 -c src/rnv2sh.c -o out/rnvf.o
${X}gcc $CF -c src/rnv2tab.c -o out/rnvtab.o
${X}gcc ${CF/-fno-builtin/} -Wno-sign-compare -c src/rnv2ref.c -o out/rnvref.o
${X}gcc $CF -c src/avlib.c -o out/avlib.o
${X}gcc $CF -c src/decbench.c -o out/decbench.o
# RNV3: decbench no longer fits its window with the larger decoder; not needed for sdstream
# link decbench 0x20000 out/decbench.o out/avlib.o out/rnvtab.o out/rnvo.o out/rnvp.o out/rnvf.o out/rnvref.o
if [ -f src/sdstream.c ]; then
  ${X}gcc $CF ${XCF} -c src/sdstream.c -o out/sdstream.o
  ${X}gcc $CF -c src/sdmem.c -o out/sdmem.o
  ${X}gcc $CF -c src/sdmem_target.c -o out/sdmem_target.o
  link sdstream 0x20000 out/sdstream.o out/sdmem.o out/sdmem_target.o out/avlib.o out/rnvtab.o out/rnvo.o out/rnvf.o
fi
# resident SD bootstrap (SDBOOT.txt): no crt0, no .data/.bss, entry first
${X}gcc ${CF/-O2/-Os} -c src/sdboot.c -o out/sdboot.o
${X}ld --no-warn-rwx-segments -T src/prog.ld --defsym BASE=0x8F71F000 --defsym WIN=0xDF0 -Map out/sdboot.map -o out/sdboot.elf out/sdboot.o
${X}objcopy -O binary -R .bss out/sdboot.elf out/sdboot.bin
${X}objdump -d out/sdboot.elf > out/sdboot.dis
${X}size out/sdboot.elf | tail -1
# per-function sizes of the decoder (static cost check, see DECBENCH.txt)
${X}objdump -d out/rnvo.o > out/rnvo.dis
${X}objdump -d out/rnvp.o > out/rnvp.dis
${X}gcc $CF -c src/tramp.S -o out/tramp.o
${X}ld -Ttext=0 -e _tramp -o out/tramp.elf out/tramp.o
${X}objcopy -O binary -j .text out/tramp.elf out/tramp.bin
${X}objdump -d out/tramp.elf > out/tramp.dis
rm -f out/*.o
ls -l out/*.bin
