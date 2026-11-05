#!/bin/bash
# Build the sdtime probe only (WSL). Separate from build.sh so it cannot disturb a bench build in progress.
set -e
cd "$(dirname "$0")"
X=~/sh-elf/bin/sh-elf-
CF="-mb -m4-single -O2 -ffreestanding -fno-builtin -fno-tree-loop-distribute-patterns -nostdlib -fno-common -Wall -Wextra -Wno-unused-parameter -Isrc"
LIBGCC=$(${X}gcc -mb -m4-single -print-libgcc-file-name)
T=$(mktemp -d)
${X}gcc $CF -c src/crt0.S -o $T/crt0.o
${X}gcc $CF -c src/sdtime.c -o $T/sdtime.o
${X}ld --no-warn-rwx-segments -T src/prog.ld --defsym BASE=0x8F7C8000 --defsym WIN=0x4000 -Map out/sdtime.map \
    -o out/sdtime.elf $T/crt0.o $T/sdtime.o $LIBGCC
${X}objcopy -O binary -R .bss out/sdtime.elf out/sdtime.bin
${X}objdump -d out/sdtime.elf > out/sdtime.dis
${X}nm -n out/sdtime.elf > out/sdtime.sym
${X}size out/sdtime.elf | tail -1
rm -rf $T
ls -l out/sdtime.bin
