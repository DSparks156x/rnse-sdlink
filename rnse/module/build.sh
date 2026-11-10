#!/bin/bash
# Build the SDLink flash module: rnse/out/sector_107.bin (+ .sha256, module.json). One sector image for every supported firmware.
# Needs: sh-elf toolchain (SH-4 big-endian, -m4-single), python3. On Windows: wsl.exe -e bash -lc "bash rnse/module/build.sh"
# Toolchain: ~/sh-elf/bin by default; SH_ELF=<prefix> (e.g. /opt/sh-elf/bin/sh-elf-) for another. tools/build-sh-elf.sh makes one.
set -e
cd "$(dirname "$0")/.."
mkdir -p out
bash build.sh > out/build.log 2>&1 || { cat out/build.log; exit 1; }
grep -q 'sdstream XCF: -DSD_LEAVEFIX' out/build.log || { echo "sdstream not built with the shipping config"; exit 1; }
python3 module/mkmodule.py
