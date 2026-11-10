#!/bin/bash
# Build the sh-elf cross toolchain rnse/ needs (SH-4 big-endian, -m4-single, C only, libgcc, no libc), the same versions and
# configure options as the one the module was developed with, so a CI build reproduces its build id:
#   binutils 2.42, gcc 13.3.0 (--with-multilib-list=m4,m4-single), newlib 4.4.0.20231231 (headers)
# Usage: tools/build-sh-elf.sh [PREFIX]     (default ~/sh-elf). Then: SH_ELF=PREFIX/bin/sh-elf- bash rnse/module/build.sh
# Debian/Ubuntu needs: build-essential curl xz-utils libgmp-dev libmpfr-dev libmpc-dev texinfo. About 15 min on 4 cores.
set -euo pipefail
PREFIX=${1:-$HOME/sh-elf}
BINUTILS=2.42
GCC=13.3.0
NEWLIB=4.4.0.20231231
J=$(nproc)
W=$(mktemp -d)
trap 'rm -rf "$W"' EXIT
cd "$W"
curl -fsSL "https://ftp.gnu.org/gnu/binutils/binutils-$BINUTILS.tar.xz" | tar xJ
curl -fsSL "https://ftp.gnu.org/gnu/gcc/gcc-$GCC/gcc-$GCC.tar.xz" | tar xJ
mkdir b-binutils b-gcc
(cd b-binutils && "../binutils-$BINUTILS/configure" --target=sh-elf --prefix="$PREFIX" --disable-nls --disable-werror \
  && make -j"$J" && make install)
export PATH="$PREFIX/bin:$PATH"
(cd b-gcc && "../gcc-$GCC/configure" --target=sh-elf --prefix="$PREFIX" --enable-languages=c --with-newlib --disable-nls \
  --disable-shared --disable-threads --disable-libssp --disable-libgomp --disable-libquadmath \
  --with-multilib-list=m4,m4-single --disable-bootstrap \
  && make -j"$J" all-gcc && make install-gcc)
# newlib: only its headers are used (the RAM programs are -nostdlib), built the same way as the reference toolchain
curl -fsSL "https://sourceware.org/pub/newlib/newlib-$NEWLIB.tar.gz" | tar xz
mkdir b-newlib
(cd b-newlib && "../newlib-$NEWLIB/configure" --target=sh-elf --prefix="$PREFIX" --disable-nls --enable-newlib-nano-malloc \
  --enable-newlib-nano-formatted-io --disable-newlib-supplied-syscalls && make -j"$J" && make install)
(cd b-gcc && make -j"$J" all-target-libgcc && make install-target-libgcc)
"$PREFIX/bin/sh-elf-gcc" --version | head -1
