#!/bin/sh
# make test, part 2: whole-encoder checks on a synthetic clip.
#  - SIMD (NEON) and scalar encoders, 1 and 3 threads, RGB24 and XRGB8888 input: identical bitstreams
#  - RNV3 reference decoder (dec/rnv3dec.c) output == encoder reconstruction on every frame (CRC32)
#  - damage-hint path (--auto-damage) stays closed-loop
#  - headroom-driven quality (default): link caps 2.5 / 8 MB/s and a --linkctl file give identical streams for 1 and 3 threads,
#    --linkctl "cap 2.5e6" == --linkcap 2.5e6, and every one of them decodes bit-exactly (closed loop)
set -e
T=$(mktemp -d)
N=${N:-120}
./synth $N > $T/clip.raw
./synth $N bgr0 > $T/clip.bgr0
./rnv2enc -q -i $T/clip.raw -o $T/a.rnv --crc $T/a.crc 2>/dev/null
./rnv2enc -q -i $T/clip.raw -o $T/b.rnv --scalar 2>/dev/null
./rnv2enc -q -i $T/clip.raw -o $T/c.rnv --threads 3 2>/dev/null
./rnv2enc -q -i $T/clip.bgr0 --fmt bgr0 -o $T/d.rnv --threads 2 2>/dev/null
./rnv2enc -q -i $T/clip.raw -o $T/e.rnv --auto-damage --crc $T/e.crc --threads 2 2>/dev/null
./rnv2enc -q -i $T/clip.raw -o $T/h1.rnv --linkcap 2.5e6 --crc $T/h1.crc 2>/dev/null
./rnv2enc -q -i $T/clip.raw -o $T/h3.rnv --linkcap 2.5e6 --threads 3 2>/dev/null
./rnv2enc -q -i $T/clip.raw -o $T/h8.rnv --linkcap 8e6 --crc $T/h8.crc --threads 2 2>/dev/null
printf '# comment
8e6
cap 2.5e6
' > $T/ctl.txt
./rnv2enc -q -i $T/clip.raw -o $T/hc.rnv --linkctl $T/ctl.txt --threads 2 2>/dev/null
fail=0
if [ -x ./rnv2enc_emu ]; then  # x86 only: NEON kernels through neon_emu.h, whole encoder
  ./rnv2enc_emu -q -i $T/clip.raw -o $T/f.rnv --threads 2 2>/dev/null
  cmp -s $T/a.rnv $T/f.rnv && echo "OK   emulated-NEON encoder == scalar bitstream" || { echo "FAIL emulated-NEON encoder differs"; fail=1; }
fi
cmp -s $T/a.rnv $T/b.rnv && echo "OK   simd == scalar bitstream" || { echo "FAIL simd != scalar bitstream"; fail=1; }
cmp -s $T/a.rnv $T/c.rnv && echo "OK   1 thread == 3 threads bitstream" || { echo "FAIL thread count changes the bitstream"; fail=1; }
cmp -s $T/a.rnv $T/d.rnv && echo "OK   RGB24 == XRGB8888 input" || { echo "FAIL input format changes the bitstream"; fail=1; }
cmp -s $T/h1.rnv $T/h3.rnv && echo "OK   linkcap 2.5e6: 1 thread == 3 threads bitstream" || { echo "FAIL linkcap: thread count changes the bitstream"; fail=1; }
cmp -s $T/h1.rnv $T/hc.rnv && echo "OK   --linkctl file (last line cap 2.5e6) == --linkcap 2.5e6" || { echo "FAIL linkctl != linkcap"; fail=1; }
for x in a e h1 h8; do
  ./rnv3dec_ref $T/$x.rnv $T/$x.dec 2>/dev/null
  bad=$(cut -d' ' -f1,2 $T/$x.dec | paste $T/$x.crc - | awk '{n++; if ($2 != $4) b++} END {print (n == 0) ? -1 : b + 0}')
  [ "$bad" = "0" ] && echo "OK   closed loop ($x): decoder == encoder recon on all $N frames" || { echo "FAIL closed loop ($x): $bad mismatching frames"; fail=1; }
done
rm -rf $T
[ $fail = 0 ] && echo "closed_loop_test: PASS" || { echo "closed_loop_test: FAIL"; exit 1; }
