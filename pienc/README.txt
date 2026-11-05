rnv2enc - real-time RNV2 encoder for the Raspberry Pi 5 (C11, NEON)
====================================================================
Encodes 800x480 RGB frames into the RNV2 stream that video/dec/rnv2dec.c decodes (same .rnv container: every frame
is preceded by a 4-byte big-endian length).  The encoder's reconstruction equals the decoder output bit for bit.
Background, results and the Pi CPU budget: REPORT.txt.

FILES
  include/rnv2enc.h     library API (rnv2enc_create / rnv2enc_encode / rnv2enc_destroy, params, per-frame info)
  src/enc.c             encoder core (pipeline, motion search, RD, rate control, bitstream, thread pool)
  src/kern_scalar.c     scalar reference kernels      src/kern_neon.c   NEON kernels (bit-identical)
  src/kern.h            kernel table                   src/csc_common.h  glass-CSC fixed-point constants
  src/neon_emu.h        x86 emulation of the NEON intrinsics used (only for checking the NEON logic off-Pi)
  src/main.c            CLI                            src/test_kernels.c  NEON == scalar self-check
  src/rnv2_tables.h     RNV2 Huffman tables (copy of video/dec/rnv2_tables.h)
  ref/rnv2dec.c         unchanged copy of video/dec/rnv2dec.c (md5 f38fe32a...) + its tables, used by make test
  tools/                synth.c (test clip), closed_loop_test.sh, bench_kernels.c, speed_runs.sh, bench.sh,
                        parity.sh + compare.py + npz2raw.py (PC parity runs), pi_estimate.py
  tools/headroom*.{c,sh}  headroom evaluation (see HEADROOM-DRIVEN QUALITY below)
  results/              parity tables (compare_p3 = --exact, compare_p4 = default), speed runs, Pi estimate

BUILD AND SELF-TEST ON THE PI (Raspberry Pi OS Bookworm 64-bit; gcc and make are preinstalled)
  cd pienc
  make                  # rnv2enc, librnv2enc.a, rnv2dec_ref, test_kernels, bench_kernels  (-O3 -mcpu=cortex-a76)
  make test             # 1) every NEON kernel == scalar (200 k randomized calls each + structured sweeps)
                        # 2) synthetic clip: NEON == scalar bitstream, 1 == 3 threads, RGB24 == XRGB8888 input,
                        #    reference decoder output == encoder reconstruction on every frame (CRC32)
  ./bench_kernels       # ns per kernel call, scalar vs NEON (feeds the speed model in REPORT.txt)
  make rnv2enc_count && tools/speed_runs.sh DIR_WITH_RAW_CLIPS     # real Pi timings, 1 and 2 threads
  On x86 Linux the same Makefile builds the scalar encoder; `make test` there runs the NEON logic through
  src/neon_emu.h instead, and `make neoncheck` compile-checks kern_neon.c for aarch64 with clang.

RUN
  rnv2enc -i IN -o OUT [options]
    -i FILE|-            raw frames, 800x480, no headers (stdin with -)
    -o FILE|-            .rnv stream (stdout with -; flushed after every frame)
    --fmt F              rgb24 (default) | bgr24 | bgr0 (= xrgb8888 / DRM XRGB8888 on little-endian, what wlroots
                         and kmsgrab deliver) | rgb0 (= xbgr8888)
    --stride N           bytes per input row (default 800 * bytes per pixel)
    --threads N          worker threads incl. the caller (use 2 on the Pi; output is identical for any N)
    --damage FILE        damage hint, one line per frame: "x,y,w,h x,y,w,h ..."; empty line = nothing changed,
                         "*" = unknown.  Blocks outside the damage become SKIP without any work.  Optional: without
                         it the encoder compares every 8x8 block with the previous frame itself (cheap).
    --recon F / --target F / --crc F / --stats F / --metrics   reconstruction, YUY2 target, per-frame CRC32,
                         per-frame stats, glass PSNR / visible-error metrics (as video/run_eval.py)
    -n N, --skip N, --bench (preload frames, time only the encoder), -q
  Codec settings default to REPORT_v3's recommendation (--qpmin 0 --lamk 0.04 --kappa 4 --thr 3 --rate 8e6
  --cyc 4.49e6 --buf 4).  Real-time preset (default): --me 2 --mewinmax 1024 --rtrc 1.
  --exact switches to the rnv2.py-parity preset (exhaustive search, rnv2.py rate control; ~10-25% slower).
  Pin the encoder away from the compositor if needed: taskset -c 2,3 rnv2enc ... --threads 2
  Speed options: --prune N (default 3; on frames with >= 2500 active blocks skip a residual candidate whose prediction distortion is
  > N x another's, 0 = analyse all), --qppred 0|1 (default 1; raise the first qp when the last classic-point frame predicts a clear
  overrun of the hard cap).  --prune 0 --qppred 0 is bit-identical to the older encoder except on key frames (frame 0 included).
  Key frames: no motion search, no ZRES candidate, half-pel planes reset to black (older builds kept stale ones: decoder mismatch after a key).

CAPTURE OPTIONS (documentation only; nothing here was built or tested on the Pi)
  The encoder only needs a stream of raw 800x480 frames.  Make the desktop output 800x480 (or scale in the capture
  tool).  Each option below ends in "| rnv2enc -i - --fmt bgr0 -o - --threads 2 | <link sender>".
  1. wlroots compositor (Raspberry Pi OS Bookworm uses labwc / wayfire, both wlroots): wf-recorder with raw output.
       wf-recorder -c rawvideo -m rawvideo -x bgr0 -r 60 -f /dev/stdout
     wf-recorder only emits a frame when the screen is damaged; there is an option to request frames continuously
     (-D / --no-damage in current versions).  Check `wf-recorder --help` on the Pi for the exact flag spelling.
     Better long-term: a ~200-line client on wlr-screencopy-unstable-v1 (copy_with_damage) that writes each frame to
     stdout and its damage rectangles to a FIFO for --damage.  That gives zero-copy-ish capture plus the SKIP hint.
  2. KMS/DRM scan-out grab (works without a compositor, e.g. if Hudiy runs fullscreen on KMS/eglfs; needs root):
       ffmpeg -loglevel error -f kmsgrab -framerate 60 -i - -vf 'hwdownload,format=bgr0,scale=800:480' \
              -f rawvideo -pix_fmt bgr0 -
     Only works if the scan-out buffer is linear; the Pi 5 display path may use tiled buffers, in which case
     hwdownload fails.  Test first.
  3. X11 session:  ffmpeg -f x11grab -framerate 60 -video_size 800x480 -i :0 -f rawvideo -pix_fmt bgr0 -
  4. Single frames for tests:  grim -t ppm - | tail -c 1152000 > frame.rgb   (PPM P6 body = RGB24; use --fmt rgb24)
  Constant frame pacing: the RNS-E side shows every decoded frame; if capture only delivers on damage, either repeat
  the last frame (it costs ~0.4 ms and ~10 bytes) or let the link sender pad with empty frames.

LIBRARY USE
  rnv2enc_params p; rnv2enc_default_params(&p); p.threads = 2;
  rnv2enc *e = rnv2enc_create(&p);
  for each frame: rnv2enc_encode(e, pixels, stride, RNV2_BGR0, rects, nrects_or_-1, &out, &len, &info);
                  send 4-byte big-endian len + out
  rnv2enc_destroy(e);
  One encoder per stream; a call must not overlap another call on the same encoder.

HEADROOM-DRIVEN QUALITY (default since r-headroom; --fixedq restores the old fixed operating point bit for bit)
  Owner rule: lossy only when the link lacks headroom.  The encoder walks a 12-step quality ladder (--levels):
    level 0       near-exact: --lamx 0.01 --kappalo 16 --thrlo 0 (the 'thr 0, kappa 16' point; ~9 MB/s on the night map)
    levels 1..10  thr / kappa stay at the classic values, lambda falls geometrically from --lamk (0.04) to --lamlo (0.0025)
    level 11      the classic default point (--lamk 0.04 --kappa 4 --thr 3)
  (thr is a cliff - thr 3 -> 0 doubles the rate - so the smooth part of the ladder is lambda; measured costs in results/headroom.txt.)
  Control, per frame, from capacity and bucket fill only (no timing, so the output is identical for any --threads):
    - allowance = rate_pf + (fill_target * bucket - fill) * 0.35 (fill_target 0.25: three quarters of the bucket stay free
      for scene cuts / scrolls), at least a quarter of rate_pf.  The block analysis is cached per (qp, kappa, thr) and levels
      1..10 differ only in lambda, so the encoder just re-runs the cheap mode decision (binary search, ~4 passes) and takes
      the FINEST level whose frame size fits the allowance (and the decode-cycle cap).  A quiet frame therefore gets the
      finest level, a burst frame a coarser one, and the bucket sits near fill_target in sustained load.
    - hysteresis: coarser levels apply at once; a level finer than the previous frame's must fit within 0.85 x allowance
      (--hyst), may be at most one step finer, and only after the level has been held --dwell (4) frames.
    - level 0 (thr 0 re-analysis, ~1.5 ms extra) is tried only from level 1 when that frame needs < 40% of the allowance;
      if it does not fit, it is not retried for 30 frames.  Once there, it is kept while it fits.
    - if even level 11 (the classic point) exceeds the soft cap, the old rules take over unchanged: mu (decode cycles, --cyc stays
      authoritative), then qp, rt_rc acceptance within the hard bucket cap.  A cycle-cap violation is a misfit at every level.
    - static (unchanged, SKIP) blocks are re-evaluated whenever the level gets finer than the one they were last judged at,
      so a quality gain also refines still pictures; otherwise they stay forced-SKIP (free).
  Link capacity input (for the streamer in pi/avlink/):
    --linkcap B/s        fixed, bytes per second of payload the link can carry (replaces --rate)
    --linkctl PATH       control channel: FIFO (mkfifo; the encoder opens it O_NONBLOCK, no writer needed) or a regular file the
                         streamer appends to (a file that shrinks is re-read from the start)
    --linkfd N           the same on an inherited fd (pipe), e.g. `rnv2enc ... --linkfd 3 3<fifo`
    Line protocol, ASCII, one update per line:   <bytes_per_s>
   or   cap <bytes_per_s>
    ('#' lines and junk ignored)
    The encoder drains all complete lines before each frame; the last one wins.  Send a line whenever the measured link
    goodput changes materially (e.g. every 250 ms, or on a >10% change): a decrease takes effect on the next frame
    (the fill is clamped to the new, smaller bucket), an increase just raises the allowance (levels get finer one step per frame).
    Library: rnv2enc_set_linkcap(enc, bytes_per_s) from any thread (applied at the start of the next rnv2enc_encode);
    rnv2enc_params.link_cap for a fixed value; rnv2enc_info.level / .link_cap report what was used.
    The stream is a pure function of the frames and the cap sequence (cap lines read before frame N apply to frame N), so a
    --linkcap run or a --linkctl file with the same cap on its last line is reproducible bit for bit.
  Other options: --fixedq (no ladder), --levels N, --lamlo, --lamx, --kappalo, --thrlo, --filltarget X, --hyst X, --dwell N, --pinlevel L
    (hold one level, for measuring).  --stats gets two more columns: level, linkcap.
  Result (night map turn_night, vs the old default at 2.4 MB/s / 3.2% codec error): cap 3 -> 3.0 MB/s, 2.3%; cap 5 -> 4.9 MB/s, 1.0%;
  cap 8 -> 7.2 MB/s, 0.35% (near-exact needs ~9 MB/s there); all within ~1.5% of the cap, cycle cap never exceeded.  Full table incl. day map,
  Android Auto clips and flicker: results/headroom.txt.
  Evaluation: tools/headroom_run.sh (needs raw clips, tools/npz2raw.py) -> results/headroom.txt; metrics by tools/headroom_eval.c
    (decode with ./rnv2dec_ref, RGB666 glass compare with the source, flicker on static pixels).
