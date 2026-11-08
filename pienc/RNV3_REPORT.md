# RNV3 encoder and decoder: offline results, 2026-11-06

**Update, same day: flashed and deployed at the owner's order (sector 96 CRC `0696DDE4`, Pi `~/avsd/pienc_rnv3/rnv3enc`). Owner on the panel: works, "bit blocky on transitions". Next work: `HANDOFF_NEXT.md`.** Sections below were written before that. Their numbers come from PC decodes against the q666 glass model, which is unverified on glass.

## Update: link use (fill pass added, recommended `--linkcap 4.5e6`)
The 3.06 MB/s in the table below averages in idle frames, where the picture already matches at the panel. It also hid a real leak: frame size jumps with lambda, so the search stopped around 50-60 KB of 82 KB on up to 45% of frames. A fill pass (`--fill 1`, default) now moves blocks to a finer decision, best gain per bit first, until the frame is full. Format unchanged; the fill build was re-checked bit-exact on 2 clips (mappan30, video).

Link use while there is something to send, MB/s (mean of non-idle frames):
- 4.8e6 cap: 4.7-4.84 on nearly every clip. Exceptions are clips with little to send, such as the gradients (grad_h264 3.96, graddark 2.97), k_real 4.09, stills 3.88.
- 4.5e6 cap: about 4.53 on continuous motion (Ken Burns, video, map pan, rotate, map3d), peak second about 4.57 on the two clips checked. Max frame stays under one superframe.

**Bytes are no longer the limit.** All-clip mean, worst-frame mean and worst-frame SSIM are the same at 4.5 and 4.8 (44.9 / 39.3 dB / 0.951). 4.5 costs 0.1-0.2 dB at most, with a slightly higher share of off pixels (3.95% vs 3.84%). 4.84 sustained is also above the 4.56 MB/s the real reader was last proven at, so I recommend 4.5.

**Decode cycles are the limit on heavy content.** At `--linkcap 4.5e6`, cycle cap 4.05 vs 5.0 Mcyc (mean / worst frame dB):

| Clip | 4.05 | 5.0 |
|---|---|---|
| kenburns_h264 | 34.2 / 27.6 | 36.1 / 29.0 (off pixels 36% to 26%, flips 37% to 51%) |
| video_h264 | 40.9 / 29.7 | 41.9 / 30.8 |
| zoomfade_h264 | 44.3 / 34.6 | 44.8 / 36.8 |
| mappan_h264 | 47.9 / 42.6 | 48.3 / 46.1 |

5.0 Mcyc is about the 14.5 ms in-stream limit by the earlier fit, extrapolated, and on RNV2 5.5 Mcyc gave 11 audio underruns on smap. Not safe without a bench run. The other route is a cheaper decoder: the agent's measurement left about 225 instructions for LAT8 parse, 270 for put and 170 in the main loop, with spills GCC won't remove.

The clip tables below predate the fill pass and the speedup. The videos in `cmp/` are from the final build. Final tables at 4.5, speedup merged: `results/live_vs_final.txt`, `results/required_tests_final.txt`, `results/frame_limits_final.txt`. All-clip means, live vs final: PSNR 44.5 vs 44.9 dB, worst frame 25.5 vs 39.5 dB, worst-frame SSIM 0.797 vs 0.952, off pixels 6.22% vs 3.89%, frames over a superframe 3.96 vs 0.

Checks on the final merged build: reference decoder and SH decoder source (host build) match the encoder on 6 clips at 180 frames, with and without dither. Pi NEON and x86 give identical frame CRCs on 4 clips, plus Ken Burns with dither. `test_kernels` passes in x86 emulation; not re-run on the Pi after the merge. Pre-merge source is in `src_pre_speed/`, agent diffs in `results/*.diff`.

## What was wrong
1. **Rate control left the link unused.** On transitions and natural content the live encoder uses 1.2-2.7 MB/s of 4.8, with qp swinging as high as 39 on 10 KB frames. It then spills up to 294 KB on bursts, which stalls the picture for several ticks. It is not dropping anything: live counters over 37 minutes showed drop=0, late=0, 3 video frames discarded.
2. **The only cheap block was a flat fill.** When a frame is over budget, blocks collapse to 8x8 fills. That is the mosaic you called classic compression look.
3. **A periodic key can re-code the whole screen on a still one.** An old live log (5 Nov) shows a run of 28-byte frames, then a key of 102 KB with 5326 INTRA and 674 RAW blocks. I did not measure how visible that is on current flags.

## What changed
Format RNV3 (magic `R3`). RNV2 modes stay. New block modes:

| Mode | What it is | Bits | SH-4A instructions per block (qemu) |
|---|---|---|---|
| LAT8 | soft block: 4 corner points shared with neighbours, bilinear | a few | 628 (704 with dither) |
| LAT4 | soft block: 3x3 points | more | 860 (926 with dither) |
| LAT2 | soft block: 5x5 luma, 3x3 chroma | most | 1891 |
| BTC2 | 2 tones, 1 bit per pixel: text and icons | 64 + colours | 636 |
| BTC4 | 4 tones on a line, 2 bits per pixel | 128 + colours | 835 |
| RAW, for reference | | 1024 | 230 |

For scale, the cycle cap of 4.05 Mcyc is 675 per block with the model's 306 for stores added to these counts. LAT8 costs 934, so a whole frame of LAT8 does not fit the cap. The encoder mixes it with MC and SKIP. LAT2 is too expensive to be chosen much.

Encoder:
- Every frame fits one superframe, keys included. Lambda holds the bytes, nu holds the decode cycles.
- A burst goes soft first and sharpens over the next frames.
- A block stops being coded once it matches the target at the panel (q666).
- Source deadband: a change within 3 levels is ignored unless the block's mean moves.
- A flat fill on a non-flat block carries a distortion penalty.
- Smooth blocks get steered more bits. Tone blocks only go where a block really is two-toned.
- `--dither 1` is optional: a screen-locked 2x2 pattern the decoder adds to LAT8 and LAT4, plain colour conversion in smooth blocks, none near black.
- `--r3 0` writes plain RNV2 with the new rate control, which needs no flash, but keeps the mosaic on the worst frames.

## Results: live b16bd396 vs new, all 27 clips (`results/live_vs_new.txt`)
Same flags otherwise (`--linkcap 4.8e6 --maxvid 161 --cyc 4.05e6`). A spilled frame of the live encoder holds the old picture for those ticks.

| Mean over clips | Live | New |
|---|---|---|
| Glass PSNR, dB | 44.5 | 44.8 |
| Worst frame, dB | 25.5 | 38.8 |
| Worst frame, blurred-eye metric, dB | 26.6 | 40.0 |
| Worst-frame SSIM | 0.797 | 0.948 |
| Pixels off by 2+ panel steps, % | 6.22 | 4.02 |
| Frames over one superframe, per clip | 3.96 | 0 |
| Link used, MB/s | 2.25 | 3.06 |

Selected clips, mean / worst frame dB:

| Clip | Live | New |
|---|---|---|
| kenburns_h264 (photo-like, every pixel moving) | 31.8 / 26.7 | 34.2 / 27.4 |
| zoomfade_h264 (app open) | 44.1 / 17.8 | 44.2 / 34.2 |
| swipe_h264 | 49.1 / 7.8 | 48.6 / 35.8 |
| list_h264 | 47.1 / 15.4 | 48.7 / 44.0 |
| mappan_h264 | 46.9 / 18.2 | 48.0 / 42.4 |
| k_real (your real capture) | 44.4 / 21.9 | 44.0 / 37.1 |

## Where it is worse or unproven
- **The handoff's "never worse than b16bd396 at any time" does not hold.** Some frames are more than 1 dB under live at the same tick: swipe 85 frames (worst -5.3 dB), zoomfade_h264 31 (-5.3), list_h264 54 (-3.9), and the moving gradient clips most frames (about -2 dB). The transition cases are the trade made on purpose: live is sharper on some frames because it spills, at the cost of stalls and mosaic. Full table: `results/required_tests.txt`.
- **Moving gradients:** grad_h264 47.3 dB against 48.7, grid metric 0.23 against 0.11, at about twice the bytes. Deadband plus RAW-heavy choices. Rare content, not solved.
- **Static-source flips are not better everywhere:** kenburns_h264 36.5% against 27.9%, grad_h264 3.36 against 2.42. On UI they fall: k_real 0.51 against 0.67, stills 0.34 against 1.03.
- **Dither is not a clear win in these numbers.** It removes the grid on gradients (grid metric 0.22 to 0.015 on grad, measured on an earlier build), but static-source flips rise on some UI clips (aa_launcher 1.51 against 0.48). Its PSNR drop is by design, since it is scored against the undithered source. Needs your eyes.
- **Decode time on the real unit is unmeasured.** Mode costs are qemu instruction counts plus the old store estimate. All 15 test streams and the mixed clips decode bit-exact on the host build of the SH source, but it has never run on an SH-4A.
- **Pi encode time now fits 60 fps on average, with spikes.** Speedup merged 2026-11-06, file input, 2 threads, live chain idle beside it, one run each. Mean / max ms per frame:

  | Clip | Before | After |
  |---|---|---|
  | Ken Burns, H.264 source | 26.1 / 66 | 14.2 / 25 |
  | Ken Burns, clean source (colour worst case) | not timed | 16.3 / 28 (18.3 / 30 with dither) |
  | Map pan | 19.0 / 47 | 11.8 / 28 |
  | aa_map3d | not timed | 10.3 / 32 |
  | List | 12.9 / 35 | 8.2 / 19 |
  | Zoom/fade | 11.2 / 55 | 7.1 / 28 |
  | k_real segment | not timed | 3.2 / 19 |

  3 threads gives no real gain (Ken Burns 13.2, map pan 12.0), so stay at 2. Single frames above 16.7 ms still happen, and the capture would drop a frame then. Not tested: the live capture path, Hudiy and its H.264 decode competing for the cores, and `--refill wall`. The motion-search change reduces half-pel use; its effect on decode time is not measured beyond the model cap holding.
- **Test content is mostly synthetic.** k_real is the only real motion capture, and it is a slow selector move.
- **Periodic keys:** the new encoder's first frame and any key are soft for a few frames. I recommend `--keyint 0`, leaving recovery to the decoder's key request. That request path was not exercised here.

## Size
`sdstream.bin` 66,548 B with the new decoder (42,636 before), 94,708 B with bss in a 128 KB window: fits sector 96 by size. `decbench` no longer fits its 0x20000 window and needs a larger one or the plain decoders removed. No flash candidate has been built with `analysis/r61_car_v1/rebuild.py`.

## Files
- `src/`, `include/`: encoder (copy of b16bd396 plus changes). `src/rnv3_modes.h` is the shared reconstruction, `src/rnv3_enc.inc` the new encoder code.
- `dec/rnv3dec.c`: reference decoder (reads R2 and R3).
- `sh/rnv3_sh.inc`, `sh/rnv2sh.c`: SH-4A decoder. `sh/sim/`: qemu instruction harness and tables.
- `tools/`: `gen_clips.py`, `score.c`, `ev.sh`, `cmp.py`, `required.py`, `view.py`, `mkcmp.py`. The `patch*.py` scripts are one-shot and already applied; do not rerun them.
- `cmp/*.mp4`: source, live, new, new + dither at the q666 glass model.
- PC clips and outputs: WSL `~/rnv`. Pi: `~/avsd/enc3` (source, test binary, 4 clips of 172 MB each), safe to delete.

## Test on the real thing
1. **No flash:** `--r3 0` with the new encoder on the current unit. Gets the rate control and never spilling.
2. **RAM test of the decoder**, as the earlier reader tests were done, then a short run on the panel. Measure decode ms in stream: the limit is about 14.5 ms.
3. **Flash sector 96** only if that holds. Then try `--dither 1` as a separate step.
