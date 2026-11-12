# A/V over SD: deployment reference

For building a Pi-side update tool. State as of 2026-11-06, read from the Pi and the unit that day. Paths under "repo" are relative to `analysis/av_over_sd_v1/`.

## The chain

```
Hudiy (labwc headless NOOP-1) -> wlcap -> rnv3enc -> avlink -> USB -> Teensy (SD emulator) -> SD slot -> RNS-E sdstream -> display + DSP
pw-record (hudiy_equalizer_sink) ------------------------^
                  avlink --key request (key.fifo)--> rnv3enc
```

## What is deployed now

| Part | Where it runs | File | Identity |
|---|---|---|---|
| Encoder | Pi | `~/avsd/current/bin/rnv3enc` | md5 `250e0d4a...`, repo `RNS-E-Hudiy` `avsd/chain/enc3`, commit `890e53c` |
| Link | Pi | `~/avsd/current/bin/avlink` | md5 `07613fb5...` (changes between builds of the same source: `-g` records the build path), repo `avsd/chain/avlink`, commit `890e53c`. `~/avsd/pi/avlink` is no longer used by the chain |
| Capture | Pi | `~/avsd/pi/wlcap/wlcap` | md5 `e2a5e72a...` |
| Flags | Pi | `~/avsd/hud2.sh` | gain changed to -9 dB after this list was made, so the md5 is new; previous version `hud2.sh.bak-gain` (md5 `f2fee3ff...`) |
| Supervisor | Pi | `~/avsd/car/avsd_run.sh`, `avsd-chain.service` | md5 `ede13959...` (run script) |
| Teensy firmware | Teensy 4.1 | on Pi as `~/avsd/teensy/firmware.hex` | sha256 `c1ba0665...`, the same hash as repo `teensy/build_resume/teensy41/firmware.hex`. Notes say this hex is flashed; not read back from the Teensy |
| Decoder and stream | RNS-E flash sector 96 | `analysis/r61_car_v1/out/cand_f85bc47a/sectors/sector_96.bin` | live CRC32 `0696DDE4` |

Other RNS-E sectors, no change needed for encoder work (from the 2026-11-06 notes, only 44 re-read today): 44 `79B684C9`, 53 `415E0921`, 88 `687F7545`, 89 `B0AE1131`, 90 `0F192CC6`.

**Do not flash sector 53 from an r61 candidate.** Candidates carry an older 53 (`3F4BE504`); the unit's has the Doom knob fix. Sector 96 is the only one a decoder change touches.

Live flags, set as defaults in `hud2.sh`:
```
rnv3enc -i - --fmt bgr0 -o - --keyint 0 --linkcap 4.5e6 --linkctl key.fifo --stats st.txt --threads 2 --maxvid 161 --cyc 4.05e6 --refill wall
avlink --video - --audio pcm.fifo --dev <by-id> --keyctl key.fifo --maxvid 161 --whole-sf 1 --gain-db -9 --stats 5 --console
```
`hud2.sh` env overrides: `ENC_BIN`, `AVLINK_BIN`, `WLCAP_BIN`, `KEYINT`, `LINKCAP`, `MAXVID`, `ENCTHREADS`, `ENCFLAGS`, `GAIN`, `RTPRIO`, `CHAINDIR`, `LINKDEV`, `CAPMODE`, `NORANDR`.

**r64 disc, burned 2026-11-06:** `build/193PU-r64.iso`, sha256 `3f95ec7a...`, SOFIDX 0720. Adds to the table above: 24-tap windowed-sinc rate match in place of the cubic, a bigger audio buffer (162.5 ms, was 62.5), longer fades, crossfade over lost audio, a frame-drop fix. Modelled sector CRCs after a disc install: 96 `803DD0C5` (a CAN flash of the same module, `cand_4b8666ec`, reads `CF7D2122`; only the unused tail differs), 90 `DD970956`, the others as above. Proof: `analysis/r64_disc_v1/out/proof.txt`.

**r62 is a bad disc; don't install it.** It silently stops during its read, with DOOM's checksum wrong. r63 was never burned. The unit's pre-install check at 01C0BACE sums each module over its whole allocation: the build tool must check that, not just the declared size.

## Compatibility rule

The current public checkout can read the installed sector 96 CRC with
`python3 rnse/can/decoder_check.py --interface can0` (Pi), or
`python -I rnse/can/decoder_check.py --transport cowcan` (PC). It displays the installed CRC;
add `--json` for a front end.
This is a read-only whole-sector B1 read, not the private `tools/sd_flash.py check` referenced in
the historical procedure below; that flash tool is not included here. See the root README for usage.
An unknown CRC means an unrecognized sector image, not necessarily an incompatible decoder.
The full-sector CRC includes unused tail bytes, so releases installed through different paths may
need more than one accepted CRC (as with r64 above).

| Decoder on the unit | Encoder stream | Result |
|---|---|---|
| New (`0696DDE4`) | R3 or `--r3 0` R2 | Works |
| Old (`29D3E06A` or earlier) | R2 | Works |
| Old | R3 | Every frame rejected on its magic. No video |

So when a format changes: **update the RNS-E first, then the Pi. On rollback: the Pi first, then the RNS-E.** The tool should know which encoder needs which minimum decoder CRC. A frame's magic is bytes 0-1 of the frame (`R2` or `R3`). The R3 reconstruction definition is in repo `enc3/src/rnv3_modes.h`.

## Pi update steps

### Stop and start the chain
```bash
touch ~/avsd/car/DISABLE; systemctl --user stop avsd-chain      # stop; DISABLE keeps it down across a reboot
rm -f ~/avsd/car/DISABLE; systemctl --user start avsd-chain     # start
```
- `avsd-chain` is a user unit (linger on), `Restart=always`. A `pkill` of one part makes the supervisor restart the whole chain.
- The supervisor waits for wayland, pulse, NOOP-1, hudiy, the sink, the Teensy and a free serial port, then runs `~/avsd/hud2.sh`. It exits and restarts if any part dies or `live.log` goes silent for 20 s.
- Logs are on tmpfs: `/run/user/1000/avsd/{live.log,enc.log,cap.log}`. Check with `~/avsd/car/avsd_status.sh`.
- Don't restart hudiy, labwc or pipewire. Use `pkill -x`, since `pkill -f` matches your own ssh command.

### Encoder
Build on the Pi (about a minute) or ship an aarch64 binary:
```bash
gcc -O3 -mcpu=cortex-a76 -std=c11 -ffp-contract=off -D_POSIX_C_SOURCE=200809L -Iinclude -Isrc -w \
    src/main.c src/enc.c src/kern_scalar.c src/kern_neon.c -o rnv3enc -lpthread -lm
```
Source: `rnse-av-sd` repo `pienc/src`, `pienc/include` (`enc3/` in the working tree). Install to a new directory and point `hud2.sh` at it, so rollback only changes one line. Check that a new build gives the expected stream on a test clip: `--crc` output must match between builds that claim to be identical.

### avlink, wlcap
Sources: repo `pi/avlink` (has a Makefile, build on the Pi), `pi/wlcap`. Replace the binary with the chain down, keeping a `.bak-<date>`.

### hud2.sh and car scripts
- `~/avsd/hud2.sh` is the only place the chain's flags live, and the service runs that one.
- `~/avsd/car/hud2.sh` is a stale copy that nothing runs. Repo `pi/scripts/hud2.sh` now matches the live one.
- `install_car.sh` still checks that `~/avsd/pienc/rnv2enc` exists. That old binary is still there, so it passes; delete it and the check fails. It also needs no change to run the new encoder.
- `avsd_run.sh` prints `rnv2enc gone` for whichever encoder dies (cosmetic). It tracks `enc.pid`, not the name, so the new binary is watched correctly.

### Teensy firmware
Chain down, because the port must be free:
```bash
sudo python3 ~/avsd/teensy/hkflash_linux.py firmware.hex
```
- Older hexes are kept beside it (`firmware.hex.bak-*`).
- Afterwards the RNS-E must re-read the card: on the bench, `benchd.py reboot` (RAM only). In the car, a power cycle.
- Console when avlink is stopped: `~/avsd/teensy/tc.py "cmd"`.
- `teensy_loader_cli` from the PC often fails after a soft reboot. Use this script.

## RNS-E flash

**The update tool can't do this yet.** Today the procedure runs on a PC (`tools/sd_flash.py`) with the CowCAN-FT adapter on the bus: it stages over SD through Pi and Teensy, over ssh, then commits over CAN. A Pi-side version has to port these parts to `can0`:

| Step | Where | Notes |
|---|---|---|
| Upload bootstrap, stage the image, verify CRC | `tools/sd_flash.py`, `analysis/r61_car_v1/sdstage` | Through the Teensy. RAM only |
| B3 END, B1 samples, B7 + B6 commit | `tools/cowcan_flash.py` (`commit_multi`), `CAN_PROTOCOL.md` at the project root | Erases, programs, one cold reset |
| Bus speed and mode | 100 kbit/s | Owner: `can0` is listen-only only while the ignition is off (seen as `LISTEN-ONLY` with ignition off, 2026-11-06). So a CAN commit from the Pi needs ignition on, and the tool must check the mode first and refuse otherwise. `~/rns-e_can/can_base_function.py` manages the mode and other services use `can0`, so the tool has to coexist with them |

Current PC procedure, about 6 s plus the reset (run from the project root, `C` = the candidate's `sectors` folder, PY = `...\Python312\python.exe`):
```
ssh hudiy 'touch ~/avsd/car/DISABLE; systemctl --user stop avsd-chain'
PY -I analysis\av_over_sd_v1\rnse_proto\benchd.py quit
PY -I tools\sd_flash.py check  --target 96=<current image>            # read only, confirms what's on it
PY -I tools\sd_flash.py commit --target 96=C\sector_96.bin --i-understand-this-can-brick-the-unit
                                                                      # "no B6 ack" is normal; wait about 25 s
PY -I tools\cowcan_flash.py bootstate                                 # restart ct must be 0
PY -I tools\sd_flash.py check  --target 96=C\sector_96.bin            # live CRC must match
ssh hudiy 'rm -f ~/avsd/car/DISABLE; systemctl --user start avsd-chain'
```

**Auto start stays off after any `sd_flash.py` run.** It writes a RAM off word that survives the commit's reset, and clears only at power-off. Turn it back on with `benchd.py serve` running, then `analysis\r61_car_v1\tva_bench.py on`, or power-cycle the unit. A Pi tool must do the same: write 0 to `8F72F004`.

Building a candidate:
1. In WSL, in `rnse_proto`: `bash build.sh`. The `decbench` link is commented out, because it no longer fits its window with the bigger decoder.
2. `PY analysis\r61_car_v1\rebuild.py`. It refuses a stale `sdstream.bin`. Output goes to `analysis/r61_car_v1/out/cand_<sha8>/`.
3. `sim\sim_r61.py` must end `ALL OK`.

SH decoder sources are `rnse_proto/src/rnv2sh.c`, `rnv2sh.h`, `rnv3_sh.inc`, with pre-RNV3 originals as `*.before_rnv3`. `enc3/tools/patch_sh.py` regenerates `rnv2sh.c` and `rnv2sh.h` under `enc3/sh/`, but no longer holds the optimised `rnv3_sh.inc`: that lives in `enc3/sh/` and must be copied across by hand.

## Health check after any update

| Check | Where | Good |
|---|---|---|
| Stream running | `live.log`: `rd_stalled`, `served` | `rd_stalled=0`, `served` rising (only while on TV) |
| Frames flowing | `live.log`: `frames`, `drop`, `waitkey` | `frames` rising, `drop=0`, `waitkey=0` |
| Key requests | `keyreq` | Rises on each TV entry; shouldn't climb steadily while watching |
| Unit key requests | `unit_keyreq` | The RNS-E's decoder waited > 250 ms for a key. Can rise on TV entry; climbing while watching = losses only the unit saw |
| Encoder limits | `enc.log` | No `rnv3enc: frame N over ...` lines |
| Unit decode | `benchd.py sdstream status` (needs CAN) | Decode errors 0, CRC bad 0, underruns 0 |

`rd_stalled=1`, `waitkey=1` and `vdiscarded` rising are normal whenever the unit isn't on TV, is off, or is asleep.

## Rollback

| Part | How |
|---|---|
| Encoder and flags | `cp ~/avsd/hud2.sh.bak-prernv3 ~/avsd/hud2.sh`, restart the chain. Do this before downgrading the decoder |
| Decoder | Sector 96 from `analysis/r61_car_v1/out/cand_f550031c/sectors/sector_96.bin`, CRC `29D3E06A` |
| Unit off auto start until power-off | `tva_bench.py off` |
| RNS-E unbootable | Stock or r-series disc install (see `build/`, `tools/burn_*.ps1`) |

## What a Pi update tool should have
- A manifest per release: file hashes, the minimum decoder CRC, build flags.
- Hash check before install, install to a new directory, keep the previous version, and switch with one line in `hud2.sh`.
- Order enforced by the compatibility rule above.
- Chain down during Teensy flash and any RNS-E staging.
- Health check after the switch, with automatic rollback if `frames` doesn't rise or the encoder dies.
- Awareness that a healthy stream can only be confirmed when the unit is on TV. Otherwise it should report "not verifiable now", not failure.

## Audio gain (reviewed 2026-11-06)
Current: `GAIN=-3` in `hud2.sh`, capturing from `echo_cancel_sink`, which follows Hudiy's volume (measured) and, by the PipeWire graph, its EQ (not measured with a non-flat EQ). Falls back to `hud2.sh`'s old tap if that sink is missing. Not yet heard in the car.

| Stage | Level | Status |
|---|---|---|
| Android Auto to Hudiy | Fixed, no phone volume effect (owner). Several tracks over 2 min peaked -5.5 to -7 dBFS, rms about -16, with no clicks or dropouts (owner's player and settings; possibly loudness-normalised) | Measured |
| Hudiy sink volume and EQ | 0 dB at defaults | Measured |
| `avlink --gain-db` | -3 dB. **Above 0 wraps into harsh distortion: there is no clip code** | Code |
| Rate match on the unit | 0 dB, clamped. Overshoot up to +1.9 dB | Code, inferred |
| DSP IIS scaler | Coefficient -9.5 dB; real gain -3.5 or +2.5 dB depending on an unread shifter | Firmware |
| DSP volume | Depends on the knob. Clip onset for a full-scale track depends on the same unknowns | Inferred |

- **Ceiling for `GAIN`: -2.5 dB,** so the DSP scaler stays clean under either shifter. Use -3 at most, and -6 if any tone or EQ boost is on, because boosts have no pre-EQ headroom from volume 16 up (datasheet).
- **FM is about 20 dB louder at the old -18 setting:** 1.7 dB from the scaler plus 18 dB from our own cut (firmware and code). FM's real digital level is unknown.
- **A UI must cap `GAIN` at -3,** and shouldn't offer positive values unless saturation is added to `avlink`.

Logs now persist in `~/avsd/logs/<date>-<boot id>-{live,enc}.log`, newest 40 runs, added to `car/avsd_run.sh`. Not yet run under the real supervisor.

## Settings
**File:** `~/avsd/avsd.env`, plain `NAME=value` lines, read by `hud2.sh` at chain start (template: repo `pi/scripts/avsd.env`). It's sourced as shell, so a UI must validate values and write nothing else. **Apply:** `systemctl --user restart avsd-chain`, a few seconds of no stream. Delete a line to get the default. Checked in a dry run with `GAIN` and `DITHER`; the rest use the same mechanism.

Ranges are what's been tested or reasoned out, not hardware limits.

| Setting | Default | Safe range | What it does | Risk outside the range |
|---|---|---|---|---|
| `GAIN` | -3 | -30 to -3 dB | Audio level sent to the RNS-E | Above -2.5 the DSP may clip; above 0 wraps into harsh distortion |
| `LINKCAP` | 4.5e6 | 3.2e6 to 4.5e6 B/s | Video rate target | The reader was last proven at 4.56 MB/s; over that, stutter or audio gaps |
| `MAXVID` | 161 | 130 to 161 sectors | Largest video frame per superframe (x512 B). Also passed to avlink | Hard limit: the unit's 170-sector slot. Over 161 breaks the stream |
| `CYC` | 4.05e6 | 3.0e6 to 4.05e6 | Decode-time cap. Lower is steadier audio, higher is better heavy-screen picture | 5.5e6 gave audio underruns on the old format. The first thing to lower for skips |
| `KEYINT` | 0 | 0, or 240 and up (frames) | Periodic full refresh. 0 = only when the link asks | Short intervals make the picture go soft on each refresh |
| `ENCTHREADS` | 2 | 2 to 3 | Encoder threads at real-time priority | 3 gains almost nothing and takes more CPU from Hudiy |
| `DITHER` | 0 | 0 or 1 | Screen-locked dither on smooth areas. Costs decode time | No owner verdict yet on this codec |
| `SRCDEAD` | 3 | 0 to 6 levels | Ignore source changes this small, which stops noise flicker | 0 brings corner flicker back; high values hold stale colour |
| `SRCMEAN` | 48 | 12 to 64 | The block mean must stay within this (1/64 level) to count as noise | Low gives more updates; high holds fades back |
| `FILLPEN` | 2 | 1 to 4 | Penalty against flat 8x8 fills. Higher means fewer mosaics, lower reaches better worst-frame numbers | Under review by the encoder session; expect this to change |
| `RTPRIO` | 5 | 0 to 5 | Real-time priority of the chain, 0 = off | Off risks video stutter; may help Hudiy audio, untested |

Not `avsd.env` settings, or not for a UI:
- **Hudiy volume and EQ** work through the chain now. Boosts eat headroom, so warn at `GAIN` above -6.
- **Live with no restart:** `printf WSF1 > /tmp/avlink_sdctl` (or `WSF0`) sets whole-superframe mode. Leave it on.
- **Auto start on TV:** RAM word `8F72F004`, 0 = on, `4F464621` = off until power-off. CAN only (`tva_bench.py on|off`).
- **Set by the supervisor, leave alone:** `LINKDEV`, `CHAINDIR`, `AVSD_STALE_S`. **Paths for updates:** `ENC_BIN`, `AVLINK_BIN`, `WLCAP_BIN`.
- **Leave alone:** `WHOLESF`, `STATS`, `CAPMODE`, `AUDIOCAP`, and `ENCFLAGS`, which replaces the encoder's flag block and silently ignores `CYC`, `DITHER`, `SRCDEAD`, `SRCMEAN`, `FILLPEN`.
- **`ENCEXTRA`** appends raw encoder flags. It's for experiments, since the encoder has about 80 flags and most are test or legacy options (full list in `enc3/src/main.c`). `--r3 0` must never be set by accident; new-format streams need the decoder, and this one is the safe direction, but it brings back mosaics.
- **Where the Pi side lives now:** `RNS-E-Hudiy/avsd`, built into `~/avsd/releases` by its `install.sh`. Change the repo first, and install from there. The files in this tree are the working copy, and its `hud2.sh` still has the old paths.
- **Light mode (`ENCEXTRA="--tonepal 3"`, needs an encoder from commit `890e53c` or later):** flat pale colours on screen, such as Maps land, each get one target at the panel levels of that colour, placed mid-level, so the phone stream's speckle goes and the tint stays. `T` is how far a pixel may be from the colour, in codes: 3 is tested, 2 leaves more speckle. 0 or no flag is off, and the stream is byte-identical to the old encoder (compared on `aa_launcher`). No extra encode time on the Pi. No Teensy or RNS-E change, and no format change. Measured on a real capture, not yet seen on the panel. `--tonelock` and `--srcdeadpale` are removed, and the encoder rejects an unknown flag, so an `avsd.env` that has them must change.
- **Transitions (`--nurelax 0.5 --nurelaxthr 0.97` in `ENCEXTRA`, on since commit `eea4a38`):** lowers the decode-cycle price while a frame has cycles left, so frames use the cap instead of stopping at 3.0-3.3 of 4.05 Mcyc. No format change, no flash. Costs 0.5-1.1 ms per frame on the Pi (Ken Burns mean 12.8, max 24.7 ms). Off: remove both flags from `~/avsd/avsd.env`.
- **Reader overload (`avlink --linkcap`, on since the back-off commit, `hud2.sh` passes `LINKCAP`):** `avlink` lowers the encoder cap 15% per 100 ms while 4+ superframes are unread, floor 55% of `LINKCAP`, then raises it 3% per 500 ms after 1 s calm. Audio left on the Pi goes out in early ticks when the reader has room. `STAT` adds `cap_kb`, `backoffs`, `catchups`. No flash. Test: `pi/avlink/test/test_backoff.py`. In the suite `over_3k` (reader limited by a fixed sleep) exceeds its 500 ms audio limit since HIGH went to 250 ms, on the old build too.
- **avlink audio (2026-11-06):** source backlog is kept up to 250 ms instead of 60 ms, and tick rate follows the source by up to +-0.72% instead of 0.18%. To check in `~/avsd/logs/*-live.log`: `silence` and `adrop` should stay flat while playing, apart from startup and source switches. No settings.
- **Encoder flags are still changing** in the encoder session, so re-check this table when its work lands.

## Notes for a settings and update UI
- **The unit now lives in the car, with no spare.** RNS-E updates have to work there, which makes the `can0` port the main job. A failed flash means the disc path, so the tool should run every CRC check before commit, as `sd_flash.py` does.
- **Change one thing at a time,** and keep a known-good set the UI can restore in one action.
- **`--cyc` is a safety limit, not a quality slider.** Too high starves the unit's audio, and the Pi can't see that (next point).
- **The Pi can't see the unit's decode or underrun counters** from its own logs. They're at `8F720100` (`STAT_BASE`) over CAN B1 (request `0x7B0`, reply `0x7B6`), layout in `rnse_proto/src/sdstream.c`. A UI could poll them over `can0` with the ignition on. Not built or tried.
- **Logs are on tmpfs, so a reboot loses them.** That's how a car session's logs were lost on 2026-11-06. A persistent log is planned; the UI should show or export it.
- **The chain sits waiting with no Teensy on USB,** and the service still shows `active`. Show the supervisor's wait state, not just the unit status.
- **Reaching the Pi:** Wi-Fi AP `Hudiy`, `<pi-address>`, ssh key `~/.ssh/id_ed25519_hudiy`. A PC joins using its saved profile and may take several seconds.
- **`avsd_status.sh` prints `throttled`.** Owner: the Pi is configured not to throttle on under-voltage, so don't surface that value as a fault without rechecking what it means there.

## Not verified
- Anything in the car on this build: ignition, sleep and wake, TV exit and re-entry over real drives, and the car DAC.
- `can0` flashing from the Pi has never been done.
- Whether the Teensy really runs `c1ba0665`: from notes, not read back.
- Recovery after a real SD power cut.
- `--keyint 0` long term: key requests worked in a short bench session (`keyreq=24`), with no periodic key as a safety net.
