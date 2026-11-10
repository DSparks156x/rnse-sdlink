# sdlink

Experimental: audio and video for an Audi RNS-E (SH7773 "193PU" head unit) delivered through its SD card slot.
A Teensy 4.1 pretends to be an SD card; a program on the RNS-E reads A/V "superframes" from it, decodes them and drives the screen
and the audio sink on the TV source. Full 48khz 16bit audio uncompressed, compressed 480p60 with custom encoder RNSV3


## Hardware
- Raspberry Pi 5 (aarch64, Pi OS Bookworm) running a Wayland compositor (labwc) with Hudiy / Android Auto
- Teensy 4.1 wired to the RNS-E SD slot (FlexIO2 + eDMA), plus an optional GT911 touch panel passthrough
- Teensy USB to the Pi (`/dev/ttyACM0`)
- Audi RNS-E with a modified firmware lineage (see `docs/BENCH_NOTES.md`)
- CowCAN (gs_usb) CAN adapter on the RNS-E's 100 kbit/s CAN bus, for the PC-side bench tools only

## How the chain fits
```
Hudiy/AA --(wlr export-dmabuf)--> pi/wlcap --raw--> pienc/rnv3enc --RNV3--> pi/avlink --USB CDC--> teensy/sdemu
  pw-record of the Hudiy sink (48 kHz s16) ---------------------------------^                    | emulated SD card
                                                                                                  v
        rnse/src/sdstream.c on the RNS-E: SD read, RNV2 decode, display flip, audio sink
        (started on TV entry by rnse/hook, or loaded into RAM by rnse/benchd.py)
```
- `pi/wlcap`: screen capture with damage tracking, newest-frame-wins.
- `pienc`: RNV3 video encoder, the working tree's `enc3` and what the Pi runs. Installed as `rnv3enc` (`docs/DEPLOYMENT.md` has the deployment and settings) (NEON on the Pi, scalar and emulated NEON on x86), rate-limited to the SD bus budget (`--linkcap`),
  with the superframe budget and still-screen convergence options. `ref/` is the older RNV2 reference decoder, `dec/` the RNV3 one.
- `pi/avlink`: packs video, audio and control into superframes and sends them to the Teensy with credit-based flow control
  (`--whole-sf 1` hands over whole superframes). Tests in `pi/avlink/test`.
- `pi/run`: what the Pi runs. `avsd_run.sh` (systemd `avsd-chain.service`) waits for the audio graph, puts it on the Pi clock
  (`avsd_clock`) and the Teensy, then `hud2.sh` starts capture, encoder and link; settings in `avsd.env`. Expects the binaries in
  `~/avsd/current/bin` (the Hudiy integration installs them there). These scripts are edited here, not synced.
- `pi/scripts`: diagnostics that need no head unit: `bench_audio.sh` (whole chain into /dev/null), `audio_diag.sh`, `tap_rate.py`, `xrun_delta.sh`.
- `teensy/sdemu`: SD card emulation firmware (SD protocol, FAT16 volume, live superframe store, optional touch HID).
- `rnse`: the RNS-E program (`src/`, built with an `sh-elf` cross toolchain), the PC-side bench host (`benchd.py`, `rnse_bench.py`,
  `sdstream.py`, `sdfast.py`) and `rnse/hook` (source of the flash-resident TV-entry starter, see `docs/HOOK.md`).
- `docs/PROTOCOL.md`: the superframe / wire / USB protocol specification. Comments in the code that say `rnse_proto` mean `rnse/`.

## Build
| part | how | needs |
|---|---|---|
| Teensy firmware | `cd teensy && pio run` | PlatformIO with the Teensy platform. `usb_serial_touch.py` patches the Teensy core's `usb_desc.h` to add a CDC+touch USB type. Not run in the last cleanup (it edits the shared core) |
| RNS-E programs | `cd rnse && mkdir -p out && bash build.sh` | `sh-elf-gcc` (SH-4, big-endian, `-m4-single`) at `~/sh-elf/bin/sh-elf-*`; output in `rnse/out/` (`sdstream.bin`, `sdboot.bin`, ...) |
| Pi binaries | `make` at the top: `bin/rnv3enc`, `bin/avlink`, `bin/wlcap` (`make test` runs the encoder self-checks) | gcc on the Pi 5; `WAYLAND_DISPLAY` and `XDG_RUNTIME_DIR` at run time for wlcap |
| Encoder only | `cd pienc && make` (`make test` for the self-checks) | gcc; NEON is used on aarch64, emulated NEON checks run on x86 |
| Hook (`rnse/hook`) | not buildable here | linked into a firmware image by private tools |

### Checks that run offline (Windows Python plus WSL gcc, no hardware)
- `python -I pi/avlink/test/run_tests.py --skip-e2e --live-quick --out <dir>`: credit/flow-control unit tests, the Teensy
  AvStream ownership test and host self-test of `sdproto`/`avstream`, wire images against `link/sd_wire.py`, and the live
  scenarios (reader away and back, overload). Stages 2 and 3 (superframes against `sdgen.py`, end to end with a simulated reader)
  need the reference generator and source clips that are not in this repo, so they are skipped.
  The host self-test reports one known difference, `sec_4064`, which predates the current work.
- `cd pienc && make test`: kernel and closed-loop checks (encoder against the reference decoder).
- `cd teensy/tools/host_link && make`: host build of the Teensy stream code.

## Install / run (bench procedure, in short)
1. Flash the Teensy (`teensy/tools/bench/flash.py` for the Windows HalfKay path, or any Teensy loader with `firmware.hex`).
2. On the Pi: `make`, copy `bin/*` to `~/avsd/current/bin` and `pi/run/*` to `~/avsd/current/chain`, install the two systemd
   user units. `hud2.sh SECONDS` alone starts capture, encoder and link (`WLCAP_BIN`, `ENC_BIN`, `AVLINK_BIN` override the paths).
3. RAM start from the PC: `python -I rnse/benchd.py serve` (owns the CowCAN, keeps the RNS-E awake), then
   `python -I rnse/benchd.py sdstream run --slot 0 --gate none --wrap 0 --mode av --secs N --target 3000 --iis`
   (the hook uses target 3000; older notes used 4800). The program uploads over SD or CAN and runs as a firmware task.
4. Python deps for the CAN side: `python-can`/`gs_usb`, `pyusb`, `libusb-package` (64-bit Python). `rnse/can/` holds the
   small CAN memory-read/write transport used by `rnse_bench.py`.

Nothing in this repo writes the RNS-E flash. The hook that starts the stream from flash is included as source only
(`docs/HOOK.md`); getting it onto a unit takes tools that are not here.

## Not included, on purpose
Firmware dumps and extracted OSM/vendor data, disc images, run logs and captured data, build outputs, flash-staging and
image-build tools, Doom, research scripts (encoder evaluation, firmware RE helpers), and the source clips used by the
end-to-end tests. Several constants in `rnse/layout.py` and `rnse/src/` are addresses and offsets recovered by reverse-engineering
the RNS-E firmware; they are tied to specific builds.

## License
PolyForm Noncommercial 1.0.0 (`LICENSE.md`): free to use, change and share for any noncommercial purpose; commercial use is
not allowed. Exceptions and files of unclear provenance are listed in `docs/THIRD_PARTY.md`.

Required Notice: Copyright (c) 2026 Drew Sparks (https://github.com/DSparks156x)

## Refreshing from the working tree
`tools/sync_from_worktree.py` (working tree next to this repo, or `SDLINK_WORKTREE`) re-copies the files listed in `tools/manifest.txt` from the author's private working tree
and reports any new unlisted source files. It needs no third-party packages. It also applies the path and address scrubbing
and a few patches (test paths, notes). Re-run it, then `git status`.
