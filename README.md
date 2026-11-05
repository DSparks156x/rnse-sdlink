# rnse-av-sd

Experimental: audio and video for an Audi RNS-E (SH7773 "193PU" head unit) delivered through its SD card slot.
A Teensy 4.1 pretends to be an SD card; a RAM program running on the RNS-E reads A/V "superframes" from it,
decodes them and drives the screen and the audio sink on the TV source. The source is typically Android Auto
(Hudiy) running on a Raspberry Pi 5.

**State: experimental, tested on one unit (the author's), on a bench. Not a product.** Nothing here is safe to run on a
car you care about. The RNS-E side loads code into RAM over CAN and pokes firmware internals at fixed addresses that
only match specific firmware builds (checked at runtime in `rnse/layout.py`, `FW_CHECKS`). Misuse of flash tools
(none are in this repo) can brick the unit. The stream is not smooth yet; see `docs/BENCH_NOTES.md`.

## Hardware
- Raspberry Pi 5 (aarch64, Pi OS Bookworm) running a Wayland compositor (labwc) with Hudiy / Android Auto
- Teensy 4.1 wired to the RNS-E SD slot (FlexIO2 + eDMA), plus an optional GT911 touch panel passthrough
- Teensy USB to the Pi (`/dev/ttyACM0`)
- Audi RNS-E with a modified firmware lineage (see `docs/BENCH_NOTES.md`)
- CowCAN (gs_usb) CAN adapter on the RNS-E's 100 kbit/s CAN bus to load the RAM program (PC side)

## How the chain fits
```
Hudiy/AA --(wlr screencopy)--> pi/wlcap --raw--> pienc/rnv2enc --RNV2--> pi/avlink --USB CDC--> teensy/sdemu
  PulseAudio monitor (parec, 48 kHz s16) ------------------------------^                         | emulated SD card
                                                                                                  v
                 rnse/src/sdstream.c (RAM program on the RNS-E: SD read, RNV2 decode, display flip, audio sink)
```
- `pi/wlcap`: screen capture with damage tracking, newest-frame-wins.
- `pienc`: RNV2 video encoder (NEON on the Pi, scalar and emulated NEON on x86), rate-limited to the SD bus budget (`--linkcap`).
- `pi/avlink`: packs video, audio and control into superframes and sends them to the Teensy with credit-based flow control.
- `teensy/sdemu`: SD card emulation firmware (SD protocol, FAT16 volume, live superframe store, optional touch HID).
- `rnse`: RAM programs for the RNS-E (`src/`, built with an `sh-elf` cross toolchain) and the PC-side bench host
  (`benchd.py`, `rnse_bench.py`, `sdstream.py`) that uploads and starts them over CAN.
- `docs/PROTOCOL.md`: the superframe / wire / USB protocol specification.

## Build
| part | how | needs |
|---|---|---|
| Teensy firmware | `cd teensy && pio run` | PlatformIO with the Teensy platform. `usb_serial_touch.py` patches the Teensy core's `usb_desc.h` to add a CDC+touch USB type |
| RNS-E RAM programs | `cd rnse && mkdir -p out && bash build.sh` | `sh-elf-gcc` (SH-4, big-endian, `-m4-single`) at `~/sh-elf/bin/sh-elf-*`; output in `rnse/out/` |
| Pi link | `cd pi/avlink && make` | gcc |
| Pi capture | `cd pi/wlcap && cc -O2 -pthread -o wlcap wlcap.c` | Wayland headers; `WAYLAND_DISPLAY` and `XDG_RUNTIME_DIR` at run time |
| Encoder | `cd pienc && make` (`make test` for the self-checks) | gcc; NEON is used on aarch64 |

## Install / run (bench procedure, in short)
1. Flash the Teensy (`teensy/tools/bench/flash.py` for the Windows HalfKay path, or any Teensy loader with `firmware.hex`).
2. On the Pi build wlcap, pienc and avlink, copy `pi/scripts/hud2.sh` next to them (it assumes `~/avsd/{pi,pienc}`),
   then `hud2.sh SECONDS` starts capture, encoder and link.
3. On the PC: `python -I rnse/benchd.py serve` (owns the CowCAN, keeps the RNS-E awake), then
   `python -I rnse/benchd.py sdstream run --slot 0 --gate none --wrap 0 --mode av --secs N --target 4800 --iis`.
   The RAM program uploads over CAN (about 100 s when it changed) and runs as a firmware task.
4. Python deps for the CAN side: `python-can`/`gs_usb`, `pyusb`, `libusb-package` (64-bit Python). `rnse/can/` holds the
   small CAN memory-read/write transport used by `rnse_bench.py`.

Nothing in this repo writes the RNS-E flash. Starting the stream from flash on TV-source entry (no CAN) is not implemented here.

## Not included, on purpose
Firmware dumps and extracted OSM/vendor data, disc images, run logs and captured data, build outputs.
Several constants in `rnse/layout.py` and `rnse/src/` are addresses and offsets recovered by reverse-engineering the
RNS-E firmware; they are tied to specific builds.

No license has been chosen yet. See `docs/THIRD_PARTY.md` for files whose provenance needs a decision.

## Refreshing from the working tree
`tools/sync_from_analysis.py` re-copies the files listed in `tools/manifest.txt` from the author's private working tree
and reports any new unlisted source files. It needs no third-party packages.
