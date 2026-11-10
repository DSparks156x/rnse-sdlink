# Third-party and unclear-provenance files

The repository is under PolyForm Noncommercial 1.0.0 (`LICENSE.md`). That license can only cover the author's own work: a file
that turns out to be derived from GPL code (the HalfKay flasher below is the candidate) stays under the GPL, and is to be
rewritten or removed rather than relicensed. Items below are still open.

| file | issue |
|---|---|
| `teensy/tools/bench/hkflash.py`, `hkflash_linux.py` | HalfKay (PJRC Teensy 4 bootloader) flasher; follows the block protocol of PJRC's `teensy_loader_cli` (GPL). Treat as a derivative until re-checked. |
| `teensy/tools/bench/flash.py` | Calls PJRC's `teensy_reboot` from the Teensy tool package (not included). |
| `teensy/usb_serial_touch.py` | Rewrites the Teensy 4 core's `usb_desc.h` (PJRC Teensyduino core) and embeds a descriptor block modeled on it. The core itself is NOT copied here. |
| `teensy/sdemu/touch.cpp` | GT911 register use follows the vendor programming guide and Linux `goodix.c` (GPL) as a reference; written independently, but check. |
| `teensy/sdemu/tonemp3.h` | 60 s generated test tone (ffmpeg + libmp3lame), 960 KB array. Own signal; could be generated at build time instead. |
| `rnse/can/*` | The author's own earlier CAN tools; depend on `gs_usb`, `pyusb`, `libusb-package` (from PyPI, not copied). |
| `rnse/src/*`, `rnse/layout.py`, `rnse/benchd.py`, `rnse/sdstream.py`, `rnse/sdbench.py`, `rnse/sdtime.py`, `rnse/can/*` | Contain addresses, offsets, byte patterns and calling conventions recovered by RE of Audi/Harman RNS-E firmware. No vendor code or binaries are included. Owner to decide on publication. |
| `rnse/module/*`, `rnse/src/sdboot.c`, `rnse/src/fwtab.h` | The flash module takes every firmware address from the per-firmware table (no firmware address in its code); `fwtab.h` names what the table holds. `sdboot.c` (RAM-only bench loader) still has literal addresses. The firmware-side hook, table values and patching tools are not in this repo. |
| `rnse/src/rnv2ref.c`, `rnv2_tables.h`, `pienc/ref/*` | Reference RNV2 decoder and tables (the author's own codec). |

Knowledge notes derived from the QNX `lanbic` sources are deliberately not in this repo.
Firmware dumps (`rnse_proto/fw`, `*.payload.bin`) are excluded and ignored by `.gitignore`.
