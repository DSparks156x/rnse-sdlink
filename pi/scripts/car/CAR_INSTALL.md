> Note: R61.md, rebuild.py, sim_r61.py and tva_bench.py named below are the author's private firmware tooling and are not in this repo; the Pi side (install_car.sh, services, hud2.sh) is.

# A/V over SD: car install

Nothing here has run in the car. Do the steps in order and stop at the first check that fails.
Firmware details, CAN commands and bench tests: `analysis/r61_car_v1/R61.md`.

## Before leaving the desk
1. **Final binaries.** Teensy hex flashed. `~/avsd/hud2.sh`, `pi/wlcap/wlcap`, `pienc/rnv2enc`, `pi/avlink/avlink` are the final ones.
   - Check: a bench run with the chain started by hand is good.
2. **RNS-E firmware.** `rebuild.py`, `sim_r61.py` says ALL OK, `tva_bench.py base` says r59, RAM harness tests passed, owner's go, then CAN commit or disc.
   - Check: `bootstate` restart count 0. With no Pi and no Teensy, TV shows the stock screen and the radio works.
3. **Pi.** Copy `pi/scripts/car/` to `~/avsd/car/`, then run `~/avsd/car/install_car.sh`, then reboot.
   - Installs: `avsd-chain` user unit, `avsd-governor` system unit, `XCURSOR_THEME=invisible`, wayvnc disabled.
   - Check: `~/avsd/car/avsd_status.sh`: service active, governor `performance`, wayvnc off, STAT lines moving.
   - The cursor setting needs the reboot; VNC is gone after it, so use `ssh hudiy`.
   - Bench again later: `touch ~/avsd/car/DISABLE; systemctl --user stop avsd-chain`, and `tva_bench.py off`.

## In the car
4. **Wiring.** Teensy in SD slot 2 (driver slot 0), USB to the Pi. **Unplug the HDMI converter**: wlcap takes the first output, and the service warns in its log if HDMI is active.
   - Check: `wlr-randr` lists NOOP-1 800x480 only.
5. **Power order.** Ignition on, nothing touched, wait for the boots.
   - Check: `avsd_status.sh` shows `Teensy on USB ok` in the log.
   - Power order does not matter (bench, open item 1 below): a late Teensy is mounted by the unit's own retry, and a Teensy restart under a mounted unit carries on (firmware a87bd52a or later).
6. **Volume low, then select TV.** Expected: picture about 1-3 s after the TV screen appears, then sound.
   - The RNS-E volume knob applies; `GAIN` -18 in hud2.sh matches the stock MP3 level.
   - `pactl get-sink-volume hudiy_equalizer_sink` should read 100%.
7. **Leave and return.** TV -> Radio: radio audible, stock screen, within 2 s. Radio -> TV: back. TV -> NAV -> TV: stream stops on NAV, resumes on return.
   - Over CAN only: `tva_bench.py status` gives the reason for each stop.
8. **Key off, key on, repeat from cold, 3 times.** Stream resumes each time with no action.
   - Silence after boot is an open bug on the bench (open item 2). If it happens, toggle source once and report.
9. **Controls.** Panel keys reach AA on `0x461` (`can_keyboard_control.py`, `tv_panel_controls`), steering wheel likewise. Touch: GT911 on the Teensy, USB HID to the Pi.
   - Check: each key does something in AA, touch lands where pressed.
   - Keys with the stream on are untested on the bench; touch was tested only by VNC/vtouch.
10. **Audio, 10 min of music.** `avsd_status.sh`: `adrop=` and `silence=` in the STAT line must stop rising.
    - No USB DAC: audio goes over SD, and the car setup is the same as the bench.
11. **Health.** `vcgencmd get_throttled` should read `0x0` (bench read 0x50000 since boot). Temperature under 80 C after 15 min.
12. **Optional, once:** AA over USB. Pi audit estimates it removes the 25-30% of frame holes made after the phone sends.

## Going back
| to | how |
|---|---|
| Stream off for now | Other source. On the Pi: `touch ~/avsd/car/DISABLE; systemctl --user stop avsd-chain` |
| Analog TV, r61 stays | Pi: `~/avsd/car/uninstall_car.sh`, restore `~/.config/labwc/autostart.bak-avsd`, `~/.local/bin/hudiy_startup_fast.sh.bak-withlut` (lutshim), reboot. Replug the HDMI converter and analog audio. Pull the Teensy. With no stream on the card, r61 never takes over (simulated, not run) |
| r59 firmware | CAN `r59_sector_44/53.bin`, or disc r59 or r36/0450 |
| Teensy | Pull it, or flash the previous hex with `hkflash_linux.py` |

## Open: decide or test before relying on it
1. **Card init vs power order: closed on the bench 2026-11-05.** Teensy held in HalfKay over an RNS-E reboot, released 30 s later: the unit initialised and mounted it by itself. Teensy restarted under a mounted unit: stock never re-inits, so the emulator now resumes as selected on the first data command before any CMD0 (`fxsd.cpp`), 3/3 reads with no RNS-E reboot. HalfKay simulates a missing card; a really unpowered Teensy (pads clamped) was not tried. Pi reboot: avlink exits on hang-up and `avsd-chain` restarts it.
2. Silence after boot (cause unknown, reliability agent). Blocks "just works".
3. 1 s of AA over the new screen is gone only if sdstream's exit fires; it did not fire in one bench toggle before tonight's change. Fallback is 1.5 s plus exit time.
4. NAV on the TV source stops the audio too (per-visit mode): sdstream has no video-only release.
