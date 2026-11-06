# Bench state and measurements (2026-11-06)

Summary of the author's private bench log. Numbers are from one unit on a bench, one or a few runs each, and are quoted as
measured. Nothing here has been run in a car. "Flashed" means written to the unit's own flash by tools that are not in
this repo; "RAM" means loaded over CAN or SD into RAM and gone at the next power-off.

## What is where

| part | where it runs | state |
|---|---|---|
| Teensy firmware (`teensy/sdemu`) | Teensy 4.1, flashed over USB | hex `c1ba0665` is on the Teensy (includes the card-resume fix, see Recovery). Earlier `ef7e280f` carried the 1500-sector pool and slot pin |
| `pi/avlink` | Pi | deployed; the Pi build has `--whole-sf` |
| Encoder (`pienc/`, from the working tree's `pienc_budget`) | Pi | deployed; see "Live Pi settings" for what runs |
| Reader, decoder, audio sink (`rnse/src/sdstream.c`, `rnv2sh.c` fast decoder) | RNS-E | as module SDST in the unit's flash (sector 96): reader without the per-superframe sleep, ring 170 sectors x 6, audio block 960 / sink 3 |
| Hook that starts it on TV entry (`rnse/hook`) | RNS-E, ACON task 100 ms tick | in the unit's flash (OSM literal + cave); auto-starts the stream when the TV screen is entered, no CAN |
| Bench host (`rnse/benchd.py`, `rnse_bench.py`, `sdfast.py`) | PC | loads RAM programs over CAN, or over SD (about 2 s instead of minutes) |

Flash-writing tools, firmware images and the image build for the hook are not in this repo.

## Throughput

- Flash build, hook start, smap clip, 41 s window late in a run: **4.56 MB/s** total over SD, 60.0 superframes/s, 59.6 fps
  shown, decode 12.21 ms mean, payload CRC bad 0, underruns 0, cuts 0, 1 read error, 1 stuck read. An earlier 21 s window just after a chain
  restart: 4.54 MB/s, 1 underrun, 0 read errors.
- RAM runs with the same reader: 4.48 to 4.56 MB/s, 1 underrun in every run (length-independent, location not found).
- **Video** alone reached about 4.2 to 4.34 MB/s on these clips (map3d, smap), not the 4.5 MB/s aimed for. The encoder has nothing
  more to spend on those clips, or runs out of Pi time (`over`/`pan` clips).
- Read cost on the unit: about 0.42 ms per read plus 100 to 109 us per sector. The old 3.4 to 3.6 MB/s ceiling came from
  `dly_tsk(1)` taking 2.8 to 3.0 ms in the reader loop, not from the SD path; the reader no longer sleeps there.
- The decoder is the next limit: at `--cyc 5.5e6` decode rose to 14.15 ms, the ring backed up and 11 underruns followed (audio rides
  in the same superframes). `--cyc 4.05e6` keeps decode near 12 ms.
- Rate match (Pi audio clock against the RNS-E sink) read -443 ppm against a clamp of -504, fill 2410 of target 3000: close to
  its limit, to be watched.

## Whole superframes ("whole-sf")

`avlink --whole-sf 1` (and the matching Teensy setting `cfg wholesf 1`, or `printf WSF1 > /tmp/avlink_sdctl` on the Pi) hands the
Teensy a complete superframe before the unit may read it, so the unit sees no mid-read holds. In the bench log it removed the stuck
reads in two 60 s runs (about one per 25 to 30 s without it); the flash run above still had one. The Teensy pool is 1500 sectors;
the Teensy evicts a superframe the unit has not started reading in time, which shows up as
EXPIRED headers and a CRC mismatch, not as corruption.

## Live Pi settings (differ from `pi/scripts/hud2.sh` in this repo)

`pi/scripts/hud2.sh` here is the source of truth for the script. The copy running on the Pi on 2026-11-06 differs in two places:

- `ENC_BIN` default is `$A/pienc_budget3/rnv2enc` instead of `$A/pienc_budget/rnv2enc`
- the default `ENCFLAGS` has `--converge 1` appended: `--cyc 4.05e6 --refill wall --dwell 0 --converge 1`

Everything else is identical. Defaults in the repo script: `--linkcap 4.8e6 --maxvid 161 --cyc 4.05e6 --refill wall --dwell 0`,
avlink `--maxvid 161 --whole-sf 1 --gain-db -18`, capture with `pw-record` (set `AUDIOCAP=parec` to revert), FIFO priorities for
the chain and PipeWire. An old reader with a 160-sector ring must not run these settings.

`--converge 1` (still-screen convergence, encoder README "SETTLED QUALITY"): offline on a real capture, blocks off target went
from 1188 to 0 and the worst error from 3 to 0 within about 0.2 to 0.5 s. Same bitstream format and decoder. The effect on the
panel is not judged yet. `--cscfit 1|2` exist in the encoder source (panel-banding experiments); the notes say 1 is in use on
the Pi, but the live `hud2.sh` carries only `--converge 1`.

The encoder source in this repo may be newer than the binary on the Pi: the Pi's `enc.c` has md5 `d721c5ba` and the working tree's
differs. Same format.

## Recovery after the SD slot loses power: fix unproven

Symptom (2026-11-05): the unit sent CMD13 once a second forever, the Teensy stayed idle, no CMD0, the stream never came back.
Cause found in `teensy/sdemu/fxsd.cpp` `card_cmd`: a mounted host was resumed only if no CMD0 had been seen since Teensy boot, so
after a VDD off/on it stayed silent. Fix: drop that condition (a host in IDLE during a real init only sends CMD0, CMD8, CMD55/ACMD41).
The fix is flashed, but flashing restarts the Teensy and the stream recovers by the old path anyway, so the stream returning
**does not prove it**. It needs a real sleep/wake or power cycle of the slot.

## Known open issues

- Teensy dropped off USB about every 15 minutes on 2026-11-05 (disconnects at 13:27 and 13:43 Pi time); the chain restarts in about
  3 to 4 s with picture and sound gone meanwhile. Pi power was marginal (EXT5V 4.93 V, `get_throttled` 0x50000, past undervoltage).
  Not shown to be the cause; not fixed.
- With the chain killed, the reader keeps the last picture on screen instead of returning to the stock TV. What the car should do
  when the Pi goes away is not decided.
- 1 underrun per run, read errors (raw return -161, retried, 1 to 4 per 30 s at the higher rates), and an occasional stuck read:
  causes unknown.
- A flip gap of up to about 7 s with no key frame is possible (from the source, not measured).
- A rare SD read stall (the unit's own `sd_raw_read` timeout; seen as one 65 ms timeout in timed runs); root cause unknown.
- Leaving the TV source can leave trash on the screen (`SD_LEAVEFIX` in `sdstream.c`, compiled only, never run). `SD_FASTSTUCK`
  never run.
- A card seated badly shows as hook stage 1 with Teensy `host_cmds=0`, CLK 0: check the slot first.

## Not tested

Full power-off cold boot of the current build; live Android Auto by ear (Pi side counters were clean for 265 s: 0 silence events,
adjust within -5..3, after `pw-record` and a 50 ms prime; the owner has not reported on it); TV exit and re-entry audio with this
build; car DAC; Teensy on the car's real power order; the Pi systemd units in a car; touch passthrough in use.

## Other facts

- The SD host path: 25 MHz works but is no faster than /4. DMA engine 320 ns per transfer on a 16-bit port; floor about 82 us per
  sector.
- Firmware dependency: the RAM program and the hook assume specific RNS-E builds (live byte checks in `rnse/layout.py`).
- Pi: performance governor and a fixed 800x480@60 headless output (`NOOP-1`) are assumed by the car scripts; nothing in this repo
  changes them except `pi/scripts/car/install_car.sh`, which you run yourself.
