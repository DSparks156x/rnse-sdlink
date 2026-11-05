# Bench notes (summary, 2026-11-05)

The author's day-by-day bench log stays in the private working tree and is not part of this repo.
What matters for anyone reproducing this:

- **Throughput:** the RNS-E SD host path tops out at about 5.3 to 5.5 MB/s (about 92 us per 512-byte sector; the DMA
  engine takes 320 ns per transfer on a 16-bit port). 25 MHz works but is no faster than /4.
  Plan with `--linkcap 4.4e6` (bytes/s) so audio always fits.
- **Chain state:** works from a cold boot on the TV source with AA video and audio. Not yet smooth like analog 480p60:
  ghosting on large screen changes (rate cap), uneven presentation (flip pacing is time-based, not vblank-locked),
  capture falling to about 45 fps under heavy motion on the Pi, occasional expired superframes (video freezes until the
  next key frame), a rare 50 s SD read stall (the unit's own `sd_raw_read` timeout; root cause unknown), and audio
  clock drift between the Pi and the RNS-E sink (rate matching exists, only partly proven on hardware).
- **Firmware dependency:** the RAM program assumes specific RNS-E firmware builds (live byte checks in `rnse/layout.py`).
  It is loaded over CAN into RAM only and is gone after a reboot.
- **Pi:** performance governor and a fixed 800x480@60 output mode help; system settings are the user's to change
  (nothing in this repo changes them). `pi/scripts/hud2.sh` documents the environment knobs
  (`KEYINT`, `LINKCAP`, `MAXVID`, `GAIN`, `ENCTHREADS`).
- **Not done:** starting the stream from flash on TV-source entry without CAN; clean stop on source exit (code exists,
  untested on hardware); Pi services for boot-time start.
