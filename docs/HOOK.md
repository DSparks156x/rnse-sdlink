# Flash-resident start hook (r61), sources only

`rnse/hook/` holds the source of the code that starts the stream by itself when the TV screen is entered. It is on the author's unit
in flash; this repo has the source and nothing to put it there (no image build, no flash or staging tools).

| file | what |
|---|---|
| `tvauto.c` | the SDST flash module: `tva_poll()` (called every 100 ms) and `tva_task()` (task 159 body). Includes `gen_cfg.h`, which the private image build generates |
| `hdr.S`, `tvauto.ld` | module header/entry table and linker script; `hdr.S` embeds the unchanged RAM build of `rnse/src/sdstream.c` |
| `tvhook_r61.s` | the small cave in the firmware's ACON task: calls the stock PWM main, then `tva_poll` if the module is present |

How it works (from the module's own comments and the build notes):
- `tva_poll` never blocks. It starts a run when source = TV, screen group = TV and PWM = on have held for 1 s and task 159 is dormant,
  and does nothing for the first 10 s after boot.
- Stops: power off at once; another screen on the TV source after 300 ms; another source after the stream's own exit (1.5 s), then a stop word.
  A run that dies within 5 s backs off 4, 8, 16, 32 s, then 30 s. Three resets during a stream turn auto start off until RAM is lost.
- `tva_task` first reads one sector at LBA 0x10000 until it sees `RSAV` with a live feed. With no Teensy, a real SD card or no Pi the
  unit stays stock. Then it copies the sdstream image to RAM, writes the control block and calls its `main()`.
- Off switch: RAM word `8F72F004` = `'OFF!'`. A warm reset keeps it, so auto start stays off until power-off or the bench tool turns it on.

Not buildable from this repo alone: the module is linked into a firmware image with tools that are not here. Addresses are for one
firmware lineage.
