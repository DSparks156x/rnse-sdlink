# SDLink flash module and the firmware contract

SDLink's RNS-E side is one 128 KiB flash sector, **sector 107** (NOR `0x00D60000..0x00D7FFFF`, XIP `0x80D60000`). The same bytes
run on every supported firmware. Everything firmware-specific lives in that firmware's own image: a small hook and a lookup table.
A Pi release can therefore update SDLink by rewriting sector 107 alone (`docs/PI_UPDATER.md`).

```
firmware (per build, private tools)              sector 107 (this repo, rnse/module)
  ACON 100 ms tick -> hook v3 --r4 = table-->      tva_poll(table)  -> validate table, run the TV-entry logic
                                                   tva_task         -> task 159: copy sdstream to RAM, start it
                                                   sdstream image   -> reads the table copy for every firmware call
```

## Build
`wsl.exe -e bash -lc "bash rnse/module/build.sh"` (or `bash rnse/module/build.sh` on Linux).
It needs `~/sh-elf/bin/sh-elf-*` (SH-4 big-endian, `-m4-single`) and python3. It runs `rnse/build.sh` (all RAM programs; sdstream
with the shipping config `-DFWTAB -DSD_LEAVEFIX`) and then `rnse/module/mkmodule.py`.

Outputs in `rnse/out/`:
- `sector_107.bin`: the whole sector, the module followed by a zero tail.
- `sector_107.bin.sha256`.
- `module.json`: build id, sizes, sum, required table version, CTL words, symbols.
- `module/`: the module ELF, disassembly and symbols.

## Sector 107 layout
| offset | size | content |
|---|---|---|
| +0x00 | 0x70 | header text; it must start with the tag `.SDS` (see "Tag" below) |
| +0x70 | 4 | destination `0x00D60000` (BE32) |
| +0x74 | 4 | declared size N = 0x80 + payload |
| +0x78 | 4 | BE32 word sum over `[+0x80, +N)` |
| +0x7C | 4 | 0 |
| +0x80 | 4 | magic `'TVA2'` (`0x54564132`) |
| +0x84 | 4 | `tva_poll` |
| +0x88 | 4 | `tva_task` |
| +0x8C | 4 | sdstream RAM image start (inside this module) |
| +0x90 | 4 | sdstream image length |
| +0x94 | 4 | build id: first 4 bytes of sha256(sdstream image) |
| +0x98 | 4 | firmware-table version this module needs (`FWT_VERSION` in `rnse/src/fwtab.h`) |
| +0x9C | 4 | stream formats the decoder plays: `SDL_STREAM_FMT_MIN << 8 \| SDL_STREAM_FMT` (`rnse/src/sdl_ver.h`); was 0 before 2026-11-10 |
| ... | | `tvauto` code, the sdstream image |
| N..0x20000 | | zero |

**Stream format version.** `rnse/src/sdl_ver.h` holds the one definition, `SDL_STREAM_FMT` (newest format the decoder plays) and
`SDL_STREAM_FMT_MIN` (oldest). Rule: **bump `SDL_STREAM_FMT` whenever the decoder's stream format changes**, i.e. anything an older
encoder/avlink could not feed or an older decoder could not play: the superframe header or TLVs, the audio format, the video container
(`video_fmt`) or the RNV bitstream. Raise `SDL_STREAM_FMT_MIN` only when the decoder drops an older format. The Pi side lists what it
sends in `pi/avlink/rnse_ver.h` (`AVL_FMT_MIN/MAX`, same numbers) and streams only a format in both ranges. `mkmodule.py` writes the
pair into header +0x9C (so an updater can read it before installing) and checks it; `tvauto` announces it at every run start
(`docs/PROTOCOL.md` 4.5). History: 1 = superframe v1, audio_fmt 1, video_fmt 2 (RNV2/RNV3 frames), 2026-11-10.

The firmware's disc container lists this module with an allocation of the whole sector. Its pre-install check sums the full
allocation, which is why the tail is zero and not FF. A disc install and a field write therefore leave the same 128 KiB.

**Tag.** `.SDS` is deliberately not one of the tags the boot ROM accepts (`.OS .APL .FNT .ETC` ...). The disc installer only
registers modules it knows by name in the boot descriptor, so nothing walks or checks this module at boot. A bad sector 107
cannot stop the unit booting. It only means no SDLink.

**Header text.** The disc installer's generic path checks only the destination (inside an erased bank, 8-aligned), the declared
size and the sum. It identifies modules by their container name (`SDST`), not by the header text, so the text is free-form.
`mkmodule.py` writes `.SDS`, the format, the build id and the sector number.

## Hook (firmware side)
The firmware build repoints the literal its 100 ms ACON tick dispatcher uses to call PWM main. That literal now points at the
hook, and the hook:
1. calls PWM main, always;
2. reads `'TVA2'` at `0x80D60080`; anything else: done;
3. **checks that the module is whole**:
   - header `+0x70 == 0x00D60000`;
   - `0x80 < N <= 0x20000` and N % 4 == 0;
   - the word sum over `[0x80D60080, 0x80D60000 + N)` equals `+0x78`.

   The result is cached in two RAM words, `{'HKOK' | 'HKNO', sum}` at CTL page +0xFF00 (`SDL_HOOKCACHE`), and redone whenever
   `+0x78` changes. A torn or partial write therefore never gets called. A power loss, the only way a running commit can tear,
   also clears the RAM, so the next boot checks again;
4. calls `tva_poll` with **r4 = the firmware's table** (FPSCR SZ/PR cleared, as the stock dispatcher does).

The hook, the table and their placement are made by the private firmware tools. A module whose magic is not `'TVA2'` is never
called. An older hook that checked `'TVA1'` never calls this module.

## Firmware table
Defined by `rnse/src/fwtab.h`, the one definition shared by tvauto, sdstream and the table generator. All words are big-endian u32:

| word | content |
|---|---|
| 0 | `'SDFW'` |
| 1 | version (`FWT_VERSION`, 1) |
| 2 | nwords, the whole table |
| 3 | fw_id (free-form, e.g. `0x64`, `0x54B`, `0x310`) |
| 4 | ram_map: which RAM map the firmware promises is free (below) |
| 5 .. 5+FWE_N | one word per entry, in the order of `FWT_SIGNED` then `FWT_VALUES` in fwtab.h |
| .. +FWE_NSIG | for every SIGNED entry, the first 4 bytes found at that address in the firmware |
| nwords-1 | CRC-32 (zlib) of everything before it |

How it is checked:
- `tva_poll` checks the table on every call: magic, version, nwords, ram_map, CRC, and each SIGNED entry's first 4 bytes read
  live from flash.
- Any failure means the unit stays stock. The reason is recorded in the TVA RAM block (below): `FA<reason><entry>`, where reason
  1 = table address, 2 = magic, 3 = version, 4 = nwords, 5 = ram_map, 6 = CRC, 7 = signature address, 8 = signature.
- A pass writes the table address to TVA word 25 and `'OK2F'` (`0x4F4B3246`) to word 26.
- `tva_task` passes the table to sdstream in CTL words 62/63 (address, then its complement). sdstream copies it and checks it
  again before any firmware call.

Entries (see the comments in fwtab.h):
- **Functions:** delay, exit, start task; the audio sink calls; raw SD read/write and its read core; source select and index;
  queue post, panel and TV select (bench keys); IIS enable; DU commit.
- **Task-159 entry:** the task's rest entry.
- **Data:** the source-index-to-controller-code table.
- **RAM globals:** tick, task control blocks, source/screen/power state, sink handle and ring, SD slot words, DU shadow registers.
- **Firmware constants:** TV source/screen values, MR values, task ids, index maps.

The kernel layout is assumed the same everywhere (task control block stride 0x40, state at +4, entry at +0x20). Hardware registers
(FFxxxxxx) are the same on every unit and stay literal.

**Versioning.** Adding, removing or reordering entries changes the table format: bump `FWT_VERSION`. A module refuses tables of
other versions, and its header word +0x98 lets an updater refuse to install a module that is newer than the unit's table.

## RAM map (`rnse/src/sdl_ram.h`, ram_map 1)
The table's ram_map word is the firmware's promise that these regions are free. The module refuses any other value.

| region | P1 range | use |
|---|---|---|
| FB | `0x8F4C0000` + 0x232800 | three 800x480 YUY2 frame buffers (DU scan-out) |
| CTL | `0x8F720000` + 0x20000 | +0 CTL words, +0x100 STAT, +0x1000 log ring, +0xF000 TVA state, +0xFF00 hook cache, +0xFFD0 debug, +0xFFF0 keys, +0x10000 PCM blocks |
| WORK | `0x8F758000` + 0x25E000 | +0 trace, +0x8000 sdstream image window (link base `0x8F760000`), +0x28000 superframe ring, +0xA7800 leave trace, +0xA8000/+0x168000 video reassembly, +0x228000 audio FIFO, then the reader and audio stacks |

On units with the EU voice-recognition module, the firmware build disables that module, which otherwise claims this RAM.

## CTL words (tvauto -> sdstream)
`mkmodule.py` holds the run parameters tvauto writes before calling sdstream's `main`:

| word | value | meaning |
|---|---|---|
| 0 | `'CTL1'` | magic |
| 1 | — | stop word, never written by tvauto; 2 = the user left the source |
| 2 | 0 | slot 0 |
| 3 | `0x60` | IIS route + no hop |
| 4 | `0x17` | mode av |
| 5 | 0 | secs (forever) |
| 6 | 0 | wrap (live) |
| 7 | 1 | p1c |
| 8 | `0x300` | sink depth 3 |
| 9 | 960 | block |
| 10 | 7800 | target |
| 11 | 3 | frame buffers (2 or 3) |
| 12 | `0x40` | read hint (sectors) |
| 13, 14 | 0 | start sequence, max superframes (0 = no limit) |
| 62/63 | — | table address and its complement |

`rnse/sdstream.py make_ctl` produces the same words from command-line options.

## TVA RAM (CTL page +0xF000), for tools
| word | content |
|---|---|
| 0 | state magic `'TVA2'` |
| 1 | off switch: `'OFF!'` (`0x4F464621`) = auto start off; a warm reset keeps it |
| 25 | validated table address |
| 26 | `'OK2F'` or `FA<reason><entry>` |
| 27 | handshake state this run: 0 none (no ack: older Teensy), 1 pending, 2 accept, 3 reject, 4 timeout (no verdict in 1 s) |
| 28 | probes that saw `RSAV` this run |
| 29 | first ack tick (pending timer), 0 = none |
| 30 | announcements sent since power-up |
| 31 | last ack word: verdict << 24 \| chosen format << 16 |
| +0x100 | trace ring |

What `tva_poll` does on each call:
- **Start.** It starts a run when source = TV, screen = TV and power = on have held for 1 s and task 159 is dormant, and not in the
  first 10 s after boot.
- **Stop.** Power off stops a run at once. Another screen on the TV source stops it after 300 ms. Another source stops it after
  sdstream's own exit (1.5 s), then with a stop word.
- **Back-off.** A run that dies within 5 s backs off 4, 8, 16, 32 s, then 30 s. Three resets during a stream turn auto start off.
- `tva_task` first reads LBA `0x10000` until it sees `RSAV` with a live feed. A normal SD card, or no Teensy, means no takeover.
- **Version handshake.** Once a probe sees `RSAV`, `tva_task` announces the module build id (`TVA_BUILD_ID` = header +0x94), the stream
  formats (`sdl_ver.h`) and the validated table's fw_id and version through the LBAs of 8 reads (`docs/PROTOCOL.md` 4.5). A Pi that
  plays none of the unit's formats rejects, and the unit then stays stock. Words 0..26 and the probe are unchanged; the handshake
  state is appended (27..31). Not in the standby build (`TVA_STANDBY`, no probe loop).
