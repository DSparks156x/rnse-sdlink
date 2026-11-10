# Updating the RNS-E module from the Pi (sector 107)

A Pi release ships `sector_107.bin` (from `rnse/module/build.sh`, with its `.sha256` and `module.json`) and writes it over the Pi's
`can0`. No disc is needed, and the image doesn't depend on the firmware, provided the unit runs a firmware build that carries the
SDLink hook and table (`docs/FIRMWARE_CONTRACT.md`).

## What gets written
- **One whole 128 KiB sector:** NOR `0x00D60000..0x00D7FFFF` (P1 `0x80D60000`). Always write the whole file, module plus zero tail,
  so that a disc install and a field write leave identical bytes and the same CRC.
- **Module header:**
  - `+0x70`: destination `0x00D60000`.
  - `+0x74`: declared size N.
  - `+0x78`: word sum over `[+0x80, +N)`.
  - `+0x80`: `'TVA2'`.
  - `+0x94`: build id.
  - `+0x98`: required table version.
  - `+0x9C`: stream formats the decoder plays, `fmt_min << 8 | fmt` (0 in modules before 2026-11-10). An updater can check that the
    Pi's chain sends one of them before installing (`docs/FIRMWARE_CONTRACT.md`, stream format version).
- **Torn writes are safe.** The firmware hook sum-checks the module each time the header sum changes and never calls a bad one, so a
  torn write means no SDLink, not a boot loop. The boot ROM does not check this sector at all.

## Compatibility gate (refuse otherwise)
1. **The unit has an SDLink firmware table.** Read the table's first 20 bytes: `'SDFW'`, version, nwords, fw_id, ram_map.
   - With a v2 module already running: TVA RAM word 26 (`0x8F72F068`) reads `0x4F4B3246` (`'OK2F'`), and word 25 (`0x8F72F064`)
     then holds the validated table address.
   - Otherwise, probe the table addresses the current firmware builds use:
     - r64 line `0x806AA900`;
     - PCBBC NA `0x806ACC00`;
     - PCBBC EU `0x806BAC00`.
2. **The image fits the table:** image `+0x98` (required table version) <= table version, and table ram_map == 1.
3. **The image checks out before sending:** `+0x70 == 0x00D60000`, N sane, the sum matches, and the sha256 matches the release's `.sha256`.
4. **The encoder matches:** the decoder in sector 107 and the Pi encoder share the stream format, so release and install them together.

## Write procedure
This uses the unit's CAN service (the B-series commands on the 0x7Bx IDs): B1 read, B3 stage, B7/B6 multi-sector commit.
1. **Preconditions.** The ignition is on (with it off, `can0` can only listen; check and refuse). Don't fight other CAN users
   on the same IDs.
2. **Skip if current.** Read the live sector 107 CRC (`rnse/can/decoder_check.py`). If it already equals the file's, stop.
3. **Stop streaming** (`avsd-chain`): staging goes through the Teensy link.
4. **Turn auto start off.** Write `'OFF!'` (`0x4F464621`) to TVA word 1 (`0x8F72F004`). Wait until task 159 is dormant: TCB
   base + 159 x 0x40, state at +4 reads `0x0010`. Both TCB base and task id are in the firmware table.
5. **Stage the image.** Stage it over SD, then B3 END: the unit's CRC must equal the host's. Spot-check samples with B1.
6. **Commit.** Send B7 + B6. There is no ack: the unit erases, programs and cold-resets (~25 s).
7. **Verify.**
   - The boot restart count at `0xB800001A` reads 0.
   - The live sector 107 CRC equals the file's.
   - The table status reads `'OK2F'` once the module has run.
   - The commit's cold reset clears the off word (modules from 2026-11-10 on: init re-arms auto start, the crash guard re-sets it). Writing 0 to `0x8F72F004` afterwards is still harmless and covers a commit that did not land (an older module keeps the off word until power-off).
   - Restart `avsd-chain`.
8. **Keep a rollback.** Keep the previous `sector_107.bin`. After any disc install, recheck the sector 107 CRC and re-apply the Pi's
   newer image if it differs.

## In rnse-carpi
carpid does this as the LIVI firmware screen's **Install** (`crates/carpid/src/firmware.rs`). Where it differs from the steps above:
- **Staging goes over CAN (B3 data frames), not SD.** The SD bootstrap (`sdboot`, `sdstage`) uses firmware-specific addresses
  that are not in the firmware table, so it only works on the bench lineage. B3 needs only the CAN service every SDLink firmware
  has. About 26 000 frames at ~400 a second; a CRC mismatch at B3 END restages slower (200, then 100 a second). The Teensy is not
  used, but the chain is still stopped.
- **The table version must equal** the image's `+0x98`, not just be at least it: the module refuses any other table version.
- **The bus is leased** (carpi-gate) from auto start off to the commit, and for the final off-word write. Every other sender on
  ICAN is refused meanwhile. Only these frames pass the lease: B9 to `0x8F72F004`, B3, B7 slot 0 sector 107, B6 with one target armed.
- **Skip if current** compares the 0xA0-byte header first and reads the whole sector only when it matches.
- **Order between builds:** build ids are hashes, so `module.json` carries `history` (`rnse/module/history.txt`, oldest first);
  `mkmodule.py` appends a new build there. Commit that line with the sources.
- **Rollback** is the previous release's `firmware/rnse/sector_107.bin` under `/opt/carpi/releases/`.

The Teensy's version reaches carpid through avlink: it asks `ver` on the console once the link runs and writes
`TEENSY_VERSION='...'` into `--rnse-file`. With the chain stopped, carpid asks itself.

Not yet run on a car: the first CAN commit from the Pi.
