# Pi 5 -> Teensy 4.1 -> RNS-E SD-slot link: SPEC v1 (2026-11-02)

> Written as a design document, since implemented and run on the bench (see the root README for the real state).
> References to other folders (datasheets, analysis notes, FACTS.md, sdgen.py, video/) point into the author's private
> working tree and are not part of this repo.
Inputs: `../FACTS.md` and `analysis/SD_LOSSLESS_STUDY.txt` (r19 addresses), plus the
SH7724 manual in `datasheets/` (its SDHI is a sister of the RNS-E's TMIO host; electrical proxy only).

Files in this folder:

| file | what it is |
|---|---|
| `SPEC.md` | this spec |
| `sd_wire.py` | reference encoder: CRC7/CRC16, all card responses, CID/CSD/SCR/SD-status, 4-bit and 1-bit wire images, superframe header. **Self-tests pass** against published SD CRC vectors (CMD0 95, CMD17 55, R1(17,0x900) 67, CMD8 87, CRC16(FF x 512) 7FA1). Every response byte in this spec was printed by it. |
| `teensy_sdemu_sketch.c` | Teensy firmware structure: FlexIO setup, CMD state machine, DMA feed, write capture outline (not compiled) |
| `rnse_modeb_driver_sketch.c` | custom RNS-E streaming driver (mode B) on the TMIO registers (not compiled) |
| `pi_link_sketch.c` | Pi side: wire-image builder and libusb framing (not compiled) |

Evidence tags:
- **P (PROVED)**: read in RNS-E code (study), computed and self-tested here, or printed in a datasheet in `datasheets/`.
- **I (INFERRED)**: engineering inference, the SD spec or the i.MX RT1062 manual quoted from memory, or Linux driver naming.
  VERIFIED 2026-11-02: IMXRT1060RM, IMXRT1060CEC, the Teensy 4.1 schematic and SD Simplified Spec 9.10 are now in
  `datasheets/`. FlexIO/IOMUXC/DMAMUX/clock values were checked against them. Full table with page refs:
  `SPEC_VERIFY.md`. Lines changed here are marked VERIFIED 2026-11-02 or CORRECTED.

---------------------------------------------------------------------------------------------------

## 0. Summary of decisions

| topic | decision |
|---|---|
| Card profile | SD 1.x SDSC, 1 GiB, CSD v1, SCR SD_SPEC=0, no CMD8 reply, no CPRM (SD_SECURITY=0) |
| Teensy peripheral | FlexIO2, clocked **by the host's SDCLK on a pin** (timer decrement source = pin). 4 shifters + 2 timers. FLEXIO2 clock 120 MHz (datasheet max). CORRECTED: DAT RX moved to S3 (parallel RX is legal only on S3/S7) |
| Pins | SDCLK=6, CMD=9, DAT0..3 = 8,7,36,37 (FLEXIO2 D10, D11, D16..D19) |
| Read data | Pi builds ready-to-shift **wire images** (528 B per sector, with per-line CRC16). Teensy eDMA streams them to the shifter with zero per-byte work |
| Flow control, read | Teensy holds the start bit (idle filler) when data is not ready; never longer than 100 ms, then it sends a synthetic IDLE superframe |
| Container | per tick a **superframe**: 1 header sector (64 B header + 448 B down-channel), audio sectors, video sectors |
| Addressing | superframe slot window: LBA = 0x10000 + (seq mod 256) * 256. Same scheme for the stock driver (A) and the custom driver (B) |
| Uplink | stage 1: **CMD60 "W32"** vendor command (32 bits up in the argument, 32 bits down in the R1 field, no DAT, no CRC token). Stage 2: CMD25 writes to LBA channel windows |
| Clocking | RNS-E SSI is the audio master. The RNS-E reports fill and its sample counter every tick; the Pi sends exactly the requested samples (bit-exact sources) or runs an ASRC (phone sources) |
| Bring-up order | **Mode A (stock driver) first**, at 1-bit / ~3 MHz, then 4-bit, then 25 MHz. Mode B only if mode A throughput or latency is short |

---------------------------------------------------------------------------------------------------

## 1. Teensy 4.1 SD-card emulation

### 1.1 Peripheral choice

| option | verdict |
|---|---|
| **FlexIO (chosen)** | Shifters can be clocked by an external pin (TIMDEC = 10b: decrement on pin both edges, shift clock = pin; RM p2998, SPI-slave Table 50-11 p2960). VERIFIED 2026-11-02. CORRECTED: only SHIFTER0/4 can **drive** 4 pins (PWIDTH=3) and only SHIFTER3/7 can **sample** 4 pins (RM p2950). DMA only on shifters 0-3 (DMAMUX sources 1 and 65). Because the shifters are clocked by SDCLK, the Teensy runs at exactly the host's rate, and it pauses whenever the host stops the clock |
| uSDHC | host-only controller; cannot be a card (I) |
| LPSPI slave | needs a chip-select; SD has none; 1 data line only |
| SAI slave (4 TX lines, BCLK in) | possible, but needs a frame sync and has no CMD path; kept as a fallback idea only |
| GPIO bit-bang | the 600 MHz M7 cannot follow 25 MHz edges; usable only for ~400 kHz ID mode experiments |

FlexIO instance: **FlexIO2**. FlexIO3 has no DMA requests on the RT1062 (VERIFIED 2026-11-02: no FLEXIO3 entry in the
DMAMUX table, RM p52-56). FlexIO1's upper pins sit on the bottom PSRAM pads (pins 48-54) (I).

FlexIO2 resource plan. VERIFIED 2026-11-02: PARAM reset 0x0220_0808 = 8 shifters, 8 timers, 32 pins (RM p2975). Still
read `FLEXIO2_PARAM` at boot as a sanity check.

| res | role | clocking | data movement |
|---|---|---|---|
| T0 | DAT word timer | 16-bit counter, TIMDEC = 10b (SDCLK pin both edges); TIMCMP = 15 (4-bit: 8 clocks per 32-bit word) or 63 (1-bit) (VERIFIED, RM p2960). CORRECTED: TIMENA = 100b (enable on SDCLK rising edge), not "always", so the compare/store lands on the falling edge. The RM says FlexIO cannot shift and store on the same cycle (p2937); verify the phase on the bench, flip PINPOL if needed | - |
| T1 | CMD word timer | same, TIMCMP = 31 (16-bit words, for low response latency) | - |
| S0 | DAT TX (read data, SCR, SD status, CRC token, busy) | T0 | eDMA, DMAMUX source 1 (FLEXIO2 request 0/1), SHIFTSDEN bit 0 |
| S1 | CORRECTED: **CMD TX** (was DAT RX) | T1 (same phase as S2) | CPU writes 1-9 words; SHIFTSDEN bit 1 stays 0 |
| S2 | CMD RX | T1 | IRQ per 16-bit word, NVIC priority 0; SHIFTSDEN bit 2 stays 0 |
| S3 | CORRECTED: **DAT RX** (host writes; was CMD TX). PWIDTH = 3, PINSEL = 16: 4-pin sampling is legal only on S3/S7 | T0 | eDMA, DMAMUX source **65** (FLEXIO2 request 2/3), SHIFTSDEN bit 3 |

VERIFIED 2026-11-02: only shifters 0-3 have DMA requests (DMAMUX source 1 = shifters 0/1, source 65 = shifters 2/3; RM
p53, p56). DMA users are S0 and S3; the CMD path is CPU-driven.

### 1.2 Pins

| SD signal | SD card pin | Teensy 4.1 pin | pad | FlexIO2 | pad setup |
|---|---|---|---|---|---|
| CLK (in) | 5 | **6** | GPIO_B0_10 | D10 | hysteresis on, no pull/keeper (PKE=0), DSE=0, ALT4 |
| CMD (bidir) | 2 | **9** | GPIO_B0_11 | D11 | DSE 4..5 (CORRECTED: 39/32 ohm typ @3.3 V, CEC Table 27), SRE=0 slow, SPEED 01/10 (100/150 MHz), HYS on, 47k pull-up (PKE=1 PUE=1 PUS=01) |
| DAT0 | 7 | **8** | GPIO_B1_00 | D16 | same as CMD |
| DAT1 | 8 | **7** | GPIO_B1_01 | D17 | same |
| DAT2 | 9 | **36** | GPIO_B1_02 | D18 | same |
| DAT3 | 1 | **37** | GPIO_B1_03 | D19 | same (the host may also use a DAT3 pull-up for detect) |
| VSS1, VSS2 | 3, 6 | GND (several wires) | | | |
| VDD | 4 | **not connected to 3V3**. 10k/20k divider to pin 14 (A0) = "host powered" sense | | | |
| bus-switch OE (car build) | | 3 | | | GPIO out |
| LA trigger / debug | | 2 | | | GPIO out, pulse on CMD decode / CRC error |

Why these pins: DAT0..3 must be four consecutive FlexIO pins in one instance (shifter PWIDTH=3 drives PINSEL..PINSEL+3; VERIFIED 2026-11-02, RM p2950: TX on S0/S4, RX on S3/S7 only).
GPIO_B1_00..03 = FLEXIO2 D16..D19 are on Teensy edge pins 8, 7, 36, 37. They avoid pin 13 (LED load), which the other
consecutive group D0..D3 (pins 10, 12, 11, 13) would need. CLK and CMD go on the nearby D10/D11 (pins 6, 9).
Pad-to-FlexIO numbering: GPIO_B0_nn = FLEXIO2 Dnn and GPIO_B1_nn = FLEXIO2 D(16+nn), ALT4. VERIFIED 2026-11-02:
B0_10/B0_11 ALT4 = FLEXIO2_FLEXIO10/11 (RM p514-515), B1_00..03 ALT4 = FLEXIO2_FLEXIO16..19 (RM p520-523); no daisy
(SELECT_INPUT) register for FLEXIO2; Teensy pins 6, 9, 8, 7, 36, 37 = these pads (schematic). FLEXIO2 PINSEL is 5 bits,
so D16-D19 are reachable.

### 1.3 Clocks

| item | value | tag |
|---|---|---|
| RNS-E SDCLK, ID mode | TMIO code 0x20 (/128) -> ~390 kHz if the source is Pck 49.95 MHz | I (study) |
| RNS-E SDCLK, transfer | code 0 (/2) -> ~25 MHz | I (study) |
| slow patch | 8068277E E400 -> E404 (/16) -> ~3.1 MHz, plus RAM table 8EAD11CA | I (study) |
| 1-bit patch | 80653152 E004 -> E001 | P (study) |
| FLEXIO2 clock | CORRECTED: **120 MHz = datasheet maximum** for FLEXIO2_CLK_ROOT (RM Table 14-5, p1025). 240 MHz would be a 2x overclock: do not use. Setup: CSCMR2 FLEXIO2_CLK_SEL = 11 (pll3_sw_clk 480 MHz), CS1CDR FLEXIO2_CLK_PRED = 001 and PODF = 001 (/2 /2), CCGR3 CG0 on (RM p1053, p1056-1057, p1081). The root is shared with FlexIO3 | P |
| ratio needed | At 25 MHz / 120 MHz each SDCLK phase is 2.4 FlexIO clocks (50 % duty assumed). The RM's generic limits are FlexIO/4 for input and FlexIO/6 for slave output (RM p2959); 25 MHz TX works only because of the full-period rising-launch budget (1.4). A host duty cycle far from 50 % (the SD spec allows tWL/tWH down to 10 ns) would break edge detection: measure it in stage 1 | VERIFIED 2026-11-02 |

### 1.4 Timing at 12.5 and 25 MHz

VERIFIED 2026-11-02, default-speed SD rules (SD spec 9.10 Fig 6-11, p307): the host samples on the **rising** edge;
card output valid **<= 14 ns after the falling edge** and held **>= 0 ns after the next rising edge**; tWL/tWH >= 10 ns.
High speed (Fig 6-13, p308): valid <= 14 ns after the **rising** edge, held >= 2.5 ns. The 50 ns ID-mode tODLY and the
card tISU/tIH 5/5 ns are in tables removed from the Simplified Spec (still I). Host input timing, SH7724 SDHI proxy
(`datasheets/SH7724 Hardware Manual.pdf`, table 52.36, P for the SH7724, I for the SH7773): **setup 5 ns, hold 2 ns**,
minimum SDCLK period 20 ns.

CORRECTED: FlexIO latency `L` (SDCLK edge at the slot to new data at the slot) = **1.5-2.5 FlexIO clocks** (sync
0.5-1.5 + 1 to output; RM p2951, p2959) + pads ~1-4 ns (I, the CEC gives no propagation delay) + ~2 ns extender
round trip. That is **15.5-26.8 ns at 120 MHz**. There is **no synchroniser bypass**: every pin input and trigger is
synchronised (RM p2951, p2953).

**Launch rule:** shift TX data on the **rising** edge (shifter TIMPOL = posedge). The new bit then appears `L` after
rising edge R0. The host samples it at R1 = R0 + T. Constraints: `L >= t_hold(2 ns)` and `L <= T - t_setup(5 ns)`.
VERIFIED 2026-11-02, this is **spec-legal**: the spec bounds the card's valid window (F0+14 .. R1+0), not the edge the
card uses inside, so any `0 <= L <= tWL + 14` (= 34 ns at 25 MHz, 50 % duty) is a compliant card output.

| SDCLK | T | window for L (rising launch) | L @120 MHz FlexIO | margin | falling launch (spec-literal) |
|---|---|---|---|---|---|
| 3.1 MHz | 320 ns | 2..315 ns | 15.5-26.8 | huge | fine |
| 12.5 MHz | 80 ns | 2..54 (spec) / 75 (host) | 15.5-26.8 | >= 27 ns | fine |
| 25 MHz | 40 ns | 2..34 ns | 15.5-26.8 | CORRECTED: **7.2 ns late, 13.5 ns early** | needs L <= 14 ns: **fails** at 120 MHz (and at the worst case even at an overclocked 240 MHz) |
| 50 MHz HS | 20 ns | 2.5..14 ns | 15.5-26.8 | **fails** | - |

Keep TIMPOL selectable at run time (USB CONFIG message) and verify on the scope (stage 5), but rising launch is the
only workable choice at 25 MHz.

**Receive (CMD, write DAT):** sample on the rising edge. SDCLK and data pass through the same synchroniser (one
synchroniser per input, shared by all users; RM p2951), so their relative timing is kept, quantised to one FlexIO
clock (8.3 ns at 120 MHz). CORRECTED: the data must be held about 8.3-16.7 ns after the rising edge. The host launches
on the falling edge, so it holds data for ~20 ns: 3-12 ns of margin at 25 MHz. This relies on real host behaviour, not
on the spec minimum tIH.

**50 MHz high speed: not feasible** (VERIFIED 2026-11-02 reasoning, SPEC_VERIFY.md 2.4): L at the legal 120 MHz exceeds
the 14 ns budget; 10 ns clock phases are 1.2 FlexIO clocks; RX hold needs ~8-17 ns where an HS host guarantees ~2 ns;
there is no synchroniser bypass; the SAI slave alternative is itself capped at a 40 ns BCLK (CEC Table 51); and the
RNS-E never runs above Pck/2 = 24.98 MHz or sends CMD6. **Final SD clock: 24.98 MHz, 4-bit, rising launch.**

**Clock integrity is the critical risk.** FlexIO counts SDCLK edges. One ringing glitch is one extra shift, which
corrupts the block. The host's CRC16 check catches it (and the container CRC32 catches anything that slips past), but
the link then has to retry. Mitigations: section 1.9.

### 1.5 Wire images (read direction): exact format

Every 512-B sector exists on the Teensy only as a **wire image**: the 32-bit words that S0 shifts out, LSB first.
The Pi builds them (`sd_wire.py: wire4/wire1`, `pi_link_sketch.c: wire4`). The Teensy builds the few it makes itself
(FAT metadata, IDLE/NOT_READY sectors, SCR, SD status) once at boot.

4-bit image, 132 words = **528 B**. Nibble bit k drives DAT k; nibble 0 of each word goes out first.

| words | content |
|---|---|
| 0 | `0x0FFFFFFF`: 7 idle nibbles, then the start nibble 0 (all four lines low) |
| 1..128 | data. Each byte is nibble-swapped (the high nibble goes first on the wire), then 4 bytes are packed little-endian. On the Teensy this is the same as writing the raw bytes to CORRECTED: **`SHIFTBUFNBS`** (nibble swap within each byte, offset 0x680, RM p3001). `SHIFTBUFNIS` reverses all 8 nibbles and would also reverse the byte order. For 1-bit images the equivalent is `SHIFTBUFBBS` |
| 129..130 | CRC16 of each DAT line, 16 nibbles, MSB first. Line k carries bit (4+k), then bit k, of every byte |
| 131 | `0xFFFFFFFF`: end nibble F, then idle |

1-bit image, 130 words = **520 B**: word 0 = `0x7FFFFFFF` (31 idle bits, start bit), words 1..128 = bytes
bit-reversed and packed little-endian, word 129 = `bitrev16(crc16) | 0xFFFF0000` (CRC, end bit, idle).

The same encoder makes the small blocks: SCR (8 B) as 4-bit = 6 words, SD status (64 B) as 4-bit = 20 words.
`sd_wire.py` decodes images back (`unwire4`) and round-trips them in its self-test (P).

Streaming rate with back-to-back images at 4-bit: 132 x 8 = 1056 clocks per sector, so 42.2 us at 25 MHz =
**12.1 MB/s ceiling**, and 6.06 MB/s at 12.5 MHz.

### 1.6 CRC strategy

| CRC | who computes it | how |
|---|---|---|
| CRC7 of commands received | Teensy, in the CMD ISR | 256-entry table, 5 lookups. On mismatch: no response, and COM_CRC_ERROR (bit 23) is set in the next R1 |
| CRC7 of responses | **precomputed** for every constant response (table 2.A.2). Runtime table only for CMD60 downlink words | |
| R2 internal CRC7 (CID, CSD) | precomputed | |
| CRC16 on read data | **Pi**, inside the wire image (~10k sectors/s, trivial on an A76). The Teensy only does it for its own boot-time sectors | |
| CRC16 on host writes | Teensy at slow clocks (needed for the token decision). At 25 MHz the token is optimistic and the Pi verifies (section 4.4) | |
| CRC32 container | Pi writes it, RNS-E checks it: header (covers the whole header sector) and per payload. End-to-end cover over USB, Teensy RAM and the SD bus | |

### 1.7 DMA and buffering

- **eDMA scatter-gather chain** feeding S0 one word per request, from wire images in RAM:
  `[img k] -> [img k+1] -> ... -> FILLER`. FILLER is a TCD that is linked to itself with source `0xFFFFFFFF` and
  SOFF=0, so the lines stay high, which is the same as holding off the start bit. The CPU appends images by re-linking
  the tail TCD. S0 never underruns while it drives the bus (an underrun is counted from SHIFTERR and reported).
- At 25 MHz / 4-bit, S0 consumes 3.1 M words/s, easy for the eDMA. CORRECTED: S0 (TX) uses DMAMUX source 1 and S3
  (RX) uses DMAMUX source 65, so TX and RX get separate request lines (RM p53, p56).
- **RAM (1 MB on the board):** RAM1 (ITCM+DTCM, 512 KB, uncached, DMA-reachable) and RAM2 (OCRAM `DMAMEM`, 512 KB,
  cached write-back).

| region | size | use |
|---|---|---|
| ITCM | ~64 KB | code |
| DTCM | ~232 KB | wire-sector pool B (440 sectors), CMD bit FIFO, write-capture ring 16 KB, TCDs |
| OCRAM | ~443 KB | wire-sector pool A (840 sectors). `arm_dcache_flush()` after the CPU copies a USB buffer in, before DMA reads |
| total pool | ~1280 wire sectors = ~7 superframes of 178 sectors (~115 ms at 60 Hz) | |
| optional PSRAM (8-16 MB, bottom pads, FlexSPI2) | | long retry history, capture buffering. Not needed for streaming; keep the live stream in internal RAM (lower and deterministic latency) |

- Pool management: a superframe-addressed slot table (8 resident slots). A slot is freed when the host has streamed
  past it (mode B) or when 2 newer superframes have been served (mode A, so retries still hit).
- Metadata sectors (MBR, VBR, FAT, root dir) are made **procedurally** on the fly: a FAT16 chain sector is computable
  from its LBA. No RAM image is needed. Boot-time wire images are cached for the ~6 non-trivial sectors.

### 1.8 Host WRITE blocks (CMD24/25)

The SD sequence: after the R1 to CMD24/25, the host sends each block on DAT0-3 (start, 512 B, 16 CRC nibbles, end).
The card answers on DAT0 with the CRC status token `0 010 1` (accepted) or `0 101 1` (CRC error), starting about
2 clocks after the end bit, then holds DAT0 low (busy) until it is ready for the next block. VERIFIED 2026-11-02: token
and busy on DAT0 only, DAT1-3 don't care (SD p34-35); codes 010/101/110 (SD p331). The start delay (~2 clocks) is in a
table removed from the Simplified Spec (still I). Busy is released on a clock edge (SD p118).

Teensy implementation:
1. `arm_write_capture()`: S3 on (CORRECTED: was S1), eDMA from S3 into a 16 KB DTCM ring. The CPU finds the first all-zero nibble (the
   start bit; T0's word phase is arbitrary, so it realigns by a nibble offset 0..7), then extracts 1024 data nibbles,
   16 CRC nibbles and the end nibble.
2. CRC token, **W-slow** (SDCLK <= ~3 MHz, bring-up): from the start-bit position and the T0 word count, software
   knows when the end bit passes. It pre-loads S0 (DAT0 only, PINCFG on DAT0 alone via a 1-bit S0 configuration) with
   the token and busy bits so the token starts 2 clocks later.
3. CRC token, **W-hw** (25 MHz): FlexIO timer T2 is started by the DAT0 start bit. It counts 1041+2 SDCLK clocks, and
   its output enables T3/S4, a 1-bit TX on DAT0 that holds token + busy. The token is sent **optimistically** as
   "accepted". The real CRC result is reported in the next downlink status word, and the RNS-E retransmits at the
   application layer. CORRECTED (RM p2995-2999): the RM documents pin triggers only for pins 0-15 (TRGSEL = 2N), and
   DAT0 is D16. So start T2 with TIMENA = 100b (pin rising edge) on its own PINSEL = 16 with PINPOL = 1 (start bit low),
   and decrement it with TIMDEC = 11b from the trigger = SDCLK pin 10 (TRGSRC = 1, TRGSEL = 20); TIMCMP = 2 x clocks - 1.
4. Busy: keep DAT0 low after the token while the capture ring has less than 1 block free (write flow control towards
   the Pi). The host waits on DAT0. CORRECTED: the SD write timeout for this CSD is min(100 x R2W(4) x TAAC(1 ms), 250 ms)
   = **250 ms** (SD p121, 4.6.2.2); cap busy at 100 ms.
5. A completed block goes to the Pi as a `HOST_WRITE` message (LBA, CRC flag, 512 B).

Because the CRC token is the hardest timing in the whole design, the **uplink starts with CMD60** (section 4.2),
which needs no DAT activity at all. CMD25 comes later.

### 1.9 Signal integrity, power, card detect

| item | spec |
|---|---|
| extender | SD-to-SD/microSD flex extender, **<= 15 cm** target, 30 cm max tested. Prefer a flex with a ground plane or ground between signals. Teensy board at the far end, wired with the shortest possible leads (< 3 cm) |
| series R | 22-33 ohm at the Teensy on CMD and DAT0-3 (source termination for the Teensy drivers); 33 ohm at the Teensy CLK input (with the pin capacitance it damps ringing) |
| CLK | pad hysteresis on. If ringing shows on the scope (any double edge): add 10-15 pF to GND at the pin after the 33 ohm, or AC termination. A one-glitch-per-hour clock still costs a retry and resync per glitch |
| drive strength | CORRECTED: DSE 4..5 = 39/32 ohm typ at 3.3 V (CEC Table 27, p42); slow slew (SRE=0) gives 1.7-2.4 ns edges at 15 pF (CEC Table 25), enough for 25 MHz |
| pull-ups | the host should have 10-100k pull-ups on CMD and DAT (SD spec). Measure with a meter, RNS-E on, no card (stage 1). Teensy pads add 47k pull-ups regardless. VERIFIED 2026-11-02: a real card has its own 50 k pull-up only on CD/DAT3 (SD p38). CORRECTED: the Teensy pulls go to its 3V3, so turn them off (PKE=0) while slot VDD is absent, or they back-feed the host |
| ground | both slot VSS pins, at least 2 ground wires in the extender; Teensy GND, Pi GND via USB. In the car, power the Pi from the same ground point as the RNS-E |
| power | **never connect slot VDD to the Teensy 3V3** (no back-feed into the RNS-E's switched SD supply, FF93F008 bit1). Teensy powered from the Pi's USB. Slot VDD is only sensed (divider to pin 14). With VDD absent, all SD pins go high-Z (the Teensy must not power the host through its ESD diodes) |
| isolation (car) | SN74CBTLV3245-class FET bus switch (~5 ohm, < 0.3 ns) between the Teensy and the slot. OE = slot VDD present AND Teensy firmware ready. It also stops the RNS-E pull-ups from feeding an unpowered Teensy. Bench: series R alone is acceptable with a sane power order |
| card detect | the extender's card body actuates the slot's mechanical switch, polled on GPIO FF93E00C (bit 0x400 slot 0, bit 0x800 slot 1) (P). Insert = high (I). Debounce 3/40 polls (P). The Teensy cannot fake a removal; a re-init is done in software (mode B) or by CMD0 behaviour |
| write protect | not consulted on the read path (I). Leave it as the extender presents it |

---------------------------------------------------------------------------------------------------

## 2. Host-driver modes

### 2.A Mode A: stock-driver compatibility

The RNS-E's own driver initialises the card at insert or boot. AFFM mounts the synthetic FAT16 volume. Our OSM task
reads with `int 80653598(u16 slot, s32 lba, void *buf, s32 nsect)` (P). The buffer is 32-byte aligned, which gives
zero-copy DMA of up to 0x1000 sectors per call (P). The task never touches the controller.

#### 2.A.1 Init sequence and what the driver checks (from the study, P unless marked)

| step | host sends | clock | driver check | Teensy action |
|---|---|---|---|---|
| 1 | soft reset (E0 = 0 then 7), clock /128 | ~390 kHz (I) | | power-on state = idle |
| 2 | CMD0 | | none | no response; state idle; 1-bit; stop any stream |
| 3 | CMD8 0x1AA | | checks the echo **if** it replies | **no response** (SD 1.x; VERIFIED 2026-11-02, SD p69). The host sets CMD8ok = 0. CORRECTED: per SD p121 an illegal command also sets ILLEGAL_COMMAND (bit 22) in the next R1 (clear condition B, p147). Make this a CONFIG flag, default on, and watch the stage-3 CMDLOG |
| 4 | CMD55(0) + ACMD41 arg 0x00FF0000, up to 100 x, 10 ms apart | | done when OCR bit31 = 1. A missing CMD55 response gives type 2 (MMC), which is **rejected** (7068040A) | R1 to CMD55 (state idle + APP_CMD). R3 OCR 0x00FF8000 for 2 polls, then 0x80FF8000 -> ready |
| 5 | CMD2 | | CID not checked (as traced) | R2 CID -> ident |
| 6 | CMD3 | | RCA taken from the R6 | R6, RCA 0x5A01 -> stby |
| 7 | CMD9 (RCA) | | CSD_STRUCTURE v1 needs type != 3 (OK, type 1). **READ_BL_LEN >= 9** | R2 CSD v1, 1 GiB, READ_BL_LEN = 9, TRAN_SPEED 0x32 |
| 8 | clock /2 | ~25 MHz (I) | | |
| 9 | CMD7 (RCA) | | R1b | R1 (stby status), release DAT0 at once -> tran |
| 10 | CMD16 512 | | | R1 |
| 11 | CMD55 + ACMD6 arg 2 (arg 0 with the 1-bit patch) | | | R1; switch S0/S3 to PWIDTH 3 (CORRECTED: was S0/S1) (or keep 1-bit) |
| 12 | CMD55 + ACMD13, 64 B in PIO | | contents not known to be checked (I) | R1 + 64-B SD status block (bus width field = current width) |
| 13 | CMD55 + ACMD51, 8 B | | **SD_SPEC 0/1 requires no CMD8 reply** (consistent). SD_SPEC=0 skips CMD6 | R1 + SCR `00 05 00 00 00 00 00 00` |
| 14 | (CMD6 high-speed switch) | | only if SD_SPEC != 0 | never sent |
| per read | CMD13 (R1 error mask FDF90008, state must be tran) then CMD18 + SECCNT + auto-CMD12 (CMD17 for 1-2 blocks) | 25 MHz | R1 error bits | R1 0x900, then the stream; stop at CMD12 |
| any | CPRM ACMD18/25/43-48, CMD8 | | failure only logs awfs_addSecuMediaErr (I) | no response, plus ILLEGAL_COMMAND in the next R1 (CORRECTED, see step 3). Bit 22 is inside the CMD13 mask FDF90008, so the first CMD13 after a CPRM attempt may report an error: check on the bench |

Address mode: SDSC means CMD17/18/24/25 carry a **byte address** (LBA x 512) (I, standard SDSC behaviour; confirm from
CMDLOG in stage 3). The Teensy divides by 512.

#### 2.A.2 Exact response bytes (CMD line, MSB first; printed by `sd_wire.py`, P)

| command | type | bytes |
|---|---|---|
| CMD55 arg 0 (idle) | R1 | `37 00 00 01 20 83` |
| ACMD41 busy | R3 | `3F 00 FF 80 00 FF` |
| ACMD41 ready | R3 | `3F 80 FF 80 00 FF` |
| CMD2 | R2 | `3F 5A 52 53 52 4E 53 41 56 10 00 00 00 01 01 AB ED` |
| CMD3 (RCA 5A01, ident) | R6 | `03 5A 01 05 00 6B` |
| CMD9 | R2 | `3F 00 0E 00 32 5B 59 83 FF FE FB FF 80 0A 40 00 AF` |
| CMD7 | R1b | `07 00 00 07 00 75` |
| CMD16 | R1 | `10 00 00 09 00 0B` |
| CMD55 (tran) | R1 | `37 00 00 09 20 33` |
| ACMD6 | R1 | `06 00 00 09 20 B9` |
| ACMD13 | R1 | `0D 00 00 09 20 5B` |
| ACMD51 | R1 | `33 00 00 09 20 91` |
| CMD13 (tran) | R1 | `0D 00 00 09 00 3F` |
| CMD17 / CMD18 | R1 | `11 00 00 09 00 67` / `12 00 00 09 00 D3` |
| CMD12 in data / in rcv | R1b | `0C 00 00 0B 00 7F` / `0C 00 00 0D 00 0B` |
| CMD24 / CMD25 | R1 | `18 00 00 09 00 5D` / `19 00 00 09 00 31` |

Registers: CID `5A 52 53 52 4E 53 41 56 10 00 00 00 01 01 AB ED` (MID 5A, OID "RS", PNM "RNSAV", 2026-11).
CSD v1 `00 0E 00 32 5B 59 83 FF FE FB FF 80 0A 40 00 AF` (C_SIZE 4095, C_SIZE_MULT 7, READ_BL_LEN 9 -> 1 073 741 824 B).
SCR `00 05 00 00 00 00 00 00`. SD status: byte0 = 0x80 (4-bit) or 0x00 (1-bit), byte8 = 0x02 (speed class 4),
byte10 = 0x90 (AU 4 MB), the rest 0.

Response latency: with S2 and S3 on the same 16-bit word timer T1, the worst case is ~15 clocks (rest of the RX word)
+ the ISR + 16 clocks (one TX word) = **~31-40 clocks**, inside the SD NCR maximum of 64 (I; the NCR table is removed
from Simplified Spec 9.10, so this cannot be checked against `datasheets/`). The TMIO response timeout
is probably longer still (SDHI-family hosts are believed to use 640 clocks, I). CMD release after a response: the host
waits NRC >= 8 clocks before its next command. The ISR leaves one all-ones pad word queued and switches S3 to high-Z on
the T1 flag that follows the last response word.

#### 2.A.3 Synthetic volume and LBA map (both modes)

Sector numbers below are device LBAs (512 B). The card is 2 097 152 sectors.

| LBA range | content |
|---|---|
| 0 | MBR, one partition type 0x06 (FAT16) at 2048 |
| 2048 | VBR: 64 sectors/cluster (32 KB), 1760 reserved, 2 FATs x 128 sectors, 512 root entries (32 sectors) -> data area at **4096** |
| 2049..4095 | reserved, FATs (procedural chain), root dir with one file `RSAVLINK.BIN` (read-only, hidden, system attributes) covering clusters from LBA 4096 to the end of the card. The name has no .mp3, so the player ignores it (study) |
| 0x10000-0x1FFFF | **superframe window**: 256 slots x 256 sectors (32 MiB). `slot_lba(seq) = 0x10000 + (seq mod 256) * 256` |
| 0x20000-0x3FFFF | **uplink write channels**: channel c (0..7) at 0x20000 + c * 0x4000 (8 MiB each), used circularly |
| 0x40000 | Teensy info sector (read): firmware version, counters, newest seq, pool fill |
| 0x40100-0x7FFFF | test pattern region: data = f(LBA), for BER and throughput tests with no Pi involved |
| everything else | zeros (FAT data of the big file) |

The FS only touches LBA 0-4095 (mount) plus whatever it reads of the file. Our task reads the windows directly. The FS
and the raw API share the driver lock (P), so they interleave safely.

#### 2.A.4 Mode A read recipe (per tick)

```
hint = previous header.next_total_hint (default 178)
read(slot, slot_lba(seq), buf, hint)        # 1 call: CMD13 + CMD18(hint) + auto-CMD12
parse header: total_sectors
if total > hint: read(slot, slot_lba(seq)+hint, buf+hint*512, total-hint)
seq += 1
```

Cost: the burst is 178 sectors x 42 us = 7.5 ms at 25 MHz, plus the per-call overhead of ~1-2 ms (I, study).
The task sleeps on the DMA event, so the transfer is bus time, not CPU time. Fits 16.7 ms, with ~5 ms of bus left over.

### 2.B Mode B: minimal custom driver (`rnse_modeb_driver_sketch.c`)

Purpose: higher sustained throughput (no per-call CMD13 and overhead, no auto-CMD12 gaps), and an uplink.

| item | design |
|---|---|
| init | **let the stock driver do it** (the card is present at boot). Then take the driver lock 80653B44 for the streaming session. A "tiny init" (CMD0, CMD55/ACMD41, CMD2, CMD3, CMD9, CMD7, ACMD6; ~30 ms) exists only for error recovery. The Teensy also accepts CMD18 in any state when its `stream_profile` config flag is set |
| registers | offsets P (study). Bit meanings from Linux `tmio_mmc.h` (I). This is supported by a match: the driver's CMD6 value 0x1C06 = RESP_R1 (4<<8) \| DATA_PRESENT (1<<11) \| READ (1<<12) \| 6 |
| endless read | STOP = 0 (SEC off, so no SECCNT limit, I). SIZE = 0x200. CMD = R1\|DATA\|READ\|MULTI\|18, arg = `slot_lba(seq)`. The Teensy then streams superframe seq, seq+1, ... concatenated (no slot padding) |
| DMA | DMAC block FF801200 as the stock driver uses it (SAR = base+0x30, CHCR 00402112, TCR 0x100 per sector) (P), re-armed per 32 KB chunk from the completion IRQ into an 8-chunk ring (256 KB). If the decoder falls behind, the chunk is not re-armed: the TMIO buffer fills and the TMIO stops SDCLK (I, TMIO/SDHI behaviour), which pauses the Teensy too. No data is lost |
| cache | the DMA writes RAM: `ocbi` (invalidate) each chunk before parsing |
| stop | STOP = STP (I), abort the DMA, CMD12 (R1b), STOP = 0 |
| errors | INFO2 CRCFAIL / ENDERR / DTO / CTO -> stop, CMD13. If it is not in tran: SOFT_RST 0->7 + tiny init. Then restart CMD18 at `slot_lba(last_good_seq+1)`. The Teensy resumes there, or answers with an EXPIRED header that names the newest seq |
| read <-> write | once per tick: stop (~20 us), ~8 x CMD60 status words (~10 us each), optional CMD25 burst to a channel window, then restart the CMD18. ~0.2 ms of overhead per tick |
| clock | stay on the stock /2 (25 MHz). Use /4 (12.5 MHz) as the degraded mode if the 25 MHz bench margin fails (video budget then ~4.5 MB/s) |

### 2.C Recommendation

Bring up **mode A first**. It needs zero driver code on the RNS-E, and it proves the whole Teensy (init, CRC7,
CRC16, wire images, timing, stalls) with the stock driver as an independent judge. Its throughput (2.A.4) already fits
the 5 MB/s video + audio budget at 25 MHz. Move to mode B only for (a) the CMD60 uplink, if writes through the stock API
prove awkward, or (b) bus headroom (> 8 MB/s) or lower latency. The stock **raw write** entry point (a sibling of
80653598, reached through disk op 8063B590) is not yet identified. Find it before relying on mode A uplink writes.

---------------------------------------------------------------------------------------------------

## 3. Stream container (read direction)

All container fields are **big-endian** (the RNS-E is a BE SH-4A; the Pi does the swapping). Payloads are opaque
codec blobs with hard per-tick ceilings.

### 3.1 Tick and ceilings

One superframe per **tick** = one Pi output frame period (nominal 60 Hz). A superframe is sent every tick even when
there is no new video, because audio needs it.

| item | default | ceiling |
|---|---|---|
| VIDEO_MAX_BYTES (config) | 83 456 B = 163 sectors (5.0 MB/s @60) | 239 sectors = 122 368 B (slot size limit) |
| AUDIO_MAX_BYTES | 8 192 B = 16 sectors (raw 48k/24-in-32 needs 6 400 B per tick) | 16 sectors |
| header | 1 sector | 1 |
| superframe max | 178 sectors typical, 256 absolute (= slot size) | |

### 3.2 Superframe layout

```
sector 0            : header sector (64-B header + 448-B down-channel area)
sector 1 .. A       : audio payload (audio_bytes, zero-padded to a sector multiple), A = ceil(audio_bytes/512)
sector A+1 .. N-1   : video payload (video_bytes, zero-padded), N = total_sectors
```

Header (offsets in bytes, BE):

| off | size | field | meaning |
|---|---|---|---|
| 0 | 4 | magic | `0x52534156` 'RSAV' |
| 4 | 2 | version | 1 |
| 6 | 2 | hdr_len | 64 |
| 8 | 4 | seq | superframe sequence, +1 per tick, wraps at 2^32 |
| 12 | 4 | flags | see below |
| 16 | 4 | audio_pts | media-clock index (48 kHz samples) of the first audio frame in this superframe |
| 20 | 4 | video_pts | presentation time of the video frame, same clock |
| 24 | 4 | video_bytes | 0 if REPEAT |
| 28 | 2 | audio_frames | sample frames per channel carried (typically 800 +- a few) |
| 30 | 1 | audio_fmt | 0 none, 1 PCM s16 stereo, 2 PCM s24-in-32 stereo, >= 16 codec-defined |
| 31 | 1 | video_fmt | codec-defined (0 = none) |
| 32 | 4 | audio_bytes | |
| 36 | 2 | audio_sec | first audio sector (= 1) |
| 38 | 2 | video_sec | first video sector |
| 40 | 2 | total_sec | N |
| 42 | 2 | next_total_hint | total_sec of seq+1 if the Pi has already queued it, else 0 |
| 44 | 4 | video_crc32 | CRC-32 (zlib) of the video_bytes |
| 48 | 4 | audio_crc32 | |
| 52 | 2 | ctrl_bytes | used bytes of the down-channel area |
| 54 | 2 | frame_id | video frame counter (repeat detection, keyframe tracking) |
| 56 | 4 | pi_time_us | Pi monotonic time when queued (latency measurement) |
| 60 | 4 | hdr_crc32 | CRC-32 over bytes 0..59 and 64..511 |
| 64 | 448 | down-channel | TLV messages {u8 type, u8 len, data}: uplink ACK seq, capture request, time sync, Teensy stats, volume |

flags: bit0 KEY (video decodable without references), bit1 REPEAT (no new video), bit2 IDLE (no media), bit3 DISCONT
(flush the decoder and audio ring), bit4 AUDIO_DISCONT, bit5 SYNTH (made by the Teensy, not the Pi), bit6 NOT_READY,
bit7 EXPIRED (bytes 64..67 = newest seq), bit8 EOS.

### 3.3 Frame boundaries and resync

- Mode A: the boundary is implicit (one slot per call).
- Mode B: superframes are concatenated. The parser takes `total_sec` from each header. **Resync** after any error or
  doubt: scan sector starts (every 512 B) for the magic **and** a valid hdr_crc32 **and** a plausible seq (within +-512
  of the expected one). Then restart the CMD18 at `slot_lba(seq)` if the gap matters.
- A payload CRC32 failure means the superframe is dropped (audio: conceal or insert silence for audio_frames; video:
  keep the last frame and send NEED_KEY in the uplink). The Pi encoder answers with a KEY frame or an intra refresh.

### 3.4 Synthetic sectors (Teensy-made, SYNTH flag set)

| case | Teensy behaviour |
|---|---|
| next sector of the current superframe not received yet (cut-through) | hold the start bit (FILLER) up to **30 ms** |
| next superframe not started (Pi late) | hold up to **20 ms**, then send a 1-sector IDLE header (SYNTH, seq = last real seq; it does not consume a seq) and keep checking. Repeat every 20 ms |
| Pi silent > 100 ms | keep sending IDLE heartbeats. The RNS-E mutes after its audio ring empties and shows the last frame or falls back |
| mode A: requested slot is in the future | hold up to 20 ms, then NOT_READY in sector 0 and the rest filled with the same sector. The RNS-E retries |
| mode A: requested slot evicted | EXPIRED header with the newest seq |

The hard rule: CORRECTED: **a start-bit hold never exceeds 50 ms**. The SD read timeout for an SDSC card is
min(100 x (TAAC + NSAC), 100 ms) (SD p121, 4.6.2.1); with our CSD (TAAC 1 ms, NSAC 0) that is exactly 100 ms, so the
old 100 ms limit had zero margin against a spec-following host. The RNS-E's own data timeout is inferred at ~2^26 SDCLK =
~2.7 s at 25 MHz (OPTION TOP = 0xD, study I).

### 3.5 Clock domains, flow control and drift

- Audio master: the RNS-E SSI, 24.576 MHz / 8, 48 kHz, SH is master, DSP slave (P, study). Nothing else may set the
  audio rate.
- Media clock = the RNS-E's played-sample count. `video_pts` and `audio_pts` are in that unit (48 kHz ticks).
- Every tick the RNS-E sends a STATUS report (4.3) with `audio_fill` (samples queued for the SSI),
  `ssi_samples_total` (sample counter, u32) and `rnse_time_us`.
- Pi rate control:
  - **pull sources** (local files, Pi-generated audio): the Pi sends exactly `request = target_fill - audio_fill + 800`
    samples, clamped to 0..1600. No resampling: **bit-exact** at the DSP input.
  - **push sources** (Android Auto, phone clock): an ASRC on the Pi (soxr/libsamplerate class). Its ratio comes from a
    PI loop on (audio_fill - target_fill) plus a rate feed-forward from d(ssi_samples_total)/d(pi_time). Crystals are
    +-100 ppm, so the worst drift is ~10 samples/s; the loop time constant is ~10 s.
  - target_fill = 2 400 samples (50 ms); low-water 960 (20 ms) -> the Pi sends an extra-large audio share.
- Video is not clock-locked. The RNS-E presents a frame at the first panel vsync where `played_samples >= video_pts`.
  A 60.00 vs ~59.9x Hz mismatch costs one repeat or drop every few seconds (acceptable; adapt later if visible).
- USB flow control: credit-based. The Teensy reports free pool sectors; the Pi never sends beyond them.

### 3.6 Latency budget (video, typical / worst)

| stage | typical | worst | note |
|---|---|---|---|
| Pi capture -> raw frame | 2 ms | 4 | Hudiy/AA surface grab |
| encode | 6 | 8 | codec budget (separate design) |
| wire images + USB | 1 | 4 | cut-through: the SD transfer starts after the first 32-sector chunk |
| Teensy queue | 0-2 | 17 | just-in-time; one tick if the Pi runs ahead |
| SD transfer (178 sectors @25 MHz) | 7.5 | 9 | mode A adds 1-2 ms per call |
| RNS-E decode | 6 | 8 | FACTS budget <= 8 ms |
| wait for vsync + half scan-out | 16 | 25 | |
| **total** | **~40 ms** | **~75 ms** | |

The audio ring target (50 ms) is about the video pipeline delay. The Pi stamps `video_pts = audio_pts(capture) + D`
with D = 50 ms, so lip-sync holds. Trim D after measuring with `pi_time_us` against the STATUS echo.

### 3.7 Bus-time budget per 16.67 ms tick (25 MHz, 4-bit, 42.2 us per sector)

| item | sectors | time | share |
|---|---|---|---|
| capacity | 394 | 16.67 ms | 100% |
| header | 1 | 0.04 | |
| audio (24-in-32 raw worst) | 13 | 0.55 | |
| video at 5 MB/s | 163 | 6.9 | |
| **read total** | **177** | **7.5** | **45%** |
| uplink: 8 x CMD60 + stop/restart | - | 0.2 | 1% |
| free for writes (capture/logs) | ~190 | ~8 | ~48% (~4.6 MB/s theoretical) |

At 12.5 MHz everything doubles: the read share is 90%, so in that degraded mode drop the video ceiling to ~4.5 MB/s and
disable capture.

### 3.8 Down-channel TLVs 0x10 / 0x11 and frames spanning several superframes (ADDED 2026-11-04)

Implemented by `rnse_proto/sdgen.py` (generator), `rnse_proto/src/sdstream.c` (RNS-E reassembly) and `pi/avlink`
(live sender); byte-identical output proven by `pi/avlink/test/run_tests.py`. A video frame larger than the per-superframe
video limit (`maxvid` sectors, default 248) is cut into fragments of `maxvid * 512` bytes, one per consecutive superframe.

| TLV | len | data (BE) | where |
|---|---|---|---|
| 0x11 FRAG | 12 | u32 frame_total_bytes, u32 offset of this fragment, u32 CRC-32 (zlib) of the whole frame | every fragment of a frame with >= 2 fragments; absent = the frame is complete in this superframe (backwards compatible) |
| 0x10 YUY2_CRC | 8 | u32 frame index, u32 CRC-32 of the decoded YUY2 frame | test streams only, on the superframe that completes a frame |

- Each fragment is that superframe's video payload: `video_bytes` / `video_crc32` cover the fragment only.
- Order in the down-channel: 0x11 first, then 0x10. `frame_id` = frame index on every fragment.
- KEY / DISCONT go on the first fragment. Fragments arrive in consecutive superframes, in order; no REPEAT in between.
- Receiver: offset 0 starts a reassembly; a gap, a wrong offset/total or a bad whole-frame CRC drops the frame and sets
  NEED_KEY. Worst case RNV2 frame (all RAW) = 771 024 B = 7 superframes.
- Sender rule (sdgen.py `--k` / avlink `--k`): a frame is queued only if all its fragments fit in the remaining
  superframes; REPEAT superframes pad the rest. Live: one fragment per tick, REPEAT when the queue is empty.

---------------------------------------------------------------------------------------------------

### 3.9 Memory-service TLVs (prototype, 2026-11-04)

The opt-in RNS-E memory service reserves **0x30** for versioned READ / checked RAM PATCH commands
and **0x31** for channel-3 reply ACKs. The byte layout, limits, request-ID rules and error handling are in
[`../rnse_proto/SDMEM.txt`](../rnse_proto/SDMEM.txt); `rnse_proto/sdmem.py` supplies the offline codec.
Each reply is a complete RSUP channel-3 message with its own sequence and CRC. A retained reply is retried
after three accepted superframes without its matching ACK. Multi-chunk reads use explicit offsets.

Control-only producers send advancing **REPEAT** superframes with `total_sec=1`, zero audio/video bytes,
and the exact `ctrl_bytes`. Synthetic IDLE / NOT_READY frames remain retry placeholders and do not
process memory commands. Ordinary streaming leaves the memory service disabled.

This implements the RNS-E service and codec, not Teensy host-write capture or a live Pi memory client.
NOR flash commits continue through the existing CAN B3/B7/B6 mechanism; PATCH writes RAM only.

## 4. Write direction (RNS-E -> Pi)

### 4.1 Channels = LBA windows (CMD24/25)

| channel | LBA base | use | priority |
|---|---|---|---|
| 0 | 0x20000 | control: acks, key events, NEED_KEY, error reports (when CMD60 is not used) | high |
| 1 | 0x24000 | text log / debug ring | low |
| 2 | 0x28000 | screen capture | lowest |
| 3 | 0x2C000 | file / memory dump upload | lowest |
| 4-7 | | spare | |

The LBA offset inside a window = a circular sequence position (window = 16 384 sectors). Each uplink sector:

| off | size | field |
|---|---|---|
| 0 | 4 | magic 'RSUP' 0x52535550 |
| 4 | 1 | channel |
| 5 | 1 | flags (bit0 first, bit1 last of a message) |
| 6 | 2 | payload length (<= 496) |
| 8 | 4 | message seq |
| 12 | 4 | CRC-32 of the payload |
| 16 | 496 | payload |

Screen capture, channel 2: the first sector carries {width, height, format (RGB565 or YUY2), stride, frame_id} and the
rows follow. A full 800x480x2 = 768 000 B frame is 1 549 sectors, ~65 ms of bus time, which is ~8 ticks of free time:
**<= ~4-7 fps uncompressed** at full spare bandwidth. Default 1 fps, or downscaled.

### 4.2 CMD60 "W32" uplink (stage 1 uplink, mode B)

- CMD60 is in the manufacturer-reserved range (VERIFIED 2026-11-02: "CMD60-63 reserved for manufacturer", SD p133, p141). CMD register value = `C_RESP_R1 | 60` = `0x043C` (I, tmio encoding).
- arg = 32 bits up. The Teensy answers R1 with **32 bits down** in the status field (the TMIO checks the CRC7 and end
  bit; the custom driver ignores the "status" meaning).
- Framing of a message over W32 words:
  `word0 = 0xA5 | type(8) | len_bytes(16)`, then ceil(len/4) data words. The Teensy reassembles and forwards an
  `UPLINK` message to the Pi. Downlink words are queued by the Pi (`DOWNLINK` message). Word 0 of each reply is
  `0x5A | credits(8) | pool_fill(16)`, so every W32 call also returns flow-control state.
- Cost: ~110 SDCLK on the wire plus ~5-10 us of SH software per command, so ~0.4-0.8 MB/s ceiling. That is plenty for
  control, logs and status, too slow for capture.

### 4.3 STATUS report (RNS-E -> Pi, every tick, 32 B = 8 W32 words or the start of a channel-0 sector)

| off | size | field |
|---|---|---|
| 0 | 4 | last_seq_ok |
| 4 | 4 | ssi_samples_total |
| 8 | 4 | rnse_time_us |
| 12 | 2 | audio_fill (samples) |
| 14 | 2 | displayed frame_id |
| 16 | 2 | decode_us (last) |
| 18 | 2 | decode_us (max since last report) |
| 20 | 2 | sd_crc_errors (count) |
| 22 | 2 | resyncs (count) |
| 24 | 4 | flags: NEED_KEY, AUDIO_UNDERRUN, PAUSED, CAPTURE_READY, ... |
| 28 | 4 | echo of the pi_time_us of the last superframe presented (end-to-end latency) |

### 4.4 Priority and half-duplex scheduling (RNS-E side)

Per tick: (1) read the superframe (audio is safe even if the video is late); (2) STATUS through W32; (3) the
write burst, bounded to `tick - read_time - 3 ms` margin; (4) restart the stream. Write reliability: in stage 7+ the
token is optimistic at 25 MHz, so each uplink message is acked by the Pi through the down-channel TLV `UPLINK_ACK`
{channel, seq}. Messages without an ack after 3 ticks are resent.

---------------------------------------------------------------------------------------------------

## 5. Pi <-> Teensy USB protocol

### 5.1 Transport

| item | value |
|---|---|
| link | USB 2.0 High Speed, Teensy = device |
| bring-up transport | Teensyduino `Serial` (CDC-ACM, already HS bulk, 512-B packets) -> `/dev/ttyACM0`. Zero custom USB code |
| production transport | custom Teensyduino USB type (edit `usb_desc.h`): vendor interface, bulk OUT 0x03 / bulk IN 0x84, 512 B. Pi: libusb-1.0 async, 8 x 64 KB transfers in flight. udev rule for access |
| need | 177 x 528 B x 60 = **5.6 MB/s** OUT (+ up to ~5 MB/s IN with capture) |
| capacity | HS bulk is ~35-45 MB/s practical on Linux (I). Teensy CDC receive rate unknown here: **measure in stage 0**. Choose CDC for production if it sustains >= 15 MB/s |
| Teensy buffering | USB RX buffers -> memcpy into the pool (6 MB/s of memcpy is ~1% CPU), then `arm_dcache_flush`. Zero-copy (USB descriptors pointing straight into the pool) is a later optimisation |
| IRQ priorities | FLEXIO2 (CMD) 0 > eDMA 32 > USB 64+. The CMD ISR must never wait behind USB |

### 5.2 Message framing (same over CDC and bulk; little-endian, both ends are LE)

Header 16 B: `u32 magic 'TLNK' 0x4B4E4C54 | u16 type | u16 flags | u32 len | u32 seq`, then `len` bytes of payload.
USB bulk is CRC-protected with retries, so there is no payload CRC; the magic re-syncs the CDC byte stream after a
reconnect.

| type | dir | payload |
|---|---|---|
| 0x01 SF_BEGIN | Pi->T | u32 sf_seq, u16 total_sec, u16 flags. Opens a slot (allows cut-through stalls) |
| 0x02 WIRE | Pi->T | u32 sf_seq, u16 first_idx, u16 count, u32 fmt (4 = 4-bit, 1 = 1-bit), count x 528 (or 520) B wire images |
| 0x03 SF_END | Pi->T | u32 sf_seq. The slot is complete |
| 0x04 META | Pi->T | u32 lba, u16 count, count x 512 raw. Overrides procedural metadata (the Teensy encodes these itself) |
| 0x05 CONFIG | Pi->T | profile, stream_profile flag, TIMPOL tx/rx, hold limits (30/20/100 ms), slot geometry, VIDEO_MAX |
| 0x06 DOWNLINK | Pi->T | n x u32 W32 reply words |
| 0x07 FLUSH | Pi->T | drop all slots >= seq (encoder restart) |
| 0x81 HOST_WRITE | T->Pi | u32 lba, u8 crc_ok, 512 B |
| 0x82 UPLINK | T->Pi | reassembled W32 message |
| 0x83 STATUS | T->Pi, 20 Hz | free pool sectors (credits), newest/served seq, underruns, CRC7 rejects, write CRC errors, hold events, state, bus width |
| 0x84 CMDLOG | T->Pi | {u32 t_us, u8 idx (bit6 = ACMD), u32 arg, u8 responded} records (bring-up; can be disabled) |
| 0x85 SERVED | T->Pi | u32 sf_seq, u32 t_us of its first start bit (latency) |

IMPLEMENTED 2026-11-04 (`pi/avlink/tlink.h` = `teensy/sdemu/avstream.h`): CONFIG = 16 B {u16 hold_cut_ms (30),
u16 hold_future_ms (20), u16 silent_ms (100), u16 stale_ms (200), u8 status_hz (20), 7 B reserved; 0 = keep}.
STATUS = 80 B {u16 ver 1, u16 size, u32 free_sectors, rx_sectors, free_slots, begun, newest_seq, served_seq,
sectors_served, holds, hold_timeouts, notready, expired, idle, evicted, converted, rx_errors, crc7_rejects,
write_crc_errors, u8 card_state, bus_width, mode, flags (bit0 newest valid, bit1 served valid, bit2 begun valid),
u32 pool_sectors}. Credits: the Pi may send WIRE sectors while `free_sectors - (sectors_sent - rx_sectors) >= n` and
SF_BEGIN while `free_slots - (begins_sent - begun) > 0` (totals are u32, differences only). The Teensy sends STATUS at
status_hz and within 2 ms of any credit change, only after the first TLNK message (the console shares the CDC port;
non-TLNK bytes are console text both ways). Slot reclaim (mode A): freed when 2 newer superframes were served; a
complete never-read slot that is not the newest is evicted after stale_ms (the RNS-E then gets EXPIRED and jumps);
on a credit overrun the oldest slot is evicted. Window reads: resident -> pool image (re-encoded if the image width
differs from the bus width), not received yet -> hold <= hold_cut_ms then NOT_READY; not begun -> hold <=
hold_future_ms then NOT_READY (or IDLE with seq = newest if the Pi was silent > silent_ms); older than the newest
begun and not resident -> EXPIRED at once; past total_sec -> zeros. The synthetic answer is reused for the rest of
that command. META / DOWNLINK are parsed and ignored (uplink work).

### 5.3 Pi software (`pi_link_sketch.c`)

The encoder thread emits superframes (header sector built per 3.2), then the wire-image builder (wire4 per sector;
unit-test it against `sd_wire.py`), then the USB writer, respecting credits and sending WIRE in 32-sector chunks for
cut-through. The reader thread handles STATUS (credits), UPLINK (rate control 3.5, NEED_KEY to the encoder),
HOST_WRITE (channel demux) and CMDLOG (to a file).

---------------------------------------------------------------------------------------------------

## 6. Bring-up plan

| stage | setup | what it proves | pass criteria |
|---|---|---|---|
| 0 | Teensy + Pi only | USB throughput and framing; wire-image builder == `sd_wire.py` | >= 15 MB/s sustained OUT for 10 min with no loss; images byte-identical |
| 1 | RNS-E on the bench with a **real** SD card through the extender + breakout; logic analyzer (>= 200 MS/s, >= 6 ch) and scope | real SDCLK in ID and transfer mode (390 kHz / 25 MHz?); duty cycle; real init order; pull-ups present (meter, no card); CD polarity; slot VDD switching; which physical slot is sd1 | numbers recorded into this spec (they replace the I tags in 1.3 and 2.A.1) |
| 2 | Teensy **passive sniffer** on the same bus (all pins inputs, real card still answering) | FlexIO pin-clocked CMD capture, CRC7 check, decoding at both clocks, CMDLOG over USB | the Teensy log matches the LA decode command for command, 0 CRC7 rejects over 1000 mounts/reads |
| 3 | patched build: **1-bit (80653152 E001) + /16 clock (8068277E E404 + table 8EAD11CA)**; Teensy replaces the card | init state machine; R1/R2/R3/R6 responses; CMD17 at 1-bit; SCR/SD status blocks; byte- vs block-addressing (CMDLOG); FAT mount | the RNS-E mounts the volume (check the driver's card-type variable and AFFM mount state through CAN memread); the task reads LBA 0x40100+ test pattern with 0 errors |
| 4 | 4-bit, still ~3.1 MHz (drop the 1-bit patch) | PWIDTH=3 path, ACMD6 switch, per-line CRC16 | 10^6 sectors test pattern, 0 errors |
| 5 | stock clock /4 then /2 (12.5 then 25 MHz) | data-valid window. Scope at the **slot pins**: DAT edge vs SDCLK rising, >= 5 ns setup / >= 2 ns hold; sweep TIMPOL (CORRECTED: FlexIO stays at 120 MHz, the datasheet max; rising launch expected); measure SDCLK duty; check CLK ringing | 10^8 sectors at 25 MHz, 0 CRC errors (the TMIO counts them as INFO2 errors: read them through the driver/CAN) |
| 6 | RNS-E task: mode A raw-read loop | real mode A throughput, per-call overhead, CPU load | >= 8 MB/s sustained, or note the real figure here |
| 7 | superframe container, audio only, s16 bit-exact (stock DSP format) | stalls/IDLE behaviour, drift loop, ring fill stability | 1 h without underrun; bit-exact capture at the SSI (DMA buffer compare) |
| 8 | video stub (fixed test frames at 5 MB/s), then the real codec | full read budget, latency (pi_time_us echo) | 60 superframes/s sustained, latency within 3.6 |
| 9 | mode B endless CMD18 + CMD60 W32 uplink | custom driver, STOP/STP semantics, resync paths (pull the Teensy reset mid-stream) | recovers in < 200 ms; status loop closed |
| 10 | CMD25 writes: W-slow at 3 MHz, then W-hw at 25 MHz | CRC token timing, busy flow control, capture channel | 10^5 blocks, 0 host write errors |

Stage 3 needs a patched firmware: build it as a normal rNN on the current bench lineage (r57 = stock lineage per
FACTS). The two literal patches are r19 addresses: **re-find them by byte pattern** (the study warns PCBBC/r42 builds
are relocated). The patches can go over CAN (flash-write-over-CAN is proven, memory: flash-write-fix) or on a burned
disc. Revert both for stage 5.

---------------------------------------------------------------------------------------------------

## 7. Open items and risks (ranked)

1. **SDCLK edge integrity at the FlexIO input** (a double count = a corrupt block). Mitigation: section 1.9; measured in stage 5.
2. **FlexIO latency at 25 MHz** (1.4). CORRECTED: with the RM's real latency (1.5-2.5 FlexIO clocks) rising launch gives ~7 ns of setup and ~13 ns of hold margin at the legal 120 MHz; 240 MHz is out of spec. 25 MHz exceeds the RM's generic FlexIO/6 slave rule, so this is the main bench item. Fallback: 12.5 MHz. 50 MHz is not feasible.
3. **CRC-token timing for writes** (1.8). Avoided by the CMD60 uplink until stage 10.
4. TMIO details marked I: STOP SEC=0 endless mode, clock gating when the buffer is full, response timeout length, CMD60 pass-through (the TMIO might reject unknown indices in "normal" mode; extended-mode bits as in 0x1C06 should force R1).
5. The stock raw-write entry point is not yet found (needed for mode A uplink writes).
6. CD polarity, sd1/sd2 slot identity, actual SD source clock (study open list, stage 1).

## 8. Documents needed

VERIFIED 2026-11-02: the RM, the CEC, the Teensy 4.1 schematic and SD Simplified Spec 9.10 are now in `datasheets/`
(see `SPEC_VERIFY.md`). Still missing: the full (non-simplified) SD timing tables (NCR/NWR/NCRC, tISU/tIH), PJRC
`imxrt.h`, and the Linux TMIO headers.

| document | why |
|---|---|
| NXP **IMXRT1060RM** (i.MX RT1060 Processor Reference Manual), chapters FlexIO, IOMUXC (pad mux/ALT tables, SW_PAD_CTL fields), DMAMUX (FLEXIO2 request numbers), eDMA (TCD scatter-gather), CCM (FLEXIO2 clock root) | every FlexIO register value in the Teensy sketch; pin ALT4 confirmation; whether shifters 4-7 have DMA |
| NXP **IMXRT1060CEC** datasheet | FLEXIO clock-root maximum, GPIO AC timing (output delay vs DSE/SRE) |
| PJRC Teensy 4.1 schematic + `imxrt.h` | pad <-> pin map, register macro names |
| **SD Physical Layer Simplified Specification** (v3.01 or later, free from sdcard.org) | default-speed AC timing (tODLY, tISU/tIH), NCR/NRC/NAC/NCRC, CRC status token and busy timing, R1 bits |
| Linux `drivers/mmc/host/tmio_mmc.h` / `renesas_sdhi` source | TMIO bit names used for mode B (CMD extension bits, INFO bits, STOP bits) |
