// config.h -- Teensy 4.1 SD-card emulator bring-up firmware: pins, constants, runtime knobs.
// Source of truth for every hardware value: ../link/SPEC.md + SPEC_VERIFY.md (RM page refs there).
#pragma once
#include <stdint.h>

#define FW_VERSION "sdemu 0.3 (2026-11-04, touch, live stream)"

// ---------------------------------------------------------------- pins (SPEC 1.2)
// Teensy pin   pad          FLEXIO2   SD signal (SD card pin / microSD pin)
//   6          GPIO_B0_10   D10       CLK  (SD 5 / uSD 5)
//   9          GPIO_B0_11   D11       CMD  (SD 2 / uSD 3)
//   8          GPIO_B1_00   D16       DAT0 (SD 7 / uSD 7)
//   7          GPIO_B1_01   D17       DAT1 (SD 8 / uSD 8)
//  36          GPIO_B1_02   D18       DAT2 (SD 9 / uSD 1)
//  37          GPIO_B1_03   D19       DAT3 (SD 1 / uSD 2)
//  14 (A0)     -            -         slot VDD sense via 10k (top) / 20k (bottom) divider
//  15          -            -         optional card-detect sense (only if your breakout has a CD pad)
//   2          -            -         debug pulse out (CRC error / command decode); leave unconnected
//   3          -            -         bus-switch OE out (car build only; high = connect)
//  13          -            -         on-board LED (status)
//  18 / 19     LPI2C1       -         GT911 touch SDA / SCL (Wire, 400 kHz, 3.3 V)
//  20          -            -         GT911 INT (address strap at reset, then input)
//  21          -            -         GT911 RST
#define PIN_CLK   6
#define PIN_CMD   9
#define PIN_DAT0  8
#define PIN_DAT1  7
#define PIN_DAT2  36
#define PIN_DAT3  37
#define PIN_VSENSE 14
#define PIN_CDSENSE 15
#define PIN_DBG   2
#define PIN_OE    3
#define PIN_LED   13
#define PIN_TINT  20
#define PIN_TRST  21

#define FX_CLK   10   // FLEXIO2 pin index of SDCLK
#define FX_CMD   11
#define FX_DAT0  16   // DAT0..3 = 16..19 (consecutive, required for PWIDTH=3)

// FlexIO2 resource map (SPEC_VERIFY section 3)
//   T0 = DAT word timer  (S0 DAT TX, S3 DAT RX)
//   T1 = CMD word timer  (S1 CMD TX, S2 CMD RX in card mode; S1 = CMD RX in monitor mode)
//   T2 = SDCLK edge counter (frequency meter, IRQ every FREQ_DIV clocks)
#define FREQ_DIV 256

// ---------------------------------------------------------------- VDD sense
// divider: slot VDD -- 10k -- pin14 -- 20k -- GND   => Vpin = Vslot * 2/3
#define VDIV_RATIO 1.5f
#define VDD_ON_V   2.70f   // slot volts: above = powered (hysteresis)
#define VDD_OFF_V  2.20f   // below = unpowered

// ---------------------------------------------------------------- card profile (SPEC 2.A)
#define CARD_RCA       0x5A01u
#define CARD_SECTORS   2097152u      // 1 GiB
#define LBA_PART       2048u
#define LBA_INFO       0x40000u
#define LBA_TESTPAT    0x40100u

// ---------------------------------------------------------------- runtime knobs (cfg command)
struct Knobs {
  uint8_t samp;       // RX sample edge: 0 = SDCLK rising (correct for SD), 1 = falling (sanity test)
  uint8_t launch;     // TX launch edge: 0 = rising (SPEC 1.4, required at 25 MHz), 1 = falling
  uint8_t wphase;     // word-timer enable: 0 = on SDCLK rising, 1 = on falling, 2 = free-running
  uint8_t dse;        // CMD/DAT pad drive strength 1..7 (4 = 39 ohm, 5 = 32 ohm)
  uint8_t sre;        // 0 = slow slew (default), 1 = fast
  uint8_t speed;      // pad SPEED field 0..3
  uint8_t hys;        // CMD/DAT input hysteresis
  uint8_t pull;       // 47k pull-ups on CMD/DAT while slot VDD present (card mode only)
  uint8_t acmd41busy; // ACMD41 polls answered "busy" before "ready"
  uint8_t illegal;    // set ILLEGAL_COMMAND (bit 22) after an unanswered illegal command (SPEC D5)
  uint8_t blockaddr;  // 0 = SDSC byte address (arg>>9), 1 = block address
  uint8_t pattern;    // data area: 1 = per-LBA test pattern, 0 = zeros
  uint8_t lbk;        // card mode: capture own DAT output with S3 and CRC-check it (loopback)
  uint8_t log;        // stream command log lines
  uint8_t dat;        // monitor mode: sniff DAT lines too
  uint8_t cdpin;      // report pin 15 card-detect level
  uint8_t stream;     // card mode: LBA 0x10000-0x1FFFF = live superframes from the Pi (1) / test pattern (0)
  uint8_t vddgate;    // 1 = drive only while slot VDD sensed (default); 0 = bench override, treat slot as powered
  uint8_t dlaunch;    // DAT TX launch edge, separate from CMD's: 0 = rising, 1 = falling
  uint8_t fastpat;    // bench: pattern area served from one pre-encoded sector (timing tests without per-sector CPU work)
};
extern Knobs K;
