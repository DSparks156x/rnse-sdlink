// fxsd.cpp -- FlexIO2 SD bus engine. Register values: ../link/SPEC_VERIFY.md section 3 (RM refs there).
//
// Monitor (S1, never drives):  T1 32-bit CMD word timer -> S1 RX (CMD) -> DMA ring (DMAMUX src 1)
//                              T0 4-bit word timer      -> S3 RX (DAT0-3, PWIDTH 3) -> DMA ring (src 65)
//                              T2 SDCLK edge counter (IRQ every FREQ_DIV clocks)
// Card (S2/S3):                T1 16-bit CMD word timer -> S2 RX (IRQ prio 0) + S1 TX (response)
//                              T0 DAT word timer        -> S0 TX (wire images by eDMA, src 1)
//                                                       -> S3 RX loopback capture (src 65, optional)
// Every timer counts SDCLK pin edges (TIMDEC=10), so the engine runs exactly at the host clock.
#include <Arduino.h>
#include <DMAChannel.h>
#include <string.h>
#include "config.h"
#include "fxsd.h"
#include "avstream.h"

// ---------------------------------------------------------------- register helpers
#define FXR(off)        (*(volatile uint32_t *)(0x401B0000u + (off)))
#define SHIFTCTL(n)     FXR(0x080 + 4 * (n))
#define SHIFTCFG(n)     FXR(0x100 + 4 * (n))
#define SHIFTBUF(n)     FXR(0x200 + 4 * (n))
#define SHIFTBUFBIS(n)  FXR(0x280 + 4 * (n))
#define TIMCTL(n)       FXR(0x400 + 4 * (n))
#define TIMCFG(n)       FXR(0x480 + 4 * (n))
#define TIMCMP(n)       FXR(0x500 + 4 * (n))

#define SCTL_TIMSEL(n)  ((uint32_t)(n) << 24)
#define SCTL_TIMPOL     (1u << 23)
#define SCTL_PINCFG(n)  ((uint32_t)(n) << 16)
#define SCTL_PINSEL(n)  ((uint32_t)(n) << 8)
#define SCTL_SMOD(n)    ((uint32_t)(n))
#define SCFG_PWIDTH(n)  ((uint32_t)(n) << 16)
#define TCTL_PINSEL(n)  ((uint32_t)(n) << 8)
#define TCTL_PINPOL     (1u << 7)
#define TCTL_TIMOD(n)   ((uint32_t)(n))
#define TCFG_TIMDEC(n)  ((uint32_t)(n) << 20)
#define TCFG_TIMENA(n)  ((uint32_t)(n) << 8)
#define MUX_SION        (1u << 4)

volatile FxCounters C;
FxState S;
Knobs K = {
  // wphase 1 = bench-proven on the RNS-E 2026-11-04 (0 corrupts 1 bit/word). launch 1 (falling) = the RNS-E rejects
  // CMD7's R1 at 25 MHz with launch 0 and completes init and reads with launch 1 (bench 2026-11-04)
  /*samp*/0, /*launch*/1, /*wphase*/1,
  /*dse*/4, /*sre*/0, /*speed*/1, /*hys*/1, /*pull*/1,
  /*acmd41busy*/2, /*illegal*/1, /*blockaddr*/0, /*pattern*/1, /*lbk*/0, /*log*/0, /*dat*/1, /*cdpin*/0, /*stream*/1, /*vddgate*/1, /*dlaunch*/1, /*fastpat*/0
};

// ---------------------------------------------------------------- live stream pool (SPEC 1.7, 5.1)
// Wire images from the Pi, 528 B each: OCRAM (DMAMEM, cached; only the CPU reads it -- the read engine copies an
// image into imgs[] in DTCM before the eDMA sees it, so no cache maintenance is needed) + a DTCM bank.
#define POOL_OCRAM 880
#define POOL_DTCM  192
DMAMEM static AvsSector pool_ocram[POOL_OCRAM] __attribute__((aligned(32)));
static AvsSector pool_dtcm[POOL_DTCM];
static AvsSector *const pool_banks[2] = {pool_ocram, pool_dtcm};
static const uint16_t pool_bank_n[2] = {POOL_OCRAM, POOL_DTCM};
static uint32_t avs_now(void) { return micros(); }
static void avs_tx(const uint8_t *p, int n) { if (Serial) Serial.write(p, (size_t)n); }

// pads in order CLK, CMD, DAT0..3
static volatile uint32_t *const MUXR[6] = {
  &IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_10, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_11,
  &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_00, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_01,
  &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_02, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_03};
static volatile uint32_t *const PADR[6] = {
  &IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_10, &IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_11,
  &IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_00, &IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_01,
  &IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_02, &IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_03};
static const uint8_t PINS[6] = {PIN_CLK, PIN_CMD, PIN_DAT0, PIN_DAT1, PIN_DAT2, PIN_DAT3};
static const uint8_t FXPIN[6] = {FX_CLK, FX_CMD, FX_DAT0, FX_DAT0 + 1, FX_DAT0 + 2, FX_DAT0 + 3};

// ---------------------------------------------------------------- state
static DMAChannel dtx(false), crx(false), drx(false);
#define CRING_WORDS 2048u
#define DRING_WORDS 16384u
static uint32_t cring[CRING_WORDS] __attribute__((aligned(CRING_WORDS * 4)));
static uint32_t dring[DRING_WORDS] __attribute__((aligned(DRING_WORDS * 4)));
static volatile uint32_t crx_laps, drx_laps;
static uint64_t c_rpos, d_rpos;            // ring read positions (words / symbols)
static uint8_t d_fmt4 = 1;                 // DAT ring format: 1 = nibble words, 0 = DAT0 bit words
static uint8_t d_waiting; static uint64_t d_wait_pos; static uint32_t d_wait_ms;

static CmdParser cmdp;
static volatile uint32_t freq_flags;
static uint32_t s0ctl, s1ctl;              // shifter control base values (PINCFG = 0)
static volatile uint8_t drive_ok;          // card mode && slot VDD present
static volatile uint32_t pend_err;         // R1 error bits reported in the next response
static volatile uint8_t acmd41_polls;
static uint32_t last_read_arg, last_read_t;

// SHIFTSIEN read-modify-write from below prio 0 (the top half also changes it)
static inline void sien_set(uint32_t m) { __disable_irq(); FLEXIO2_SHIFTSIEN |= m; __enable_irq(); }
static inline void sien_clr(uint32_t m) { __disable_irq(); FLEXIO2_SHIFTSIEN &= ~m; __enable_irq(); }

// Card CMD path is split (bench 2026-11-04: the RNS-E goes to ~25 MHz before CMD7, one 16-bit word = 640 ns):
//   top half    fx_isr, prio 0: S2 RX word -> rxq, S1 TX feed. Moves words only.
//   bottom half cmd_bh, prio 16: parser, CRC, card_cmd, respond, log.
#define IRQ_CMDBH IRQ_SPDIF                  // unused here; pended by software only
#define RXQ_N 64u
static volatile uint32_t rxq[RXQ_N];
static volatile uint32_t rxq_h, rxq_t;

// word-level trace of the top half: 'trace' arms it, it freezes 48 words after the next CMD7 is decoded
struct TraceRec { uint32_t cyc; uint16_t w; uint8_t ss, txi; };
#define TR_N 128u
static TraceRec tr[TR_N];
static volatile uint32_t tr_h;
static volatile int32_t tr_left = -1;             // -1 off, 0x7FFFFFFF armed, else words until freeze (0 = frozen)

// CMD TX
static uint32_t tx_w[10];
static volatile uint8_t tx_n, tx_i, tx_pad;
static volatile uint8_t tx_start;             // first word waits for the top half (just after a word boundary)
static uint8_t q_resp[17];                   // response deferred while the previous one is still on the wire
static volatile uint8_t q_n;
static volatile uint8_t echo_pending;        // next parsed token should be our own response

// read engine
#define NIMG 3
struct Img { uint32_t w[WIRE_MAX_WORDS]; uint16_t n; };
static Img imgs[NIMG];
static volatile uint32_t rd_gen, dma_gen, rd_next_lba;
static volatile uint8_t rd_active, rd_multi, rd_small, q_head, q_count, dma_busy, rel_pending, built_cnt, rd_hold;
static const uint8_t *small_ptr; static int small_len;
static Img fast; static uint8_t fast_ready, fast_bus4;

// log ring
#define LOG_N 512
static LogRec logr[LOG_N];
static volatile uint16_t log_head, log_tail;
volatile uint32_t fx_log_dropped;

static void log_push(const LogRec &r) {
  __disable_irq();
  uint16_t h = log_head, n = (uint16_t)((h + 1) % LOG_N);
  if (n == log_tail) fx_log_dropped++;
  else { logr[h] = r; log_head = n; }
  __enable_irq();
}
int fx_log_pop(LogRec &r) {
  if (log_tail == log_head) return 0;
  r = logr[log_tail];
  log_tail = (uint16_t)((log_tail + 1) % LOG_N);
  return 1;
}
static void log_tok(const CmdTok &t, uint8_t responded) {
  if (!K.log) return;
  LogRec r;
  memset(&r, 0, sizeof r);
  r.t_us = micros(); r.kind = t.host ? 'C' : 'R'; r.idx = t.idx; r.app = t.app; r.crc = t.crc;
  r.responded = responded; r.nbytes = t.nbytes; r.arg = t.arg; memcpy(r.b, t.b, 17);
  log_push(r);
}
static void log_event(uint8_t code, uint32_t arg) {
  LogRec r;
  memset(&r, 0, sizeof r);
  r.t_us = micros(); r.kind = 'E'; r.idx = code; r.arg = arg;
  log_push(r);
}

// ---------------------------------------------------------------- pads
static void pads_off(void) {
  for (int i = 0; i < 6; i++) {
    pinMode(PINS[i], INPUT);
    *PADR[i] = IOMUXC_PAD_HYS;               // no pull, no keeper, no drive: high impedance
  }
}
static uint32_t pad_io_value(void) {
  uint32_t v = IOMUXC_PAD_DSE(K.dse ? K.dse : 1) | IOMUXC_PAD_SPEED(K.speed) | (K.sre ? IOMUXC_PAD_SRE : 0) |
               (K.hys ? IOMUXC_PAD_HYS : 0);
  if (S.mode == FX_CARD && S.vdd_ok && K.pull) v |= IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(1);
  return v;
}
static void pads_flexio(void) {
  *PADR[0] = IOMUXC_PAD_HYS;                 // CLK: input only, no pull/keeper (SPEC_VERIFY P14)
  *MUXR[0] = 4 | MUX_SION;                   // SION: force the input path (harmless, never drives)
  uint32_t io = (S.mode == FX_CARD) ? pad_io_value() : IOMUXC_PAD_HYS;   // monitor: no pulls ever
  for (int i = 1; i < 6; i++) {
    *PADR[i] = io;
    *MUXR[i] = 4 | MUX_SION;                 // ALT4 = FLEXIO2 (RM p514-523); SION so S2/S3 read back our own drive
  }
}
static void pads_update_pulls(void) {        // VDD changed in card mode
  if (S.mode != FX_CARD) return;
  uint32_t io = pad_io_value();
  for (int i = 1; i < 6; i++) *PADR[i] = io;
}

// ---------------------------------------------------------------- DMA rings
static void crx_isr(void) { crx.clearInterrupt(); crx_laps++; asm volatile("dsb"); }
static void drx_isr(void) { drx.clearInterrupt(); drx_laps++; asm volatile("dsb"); }

static void dmamux_off(DMAChannel &ch) { *(&DMAMUX_CHCFG0 + ch.channel) = 0; }

static void ring_start(DMAChannel &ch, volatile uint32_t &src, uint32_t *ring, uint32_t words,
                       uint8_t mux, void (*isr)(void), volatile uint32_t &laps) {
  ch.disable();
  laps = 0;
  memset(ring, 0xFF, words * 4);
  ch.TCD->CSR = 0;
  ch.source(src);
  ch.destinationCircular(ring, words * 4);
  ch.interruptAtCompletion();
  ch.triggerAtHardwareEvent(mux);
  ch.attachInterrupt(isr, 64);
  ch.enable();
}
static uint64_t ring_wpos(DMAChannel &ch, volatile uint32_t &laps, uint32_t *ring, uint32_t words) {
  uint32_t l1, a, l2;
  do { l1 = laps; a = (uint32_t)ch.TCD->DADDR; l2 = laps; } while (l1 != l2);
  uint32_t idx = ((a - (uint32_t)ring) / 4u) & (words - 1);
  return (uint64_t)l1 * words + idx;
}

// ---------------------------------------------------------------- FlexIO setup
static void flexio_clock(void) {
  CCM_CCGR3 &= ~CCM_CCGR3_FLEXIO2(3);
  CCM_CSCMR2 = (CCM_CSCMR2 & ~CCM_CSCMR2_FLEXIO2_CLK_SEL(3)) | CCM_CSCMR2_FLEXIO2_CLK_SEL(3);  // pll3_sw 480
  CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_FLEXIO2_CLK_PRED(7) | CCM_CS1CDR_FLEXIO2_CLK_PODF(7))) |
               CCM_CS1CDR_FLEXIO2_CLK_PRED(1) | CCM_CS1CDR_FLEXIO2_CLK_PODF(1);                 // /2 /2 = 120
  CCM_CCGR3 |= CCM_CCGR3_FLEXIO2(3);
}
static void flexio_reset(void) {
  FLEXIO2_CTRL = 2;            // SWRST
  FLEXIO2_CTRL = 0;
  FLEXIO2_SHIFTSIEN = 0; FLEXIO2_SHIFTEIEN = 0; FLEXIO2_TIMIEN = 0; FLEXIO2_SHIFTSDEN = 0;
}

static uint32_t t_ctl, t_cfg, rxpol, txpol, dtxpol;
static void phase_values(void) {
  uint32_t inv = (K.wphase == 1) ? 1 : 0;
  t_ctl = TCTL_PINSEL(FX_CLK) | (inv ? TCTL_PINPOL : 0) | TCTL_TIMOD(3);       // 16-bit counter on SDCLK
  t_cfg = TCFG_TIMDEC(2) | TCFG_TIMENA(K.wphase == 2 ? 0 : 4);                 // pin both edges; enable edge
  rxpol = ((K.samp ? 1u : 0u) ^ inv) ? SCTL_TIMPOL : 0;
  txpol = ((K.launch ? 1u : 0u) ^ inv) ? SCTL_TIMPOL : 0;
  dtxpol = ((K.dlaunch ? 1u : 0u) ^ inv) ? SCTL_TIMPOL : 0;
}
static void freq_timer(void) {
  TIMCMP(2) = 2 * FREQ_DIV - 1;
  TIMCFG(2) = TCFG_TIMDEC(2);
  TIMCTL(2) = TCTL_PINSEL(FX_CLK) | TCTL_TIMOD(3);
  FLEXIO2_TIMIEN = 4;
}

// ---------------------------------------------------------------- DAT ring control
static void dat_ring_restart(uint8_t fmt4) {
  drx.disable();
  d_fmt4 = fmt4;
  d_rpos = 0; d_waiting = 0;
  ring_start(drx, FLEXIO2_SHIFTBUF3, dring, DRING_WORDS, DMAMUX_SOURCE_FLEXIO2_REQUEST2, drx_isr, drx_laps);
}

// ---------------------------------------------------------------- card: CMD TX
FASTRUN static void respond(const uint8_t *b, int n) {
  if (!drive_ok) { C.suppressed_novdd++; return; }
  __disable_irq();                           // tx_feed (top half) also uses tx_*, q_* and calls us
  if ((FLEXIO2_SHIFTSIEN & 2u) || tx_start) {  // previous response (e.g. a 136-bit R2) still shifting out:
    memcpy(q_resp, b, n); q_n = (uint8_t)n;  // restarting now duplicates a word; send it on release
    __enable_irq();
    return;
  }
  tx_n =(uint8_t)pack_resp16(b, n, tx_w); tx_i = 0; tx_pad = 0;
  // Pad first: the shifter holds idle ones, so driving now is harmless. The other order lost the first 1-2 bits at
  // 25 MHz when a word boundary fell between the buffer load and the slow SHIFTCTL write (bench 2026-11-04).
  SHIFTCTL(1) = s1ctl | SCTL_PINCFG(3);
  // First word is written by the top half on the next S2 RX word, i.e. right after a T1 boundary. A write landing on
  // the boundary could be lost, tx_feed then sent word 2 first: first 16 bits missing, host CRC error (bench 2026-11-05).
  tx_start = 1;
  echo_pending = 1;
  C.responded++;
  __enable_irq();
}
FASTRUN static void tx_feed(void) {
  if (!tx_pad && (FLEXIO2_SHIFTERR & 2)) { FLEXIO2_SHIFTERR = 2; C.tx_underrun_resp++; }   // response corrupted
  if (tx_i < tx_n) SHIFTBUFBIS(1) = tx_w[tx_i++];
  else if (!tx_pad) { SHIFTBUF(1) = 0xFFFFFFFFu; tx_pad = 1; }
  else {                                     // last word fully out: release CMD
    SHIFTCTL(1) = s1ctl; FLEXIO2_SHIFTSIEN &= ~2u;
    if (q_n) { uint8_t n = q_n; q_n = 0; respond(q_resp, n); }
  }
}

// ---------------------------------------------------------------- card: read engine
// Masks the bottom half and below but NOT the prio-0 top half: the TCD writes below outlast a 640 ns word at 25 MHz,
// so a full __disable_irq here lost CMD RX words (rx_overrun, host CRC7 bad) during reads.
static inline uint32_t bh_lock(void) {
  uint32_t o; asm volatile("mrs %0, basepri" : "=r"(o));
  asm volatile("msr basepri, %0" :: "r"(16u));
  return o;
}
static inline void bh_unlock(uint32_t o) { asm volatile("msr basepri, %0" :: "r"(o)); }

static uint32_t rd_block_t;                  // micros() of the last block finished by the DMA
FASTRUN static void kick_dma(void) {
  uint32_t lk = bh_lock();
  if (rd_active && !dma_busy && q_count > 0) {
    Img &im = imgs[q_head];
    dtx.TCD->SADDR = im.w;
    dtx.TCD->SOFF = 4;
    dtx.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);
    dtx.TCD->NBYTES = 4;
    dtx.TCD->SLAST = 0;
    dtx.TCD->DADDR = &FLEXIO2_SHIFTBUF0;
    dtx.TCD->DOFF = 0;
    dtx.TCD->CITER = im.n;
    dtx.TCD->DLASTSGA = 0;
    dtx.TCD->BITER = im.n;
    dtx.TCD->CSR = DMA_TCD_CSR_INTMAJOR | DMA_TCD_CSR_DREQ;
    dma_gen = rd_gen; dma_busy = 1;
    FLEXIO2_SHIFTSDEN |= 1;
    dtx.enable();
  }
  bh_unlock(lk);
}
// returns 0 when the sector is not available yet (live stream: hold the start bit, ask again later)
static int build_img(Img &im, uint32_t lba) {
  if (rd_small) {
    im.n = (uint16_t)(S.bus4 ? wire4(small_ptr, small_len, im.w) : wire1(small_ptr, small_len, im.w));
  } else if (K.fastpat && lba >= LBA_TESTPAT && lba - LBA_TESTPAT < 0x10000u) {   // bench: one pre-encoded sector, no per-sector work
    if (!fast_ready || fast_bus4 != S.bus4) {
      uint8_t sec[512];
      pattern_sector(LBA_TESTPAT, sec);
      fast.n = (uint16_t)(S.bus4 ? wire4(sec, 512, fast.w) : wire1(sec, 512, fast.w));
      fast_bus4 = S.bus4; fast_ready = 1;
    }
    memcpy(im.w, fast.w, (size_t)fast.n * 4); im.n = fast.n;
  } else if (K.stream && lba - AVS_WIN_LBA < AVS_WIN_SECT) {   // superframe window: Pi stream / synthetic
    if (AVS.get(lba, S.bus4, im.w, &im.n) == AVS_HOLD) return 0;
  } else {
    uint8_t sec[512];
    gen_sector(lba, sec, K.pattern, fx_info_sector);
    im.n = (uint16_t)(S.bus4 ? wire4(sec, 512, im.w) : wire1(sec, 512, im.w));
  }
  return 1;
}
static void pump(void) {                     // IRQ_SOFTWARE, prio 64 (and called from the DMA ISR)
  kick_dma();
  for (;;) {
    uint32_t gen = rd_gen;
    if (!rd_active || q_count >= NIMG) break;
    if (!rd_multi && built_cnt >= 1) break;
    int slot = (q_head + q_count) % NIMG;
    uint32_t lba = rd_next_lba;
    if (!build_img(imgs[slot], lba)) { rd_hold = 1; break; }   // fx_poll re-pends us
    __disable_irq();
    int ok = (gen == rd_gen) && rd_active;
    if (ok) { q_count++; rd_next_lba = lba + 1; built_cnt++; }
    __enable_irq();
    if (!ok) return;
    kick_dma();
  }
}
static void dtx_isr(void) {
  dtx.clearInterrupt();
  asm volatile("dsb");
  if (!dma_busy || dma_gen != rd_gen) return;
  dma_busy = 0;
  rd_block_t = micros();
  q_head = (uint8_t)((q_head + 1) % NIMG);
  q_count--;
  if (rd_small) C.small_blocks++; else C.sectors_served++;
  if (!rd_multi) { rel_pending = 1; sien_set(1); }   // release DAT when the pad word loads
  else pump();
}
FASTRUN static void s0_release_now(void) {
  FLEXIO2_SHIFTSIEN &= ~1u;
  if (rel_pending) {
    rel_pending = 0;
    SHIFTCTL(0) = s0ctl;
    FLEXIO2_SHIFTSDEN &= ~1u;
    rd_active = 0;
    if (S.card_state == ST_DATA) S.card_state = ST_TRAN;
  }
}
FASTRUN static void stop_read(void) {
  rd_active = 0; rd_gen++;
  dtx.disable();
  FLEXIO2_SHIFTSDEN &= ~1u;
  sien_clr(1);
  SHIFTCTL(0) = s0ctl;                       // release DAT
  SHIFTBUF(0) = 0xFFFFFFFFu;                 // next loads are idle ones
  dma_busy = 0; q_count = 0; q_head = 0; rel_pending = 0; built_cnt = 0;
}
FASTRUN static void start_read(uint32_t lba, uint8_t multi, const uint8_t *small, int slen) {
  stop_read();
  if (!drive_ok) return;
  rd_multi = multi; rd_small = small ? 1 : 0; small_ptr = small; small_len = slen;
  rd_next_lba = lba; built_cnt = 0; rd_hold = 0;
  AVS.read_begin();
  rd_active = 1;
  SHIFTCTL(0) = s0ctl | SCTL_PINCFG(3);      // drive idle ones until the first image (start-bit hold)
  NVIC_SET_PENDING(IRQ_SOFTWARE);
}

// ---------------------------------------------------------------- card: bus width
static void set_width(uint8_t w4) {
  S.bus4 = w4;
  TIMCTL(0) = 0;                             // timer off while changing
  TIMCMP(0) = w4 ? 15 : 63;                  // 8 clocks/word (4-bit) or 32 (1-bit)
  SHIFTCFG(0) = SCFG_PWIDTH(w4 ? 3 : 0);
  SHIFTCFG(3) = SCFG_PWIDTH(w4 ? 3 : 0);
  SHIFTBUF(0) = 0xFFFFFFFFu;
  TIMCTL(0) = t_ctl;
}

// ---------------------------------------------------------------- card: command handler (ISR, prio 0)
static inline uint32_t stat(int app) {
  uint32_t v = ((uint32_t)S.card_state << 9) | R1_READY_FOR_DATA | (app ? R1_APP_CMD : 0) | pend_err;
  pend_err = 0;
  return v;
}
static inline int illegal(void) { C.illegal++; if (K.illegal) pend_err |= R1_ILLEGAL_CMD; return 0; }

static void card_reset(void) {
  stop_read();
  q_n = 0; tx_start = 0;
  S.card_state = ST_IDLE; acmd41_polls = 0; pend_err = 0;
  set_width(0);
  cmdp.last55 = 0;
}

FASTRUN static int card_cmd(uint8_t idx, uint32_t arg, uint8_t app) {
  uint8_t r[6];
  uint8_t st = S.card_state;
  if (app) {
    switch (idx) {
      case 41: {
        C.acmd41++;
        if (st != ST_IDLE && st != ST_READY) return illegal();
        uint32_t ocr = 0x00FF8000u | ((++acmd41_polls > K.acmd41busy) ? 0x80000000u : 0);
        make_r3(r, ocr); respond(r, 6);
        if (ocr >> 31) S.card_state = ST_READY;
        return 1;
      }
      case 6:
        C.acmd6++;
        if (st != ST_TRAN) return illegal();
        make_r1(r, 6, stat(1)); respond(r, 6);
        set_width((arg & 3) == 2);
        log_event(6, arg & 3);
        return 1;
      case 13:
        C.acmd13++;
        if (st != ST_TRAN) return illegal();
        make_r1(r, 13, stat(1)); respond(r, 6);
        S.card_state = ST_DATA;
        start_read(0, 0, S.bus4 ? SDSTAT4 : SDSTAT1, 64);
        return 1;
      case 51:
        C.acmd51++;
        if (st != ST_TRAN) return illegal();
        make_r1(r, 51, stat(1)); respond(r, 6);
        S.card_state = ST_DATA;
        start_read(0, 0, SCR_BYTES, 8);
        return 1;
      case 42: case 23:                       // SET_CLR_CARD_DETECT, SET_WR_BLK_ERASE_COUNT: R1 only
        if (st != ST_TRAN) return illegal();
        make_r1(r, idx, stat(1)); respond(r, 6);
        return 1;
      default:                                // CPRM ACMD18/25/26/38/43-49, ACMD22 ...
        return illegal();
    }
  }
  switch (idx) {
    case 0:
      C.cmd0++;
      card_reset();
      return 0;
    case 8:
      C.cmd8++;                               // SD 1.x profile: no reply (SPEC 2.A.1 step 3)
      return illegal();
    case 55:
      if (st >= ST_STBY && (arg >> 16) != CARD_RCA) return 0;
      make_r1(r, 55, stat(1)); respond(r, 6);
      return 1;
    case 2:
      C.cmd2++;
      if (st != ST_READY) return illegal();
      respond(R2_CID, 17); S.card_state = ST_IDENT;
      return 1;
    case 3: {
      C.cmd3++;
      if (st != ST_IDENT && st != ST_STBY) return illegal();
      uint32_t e = pend_err; pend_err = 0;
      uint32_t f = (CARD_RCA << 16) | ((e & R1_COM_CRC_ERR) ? 0x8000u : 0) | ((e & R1_ILLEGAL_CMD) ? 0x4000u : 0) |
                   ((uint32_t)st << 9) | R1_READY_FOR_DATA;
      make_r1(r, 3, f); respond(r, 6);
      S.card_state = ST_STBY;
      return 1;
    }
    case 9: case 10:
      if (idx == 9) C.cmd9++;
      if (st != ST_STBY || (arg >> 16) != CARD_RCA) return illegal();
      respond(idx == 9 ? R2_CSD : R2_CID, 17);
      return 1;
    case 7:
      C.cmd7++;
      if (tr_left == 0x7FFFFFFF) tr_left = 48;
      if ((arg >> 16) == CARD_RCA) {
        if (st != ST_STBY && st != ST_TRAN) return illegal();
        make_r1(r, 7, stat(0)); respond(r, 6);
        S.card_state = ST_TRAN;
        return 1;
      }
      if (st == ST_TRAN || st == ST_DATA) { stop_read(); S.card_state = ST_STBY; }   // deselect, no reply
      return 0;
    case 13:
      C.cmd13++;
      if (st < ST_STBY || (arg >> 16) != CARD_RCA) return 0;
      make_r1(r, 13, stat(0)); respond(r, 6);
      return 1;
    case 16:
      C.cmd16++;
      if (st != ST_TRAN) return illegal();
      make_r1(r, 16, stat(0) | (arg != 512 ? R1_BLOCK_LEN_ERR : 0)); respond(r, 6);
      return 1;
    case 17: case 18: {
      if (idx == 17) C.cmd17++; else C.cmd18++;
      if (st != ST_TRAN) return illegal();
      uint32_t now = micros();
      if (arg == last_read_arg && (now - last_read_t) < 200000u) C.retries++;
      last_read_arg = arg; last_read_t = now;
      uint32_t lba = K.blockaddr ? arg : (arg >> 9);
      S.last_lba = lba;
      make_r1(r, idx, stat(0)); respond(r, 6);
      S.card_state = ST_DATA;
      start_read(lba, idx == 18, nullptr, 0);
      return 1;
    }
    case 12:
      C.cmd12++;
      if (st != ST_DATA) return illegal();
      stop_read();                            // release DAT first: the R1b host then sees DAT0 high
      make_r1(r, 12, stat(0)); respond(r, 6);
      S.card_state = ST_TRAN;
      return 1;
    case 24: case 25:
      C.writes_refused++;                     // writes are not part of this bring-up firmware
      return illegal();
    case 4: case 15:
      return 0;
    default:
      C.other++;
      return illegal();
  }
}

struct CmdRec { uint32_t t_us, arg; uint8_t idx, st, q, flg; };
#define CMDLOG_N 64u
static CmdRec cmdlog[CMDLOG_N];
static volatile uint32_t cmdlog_h;
int fx_cmdlog_get(uint32_t i, uint32_t *t, uint32_t *arg, uint8_t *idx, uint8_t *st, uint8_t *q, uint8_t *flg) {
  uint32_t h = cmdlog_h, n = h < CMDLOG_N ? h : CMDLOG_N;
  if (i >= n) return 0;
  const CmdRec &r = cmdlog[(h - n + i) & (CMDLOG_N - 1)];
  *t = r.t_us; *arg = r.arg; *idx = r.idx; *st = r.st; *q = r.q; *flg = r.flg;
  return 1;
}
void fx_read_state(char *o, int n) {
  snprintf(o, (size_t)n, "RD now=%lu active=%u multi=%u dma_busy=%u q=%u built=%u hold=%u next_lba=%lu last_block_t=%lu state=%u s0ctl_pincfg=%lu rel=%u",
           (unsigned long)micros(), rd_active, rd_multi, dma_busy, q_count, built_cnt, rd_hold, (unsigned long)rd_next_lba, (unsigned long)rd_block_t,
           S.card_state, (unsigned long)((SHIFTCTL(0) >> 16) & 3u), rel_pending);
}

FASTRUN static void card_tok(const CmdTok &t) {
  uint8_t echo = echo_pending;
  echo_pending = 0;
  // A damaged echo can parse as a host command with a bad CRC. It must not raise COM_CRC_ERR in the next real R1.
  if (!t.host || (echo && t.crc != 1)) {      // our own response echoed on CMD (or garbage)
    C.resp_seen++;
    if (!t.crc) C.resp_crc_bad++;
    if (!t.crc || K.log >= 2) log_tok(t, 0);  // bad echoes always, all echoes at log 2
    return;
  }
  C.host_cmds++;
  {
    uint32_t h = cmdlog_h;
    CmdRec &r = cmdlog[h & (CMDLOG_N - 1)];
    r.t_us = micros(); r.arg = t.arg; r.idx = t.idx; r.st = S.card_state; r.q = q_count;
    r.flg = (uint8_t)((t.app ? 1 : 0) | (t.crc == 1 ? 2 : 0) | (rd_active ? 4 : 0) | (dma_busy ? 8 : 0) | (rd_hold ? 16 : 0));
    cmdlog_h = h + 1;
  }
  if (t.crc != 1) {
    C.host_crc_bad++;
    pend_err |= R1_COM_CRC_ERR;
    digitalWriteFast(PIN_DBG, HIGH);
    log_tok(t, 0);
    digitalWriteFast(PIN_DBG, LOW);
    return;
  }
  int resp = card_cmd(t.idx, t.arg, t.app);
  if (!resp) C.unanswered++;
  if (!resp && !t.app && t.idx == 55) cmdp.last55 = 0;   // unanswered CMD55: next command is not an ACMD
  log_tok(t, (uint8_t)resp);
}

// ---------------------------------------------------------------- monitor token handler (main loop)
static void mon_tok(const CmdTok &t) {
  if (t.host) {
    C.host_cmds++;
    if (t.crc != 1) { C.host_crc_bad++; digitalWriteFast(PIN_DBG, HIGH); digitalWriteFast(PIN_DBG, LOW); }
    if (t.app && t.idx == 6) S.bus4 = (t.arg & 3) == 2;
    if (!t.app && t.idx == 0) S.bus4 = 0;
    if (!t.app && (t.idx == 17 || t.idx == 18)) S.last_lba = t.arg;
    digitalToggleFast(PIN_LED);
  } else {
    C.resp_seen++;
    if (t.crc == 0) C.resp_crc_bad++;
  }
  log_tok(t, 0);
}

// ---------------------------------------------------------------- IRQ handlers
FASTRUN static void fx_isr(void) {
  uint32_t ss = FLEXIO2_SHIFTSTAT;
  uint32_t sien = FLEXIO2_SHIFTSIEN;
  if (S.mode == FX_CARD) {
    if (ss & 4) {
      if (tx_start) {                        // fresh word boundary: a full word time before the next load
        tx_start = 0;
        SHIFTBUFBIS(1) = tx_w[tx_i++];
        FLEXIO2_SHIFTERR = 2;                // forget idle underruns; a new one = late feed (counted)
        FLEXIO2_SHIFTSIEN |= 2;
        sien &= ~2u;                         // not this pass: the status bit read above is stale
      }
      uint32_t w = SHIFTBUFBIS(2) & 0xFFFFu; // 16 newest CMD bits in [15:0], oldest at bit 15
      uint32_t h = rxq_h;
      if (w != 0xFFFFu || cmdp.busy || h != rxq_t) {   // nothing to parse in idle words
        if (h - rxq_t >= RXQ_N) C.rx_overrun++;
        else { rxq[h & (RXQ_N - 1)] = w; rxq_h = h + 1; NVIC_SET_PENDING(IRQ_CMDBH); }
      }
      if (tr_left > 0) {
        TraceRec &t = tr[tr_h & (TR_N - 1)];
        t.cyc = ARM_DWT_CYCCNT; t.w = (uint16_t)w; t.ss = (uint8_t)ss; t.txi = tx_i;
        tr_h = tr_h + 1;
        if (tr_left != 0x7FFFFFFF) tr_left = tr_left - 1;
      }
    }
    if ((ss & 2) && (sien & 2)) tx_feed();
    if ((ss & 1) && (sien & 1)) s0_release_now();
  }
  if (FLEXIO2_TIMSTAT & 4) { FLEXIO2_TIMSTAT = 4; freq_flags++; }
  asm volatile("dsb");
}

static void cmd_bh(void) {                   // bottom half, prio 16: everything that takes more than a word time
  while (rxq_t != rxq_h) {
    uint32_t t = rxq_t;
    uint32_t w = rxq[t & (RXQ_N - 1)];
    rxq_t = t + 1;
    cmdp.feed(w << 16, 16);
  }
}

void fx_trace_arm(void) { tr_h = 0; tr_left = 0x7FFFFFFF; }
int fx_trace_get(uint32_t i, uint32_t *cyc, uint16_t *w, uint8_t *ss, uint8_t *txi) {
  if (tr_left != 0) return -1;               // not frozen yet
  uint32_t h = tr_h, n = h < TR_N ? h : TR_N;
  if (i >= n) return 0;
  const TraceRec &t = tr[(h - n + i) & (TR_N - 1)];
  *cyc = t.cyc; *w = t.w; *ss = t.ss; *txi = t.txi;
  return 1;
}

// ---------------------------------------------------------------- mode control
static void engine_stop(void) {
  NVIC_DISABLE_IRQ(IRQ_FLEXIO2);
  NVIC_DISABLE_IRQ(IRQ_CMDBH);
  rxq_h = 0; rxq_t = 0;
  dtx.disable(); crx.disable(); drx.disable();
  dmamux_off(dtx); dmamux_off(crx); dmamux_off(drx);
  rd_active = 0; rd_gen++; dma_busy = 0; q_count = 0; q_head = 0; rel_pending = 0;
  drive_ok = 0; tx_start = 0;
  flexio_reset();
  digitalWriteFast(PIN_OE, LOW);
}

static void engine_start(void) {
  phase_values();
  cmdp.reset();
  c_rpos = 0; d_rpos = 0; d_waiting = 0;
  pads_flexio();
  freq_timer();
  if (S.mode == FX_MON) {
    cmdp.on_tok = mon_tok;
    TIMCMP(0) = 15; TIMCFG(0) = t_cfg;       // DAT: 4-bit capture, 8 clocks per word
    TIMCMP(1) = 63; TIMCFG(1) = t_cfg;       // CMD: 32 clocks per word
    SHIFTCFG(1) = 0;
    SHIFTCTL(1) = SCTL_TIMSEL(1) | rxpol | SCTL_PINSEL(FX_CMD) | SCTL_SMOD(1);
    SHIFTCFG(3) = SCFG_PWIDTH(3);
    SHIFTCTL(3) = SCTL_TIMSEL(0) | rxpol | SCTL_PINSEL(FX_DAT0) | SCTL_SMOD(1);
    TIMCTL(0) = t_ctl; TIMCTL(1) = t_ctl;
    FLEXIO2_SHIFTSDEN = 2u | (K.dat ? 8u : 0u);
    ring_start(crx, FLEXIO2_SHIFTBUFBIS1, cring, CRING_WORDS, DMAMUX_SOURCE_FLEXIO2_REQUEST1, crx_isr, crx_laps);
    if (K.dat) { d_fmt4 = 1; ring_start(drx, FLEXIO2_SHIFTBUF3, dring, DRING_WORDS, DMAMUX_SOURCE_FLEXIO2_REQUEST2, drx_isr, drx_laps); }
  } else {
    cmdp.on_tok = card_tok;
    S.card_state = ST_IDLE; S.bus4 = 0; acmd41_polls = 0; pend_err = 0;
    TIMCMP(0) = 63; TIMCFG(0) = t_cfg;       // DAT: 1-bit until ACMD6
    TIMCMP(1) = 31; TIMCFG(1) = t_cfg;       // CMD: 16 clocks per word (response latency, SPEC 2.A.2)
    s0ctl = SCTL_TIMSEL(0) | dtxpol | SCTL_PINSEL(FX_DAT0) | SCTL_SMOD(2);
    s1ctl = SCTL_TIMSEL(1) | txpol | SCTL_PINSEL(FX_CMD) | SCTL_SMOD(2);
    SHIFTCFG(0) = SCFG_PWIDTH(0); SHIFTCTL(0) = s0ctl; SHIFTBUF(0) = 0xFFFFFFFFu;
    SHIFTCFG(1) = 0;              SHIFTCTL(1) = s1ctl; SHIFTBUF(1) = 0xFFFFFFFFu;
    SHIFTCFG(2) = 0;
    SHIFTCTL(2) = SCTL_TIMSEL(1) | rxpol | SCTL_PINSEL(FX_CMD) | SCTL_SMOD(1);
    SHIFTCFG(3) = SCFG_PWIDTH(0);
    SHIFTCTL(3) = SCTL_TIMSEL(0) | rxpol | SCTL_PINSEL(FX_DAT0) | SCTL_SMOD(1);
    TIMCTL(0) = t_ctl; TIMCTL(1) = t_ctl;
    FLEXIO2_SHIFTSIEN = 4;                   // S2 CMD RX word interrupt
    FLEXIO2_SHIFTSDEN = K.lbk ? 8u : 0u;
    dtx.triggerAtHardwareEvent(DMAMUX_SOURCE_FLEXIO2_REQUEST0);
    dtx.attachInterrupt(dtx_isr, 64);
    if (K.lbk) { d_fmt4 = 0; ring_start(drx, FLEXIO2_SHIFTBUF3, dring, DRING_WORDS, DMAMUX_SOURCE_FLEXIO2_REQUEST2, drx_isr, drx_laps); }
    drive_ok = S.vdd_ok;
    digitalWriteFast(PIN_OE, drive_ok ? HIGH : LOW);
  }
  FLEXIO2_CTRL = 1;                          // FLEXEN
  NVIC_SET_PRIORITY(IRQ_FLEXIO2, 0);
  NVIC_ENABLE_IRQ(IRQ_CMDBH);
  NVIC_ENABLE_IRQ(IRQ_FLEXIO2);
}

void fx_set_mode(FxMode m) {
  engine_stop();
  S.mode = m;
  S.card_state = ST_IDLE;
  if (m == FX_OFF) { pads_off(); digitalWriteFast(PIN_LED, LOW); return; }
  engine_start();
}
void fx_apply_knobs(void) { fx_set_mode(S.mode); }

void fx_begin(void) {
  sdp_init();
  AVS.init(pool_banks, pool_bank_n, 2, avs_now, avs_tx);
  pinMode(PIN_DBG, OUTPUT); digitalWriteFast(PIN_DBG, LOW);
  pinMode(PIN_OE, OUTPUT); digitalWriteFast(PIN_OE, LOW);
  pinMode(PIN_LED, OUTPUT); digitalWriteFast(PIN_LED, LOW);
  pinMode(PIN_CDSENSE, INPUT);               // no pull: never feed the slot
  pads_off();
  analogReadResolution(12);
  analogReadAveraging(4);
  flexio_clock();
  flexio_reset();
  S.param = FLEXIO2_PARAM;
  dtx.begin(true); crx.begin(true); drx.begin(true);
  attachInterruptVector(IRQ_FLEXIO2, fx_isr);
  NVIC_SET_PRIORITY(IRQ_FLEXIO2, 0);
  attachInterruptVector(IRQ_CMDBH, cmd_bh);
  NVIC_SET_PRIORITY(IRQ_CMDBH, 16);
  attachInterruptVector(IRQ_SOFTWARE, pump);
  NVIC_SET_PRIORITY(IRQ_SOFTWARE, 64);
  NVIC_ENABLE_IRQ(IRQ_SOFTWARE);
  S.mode = FX_OFF;
  fx_clear_counters();
}

void fx_clear_counters(void) {
  memset((void *)&C, 0, sizeof(FxCounters));
  fx_log_dropped = 0;
}

// ---------------------------------------------------------------- VDD / frequency / rings (main loop)
static uint32_t vdd_last_ms, f10_last_us, f10_last_flags, f1_last_us, f1_last_flags;
static uint32_t f_peak, f_min;

static void vdd_poll(void) {
  uint32_t now = millis();
  if (now == vdd_last_ms) return;
  vdd_last_ms = now;
  float v = analogRead(PIN_VSENSE) * (3.3f / 4095.0f) * VDIV_RATIO;
  S.vdd_volts = v;
  uint8_t was = S.vdd_ok;
  if (!K.vddgate) S.vdd_ok = 1;              // bench override: slot treated as powered
  else if (!was && v > VDD_ON_V) S.vdd_ok = 1;
  else if (was && v < VDD_OFF_V) S.vdd_ok = 0;
  if (S.vdd_ok == was) return;
  if (S.vdd_ok) C.vdd_on_events++; else C.vdd_off_events++;
  log_event(S.vdd_ok ? 'V' : 'v', (uint32_t)(v * 1000));
  if (S.mode == FX_CARD) {
    __disable_irq();
    card_reset();                            // a power cycle resets the card
    SHIFTCTL(1) = s1ctl; FLEXIO2_SHIFTSIEN &= ~2u; tx_start = 0;   // CMD released
    drive_ok = S.vdd_ok;
    __enable_irq();
    pads_update_pulls();                     // pulls only while the host powers the slot
    digitalWriteFast(PIN_OE, drive_ok ? HIGH : LOW);
    digitalWriteFast(PIN_LED, drive_ok ? HIGH : LOW);
  }
}

static void freq_poll(void) {
  uint32_t now = micros();
  if (now - f10_last_us >= 10000u) {
    uint32_t fl = freq_flags, d = fl - f10_last_flags;
    uint32_t hz = (uint32_t)((uint64_t)d * FREQ_DIV * 1000000u / (now - f10_last_us));
    if (hz > f_peak) f_peak = hz;
    if (d && (f_min == 0 || hz < f_min)) f_min = hz;
    f10_last_us = now; f10_last_flags = fl;
  }
  if (now - f1_last_us >= 1000000u) {
    uint32_t fl = freq_flags;
    S.clk_hz_avg = (uint32_t)((uint64_t)(fl - f1_last_flags) * FREQ_DIV * 1000000u / (now - f1_last_us));
    S.clk_hz_peak = f_peak; S.clk_hz_min = f_min;
    f_peak = 0; f_min = 0;
    f1_last_us = now; f1_last_flags = fl;
  }
}

struct DatCtx { const uint32_t *ring; uint32_t mask; uint8_t fmt4; };
static uint8_t sym_ring(uint64_t i, void *vctx) {
  const DatCtx *c = (const DatCtx *)vctx;
  if (c->fmt4) return (uint8_t)((c->ring[(uint32_t)(i >> 3) & c->mask] >> ((uint32_t)(i & 7) * 4)) & 0xF);
  return (uint8_t)(((c->ring[(uint32_t)(i >> 5) & c->mask] >> (uint32_t)(i & 31)) & 1) | 0xE);
}

static uint32_t dlog_sec, dlog_cnt;
static void dat_scan(void) {
  DatCtx ctx = {dring, DRING_WORDS - 1, d_fmt4};
  const uint32_t spw = d_fmt4 ? 8 : 32;      // symbols per word
  uint64_t wsym = ring_wpos(drx, drx_laps, dring, DRING_WORDS) * spw;
  if (wsym < d_rpos) return;
  if (wsym - d_rpos > (uint64_t)(DRING_WORDS - 256) * spw) {
    C.ring_overrun_dat++; d_rpos = wsym; d_waiting = 0; return;
  }
  int budget = 64;                           // blocks per call, keeps the loop responsive
  while (d_rpos < wsym) {
    if ((d_rpos % spw) == 0 && d_rpos + spw <= wsym && dring[(uint32_t)(d_rpos / spw) & ctx.mask] == 0xFFFFFFFFu) {
      d_rpos += spw; continue;               // idle word
    }
    if (sym_ring(d_rpos, &ctx) & 1) { d_rpos++; continue; }   // DAT0 high: idle
    int force = d_waiting && d_wait_pos == d_rpos && (millis() - d_wait_ms) > 50;
    DatResult r;
    int k = dat_try_block(sym_ring, &ctx, d_rpos, wsym - d_rpos, force, r);
    if (k == 0) {
      if (!d_waiting || d_wait_pos != d_rpos) { d_waiting = 1; d_wait_pos = d_rpos; d_wait_ms = millis(); }
      break;
    }
    d_waiting = 0;
    if (k < 0) { d_rpos++; continue; }
    if (r.ok) C.dat_ok++; else C.dat_bad++;
    if (r.width == 4) C.dat_w4++; else C.dat_w1++;
    uint32_t sec = millis() / 1000;
    if (sec != dlog_sec) { dlog_sec = sec; dlog_cnt = 0; }
    if (K.log && ((!r.ok && dlog_cnt < 20) || K.log >= 2)) {
      dlog_cnt++;
      LogRec L; memset(&L, 0, sizeof L);
      L.t_us = micros(); L.kind = 'D'; L.crc = r.ok; L.len = r.len; L.width = r.width;
      memcpy(L.b, r.first, 8);
      log_push(L);
    }
    d_rpos += r.len ? dat_block_symbols(r.width, r.len) : (wsym - d_rpos);   // forced, too short: skip it
    if (--budget == 0) break;
  }
}

static void cmd_ring_scan(void) {
  uint64_t w = ring_wpos(crx, crx_laps, cring, CRING_WORDS);
  if (w < c_rpos) return;
  if (w - c_rpos > CRING_WORDS - 64) { C.ring_overrun_cmd++; c_rpos = w; cmdp.reset(); return; }
  int budget = 4096;
  while (c_rpos < w && budget--) { cmdp.feed(cring[(uint32_t)c_rpos & (CRING_WORDS - 1)], 32); c_rpos++; }
}

static uint8_t last_bus4;
void fx_poll(void) {
  vdd_poll();
  AVS.card_state = S.card_state; AVS.bus_width = S.bus4 ? 4 : 1; AVS.mode = (uint8_t)S.mode;
  AVS.crc7_rejects = C.host_crc_bad;
  freq_poll();
  if (S.mode == FX_MON) {
    cmd_ring_scan();
    if (K.dat) dat_scan();
  } else if (S.mode == FX_CARD) {
    if (rd_hold && rd_active) { rd_hold = 0; NVIC_SET_PENDING(IRQ_SOFTWARE); }   // retry a held window sector
    if (FLEXIO2_SHIFTERR & 4) { FLEXIO2_SHIFTERR = 4; C.rx_overrun++; }   // CMD RX word lost (ISR late)
    if (FLEXIO2_SHIFTERR & 1) {                  // S0 loaded with no word ready: DMA late, bad data if inside a block
      FLEXIO2_SHIFTERR = 1;
      if (rd_active && dma_busy) C.dat_underrun++;
    }
    if (K.lbk) {
      if (S.bus4 != last_bus4) { last_bus4 = S.bus4; dat_ring_restart(S.bus4); }   // capture format follows width
      dat_scan();
    }
  }
}

// ---------------------------------------------------------------- diagnostics
uint32_t fx_pins(void) {
  uint32_t v = 0;
  if (S.mode == FX_OFF) {
    for (int i = 0; i < 6; i++) v |= (uint32_t)(digitalReadFast(PINS[i]) ? 1 : 0) << i;
  } else {
    uint32_t p = FLEXIO2_PIN;
    for (int i = 0; i < 6; i++) v |= ((p >> FXPIN[i]) & 1u) << i;
  }
  return v;
}

float fx_duty(uint32_t samples, uint32_t *edges_out) {
  uint32_t hi = 0, f0 = freq_flags;
  if (S.mode == FX_OFF) {
    for (uint32_t i = 0; i < samples; i++) hi += digitalReadFast(PIN_CLK) ? 1 : 0;
  } else {
    for (uint32_t i = 0; i < samples; i++) {
      hi += (FLEXIO2_PIN >> FX_CLK) & 1u;
      for (volatile int d = (int)(i & 7); d > 0; d--) {}      // dither the sample phase
    }
  }
  if (edges_out) *edges_out = (freq_flags - f0) * FREQ_DIV;
  return samples ? (float)hi / (float)samples : 0.0f;
}

static void put_be32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
void fx_info_sector(uint8_t *s) {
  memset(s, 0, 512);
  put_be32(s, 0x52534946u);                  // 'RSIF'
  strncpy((char *)s + 4, FW_VERSION, 44);
  put_be32(s + 48, millis());
  put_be32(s + 52, C.host_cmds); put_be32(s + 56, C.host_crc_bad); put_be32(s + 60, C.sectors_served);
  put_be32(s + 64, C.cmd17); put_be32(s + 68, C.cmd18); put_be32(s + 72, C.cmd12); put_be32(s + 76, C.retries);
  put_be32(s + 80, C.illegal); put_be32(s + 84, C.rx_overrun); put_be32(s + 88, S.clk_hz_avg);
  put_be32(s + 92, S.bus4); put_be32(s + 96, C.dat_ok); put_be32(s + 100, C.dat_bad);
  AVS.info(s + 104, 40);                     // stream: newest/begun/served seq, pool free/size, slots, counters
}

// ---------------------------------------------------------------- live knob update (no engine reset)
// Pads (dse/sre/speed/hys/pull) and the TX launch edge can change while the host is talking, so an
// eye sweep does not force the RNS-E to re-initialise the card. samp/wphase/lbk/dat need fx_apply_knobs().
void fx_update_live(void) {
  if (S.mode == FX_OFF) return;
  if (S.mode == FX_CARD) {
    __disable_irq();
    uint32_t inv = (K.wphase == 1) ? 1 : 0;
    txpol = ((K.launch ? 1u : 0u) ^ inv) ? SCTL_TIMPOL : 0;
    dtxpol = ((K.dlaunch ? 1u : 0u) ^ inv) ? SCTL_TIMPOL : 0;
    s0ctl = (s0ctl & ~SCTL_TIMPOL) | dtxpol;
    s1ctl = (s1ctl & ~SCTL_TIMPOL) | txpol;
    SHIFTCTL(0) = (SHIFTCTL(0) & ~SCTL_TIMPOL) | dtxpol;
    SHIFTCTL(1) = (SHIFTCTL(1) & ~SCTL_TIMPOL) | txpol;
    __enable_irq();
    pads_update_pulls();                     // rewrites the CMD/DAT pad value incl. DSE/SRE/SPEED/HYS
  }
}
