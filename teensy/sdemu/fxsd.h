// fxsd.h -- FlexIO2 SD bus engine: passive monitor (S1) and card emulation (S2 1-bit / S3 4-bit).
#pragma once
#include <stdint.h>
#include "sdproto.h"

enum FxMode { FX_OFF = 0, FX_MON = 1, FX_CARD = 2 };

struct FxCounters {
  // command line
  uint32_t host_cmds, host_crc_bad, resp_seen, resp_crc_bad;
  uint32_t responded, unanswered, illegal, suppressed_novdd;
  uint32_t cmd0, cmd8, acmd41, cmd2, cmd3, cmd9, cmd7, cmd13, cmd16, acmd6, acmd13, acmd51;
  uint32_t cmd17, cmd18, cmd12, writes_refused, retries, other;
  uint32_t rx_overrun, dat_underrun, tx_underrun_resp;
  // data line
  uint32_t sectors_served, small_blocks, rd_restarts;
  uint32_t dat_ok, dat_bad, dat_w1, dat_w4, ring_overrun_cmd, ring_overrun_dat;
  // power
  uint32_t vdd_on_events, vdd_off_events;
};
extern volatile FxCounters C;

struct FxState {
  FxMode mode;
  uint8_t vdd_ok;          // slot VDD present (debounced, hysteresis)
  float vdd_volts;
  uint8_t card_state;      // ST_*
  uint8_t bus4;            // card mode: 4-bit selected by ACMD6
  uint32_t clk_hz_avg;     // SDCLK, last 1 s average
  uint32_t clk_hz_peak;    // max over 10 ms windows in the last 1 s
  uint32_t clk_hz_min;     // min nonzero 10 ms window in the last 1 s
  uint32_t last_lba;
  uint32_t param;          // FLEXIO2_PARAM (expect 0x02200808)
};
extern FxState S;

void fx_begin(void);                 // once from setup()
void fx_set_mode(FxMode m);
void fx_apply_knobs(void);           // re-apply pad/phase knobs (resets the engine in the current mode)
void fx_update_live(void);           // pads + TX launch edge, no reset (eye sweeps)
void fx_poll(void);                  // main loop: VDD, frequency, rings, log
void fx_clear_counters(void);
uint32_t fx_pins(void);              // bit0 CLK, bit1 CMD, bit2..5 DAT0..3 (current pad input levels)
float fx_duty(uint32_t samples, uint32_t *edges_out);   // SDCLK high fraction by random sampling
void fx_info_sector(uint8_t *s);     // contents of LBA 0x40000
void fx_trace_arm(void);              // word trace around the next CMD7
int fx_trace_get(uint32_t i, uint32_t *cyc, uint16_t *w, uint8_t *ss, uint8_t *txi);   // -1 not frozen, 0 end, 1 ok


// log records (printed by the main sketch)
struct LogRec {
  uint32_t t_us;
  uint8_t kind;        // 'C' host cmd, 'R' card response (seen), 'D' data block, 'E' event
  uint8_t idx, app, crc, responded, nbytes;
  uint32_t arg;
  uint8_t b[17];
  int16_t len; int8_t width;
};
int fx_cmdlog_get(uint32_t i, uint32_t *t, uint32_t *arg, uint8_t *idx, uint8_t *st, uint8_t *q, uint8_t *flg);
void fx_read_state(char *o, int n);
int fx_log_pop(LogRec &r);           // 1 if a record was returned
extern volatile uint32_t fx_log_dropped;
