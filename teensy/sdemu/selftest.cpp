// selftest.cpp -- see selftest.h. No hardware access; compiles for the Teensy and natively on a PC.
#include "selftest.h"
#include "sdproto.h"
#include "config.h"
#include "avstream.h"
#include <stdio.h>
#include <string.h>

static void hexs(char *o, const uint8_t *b, int n) {
  static const char H[] = "0123456789ABCDEF";
  for (int i = 0; i < n; i++) { o[2 * i] = H[b[i] >> 4]; o[2 * i + 1] = H[b[i] & 15]; }
  o[2 * n] = 0;
}
static void hexw(char *o, const uint32_t *w, int n) {
  for (int i = 0; i < n; i++) snprintf(o + 8 * i, 9, "%08lX", (unsigned long)w[i]);
  if (n == 0) o[0] = 0;
}
static char vbuf[WIRE_MAX_WORDS * 8 + 8];
static void eb(st_emit_fn e, const char *k, const uint8_t *b, int n) { hexs(vbuf, b, n); e(k, vbuf); }
static void ew(st_emit_fn e, const char *k, const uint32_t *w, int n) { hexw(vbuf, w, n); e(k, vbuf); }

// ---------------------------------------------------------------- parser / decoder tests
static CmdTok st_toks[8]; static int st_ntok;
static void st_tok(const CmdTok &t) { if (st_ntok < 8) st_toks[st_ntok++] = t; }
struct BitBuf { uint32_t w[64]; int nbits; };
static void bb_put(BitBuf &b, uint32_t bit) {
  uint32_t m = 0x80000000u >> (b.nbits & 31);
  if (bit) b.w[b.nbits >> 5] |= m; else b.w[b.nbits >> 5] &= ~m;
  b.nbits++;
}
static void bb_bytes(BitBuf &b, const uint8_t *p, int n) {
  for (int i = 0; i < n; i++) for (int j = 7; j >= 0; j--) bb_put(b, (p[i] >> j) & 1);
}
static void bb_ones(BitBuf &b, int n) { while (n--) bb_put(b, 1); }
struct ImgCtx { const uint32_t *w; uint8_t fmt4; };
static uint8_t sym_img(uint64_t i, void *v) {
  const ImgCtx *c = (const ImgCtx *)v;
  if (c->fmt4) return (uint8_t)((c->w[i >> 3] >> ((i & 7) * 4)) & 0xF);
  return (uint8_t)(((c->w[i >> 5] >> (i & 31)) & 1) | 0xE);
}

static int test_parsers(char *msg, int ml) {
  BitBuf b; memset(&b, 0xFF, sizeof b); b.nbits = 0;
  uint8_t t[17];
  bb_ones(b, 5);  make_cmd(t, 17, 0x12345678u); bb_bytes(b, t, 6);
  bb_ones(b, 3);  make_r1(t, 17, 0x900); bb_bytes(b, t, 6);
  bb_ones(b, 11); make_cmd(t, 9, CARD_RCA << 16); bb_bytes(b, t, 6);
  bb_ones(b, 2);  bb_bytes(b, R2_CSD, 17);
  bb_ones(b, 9);  make_cmd(t, 55, 0); bb_bytes(b, t, 6);
  bb_ones(b, 4);  make_r1(t, 55, 0x120); bb_bytes(b, t, 6);
  bb_ones(b, 8);  make_cmd(t, 41, 0x00FF0000u); bb_bytes(b, t, 6);
  bb_ones(b, 2);  make_r3(t, 0x80FF8000u); bb_bytes(b, t, 6);
  bb_ones(b, 40);
  int total = b.nbits;
  for (int pass = 0; pass < 2; pass++) {     // 16-bit words (card ISR) and 32-bit words (monitor ring)
    CmdParser p; p.reset(); p.on_tok = st_tok; st_ntok = 0;
    int step = pass ? 32 : 16;
    for (int i = 0; i < total; i += step) {
      uint32_t w = b.w[i >> 5];
      if (step == 16) w = (i & 16) ? (w << 16) : (w & 0xFFFF0000u);
      p.feed(w, step);
    }
    int ok = st_ntok == 8 &&
      st_toks[0].host && st_toks[0].idx == 17 && st_toks[0].arg == 0x12345678u && st_toks[0].crc == 1 &&
      !st_toks[1].host && st_toks[1].idx == 17 && st_toks[1].arg == 0x900 && st_toks[1].crc == 1 &&
      st_toks[2].host && st_toks[2].idx == 9 && st_toks[2].crc == 1 &&
      !st_toks[3].host && st_toks[3].nbytes == 17 && st_toks[3].crc == 1 &&
      st_toks[4].host && st_toks[4].idx == 55 && !st_toks[4].app &&
      !st_toks[5].host && st_toks[5].crc == 1 &&
      st_toks[6].host && st_toks[6].idx == 41 && st_toks[6].app && st_toks[6].arg == 0x00FF0000u &&
      !st_toks[7].host && st_toks[7].crc == 2 && st_toks[7].arg == 0x80FF8000u;
    if (!ok) { snprintf(msg, ml, "FAIL cmd parser (word=%d bits, tokens=%d)", step, st_ntok); return 0; }
  }
  // a corrupted command must be reported with crc=0
  {
    BitBuf c; memset(&c, 0xFF, sizeof c); c.nbits = 0;
    bb_ones(c, 7); make_cmd(t, 13, CARD_RCA << 16); t[3] ^= 0x04; bb_bytes(c, t, 6); bb_ones(c, 64);
    CmdParser p; p.reset(); p.on_tok = st_tok; st_ntok = 0;
    for (int i = 0; i < c.nbits; i += 32) p.feed(c.w[i >> 5], 32);
    if (st_ntok != 1 || st_toks[0].crc != 0) { snprintf(msg, ml, "FAIL cmd parser crc reject"); return 0; }
  }
  static uint32_t img[WIRE_MAX_WORDS];
  uint8_t sec[512];
  pattern_sector(0x40100, sec);
  struct Case { int fmt4; const uint8_t *d; int len; uint64_t start; };
  const Case cases[4] = {{1, sec, 512, 7}, {0, sec, 512, 31}, {1, SCR_BYTES, 8, 7}, {0, SDSTAT1, 64, 31}};
  for (int c = 0; c < 4; c++) {
    int n = cases[c].fmt4 ? wire4(cases[c].d, cases[c].len, img) : wire1(cases[c].d, cases[c].len, img);
    ImgCtx ic = {img, (uint8_t)cases[c].fmt4};
    uint64_t avail = (uint64_t)n * (cases[c].fmt4 ? 8 : 32) - cases[c].start;
    DatResult r;
    int k = dat_try_block(sym_img, &ic, cases[c].start, avail, 1, r);
    if (k != 1 || !r.ok || r.len != cases[c].len || r.width != (cases[c].fmt4 ? 4 : 1) ||
        memcmp(r.first, cases[c].d, 8) != 0) {
      snprintf(msg, ml, "FAIL dat decoder case %d (k=%d ok=%d len=%d w=%d)", c, k, r.ok, r.len, r.width);
      return 0;
    }
    img[3] ^= 0x10;                           // corrupt one symbol: must fail now
    k = dat_try_block(sym_img, &ic, cases[c].start, avail, 1, r);
    if (k != 1 || r.ok) { snprintf(msg, ml, "FAIL dat decoder accepted corrupt case %d", c); return 0; }
  }
  snprintf(msg, ml, "PASS cmd parser 16/32-bit, dat decoder 1/4-bit");
  return 1;
}

// ---------------------------------------------------------------- main self-test
int sdp_selftest(st_emit_fn e) {
  uint8_t r[17]; int fails = 0;
  make_cmd(r, 0, 0);       eb(e, "cmd0", r, 6);      fails += r[5] != 0x95;
  make_cmd(r, 17, 0);      eb(e, "cmd17", r, 6);     fails += r[5] != 0x55;
  make_cmd(r, 8, 0x1AA);   eb(e, "cmd8", r, 6);      fails += r[5] != 0x87;
  make_r1(r, 17, 0x900);   eb(e, "r1_17_900", r, 6); fails += r[5] != 0x67;
  static uint8_t ff[512]; memset(ff, 0xFF, 512);
  uint32_t c16 = crc16(ff, 512);
  ew(e, "crc16_ff512", &c16, 1);                      fails += c16 != 0x7FA1;
  fails += R2_CID[16] != CID_REF[15];
  fails += R2_CSD[16] != CSD_REF[15];
  const uint32_t RCA = CARD_RCA;
  make_r1(r, 55, 0x120);  eb(e, "resp_cmd55_idle", r, 6);
  make_r3(r, 0x00FF8000); eb(e, "resp_acmd41_busy", r, 6);
  make_r3(r, 0x80FF8000); eb(e, "resp_acmd41_ready", r, 6);
  eb(e, "resp_cmd2", R2_CID, 17);
  make_r1(r, 3, (RCA << 16) | 0x500); eb(e, "resp_cmd3", r, 6);
  eb(e, "resp_cmd9", R2_CSD, 17);
  make_r1(r, 7, 0x700);   eb(e, "resp_cmd7", r, 6);
  make_r1(r, 16, 0x900);  eb(e, "resp_cmd16", r, 6);
  make_r1(r, 55, 0x920);  eb(e, "resp_cmd55_tran", r, 6);
  make_r1(r, 6, 0x920);   eb(e, "resp_acmd6", r, 6);
  make_r1(r, 13, 0x920);  eb(e, "resp_acmd13", r, 6);
  make_r1(r, 51, 0x920);  eb(e, "resp_acmd51", r, 6);
  make_r1(r, 13, 0x900);  eb(e, "resp_cmd13", r, 6);
  make_r1(r, 17, 0x900);  eb(e, "resp_cmd17", r, 6);
  make_r1(r, 18, 0x900);  eb(e, "resp_cmd18", r, 6);
  make_r1(r, 12, 0xB00);  eb(e, "resp_cmd12_data", r, 6);
  make_r1(r, 12, 0xD00);  eb(e, "resp_cmd12_rcv", r, 6);
  make_r1(r, 24, 0x900);  eb(e, "resp_cmd24", r, 6);
  make_r1(r, 25, 0x900);  eb(e, "resp_cmd25", r, 6);
  eb(e, "scr", SCR_BYTES, 8);
  eb(e, "sdstat4", SDSTAT4, 64);
  eb(e, "sdstat1", SDSTAT1, 64);
  uint16_t l4[4]; crc16_lines4(SCR_BYTES, 8, l4);
  uint32_t l4w[4] = {l4[0], l4[1], l4[2], l4[3]};
  ew(e, "crc16lines_scr", l4w, 4);
  static uint32_t w[WIRE_MAX_WORDS];
  static uint8_t blk[512]; for (int i = 0; i < 512; i++) blk[i] = (uint8_t)i;
  int n = wire1(blk, 512, w);       ew(e, "wire1_blk", w, n - 1);   // without the trailing pad word
  n = wire4(blk, 512, w);           ew(e, "wire4_blk", w, n - 1);
  n = wire4(SCR_BYTES, 8, w);       ew(e, "wire4_scr", w, n - 1);
  n = wire1(SCR_BYTES, 8, w);       ew(e, "wire1_scr", w, n - 1);
  static const uint32_t lbas[] = {0, 2048, 2049, LBA_FAT1, LBA_FAT1 + 1, LBA_FAT1 + 127, LBA_FAT2, LBA_FAT2 + 127,
                                  LBA_ROOT, LBA_ROOT + 1, LBA_DATA, LBA_DATA + 1, LBA_DATA + 64, 0x40100, 0x7FFFF,
                                  2097151};
  static uint8_t sec[512];
  for (unsigned i = 0; i < sizeof lbas / sizeof lbas[0]; i++) {
    gen_sector(lbas[i], sec, 1, nullptr);
    char key[24]; snprintf(key, sizeof key, "sec_%lu", (unsigned long)lbas[i]);
    uint32_t c = crc32z(sec, 512);
    ew(e, key, &c, 1);
  }
  char msg[96];
  int pok = test_parsers(msg, sizeof msg);
  e("parsers", msg);
  fails += !pok;
  char amsg[160];
  int af = avs_selftest(amsg, sizeof amsg);         // live stream store: pool, slots, holds, synthetic headers
  e("avstream", amsg);
  fails += af;
  e("internal", fails ? "FAIL" : "PASS");
  return fails;
}
