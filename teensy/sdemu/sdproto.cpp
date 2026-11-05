// sdproto.cpp -- see sdproto.h. Hardware independent; mirrors ../link/sd_wire.py.
#include "sdproto.h"
#include <string.h>
#if defined(__arm__)
#include <Arduino.h>
#define PROGMEM_ATTR PROGMEM
#else
#define PROGMEM_ATTR
#endif
#include "tonemp3.h"

#define VOL_TOTAL 2097152u
#define VOL_PART_LBA 2048u
#define VOL_PART_SECTORS (VOL_TOTAL - VOL_PART_LBA)   // 2095104
#define VOL_INFO_LBA 0x40000u
#define LBA_BIN (LBA_DATA + VOL_SPC)                   // 4160: first sector of RSAVLINK.BIN

static uint8_t  crc7_tab[256];    // table for (crc7 << 1) register form
static uint16_t crc16_tab[256];
static uint32_t crc32_tab[256];
static uint8_t  rev8[256];

static inline uint32_t rbit32(uint32_t v) {
#if defined(__arm__)
  uint32_t r; __asm__("rbit %0, %1" : "=r"(r) : "r"(v)); return r;
#else
  v = ((v >> 1) & 0x55555555u) | ((v & 0x55555555u) << 1);
  v = ((v >> 2) & 0x33333333u) | ((v & 0x33333333u) << 2);
  v = ((v >> 4) & 0x0F0F0F0Fu) | ((v & 0x0F0F0F0Fu) << 4);
  v = ((v >> 8) & 0x00FF00FFu) | ((v & 0x00FF00FFu) << 8);
  return (v >> 16) | (v << 16);
#endif
}
static inline uint32_t bswap32(uint32_t v) { return __builtin_bswap32(v); }
static inline uint32_t ld_le32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }  // LE CPU
static inline void st_be32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
static inline void st_le32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static inline void st_le16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static inline uint32_t nibswap32(uint32_t w) { return ((w & 0x0F0F0F0Fu) << 4) | ((w >> 4) & 0x0F0F0F0Fu); }

void sdp_init(void) {
  for (int i = 0; i < 256; i++) {
    uint8_t c = (uint8_t)i;
    for (int j = 0; j < 8; j++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x12) : (uint8_t)(c << 1);
    crc7_tab[i] = c;
    uint16_t d = (uint16_t)(i << 8);
    for (int j = 0; j < 8; j++) d = (d & 0x8000) ? (uint16_t)((d << 1) ^ 0x1021) : (uint16_t)(d << 1);
    crc16_tab[i] = d;
    uint32_t z = (uint32_t)i;
    for (int j = 0; j < 8; j++) z = (z & 1) ? (z >> 1) ^ 0xEDB88320u : (z >> 1);
    crc32_tab[i] = z;
    rev8[i] = (uint8_t)(rbit32((uint32_t)i) >> 24);
  }
  // registers (SPEC 2.A.2): R2 = 0x3F + 15 content bytes + (crc7 << 1 | 1)
  R2_CID[0] = 0x3F; memcpy(R2_CID + 1, CID_REF, 15); R2_CID[16] = crc7_end(CID_REF, 15);
  R2_CSD[0] = 0x3F; memcpy(R2_CSD + 1, CSD_REF, 15); R2_CSD[16] = crc7_end(CSD_REF, 15);
  memset(SCR_BYTES, 0, 8); SCR_BYTES[1] = 0x05;                 // SD_SPEC 0, bus widths 1+4
  memset(SDSTAT4, 0, 64); SDSTAT4[0] = 0x80; SDSTAT4[8] = 0x02; SDSTAT4[10] = 0x90;
  memcpy(SDSTAT1, SDSTAT4, 64); SDSTAT1[0] = 0x00;
}

uint8_t crc7(const uint8_t *p, int n) { uint8_t c = 0; while (n--) c = crc7_tab[c ^ *p++]; return c >> 1; }
uint8_t crc7_end(const uint8_t *p, int n) { uint8_t c = 0; while (n--) c = crc7_tab[c ^ *p++]; return c | 1; }
uint16_t crc16(const uint8_t *p, int n) {
  uint16_t c = 0;
  while (n--) c = (uint16_t)((c << 8) ^ crc16_tab[((c >> 8) ^ *p++) & 0xFF]);
  return c;
}
uint32_t crc32z(const uint8_t *p, int n, uint32_t c) {
  c = ~c;
  while (n--) c = (c >> 8) ^ crc32_tab[(c ^ *p++) & 0xFF];
  return ~c;
}

// gather bits k, k+4, ..., k+28 of a wire word (time order, LSB = first) into 8 bits
static inline uint32_t gather4(uint32_t x) {
  x &= 0x11111111u;
  x = (x | (x >> 3)) & 0x03030303u;
  x = (x | (x >> 6)) & 0x000F000Fu;
  x = (x | (x >> 12)) & 0xFFu;
  return x;
}
static inline void crc_lines_word(uint16_t c[4], uint32_t w) {   // w = 4-bit wire word
  for (int k = 0; k < 4; k++) {
    uint8_t b = rev8[gather4(w >> k)];                             // MSB = first in time
    c[k] = (uint16_t)((c[k] << 8) ^ crc16_tab[((c[k] >> 8) ^ b) & 0xFF]);
  }
}
void crc16_lines4(const uint8_t *p, int n, uint16_t out[4]) {
  uint16_t c[4] = {0, 0, 0, 0};
  for (int i = 0; i < n; i += 4) crc_lines_word(c, nibswap32(ld_le32(p + i)));
  for (int k = 0; k < 4; k++) out[k] = c[k];
}

// ---------------------------------------------------------------- responses
uint8_t R2_CID[17], R2_CSD[17], SCR_BYTES[8], SDSTAT4[64], SDSTAT1[64];
const uint8_t CID_REF[16] = {0x5A,0x52,0x53,0x52,0x4E,0x53,0x41,0x56,0x10,0x00,0x00,0x00,0x01,0x01,0xAB,0xED};
const uint8_t CSD_REF[16] = {0x00,0x0E,0x00,0x32,0x5B,0x59,0x83,0xFF,0xFE,0xFB,0xFF,0x80,0x0A,0x40,0x00,0xAF};

void make_r1(uint8_t o[6], uint8_t idx, uint32_t s) {
  o[0] = idx & 0x3F; o[1] = s >> 24; o[2] = s >> 16; o[3] = s >> 8; o[4] = s; o[5] = crc7_end(o, 5);
}
void make_r3(uint8_t o[6], uint32_t ocr) {
  o[0] = 0x3F; o[1] = ocr >> 24; o[2] = ocr >> 16; o[3] = ocr >> 8; o[4] = ocr; o[5] = 0xFF;
}
void make_cmd(uint8_t o[6], uint8_t idx, uint32_t a) {
  o[0] = 0x40 | (idx & 0x3F); o[1] = a >> 24; o[2] = a >> 16; o[3] = a >> 8; o[4] = a; o[5] = crc7_end(o, 5);
}
int pack_resp16(const uint8_t *b, int nb, uint32_t *out) {
  int n = 0;
  for (int i = 0; i < nb; i += 2) {
    uint32_t hi = b[i], lo = (i + 1 < nb) ? b[i + 1] : 0xFF;
    out[n++] = (hi << 24) | (lo << 16) | 0xFFFFu;
  }
  return n;
}

// ---------------------------------------------------------------- wire images
int wire1(const uint8_t *blk, int len, uint32_t *out) {
  int n = 0;
  out[n++] = 0x7FFFFFFFu;                                        // 31 idle ones, start bit
  for (int i = 0; i < len; i += 4) out[n++] = bswap32(rbit32(ld_le32(blk + i)));  // bytes bit-reversed
  out[n++] = (rbit32(crc16(blk, len)) >> 16) | 0xFFFF0000u;      // CRC16 MSB first, end bit, idle
  out[n++] = 0xFFFFFFFFu;                                        // pad (release marker)
  return n;
}
int wire4(const uint8_t *blk, int len, uint32_t *out) {
  int n = 0;
  uint16_t c[4] = {0, 0, 0, 0};
  out[n++] = 0x0FFFFFFFu;                                        // 7 idle nibbles, start nibble 0
  for (int i = 0; i < len; i += 4) {
    uint32_t w = nibswap32(ld_le32(blk + i));                     // high nibble first on the wire
    out[n++] = w;
    crc_lines_word(c, w);
  }
  uint32_t w0 = 0, w1 = 0;
  for (int j = 0; j < 16; j++) {                                 // CRC MSB first, all lines at once
    uint32_t nib = 0;
    for (int k = 0; k < 4; k++) nib |= (uint32_t)((c[k] >> (15 - j)) & 1) << k;
    if (j < 8) w0 |= nib << (4 * j); else w1 |= nib << (4 * (j - 8));
  }
  out[n++] = w0; out[n++] = w1;
  out[n++] = 0xFFFFFFFFu;                                        // end nibble F + idle
  out[n++] = 0xFFFFFFFFu;                                        // pad (release marker)
  return n;
}

// ---------------------------------------------------------------- synthetic volume
const char TEST_TXT[] =
  "RNS-E SD emulator test volume (Teensy 4.1 sdemu bring-up firmware).\r\n"
  "Every data sector from LBA 4160 up (file RSAVLINK.BIN) carries a per-LBA pattern:\r\n"
  "big-endian u32 words: 'RSPT', LBA, ~LBA, 1, then 124 xorshift32 words seeded with\r\n"
  "(LBA * 0x9E3779B9) ^ 0xA5A5A5A5 (x ^= x<<13; x ^= x>>17; x ^= x<<5). LBA 0x40000 = info.\r\n";

void pattern_sector(uint32_t lba, uint8_t *s) {
  st_be32(s + 0, 0x52535054u); st_be32(s + 4, lba); st_be32(s + 8, ~lba); st_be32(s + 12, 1);
  uint32_t x = (lba * 0x9E3779B9u) ^ 0xA5A5A5A5u;
  if (!x) x = 1;
  for (int i = 4; i < 128; i++) { x ^= x << 13; x ^= x >> 17; x ^= x << 5; st_be32(s + 4 * i, x); }
}

static void dirent(uint8_t *e, const char *name11, uint8_t attr, uint16_t clus, uint32_t size) {
  memcpy(e, name11, 11); e[11] = attr;
  const uint16_t t = 12u << 11, d = (uint16_t)(((2026 - 1980) << 9) | (11 << 5) | 2);
  st_le16(e + 14, t); st_le16(e + 16, d); st_le16(e + 18, d); st_le16(e + 22, t); st_le16(e + 24, d);
  st_le16(e + 26, clus); st_le32(e + 28, size);
}

uint8_t vol_mp3 = 0;

void gen_sector(uint32_t lba, uint8_t *s, int pattern, info_fn info) {
  memset(s, 0, 512);
  if (lba == 0) {                                                 // MBR
    st_le32(s + 440, 0x52534156u);
    uint8_t *p = s + 446;
    p[0] = 0x00; p[1] = 0xFE; p[2] = 0xFF; p[3] = 0xFF; p[4] = 0x06; p[5] = 0xFE; p[6] = 0xFF; p[7] = 0xFF;
    st_le32(p + 8, VOL_PART_LBA); st_le32(p + 12, VOL_PART_SECTORS);
    s[510] = 0x55; s[511] = 0xAA;
  } else if (lba == VOL_PART_LBA) {                               // FAT16 VBR
    s[0] = 0xEB; s[1] = 0x3C; s[2] = 0x90; memcpy(s + 3, "RNSAVLNK", 8);
    st_le16(s + 11, 512); s[13] = VOL_SPC; st_le16(s + 14, VOL_RESERVED); s[16] = 2;
    st_le16(s + 17, 512); st_le16(s + 19, 0); s[21] = 0xF8; st_le16(s + 22, VOL_FATSZ);
    st_le16(s + 24, 63); st_le16(s + 26, 255); st_le32(s + 28, VOL_PART_LBA); st_le32(s + 32, VOL_PART_SECTORS);
    s[36] = 0x80; s[38] = 0x29; st_le32(s + 39, 0x20261102u);
    memcpy(s + 43, "RNSAVLINK  ", 11); memcpy(s + 54, "FAT16   ", 8);
    s[62] = 0xF4; s[63] = 0xEB; s[64] = 0xFD;                       // hlt; jmp $-1
    s[510] = 0x55; s[511] = 0xAA;
  } else if (lba >= LBA_FAT1 && lba < LBA_ROOT) {                 // FAT1 / FAT2 (identical)
    uint32_t base = ((lba - LBA_FAT1) % VOL_FATSZ) * 256u;
    for (uint32_t i = 0; i < 256; i++) {
      uint32_t e = base + i, v;
      const uint32_t last = VOL_CLUSTERS + 1;                     // 32705
      if (e == 0) v = 0xFFF8; else if (e == 1 || e == 2) v = 0xFFFF;
      else if (e == MP3_CLUS1 - 1u) v = 0xFFFF;                   // end of RSAVLINK.BIN
      else if (e < last) v = e + 1; else if (e == last) v = 0xFFFF; else v = 0;
      st_le16(s + 2 * i, (uint16_t)v);
    }
  } else if (lba == LBA_ROOT) {                                   // root directory, first sector
    dirent(s + 0, "RNSAVLINK  ", 0x08, 0, 0);
    dirent(s + 32, "TEST    TXT", 0x01, 2, (uint32_t)(sizeof(TEST_TXT) - 1));
    dirent(s + 64, "RSAVLINKBIN", 0x07, 3, (VOL_CLUSTERS - 1 - MP3_CLUSTERS) * VOL_SPC * 512u);
    if (vol_mp3 == 1) dirent(s + 96, "SILENCE MP3", 0x20, MP3_CLUS1, MP3_SIZE);
    if (vol_mp3 == 2) dirent(s + 96, "TONE    MP3", 0x20, MP3_CLUS1, TONE_MP3_SIZE);
  } else if (lba == LBA_DATA) {
    memcpy(s, TEST_TXT, sizeof(TEST_TXT) - 1);
  } else if (lba == VOL_INFO_LBA && info) {
    info(s);
  } else if (lba >= LBA_MP3 && lba < VOL_TOTAL && vol_mp3 == 2) {  // real encoded test tone (flash)
    uint32_t off = (lba - LBA_MP3) * 512u;
    if (off < TONE_MP3_SIZE) memcpy(s, TONE_MP3 + off, TONE_MP3_SIZE - off < 512u ? TONE_MP3_SIZE - off : 512u);
  } else if (lba >= LBA_MP3 && lba < VOL_TOTAL) {                 // silent frames: header, then all zero
    static const uint8_t hdr[4] = {0xFF, 0xFB, 0x94, 0x00};       // MPEG-1 L3, no CRC, 128 kbps, 48 kHz, stereo
    uint32_t off = (lba - LBA_MP3) * 512u;
    for (uint32_t i = 0; i < 512; i++) {
      uint32_t p = off + i;
      if (p < MP3_SIZE && p % MP3_FRAME < 4) s[i] = hdr[p % MP3_FRAME];
    }
  } else if (lba >= LBA_BIN && lba < VOL_TOTAL && pattern) {
    pattern_sector(lba, s);
  }
}

// ---------------------------------------------------------------- CMD parser
void CmdParser::feed(uint32_t w, int n) {
  while (n > 0) {
    if (!busy) {
      uint32_t inv = ~w;
      if (n < 32) inv &= ~((1u << (32 - n)) - 1u);                // only the n valid (top) bits
      if (!inv) return;                                           // all idle ones
      int lead = __builtin_clz(inv);
      w <<= lead; n -= lead;
      busy = 1; got = 0; need = 48; memset(buf, 0, sizeof buf);
    }
    while (n > 0 && got < need) {
      uint32_t bit = w >> 31; w <<= 1; n--;
      if (bit) buf[got >> 3] |= (uint8_t)(0x80u >> (got & 7));
      got++;
      if (got == 2) need = bit ? 48 : (expect_r2 ? 136 : 48);
    }
    if (got >= need) finish();
  }
}

void CmdParser::finish() {
  busy = 0;
  CmdTok t;
  memcpy(t.b, buf, sizeof buf);
  t.nbytes = (uint8_t)(got / 8);
  t.host = (buf[0] & 0x40) ? 1 : 0;
  t.idx = buf[0] & 0x3F;
  t.arg = ((uint32_t)buf[1] << 24) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 8) | buf[4];
  t.app = 0;
  if (t.host) {
    t.app = last55;
    t.crc = (crc7_end(buf, 5) == buf[5]) ? 1 : 0;
    expect_r2 = (!t.app && (t.idx == 2 || t.idx == 9 || t.idx == 10)) ? 1 : 0;
    expect_r3 = ((t.app && t.idx == 41) || (!t.app && t.idx == 1)) ? 1 : 0;
    last55 = (!t.app && t.idx == 55 && t.crc) ? 1 : 0;
  } else {
    if (t.nbytes == 17) t.crc = (crc7_end(buf + 1, 15) == buf[16]) ? 1 : 0;
    else if (expect_r3) t.crc = (buf[0] == 0x3F && buf[5] == 0xFF) ? 2 : 0;
    else t.crc = (crc7_end(buf, 5) == buf[5]) ? 1 : 0;
    expect_r2 = 0; expect_r3 = 0;
  }
  if (on_tok) on_tok(t);
}

// ---------------------------------------------------------------- DAT decoder
uint32_t dat_block_symbols(int width, int len) {
  return width == 4 ? (uint32_t)(1 + 2 * len + 16 + 1) : (uint32_t)(1 + 8 * len + 16 + 1);
}

static int decode_one(sym_fn get, void *ctx, uint64_t start, int width, int len, DatResult &r) {
  uint8_t d[512];
  uint64_t s = start + 1;
  int ok;
  if (width == 4) {
    for (int i = 0; i < len; i++) d[i] = (uint8_t)((get(s + 2 * i, ctx) << 4) | get(s + 2 * i + 1, ctx));
    uint16_t got[4] = {0, 0, 0, 0}, exp[4];
    for (int j = 0; j < 16; j++) {
      uint8_t nib = get(s + 2 * len + j, ctx);
      for (int k = 0; k < 4; k++) got[k] |= (uint16_t)(((nib >> k) & 1) << (15 - j));
    }
    crc16_lines4(d, len, exp);
    ok = got[0] == exp[0] && got[1] == exp[1] && got[2] == exp[2] && got[3] == exp[3];
    r.end_ok = get(s + 2 * len + 16, ctx) == 0xF;
  } else {
    for (int i = 0; i < len; i++) {
      uint8_t b = 0;
      for (int j = 0; j < 8; j++) b = (uint8_t)((b << 1) | (get(s + 8 * i + j, ctx) & 1));
      d[i] = b;
    }
    uint16_t got = 0;
    for (int j = 0; j < 16; j++) got = (uint16_t)((got << 1) | (get(s + 8 * len + j, ctx) & 1));
    ok = got == crc16(d, len);
    r.end_ok = get(s + 8 * len + 16, ctx) & 1;
  }
  r.width = (int8_t)width; r.len = (int16_t)len; r.ok = (uint8_t)(ok && r.end_ok);
  memcpy(r.first, d, 8);
  return r.ok;
}

int dat_try_block(sym_fn get, void *ctx, uint64_t start, uint64_t avail, int force, DatResult &r) {
  uint8_t s0 = get(start, ctx);
  int width;
  if (s0 == 0) width = 4;
  else if ((s0 & 1) == 0) width = 1;
  else return -1;
  static const int lens[3] = {8, 64, 512};
  int checked_all = 1, last = 0;
  for (int i = 0; i < 3; i++) {
    if (avail < dat_block_symbols(width, lens[i])) { checked_all = 0; continue; }
    last = lens[i];
    if (decode_one(get, ctx, start, width, lens[i], r)) return 1;
  }
  if (!checked_all && !force) return 0;
  if (!last) { r.width = (int8_t)width; r.len = 0; r.ok = 0; r.end_ok = 0; memset(r.first, 0, 8); }
  else decode_one(get, ctx, start, width, last, r);
  r.ok = 0;
  return 1;
}
