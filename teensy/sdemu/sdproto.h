// sdproto.h -- SD protocol pieces with no hardware dependency (also compiles on a PC for tests):
// CRC7/CRC16/CRC32, responses, CID/CSD/SCR/SD status, wire images, synthetic FAT16 volume,
// CMD-line bit parser and DAT block decoder. Must match ../link/sd_wire.py bit for bit.
#pragma once
#include <stdint.h>
#include <stddef.h>

// ---------------------------------------------------------------- CRC
void     sdp_init(void);                          // builds tables; call once
uint8_t  crc7(const uint8_t *p, int n);           // 7-bit value
uint8_t  crc7_end(const uint8_t *p, int n);       // (crc7 << 1) | 1  (last byte of a token)
uint16_t crc16(const uint8_t *p, int n);          // SD CRC16-CCITT, MSB first, init 0
void     crc16_lines4(const uint8_t *p, int n, uint16_t out[4]);  // per-DAT-line CRC16 (4-bit mode)
uint32_t crc32z(const uint8_t *p, int n, uint32_t c = 0);         // zlib CRC-32

// ---------------------------------------------------------------- card status / responses
enum { ST_IDLE, ST_READY, ST_IDENT, ST_STBY, ST_TRAN, ST_DATA, ST_RCV, ST_PRG };
#define R1_READY_FOR_DATA (1u << 8)
#define R1_APP_CMD        (1u << 5)
#define R1_ILLEGAL_CMD    (1u << 22)
#define R1_COM_CRC_ERR    (1u << 23)
#define R1_BLOCK_LEN_ERR  (1u << 29)
#define R1_OUT_OF_RANGE   (1u << 31)

void make_r1(uint8_t out[6], uint8_t idx, uint32_t status);   // also used for R6 (idx 3)
void make_r3(uint8_t out[6], uint32_t ocr);
void make_cmd(uint8_t out[6], uint8_t idx, uint32_t arg);      // host token (for tests)
extern uint8_t R2_CID[17], R2_CSD[17];                          // full 136-bit responses
extern uint8_t SCR_BYTES[8], SDSTAT4[64], SDSTAT1[64];
extern const uint8_t CID_REF[16], CSD_REF[16];                  // SPEC 2.A.2 constants (for selftest)

// pack response bytes into 16-bit CMD TX words (value for SHIFTBUFBIS: bits [31:16] MSB first)
int pack_resp16(const uint8_t *b, int nbytes, uint32_t *out);

// ---------------------------------------------------------------- wire images (SPEC 1.5)
// out gets the image words + 1 trailing all-ones pad word; returns total word count.
int wire1(const uint8_t *blk, int len, uint32_t *out);   // 1-bit: len/4 + 3 words
int wire4(const uint8_t *blk, int len, uint32_t *out);   // 4-bit: len/4 + 5 words
#define WIRE_MAX_WORDS (512 / 4 + 5)

// ---------------------------------------------------------------- synthetic volume (SPEC 2.A.3)
// Layout (device LBAs): 0 MBR | 2048 VBR | 3808 FAT1 | 3936 FAT2 | 4064 root (32) | 4096 data.
// Cluster = 64 sectors. TEST.TXT = cluster 2 (LBA 4096). RSAVLINK.BIN = clusters 3..32705
// (LBA 4160..2097151). Data sectors carry pattern_sector(lba) when pattern=1.
#define VOL_RESERVED 1760u
#define VOL_FATSZ    128u
#define VOL_SPC      64u
#define LBA_FAT1     (2048u + VOL_RESERVED)
#define LBA_FAT2     (LBA_FAT1 + VOL_FATSZ)
#define LBA_ROOT     (LBA_FAT2 + VOL_FATSZ)
#define LBA_DATA     (LBA_ROOT + 32u)
#define VOL_CLUSTERS 32704u
// SILENCE.MP3 = the last 32 clusters (1 MiB): valid silent MPEG-1 Layer III frames, 128 kbps 48 kHz, 65.5 s.
// RSAVLINK.BIN stops one cluster before it.
#define MP3_CLUSTERS 32u
#define MP3_CLUS1    (VOL_CLUSTERS + 2u - MP3_CLUSTERS)      // 32674
#define LBA_MP3      (LBA_DATA + (MP3_CLUS1 - 2u) * VOL_SPC) // 2095104
#define MP3_FRAME    384u                                    // 144 * 128000 / 48000, no padding
#define MP3_SIZE     ((MP3_CLUSTERS * VOL_SPC * 512u / MP3_FRAME) * MP3_FRAME)  // 2730 whole frames
typedef void (*info_fn)(uint8_t *sec);
extern uint8_t vol_mp3;                               // 0 = no MP3, 1 = SILENCE.MP3 (firmware may reject it), 2 = TONE.MP3 (real, L 1 kHz + R 1.5 kHz)
void gen_sector(uint32_t lba, uint8_t *sec, int pattern, info_fn info);
void pattern_sector(uint32_t lba, uint8_t *sec);   // 'RSPT' | lba | ~lba | 1 | xorshift32 words, BE
extern const char TEST_TXT[];

// ---------------------------------------------------------------- CMD-line bit parser
// Feeds a bit stream (MSB-aligned words, bit 31 = oldest). Finds host commands (48 bit) and card
// responses (48 or 136 bit, length from the previous command). Calls on_tok for each token.
struct CmdTok {
  uint8_t host;      // 1 = host command, 0 = card response
  uint8_t app;       // host: previous command was CMD55
  uint8_t nbytes;    // 6 or 17
  uint8_t crc;       // 1 ok, 0 bad, 2 not applicable (R3)
  uint8_t idx;       // command index (host) or response index field
  uint32_t arg;      // argument / status / OCR
  uint8_t b[17];
};
struct CmdParser {
  uint8_t busy, nb_need, expect_r2, expect_r3, last55;
  uint16_t got, need;
  uint8_t buf[17];
  void (*on_tok)(const CmdTok &t);
  void reset() { busy = 0; got = 0; expect_r2 = 0; expect_r3 = 0; last55 = 0; }
  void feed(uint32_t w, int n);
 private:
  void finish();
};

// ---------------------------------------------------------------- DAT block decoder
// Symbols: 4-bit capture (nibble per SDCLK, bit k = DAT k) or 1-bit capture (DAT0 bit per SDCLK).
// get(i) returns the symbol as a nibble (1-bit capture: bit0 = DAT0, upper bits forced to 1).
struct DatResult { int8_t width; int16_t len; uint8_t ok; uint8_t end_ok; uint8_t first[8]; };
typedef uint8_t (*sym_fn)(uint64_t i, void *ctx);
// start = index of the start symbol. avail = symbols available from start. Tries len 8, 64, 512.
// returns: 1 decoded (res filled), 0 need more data, -1 not a block start.
int dat_try_block(sym_fn get, void *ctx, uint64_t start, uint64_t avail, int force, DatResult &res);
uint32_t dat_block_symbols(int width, int len);   // symbols from start bit through end bit
