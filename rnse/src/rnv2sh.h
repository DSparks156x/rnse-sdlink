/* rnv2sh.h -- RNV2 video decoder for the RNS-E (SH-4A, big-endian), shared by decbench / sdstream and the host
 * harnesses. Bitstream = video/dec/rnv2dec.c (the bit-exact reference, unchanged format).
 * Differences from the reference decoder (output pixels are identical for valid streams):
 *  - N display buffers (2 or 3). A SKIP block is copied only if it changed within the last N-1 frames (per-block
 *    age counter); with N = 2 this is exactly the reference's "prevmode != SKIP" rule.
 *  - all pixel/bitstream accesses are aligned 32-bit words (LDW/STW); no unaligned access, no division, no libc.
 *  - bounds checks: frame/substream lengths, skip runs, coefficient position, escape size, MV range, RAW stream
 *    length, Exp-Golomb length, Huffman miss. A bad frame returns an error code instead of reading wild memory.
 *  - residual in place on a 32-word temp (cache-resident) then one store pass: P1 is write-through on this unit,
 *    so the frame buffer is written exactly once per coded block and never read back.
 * Compiled three times from rnv2sh.c: rnvo_* (plain), rnvp_* (with PREF of the next reference line pair) and
 * rnvf_* (RNV_FAST: register bit window, sign table, residual added in the second transform pass, PREF of the next reference row;
 * same pixels, about 26% less time on the unit). sdstream uses rnvf; its --pref picks the plain rnvo, for A/B. */
#ifndef RNV2SH_H
#define RNV2SH_H

#include <stdint.h>   /* freestanding header (gcc) */
typedef uint32_t rnv_u32;
typedef int32_t rnv_s32;
typedef uint16_t rnv_u16;
typedef uint8_t rnv_u8;

#define RNV_W 800
#define RNV_H 480
#define RNV_BW 100
#define RNV_BH 60
#define RNV_NB (RNV_BW * RNV_BH)
#define RNV_STRIDE (RNV_W * 2)
#define RNV_FRAME_BYTES (RNV_STRIDE * RNV_H) /* 768000 */

enum { RNV_OK = 0, RNV_E_MAGIC, RNV_E_LEN, RNV_E_RUN, RNV_E_HUFF, RNV_E_MV, RNV_E_COEF, RNV_E_UE, RNV_E_OVERRUN,
       RNV_E_RAW, RNV_E_QP };

typedef struct {
    rnv_u8 *buf[3];
    int nbuf;          /* 2 or 3 */
    int cur;           /* buffer the next frame is decoded into */
    int last;          /* buffer holding the last decoded frame (reference) */
    rnv_u32 frames;    /* frames decoded since init */
    rnv_u32 last_err;  /* last error code */
    rnv_u32 err_blk;   /* block index of the last error */
    rnv_u8 age[RNV_NB];
    rnv_u8 rowm[RNV_BW + 2];   /* RNV3: mode of the block above, per column */
    rnv_u8 topY[RNV_BW][5], topC[2][RNV_BW][3], crn[3][3][RNV_BW + 1];
} rnv_state;

/* tables shared by both variants (Huffman fast tables, clamp, symbol split); call once */
void rnv_tables_init(void);
/* bufs: nbuf frame buffers (768000 B each, 4-byte aligned; 32 bytes readable past the end of each).
 * Fills them black (Y 0, Cb/Cr 128) exactly like the reference rnv_init. */
void rnv_state_reset_keep(rnv_state *s, const rnv_u8 *shown);
void rnv_state_init(rnv_state *s, rnv_u8 *b0, rnv_u8 *b1, rnv_u8 *b2, int nbuf);
/* reset to black (DISCONT) without changing buffers */
void rnv_state_reset(rnv_state *s);
/* decode one RNV2 frame (f = frame bytes incl. 24-byte header, len = byte count). Needs 8 readable bytes past
 * f+len. On success returns 0, s->last = decoded buffer index, s->cur advances. On error returns RNV_E_*,
 * marks every block for re-copy and leaves s->last on the partially decoded buffer (caller decides). */
int rnvo_decode(rnv_state *s, const rnv_u8 *f, rnv_u32 len);
int rnvp_decode(rnv_state *s, const rnv_u8 *f, rnv_u32 len);
int rnvf_decode(rnv_state *s, const rnv_u8 *f, rnv_u32 len);

/* reference decoder (video/dec/rnv2dec.c logic, ping-pong 2 buffers), for A/B timing on the unit */
typedef struct { rnv_u8 *buf[2]; int cur; rnv_u8 prevmode[RNV_NB]; } rnvref_dec;
void rnvref_init(rnvref_dec *d, rnv_u8 *a, rnv_u8 *b);
rnv_u8 *rnvref_decode(rnvref_dec *d, const rnv_u8 *f, rnv_u32 len);

#endif
