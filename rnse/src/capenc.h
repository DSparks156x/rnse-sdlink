/* capenc.h -- RNS-E screen-capture encoder for uplink channel 2 (link/SPEC.md 4.1). See CAPTURE.txt.
 *
 * Model: the encoder keeps a SHADOW copy of each captured DU plane = exactly the picture the Pi has reconstructed.
 * Per call it finds 64x16 tiles where the live (scanned-out) buffer differs from the shadow, encodes them losslessly
 * (SOLID / RAW / OPS = run ops against the previous pixel, the row above, and the Pi's old pixel), hands messages to
 * uplink_put() and copies the sent tiles into the shadow. Tiles that do not fit the byte budget stay different and
 * go out on a later call, so the Pi always converges to the exact picture. Lost messages -> the Pi asks for a
 * refresh -> capenc_refresh(): new epoch, every tile is resent intra (no reference to old pixels).
 *
 * Wire format (all multi-byte fields big-endian; pixel bytes in DU memory order). An uplink message is a
 * concatenation of records; every record starts with {u8 type, u8 layer, u16 rec_len (incl. this header)}.
 *   1 HDR   +4 u32 epoch, +8 u16 w, +10 u16 h, +12 u8 fmt, +13 u8 bpp, +14 u8 tw, +15 u8 th, +16 s16 x, +18 s16 y,
 *           +20 u8 z, +21 u8 flags (bit0 colour key valid), +22 u16 0, +24 u32 key, +28 u32 du_mr      (32 B)
 *   2 PAL   +4 u32 epoch, +8 u16 n, +10 u16 0, +12 n x u32 0x00RRGGBB (DU CLUT words)
 *   3 TILES +4 u32 epoch, +8 u32 frame_id, +12 u16 w, +14 u16 h, +16 u8 fmt, +17 u8 bpp, +18 u8 tw, +19 u8 th,
 *           +20 u16 ntiles, +22 u16 0, then ntiles x { u16 tile_index, u8 mode, data }:
 *             mode 0 SOLID: bpp bytes          mode 1 RAW: tw'*th' pixels (tw',th' = clipped tile size)
 *             mode 2 OPS:  u16 len, len op bytes. Op byte = kind<<6 | (count-1), count 1..64, raster order in
 *               the tile, prev pixel starts at 0:  0 REP (prev pixel) 1 UP (pixel tw' earlier in this tile)
 *               2 LIT (count pixels follow) 3 OLD (Pi's current pixel at that position; never in intra tiles)
 *   4 END   +4 u32 epoch, +8 u32 frame_id, +12 u32 pending (dirty tiles not sent yet), +16 u32 dirty found,
 *           +20 u32 flags (bit0 crc valid, bit1 scan subsampled), +24 u32 CRC-32 (zlib) of the shadow
 *           (rows of w*bpp bytes, top to bottom) -- valid when bit0 set, sent only with pending == 0   (28 B)
 */
#ifndef CAPENC_H
#define CAPENC_H
#include "rnse.h"

#define CAP_TW          64
#define CAP_TH          16
#define CAP_MAX_TILES   1024              /* 800x480 = 13 x 30 = 390 */
#define CAP_MSG_MAX     7936              /* 16 RSUP sectors x 496 B payload */
#define CAP_CHAN        2

#define CAP_R_HDR       1
#define CAP_R_PAL       2
#define CAP_R_TILES     3
#define CAP_R_END       4
#define CAP_M_SOLID     0
#define CAP_M_RAW       1
#define CAP_M_OPS       2

/* fmt codes (same numbers as DU MR DDDF where possible) */
#define CAP_F_PAL8      0
#define CAP_F_RGB565    1
#define CAP_F_ARGB1555  2
#define CAP_F_YUY2      3
#define CAP_F_XRGB8888  4

/* transport interface (owned by the transport agent; uplink_stub.c is the sim/reference version).
 * uplink_put: the transport fragments the message into RSUP sectors (SPEC 4.1: first/last flags, message seq,
 * payload CRC-32). Returns 0 if accepted, < 0 if not (nothing of it was sent). uplink_space: payload bytes it would
 * accept right now on that channel. */
int uplink_put(u32 chan, const void *msg, u32 len);
u32 uplink_space(u32 chan);

typedef struct {
    u32 w, h, bpp, fmt, layer;  /* plane geometry: bpp 1, 2 or 4 */
    u32 live_stride;            /* bytes per line of the live (DU) buffer, = MWR * bpp */
    u8 *shadow;                 /* >= h * ((w*bpp+3)&~3) bytes, private RAM, cached */
    s32 x, y; u32 z, key, key_valid, du_mr;   /* HDR info only (Pi compositing) */
    const u32 *pal;             /* PAL8: CLUT source (e.g. the DU palette shadow), 256 words; else 0 */
    u32 live_prep;              /* 1 = live buffer is read through a cached alias: ocbp each line before reading
                                   (write-back + invalidate; never discards data). 0 = uncached alias (P2) */
    u32 scan_div;               /* 1 = exact detection every call; 2/4/8 = check every Nth row per call */
    u32 crc_every;              /* 0 = never; N = CRC of the shadow in END every N converged calls */
} capenc_cfg;

typedef struct {
    u32 calls, dirty, sent, pending, bytes, msgs, putfail, solid, raw, ops, crcs;
} capenc_stats;

typedef struct {
    capenc_cfg c;
    u32 sstride, ntx, nty, ntiles;
    u32 epoch, frame_id, cursor, scan_phase, crc_ctr, crc_dirty, hdr_pending, pal_pending, refused;
    u32 mlen, mtiles, mhas_hdr, mhas_pal, mtiles_rec;   /* message under construction */
    capenc_stats st;
    u32 last_dirty, last_sent, last_pending, last_bytes;
    u16 mlist[CAP_MSG_MAX / 4];
    u8 tflag[CAP_MAX_TILES];    /* bit0 intra required (forced dirty), bit1 dirty in this call */
    u32 pal[256];
    u32 tmp[CAP_TW * CAP_TH];   /* snapshot of one live tile (u32 units so any bpp fits aligned) */
    u8 msg[CAP_MSG_MAX];
} capenc;

int capenc_init(capenc *e, const capenc_cfg *cfg);         /* 0 ok, -1 bad geometry. Starts epoch 1, all intra */
void capenc_refresh(capenc *e);                            /* Pi lost something: new epoch, resend all intra */
/* One capture step on the live buffer (P1/P0 address of what the DU scans out). budget = payload bytes this call
 * may hand to uplink_put (also capped by uplink_space). Returns payload bytes accepted by the transport. */
u32 capenc_frame(capenc *e, const u8 *live, u32 budget);
u32 cap_crc32(u32 crc, const u8 *p, u32 n);                 /* zlib CRC-32; crc starts at 0 */
#endif
