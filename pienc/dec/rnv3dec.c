/* RNV2 reference decoder (portable C, big- or little-endian).  Derived from rnvdec.c (RNV1, unchanged).
 * RNV2 differences: header magic 'R2'; QSTEP2 (q0 = 1); cbp has 8 bits (Y0..Y3, Cb top, Cb bottom, Cr top, Cr bottom)
 * coded as luma nibble + chroma nibble (two 16-symbol canonical Huffman codes); chroma residual is full 4:2:2
 * (each chroma 4x4 covers 4 rows of the block instead of 8 replicated rows).  Retrained coefficient table.
 * Decodes directly into packed YUY2 display buffers (bytes Y0 Cb Y1 Cr), ping-pong A/B.
 * Bit-exact with rnv.py.  Build host:  gcc -O2 -DRNV_MAIN rnv2dec.c -o rnv2dec
 *                          Build SH:    sh-elf-gcc -O2 -m4a -mb -c rnv2dec.c        (code size / inner-loop listing)
 * Usage (host): rnv2dec file.rnv [crc_out.txt] [frames_out.yuy2] [modes_out.u8]
 */
#include <stdint.h>
#include <string.h>

#define W 800
#define H 480
#define BW 100
#define BH 60
#define NB (BW * BH)
#define STRIDE (W * 2) /* bytes per YUY2 row */

#include "rnv2_tables.h"
#include "../src/rnv3_modes.h"

enum { SKIP = 0, MC, MCR, ZRES, INTRA, RAW };
#define RNV3_QC2 2
#define RUNMAX 16
#define LVMAX 13
#define NSYM (2 * RUNMAX * LVMAX)

static const uint8_t ZZ[16] = {0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15};


/* ---------------------------------------------------------------- statistics (for the cycle model) */
typedef struct {
    long blk[M_COUNT], skipcopy, mc_int, mc_odd, mc_h, mc_v, mc_hv, sb_dc, sb_ac, coefs, esc, bits, sb_dc_c, sb_ac_c, iwht_cyc;
} rnv_stats;
rnv_stats ST;

/* ---------------------------------------------------------------- bit reader (MSB first) */
typedef struct { const uint8_t *p, *end; uint32_t cache; int n; } br_t;

static void br_init(br_t *b, const uint8_t *p, uint32_t len) { b->p = p; b->end = p + len; b->cache = 0; b->n = 0; }
static inline void br_fill(br_t *b) {
    while (b->n <= 24) {
        uint32_t v = (b->p < b->end) ? *b->p : 0;
        b->p++;
        b->cache |= v << (24 - b->n);
        b->n += 8;
    }
}
static inline uint32_t br_peek(br_t *b, int k) { br_fill(b); return b->cache >> (32 - k); }
static inline void br_skip(br_t *b, int k) { b->cache <<= k; b->n -= k; }
static inline uint32_t br_get(br_t *b, int k) { uint32_t v = br_peek(b, k); br_skip(b, k); return v; }

static uint32_t br_ue(br_t *b) {
    int z = 0;
    br_fill(b);
    while (!(b->cache & 0x80000000u)) { br_skip(b, 1); z++; if (b->n <= 0) br_fill(b); }
    br_skip(b, 1);
    if (!z) return 0;
    if (z <= 24) return ((1u << z) | br_get(b, z)) - 1;
    { uint32_t hi = br_get(b, z - 16); return ((((1u << (z - 16)) | hi) << 16) | br_get(b, 16)) - 1; }
}
static inline int32_t br_se(br_t *b) { uint32_t k = br_ue(b); return (k & 1) ? (int32_t)((k + 1) >> 1) : -(int32_t)(k >> 1); }

/* ---------------------------------------------------------------- canonical huffman, 10-bit fast table */
#define L1BITS 10
typedef struct { uint16_t l1[1 << L1BITS]; int first[18], cnt[18], off[18]; uint16_t sym[NSYM]; } huff_t;
static huff_t HC, HBL, HBC;

static void huff_build(huff_t *h, const uint8_t *len, int n) {
    int l, i, k = 0, code = 0;
    memset(h, 0, sizeof *h);
    for (i = 0; i < n; i++) h->cnt[len[i]]++;
    for (l = 1; l <= 16; l++) {
        h->off[l] = k;
        for (i = 0; i < n; i++) if (len[i] == l) h->sym[k++] = (uint16_t)i;
    }
    h->cnt[0] = 0;
    for (l = 1; l <= 16; l++) { code = (code + h->cnt[l - 1]) << 1; if (l == 1) code = 0; h->first[l] = code; }
    for (l = 1; l <= L1BITS; l++)
        for (i = 0; i < h->cnt[l]; i++) {
            int c = (h->first[l] + i) << (L1BITS - l), m;
            for (m = 0; m < (1 << (L1BITS - l)); m++) h->l1[c + m] = (uint16_t)((h->sym[h->off[l] + i] << 5) | l);
        }
}
static inline int huff_dec(br_t *b, const huff_t *h) {
    uint32_t pk = br_peek(b, 16);
    uint16_t e = h->l1[pk >> (16 - L1BITS)];
    if (e) { br_skip(b, e & 31); return e >> 5; }
    { int l; for (l = L1BITS + 1; l <= 16; l++) {
        int c = (int)(pk >> (16 - l)) - h->first[l];
        if (c >= 0 && c < h->cnt[l]) { br_skip(b, l); return h->sym[h->off[l] + c]; }
    } }
    return -1;
}

/* ---------------------------------------------------------------- pixel helpers */
static uint8_t CLAMP_[1024];
#define CLAMP(v) CLAMP_[(v) + 384]

#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define PACK(y0, cb, y1, cr) (((uint32_t)(y0) << 24) | ((uint32_t)(cb) << 16) | ((uint32_t)(y1) << 8) | (uint32_t)(cr))
/* odd shift: w0=[Ya Cb Yb Cr] w1=[Yc ..] -> [Yb Cb Yc Cr] */
#define ODDW(w0, w1) ((((w0) & 0xFF00u) << 16) | ((w0) & 0x00FF00FFu) | (((w1) >> 16) & 0xFF00u))
#define BY(w, i) (((w) >> (24 - 8 * (i))) & 0xFF)
#else
#define PACK(y0, cb, y1, cr) ((uint32_t)(y0) | ((uint32_t)(cb) << 8) | ((uint32_t)(y1) << 16) | ((uint32_t)(cr) << 24))
#define ODDW(w0, w1) ((((w0) >> 16) & 0xFFu) | ((w0) & 0xFF00FF00u) | (((w1) & 0xFFu) << 16))
#define BY(w, i) (((w) >> (8 * (i))) & 0xFF)
#endif
#define AVG4(a, b) (((a) | (b)) - ((((a) ^ (b)) >> 1) & 0x7F7F7F7Fu))

static inline uint32_t LD(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; } /* aligned in practice */

/* word at (pixel x, any parity, row pointer r): bytes Y(x) C(x>>1) Y(x+1) Cr */
static inline uint32_t SW(const uint8_t *r, int x) {
    if (!(x & 1)) return LD(r + 2 * x);
    return ODDW(LD(r + 2 * x - 2), LD(r + 2 * x + 2));
}

/* motion-compensated prediction of one 8x8 block into dst (32 words) */
static void mc_block(uint32_t *dst, const uint8_t *ref, int x0, int y0, int mx, int my) {
    int ix = mx >> 1, fx = mx & 1, iy = my >> 1, fy = my & 1, r, w;
    const uint8_t *row = ref + (y0 + iy) * STRIDE;
    int xs = x0 + ix;
    if (!fx && !fy) {
        if (!(xs & 1)) { ST.mc_int++;
            for (r = 0; r < 8; r++, row += STRIDE) for (w = 0; w < 4; w++) dst[r * 4 + w] = LD(row + 2 * xs + 4 * w);
        } else { ST.mc_odd++;
            for (r = 0; r < 8; r++, row += STRIDE) {
                const uint8_t *p = row + 2 * xs - 2;
                uint32_t a = LD(p), b, c, d, e;
                b = LD(p + 4); c = LD(p + 8); d = LD(p + 12); e = LD(p + 16);
                dst[r * 4 + 0] = ODDW(a, b); dst[r * 4 + 1] = ODDW(b, c); dst[r * 4 + 2] = ODDW(c, d); dst[r * 4 + 3] = ODDW(d, e);
            }
        }
    } else if (fx && !fy) { ST.mc_h++;
        for (r = 0; r < 8; r++, row += STRIDE) for (w = 0; w < 4; w++) {
            uint32_t a = SW(row, xs + 2 * w), b = SW(row, xs + 2 * w + 1); dst[r * 4 + w] = AVG4(a, b); }
    } else if (!fx && fy) { ST.mc_v++;
        for (r = 0; r < 8; r++, row += STRIDE) for (w = 0; w < 4; w++) {
            uint32_t a = SW(row, xs + 2 * w), b = SW(row + STRIDE, xs + 2 * w); dst[r * 4 + w] = AVG4(a, b); }
    } else { ST.mc_hv++;
        for (r = 0; r < 8; r++, row += STRIDE) for (w = 0; w < 4; w++) {
            uint32_t a = SW(row, xs + 2 * w), b = SW(row, xs + 2 * w + 1);
            uint32_t c = SW(row + STRIDE, xs + 2 * w), d = SW(row + STRIDE, xs + 2 * w + 1);
            uint32_t h0 = AVG4(a, b), h1 = AVG4(c, d); dst[r * 4 + w] = AVG4(h0, h1); }
    }
}

static inline void store_block(uint8_t *cur, int x0, int y0, const uint32_t *src) {
    uint8_t *row = cur + y0 * STRIDE + 2 * x0; int r;
    for (r = 0; r < 8; r++, row += STRIDE) memcpy(row, src + 4 * r, 16);
}

/* inverse 4x4 WHT, bit-exact with rnv.inv4_int: out = (H C H + 2) >> 2 */
static void iwht4(const int32_t *c, int32_t *o) {
    int32_t t[16]; int i;
    for (i = 0; i < 4; i++) { /* columns: H @ C */
        int32_t s0 = c[i] + c[4 + i], s1 = c[i] - c[4 + i], t0 = c[8 + i] + c[12 + i], t1 = c[8 + i] - c[12 + i];
        t[i] = s0 + t0; t[4 + i] = s0 - t0; t[8 + i] = s1 - t1; t[12 + i] = s1 + t1;
    }
    for (i = 0; i < 4; i++) { /* rows: (.) @ H */
        int32_t *q = t + 4 * i;
        int32_t s0 = q[0] + q[1], s1 = q[0] - q[1], t0 = q[2] + q[3], t1 = q[2] - q[3];
        o[4 * i + 0] = (s0 + t0 + 2) >> 2; o[4 * i + 1] = (s0 - t0 + 2) >> 2;
        o[4 * i + 2] = (s1 - t1 + 2) >> 2; o[4 * i + 3] = (s1 + t1 + 2) >> 2;
    }
}

/* decode one coded 4x4 subblock: returns residual (16) */
static void decode_sb(br_t *b, int step, int32_t *res, int chroma) {
    int32_t c[16]; int pos = -1, n = 0;
    memset(c, 0, sizeof c);
    for (;;) {
        int s = huff_dec(b, &HC), last = s / (RUNMAX * LVMAX), run = (s / LVMAX) % RUNMAX, lv = s % LVMAX + 1;
        if (lv == LVMAX) { lv += (int)br_ue(b); ST.esc++; }
        if (br_get(b, 1)) lv = -lv;
        pos += run + 1; c[ZZ[pos]] = lv * step; n++;
        if (last) break;
    }
    ST.coefs += n;
    if (n == 1 && pos == 0) { int i, v = (c[0] + 2) >> 2; if (chroma) ST.sb_dc_c++; else ST.sb_dc++; for (i = 0; i < 16; i++) res[i] = v; }
    else { if (chroma) ST.sb_ac_c++; else ST.sb_ac++; ST.iwht_cyc += (20 + 16 * n < 218) ? 20 + 16 * n : 218; iwht4(c, res); }
}

/* add residual (subblocks per cbp) to predicted words and store */
static void add_res_store(uint8_t *cur, int x0, int y0, const uint32_t *pred, int cbp, int32_t r[8][16]) {
    uint8_t *row = cur + y0 * STRIDE + 2 * x0; int y, w;
    for (y = 0; y < 8; y++, row += STRIDE) {
        int sby = (y >> 2) * 2, ry = y & 3, cbs = 4 + (y >> 2), crs = 6 + (y >> 2);
        for (w = 0; w < 4; w++) {
            uint32_t p = pred[4 * y + w];
            int sb = sby + (w >> 1), cx = 2 * (w & 1);
            int y0v = BY(p, 0), cb = BY(p, 1), y1v = BY(p, 2), cr = BY(p, 3);
            if (cbp & (1 << sb)) { y0v = CLAMP(y0v + r[sb][4 * ry + cx]); y1v = CLAMP(y1v + r[sb][4 * ry + cx + 1]); }
            if (cbp & (1 << cbs)) cb = CLAMP(cb + r[cbs][4 * ry + w]);
            if (cbp & (1 << crs)) cr = CLAMP(cr + r[crs][4 * ry + w]);
            { uint32_t o = PACK(y0v, cb, y1v, cr); memcpy(row + 4 * w, &o, 4); }
        }
    }
}

/* ---------------------------------------------------------------- RNV3 */
static void store_planar(uint8_t *cur, int x0, int y0, const uint8_t y[8][8], const uint8_t u[8][4], const uint8_t v[8][4]) {
    int r, x; uint8_t *row = cur + y0 * STRIDE + 2 * x0;
    for (r = 0; r < 8; r++, row += STRIDE) for (x = 0; x < 4; x++) { row[4 * x] = y[r][2 * x]; row[4 * x + 1] = u[r][x]; row[4 * x + 2] = y[r][2 * x + 1]; row[4 * x + 3] = v[r][x]; }
}
/* one component's corners; kc[i] >= 0 = already known from a neighbour */
static void corners(br_t *b, int *last, int c[4], int qc, const int kc[3]) {
    c[0] = kc[0] >= 0 ? kc[0] : *last + qc * br_se(b);
    c[1] = kc[1] >= 0 ? kc[1] : c[0] + qc * br_se(b);
    c[2] = kc[2] >= 0 ? kc[2] : c[0] + qc * br_se(b);
    c[3] = c[2] + c[1] - c[0] + qc * br_se(b);
    *last = c[3];
}

/* frame decoder follows */
/* ---------------------------------------------------------------- frame decoder */
typedef struct { uint8_t *buf[2]; int cur; uint8_t prevmode[NB]; uint8_t mode[NB]; } rnv_dec;

void rnv_init(rnv_dec *d, uint8_t *a, uint8_t *b) {
    int i;

    for (i = 0; i < 1024; i++) CLAMP_[i] = (uint8_t)(i - 384 < 0 ? 0 : i - 384 > 255 ? 255 : i - 384);
    huff_build(&HC, COEF_LEN, NSYM); huff_build(&HBL, CBPL_LEN, 16); huff_build(&HBC, CBPC_LEN, 16);
    for (i = 0; i < W * H; i++) { uint32_t o = PACK(0, 128, 0, 128); if (!(i & 1)) { memcpy(a + 2 * i, &o, 4); memcpy(b + 2 * i, &o, 4); } }
    d->buf[0] = a; d->buf[1] = b; d->cur = 1;
    memset(d->prevmode, INTRA, NB);
}

static uint32_t rd32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]; }

/* decode one frame; returns pointer to the new front buffer */
/* R3 shares its syntax with R2 except the mode code and the new modes */
static uint8_t *r3_decode(rnv_dec *d, const uint8_t *f, uint32_t len);
uint8_t *rnv_decode(rnv_dec *d, const uint8_t *f, uint32_t len) {
    if (f[0] == 'R' && f[1] == '3') return r3_decode(d, f, len);
    uint8_t *ref = d->buf[d->cur ^ 1], *cur = d->buf[d->cur];
    int qp = f[4], step, blk = 0, mvx = 0, mvy = 0, fy = 128, fu = 128, fv = 128;
    uint32_t l0 = rd32(f + 8), l1 = rd32(f + 12), l2 = rd32(f + 16), l3 = rd32(f + 20);
    const uint8_t *s3 = f + 24 + l0 + l1 + l2;
    br_t s0, s1, s2; uint32_t pred[32]; int32_t res[8][16];
    (void)len;
    if (f[0] != 'R' || f[1] != '2') return cur; /* not an RNV2 frame */
    step = QSTEP2[qp];
    br_init(&s0, f + 24, l0); br_init(&s1, f + 24 + l0, l1); br_init(&s2, f + 24 + l0 + l1, l2);
    ST.bits += 8 * (24 + l0 + l1 + l2 + l3);
    while (blk < NB) {
        int run = (int)br_ue(&s0), e = blk + run, mode, cbp = 0, i, x0, y0;
        for (; blk < e; blk++) { /* skipped blocks: copy only if the block changed last frame */
            ST.blk[SKIP]++;
            if (d->prevmode[blk] != SKIP) {
                int bx = (blk % BW) * 8, by = (blk / BW) * 8, r; ST.skipcopy++;
                for (r = 0; r < 8; r++) memcpy(cur + (by + r) * STRIDE + 2 * bx, ref + (by + r) * STRIDE + 2 * bx, 16);
            }
            d->prevmode[blk] = SKIP;
        }
        if (blk >= NB) break;
        mode = 1 + (int)br_get(&s0, 2);
        if (mode == 4) mode += (int)br_get(&s0, 1);
        ST.blk[mode]++;
        x0 = (blk % BW) * 8; y0 = (blk / BW) * 8;
        if (mode == RAW) { /* raw YUY2 block: 8 rows x 16 bytes from stream 3 */
            int r; for (r = 0; r < 8; r++, s3 += 16) memcpy(cur + (y0 + r) * STRIDE + 2 * x0, s3, 16);
            d->prevmode[blk] = RAW; blk++; continue;
        }
        if (mode != MC) { cbp = huff_dec(&s0, &HBL); cbp |= huff_dec(&s0, &HBC) << 4; }
        if (mode == MC || mode == MCR) {
            mvx += br_se(&s1); mvy += br_se(&s1);
            mc_block(pred, ref, x0, y0, mvx, mvy);
        } else if (mode == ZRES) {
            const uint8_t *row = ref + y0 * STRIDE + 2 * x0; int r;
            for (r = 0; r < 8; r++, row += STRIDE) memcpy(pred + 4 * r, row, 16);
        } else {
            uint32_t o;
            fy += br_se(&s1); fu += br_se(&s1); fv += br_se(&s1);
            o = PACK(fy, fu, fy, fv);
            for (i = 0; i < 32; i++) pred[i] = o;
        }
        if (cbp) {
            for (i = 0; i < 8; i++) if (cbp & (1 << i)) decode_sb(&s2, step, res[i], i >= 4);
            add_res_store(cur, x0, y0, pred, cbp, res);
        } else store_block(cur, x0, y0, pred);
        d->prevmode[blk] = (uint8_t)mode;
        blk++;
    }
    d->cur ^= 1;
    return cur;
}

static int nbm(const rnv_dec *d, int b, int m) { return d->mode[b] == m; }
static uint8_t *r3_decode(rnv_dec *d, const uint8_t *f, uint32_t len) {
    uint8_t *ref = d->buf[d->cur ^ 1], *cur = d->buf[d->cur];
    int qp = f[4], step, blk = 0, mvx = 0, mvy = 0, fy = 128, fu = 128, fv = 128, dith = f[5] & 1, q4 = f[6] & 15, q2a = f[6] >> 4, q2b = f[7] & 15, q4c = f[7] >> 4;
    uint32_t l0 = rd32(f + 8), l1 = rd32(f + 12), l2 = rd32(f + 16), l3 = rd32(f + 20);
    const uint8_t *s3 = f + 24 + l0 + l1 + l2;
    br_t s0, s1, s2; uint32_t pred[32]; int32_t res[8][16];
    /* lattice state.  Edges and corners shared with neighbours; everything is read before the same block overwrites it */
    static int topY[BW][5], rgtY[5], topC[2][BW][3], rgtC[2][3], crn[3][3][BW + 1];
    int last[3][3], btc[6], i, j, k;
    (void)len;
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) last[i][j] = 128;
    for (i = 0; i < 6; i++) btc[i] = 128;
    step = QSTEP2[qp];
    br_init(&s0, f + 24, l0); br_init(&s1, f + 24 + l0, l1); br_init(&s2, f + 24 + l0 + l1, l2);
    ST.bits += 8 * (24 + l0 + l1 + l2 + l3);
    while (blk < NB) {
        int run = (int)br_ue(&s0), e = blk + run, mode, cbp = 0, x0, y0, bx, by;
        for (; blk < e; blk++) {
            ST.blk[SKIP]++;
            if (d->prevmode[blk] != SKIP) {
                int ax = (blk % BW) * 8, ay = (blk / BW) * 8, r; ST.skipcopy++;
                for (r = 0; r < 8; r++) memcpy(cur + (ay + r) * STRIDE + 2 * ax, ref + (ay + r) * STRIDE + 2 * ax, 16);
            }
            d->prevmode[blk] = SKIP; d->mode[blk] = SKIP;
        }
        if (blk >= NB) break;
        /* MC 00, MCR 010, ZRES 011, INTRA 100, LAT 101dd, BTC4 110, BTC2 1110, RAW 1111 */
        { int c = (int)br_get(&s0, 2);
          if (c == 0) mode = MC;
          else if (c == 1) mode = br_get(&s0, 1) ? ZRES : MCR;
          else if (c == 2) { if (br_get(&s0, 1)) mode = M_LAT8 + (int)br_get(&s0, 2); else mode = INTRA; }
          else { if (!br_get(&s0, 1)) mode = M_BTC4; else mode = br_get(&s0, 1) ? RAW : M_BTC2; } }
        ST.blk[mode]++;
        bx = blk % BW; by = blk / BW; x0 = bx * 8; y0 = by * 8;
        d->mode[blk] = (uint8_t)mode;
        if (mode == RAW) {
            int r; for (r = 0; r < 8; r++, s3 += 16) memcpy(cur + (y0 + r) * STRIDE + 2 * x0, s3, 16);
            d->prevmode[blk] = RAW; blk++; continue;
        }
        if (mode >= M_LAT8 && mode <= M_LAT2) {
            int dn = mode - M_LAT8, kL = bx > 0 && nbm(d, blk - 1, mode), kT = by > 0 && nbm(d, blk - BW, mode);
            int kTL = kL || kT || (bx > 0 && by > 0 && nbm(d, blk - BW - 1, mode)), kTR = kT || (by > 0 && bx < BW - 1 && nbm(d, blk - BW + 1, mode));
            int G[5][5], g3[3][3], gc[2][3][3], c[4], kc[3], comp; int nE = dn == 2 ? 4 : dn == 1 ? 2 : 1; /* last index of an edge */
            for (comp = 0; comp < 3; comp++) {
                int nC = dn == 2 ? 2 : 1; /* last index of a chroma edge */
                kc[0] = kL ? (comp == 0 ? rgtY[0] : rgtC[comp - 1][0]) : kTL ? crn[dn][comp][bx] : -1;
                kc[1] = kTR ? crn[dn][comp][bx + 1] : -1;
                kc[2] = kL ? (comp == 0 ? rgtY[nE] : rgtC[comp - 1][nC]) : -1;
                corners(&s1, &last[dn][comp], c, comp == 0 && dn == 2 ? RNV3_QC2 : 1, kc);
                crn[dn][comp][bx] = c[2]; crn[dn][comp][bx + 1] = c[3];
                if (comp == 0) {
                    if (dn == 0) { g3[0][0] = c[0]; g3[0][2] = c[1]; g3[2][0] = c[2]; g3[2][2] = c[3]; }
                    else if (dn == 1) {
                        g3[0][0] = c[0]; g3[0][2] = c[1]; g3[2][0] = c[2]; g3[2][2] = c[3];
                        g3[0][1] = kT ? topY[bx][1] : lat_rup(c[0], c[1]) + q4 * br_se(&s1);
                        g3[1][0] = kL ? rgtY[1] : lat_rup(c[0], c[2]) + q4 * br_se(&s1);
                        g3[1][2] = lat_rup(c[1], c[3]) + q4 * br_se(&s1);
                        g3[2][1] = lat_rup(c[2], c[3]) + q4 * br_se(&s1);
                        g3[1][1] = lat_rup(g3[1][0], g3[1][2]) + q4 * br_se(&s1);
                    } else {
                        G[0][0] = c[0]; G[0][4] = c[1]; G[4][0] = c[2]; G[4][4] = c[3];
                        G[0][2] = kT ? topY[bx][2] : lat_rup(c[0], c[1]) + q2a * br_se(&s1);
                        G[2][0] = kL ? rgtY[2] : lat_rup(c[0], c[2]) + q2a * br_se(&s1);
                        G[2][4] = lat_rup(c[1], c[3]) + q2a * br_se(&s1);
                        G[4][2] = lat_rup(c[2], c[3]) + q2a * br_se(&s1);
                        G[2][2] = lat_rup(G[2][0], G[2][4]) + q2a * br_se(&s1);
                        for (i = 0; i <= 4; i += 2) {
                            if (i == 0 && kT) { G[0][1] = topY[bx][1]; G[0][3] = topY[bx][3]; continue; }
                            G[i][1] = lat_rdn(G[i][0], G[i][2]) + q2b * br_se(&s1); G[i][3] = lat_rdn(G[i][2], G[i][4]) + q2b * br_se(&s1);
                        }
                        for (i = 1; i <= 3; i += 2) for (j = 0; j <= 4; j++) {
                            if (j == 0 && kL) { G[i][0] = rgtY[i]; continue; }
                            G[i][j] = lat_rdn(G[i - 1][j], G[i + 1][j]) + q2b * br_se(&s1);
                        }
                    }
                } else {
                    int (*g)[3] = gc[comp - 1];
                    if (dn < 2) { g[0][0] = c[0]; g[0][2] = c[1]; g[2][0] = c[2]; g[2][2] = c[3]; }
                    else {
                        g[0][0] = c[0]; g[0][2] = c[1]; g[2][0] = c[2]; g[2][2] = c[3];
                        g[0][1] = kT ? topC[comp - 1][bx][1] : lat_rup(c[0], c[1]) + q4c * br_se(&s1);
                        g[1][0] = kL ? rgtC[comp - 1][1] : lat_rup(c[0], c[2]) + q4c * br_se(&s1);
                        g[1][2] = lat_rup(c[1], c[3]) + q4c * br_se(&s1);
                        g[2][1] = lat_rup(c[2], c[3]) + q4c * br_se(&s1);
                        g[1][1] = lat_rup(g[1][0], g[1][2]) + q4c * br_se(&s1);
                    }
                }
            }
            { uint32_t wd[32]; int r_;
              if (dn == 2) lat2_words(G, gc[0], gc[1], wd);
              else {
                  int cu[4] = {gc[0][0][0], gc[0][0][2], gc[0][2][0], gc[0][2][2]}, cv[4] = {gc[1][0][0], gc[1][0][2], gc[1][2][0], gc[1][2][2]};
                  if (dn == 1) lat4_words(g3, cu, cv, dith, wd);
                  else { int cy_[4] = {g3[0][0], g3[0][2], g3[2][0], g3[2][2]}; lat8_words(cy_, cu, cv, dith, wd); }
              }
              for (r_ = 0; r_ < 8; r_++) { uint8_t *row_ = cur + (y0 + r_) * STRIDE + 2 * x0; int x_; for (x_ = 0; x_ < 4; x_++) { uint32_t v_ = wd[4 * r_ + x_]; row_[4 * x_] = (uint8_t)(v_ >> 24); row_[4 * x_ + 1] = (uint8_t)(v_ >> 16); row_[4 * x_ + 2] = (uint8_t)(v_ >> 8); row_[4 * x_ + 3] = (uint8_t)v_; } } }
            /* edges for the neighbours */
            if (dn == 2) { for (k = 0; k < 5; k++) { topY[bx][k] = G[4][k]; rgtY[k] = G[k][4]; } }
            else if (dn == 1) { for (k = 0; k < 3; k++) { topY[bx][k] = g3[2][k]; rgtY[k] = g3[k][2]; } }
            else { rgtY[0] = g3[0][2]; rgtY[1] = g3[2][2]; }
            for (k = 0; k < 2; k++) {
                if (dn == 2) { int m_; for (m_ = 0; m_ < 3; m_++) { topC[k][bx][m_] = gc[k][2][m_]; rgtC[k][m_] = gc[k][m_][2]; } }
                else { rgtC[k][0] = gc[k][0][2]; rgtC[k][1] = gc[k][2][2]; }
            }
            d->prevmode[blk] = (uint8_t)mode; blk++;
            continue;
        }
        if (mode == M_BTC4 || mode == M_BTC2) {
            uint8_t y[8][8], u[8][4], v[8][4]; int r, x;
            for (k = 0; k < 6; k++) btc[k] += br_se(&s1);
            if (mode == M_BTC4) {
                int Y[4], U[7], V[7];
                btc4_y(btc[0], btc[1], Y); btc4_c(btc[2], btc[3], U); btc4_c(btc[4], btc[5], V);
                for (r = 0; r < 8; r++) { unsigned bits = (unsigned)s3[2 * r] << 8 | s3[2 * r + 1]; for (x = 0; x < 4; x++) { int a = (bits >> (14 - 4 * x)) & 3, b = (bits >> (12 - 4 * x)) & 3; y[r][2 * x] = (uint8_t)Y[a]; y[r][2 * x + 1] = (uint8_t)Y[b]; u[r][x] = (uint8_t)U[a + b]; v[r][x] = (uint8_t)V[a + b]; } }
                s3 += 16;
            } else {
                for (r = 0; r < 8; r++) for (x = 0; x < 4; x++) { int a = (s3[r] >> (7 - 2 * x)) & 1, b = (s3[r] >> (6 - 2 * x)) & 1, s = a + b; y[r][2 * x] = (uint8_t)btc[a]; y[r][2 * x + 1] = (uint8_t)btc[b]; u[r][x] = (uint8_t)btc2_c(btc[2], btc[3], s); v[r][x] = (uint8_t)btc2_c(btc[4], btc[5], s); }
                s3 += 8;
            }
            store_planar(cur, x0, y0, y, u, v);
            d->prevmode[blk] = (uint8_t)mode; blk++;
            continue;
        }
        if (mode != MC) { cbp = huff_dec(&s0, &HBL); cbp |= huff_dec(&s0, &HBC) << 4; }
        if (mode == MC || mode == MCR) {
            mvx += br_se(&s1); mvy += br_se(&s1);
            mc_block(pred, ref, x0, y0, mvx, mvy);
        } else if (mode == ZRES) {
            const uint8_t *row = ref + y0 * STRIDE + 2 * x0; int r;
            for (r = 0; r < 8; r++, row += STRIDE) memcpy(pred + 4 * r, row, 16);
        } else {
            uint32_t o;
            fy += br_se(&s1); fu += br_se(&s1); fv += br_se(&s1);
            o = PACK(fy, fu, fy, fv);
            for (i = 0; i < 32; i++) pred[i] = o;
        }
        if (cbp) {
            for (i = 0; i < 8; i++) if (cbp & (1 << i)) decode_sb(&s2, step, res[i], i >= 4);
            add_res_store(cur, x0, y0, pred, cbp, res);
        } else store_block(cur, x0, y0, pred);
        d->prevmode[blk] = (uint8_t)mode;
        blk++;
    }
    d->cur ^= 1;
    return cur;
}

#ifdef RNV_MAIN
#include <stdio.h>
#include <stdlib.h>
static uint32_t crc_tab[256];
static uint32_t crc32(const uint8_t *p, size_t n) {
    uint32_t c = 0xFFFFFFFFu; size_t i;
    if (!crc_tab[1]) { int k; for (i = 0; i < 256; i++) { uint32_t v = (uint32_t)i; for (k = 0; k < 8; k++) v = (v >> 1) ^ (0xEDB88320u & -(v & 1)); crc_tab[i] = v; } }
    for (i = 0; i < n; i++) c = crc_tab[(c ^ p[i]) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}
int main(int argc, char **argv) {
    FILE *f = fopen(argv[1], "rb"), *co = argc > 2 ? fopen(argv[2], "w") : NULL, *yo = argc > 3 ? fopen(argv[3], "wb") : NULL, *mo = argc > 4 ? fopen(argv[4], "wb") : NULL;
    static uint8_t A[W * H * 2], B[W * H * 2]; static rnv_dec d; uint8_t hdr[4]; int n = 0;
    uint8_t *buf = malloc(4 << 20);
    if (!f) return 1;
    rnv_init(&d, A, B);
    while (fread(hdr, 1, 4, f) == 4) {
        uint32_t len = rd32(hdr) & 0x7FFFFFFF; uint8_t *out; rnv_stats s0 = ST;
        if (fread(buf, 1, len, f) != len) break;
        out = rnv_decode(&d, buf, len);
        if (co) fprintf(co, "%d %u %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld\n", n, crc32(out, W * H * 2),
                        ST.blk[0] - s0.blk[0], ST.blk[1] - s0.blk[1], ST.blk[2] - s0.blk[2], ST.blk[3] - s0.blk[3], ST.blk[4] - s0.blk[4] + ST.blk[5] - s0.blk[5],
                        ST.skipcopy - s0.skipcopy, ST.mc_int - s0.mc_int, ST.mc_odd - s0.mc_odd, ST.mc_h - s0.mc_h, ST.mc_v - s0.mc_v,
                        ST.mc_hv - s0.mc_hv, ST.sb_dc - s0.sb_dc, ST.sb_ac - s0.sb_ac, ST.coefs - s0.coefs, ST.bits - s0.bits,
                        ST.blk[5] - s0.blk[5], ST.sb_dc_c - s0.sb_dc_c, ST.sb_ac_c - s0.sb_ac_c, ST.iwht_cyc - s0.iwht_cyc);
        if (yo) fwrite(out, 1, W * H * 2, yo);
        if (mo) fwrite(d.prevmode, 1, NB, mo);
        n++;
    }
    fprintf(stderr, "%d frames\n", n);
    return 0;
}
#endif
