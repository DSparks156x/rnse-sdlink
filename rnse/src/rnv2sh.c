/* rnv2sh.c -- RNV2 frame decoder, optimised for the SH-4A (see rnv2sh.h for what differs from the reference).
 * Built twice: -DRNV_PF=0 -> rnvo_decode, -DRNV_PF=1 -> rnvp_decode (adds PREF of the next reference line pair on
 * copy-type blocks). Bitstream semantics follow video/dec/rnv2dec.c line by line (stream s0 run/mode/cbp,
 * s1 motion/flat colour, s2 coefficients, s3 raw blocks). */
#include "rnv2_int.h"

#ifndef RNV_PF
#define RNV_PF 0
#endif
#if RNV_PF
#define DECODE_FN rnvp_decode
#define PF(p) __builtin_prefetch((const void *)(p))
#else
#define DECODE_FN rnvo_decode
#define PF(p) ((void)0)
#endif
/* RNV_FAST = rnvf_decode: same bitstream, same pixels, fewer instructions (bench 2026-11-05: IPC is about 1 and memory is under a quarter of the time).
 * Coefficients through a register bit window and a level/sign table, no sign branch, residual added straight in the second transform pass. */
#ifndef RNV_FAST
#define RNV_FAST 0
#endif
#if RNV_FAST
#undef DECODE_FN
#define DECODE_FN rnvf_decode
#endif
/* PREF does not stall (bench 2026-11-05: 56 ns against 150 ns for a read miss, a full-frame reference copy drops 5.70 -> 4.31 ms). Guessing the next pair's reference at coded blocks was worse: not done. */
#if RNV_FAST && !defined(RNV_HOST)
#define RP(p) __builtin_prefetch((const void *)(p))
#else
#define RP(p) ((void)0)
#endif

#define AI static inline __attribute__((always_inline))
#define S RNV_STRIDE
enum { SKIP = 0, MC, MCR, ZRES, INTRA, RAW };

static const rnv_u8 ZZ[16] = {0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15};

/* ------------------------------------------------------------------ bit reader: aligned base w, bit position p */
AI rnv_u32 peek32(const rnv_u8 *w, rnv_u32 p)
{
    const rnv_u8 *q = w + ((p >> 3) & ~3u);
    rnv_u32 s = p & 31;
    return (LDW(q) << s) | ((LDW(q + 4) >> 1) >> (31 - s));
}
AI rnv_u32 getb(const rnv_u8 *w, rnv_u32 *pp, int n)
{
    rnv_u32 v = peek32(w, *pp) >> (32 - n);
    *pp += n;
    return v;
}

static rnv_u32 ue_slow(const rnv_u8 *w, rnv_u32 *pp, int *bad)
{
    rnv_u32 p = *pp, v = peek32(w, p), z, x;
    if (!(v & 0xFFFF)) { *bad = 1; return 0; }           /* >= 32 leading zeros: corrupt */
    z = 16 + ((v & 0xFF00) ? rnv_clz8[(v >> 8) & 0xFF] : 8 + rnv_clz8[v & 0xFF]);
    p += z + 1;
    if (z <= 24) { x = peek32(w, p) >> (32 - z); p += z; *pp = p; return ((1u << z) | x) - 1; }
    x = peek32(w, p) >> (32 - (z - 16)); p += z - 16;
    v = peek32(w, p) >> 16; p += 16;
    *pp = p;
    return ((((1u << (z - 16)) | x) << 16) | v) - 1;
}
AI rnv_u32 ue(const rnv_u8 *w, rnv_u32 *pp, int *bad)
{
    rnv_u32 p = *pp, v = peek32(w, p), z, nb;
    if (v & 0x80000000u) { *pp = p + 1; return 0; }
    if (v >> 16) {
        z = (v >> 24) ? rnv_clz8[v >> 24] : 8 + rnv_clz8[(v >> 16) & 0xFF];
        nb = 2 * z + 1;
        *pp = p + nb;
        return (v >> (32 - nb)) - 1;
    }
    return ue_slow(w, pp, bad);
}
AI rnv_s32 se(const rnv_u8 *w, rnv_u32 *pp, int *bad)
{
    rnv_u32 k = ue(w, pp, bad);
    return (k & 1) ? (rnv_s32)((k + 1) >> 1) : -(rnv_s32)(k >> 1);
}

static int hslow(const rnv_huff *h, rnv_u32 v, rnv_u32 *pp)
{
    rnv_u32 pk = v >> 16;
    int l;
    for (l = RNV_L1BITS + 1; l <= 16; l++) {
        int c = (int)(pk >> (16 - l)) - h->first[l];
        if (c >= 0 && c < h->cnt[l]) { *pp += l; return h->info[h->off[l] + c]; }
    }
    return -1;
}
AI int hdec(const rnv_huff *h, const rnv_u8 *w, rnv_u32 *pp)
{
    rnv_u32 v = peek32(w, *pp);
    rnv_u32 e = h->l1[v >> (32 - RNV_L1BITS)];
    if (e) { *pp += e & 31; return (int)(e >> 5); }
    return hslow(h, v, pp);
}

/* ------------------------------------------------------------------ block copies (8 rows x 16 bytes) */
AI void copy8(rnv_u8 *d, int ds, const rnv_u8 *s, int ss)
{
    int r;
    _Pragma("GCC unroll 8") for (r = 0; r < 8; r++) {
        rnv_u32 a = LDW(s), b = LDW(s + 4), c = LDW(s + 8), e = LDW(s + 12);
        STW(d, a); STW(d + 4, b); STW(d + 8, c); STW(d + 12, e);
        d += ds; s += ss;
    }
}
/* copy from the reference: same, with the next row on its way before this one is read */
AI void copy8r(rnv_u8 *d, int ds, const rnv_u8 *s, int ss)
{
    int r;
    _Pragma("GCC unroll 8") for (r = 0; r < 8; r++) {
        rnv_u32 a, b, c, e;
        RP(s + ss);
        a = LDW(s); b = LDW(s + 4); c = LDW(s + 8); e = LDW(s + 12);
        STW(d, a); STW(d + 4, b); STW(d + 8, c); STW(d + 12, e);
        d += ds; s += ss;
    }
}
AI void fill8(rnv_u8 *d, int ds, rnv_u32 v)
{
    int r;
    _Pragma("GCC unroll 8") for (r = 0; r < 8; r++) { STW(d, v); STW(d + 4, v); STW(d + 8, v); STW(d + 12, v); d += ds; }
}
/* RAW block from stream 3 (any byte alignment) */
AI void rawcopy(rnv_u8 *d, const rnv_u8 *s)
{
    rnv_u32 k = (rnv_u32)(unsigned long)s & 3u;
    int r;
    if (!k) { copy8r(d, S, s, 16); return; }
    {
        const rnv_u8 *b = s - k;
        rnv_u32 sh = 8 * k, rs = 32 - sh, a0 = LDW(b);
        _Pragma("GCC unroll 8") for (r = 0; r < 8; r++) {
            rnv_u32 a1, a2, a3, a4;
            RP(b + 36);
            a1 = LDW(b + 4); a2 = LDW(b + 8); a3 = LDW(b + 12); a4 = LDW(b + 16);
            STW(d, (a0 << sh) | (a1 >> rs)); STW(d + 4, (a1 << sh) | (a2 >> rs));
            STW(d + 8, (a2 << sh) | (a3 >> rs)); STW(d + 12, (a3 << sh) | (a4 >> rs));
            a0 = a4; b += 16; d += S;
        }
    }
}

/* ------------------------------------------------------------------ motion compensation
 * b = reference row pointer at byte 2*xs (even xs) or 2*xs-2 (odd xs). For one row, with a0..a4 = 5 aligned words:
 *   even: E[w] = SW(xs+2w) = a_w,              O[w] = SW(xs+2w+1) = ODDW(a_w, a_w+1)
 *   odd:  E[w] = ODDW(a_w, a_w+1),             O[w] = a_w+1
 * integer: E | h: AVG4(E,O) | v: AVG4(E_r, E_r+1) | hv: AVG4(AVG4(E_r,O_r), AVG4(E_r+1,O_r+1))  (= reference) */
#define LD5(b) RP((b) + S); RP((b) + S + 16); rnv_u32 a0 = LDW(b), a1 = LDW((b) + 4), a2 = LDW((b) + 8), a3 = LDW((b) + 12), a4 = LDW((b) + 16)
#define EO(odd) \
    rnv_u32 E0 = odd ? ODDW(a0, a1) : a0, E1 = odd ? ODDW(a1, a2) : a1, E2 = odd ? ODDW(a2, a3) : a2, \
            E3 = odd ? ODDW(a3, a4) : a3; \
    rnv_u32 O0 = odd ? a1 : ODDW(a0, a1), O1 = odd ? a2 : ODDW(a1, a2), O2 = odd ? a3 : ODDW(a2, a3), \
            O3 = odd ? a4 : ODDW(a3, a4); (void)O0; (void)O1; (void)O2; (void)O3
#define ST4(d, x0, x1, x2, x3) do { STW(d, x0); STW((d) + 4, x1); STW((d) + 8, x2); STW((d) + 12, x3); } while (0)

AI void mc_case(rnv_u8 *d, int ds, const rnv_u8 *b, const int odd, const int fx, const int fy)
{
    int r;
    if (!fx && !fy) {
        _Pragma("GCC unroll 2") for (r = 0; r < 8; r++, b += S, d += ds) { LD5(b); EO(odd); ST4(d, E0, E1, E2, E3); }
    } else if (fx && !fy) {
        _Pragma("GCC unroll 2") for (r = 0; r < 8; r++, b += S, d += ds) {
            LD5(b); EO(odd); ST4(d, AVG4(E0, O0), AVG4(E1, O1), AVG4(E2, O2), AVG4(E3, O3));
        }
    } else if (!fx && fy) {
        rnv_u32 p0, p1, p2, p3;
        { LD5(b); EO(odd); p0 = E0; p1 = E1; p2 = E2; p3 = E3; }
        _Pragma("GCC unroll 2") for (r = 0; r < 8; r++, d += ds) {
            b += S;
            { LD5(b); EO(odd); ST4(d, AVG4(p0, E0), AVG4(p1, E1), AVG4(p2, E2), AVG4(p3, E3)); p0 = E0; p1 = E1; p2 = E2; p3 = E3; }
        }
    } else {
        rnv_u32 p0, p1, p2, p3;
        { LD5(b); EO(odd); p0 = AVG4(E0, O0); p1 = AVG4(E1, O1); p2 = AVG4(E2, O2); p3 = AVG4(E3, O3); }
        _Pragma("GCC unroll 2") for (r = 0; r < 8; r++, d += ds) {
            b += S;
            {
                LD5(b); EO(odd);
                rnv_u32 h0 = AVG4(E0, O0), h1 = AVG4(E1, O1), h2 = AVG4(E2, O2), h3 = AVG4(E3, O3);
                ST4(d, AVG4(p0, h0), AVG4(p1, h1), AVG4(p2, h2), AVG4(p3, h3));
                p0 = h0; p1 = h1; p2 = h2; p3 = h3;
            }
        }
    }
}

/* refblk = ref + block offset; mv in half-pels (already range-checked) */
static void mc(rnv_u8 *d, int ds, const rnv_u8 *refblk, int mvx, int mvy)
{
    int ix = mvx >> 1, fx = mvx & 1, iy = mvy >> 1, fy = mvy & 1;
    const rnv_u8 *b = refblk + iy * S + 2 * ix;
    if (ix & 1) {
        b -= 2;
        switch (fx | (fy << 1)) {
        case 0: mc_case(d, ds, b, 1, 0, 0); break;
        case 1: mc_case(d, ds, b, 1, 1, 0); break;
        case 2: mc_case(d, ds, b, 1, 0, 1); break;
        default: mc_case(d, ds, b, 1, 1, 1); break;
        }
    } else {
        switch (fx | (fy << 1)) {
        case 0: copy8r(d, ds, b, S); break;
        case 1: mc_case(d, ds, b, 0, 1, 0); break;
        case 2: mc_case(d, ds, b, 0, 0, 1); break;
        default: mc_case(d, ds, b, 0, 1, 1); break;
        }
    }
}

/* ------------------------------------------------------------------ residual */
static void iwht4(const rnv_s32 *c, rnv_s32 *o)
{
    rnv_s32 t[16];
    int i;
    for (i = 0; i < 4; i++) {
        rnv_s32 s0 = c[i] + c[4 + i], s1 = c[i] - c[4 + i], t0 = c[8 + i] + c[12 + i], t1 = c[8 + i] - c[12 + i];
        t[i] = s0 + t0; t[4 + i] = s0 - t0; t[8 + i] = s1 - t1; t[12 + i] = s1 + t1;
    }
    for (i = 0; i < 4; i++) {
        const rnv_s32 *q = t + 4 * i;
        rnv_s32 s0 = q[0] + q[1], s1 = q[0] - q[1], t0 = q[2] + q[3], t1 = q[2] - q[3];
        o[4 * i + 0] = (s0 + t0 + 2) >> 2; o[4 * i + 1] = (s0 - t0 + 2) >> 2;
        o[4 * i + 2] = (s1 - t1 + 2) >> 2; o[4 * i + 3] = (s1 + t1 + 2) >> 2;
    }
}

/* luma 4x4 at pr (pred temp, row stride 16 bytes): 4 rows x 2 words */
AI void add_y(rnv_u8 *pr, const rnv_s32 *r, const rnv_u8 *CL)
{
    int y;
    _Pragma("GCC unroll 8") for (y = 0; y < 4; y++, pr += 16, r += 4) {
        rnv_u32 p = LDW(pr), q = LDW(pr + 4);
        rnv_u32 a = CL[(int)(p >> 24) + r[0]], b = CL[(int)((p >> 8) & 0xFF) + r[1]];
        rnv_u32 c = CL[(int)(q >> 24) + r[2]], e = CL[(int)((q >> 8) & 0xFF) + r[3]];
        STW(pr, (p & 0x00FF00FFu) | (a << 24) | (b << 8));
        STW(pr + 4, (q & 0x00FF00FFu) | (c << 24) | (e << 8));
    }
}
AI void add_ydc(rnv_u8 *pr, const rnv_u8 *c)
{
    int y;
    _Pragma("GCC unroll 8") for (y = 0; y < 4; y++, pr += 16) {
        rnv_u32 p = LDW(pr), q = LDW(pr + 4);
        STW(pr, (p & 0x00FF00FFu) | ((rnv_u32)c[p >> 24] << 24) | ((rnv_u32)c[(p >> 8) & 0xFF] << 8));
        STW(pr + 4, (q & 0x00FF00FFu) | ((rnv_u32)c[q >> 24] << 24) | ((rnv_u32)c[(q >> 8) & 0xFF] << 8));
    }
}
/* chroma 4x4 (Cb: sh 16, Cr: sh 0) over 4 rows x 4 words */
AI void add_c(rnv_u8 *pr, const rnv_s32 *r, const rnv_u8 *CL, const int sh)
{
    int y, w;
    const rnv_u32 m = ~(0xFFu << sh);
    _Pragma("GCC unroll 8") for (y = 0; y < 4; y++, pr += 16, r += 4)
        _Pragma("GCC unroll 4") for (w = 0; w < 4; w++) {
            rnv_u32 p = LDW(pr + 4 * w);
            rnv_u32 v = CL[(int)((p >> sh) & 0xFF) + r[w]];
            STW(pr + 4 * w, (p & m) | (v << sh));
        }
}
AI void add_cdc(rnv_u8 *pr, const rnv_u8 *c, const int sh)
{
    int y, w;
    const rnv_u32 m = ~(0xFFu << sh);
    _Pragma("GCC unroll 8") for (y = 0; y < 4; y++, pr += 16)
        _Pragma("GCC unroll 4") for (w = 0; w < 4; w++) {
            rnv_u32 p = LDW(pr + 4 * w);
            STW(pr + 4 * w, (p & m) | ((rnv_u32)c[(p >> sh) & 0xFF] << sh));
        }
}

#if RNV_FAST
/* First pass (columns), clearing the coefficients as it goes; second pass (rows) adds to the prediction bytes directly.
 * Only for |residual| <= 384, which sum(|coef|) <= 1536 guarantees: same result as iwht4 + add_y / add_c. Byte order in memory is Y0 Cb Y1 Cr on the SH and on the host. */
AI void wht_p1(rnv_s32 *c, rnv_s32 *t)
{
    int i;
    _Pragma("GCC unroll 4") for (i = 0; i < 4; i++) {
        rnv_s32 a = c[i], b = c[4 + i], d = c[8 + i], e = c[12 + i];
        rnv_s32 s0 = a + b, s1 = a - b, t0 = d + e, t1 = d - e;
        c[i] = 0; c[4 + i] = 0; c[8 + i] = 0; c[12 + i] = 0;
        t[i] = s0 + t0; t[4 + i] = s0 - t0; t[8 + i] = s1 - t1; t[12 + i] = s1 + t1;
    }
}
#define BYA(p, o) ((p) = CL[(int)(p) + (o)])
#define WHT_FN static __attribute__((noinline))
typedef struct { rnv_u32 wb, wn; const rnv_u8 *wq; int pos; rnv_u32 sl; } rnv_win;
/* z * 4: stored directly as a byte offset into cf */
static const rnv_u8 ZZ4[16] = {0, 4, 16, 32, 20, 8, 12, 24, 36, 48, 52, 40, 28, 44, 56, 60};
/* Runs until the last coefficient of a subblock (returns 0). Stops before consuming anything on an escape or a code over 10 bits (rnv_cf level >= LVMAX, returns 1), or on a run past the block (returns 2). */
static __attribute__((noinline)) int coefs_fast(rnv_win *w, rnv_s32 *cf, const rnv_s32 *lvt)
{
    const rnv_u32 *CF = rnv_cf;
    const rnv_u8 *zz = ZZ4, *q = w->wq;
    rnv_u32 wb = w->wb, wn = w->wn, sl = w->sl, ent, lv, l;
    rnv_s32 v;
    int pos = w->pos, r = 0;
    do {
        ent = CF[wb >> (32 - RNV_L1BITS)];
        lv = (ent >> 16) & 0xFF;
        if (__builtin_expect(lv >= RNV_LVMAX, 0)) { r = 1; break; }
        l = ent & 0xFF;
        pos += (ent >> 8) & 0xFF;
        if (__builtin_expect(pos > 15, 0)) { pos -= (ent >> 8) & 0xFF; r = 2; break; }
        wb <<= l;
        v = lvt[2 * lv + (wb >> 31)];
        wb <<= 1;
        wn -= l + 1;
        sl += lv;
        *(rnv_s32 *)((char *)cf + zz[pos]) = v;
        if (wn < 17) { RP(q + 32); wb |= LDH(q) << (16 - wn); q += 2; wn += 16; }
    } while (!(ent >> 24));
    w->wb = wb; w->wn = wn; w->wq = q; w->pos = pos; w->sl = sl;
    return r;
}
WHT_FN void wht_add_y(rnv_s32 *c, rnv_u8 *pr, const rnv_u8 *CL)
{
    rnv_s32 t[16];
    const rnv_s32 *q = t;
    int y;
    wht_p1(c, t);
    _Pragma("GCC unroll 4") for (y = 0; y < 4; y++, q += 4, pr += 16) {
        rnv_s32 a = q[0], b = q[1], d = q[2], e = q[3];
        rnv_s32 s0 = a + b + 2, s1 = a - b + 2, t0 = d + e, t1 = d - e;
        BYA(pr[0], (s0 + t0) >> 2); BYA(pr[2], (s0 - t0) >> 2);
        BYA(pr[4], (s1 - t1) >> 2); BYA(pr[6], (s1 + t1) >> 2);
    }
}
WHT_FN void wht_add_c(rnv_s32 *c, rnv_u8 *pr, const rnv_u8 *CL)   /* pr = first sample: Cb at +1, Cr at +3 */
{
    rnv_s32 t[16];
    const rnv_s32 *q = t;
    int y;
    wht_p1(c, t);
    _Pragma("GCC unroll 4") for (y = 0; y < 4; y++, q += 4, pr += 16) {
        rnv_s32 a = q[0], b = q[1], d = q[2], e = q[3];
        rnv_s32 s0 = a + b + 2, s1 = a - b + 2, t0 = d + e, t1 = d - e;
        BYA(pr[0], (s0 + t0) >> 2); BYA(pr[4], (s0 - t0) >> 2);
        BYA(pr[8], (s1 - t1) >> 2); BYA(pr[12], (s1 + t1) >> 2);
    }
}
#endif

#if RNV_FAST
enum { M_LAT8 = 6, M_LAT4, M_LAT2, M_BTC4, M_BTC2 };
#define RNV3_QC2 2
#define LAT_DITH_BASE 2
#include "rnv3_sh.inc"
#endif
/* ------------------------------------------------------------------ frame */
static rnv_u32 rd32(const rnv_u8 *p) { return ((rnv_u32)p[0] << 24) | ((rnv_u32)p[1] << 16) | ((rnv_u32)p[2] << 8) | p[3]; }

static int fail(rnv_state *s, int code, int blk)
{
    int i;
    for (i = 0; i < RNV_NB; i++) s->age[i] = 0;
    s->last_err = (rnv_u32)code; s->err_blk = (rnv_u32)blk;
    s->last = s->cur;
    if (++s->cur == s->nbuf) s->cur = 0;
    return code;
}

#define BR_INIT(w, p, e, ptr, n) do { unsigned long a_ = (unsigned long)(ptr); w = (const rnv_u8 *)(a_ & ~3ul); \
                                      p = (rnv_u32)(a_ & 3u) * 8; e = p + (n) * 8; } while (0)
#define ADV() do { off += 16; x0 += 8; if (x0 == RNV_W) { x0 = 0; y0 += 8; off += 7 * S; } blk++; } while (0)
#define BAD(c) do { code = (c); goto out_err; } while (0)

int DECODE_FN(rnv_state *s, const rnv_u8 *f, rnv_u32 len)
{
    rnv_u8 *cur, *age = s->age;
    const rnv_u8 *ref, *w0, *w1, *w2, *s3, *s3e;
    const rnv_u8 *CL = rnv_clamp + 384;
    rnv_u32 l0, l1, l2, l3, p0, p1, p2, e0, e1, e2;
    rnv_u32 pred[32];
    rnv_s32 cf[16], res[16];
    rnv_u8 *pb = (rnv_u8 *)pred;
    int qp, step, blk = 0, x0 = 0, y0 = 0, off = 0, mvx = 0, mvy = 0, fy = 128, fu = 128, fv = 128, code = 0, bad = 0;
    const int sat = s->nbuf - 1;
    int i;
#if RNV_FAST
    rnv_s32 lvt[2 * RNV_LVMAX + 2];
    rnv_win W;
    int r3 = 0, lm = 0, tl = 0; r3local z;
#endif

    if (len < 24) return fail(s, RNV_E_LEN, 0);
#if RNV_FAST
    if (f[0] != 'R' || (f[1] != '2' && f[1] != '3')) return fail(s, RNV_E_MAGIC, 0);
    r3 = f[1] == '3';
    if (r3) {
        int a_, b_;
        z.q4 = f[6] & 15; z.q2a = f[6] >> 4; z.q2b = f[7] & 15; z.q4c = f[7] >> 4; z.dith = f[5] & 1;
        if (!z.q4 || !z.q2a || !z.q2b || !z.q4c) return fail(s, RNV_E_LEN, 0);
        for (a_ = 0; a_ < 3; a_++) for (b_ = 0; b_ < 3; b_++) z.last[a_][b_] = 128;
        for (a_ = 0; a_ < 6; a_++) z.btc[a_] = 128;
        for (a_ = 0; a_ < RNV_BW + 2; a_++) s->rowm[a_] = 0;
    }
#else
    if (f[0] != 'R' || f[1] != '2') return fail(s, RNV_E_MAGIC, 0);
#endif
    qp = f[4];
    if (qp > 51) return fail(s, RNV_E_QP, 0);
    l0 = rd32(f + 8); l1 = rd32(f + 12); l2 = rd32(f + 16); l3 = rd32(f + 20);
    if (l0 > len || l1 > len || l2 > len || l3 > len || 24 + l0 + l1 + l2 + l3 > len) return fail(s, RNV_E_LEN, 0);
    step = rnv_qstep()[qp];
    cur = s->buf[s->cur];
    ref = s->buf[s->last];
    BR_INIT(w0, p0, e0, f + 24, l0);
    BR_INIT(w1, p1, e1, f + 24 + l0, l1);
    BR_INIT(w2, p2, e2, f + 24 + l0 + l1, l2);
    s3 = f + 24 + l0 + l1 + l2; s3e = s3 + l3;
    for (i = 0; i < 16; i++) cf[i] = 0;
#if RNV_FAST
    for (i = 1; i <= RNV_LVMAX; i++) { lvt[2 * i] = i * step; lvt[2 * i + 1] = -(i * step); }
#define WSYNC() ((rnv_u32)(W.wq - w2) * 8 - W.wn)
#define WLOAD(p) do { rnv_u32 p_ = (p); W.wq = w2 + ((p_ >> 3) & ~3u); W.wb = LDW(W.wq) << (p_ & 31); W.wn = 32 - (p_ & 31); W.wq += 4; \
                      if (W.wn < 17) { W.wb |= LDH(W.wq) << (16 - W.wn); W.wq += 2; W.wn += 16; } } while (0)
    WLOAD(p2);
#endif

    while (blk < RNV_NB) {
        rnv_u32 run = ue(w0, &p0, &bad);
        int mode, cbp = 0;
        if (bad || run > (rnv_u32)(RNV_NB - blk)) BAD(RNV_E_RUN);
        while (run--) {                                   /* skipped blocks: copy only if changed recently */
            int a = age[blk];
#if RNV_FAST
            if (r3) { int bx_ = x0 >> 3; tl = s->rowm[bx_]; s->rowm[bx_] = 0; lm = 0; }
#endif
            if (a < sat) {
                if (RNV_PF && !(x0 & 8)) { int r; for (r = 0; r < 8; r++) PF(ref + off + 32 + r * S); }
                copy8r(cur + off, S, ref + off, S);
                age[blk] = (rnv_u8)(a + 1);
            }
            ADV();
        }
        if (blk >= RNV_NB) break;
        if (p0 > e0) BAD(RNV_E_OVERRUN);
#if RNV_FAST
        if (r3) {
            rnv_u32 v_ = peek32(w0, p0) >> 27;   /* MC 00, MCR 010, ZRES 011, INTRA 100, LAT 101dd, BTC4 110, BTC2 1110, RAW 1111 */
            if (v_ < 8) { mode = MC; p0 += 2; }
            else if (v_ < 12) { mode = MCR; p0 += 3; }
            else if (v_ < 16) { mode = ZRES; p0 += 3; }
            else if (v_ < 20) { mode = INTRA; p0 += 3; }
            else if (v_ < 24) { mode = M_LAT8 + (int)(v_ & 3); p0 += 5; if (mode > M_LAT2) BAD(RNV_E_HUFF); }
            else if (v_ < 28) { mode = M_BTC4; p0 += 3; }
            else if (v_ < 30) { mode = M_BTC2; p0 += 4; }
            else { mode = RAW; p0 += 4; }
            {
                int bx_ = x0 >> 3, kL = 0, kT, kTL, kTR, m_, up;
                if (!bx_) { lm = 0; tl = 0; }
                up = s->rowm[bx_];
                if (mode >= M_LAT8 && mode <= M_LAT2) {
                    m_ = mode;
                    kL = lm == m_; kT = up == m_; kTL = kL || kT || tl == m_; kTR = kT || s->rowm[bx_ + 1] == m_;
                    if (lat_block(s, &z, w1, &p1, mode - M_LAT8, bx_, kL, kT, kTL, kTR, cur + off)) BAD(RNV_E_UE);
                    if (p1 > e1) BAD(RNV_E_OVERRUN);
                    goto r3_done;
                }
                if (mode == M_BTC4 || mode == M_BTC2) {
                    int k_, nb_ = mode == M_BTC4 ? 16 : 8;
                    if (s3e - s3 < nb_) BAD(RNV_E_RAW);
                    for (k_ = 0; k_ < 6; k_++) z.btc[k_] += se(w1, &p1, &bad);
                    if (bad) BAD(RNV_E_UE);
                    if (mode == M_BTC4) btc4_block(cur + off, z.btc, s3); else btc2_block(cur + off, z.btc, s3);
                    s3 += nb_;
                    goto r3_done;
                }
                goto r3_old;
            r3_done:
                tl = up; s->rowm[bx_] = (rnv_u8)mode; lm = mode;
                age[blk] = 0; ADV();
                continue;
            r3_old:
                tl = up; s->rowm[bx_] = (rnv_u8)mode; lm = mode;
            }
        } else
#endif
        {
        mode = 1 + (int)getb(w0, &p0, 2);
        if (mode == 4) mode += (int)getb(w0, &p0, 1);
        }
        if (mode == RAW) {
            if (s3e - s3 < 128) BAD(RNV_E_RAW);
            rawcopy(cur + off, s3); s3 += 128;
            age[blk] = 0; ADV();
            continue;
        }
        if (mode != MC) {
            int a = hdec(&rnv_hbl, w0, &p0), b = hdec(&rnv_hbc, w0, &p0);
            if ((a | b) < 0) BAD(RNV_E_HUFF);
            cbp = a | (b << 4);
        }
        if (mode == MC || mode == MCR) {
            int fx, fyy, xs, ys;
            mvx += se(w1, &p1, &bad); mvy += se(w1, &p1, &bad);
            if (bad) BAD(RNV_E_UE);
            fx = mvx & 1; fyy = mvy & 1; xs = x0 + (mvx >> 1); ys = y0 + (mvy >> 1);
            if (xs < 0 || xs + 7 + fx > RNV_W - 1 || ys < 0 || ys + 7 + fyy > RNV_H - 1) BAD(RNV_E_MV);
            if (RNV_PF && !(x0 & 8)) { int r; for (r = 0; r < 8; r++) PF(ref + off + 32 + (mvy >> 1) * S + (mvx & ~1) + r * S); }
            if (!cbp) { mc(cur + off, S, ref + off, mvx, mvy); age[blk] = 0; ADV(); continue; }
            mc(pb, 16, ref + off, mvx, mvy);
        } else if (mode == ZRES) {
            if (!cbp) { copy8r(cur + off, S, ref + off, S); age[blk] = 0; ADV(); continue; }
            copy8r(pb, 16, ref + off, S);
        } else {
            rnv_u32 o;
            fy += se(w1, &p1, &bad); fu += se(w1, &p1, &bad); fv += se(w1, &p1, &bad);
            if (bad) BAD(RNV_E_UE);
            o = RNV_PACK(fy & 0xFF, fu & 0xFF, fy & 0xFF, fv & 0xFF);
            if (!cbp) { fill8(cur + off, S, o); age[blk] = 0; ADV(); continue; }
            fill8(pb, 16, o);
        }
#if RNV_FAST
        /* coded subblocks in bit order 0..7: Y TL TR BL BR, Cb top, Cb bottom, Cr top, Cr bottom */
        for (i = 0; i < 8; i++) {
            int pos;
            rnv_u32 sl;
            if (!(cbp & (1 << i))) continue;
            W.pos = -1; W.sl = 0;
            for (;;) {
                int r = coefs_fast(&W, cf, lvt);
                rnv_u32 q, ent, x;
                int lv, last;
                rnv_s32 v;
                if (!r) break;
                if (r == 2) BAD(RNV_E_COEF);
                q = WSYNC();
                ent = rnv_cf[W.wb >> (32 - RNV_L1BITS)];
                if (((ent >> 16) & 0xFF) == RNV_LVMAX) {      /* an escape with a short code */
                    q += ent & 0xFF;
                    lv = RNV_LVMAX; W.pos += (ent >> 8) & 0xFF; last = (int)(ent >> 24);
                } else {                                  /* code longer than 10 bits */
                    int e = hslow(&rnv_hc, peek32(w2, q), &q);
                    if (e < 0) BAD(RNV_E_HUFF);
                    lv = (e & 15) + 1; W.pos += ((e >> 4) & 15) + 1; last = e >> 8;
                }
                if (W.pos > 15) BAD(RNV_E_COEF);
                x = 0;
                if (lv == RNV_LVMAX) {                    /* escape: level += ue(), then the sign */
                    x = ue(w2, &q, &bad);
                    if (bad || x > 4096) BAD(RNV_E_COEF);
                }
                v = (lv + (int)x) * step;
                W.sl += (rnv_u32)lv + x;
                if (getb(w2, &q, 1)) v = -v;
                WLOAD(q);
                cf[ZZ[W.pos]] = v;
                if (last) break;
            }
            pos = W.pos; sl = W.sl;
            if (pos == 0) {                                /* DC only: constant residual */
                int dc = (cf[0] + 2) >> 2;
                cf[0] = 0;
                if (dc < -384) dc = -384; else if (dc > 384) dc = 384;   /* never for valid streams */
                if (i < 4) add_ydc(pb + (i >> 1) * 64 + (i & 1) * 8, CL + dc);
                else add_cdc(pb + (i & 1) * 64, CL + dc, i < 6 ? 16 : 0);
                continue;
            }
            if (__builtin_expect(sl * (rnv_u32)step > 1536, 0)) {   /* |res| could leave the clamp table: never for valid streams */
                int k;
                iwht4(cf, res);
                for (k = 0; k < 16; k++) { cf[k] = 0; res[k] = res[k] < -384 ? -384 : res[k] > 384 ? 384 : res[k]; }
                if (i < 4) add_y(pb + (i >> 1) * 64 + (i & 1) * 8, res, CL);
                else if (i < 6) add_c(pb + (i & 1) * 64, res, CL, 16);
                else add_c(pb + (i & 1) * 64, res, CL, 0);
                continue;
            }
            if (i < 4) wht_add_y(cf, pb + (i >> 1) * 64 + (i & 1) * 8, CL);
            else wht_add_c(cf, pb + (i & 1) * 64 + (i < 6 ? 1 : 3), CL);
        }
#else
        /* coded subblocks in bit order 0..7: Y TL TR BL BR, Cb top, Cb bottom, Cr top, Cr bottom */
        for (i = 0; i < 8; i++) {
            int pos = -1, n = 0, dc;
            rnv_u32 asum = 0;
            rnv_u8 zl[16];
            if (!(cbp & (1 << i))) continue;
            for (;;) {
                rnv_u32 pk = peek32(w2, p2), ent = rnv_hc.l1[pk >> (32 - RNV_L1BITS)], len;
                int e, lv, z;
                rnv_s32 v;
                if (ent) { len = ent & 31; e = (int)(ent >> 5); }
                else {                                    /* code longer than 10 bits */
                    rnv_u32 q = p2;
                    e = hslow(&rnv_hc, pk, &q);
                    if (e < 0) BAD(RNV_E_HUFF);
                    len = q - p2;
                }
                lv = (e & 15) + 1;
                pos += ((e >> 4) & 15) + 1;
                if (pos > 15) BAD(RNV_E_COEF);
                if (lv != RNV_LVMAX) {
                    v = lv * step;
                    asum += (rnv_u32)v;
                    if ((pk << len) & 0x80000000u) v = -v;   /* sign bit follows the code (len <= 16) */
                    p2 += len + 1;
                } else {                                  /* escape: level += ue(), then the sign */
                    rnv_u32 x;
                    p2 += len;
                    x = ue(w2, &p2, &bad);
                    if (bad || x > 4096) BAD(RNV_E_COEF);
                    v = (lv + (int)x) * step;
                    asum += (rnv_u32)v;
                    if (getb(w2, &p2, 1)) v = -v;
                }
                z = ZZ[pos];
                cf[z] = v; zl[n++] = (rnv_u8)z;
                if (e >> 8) break;
            }
            if (n == 1 && pos == 0) {                      /* DC only: constant residual */
                dc = (cf[0] + 2) >> 2;
                cf[0] = 0;
                if (dc < -384) dc = -384; else if (dc > 384) dc = 384;   /* never for valid streams */
                if (i < 4) add_ydc(pb + (i >> 1) * 64 + (i & 1) * 8, CL + dc);
                else add_cdc(pb + (i & 1) * 64, CL + dc, i < 6 ? 16 : 0);
                continue;
            }
            iwht4(cf, res);
            while (n) cf[zl[--n]] = 0;
            if (asum > 1536) {                             /* |res| could leave the clamp table: never for valid streams */
                int k;
                for (k = 0; k < 16; k++) res[k] = res[k] < -384 ? -384 : res[k] > 384 ? 384 : res[k];
            }
            if (i < 4) add_y(pb + (i >> 1) * 64 + (i & 1) * 8, res, CL);
            else if (i < 6) add_c(pb + (i & 1) * 64, res, CL, 16);
            else add_c(pb + (i & 1) * 64, res, CL, 0);
        }
#endif
        copy8(cur + off, S, pb, 16);
        age[blk] = 0;
        ADV();
    }
#if RNV_FAST
    p2 = WSYNC();
#endif
    if (p0 > e0 || p1 > e1 || p2 > e2) { code = RNV_E_OVERRUN; goto out_err; }
    s->last = s->cur;
    if (++s->cur == s->nbuf) s->cur = 0;
    s->frames++;
    return RNV_OK;
out_err:
    for (i = 0; i < 16; i++) cf[i] = 0;
    return fail(s, code, blk);
}
