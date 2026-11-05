/* rnv2enc NEON kernels (AArch64 / ARMv8 ASIMD).  Results are bit-identical to kern_scalar.c (make test).
 * On x86 the same source compiles against src/neon_emu.h (-DRNV_NEON_EMU) so the logic can be checked without
 * an ARM machine; the real proof is `make test` on the Pi. */
#include "kern.h"
#ifdef RNV2_HAVE_NEON
#ifdef __ARM_NEON
#include <arm_neon.h>
#else
#include "neon_emu.h"
#endif
#include <string.h>

/* ------------------------------------------------------------------ SAD / distortion */
static uint32_t sad8x8_n(const uint8_t *a, int as, const uint8_t *b, int bs) {
    uint16x8_t acc = vabdl_u8(vld1_u8(a), vld1_u8(b));
    int r;
    for (r = 1; r < 8; r++) acc = vabal_u8(acc, vld1_u8(a + r * as), vld1_u8(b + r * bs));
    return vaddlvq_u16(acc);
}

static void pd8_n(const uint8_t *s, const uint8_t *r, int kappa16, int thr, uint32_t out[8]) {
    const uint16x8_t T = vdupq_n_u16((uint16_t)thr);
    int sb;
    for (sb = 0; sb < 8; sb++) {
        uint8x16_t d = vabdq_u8(vld1q_u8(s + 16 * sb), vld1q_u8(r + 16 * sb));
        uint16x8_t a0 = vmovl_u8(vget_low_u8(d)), a1 = vmovl_u8(vget_high_u8(d));
        uint16x8_t m0 = vqsubq_u16(a0, T), m1 = vqsubq_u16(a1, T);
        uint16x8_t a16_0 = vshlq_n_u16(a0, 4), a16_1 = vshlq_n_u16(a1, 4);
        uint16x8_t mk0 = vmulq_n_u16(m0, (uint16_t)kappa16), mk1 = vmulq_n_u16(m1, (uint16_t)kappa16);
        /* two independent accumulator chains (A76 multiply-accumulate latency) */
        uint32x4_t acc = vmull_u16(vget_low_u16(a16_0), vget_low_u16(a0)), acc2 = vmull_u16(vget_low_u16(mk0), vget_low_u16(m0));
        acc = vmlal_u16(acc, vget_high_u16(a16_0), vget_high_u16(a0)); acc2 = vmlal_u16(acc2, vget_high_u16(mk0), vget_high_u16(m0));
        acc = vmlal_u16(acc, vget_low_u16(a16_1), vget_low_u16(a1)); acc2 = vmlal_u16(acc2, vget_low_u16(mk1), vget_low_u16(m1));
        acc = vmlal_u16(acc, vget_high_u16(a16_1), vget_high_u16(a1)); acc2 = vmlal_u16(acc2, vget_high_u16(mk1), vget_high_u16(m1));
        out[sb] = vaddvq_u32(vaddq_u32(acc, acc2));
    }
}

/* ------------------------------------------------------------------ WHT */
/* 4x4 transpose of two blocks at once: lanes 0-3 block A, lanes 4-7 block B */
static inline void tr4x2(int16x8_t *a0, int16x8_t *a1, int16x8_t *a2, int16x8_t *a3) {
    int16x8x2_t t01 = vtrnq_s16(*a0, *a1), t23 = vtrnq_s16(*a2, *a3);
    int32x4x2_t u02 = vtrnq_s32(vreinterpretq_s32_s16(t01.val[0]), vreinterpretq_s32_s16(t23.val[0]));
    int32x4x2_t u13 = vtrnq_s32(vreinterpretq_s32_s16(t01.val[1]), vreinterpretq_s32_s16(t23.val[1]));
    *a0 = vreinterpretq_s16_s32(u02.val[0]); *a2 = vreinterpretq_s16_s32(u02.val[1]);
    *a1 = vreinterpretq_s16_s32(u13.val[0]); *a3 = vreinterpretq_s16_s32(u13.val[1]);
}
static inline void bfly16(int16x8_t *a0, int16x8_t *a1, int16x8_t *a2, int16x8_t *a3) {
    int16x8_t s0 = vaddq_s16(*a0, *a1), s1 = vsubq_s16(*a0, *a1), t0 = vaddq_s16(*a2, *a3), t1 = vsubq_s16(*a2, *a3);
    *a0 = vaddq_s16(s0, t0); *a1 = vsubq_s16(s0, t0); *a2 = vsubq_s16(s1, t1); *a3 = vaddq_s16(s1, t1);
}
static void fwd8_n(const uint8_t *s, const uint8_t *p, int16_t *T) {
    int pr;
    for (pr = 0; pr < 4; pr++) {
        const uint8_t *sa = s + 32 * pr, *pa = p + 32 * pr;
        uint8x16_t SA = vld1q_u8(sa), SB = vld1q_u8(sa + 16), PA = vld1q_u8(pa), PB = vld1q_u8(pa + 16);
        int16x8_t A01 = vreinterpretq_s16_u16(vsubl_u8(vget_low_u8(SA), vget_low_u8(PA)));   /* A rows 0,1 */
        int16x8_t A23 = vreinterpretq_s16_u16(vsubl_u8(vget_high_u8(SA), vget_high_u8(PA))); /* A rows 2,3 */
        int16x8_t B01 = vreinterpretq_s16_u16(vsubl_u8(vget_low_u8(SB), vget_low_u8(PB)));
        int16x8_t B23 = vreinterpretq_s16_u16(vsubl_u8(vget_high_u8(SB), vget_high_u8(PB)));
        int16x8_t r0 = vcombine_s16(vget_low_s16(A01), vget_low_s16(B01));
        int16x8_t r1 = vcombine_s16(vget_high_s16(A01), vget_high_s16(B01));
        int16x8_t r2 = vcombine_s16(vget_low_s16(A23), vget_low_s16(B23));
        int16x8_t r3 = vcombine_s16(vget_high_s16(A23), vget_high_s16(B23));
        int16_t *ta = T + 32 * pr, *tb = ta + 16;
        bfly16(&r0, &r1, &r2, &r3);      /* H @ X  (rows) */
        tr4x2(&r0, &r1, &r2, &r3);
        bfly16(&r0, &r1, &r2, &r3);      /* (.) @ H -> columns of T */
        tr4x2(&r0, &r1, &r2, &r3);       /* back to raster rows */
        vst1_s16(ta, vget_low_s16(r0)); vst1_s16(ta + 4, vget_low_s16(r1)); vst1_s16(ta + 8, vget_low_s16(r2)); vst1_s16(ta + 12, vget_low_s16(r3));
        vst1_s16(tb, vget_high_s16(r0)); vst1_s16(tb + 4, vget_high_s16(r1)); vst1_s16(tb + 8, vget_high_s16(r2)); vst1_s16(tb + 12, vget_high_s16(r3));
    }
}

static int quant8_n(const int16_t *T, uint32_t A, uint32_t B, int S, int16_t *L) {
    const uint32x4_t vB = vdupq_n_u32(B); const uint16x8_t vmax = vdupq_n_u16(2047); const int32x4_t vS = vdupq_n_s32(-S);
    int sb, mask = 0;
    if (!(A & (A - 1))) { /* A = 2^m (power-of-two steps, incl. qp 0): (a*2^m + B) >> S == (a + (B >> m)) >> (S - m), 16-bit */
        int m = 0; uint16x8_t vK; int16x8_t vsh;
        while ((1u << m) < A) m++;
        vK = vdupq_n_u16((uint16_t)(B >> m)); vsh = vdupq_n_s16((int16_t)-(S - m));
        for (sb = 0; sb < 8; sb++) {
            int16x8_t t0 = vld1q_s16(T + 16 * sb), t1 = vld1q_s16(T + 16 * sb + 8);
            uint16x8_t l0 = vminq_u16(vshlq_u16(vaddq_u16(vreinterpretq_u16_s16(vabsq_s16(t0)), vK), vsh), vmax);
            uint16x8_t l1 = vminq_u16(vshlq_u16(vaddq_u16(vreinterpretq_u16_s16(vabsq_s16(t1)), vK), vsh), vmax);
            int16x8_t s0 = vreinterpretq_s16_u16(l0), s1 = vreinterpretq_s16_u16(l1);
            s0 = vbslq_s16(vcltq_s16(t0, vdupq_n_s16(0)), vnegq_s16(s0), s0);
            s1 = vbslq_s16(vcltq_s16(t1, vdupq_n_s16(0)), vnegq_s16(s1), s1);
            vst1q_s16(L + 16 * sb, s0); vst1q_s16(L + 16 * sb + 8, s1);
            if (vmaxvq_u16(vorrq_u16(l0, l1))) mask |= 1 << sb;
        }
        return mask;
    }
    for (sb = 0; sb < 8; sb++) {
        int16x8_t t0 = vld1q_s16(T + 16 * sb), t1 = vld1q_s16(T + 16 * sb + 8);
        uint16x8_t a0 = vreinterpretq_u16_s16(vabsq_s16(t0)), a1 = vreinterpretq_u16_s16(vabsq_s16(t1));
        uint32x4_t p0 = vmlal_n_u16(vB, vget_low_u16(a0), (uint16_t)A), p1 = vmlal_n_u16(vB, vget_high_u16(a0), (uint16_t)A);
        uint32x4_t p2 = vmlal_n_u16(vB, vget_low_u16(a1), (uint16_t)A), p3 = vmlal_n_u16(vB, vget_high_u16(a1), (uint16_t)A);
        uint16x8_t l0 = vminq_u16(vcombine_u16(vmovn_u32(vshlq_u32(p0, vS)), vmovn_u32(vshlq_u32(p1, vS))), vmax);
        uint16x8_t l1 = vminq_u16(vcombine_u16(vmovn_u32(vshlq_u32(p2, vS)), vmovn_u32(vshlq_u32(p3, vS))), vmax);
        int16x8_t s0 = vreinterpretq_s16_u16(l0), s1 = vreinterpretq_s16_u16(l1);
        s0 = vbslq_s16(vcltq_s16(t0, vdupq_n_s16(0)), vnegq_s16(s0), s0);
        s1 = vbslq_s16(vcltq_s16(t1, vdupq_n_s16(0)), vnegq_s16(s1), s1);
        vst1q_s16(L + 16 * sb, s0); vst1q_s16(L + 16 * sb + 8, s1);
        if (vmaxvq_u16(vorrq_u16(l0, l1))) mask |= 1 << sb;
    }
    return mask;
}

static inline void tr4_32(int32x4_t *a0, int32x4_t *a1, int32x4_t *a2, int32x4_t *a3) {
    int32x4x2_t t01 = vtrnq_s32(*a0, *a1), t23 = vtrnq_s32(*a2, *a3);
    *a0 = vcombine_s32(vget_low_s32(t01.val[0]), vget_low_s32(t23.val[0]));
    *a2 = vcombine_s32(vget_high_s32(t01.val[0]), vget_high_s32(t23.val[0]));
    *a1 = vcombine_s32(vget_low_s32(t01.val[1]), vget_low_s32(t23.val[1]));
    *a3 = vcombine_s32(vget_high_s32(t01.val[1]), vget_high_s32(t23.val[1]));
}
static inline void bfly32(int32x4_t *a0, int32x4_t *a1, int32x4_t *a2, int32x4_t *a3) {
    int32x4_t s0 = vaddq_s32(*a0, *a1), s1 = vsubq_s32(*a0, *a1), t0 = vaddq_s32(*a2, *a3), t1 = vsubq_s32(*a2, *a3);
    *a0 = vaddq_s32(s0, t0); *a1 = vsubq_s32(s0, t0); *a2 = vsubq_s32(s1, t1); *a3 = vaddq_s32(s1, t1);
}
static void iwht_n(const int32_t *c, int32_t *o) {
    int32x4_t r0 = vld1q_s32(c), r1 = vld1q_s32(c + 4), r2 = vld1q_s32(c + 8), r3 = vld1q_s32(c + 12);
    bfly32(&r0, &r1, &r2, &r3);
    tr4_32(&r0, &r1, &r2, &r3);
    bfly32(&r0, &r1, &r2, &r3);
    r0 = vrshrq_n_s32(r0, 2); r1 = vrshrq_n_s32(r1, 2); r2 = vrshrq_n_s32(r2, 2); r3 = vrshrq_n_s32(r3, 2);
    tr4_32(&r0, &r1, &r2, &r3);
    vst1q_s32(o, r0); vst1q_s32(o + 4, r1); vst1q_s32(o + 8, r2); vst1q_s32(o + 12, r3);
}

static void addclamp_n(const uint8_t *p, const int32_t *R, int mask, uint8_t *rec) {
    int sb;
    for (sb = 0; sb < 8; sb++, p += 16, R += 16, rec += 16) {
        uint8x16_t pv = vld1q_u8(p);
        if (mask & (1 << sb)) {
            int16x8_t r0 = vcombine_s16(vqmovn_s32(vld1q_s32(R)), vqmovn_s32(vld1q_s32(R + 4)));
            int16x8_t r1 = vcombine_s16(vqmovn_s32(vld1q_s32(R + 8)), vqmovn_s32(vld1q_s32(R + 12)));
            int16x8_t s0 = vqaddq_s16(vreinterpretq_s16_u16(vmovl_u8(vget_low_u8(pv))), r0);
            int16x8_t s1 = vqaddq_s16(vreinterpretq_s16_u16(vmovl_u8(vget_high_u8(pv))), r1);
            vst1q_u8(rec, vcombine_u8(vqmovun_s16(s0), vqmovun_s16(s1)));
        } else vst1q_u8(rec, pv);
    }
}

/* ------------------------------------------------------------------ half-pel planes, projections */
static void halfpel_row_n(const uint8_t *a, const uint8_t *b, int w, uint8_t *h, uint8_t *v, uint8_t *hv) {
    int x = 0;
    for (; x + 16 <= w - 1; x += 16) {
        uint8x16_t a0 = vld1q_u8(a + x), a1 = vld1q_u8(a + x + 1), b0 = vld1q_u8(b + x), b1 = vld1q_u8(b + x + 1);
        uint8x16_t ha = vrhaddq_u8(a0, a1), hb = vrhaddq_u8(b0, b1);
        vst1q_u8(h + x, ha); vst1q_u8(v + x, vrhaddq_u8(a0, b0)); vst1q_u8(hv + x, vrhaddq_u8(ha, hb));
    }
    for (; x < w; x++) {
        int ha = x < w - 1 ? (a[x] + a[x + 1] + 1) >> 1 : a[x];
        int hb = x < w - 1 ? (b[x] + b[x + 1] + 1) >> 1 : b[x];
        h[x] = (uint8_t)ha; v[x] = (uint8_t)((a[x] + b[x] + 1) >> 1); hv[x] = (uint8_t)((ha + hb + 1) >> 1);
    }
}

static void proj_rows_n(const uint8_t *p, int stride, int w, int h, int sw, uint32_t *rows, uint32_t *cols) {
    int y, x, nsx = w / sw;
    for (y = 0; y < h; y++, p += stride) {
        int k;
        for (k = 0; k < nsx; k++) {
            uint16x8_t acc = vdupq_n_u16(0); uint32_t s;
            for (x = k * sw; x + 16 <= (k + 1) * sw; x += 16) acc = vpadalq_u8(acc, vld1q_u8(p + x));
            s = vaddlvq_u16(acc);
            for (; x < (k + 1) * sw; x++) s += p[x];
            rows[y * nsx + k] += s;
        }
        for (x = 0; x + 16 <= w; x += 16) {
            uint8x16_t v = vld1q_u8(p + x);
            uint16x8_t lo = vmovl_u8(vget_low_u8(v)), hi = vmovl_u8(vget_high_u8(v));
            vst1q_u32(cols + x, vaddw_u16(vld1q_u32(cols + x), vget_low_u16(lo)));
            vst1q_u32(cols + x + 4, vaddw_u16(vld1q_u32(cols + x + 4), vget_high_u16(lo)));
            vst1q_u32(cols + x + 8, vaddw_u16(vld1q_u32(cols + x + 8), vget_low_u16(hi)));
            vst1q_u32(cols + x + 12, vaddw_u16(vld1q_u32(cols + x + 12), vget_high_u16(hi)));
        }
        for (; x < w; x++) cols[x] += p[x];
    }
}

/* ------------------------------------------------------------------ glass CSC, 8 pixel pairs per step */
static inline int32x4_t cl255_32(int32x4_t x) { return vminq_s32(vmaxq_s32(x, vdupq_n_s32(0)), vdupq_n_s32(255)); }
static inline int32x4_t sh16r(int32x4_t x) { return vshrq_n_s32(vaddq_s32(x, vdupq_n_s32(32768)), 16); }
/* 8 lanes as two int32x4 halves */
typedef struct { int32x4_t l, h; } i32x8;
static inline i32x8 w8(uint8x8_t v) { uint16x8_t u = vmovl_u8(v); i32x8 r; r.l = vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(u))); r.h = vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(u))); return r; }
static inline int16x8_t n8(i32x8 a) { return vcombine_s16(vmovn_s32(a.l), vmovn_s32(a.h)); }
static inline uint8x8_t q6_8(uint8x8_t x) { return vorr_u8(vand_u8(x, vdup_n_u8(0xFC)), vshr_n_u8(x, 6)); }
/* per-pixel glass error (3 channels) for y (int16x8, 0..255) with offsets oR/oG/oB (int16x8) vs targets t* (u8x8) */
static inline uint32x4x2_t perr8(int16x8_t y, int16x8_t oR, int16x8_t oG, int16x8_t oB, uint8x8_t tR, uint8x8_t tG, uint8x8_t tB) {
    uint8x8_t gR = q6_8(vqmovun_s16(vaddq_s16(y, oR))), gG = q6_8(vqmovun_s16(vaddq_s16(y, oG))), gB = q6_8(vqmovun_s16(vaddq_s16(y, oB)));
    uint8x8_t dR = vabd_u8(gR, tR), dG = vabd_u8(gG, tG), dB = vabd_u8(gB, tB);
    uint16x8_t sR = vmull_u8(dR, dR), sG = vmull_u8(dG, dG), sB = vmull_u8(dB, dB);
    uint32x4x2_t r;
    r.val[0] = vaddw_u16(vaddl_u16(vget_low_u16(sR), vget_low_u16(sG)), vget_low_u16(sB));
    r.val[1] = vaddw_u16(vaddl_u16(vget_high_u16(sR), vget_high_u16(sG)), vget_high_u16(sB));
    return r;
}
static inline uint16x8_t sel16(uint32x4_t ml, uint32x4_t mh, uint16x8_t a, uint16x8_t b) { /* mask ? a : b */
    uint16x8_t m = vcombine_u16(vmovn_u32(ml), vmovn_u32(mh));
    return vbslq_u16(m, a, b);
}

static void csc8(const uint8_t *rgb, uint8_t *Y, uint8_t *U, uint8_t *V) {
    uint8x16x3_t px = vld3q_u8(rgb); /* 16 pixels */
    uint8x16x2_t ur = vuzpq_u8(px.val[0], px.val[0]), ug = vuzpq_u8(px.val[1], px.val[1]), ub = vuzpq_u8(px.val[2], px.val[2]);
    uint8x8_t r0 = vget_low_u8(ur.val[0]), r1 = vget_low_u8(ur.val[1]), g0 = vget_low_u8(ug.val[0]), g1 = vget_low_u8(ug.val[1]);
    uint8x8_t b0 = vget_low_u8(ub.val[0]), b1 = vget_low_u8(ub.val[1]);
    uint8x8_t t0r = q6_8(r0), t0g = q6_8(g0), t0b = q6_8(b0), t1r = q6_8(r1), t1g = q6_8(g1), t1b = q6_8(b1);
    i32x8 R0 = w8(r0), G0 = w8(g0), B0 = w8(b0), R1 = w8(r1), G1 = w8(g1), B1 = w8(b1);
    i32x8 S0, S1, cb0, cb1, cr0, cr1, lu, lv, cu[4], cv[4];
    uint32x4_t bel = vdupq_n_u32(0xFFFFFFFFu), beh = bel;
    uint16x8_t bu = vdupq_n_u16(0), bv = bu, by0 = bu, by1 = bu;
    int c, d;
#define H2(EXPR) do { int32x4_t L_, H_; { int32x4_t r0_ = R0.l, g0_ = G0.l, b0_ = B0.l, r1_ = R1.l, g1_ = G1.l, b1_ = B1.l; (void)r0_; (void)g0_; (void)b0_; (void)r1_; (void)g1_; (void)b1_; L_ = EXPR; } \
                                                   { int32x4_t r0_ = R0.h, g0_ = G0.h, b0_ = B0.h, r1_ = R1.h, g1_ = G1.h, b1_ = B1.h; (void)r0_; (void)g0_; (void)b0_; (void)r1_; (void)g1_; (void)b1_; H_ = EXPR; } OUT.l = L_; OUT.h = H_; } while (0)
#define OUT S0
    H2(vaddq_s32(vaddq_s32(r0_, g0_), b0_));
#undef OUT
#define OUT S1
    H2(vaddq_s32(vaddq_s32(r1_, g1_), b1_));
#undef OUT
#define CBX(r, g, b) vmlaq_n_s32(vmlaq_n_s32(vmulq_n_s32(r, -11058), g, -21710), b, 32768)
#define CRX(r, g, b) vmlaq_n_s32(vmlaq_n_s32(vmulq_n_s32(r, 32768), g, -27439), b, -5329)
#define OUT cb0
    H2(CBX(r0_, g0_, b0_));
#undef OUT
#define OUT cb1
    H2(CBX(r1_, g1_, b1_));
#undef OUT
#define OUT cr0
    H2(CRX(r0_, g0_, b0_));
#undef OUT
#define OUT cr1
    H2(CRX(r1_, g1_, b1_));
#undef OUT
#define LSX(M) vmlaq_n_s32(vmlaq_n_s32(vmlaq_n_s32(vmlaq_n_s32(vmlaq_n_s32(vmulq_n_s32(r0_, M[0]), g0_, M[1]), b0_, M[2]), r1_, M[3]), g1_, M[4]), b1_, M[5])
#define OUT lu
    H2(LSX(RNV2_LSU));
#undef OUT
#define OUT lv
    H2(LSX(RNV2_LSV));
#undef OUT
    {
        const int32x4_t K128 = vdupq_n_s32(128 << 16), K256 = vdupq_n_s32((256 << 16) + 65536);
#define C1(dst, x) do { dst.l = cl255_32(sh16r(vaddq_s32(x.l, K128))); dst.h = cl255_32(sh16r(vaddq_s32(x.h, K128))); } while (0)
        C1(cu[0], lu); C1(cv[0], lv);
        cu[1].l = cl255_32(vshrq_n_s32(vaddq_s32(vaddq_s32(cb0.l, cb1.l), K256), 17)); cu[1].h = cl255_32(vshrq_n_s32(vaddq_s32(vaddq_s32(cb0.h, cb1.h), K256), 17));
        cv[1].l = cl255_32(vshrq_n_s32(vaddq_s32(vaddq_s32(cr0.l, cr1.l), K256), 17)); cv[1].h = cl255_32(vshrq_n_s32(vaddq_s32(vaddq_s32(cr0.h, cr1.h), K256), 17));
        C1(cu[2], cb0); C1(cv[2], cr0); C1(cu[3], cb1); C1(cv[3], cr1);
#undef C1
    }
    for (c = 0; c < 4; c++) {
        const int32x4_t K128 = vdupq_n_s32(128), K384 = vdupq_n_s32(384), K98 = vdupq_n_s32(98304);
        i32x8 uu, vv, oR, oG, oB, so, y0, y1;
        int16x8_t oR16, oG16, oB16, y016, y116;
        uint32x4x2_t e0, e1; uint32x4_t el, eh, ml, mh;
        uu.l = vsubq_s32(cu[c].l, K128); uu.h = vsubq_s32(cu[c].h, K128);
        vv.l = vsubq_s32(cv[c].l, K128); vv.h = vsubq_s32(cv[c].h, K128);
        oR.l = sh16r(vmulq_n_s32(vv.l, 91881)); oR.h = sh16r(vmulq_n_s32(vv.h, 91881));
        oG.l = sh16r(vmlaq_n_s32(vmulq_n_s32(uu.l, -22554), vv.l, -46802)); oG.h = sh16r(vmlaq_n_s32(vmulq_n_s32(uu.h, -22554), vv.h, -46802));
        oB.l = sh16r(vmulq_n_s32(uu.l, 116130)); oB.h = sh16r(vmulq_n_s32(uu.h, 116130));
        so.l = vmlaq_n_s32(vmulq_n_s32(vv.l, 45080), uu.l, 93576); so.h = vmlaq_n_s32(vmulq_n_s32(vv.h, 45080), uu.h, 93576);
#define YF(S, so_) cl255_32(vsubq_s32(vshrq_n_s32(vmulq_n_s32(vaddq_s32(vshrq_n_s32(vaddq_s32(vsubq_s32(vshlq_n_s32(S, 16), so_), K98), 16), K384), 21846), 16), K128))
        y0.l = YF(S0.l, so.l); y0.h = YF(S0.h, so.h); y1.l = YF(S1.l, so.l); y1.h = YF(S1.h, so.h);
#undef YF
        oR16 = n8(oR); oG16 = n8(oG); oB16 = n8(oB); y016 = n8(y0); y116 = n8(y1);
        e0 = perr8(y016, oR16, oG16, oB16, t0r, t0g, t0b);
        e1 = perr8(y116, oR16, oG16, oB16, t1r, t1g, t1b);
        el = vaddq_u32(e0.val[0], e1.val[0]); eh = vaddq_u32(e0.val[1], e1.val[1]);
        ml = vcltq_u32(el, bel); mh = vcltq_u32(eh, beh);
        bel = vbslq_u32(ml, el, bel); beh = vbslq_u32(mh, eh, beh);
        bu = sel16(ml, mh, vreinterpretq_u16_s16(n8(cu[c])), bu); bv = sel16(ml, mh, vreinterpretq_u16_s16(n8(cv[c])), bv);
        by0 = sel16(ml, mh, vreinterpretq_u16_s16(y016), by0); by1 = sel16(ml, mh, vreinterpretq_u16_s16(y116), by1);
    }
    { /* Y nudge onto a better 6-bit level */
        int16x8_t uu16 = vsubq_s16(vreinterpretq_s16_u16(bu), vdupq_n_s16(128)), vv16 = vsubq_s16(vreinterpretq_s16_u16(bv), vdupq_n_s16(128));
        i32x8 uu, vv; int16x8_t oR, oG, oB, n0, n1, y0 = vreinterpretq_s16_u16(by0), y1 = vreinterpretq_s16_u16(by1);
        uint32x4x2_t e0, e1;
        uu.l = vmovl_s16(vget_low_s16(uu16)); uu.h = vmovl_s16(vget_high_s16(uu16));
        vv.l = vmovl_s16(vget_low_s16(vv16)); vv.h = vmovl_s16(vget_high_s16(vv16));
        { i32x8 t; t.l = sh16r(vmulq_n_s32(vv.l, 91881)); t.h = sh16r(vmulq_n_s32(vv.h, 91881)); oR = n8(t); }
        { i32x8 t; t.l = sh16r(vmlaq_n_s32(vmulq_n_s32(uu.l, -22554), vv.l, -46802)); t.h = sh16r(vmlaq_n_s32(vmulq_n_s32(uu.h, -22554), vv.h, -46802)); oG = n8(t); }
        { i32x8 t; t.l = sh16r(vmulq_n_s32(uu.l, 116130)); t.h = sh16r(vmulq_n_s32(uu.h, 116130)); oB = n8(t); }
        e0 = perr8(y0, oR, oG, oB, t0r, t0g, t0b); e1 = perr8(y1, oR, oG, oB, t1r, t1g, t1b);
        n0 = y0; n1 = y1;
        for (d = -4; d <= 4; d += 2) {
            int16x8_t yc; uint32x4x2_t e; uint32x4_t ml, mh;
            if (!d) continue;
            yc = vreinterpretq_s16_u16(vmovl_u8(vqmovun_s16(vaddq_s16(y0, vdupq_n_s16((int16_t)d)))));
            e = perr8(yc, oR, oG, oB, t0r, t0g, t0b);
            ml = vcltq_u32(e.val[0], e0.val[0]); mh = vcltq_u32(e.val[1], e0.val[1]);
            e0.val[0] = vbslq_u32(ml, e.val[0], e0.val[0]); e0.val[1] = vbslq_u32(mh, e.val[1], e0.val[1]);
            n0 = vreinterpretq_s16_u16(sel16(ml, mh, vreinterpretq_u16_s16(yc), vreinterpretq_u16_s16(n0)));
            yc = vreinterpretq_s16_u16(vmovl_u8(vqmovun_s16(vaddq_s16(y1, vdupq_n_s16((int16_t)d)))));
            e = perr8(yc, oR, oG, oB, t1r, t1g, t1b);
            ml = vcltq_u32(e.val[0], e1.val[0]); mh = vcltq_u32(e.val[1], e1.val[1]);
            e1.val[0] = vbslq_u32(ml, e.val[0], e1.val[0]); e1.val[1] = vbslq_u32(mh, e.val[1], e1.val[1]);
            n1 = vreinterpretq_s16_u16(sel16(ml, mh, vreinterpretq_u16_s16(yc), vreinterpretq_u16_s16(n1)));
        }
        {
            uint8x8x2_t yy; yy.val[0] = vmovn_u16(vreinterpretq_u16_s16(n0)); yy.val[1] = vmovn_u16(vreinterpretq_u16_s16(n1));
            vst2_u8(Y, yy); vst1_u8(U, vmovn_u16(bu)); vst1_u8(V, vmovn_u16(bv));
        }
    }
#undef H2
#undef CBX
#undef CRX
#undef LSX
}

static void csc_glass_n(const uint8_t *rgb, int npairs, uint8_t *Y, uint8_t *U, uint8_t *V) {
    int p = 0;
    for (; p + 8 <= npairs; p += 8) csc8(rgb + 6 * p, Y + 2 * p, U + p, V + p);
    if (p < npairs) { /* tail through a padded buffer (same kernel => same results) */
        uint8_t tr[48] = {0}, ty[16], tu[8], tv[8]; int n = npairs - p;
        memcpy(tr, rgb + 6 * p, (size_t)6 * n);
        csc8(tr, ty, tu, tv);
        memcpy(Y + 2 * p, ty, (size_t)2 * n); memcpy(U + p, tu, (size_t)n); memcpy(V + p, tv, (size_t)n);
    }
}

const rnv2_kern rnv2_kern_neon = {"neon", csc_glass_n, sad8x8_n, pd8_n, fwd8_n, quant8_n, iwht_n, addclamp_n, halfpel_row_n, proj_rows_n};
#else
typedef int rnv2_kern_neon_unused;
#endif
