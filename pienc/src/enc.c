/* rnv2enc core: real-time RNV2 encoder.  Algorithm = video/rnv2.py (Encoder2) re-organised per block:
 *   1 change detection (+damage hint) and glass CSC only for changed 8x8 blocks
 *   2 global motion candidates from row/column projections (scroll / pan)
 *   3 block motion search: zero, previous MV, global candidates, small descent; two Jacobi neighbour passes with a
 *     12-point refine (as rnv2.py), half-pel last
 *   4 per-block RD analysis at qp (MC / ZRES / INTRA residual candidates, perceptual distortion, Huffman bit counts,
 *     SH-4A cycle model) - cached, so a cycle-cap retry (mu change) only re-runs step 5
 *   5 per-block mode decision at (lambda, mu); exact frame size and cycles from a cheap serial pass
 *   6 rnv2.py leaky-bucket / cycle-cap control loop over steps 4-5
 *   7 commit: reconstruction (bit-exact with rnv2dec.c) + serial bitstream write, half-pel planes for next frame
 * Every per-block decision depends only on frame-level inputs (Jacobi-style ME, no intra-frame causal state), so the
 * bitstream is identical for any thread count. */
#define _GNU_SOURCE
#include "rnv2enc.h"
#include "kern.h"
#include "rnv2_tables.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <stdatomic.h>
#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

#define W RNV2_W
#define H RNV2_H
#define CW (W / 2)
#define BW (W / 8)
#define BH (H / 8)
#define NB (BW * BH)
enum { SKIP = 0, MC, MCR, ZRES, INTRA, RAW };
#define RUNMAX 16
#define LVMAX 13
static const uint8_t ZZ[16] = {0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15};
/* SH-4A decode cost model (rnv.CY + rnv2 add_c4/add_c4dc), cycles */
enum { CY_FRAME = 20000, CY_SKIP = 6, CY_MEM = 306, CY_SKIPCOPY = 100 + 306, CY_HDR = 150, CY_MC_INT = 100, CY_MC_ODD = 398,
       CY_MC_H = 646, CY_MC_V = 318, CY_MC_HV = 924, CY_HALF_MEM = 38, CY_ZRES = 100 + 306, CY_FILL = 54 + 306,
       CY_RAW = 150 + 306 + 77, CY_COEF = 55, CY_IWHT_FULL = 218, CY_IWHT_BASE = 20, CY_IWHT_PER = 16, CY_DC = 30,
       CY_ADD_Y = 184, CY_ADD_YDC = 152, CY_ADD_C4 = 207, CY_ADD_C4DC = 177 };
static const int WSB[8] = {20, 20, 20, 20, 44, 44, 33, 33}; /* 1, 2.2, 1.65 (rnv2.WCB/WCR) x20 */
#define DSCALE 320.0                                         /* pd8 is x16, weights x20 */
#define MAXG 8

typedef struct {
    uint32_t ds[8], dm[8], di[8];
    uint32_t don[3][8];
    uint16_t bits[3][8], cyc[3][8];
    uint8_t nz[3], fill[3], has_mc, valid;
    uint16_t mvbits, mcc;
    double Don[3][8], Doff[3][8], Jds, Jdm; /* WSB * distortion / DSCALE, cached by the analysis: decide runs several times per frame */
    int16_t L3[3][128];                     /* quantised levels of each residual candidate (commit and the stream writer use these) */
} blkstat;
typedef struct { uint8_t mode, cbp, cand; int32_t cbits, cyc; } blkdec;
typedef struct { uint8_t *buf; size_t cap, n; uint64_t acc; int nb; } bw_t;
typedef void (*job_fn)(rnv2enc *e, int idx);
enum { GNSX = 4, GSW = W / GNSX, GNBAND = 3, GBHT = H / GNBAND, GRNG = 120 };
typedef struct { uint32_t rows[2][H * GNSX], cols[2][GNBAND][W]; int32_t prof[9][2][W]; int res[9][2]; } gme_t;
static uint32_t COEF_CODE[416], CBPL_CODE[16], CBPC_CODE[16];
/* -DRNV_COUNT: kernel work counters for the Pi speed model (single thread only; not thread safe) */
#ifdef RNV_COUNT
long long rnv_cnt[8]; /* csc pairs, sad calls, pd8 calls, fwd8 calls, iwht calls, addclamp calls, halfpel px, proj px */
#define KC(i, n) (rnv_cnt[i] += (n))
#else
#define KC(i, n) ((void)0)
#endif

struct rnv2enc {
    rnv2enc_params p;
    const rnv2_kern *k;
    int kappa16, thr;
    uint8_t *tY, *tU, *tV, *refrgb;
    const uint8_t *cur; int curstride; enum rnv2_fmt fmt;
    uint8_t *R[2][3];          /* recon double buffer */
    uint8_t *HP[3][3];         /* half-pel planes of the reference: [Y,U,V][h,v,hv] */
    int rb, first;
    uint8_t prevmode[NB], changed[NB], forced[NB], damaged[NB], medone[NB], hdirty[BH], mchg[2][NB];
    uint8_t (*rec)[128];
    uint8_t skq[NB]; double skmu[NB]; int qp0; double mu0; /* static-skip validity: (qp, mu, level) of the last evaluated SKIP */
    int16_t prev_mv[NB][2], mv0[NB][2], mv1[NB][2];
    uint32_t mc0[NB], mc1[NB];
    blkstat *bs; blkdec *bd;
    double fill, rate_pf, buf, mu; int qp, fn;
    /* headroom-driven quality: operating point ladder lvl 0 (near-exact) .. nlev-1 (= --lamk/--kappa/--thr) */
    int nlev, lvl, alvl, l0cool, since; double lamk; uint8_t skl[NB];
    _Atomic double pend_cap; /* live link capacity request (B/s), 0 = none; applied at the next encode call */
    int ng, g[MAXG][2], prev_g[2], qp_raised;
    gme_t *G;
    struct qtab_s *qt[52][2];
    /* frame-level job parameters */
    int cqp, step, wqp; double lam, cmu; int pass;
    bw_t w0, w1, w2; uint8_t *s3; size_t s3cap, s3n;
    uint8_t *obuf; size_t ocap;
    int n_forced, n_search;
    /* thread pool */
    int nth; pthread_t th[16];
    pthread_mutex_t m; pthread_cond_t cv, cvdone;
    int gen, quit; job_fn jfn; int njobs; _Atomic uint64_t next; atomic_int done; /* next = gen << 32 | next job index */
    /* per-frame work lists / flags */
    uint8_t key, tvalid, cscneed[NB]; /* key = coded from black; tvalid: tY/U/V = csc(refrgb) everywhere (not before the first frame) */
    uint16_t act[NB]; int nact, ncoded, nprev_act;
    uint16_t hx0[BH], hx1[BH];        /* x range of the blocks written to each band (half-pel update range) */
    int kl0;                          /* last key at level 0: 0 unknown, 1 fitted, 2 did not fit */
    double pbpa, pkey; int pq, pkq;    /* bytes per active block / of the last key at the classic point, at qp pq / pkq: first-qp prediction */
    blkstat *bs2; int bs2_cls, bs2_qp; /* the other analysis class, kept so a failed level-0 probe does not re-run the level-1 analysis */
};

/* ------------------------------------------------------------------ utils */
static double now_ms(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1e3 + t.tv_nsec * 1e-6; }
static inline int ue_len(uint32_t v) { return 2 * (31 - __builtin_clz(v + 1)) + 1; }
static inline uint32_t se_map(int v) { return v > 0 ? (uint32_t)(2 * v - 1) : (uint32_t)(-2 * v); }
static void *xmalloc(size_t n) { void *p = NULL; if (posix_memalign(&p, 64, n ? n : 64)) { fprintf(stderr, "rnv2enc: out of memory\n"); exit(1); } return p; }

static void bw_reset(bw_t *w) { w->n = 0; w->acc = 0; w->nb = 0; }
static void bw_grow(bw_t *w) { size_t nc = w->cap ? 2 * w->cap : 1 << 16; uint8_t *nbuf = xmalloc(nc); if (w->n) memcpy(nbuf, w->buf, w->n); free(w->buf); w->buf = nbuf; w->cap = nc; }
static inline void bw_put(bw_t *w, uint32_t v, int n) { /* n <= 32; MSB first; keeps < 32 bits pending */
    if (!n) return;
    w->acc = (w->acc << n) | (n == 32 ? v : (v & ((1u << n) - 1)));
    w->nb += n;
    if (w->nb >= 32) {
        uint32_t x;
        if (w->n + 4 > w->cap) bw_grow(w);
        w->nb -= 32; x = (uint32_t)(w->acc >> w->nb);
        w->buf[w->n] = (uint8_t)(x >> 24); w->buf[w->n + 1] = (uint8_t)(x >> 16); w->buf[w->n + 2] = (uint8_t)(x >> 8); w->buf[w->n + 3] = (uint8_t)x;
        w->n += 4;
    }
}
static inline void bw_ue(bw_t *w, uint32_t v) { uint64_t x = (uint64_t)v + 1; int nb = 0; while ((x >> nb) > 1) nb++; bw_put(w, 0, nb); bw_put(w, (uint32_t)x, nb + 1); }
static void bw_flush(bw_t *w) {
    if (w->n + 5 > w->cap) bw_grow(w);
    while (w->nb >= 8) { w->nb -= 8; w->buf[w->n++] = (uint8_t)(w->acc >> w->nb); }
    if (w->nb) { w->buf[w->n++] = (uint8_t)(w->acc << (8 - w->nb)); w->nb = 0; }
}

/* ------------------------------------------------------------------ thread pool */
/* A job is claimed with a CAS on (generation, index).  run_jobs waits for the jobs, not for the workers: a worker that is still
 * asleep or preempted (the Pi's cores are shared with the compositor) never delays a frame unless it holds a claimed job. */
static inline void claim_loop(rnv2enc *e, int g, job_fn fn, int n) {
    for (;;) {
        uint64_t v = atomic_load(&e->next); int i = (int)(uint32_t)v;
        if ((int)(v >> 32) != g || i >= n) return;
        if (atomic_compare_exchange_weak(&e->next, &v, v + 1)) {
            fn(e, i);
            if (atomic_fetch_add(&e->done, 1) + 1 == n) { pthread_mutex_lock(&e->m); pthread_cond_signal(&e->cvdone); pthread_mutex_unlock(&e->m); }
        }
    }
}
static void *worker(void *arg) {
    rnv2enc *e = arg; int g = 0;
    for (;;) {
        job_fn fn; int n;
        pthread_mutex_lock(&e->m);
        while (e->gen == g && !e->quit) pthread_cond_wait(&e->cv, &e->m);
        if (e->quit) { pthread_mutex_unlock(&e->m); return NULL; }
        g = e->gen; fn = e->jfn; n = e->njobs;
        pthread_mutex_unlock(&e->m);
        claim_loop(e, g, fn, n);
    }
}
static void run_jobs(rnv2enc *e, job_fn fn, int n) {
    int i, g;
    if (e->nth <= 1) { for (i = 0; i < n; i++) fn(e, i); return; }
    pthread_mutex_lock(&e->m);
    g = ++e->gen; e->jfn = fn; e->njobs = n; atomic_store(&e->done, 0); atomic_store(&e->next, (uint64_t)(uint32_t)g << 32);
    pthread_cond_broadcast(&e->cv);
    pthread_mutex_unlock(&e->m);
    claim_loop(e, g, fn, n);
    for (i = 0; i < 200 && atomic_load(&e->done) < n; i++) ; /* the other thread is usually finishing its last row */
    if (atomic_load(&e->done) < n) {
        pthread_mutex_lock(&e->m);
        while (atomic_load(&e->done) < n) pthread_cond_wait(&e->cvdone, &e->m);
        pthread_mutex_unlock(&e->m);
    }
}
/* run in parallel only when the work is worth a thread wake-up (a futex wake and a core out of idle cost ~50 us of CPU) */
static inline void run_w(rnv2enc *e, job_fn fn, int n, int work, int minwork) {
    int i;
    if (work < minwork) { for (i = 0; i < n; i++) fn(e, i); return; }
    run_jobs(e, fn, n);
}

/* ------------------------------------------------------------------ block gather / scatter (SB layout) */
static inline void gather_y(const uint8_t *p, int stride, uint8_t *o) {
    int r;
    for (r = 0; r < 8; r++, p += stride) { uint8_t *d = o + ((r >> 2) * 2) * 16 + (r & 3) * 4; memcpy(d, p, 4); memcpy(d + 16, p + 4, 4); }
}
static inline void gather_c(const uint8_t *p, int stride, uint8_t *o) { /* o = start of the component's 2 subblocks */
    int r;
    for (r = 0; r < 8; r++, p += stride) memcpy(o + (r >> 2) * 16 + (r & 3) * 4, p, 4);
}
static inline void gather_blk(const uint8_t *Y, const uint8_t *U, const uint8_t *V, int b, uint8_t *o) {
    int x0 = (b % BW) * 8, y0 = (b / BW) * 8;
    gather_y(Y + y0 * W + x0, W, o); gather_c(U + y0 * CW + x0 / 2, CW, o + 64); gather_c(V + y0 * CW + x0 / 2, CW, o + 96);
}
static inline void scatter_blk(uint8_t *Y, uint8_t *U, uint8_t *V, int b, const uint8_t *s) {
    int x0 = (b % BW) * 8, y0 = (b / BW) * 8, r;
    uint8_t *py = Y + y0 * W + x0, *pu = U + y0 * CW + x0 / 2, *pv = V + y0 * CW + x0 / 2;
    for (r = 0; r < 8; r++, py += W, pu += CW, pv += CW) {
        const uint8_t *d = s + ((r >> 2) * 2) * 16 + (r & 3) * 4;
        memcpy(py, d, 4); memcpy(py + 4, d + 16, 4);
        memcpy(pu, s + 64 + (r >> 2) * 16 + (r & 3) * 4, 4); memcpy(pv, s + 96 + (r >> 2) * 16 + (r & 3) * 4, 4);
    }
}
static inline const uint8_t *yplane(const rnv2enc *e, int f) { return f ? e->HP[0][f - 1] : e->R[e->rb][0]; }
static inline const uint8_t *cplane(const rnv2enc *e, int c, int f) { return f ? e->HP[c][f - 1] : e->R[e->rb][c]; }
/* motion-compensated prediction (rnv.pred_luma / pred_chroma = decoder packed MC), mv valid */
static void mc_pred(const rnv2enc *e, int b, int mx, int my, uint8_t *o) {
    int x0 = (b % BW) * 8, y0 = (b / BW) * 8, ix = mx >> 1, fx = mx & 1, iy = my >> 1, fy = my & 1;
    int fxc = fx & (ix & 1), cx = x0 / 2 + (ix >> 1), y = y0 + iy;
    gather_y(yplane(e, fy * 2 + fx) + y * W + x0 + ix, W, o);
    gather_c(cplane(e, 1, fy * 2 + fxc) + y * CW + cx, CW, o + 64);
    gather_c(cplane(e, 2, fy * 2 + fxc) + y * CW + cx, CW, o + 96);
}
static inline int mv_valid(int b, int mx, int my) {
    int x0 = (b % BW) * 8, y0 = (b / BW) * 8, ix = mx >> 1, fx = mx & 1, iy = my >> 1, fy = my & 1;
    return x0 + ix >= 0 && x0 + ix + 7 + fx <= W - 1 && y0 + iy >= 0 && y0 + iy + 7 + fy <= H - 1;
}
static int mc_cycles(int mx, int my) {
    int fx = mx & 1, fy = my & 1, odd = (mx >> 1) & 1;
    int c = fx && fy ? CY_MC_HV : fx ? CY_MC_H : fy ? CY_MC_V : odd ? CY_MC_ODD : CY_MC_INT;
    return c + CY_MEM + ((fx | fy) ? CY_HALF_MEM : 0);
}

/* ------------------------------------------------------------------ quantiser tables */
/* quantiser: L = sign(T) * min(2047, floor(|T| / (4 step) + dz)) (rnv.quantise on the orthonormal WHT, T = 4 C).
 * Evaluated as (|T| * A + B) >> S (S = 15 + floor(log2(4 step)), A < 2^16) with (A, B) searched so that it equals the exact table for every |T| <= 4080;
 * that form runs in the SIMD kernel.  If no (A, B) matches, the table path is used (same results, scalar). */
typedef struct qtab_s { int16_t t[4096]; uint32_t A, B; int S, ok; } qtab_t;
static const qtab_t *qtab(rnv2enc *e, int qp, int intra) {
    if (!e->qt[qp][intra]) {
        qtab_t *q = xmalloc(sizeof *q); int i; double st = QSTEP2[qp], dz = intra ? e->p.dz_intra : e->p.dz_inter;
        int S = 15 + (int)floor(log2(4 * st)); uint32_t A0 = (uint32_t)floor(ldexp(1.0, S) / (4 * st)), A, B, B0 = (uint32_t)floor(dz * ldexp(1.0, S));
        for (i = 0; i < 4096; i++) { double L = floor(i / 4.0 / st + dz); q->t[i] = (int16_t)(L > 2047 ? 2047 : L); }
        q->ok = 0; q->A = q->B = 0; q->S = S;
        for (A = A0; A <= A0 + 1 && !q->ok; A++) for (B = B0 > 512 ? B0 - 512 : 0; B <= B0 + 512 && !q->ok; B++) {
            if (A > 65535) continue;
            for (i = 0; i <= 4080; i++) { uint32_t l = (i * A + B) >> S; if ((int)(l > 2047 ? 2047 : l) != q->t[i]) break; }
            if (i > 4080) { q->ok = 1; q->A = A; q->B = B; }
        }
        e->qt[qp][intra] = q;
    }
    return e->qt[qp][intra];
}
/* quantise 8 subblocks; returns nz mask; per-sb bits (Huffman + sign + escape) and SH-4A cycles; L raster order */
static int quant_bits(const rnv2enc *e, const int16_t *T, const qtab_t *q, int16_t *L, uint16_t *bits, uint16_t *cyc) {
    int sb, k, mask = 0;
    if (q->ok) mask = e->k->quant8(T, q->A, q->B, q->S, L);
    else for (sb = 0; sb < 8; sb++) {
        int nz = 0;
        for (k = 0; k < 16; k++) { int v = T[16 * sb + k], a = v < 0 ? -v : v, l = q->t[a]; L[16 * sb + k] = (int16_t)(v < 0 ? -l : l); nz |= l; }
        if (nz) mask |= 1 << sb;
    }
    for (sb = 0; sb < 8; sb++) {
        const int16_t *l = L + 16 * sb; unsigned m = 0; int b = 0, prev = -1, nnz, lastpos;
        if (!(mask & (1 << sb))) { bits[sb] = 0; cyc[sb] = 0; continue; }
#ifdef __ARM_NEON
        { /* zigzag-order nonzero mask */
            static const uint8_t ZZT[16] = {0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15};
            static const uint8_t BIT[16] = {1, 2, 4, 8, 16, 32, 64, 128, 1, 2, 4, 8, 16, 32, 64, 128};
            int16x8_t a0 = vld1q_s16(l), a1 = vld1q_s16(l + 8);
            uint8x16_t nzv = vcombine_u8(vmovn_u16(vtstq_s16(a0, a0)), vmovn_u16(vtstq_s16(a1, a1)));
            nzv = vandq_u8(vqtbl1q_u8(nzv, vld1q_u8(ZZT)), vld1q_u8(BIT));
            m = (unsigned)vaddv_u8(vget_low_u8(nzv)) | ((unsigned)vaddv_u8(vget_high_u8(nzv)) << 8);
        }
#else
        for (k = 0; k < 16; k++) m |= (unsigned)(l[ZZ[k]] != 0) << k;
#endif
        nnz = __builtin_popcount(m); lastpos = 31 - __builtin_clz(m);
        while (m) {
            int pos = __builtin_ctz(m), v = l[ZZ[pos]], a = v < 0 ? -v : v, sym;
            m &= m - 1;
            sym = (pos == lastpos) * (RUNMAX * LVMAX) + (pos - prev - 1) * LVMAX + (a < LVMAX ? a : LVMAX) - 1; prev = pos;
            b += COEF_LEN[sym] + 1 + (a >= LVMAX ? ue_len((uint32_t)(a - LVMAX)) : 0);
        }
        {
            int chroma = sb >= 4, dco = (nnz == 1 && l[0] != 0);
            int iw = CY_IWHT_BASE + CY_IWHT_PER * nnz; if (iw > CY_IWHT_FULL) iw = CY_IWHT_FULL;
            cyc[sb] = (uint16_t)((dco ? CY_DC + (chroma ? CY_ADD_C4DC : CY_ADD_YDC) : iw + (chroma ? CY_ADD_C4 : CY_ADD_Y)) + CY_COEF * nnz);
        }
        bits[sb] = (uint16_t)b;
    }
    return mask;
}

static void recon_res(rnv2enc *e, const int16_t *L, int mask, int step, const uint8_t *P, uint8_t *rec) {
    int32_t R[128]; int sb, k;
    for (sb = 0; sb < 8; sb++) if (mask & (1 << sb)) { int32_t c[16]; for (k = 0; k < 16; k++) c[k] = L[16 * sb + k] * step; KC(4, 1); e->k->iwht(c, R + 16 * sb); }
    KC(5, 1); e->k->addclamp(P, R, mask, rec);
}

/* ------------------------------------------------------------------ phase 1: change detection + CSC */
/* 8 pixels of the input format -> RGB24 */
static inline void line8_rgb(const uint8_t *s, enum rnv2_fmt f, uint8_t *d) {
#ifdef __ARM_NEON
    if (f == RNV2_BGR24) { uint8x8x3_t v = vld3_u8(s), o; o.val[0] = v.val[2]; o.val[1] = v.val[1]; o.val[2] = v.val[0]; vst3_u8(d, o); }
    else { uint8x8x4_t v = vld4_u8(s); uint8x8x3_t o; if (f == RNV2_BGR0) { o.val[0] = v.val[2]; o.val[1] = v.val[1]; o.val[2] = v.val[0]; } else { o.val[0] = v.val[0]; o.val[1] = v.val[1]; o.val[2] = v.val[2]; } vst3_u8(d, o); }
#else
    int x;
    switch (f) {
    case RNV2_BGR24: for (x = 0; x < 8; x++, s += 3, d += 3) { d[0] = s[2]; d[1] = s[1]; d[2] = s[0]; } break;
    case RNV2_BGR0: for (x = 0; x < 8; x++, s += 4, d += 3) { d[0] = s[2]; d[1] = s[1]; d[2] = s[0]; } break;
    default: for (x = 0; x < 8; x++, s += 4, d += 3) { d[0] = s[0]; d[1] = s[1]; d[2] = s[2]; } break;
    }
#endif
}
/* CSC of one row segment of refrgb (npairs from pixel x).  The kernel is a pure per-pair function, so a 16-pixel group equal to the same
 * group one row up (already converted: same band, or an unchanged block) or to the group on its left reuses that output.  UI and map
 * content is mostly flat or vertically uniform, and this kernel is the most expensive per-pixel step in the encoder. */
static void csc_seg(rnv2enc *e, int y, int x, int npairs, int r) {
    const uint8_t *rgb = e->refrgb + ((size_t)y * W + x) * 3;
    uint8_t *Y = e->tY + y * W + x, *U = e->tU + y * CW + x / 2, *V = e->tV + y * CW + x / 2;
    int p = 0;
    while (p < npairs) {
        int n = npairs - p >= 8 ? 8 : npairs - p;
        if (r > 0 && !memcmp(rgb + 6 * p, rgb + 6 * p - W * 3, (size_t)6 * n)) { memcpy(Y + 2 * p, Y + 2 * p - W, (size_t)2 * n); memcpy(U + p, U + p - CW, (size_t)n); memcpy(V + p, V + p - CW, (size_t)n); }
        else if (p >= n && !memcmp(rgb + 6 * p, rgb + 6 * (p - n), (size_t)6 * n)) { memcpy(Y + 2 * p, Y + 2 * (p - n), (size_t)2 * n); memcpy(U + p, U + p - n, (size_t)n); memcpy(V + p, V + p - n, (size_t)n); }
        else { KC(0, n); e->k->csc_glass(rgb + 6 * p, n, Y + 2 * p, U + p, V + p); }
        p += n;
    }
}
static void job_csc(rnv2enc *e, int by) {
    int bx, r, y0 = by * 8, bx0 = 0, bx1 = BW, first = e->first;
    const uint8_t *dmg = e->damaged + by * BW; int bpp = e->fmt == RNV2_RGB24 || e->fmt == RNV2_BGR24 ? 3 : 4;
    if (!first) { /* x range of the damage in this band; none: nothing to convert or compare */
        while (bx0 < BW && !dmg[bx0]) bx0++;
        while (bx1 > bx0 && !dmg[bx1 - 1]) bx1--;
        if (bx0 >= bx1) { memset(e->changed + by * BW, 0, BW); memset(e->forced + by * BW, 1, BW); memset(e->cscneed + by * BW, 0, BW); return; }
    }
    for (bx = 0; bx < BW; bx++) {
        int b = by * BW + bx, diff = 0, need; uint8_t line[24];
        if (!dmg[bx] && !first) { e->changed[b] = 0; e->forced[b] = 1; e->cscneed[b] = 0; continue; }
        for (r = 0; r < 8; r++) { /* compare against, and update, the RGB24 copy of the previous source in one pass */
            const uint8_t *sp = e->cur + (size_t)(y0 + r) * e->curstride + (size_t)bx * 8 * bpp, *lp = sp; uint8_t *rp = e->refrgb + (size_t)(y0 + r) * W * 3 + bx * 24;
            if (e->fmt != RNV2_RGB24) { line8_rgb(sp, e->fmt, line); lp = line; }
            if (!e->tvalid || memcmp(lp, rp, 24)) { memcpy(rp, lp, 24); diff = 1; }
        }
        need = diff;
        e->cscneed[b] = (uint8_t)need; /* a key frame recodes every block, but the target of an unchanged source block is still valid */
        if (first) diff = 1;
        e->changed[b] = (uint8_t)diff;
        /* unchanged source, SKIP last frame, and the block was last evaluated at a qp / mu no lower than now (a lower
           qp or mu could make rnv2.py refine it): the decision cannot change -> SKIP without evaluation */
        e->forced[b] = (uint8_t)(!diff && e->p.static_skip && e->prevmode[b] == SKIP && e->qp0 >= e->skq[b] && e->mu0 >= e->skmu[b] && e->lvl >= e->skl[b]);
    }
    for (r = 0; r < 8; r++) { /* CSC on runs of blocks that need it */
        int y = y0 + r;
        for (bx = bx0; bx < bx1;) {
            int b0 = bx;
            if (!e->cscneed[by * BW + bx]) { bx++; continue; }
            while (bx < bx1 && e->cscneed[by * BW + bx]) bx++;
            csc_seg(e, y, b0 * 8, (bx - b0) * 4, r);
        }
    }
}

/* ------------------------------------------------------------------ phase 2: global motion from projections */
/* Row profiles per vertical strip (4 x 200 px) and column profiles per horizontal band (3 x 160 rows) of the current
 * target and the reference.  A 1-D SAD search over shifts finds scroll (dy) and pan (dx) per strip / band and for the
 * whole frame; static chrome cancels out of the profile difference, so a list scrolling inside a static frame is found.
 * Candidates (full-pel, converted to half-pel units) are handed to every block's motion search. */
static void search1d(const int32_t *c, const int32_t *r, int n, int R, int *best, int *second, int64_t *bcost, int64_t *zcost) {
    int d, d1 = 0, d2 = 0; int64_t b1 = INT64_MAX, b2 = INT64_MAX;
    for (d = -R; d <= R; d++) {
        int i0 = d < 0 ? -d : 0, i1 = d > 0 ? n - d : n, i; int32_t s = 0; int64_t sc;
        const int32_t *rr = r + d;
        if (i1 - i0 < n / 2) continue;
        for (i = i0; i < i1; i++) { int32_t x = c[i] - rr[i]; s += x < 0 ? -x : x; }
        sc = ((int64_t)s << 12) / (i1 - i0);
        if (d == 0) *zcost = sc;
        if (sc < b1) { if (abs(d - d1) > 2) { b2 = b1; d2 = d1; } b1 = sc; d1 = d; }
        else if (sc < b2 && abs(d - d1) > 2) { b2 = sc; d2 = d; }
    }
    *best = d1; *second = d2; *bcost = b1;
}
static void add_g(rnv2enc *e, int dx, int dy) {
    int i;
    if (!dx && !dy) return;
    if (e->ng >= MAXG) return;
    for (i = 0; i < e->ng; i++) if (e->g[i][0] == 2 * dx && e->g[i][1] == 2 * dy) return;
    e->g[e->ng][0] = 2 * dx; e->g[e->ng][1] = 2 * dy; e->ng++;
}
static void job_proj(rnv2enc *e, int idx) { /* idx: image (0 cur, 1 ref) x band */
    int img = idx / GNBAND, k = idx % GNBAND;
    memset(e->G->rows[img] + k * GBHT * GNSX, 0, sizeof(uint32_t) * GBHT * GNSX); memset(e->G->cols[img][k], 0, sizeof(uint32_t) * W);
    KC(7, W * GBHT); e->k->proj_rows((img ? e->R[e->rb][0] : e->tY) + k * GBHT * W, W, W, GBHT, GSW, e->G->rows[img] + k * GBHT * GNSX, e->G->cols[img][k]);
}
static void job_gsearch(rnv2enc *e, int j) { /* 0 full rows, 1 full cols, 2-5 strips (rows), 6-8 bands (cols) */
    int n = (j == 0 || (j >= 2 && j <= 5)) ? H : W, t; int64_t bc, zc = 0;
    search1d(e->G->prof[j][0], e->G->prof[j][1], n, GRNG, &e->G->res[j][0], &e->G->res[j][1], &bc, &zc);
    if (bc >= zc) e->G->res[j][0] = 0;
    (void)t;
}
static void global_motion(rnv2enc *e, int nchanged) {
    int k, y, x, img, j;
    e->ng = 0;
    if (nchanged < 24) { e->prev_g[0] = e->prev_g[1] = 0; return; } /* nothing moves: no global candidates */
    run_jobs(e, job_proj, 2 * GNBAND);
    for (img = 0; img < 2; img++) {
        for (y = 0; y < H; y++) {
            int32_t f = 0;
            for (k = 0; k < GNSX; k++) { int32_t v = (int32_t)e->G->rows[img][y * GNSX + k]; e->G->prof[2 + k][img][y] = v; f += v; }
            e->G->prof[0][img][y] = f;
        }
        for (x = 0; x < W; x++) {
            int32_t f = 0;
            for (k = 0; k < GNBAND; k++) { e->G->prof[6 + k][img][x] = (int32_t)e->G->cols[img][k][x]; f += (int32_t)e->G->cols[img][k][x]; }
            e->G->prof[1][img][x] = f;
        }
    }
    run_jobs(e, job_gsearch, 9);
    {
        int dyF = e->G->res[0][0], dyF2 = e->G->res[0][1], dxF = e->G->res[1][0], dxF2 = e->G->res[1][1];
        add_g(e, dxF, dyF);
        for (j = 2; j <= 5; j++) add_g(e, 0, e->G->res[j][0]);
        for (j = 6; j <= 8; j++) add_g(e, e->G->res[j][0], 0);
        add_g(e, e->prev_g[0], e->prev_g[1]);
        add_g(e, dxF, 0); add_g(e, 0, dyF);
        add_g(e, dxF2, dyF); add_g(e, dxF, dyF2);
        e->prev_g[0] = dxF; e->prev_g[1] = dyF;
    }
}

/* ------------------------------------------------------------------ phase 3: block motion search (luma SAD) */
#define LAM_MV 4
#define PRUNE_NACT 2500 /* active blocks from which the analysis prunes candidates (--prune); lighter frames always analyse everything */
/* cost of (mx, my), or any value >= lim once it is certain the candidate cannot beat lim (rows 4-7 are not read then) */
static inline uint32_t me_cost(const rnv2enc *e, int x0, int y0, const uint8_t *src, int mx, int my, uint32_t lim) {
    int ix = mx >> 1, fx = mx & 1, iy = my >> 1, fy = my & 1; const uint8_t *ref; uint32_t mc;
    if (!(x0 + ix >= 0 && x0 + ix + 7 + fx <= W - 1 && y0 + iy >= 0 && y0 + iy + 7 + fy <= H - 1)) return 0xFFFFFFFFu;
    KC(1, 1);
    ref = yplane(e, fy * 2 + fx) + (y0 + iy) * W + x0 + ix;
    mc = LAM_MV * ((mx != 0) + (my != 0));
#ifdef __ARM_NEON
    if (e->p.simd) {
        uint16x8_t acc = vabdl_u8(vld1_u8(src), vld1_u8(ref)); uint32_t h; int r;
        acc = vabal_u8(acc, vld1_u8(src + W), vld1_u8(ref + W)); acc = vabal_u8(acc, vld1_u8(src + 2 * W), vld1_u8(ref + 2 * W)); acc = vabal_u8(acc, vld1_u8(src + 3 * W), vld1_u8(ref + 3 * W));
        h = vaddlvq_u16(acc) + mc;
        if (h >= lim) return h;
        for (r = 4; r < 8; r++) acc = vabal_u8(acc, vld1_u8(src + r * W), vld1_u8(ref + r * W));
        return vaddlvq_u16(acc) + mc;
    }
#endif
    return e->k->sad8x8(src, W, ref, W) + mc;
}
#define TRY(MX, MY) do { int mx_ = (MX), my_ = (MY); if (mx_ != bx_ || my_ != by_) { uint32_t c_ = me_cost(e, x0_, y0_, src, mx_, my_, bc); if (c_ < bc) { bc = c_; bx_ = mx_; by_ = my_; } } } while (0)

static void job_me_init(rnv2enc *e, int row) {
    int i, b;
    for (b = row * BW; b < (row + 1) * BW; b++) {
        int x0_ = (b - row * BW) * 8, y0_ = row * 8;
        const uint8_t *src = e->tY + y0_ * W + x0_;
        int bx_ = 0, by_ = 0; uint32_t bc;
        e->medone[b] = 1;
        if (e->forced[b]) { e->mv0[b][0] = e->mv0[b][1] = 0; e->mc0[b] = 0; continue; }
        bc = me_cost(e, x0_, y0_, src, 0, 0, 0xFFFFFFFFu);
        if (bc) {
            int it;
            e->medone[b] = 0;
            TRY(e->prev_mv[b][0], e->prev_mv[b][1]);
            for (i = 0; i < e->ng; i++) TRY(e->g[i][0], e->g[i][1]);
            for (it = 0; it < (e->p.me_level >= 2 ? 8 : 4) && bc; it++) { /* small full-pel descent */
                int cx = bx_, cy = by_; uint32_t c0 = bc;
                TRY(cx + 2, cy); TRY(cx - 2, cy); TRY(cx, cy + 2); TRY(cx, cy - 2);
                if (bc == c0) break;
            }
            if (e->p.me_level >= 2 && bc > (uint32_t)e->p.me_win_thr && bc < (uint32_t)e->p.me_win_max) { /* +-3 full-pel window around the best start (rnv2.py searches +-3 around its global peaks) */
                int cx = bx_, cy = by_, dx, dy;
                if (e->p.me_level >= 3) { for (dy = -6; dy <= 6; dy += 2) for (dx = -6; dx <= 6; dx += 2) TRY(cx + dx, cy + dy); }
                else { /* sparse: rings at 2 and 3 px in 8 directions, then a short descent */
                    static const int8_t R16[16][2] = {{4, 0}, {-4, 0}, {0, 4}, {0, -4}, {4, 4}, {-4, -4}, {4, -4}, {-4, 4},
                                                      {6, 0}, {-6, 0}, {0, 6}, {0, -6}, {6, 6}, {-6, -6}, {6, -6}, {-6, 6}};
                    for (i = 0; i < 16; i++) TRY(cx + R16[i][0], cy + R16[i][1]);
                    for (it = 0; it < 4 && bc && (bx_ != cx || by_ != cy); it++) {
                        int qx = bx_, qy = by_; uint32_t c0 = bc;
                        TRY(qx + 2, qy); TRY(qx - 2, qy); TRY(qx, qy + 2); TRY(qx, qy - 2);
                        if (bc == c0) break;
                    }
                }
            }
        }
        e->mv0[b][0] = (int16_t)bx_; e->mv0[b][1] = (int16_t)by_; e->mc0[b] = bc;
    }
}
static void job_me_pass(rnv2enc *e, int row) { /* Jacobi: reads pass input (mv0/mc0 or mv1/mc1), writes the other */
    static const int8_t P12[12][2] = {{2, 0}, {-2, 0}, {0, 2}, {0, -2}, {2, 2}, {-2, -2}, {2, -2}, {-2, 2}, {4, 0}, {-4, 0}, {0, 4}, {0, -4}};
    static const int8_t P8[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};
    int16_t (*mi)[2] = (e->pass & 1) ? e->mv1 : e->mv0, (*mo)[2] = (e->pass & 1) ? e->mv0 : e->mv1;
    uint32_t *ci = (e->pass & 1) ? e->mc1 : e->mc0, *co = (e->pass & 1) ? e->mc0 : e->mc1;
    int last = e->pass == (e->p.me_level >= 1 ? 1 : 0), b, i;
    const uint8_t *pc = e->mchg[e->pass & 1]; uint8_t *oc = e->mchg[(e->pass & 1) ^ 1];
    for (b = row * BW; b < (row + 1) * BW; b++) {
        int x0_ = (b - row * BW) * 8, y0_ = row * 8;
        const uint8_t *src = e->tY + y0_ * W + x0_;
        int bx_ = mi[b][0], by_ = mi[b][1], bxr = b - row * BW, byr = row; uint32_t bc = ci[b];
        if (!e->medone[b] && bc) {
            /* Jacobi fixed point: if neither this block nor a neighbour changed in the previous pass, the
               neighbour + 12-point step would reproduce the previous result exactly -> skip it */
            int same = e->pass > 0 && !pc[b] && (bxr == 0 || !pc[b - 1]) && (bxr == BW - 1 || !pc[b + 1]) &&
                       (byr == 0 || !pc[b - BW]) && (byr == BH - 1 || !pc[b + BW]);
            if (!same) {
                if (bxr > 0) TRY(mi[b - 1][0], mi[b - 1][1]);
                if (bxr < BW - 1) TRY(mi[b + 1][0], mi[b + 1][1]);
                if (byr > 0) TRY(mi[b - BW][0], mi[b - BW][1]);
                if (byr < BH - 1) TRY(mi[b + BW][0], mi[b + BW][1]);
                /* no usable match nearby (cut / new content): skip the local refinements */
                if (bc < (uint32_t)e->p.me_win_max) { int cx = bx_, cy = by_; for (i = 0; i < 12 && bc; i++) TRY(cx + P12[i][0], cy + P12[i][1]); }
            }
            if (last && bc && bc < (uint32_t)e->p.me_win_max) { int cx = bx_, cy = by_; for (i = 0; i < 8; i++) TRY(cx + P8[i][0], cy + P8[i][1]); }
        }
        oc[b] = (uint8_t)(bx_ != mi[b][0] || by_ != mi[b][1]);
        mo[b][0] = (int16_t)bx_; mo[b][1] = (int16_t)by_; co[b] = bc;
    }
}

/* ------------------------------------------------------------------ phase 4: per-block RD analysis at qp */
static void analyse_block(rnv2enc *e, int b) {
    blkstat *s = &e->bs[b];
    uint8_t S[128], Z[128], P[128], I[128], rec[128]; int16_t T[128];
    const qtab_t *qi, *qa;
    const int16_t (*mv)[2] = e->mv0; /* final mv field */
    int c, k, mx = mv[b][0], my = mv[b][1], sy = 0, su = 0, sv = 0; int64_t Sds = 0, Sdm = 0, Sdi = 0; unsigned skipc = 0;
    s->valid = 0;
    if (e->forced[b]) return;
    s->valid = 1;
    qi = qtab(e, e->cqp, 0); qa = qtab(e, e->cqp, 1);
    gather_blk(e->tY, e->tU, e->tV, b, S);
    gather_blk(e->R[e->rb][0], e->R[e->rb][1], e->R[e->rb][2], b, Z);
    KC(2, 1), e->k->pd8(S, Z, e->kappa16, e->thr, s->ds);
    if (!(s->ds[0] | s->ds[1] | s->ds[2] | s->ds[3] | s->ds[4] | s->ds[5] | s->ds[6] | s->ds[7])) {
        /* reference == target: SKIP has zero distortion and the lowest rate and cycles of every mode, at any (lambda > 0, mu) */
        s->valid = 2; return;
    }
    for (k = 0; k < 64; k++) sy += S[k];
    for (k = 64; k < 96; k++) su += S[k];
    for (k = 96; k < 128; k++) sv += S[k];
    s->fill[0] = (uint8_t)((sy + 32) >> 6); s->fill[1] = (uint8_t)((su + 16) >> 5); s->fill[2] = (uint8_t)((sv + 16) >> 5);
    memset(I, s->fill[0], 64); memset(I + 64, s->fill[1], 32); memset(I + 96, s->fill[2], 32);
    KC(2, 1), e->k->pd8(S, I, e->kappa16, e->thr, s->di);
    s->has_mc = (mx || my);
    if (s->has_mc) { mc_pred(e, b, mx, my, P); KC(2, 1), e->k->pd8(S, P, e->kappa16, e->thr, s->dm); }
    {
        int lx = b > 0 ? mv[b - 1][0] : 0, ly = b > 0 ? mv[b - 1][1] : 0;
        s->mvbits = (uint16_t)(ue_len(se_map(mx - lx)) + ue_len(se_map(my - ly)));
        s->mcc = (uint16_t)mc_cycles(mx, my);
    }
    for (k = 0; k < 8; k++) { Sds += (int64_t)WSB[k] * s->ds[k]; if (s->has_mc) Sdm += (int64_t)WSB[k] * s->dm[k]; }
    s->Jds = Sds / DSCALE; s->Jdm = Sdm / DSCALE;
    if (e->p.prune && e->nact >= PRUNE_NACT) { /* heavy frame: residual candidates whose prediction is far worse than another candidate's rarely win, don't transform them */
        int64_t Smin = s->has_mc && Sdm < Sds ? Sdm : Sds;
        for (k = 0; k < 8; k++) Sdi += (int64_t)WSB[k] * s->di[k];
        if (s->has_mc && Sds > e->p.prune * Sdm) skipc |= 2;  /* ZRES: MC predicts much better */
        if (s->has_mc && Sdm > e->p.prune * Sds) skipc |= 1;  /* MCR: the reference in place predicts much better */
        if (Sdi > 2 * e->p.prune * Smin) skipc |= 4;          /* INTRA: an inter prediction is much better than the flat fill */
    }
    for (c = 0; c < 3; c++) {
        const uint8_t *pr = c == 0 ? P : c == 1 ? Z : I; const uint32_t *doff = c == 0 ? s->dm : c == 1 ? s->ds : s->di; int sb;
        if ((c == 0 && !s->has_mc) || (c == 1 && e->key) || (skipc >> c & 1)) { s->nz[c] = 0; continue; } /* key: a residual on black is a worse INTRA */
        KC(3, 1), e->k->fwd8(S, pr, T);
        s->nz[c] = (uint8_t)quant_bits(e, T, c == 2 ? qa : qi, s->L3[c], s->bits[c], s->cyc[c]);
        if (s->nz[c]) {
            recon_res(e, s->L3[c], s->nz[c], e->step, pr, rec); KC(2, 1), e->k->pd8(S, rec, e->kappa16, e->thr, s->don[c]);
            for (sb = 0; sb < 8; sb++) if (s->nz[c] & (1 << sb)) { s->Don[c][sb] = WSB[sb] * (double)s->don[c][sb] / DSCALE; s->Doff[c][sb] = WSB[sb] * (double)doff[sb] / DSCALE; }
        }
    }
}
static void job_analyse(rnv2enc *e, int row) { int b; for (b = row * BW; b < (row + 1) * BW; b++) analyse_block(e, b); }

/* ------------------------------------------------------------------ phase 5: mode decision at (lambda, mu) */
static void decide_block(rnv2enc *e, int b) {
    const blkstat *s = &e->bs[b]; blkdec *d = &e->bd[b];
    double lam = e->lam, mu = e->cmu, J[6], Dc[3]; int cbp[3], B[3], cy[3], c, sb, m, best;
    int cskip = e->prevmode[b] != SKIP ? CY_SKIPCOPY : CY_SKIP;
    if (s->valid != 1) { d->mode = SKIP; d->cbp = 0; d->cbits = 0; d->cyc = cskip; return; }
    for (c = 0; c < 3; c++) {
        const uint32_t *doff = c == 0 ? s->dm : c == 1 ? s->ds : s->di; int64_t D = 0;
        cbp[c] = 0; B[c] = 0; cy[c] = 0;
        if (c == 0 && !s->has_mc) { Dc[c] = 0; continue; }
        for (sb = 0; sb < 8; sb++) {
            int on = 0;
            if (s->nz[c] & (1 << sb)) on = s->Don[c][sb] + lam * (s->bits[c][sb] + mu * s->cyc[c][sb]) < s->Doff[c][sb];
            if (on) { cbp[c] |= 1 << sb; B[c] += s->bits[c][sb]; cy[c] += s->cyc[c][sb]; D += (int64_t)WSB[sb] * s->don[c][sb]; }
            else D += (int64_t)WSB[sb] * doff[sb];
        }
        Dc[c] = D / DSCALE;
    }
    J[SKIP] = s->Jds + lam * 1.0 + lam * mu * cskip;
    J[MC] = s->has_mc ? s->Jdm + lam * (3 + s->mvbits) + lam * mu * (CY_HDR + s->mcc) : INFINITY;
#define CBPB(x) (CBPL_LEN[(x) & 15] + CBPC_LEN[(x) >> 4])
    J[MCR] = (s->has_mc && cbp[0]) ? Dc[0] + lam * (3 + s->mvbits + B[0] + CBPB(cbp[0])) + lam * mu * (CY_HDR + s->mcc + cy[0]) : INFINITY;
    J[ZRES] = cbp[1] ? Dc[1] + lam * (3 + B[1] + CBPB(cbp[1])) + lam * mu * (CY_HDR + CY_ZRES + cy[1]) : INFINITY;
    J[INTRA] = Dc[2] + lam * (3 + 15 + B[2] + CBPB(cbp[2])) + lam * mu * (CY_HDR + CY_FILL + cy[2]);
    J[RAW] = e->p.use_raw ? lam * (3 + 1024) + lam * mu * (CY_HDR + CY_RAW) : INFINITY;
    best = 0;
    for (m = 1; m < 6; m++) if (J[m] < J[best]) best = m;
    d->mode = (uint8_t)best; d->cbp = 0; d->cbits = 0;
    switch (best) {
    case SKIP: d->cyc = cskip; break;
    case MC: d->cyc = CY_HDR + s->mcc; break;
    case MCR: d->cand = 0; d->cbp = (uint8_t)cbp[0]; d->cbits = B[0]; d->cyc = CY_HDR + s->mcc + cy[0]; break;
    case ZRES: d->cand = 1; d->cbp = (uint8_t)cbp[1]; d->cbits = B[1]; d->cyc = CY_HDR + CY_ZRES + cy[1]; break;
    case INTRA: d->cand = 2; d->cbp = (uint8_t)cbp[2]; d->cbits = B[2]; d->cyc = CY_HDR + CY_FILL + cy[2]; break;
    default: d->cyc = CY_HDR + CY_RAW; break;
    }
}
/* only the active blocks: the decision of a forced block is fixed for the whole frame */
#define DSLICE 128
static void job_decide(rnv2enc *e, int j) { int i, n = (j + 1) * DSLICE; if (n > e->nact) n = e->nact; for (i = j * DSLICE; i < n; i++) decide_block(e, e->act[i]); }

/* exact frame size (bytes incl. 24-byte header) and modelled decode cycles of the current decisions */
static void frame_size(rnv2enc *e, size_t *bytes, double *cyc) {
    int64_t b0 = 0, b1 = 0, b2 = 0, nraw = 0; int run = 0, pmx = 0, pmy = 0, pf[3] = {128, 128, 128}, b, nc = 0; double cy = CY_FRAME;
    for (b = 0; b < NB; b++) {
        const blkdec *d = &e->bd[b]; int m = d->mode;
        cy += d->cyc;
        if (m == SKIP) { run++; continue; }
        b0 += ue_len((uint32_t)run) + (m >= INTRA ? 3 : 2); run = 0; nc++;
        if (m == MCR || m == ZRES || m == INTRA) b0 += CBPB(d->cbp);
        if (m == MC || m == MCR) { int mx = e->mv0[b][0], my = e->mv0[b][1]; b1 += ue_len(se_map(mx - pmx)) + ue_len(se_map(my - pmy)); pmx = mx; pmy = my; }
        if (m == INTRA) { int k; for (k = 0; k < 3; k++) { b1 += ue_len(se_map(e->bs[b].fill[k] - pf[k])); pf[k] = e->bs[b].fill[k]; } }
        if (m == RAW) nraw++;
        b2 += d->cbits;
    }
    if (run) b0 += ue_len((uint32_t)run);
    *bytes = (size_t)(24 + (b0 + 7) / 8 + (b1 + 7) / 8 + (b2 + 7) / 8 + 128 * nraw);
    *cyc = cy; e->ncoded = nc;
}

/* ------------------------------------------------------------------ phase 7: commit (recon + levels) */
static void commit_block(rnv2enc *e, int b) {
    const blkdec *d = &e->bd[b];
    uint8_t P[128], *rec = e->rec[b]; int mx = e->mv0[b][0], my = e->mv0[b][1];
    switch (d->mode) {
    case SKIP: return; /* reference unchanged */
    case MC: mc_pred(e, b, mx, my, rec); break;
    case RAW: gather_blk(e->tY, e->tU, e->tV, b, rec); break;
    default: {
        const blkstat *s = &e->bs[b];
        if (d->mode == MCR) mc_pred(e, b, mx, my, P);
        else if (d->mode == ZRES) gather_blk(e->R[e->rb][0], e->R[e->rb][1], e->R[e->rb][2], b, P);
        else { memset(P, s->fill[0], 64); memset(P + 64, s->fill[1], 32); memset(P + 96, s->fill[2], 32); }
        recon_res(e, s->L3[d->cand], d->cbp, e->step, P, rec);
    } }
}
static void job_commit(rnv2enc *e, int row) { int b; for (b = row * BW; b < (row + 1) * BW; b++) commit_block(e, b); }
/* after all blocks are committed (MC reads the old reference), write the coded blocks into the single reference */
static void job_scatter(rnv2enc *e, int row) {
    int b, dirty = 0, x0 = W, x1 = 0;
    for (b = row * BW; b < (row + 1) * BW; b++) if (e->bd[b].mode != SKIP) {
        int x = (b - row * BW) * 8;
        scatter_blk(e->R[0][0], e->R[0][1], e->R[0][2], b, e->rec[b]); dirty = 1;
        if (x < x0) x0 = x;
        x1 = x + 8;
    }
    e->hdirty[row] = (uint8_t)dirty; e->hx0[row] = (uint16_t)x0; e->hx1[row] = (uint16_t)x1;
}

static void job_halfpel(rnv2enc *e, int idx) { /* idx = 3 components x BH row bands */
    int c = idx / BH, band = idx % BH, w = c ? CW : W, y;
    const uint8_t *P = e->R[e->rb][c]; int xa = W, xb = 0, n;
    if (e->hdirty[band]) { xa = e->hx0[band]; xb = e->hx1[band]; }
    if (band + 1 < BH && e->hdirty[band + 1]) { if (e->hx0[band + 1] < xa) xa = e->hx0[band + 1]; if (e->hx1[band + 1] > xb) xb = e->hx1[band + 1]; }
    if (xa >= xb) return; /* rows 8b..8b+8 unchanged */
    if (c) { xa /= 2; xb /= 2; }
    if (xa > 0) xa--;     /* h[x] also depends on x + 1 */
    n = xb - xa;
    for (y = band * 8; y < band * 8 + 8; y++) {
        const uint8_t *a = P + y * w + xa, *bb = y < H - 1 ? a + w : a; uint8_t *ph = e->HP[c][0] + y * w + xa, *phv = e->HP[c][2] + y * w + xa;
        KC(6, n);
        if (xb < w) { /* the kernel treats its last pixel as the end of the row: run one pixel long, keep that pixel's old values */
            uint8_t sh = ph[n], shv = phv[n];
            e->k->halfpel_row(a, bb, n + 1, ph, e->HP[c][1] + y * w + xa, phv);
            ph[n] = sh; phv[n] = shv;
        } else e->k->halfpel_row(a, bb, n, ph, e->HP[c][1] + y * w + xa, phv);
    }
}

/* serial bitstream writer */
static void write_stream(rnv2enc *e, int qp) {
    int b, run = 0, pmx = 0, pmy = 0, pf[3] = {128, 128, 128};
    static const uint8_t MCODE[6][2] = {{0, 0}, {0, 2}, {1, 2}, {2, 2}, {6, 3}, {7, 3}};
    bw_reset(&e->w0); bw_reset(&e->w1); bw_reset(&e->w2); e->s3n = 0;
    for (b = 0; b < NB; b++) {
        const blkdec *d = &e->bd[b]; int m = d->mode;
        if (m == SKIP) { run++; continue; }
        bw_ue(&e->w0, (uint32_t)run); run = 0;
        bw_put(&e->w0, MCODE[m][0], MCODE[m][1]);
        if (m == MCR || m == ZRES || m == INTRA) {
            bw_put(&e->w0, (uint32_t)CBPL_CODE[d->cbp & 15], CBPL_LEN[d->cbp & 15]);
            bw_put(&e->w0, (uint32_t)CBPC_CODE[d->cbp >> 4], CBPC_LEN[d->cbp >> 4]);
        }
        if (m == MC || m == MCR) {
            int mx = e->mv0[b][0], my = e->mv0[b][1];
            bw_ue(&e->w1, se_map(mx - pmx)); bw_ue(&e->w1, se_map(my - pmy)); pmx = mx; pmy = my;
        }
        if (m == INTRA) { int k; for (k = 0; k < 3; k++) { bw_ue(&e->w1, se_map(e->bs[b].fill[k] - pf[k])); pf[k] = e->bs[b].fill[k]; } }
        if (m == RAW) {
            uint8_t S[128]; int r, x;
            if (e->s3n + 128 > e->s3cap) { size_t nc = e->s3cap * 2; uint8_t *nb = xmalloc(nc); memcpy(nb, e->s3, e->s3n); free(e->s3); e->s3 = nb; e->s3cap = nc; }
            gather_blk(e->tY, e->tU, e->tV, b, S);
            for (r = 0; r < 8; r++) {
                const uint8_t *yr = S + ((r >> 2) * 2) * 16 + (r & 3) * 4, *ur = S + 64 + (r >> 2) * 16 + (r & 3) * 4, *vr = S + 96 + (r >> 2) * 16 + (r & 3) * 4;
                uint8_t *o = e->s3 + e->s3n + 16 * r;
                for (x = 0; x < 4; x++) {
                    int ly = 2 * x; /* luma column 2x, 2x+1 */
                    o[4 * x + 0] = ly < 4 ? yr[ly] : yr[16 + ly - 4];
                    o[4 * x + 1] = ur[x];
                    o[4 * x + 2] = ly + 1 < 4 ? yr[ly + 1] : yr[16 + ly + 1 - 4];
                    o[4 * x + 3] = vr[x];
                }
            }
            e->s3n += 128;
        }
        if (d->cbp) {
            int sb, pos;
            for (sb = 0; sb < 8; sb++) if (d->cbp & (1 << sb)) {
                const int16_t *l = e->bs[b].L3[d->cand] + 16 * sb; int lastpos = -1, prev = -1;
                for (pos = 0; pos < 16; pos++) if (l[ZZ[pos]]) lastpos = pos;
                for (pos = 0; pos <= lastpos; pos++) {
                    int v = l[ZZ[pos]], a, sym;
                    if (!v) continue;
                    a = v < 0 ? -v : v;
                    sym = (pos == lastpos) * (RUNMAX * LVMAX) + (pos - prev - 1) * LVMAX + (a < LVMAX ? a : LVMAX) - 1; prev = pos;
                    bw_put(&e->w2, (uint32_t)COEF_CODE[sym], COEF_LEN[sym]);
                    if (a >= LVMAX) bw_ue(&e->w2, (uint32_t)(a - LVMAX));
                    bw_put(&e->w2, v < 0, 1);
                }
            }
        }
    }
    if (run) bw_ue(&e->w0, (uint32_t)run);
    bw_flush(&e->w0); bw_flush(&e->w1); bw_flush(&e->w2);
    {
        size_t n = 24 + e->w0.n + e->w1.n + e->w2.n + e->s3n; uint8_t *o; size_t l[4]; int i;
        if (n > e->ocap) { free(e->obuf); e->ocap = n * 2; e->obuf = xmalloc(e->ocap); }
        o = e->obuf; l[0] = e->w0.n; l[1] = e->w1.n; l[2] = e->w2.n; l[3] = e->s3n;
        o[0] = 'R'; o[1] = '2'; o[2] = (uint8_t)((e->fn >> 8) & 0xFF); o[3] = (uint8_t)(e->fn & 0xFF); o[4] = (uint8_t)qp; o[5] = 1; o[6] = o[7] = 0;
        for (i = 0; i < 4; i++) { o[8 + 4 * i] = (uint8_t)(l[i] >> 24); o[9 + 4 * i] = (uint8_t)(l[i] >> 16); o[10 + 4 * i] = (uint8_t)(l[i] >> 8); o[11 + 4 * i] = (uint8_t)l[i]; }
        o += 24;
        memcpy(o, e->w0.buf, l[0]); o += l[0]; memcpy(o, e->w1.buf, l[1]); o += l[1]; memcpy(o, e->w2.buf, l[2]); o += l[2]; memcpy(o, e->s3, l[3]);
    }
}

static void job_post(rnv2enc *e, int idx) { if (idx == 0) write_stream(e, e->wqp); else job_halfpel(e, idx - 1); }

/* ------------------------------------------------------------------ canonical codes (rnv.canon_codes) */
static void canon(const unsigned char *L, int n, uint32_t *code) {
    int order[416], i, j; uint32_t c = 0; int prev = 0;
    for (i = 0; i < n; i++) order[i] = i;
    for (i = 1; i < n; i++) { int v = order[i]; j = i - 1; while (j >= 0 && (L[order[j]] > L[v] || (L[order[j]] == L[v] && order[j] > v))) { order[j + 1] = order[j]; j--; } order[j + 1] = v; }
    for (i = 0; i < n; i++) { int s = order[i]; if (i) { c = (c + 1) << (L[s] - prev); } prev = L[s]; code[s] = c; }
}

/* ------------------------------------------------------------------ public API */
void rnv2enc_default_params(rnv2enc_params *p) {
    memset(p, 0, sizeof *p);
    p->qp_min = 0; p->qp_max = 51; p->rate = 8e6; p->buf_frames = 4; p->fps = 60; p->cyc_budget = 4.49e6; p->lam_k = 0.04;
    p->kappa = 4; p->thr = 3; p->dz_inter = 0.22; p->dz_intra = 0.33; p->mu_min = 0.002; p->mu_max = 2.0; p->use_raw = 1;
    p->link_cap = 0; p->adapt = 1; p->nlevels = 12; p->lam_lo = 0.0025; p->lam_x = 0.01; p->kappa_lo = 16; p->thr_lo = 0; p->fill_target = 0.25; p->hyst = 0.85; p->dwell = 4; p->pin_level = -1;
    p->max_tries = 12; p->threads = 1; p->static_skip = 1; p->simd = 1; p->me_level = 2; p->me_win_thr = 0; p->me_win_max = 1024; p->rt_rc = 1; p->prune = 3; p->qp_pred = 1;
}

static void set_level(rnv2enc *e, int L);
/* Next frame is a key: coded from black, exactly like frame 0, so a decoder that lost frames can rejoin. */
void rnv2enc_force_key(rnv2enc *e) {
    int i;
    memset(e->R[0][0], 0, W * H); memset(e->R[0][1], 128, CW * H); memset(e->R[0][2], 128, CW * H);
    for (i = 0; i < 3; i++) { memset(e->HP[0][i], 0, W * H); memset(e->HP[1][i], 128, CW * H); memset(e->HP[2][i], 128, CW * H); } /* half-pel planes of black: the old ones must not survive the key */
    e->rb = 0; e->first = 1; memset(e->prevmode, INTRA, NB); memset(e->prev_mv, 0, sizeof e->prev_mv);
}

rnv2enc *rnv2enc_create(const rnv2enc_params *p) {
    rnv2enc *e = calloc(1, sizeof *e); int i, c;
    static int inited;
    if (!e) return NULL;
    if (!inited) { rnv2_csc_init(); canon(COEF_LEN, 416, COEF_CODE); canon(CBPL_LEN, 16, CBPL_CODE); canon(CBPC_LEN, 16, CBPC_CODE); inited = 1; }
    e->p = *p;
    e->k = &rnv2_kern_scalar;
#ifdef RNV2_HAVE_NEON
    if (p->simd) e->k = &rnv2_kern_neon;
#endif
    e->nlev = p->adapt && p->nlevels > 1 ? (p->nlevels > 64 ? 64 : p->nlevels) : 1;
    if (p->link_cap > 0) e->p.rate = p->link_cap;
    e->lvl = e->alvl = p->pin_level >= 0 && p->pin_level < e->nlev ? p->pin_level : e->nlev - 1;
    { double kmax = p->kappa > p->kappa_lo ? p->kappa : p->kappa_lo, tmin = p->thr < p->thr_lo ? p->thr : p->thr_lo; int tmax = (int)lrint(p->thr > p->thr_lo ? p->thr : p->thr_lo);
      if (e->nlev == 1) { kmax = p->kappa; tmin = p->thr; tmax = (int)lrint(p->thr); }
      if (lrint(kmax * 16) > 256 || lrint(kmax * 16) < 0 || lrint(tmin) < 0 || tmax > 255) e->k = &rnv2_kern_scalar; } /* NEON pd8 range limit */
    set_level(e, e->lvl);
    atomic_init(&e->pend_cap, 0.0);
    e->tY = xmalloc(W * H); e->tU = xmalloc(CW * H); e->tV = xmalloc(CW * H);
    e->refrgb = xmalloc(W * H * 3);
    for (i = 0; i < 2; i++) { e->R[i][0] = xmalloc(W * H); e->R[i][1] = xmalloc(CW * H); e->R[i][2] = xmalloc(CW * H); }
    for (c = 0; c < 3; c++) for (i = 0; i < 3; i++) e->HP[c][i] = xmalloc(c ? CW * H : W * H);
    memset(e->R[0][0], 0, W * H); memset(e->R[0][1], 128, CW * H); memset(e->R[0][2], 128, CW * H);
    e->rb = 0; e->first = 1; memset(e->hdirty, 1, BH);
    for (i = 0; i < BH; i++) { e->hx0[i] = 0; e->hx1[i] = W; }
    e->G = xmalloc(sizeof(gme_t));
    for (i = 0; i < 52; i++) { qtab(e, i, 0); qtab(e, i, 1); } /* build all quantiser tables up front (no first-use stalls) */
    e->rec = xmalloc(128 * NB);
    e->bs = xmalloc(sizeof(blkstat) * NB); e->bs2 = xmalloc(sizeof(blkstat) * NB); e->bs2_cls = -1; e->bd = xmalloc(sizeof(blkdec) * NB);
    memset(e->prevmode, INTRA, NB); memset(e->prev_mv, 0, sizeof e->prev_mv);
    e->rate_pf = e->p.rate / p->fps; e->buf = p->buf_frames * e->p.rate / p->fps; e->fill = 0; e->qp = p->qp_min; e->mu = p->mu_min;
    e->w2.cap = 0; e->s3cap = 1 << 20; e->s3 = xmalloc(e->s3cap); e->ocap = 1 << 21; e->obuf = xmalloc(e->ocap);
    run_jobs(e, job_halfpel, 3 * BH);
    e->nth = p->threads < 1 ? 1 : p->threads > 16 ? 16 : p->threads;
    pthread_mutex_init(&e->m, NULL); pthread_cond_init(&e->cv, NULL); pthread_cond_init(&e->cvdone, NULL);
    for (i = 1; i < e->nth; i++) pthread_create(&e->th[i], NULL, worker, e);
    return e;
}

void rnv2enc_destroy(rnv2enc *e) {
    int i, c;
    if (!e) return;
    pthread_mutex_lock(&e->m); e->quit = 1; pthread_cond_broadcast(&e->cv); pthread_mutex_unlock(&e->m);
    for (i = 1; i < e->nth; i++) pthread_join(e->th[i], NULL);
    free(e->tY); free(e->tU); free(e->tV); free(e->refrgb);
    for (i = 0; i < 2; i++) for (c = 0; c < 3; c++) free(e->R[i][c]);
    for (c = 0; c < 3; c++) for (i = 0; i < 3; i++) free(e->HP[c][i]);
    for (i = 0; i < 52; i++) { free(e->qt[i][0]); free(e->qt[i][1]); }
    free(e->G); free(e->rec); free(e->bs); free(e->bs2); free(e->bd); free(e->w0.buf); free(e->w1.buf); free(e->w2.buf); free(e->s3); free(e->obuf);
    free(e);
}

/* operating point of ladder level L.  nlev-1 = (lam_k, kappa, thr) = the classic point.  Levels 1..nlev-2 keep that kappa / thr and
 * lower only lambda, geometrically down to lam_lo (measured on the night map: 2.4 -> 7.6 MB/s, glass-vs-target error 3.2% -> 0.2%, a
 * smooth ladder; thr is a cliff, thr 3 -> 0 doubles the rate).  Level 0 = (lam_x, kappa_lo, thr_lo): thr 0 near-exact. */
static void set_level(rnv2enc *e, int L) {
    const rnv2enc_params *p = &e->p; int N = e->nlev;
    double lo = p->lam_lo > 0 && p->lam_lo < p->lam_k ? p->lam_lo : p->lam_k;
    if (L >= N - 1 || (N < 3 && L > 0)) { e->lamk = p->lam_k; e->kappa16 = (int)lrint(p->kappa * 16); e->thr = (int)lrint(p->thr); return; }
    if (L == 0) {
        e->lamk = p->lam_x < p->lam_k ? p->lam_x : p->lam_k;
        e->kappa16 = (int)lrint((p->kappa_lo > p->kappa ? p->kappa_lo : p->kappa) * 16); e->thr = (int)lrint(p->thr_lo < p->thr ? p->thr_lo : p->thr); return;
    }
    e->lamk = lo * pow(p->lam_k / lo, (double)(L - 1) / (N - 2));
    e->kappa16 = (int)lrint(p->kappa * 16); e->thr = (int)lrint(p->thr);
}
/* lambda of ladder level L >= 1 (levels >= 1 share kappa / thr, hence the cached block analysis: only the decision is redone) */
static void set_lam(rnv2enc *e, int L) {
    double st = QSTEP2[e->cqp];
    set_level(e, L);
    e->lam = e->lamk * (st > 2 ? st : 2) * (st > 2 ? st : 2);
}
static void set_cap(rnv2enc *e, double cap) {
    e->p.rate = cap; e->rate_pf = cap / e->p.fps; e->buf = e->p.buf_frames * cap / e->p.fps;
    if (e->fill > e->buf) e->fill = e->buf; /* the streamer owns whatever was already queued beyond the new bucket */
}
void rnv2enc_set_linkcap(rnv2enc *e, double bytes_per_s) { if (bytes_per_s > 0) atomic_store(&e->pend_cap, bytes_per_s); }
static void set_qp(rnv2enc *e, int qp) {
    double st = QSTEP2[qp];
    e->cqp = qp; e->step = QSTEP2[qp]; e->lam = e->lamk * (st > 2 ? st : 2) * (st > 2 ? st : 2);
    qtab(e, qp, 0); qtab(e, qp, 1);
}

int rnv2enc_encode(rnv2enc *e, const uint8_t *pix, int stride, enum rnv2_fmt fmt, const rnv2_rect *dmg, int ndmg,
                   const uint8_t **out, size_t *outlen, rnv2enc_info *info) {
    double t0 = now_ms(), t1, t2, t3, ta = 0, td = 0, t5, t6, cap, soft, mu, cyc = 0; size_t n = 0; int qp, tries = 0, aqp = -1, b, i, nana = 0, rtok = 0, ad, Lprev, acls = -1, want0;
    double alw = 0; int hint = -1, evs = 0, keytry0 = 0;
    e->bs2_cls = -1;
    if (info) memset(info, 0, sizeof *info);
    { double c = atomic_exchange(&e->pend_cap, 0.0); if (c > 0 && c != e->p.rate) set_cap(e, c); }
    e->cur = pix; e->curstride = stride; e->fmt = fmt;
    if (ndmg < 0) memset(e->damaged, 1, NB);
    else {
        memset(e->damaged, 0, NB);
        for (i = 0; i < ndmg; i++) {
            int x0 = dmg[i].x < 0 ? 0 : dmg[i].x, y0 = dmg[i].y < 0 ? 0 : dmg[i].y, x1 = dmg[i].x + dmg[i].w, y1 = dmg[i].y + dmg[i].h, bx, by;
            if (x1 > W) x1 = W; if (y1 > H) y1 = H;
            for (by = y0 / 8; by * 8 < y1; by++) for (bx = x0 / 8; bx * 8 < x1; bx++) e->damaged[by * BW + bx] = 1;
        }
    }
    e->qp0 = (e->p.rt_rc && e->qp_raised) ? e->qp : e->qp - 2; /* rt_rc: hold qp after a frame that had to raise it */
    if (e->qp0 < e->p.qp_min) e->qp0 = e->p.qp_min;
    e->mu0 = e->mu * 0.6 > e->p.mu_min ? e->mu * 0.6 : e->p.mu_min;
#ifdef RNV_EXACT_TEST /* test build: no key-frame shortcuts (they are the only intended bitstream change on a stream without forced keys) */
    e->key = 0;
#else
    e->key = (uint8_t)e->first;
#endif
    { int nd = NB; if (ndmg >= 0 && !e->first) { nd = 0; for (b = 0; b < NB; b++) nd += e->damaged[b]; } run_w(e, job_csc, BH, nd, 128); }
    e->first = 0; e->tvalid = 1;
    /* blocks that need any work this frame; the decision of the others is SKIP for the whole frame */
    e->nprev_act = e->nact; e->nact = 0;
    for (b = 0; b < NB; b++) {
        if (!e->forced[b]) e->act[e->nact++] = (uint16_t)b;
        else { blkdec *d = &e->bd[b]; d->mode = SKIP; d->cbp = 0; d->cbits = 0; d->cyc = e->prevmode[b] != SKIP ? CY_SKIPCOPY : CY_SKIP; }
    }
    t1 = now_ms();
    if (e->key) { /* the reference is black: nothing to search */
        e->ng = 0; e->prev_g[0] = e->prev_g[1] = 0; t2 = now_ms();
        memset(e->mv0, 0, sizeof e->mv0); memset(e->mc0, 0, sizeof e->mc0); memset(e->medone, 1, NB);
    } else {
        { int nch = 0; for (b = 0; b < NB; b++) nch += e->changed[b]; global_motion(e, nch); }
        t2 = now_ms();
        run_w(e, job_me_init, BH, e->nact, 64);
        for (e->pass = 0; e->pass <= (e->p.me_level >= 1 ? 1 : 0); e->pass++) run_w(e, job_me_pass, BH, e->nact, 64);
        if (e->pass & 1) memcpy(e->mv0, e->mv1, sizeof e->mv0); /* final field in mv0 */
    }
    t3 = now_ms();
    e->n_forced = NB - e->nact; e->n_search = 0;
    for (b = 0; b < NB; b++) e->n_search += !e->medone[b];
    /* rnv2.py Encoder2.encode rate / cycle control */
    cap = e->buf + e->rate_pf - e->fill;
    soft = e->rate_pf + (e->buf - e->fill > 0 ? e->buf - e->fill : 0) * 0.35;
    qp = e->qp0; mu = e->mu0; /* = max(qp_min, qp - 2), max(mu_min, 0.6 mu) as rnv2.py (rt_rc: see rnv2enc.h) */
    ad = e->nlev > 2 && e->p.pin_level < 0; /* headroom-driven level selection active */
    if (e->p.qp_pred && ad) { /* the first qp comes from the size of the last comparable frame: an analysis pass that is sure to overrun the hard cap is the costliest retry */
        double est = e->key ? e->pkey : e->pbpa * e->nact; int pq = e->key ? e->pkq : e->pq;
        if (est > 0) {
            double eq = est * pow(2.0, -(qp - pq) / 7.5);
            if (eq > 1.15 * cap) { qp += (int)ceil(7.5 * log2(eq / cap)); if (qp > e->p.qp_max) qp = e->p.qp_max; } /* clearly over: anything near the cap is left to the normal path */
        }
    }
    Lprev = e->lvl; want0 = ad && Lprev == 0 && (!e->key || e->kl0 != 2); if (e->l0cool > 0) e->l0cool--; /* a key that did not fit near-exact last time: no wasted analysis pass */
    keytry0 = e->key && want0;
    alw = e->rate_pf + (e->p.fill_target * e->buf - e->fill) * 0.35; /* per-frame allowance for choosing the level: keeps (1-fill_target) of the bucket free for bursts */
    if (alw < 0.25 * e->rate_pf) alw = 0.25 * e->rate_pf;
/* the previous analysis stays in bs2: going back to the class analysed just before (failed level-0 probe) needs no new pass */
#define ANALYSE(LV) do { double t_ = now_ms(); int cl_ = (LV) ? 1 : 0; blkstat *sw_ = e->bs; e->lvl = (LV); set_level(e, e->lvl); set_qp(e, qp); e->bs = e->bs2; e->bs2 = sw_; { int c2_ = e->bs2_cls, q2_ = e->bs2_qp; e->bs2_cls = acls; e->bs2_qp = aqp; if (!(c2_ == cl_ && q2_ == qp)) { run_w(e, job_analyse, BH, e->nact, 64); nana++; } } aqp = qp; acls = cl_; ta += now_ms() - t_; } while (0)
#define DECIDE() run_w(e, job_decide, (e->nact + DSLICE - 1) / DSLICE, e->nact, 256)
#define EVAL(LV) do { e->lvl = (LV); set_lam(e, e->lvl); DECIDE(); frame_size(e, &n, &cyc); evs++; } while (0)
#define FITS(LV) ((double)n <= alw * ((LV) < Lprev ? e->p.hyst : 1.0)) /* bytes only: a decode-cycle overrun is fixed by the mu loop below */
    for (;;) {
        double tt = now_ms();
        if (!ad) {
            if (qp != aqp) { ANALYSE(e->lvl); tt = now_ms(); }
            e->cmu = mu;
            DECIDE();
            frame_size(e, &n, &cyc);
        } else {
            int lo = Lprev > 1 ? (e->since >= e->p.dwell ? Lprev - 1 : Lprev) : 1, hi = e->nlev - 1, a, bb, done = 0;
            e->cmu = mu;
            if (want0) { /* previous frame was near-exact: try to stay there */
                if (qp != aqp || acls != 0) { ANALYSE(0); tt = now_ms(); }
                EVAL(0);
                if (FITS(0)) done = 1; else { want0 = 0; e->l0cool = 30; }
            }
            if (!done) {
                int last = -1;
                if (qp != aqp || acls != 1) { ANALYSE(1); tt = now_ms(); }
                /* smallest level whose frame fits the allowance (monotone in lambda); none fits -> coarsest = classic point.
                   Search outward from the likely answer (this frame's previous try, else the lowest allowed level, else coarsest
                   for a burst) instead of bisecting: a retry or a steady frame usually needs 1-2 passes, not 4. */
                a = lo; bb = hi;
                if (a < bb) {
                    int m = hint >= 0 ? hint : (e->key || e->nact > 2 * e->nprev_act + 256) ? hi : lo, st = 1;
                    if (m < lo) m = lo;
                    if (m > hi) m = hi;
                    EVAL(m); last = m;
                    if (FITS(m)) { bb = m; while (a < bb) { int q = bb - st; if (q < a) q = a; EVAL(q); last = q; if (FITS(q)) { bb = q; st *= 2; } else { a = q + 1; break; } } }
                    else { a = m < bb ? m + 1 : bb; while (a < bb) { int q = a + st - 1; if (q >= bb) break; EVAL(q); last = q; if (FITS(q)) { bb = q; break; } a = q + 1; st *= 2; } }
                    while (a < bb) { int q = (a + bb) / 2; EVAL(q); last = q; if (FITS(q)) bb = q; else a = q + 1; }
                }
                if (last != a) EVAL(a);
                hint = a;
                if (a == 1 && Lprev <= 1 && e->l0cool == 0 && (Lprev == 0 || e->since >= e->p.dwell) && (double)n <= 0.4 * alw) { /* lots of room at the finest lambda: try the near-exact point */
                    ANALYSE(0); tt = now_ms(); EVAL(0);
                    if (!FITS(0)) { e->l0cool = 30; ANALYSE(1); tt = now_ms(); EVAL(1); }
                }
            }
        }
        td += now_ms() - tt;
        tries++;
        /* rt_rc: after rt_rc analysis passes accept anything within the hard bucket cap (no more qp re-analysis) */
        rtok = e->p.rt_rc && nana >= e->p.rt_rc && (double)n <= cap;
        if (((n <= soft || rtok) && cyc <= e->p.cyc_budget) || (qp >= e->p.qp_max && mu >= e->p.mu_max) || tries >= e->p.max_tries) break;
        if (cyc > e->p.cyc_budget && mu < e->p.mu_max) { double r = pow(cyc / e->p.cyc_budget, 3); mu = mu * (r > 2 ? r : 2); if (mu > e->p.mu_max) mu = e->p.mu_max; }
        if ((double)n > soft && !rtok) { int dq = (int)ceil(6 * log2(n / soft) * 0.8); qp += dq > 1 ? dq : 1; if (qp > e->p.qp_max) qp = e->p.qp_max; }
        else if (cyc > e->p.cyc_budget && mu >= e->p.mu_max) { qp += 2; if (qp > e->p.qp_max) qp = e->p.qp_max; }
    }
    e->alvl = e->lvl; if (ad) e->since = e->lvl != Lprev ? 0 : e->since + 1;
    if (e->key) {
        if (e->lvl == e->nlev - 1) { e->pkey = (double)n; e->pkq = qp; } else e->pkey = 0;
        if (keytry0) e->kl0 = e->lvl == 0 ? 1 : 2; else if (e->lvl <= 1) e->kl0 = 0; /* finest lambda fitted: worth a near-exact try next time */
    }
    else if (e->nact >= 128) { if (e->lvl == e->nlev - 1) { e->pbpa = (double)n / e->nact; e->pq = qp; } else e->pbpa = 0; }
    e->mu = mu;
    if ((double)n > cap) fprintf(stderr, "rnv2enc: frame %d exceeds the bucket hard cap (%zu > %.0f)\n", e->fn, n, cap);
    e->fill = e->fill + n - e->rate_pf; if (e->fill < 0) e->fill = 0;
    e->qp_raised = qp > e->qp0;
    e->qp = qp;
    if (rtok && (double)n > soft) { /* rt_rc accepted an oversize frame: raise qp for the next frame instead */
        int dq = (int)ceil(6 * log2(n / soft) * 0.8);
        e->qp = qp + (dq > 1 ? dq : 1); if (e->qp > e->p.qp_max) e->qp = e->p.qp_max;
        e->qp_raised = 1;
    }
    t5 = now_ms();
    run_w(e, job_commit, BH, e->ncoded, 64);
    run_w(e, job_scatter, BH, e->ncoded, 256);
    e->wqp = qp;
    run_w(e, job_post, 1 + 3 * BH, e->ncoded, 128); /* serial bitstream write overlapped with the half-pel planes of the new reference */
    t6 = now_ms();
    for (b = 0; b < NB; b++) {
        int m = e->bd[b].mode;
        e->prevmode[b] = (uint8_t)m;
        if (m == SKIP && !e->forced[b]) { e->skq[b] = (uint8_t)qp; e->skmu[b] = mu; e->skl[b] = (uint8_t)e->alvl; }
        if (m == SKIP) e->prev_mv[b][0] = e->prev_mv[b][1] = 0; else { e->prev_mv[b][0] = e->mv0[b][0]; e->prev_mv[b][1] = e->mv0[b][1]; }
        if (info) info->modes[m]++;
    }
    *out = e->obuf; *outlen = 24 + e->w0.n + e->w1.n + e->w2.n + e->s3n;
    if (*outlen != n) fprintf(stderr, "rnv2enc: internal size mismatch %zu vs %zu\n", *outlen, n);
    if (info) {
        info->bytes = (int)*outlen; info->qp = qp; info->level = e->alvl; info->link_cap = e->p.rate; info->lamk = e->lamk; info->tries = tries; info->cycles = cyc; info->mu = mu; info->fill = e->fill;
        info->n_skip_forced = e->n_forced; info->n_searched = e->n_search;
        info->sizes[0] = (int)e->w0.n; info->sizes[1] = (int)e->w1.n; info->sizes[2] = (int)e->w2.n; info->sizes[3] = (int)e->s3n;
        info->gmv_n = e->ng; for (i = 0; i < e->ng; i++) { info->gmv[i][0] = e->g[i][0]; info->gmv[i][1] = e->g[i][1]; }
        info->t_ms[0] = t1 - t0; info->t_ms[1] = t2 - t1; info->t_ms[2] = t3 - t2; info->t_ms[3] = ta; info->t_ms[4] = td;
        info->t_ms[5] = evs + 100 * nana; info->t_ms[6] = t6 - t5; info->t_ms[7] = now_ms() - t0;
    }
    e->fn++;
    return 0;
}

static void pack_yuy2(const uint8_t *Y, const uint8_t *U, const uint8_t *V, uint8_t *d) {
    int i;
    for (i = 0; i < W * H / 2; i++) { d[4 * i] = Y[2 * i]; d[4 * i + 1] = U[i]; d[4 * i + 2] = Y[2 * i + 1]; d[4 * i + 3] = V[i]; }
}
void rnv2enc_recon_yuy2(const rnv2enc *e, uint8_t *dst) { pack_yuy2(e->R[e->rb][0], e->R[e->rb][1], e->R[e->rb][2], dst); }
void rnv2enc_target_yuy2(const rnv2enc *e, uint8_t *dst) { pack_yuy2(e->tY, e->tU, e->tV, dst); }
void rnv2enc_planes(const rnv2enc *e, const uint8_t **rY, const uint8_t **rU, const uint8_t **rV,
                    const uint8_t **tY, const uint8_t **tU, const uint8_t **tV) {
    *rY = e->R[e->rb][0]; *rU = e->R[e->rb][1]; *rV = e->R[e->rb][2]; *tY = e->tY; *tU = e->tU; *tV = e->tV;
}
const char *rnv2enc_kernel_name(const rnv2enc *e) { return e->k->name; }
