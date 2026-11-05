/* make test, part 1: every NEON kernel must equal the scalar reference bit for bit (random + edge inputs).
 * Built with real NEON on aarch64, or with src/neon_emu.h on x86 (make emutest). */
#include "kern.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef RNV2_HAVE_NEON
static uint32_t rs = 12345;
static uint32_t rnd(void) { rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5; return rs; }
static uint8_t r8(int mode) { /* mix of uniform, extremes and near-equal values */
    uint32_t v = rnd();
    switch (mode % 4) { case 0: return (uint8_t)v; case 1: return (v & 1) ? 255 : 0; case 2: return (uint8_t)(120 + (v % 16)); default: return (uint8_t)((v & 3) ? (v >> 8) % 8 : 248 + (v >> 8) % 8); }
}
static int fails;
#endif
#define CHECK(cond, ...) do { if (!(cond)) { if (fails++ < 20) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } } while (0)

int main(void) {
#ifndef RNV2_HAVE_NEON
    printf("test_kernels: no NEON in this build (x86 without -DRNV_NEON_EMU): nothing to compare. Use `make emutest`.\n");
    return 0;
#else
    const rnv2_kern *S = &rnv2_kern_scalar, *N = &rnv2_kern_neon;
    int it, i;
    rnv2_csc_init();
    printf("test_kernels: %s vs %s\n", S->name, N->name);
    /* CSC: random runs of 1..40 pairs (exercises the tail path), plus a sweep of structured colours */
    for (it = 0; it < 200000; it++) {
        uint8_t rgb[240], y1[80], u1[40], v1[40], y2[80], u2[40], v2[40]; int np = 1 + rnd() % 40, m = rnd();
        for (i = 0; i < np * 6; i++) rgb[i] = r8(m + (i / 3));
        S->csc_glass(rgb, np, y1, u1, v1); N->csc_glass(rgb, np, y2, u2, v2);
        CHECK(!memcmp(y1, y2, 2 * np) && !memcmp(u1, u2, np) && !memcmp(v1, v2, np), "csc_glass iter %d np %d", it, np);
    }
    { /* every (r,g,b) on a 17-step grid against every grid colour as the pair partner */
        static uint8_t rgb[4913 * 6]; static uint8_t y1[4913 * 2], u1[4913], v1[4913], y2[4913 * 2], u2[4913], v2[4913]; int a, b, c, p;
        for (p = 0; p < 4913; p += 97) {
            int k = 0;
            for (a = 0; a < 17; a++) for (b = 0; b < 17; b++) for (c = 0; c < 17; c++, k++) {
                rgb[6 * k] = (uint8_t)(a * 16 - (a == 16)); rgb[6 * k + 1] = (uint8_t)(b * 16 - (b == 16)); rgb[6 * k + 2] = (uint8_t)(c * 16 - (c == 16));
                rgb[6 * k + 3] = (uint8_t)((p % 17) * 15); rgb[6 * k + 4] = (uint8_t)((p / 17 % 17) * 15); rgb[6 * k + 5] = (uint8_t)((p / 289) * 15);
            }
            S->csc_glass(rgb, 4913, y1, u1, v1); N->csc_glass(rgb, 4913, y2, u2, v2);
            CHECK(!memcmp(y1, y2, sizeof y1) && !memcmp(u1, u2, sizeof u1) && !memcmp(v1, v2, sizeof v1), "csc_glass grid partner %d", p);
        }
    }
    printf("  csc_glass     done\n");
    for (it = 0; it < 200000; it++) { /* SAD, pd8, fwd8, addclamp, iwht */
        uint8_t a[16 * 24], b[16 * 24], s[128], r[128], rec1[128], rec2[128]; int m = rnd(), as = 8 + rnd() % 16, bs = 8 + rnd() % 16;
        uint32_t o1[8], o2[8]; int16_t T1[128], T2[128]; int32_t R[128], c[16], w1[16], w2[16]; int mask = rnd() & 255, k16 = (int)(rnd() % 257), thr = (int)(rnd() % 8);
        for (i = 0; i < (int)sizeof a; i++) { a[i] = r8(m + i / 7); b[i] = r8(m + i / 5); }
        for (i = 0; i < 128; i++) { s[i] = r8(m + i / 16); r[i] = r8(m + 1 + i / 9); }
        CHECK(S->sad8x8(a, as, b, bs) == N->sad8x8(a, as, b, bs), "sad8x8 %d", it);
        S->pd8(s, r, k16, thr, o1); N->pd8(s, r, k16, thr, o2); CHECK(!memcmp(o1, o2, sizeof o1), "pd8 %d k16 %d thr %d", it, k16, thr);
        S->fwd8(s, r, T1); N->fwd8(s, r, T2); CHECK(!memcmp(T1, T2, sizeof T1), "fwd8 %d", it);
        for (i = 0; i < 128; i++) { int sc = (rnd() & 7) == 0 ? 40000 : (rnd() & 1) ? 600 : 40; R[i] = (int32_t)(rnd() % (2 * sc + 1)) - sc; }
        S->addclamp(s, R, mask, rec1); N->addclamp(s, R, mask, rec2); CHECK(!memcmp(rec1, rec2, 128), "addclamp %d", it);
        for (i = 0; i < 16; i++) { int sc = (rnd() & 3) ? 2047 * ((rnd() & 1) ? 724 : 1) : 300; c[i] = (rnd() & 1) ? 0 : (int32_t)(rnd() % (2 * sc + 1)) - sc; }
        { int16_t Tq[128], L1[128], L2[128]; int sh = 17 + (int)(rnd() % 10); uint32_t A = (rnd() & 1) ? 16384 + rnd() % 16385 : 1u << (14 + (int)(rnd() % 2)), B = rnd() % (1u << (sh - 1)); int m1, m2;
          for (i = 0; i < 128; i++) Tq[i] = (int16_t)((int)(rnd() % 8161) - 4080);
          if (rnd() & 1) for (i = 0; i < 128; i++) Tq[i] = (int16_t)(Tq[i] / 64);
          m1 = S->quant8(Tq, A, B, sh, L1); m2 = N->quant8(Tq, A, B, sh, L2);
          CHECK(m1 == m2 && !memcmp(L1, L2, sizeof L1), "quant8 %d A %u B %u S %d", it, A, B, sh); }
        S->iwht(c, w1); N->iwht(c, w2); CHECK(!memcmp(w1, w2, sizeof w1), "iwht %d", it);
    }
    printf("  sad/pd8/fwd8/quant8/addclamp/iwht done\n");
    for (it = 0; it < 300; it++) { /* half-pel rows and projections on random widths */
        static uint8_t a[1024], b[1024], h1[1024], v1[1024], hv1[1024], h2[1024], v2[1024], hv2[1024];
        static uint8_t img[64 * 816]; static uint32_t r1[64 * 8], c1[816], r2[64 * 8], c2[816];
        int w = 1 + rnd() % 1000, m = rnd(), sw = 8 + rnd() % 200, ww = sw * (1 + rnd() % 4), hh = 1 + rnd() % 64;
        if (ww > 816) ww = sw;
        for (i = 0; i < w; i++) { a[i] = r8(m + i / 3); b[i] = r8(m + i / 2); }
        S->halfpel_row(a, b, w, h1, v1, hv1); N->halfpel_row(a, b, w, h2, v2, hv2);
        CHECK(!memcmp(h1, h2, w) && !memcmp(v1, v2, w) && !memcmp(hv1, hv2, w), "halfpel_row w %d", w);
        for (i = 0; i < (int)sizeof img; i++) img[i] = r8(m + i / 11);
        memset(r1, 0, sizeof r1); memset(r2, 0, sizeof r2); memset(c1, 0, sizeof c1); memset(c2, 0, sizeof c2);
        S->proj_rows(img, 816, ww, hh, sw, r1, c1); N->proj_rows(img, 816, ww, hh, sw, r2, c2);
        CHECK(!memcmp(r1, r2, sizeof r1) && !memcmp(c1, c2, sizeof c1), "proj_rows w %d sw %d h %d", ww, sw, hh);
    }
    printf("  halfpel/proj  done\n");
    printf(fails ? "test_kernels: %d FAILURES\n" : "test_kernels: ALL KERNELS BIT-IDENTICAL\n", fails);
    return fails != 0;
#endif
}
