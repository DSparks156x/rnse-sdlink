/* Kernel micro-benchmark: ns per unit for the scalar and (when compiled in) NEON kernels.
 * On the Pi this measures the real NEON speed-up; the REPORT's Pi estimate uses these units with the per-frame
 * work counts of an RNV_COUNT build.  Usage: bench_kernels */
#define _GNU_SOURCE
#include "kern.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static double now_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1e9 + t.tv_nsec; }
static uint32_t rs = 7;
static uint32_t rnd(void) { rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5; return rs; }
static volatile uint32_t sink;

static void run(const rnv2_kern *k) {
    static uint8_t rgb[800 * 3 * 8], Y[800 * 8], U[400 * 8], V[400 * 8], pl[800 * 64], pl2[800 * 64], h[800], v[800], hv[800];
    static uint8_t s[128 * 64], r[128 * 64], rec[128]; static int16_t T[128]; static int32_t R[128], c[16], o[16]; static uint32_t rows[64 * 4], cols[800], out[8];
    int i, n; double t;
    for (i = 0; i < (int)sizeof rgb; i++) rgb[i] = (uint8_t)(rnd() >> 24);
    for (i = 0; i < (int)sizeof pl; i++) { pl[i] = (uint8_t)(rnd() >> 24); pl2[i] = (uint8_t)(pl[i] + (rnd() & 7)); }
    for (i = 0; i < (int)sizeof s; i++) { s[i] = (uint8_t)(rnd() >> 24); r[i] = (uint8_t)(s[i] ^ (rnd() & 15)); }
    for (i = 0; i < 128; i++) R[i] = (int32_t)(rnd() % 61) - 30;
    for (i = 0; i < 16; i++) c[i] = (int32_t)(rnd() % 201) - 100;
    printf("%-7s", k->name);
    t = now_ns(); for (n = 0; n < 200; n++) k->csc_glass(rgb, 400 * 8, Y, U, V); printf("  csc %6.2f ns/pair", (now_ns() - t) / (200.0 * 3200));
    t = now_ns(); for (n = 0; n < 2000000; n++) sink += k->sad8x8(pl + (n & 63), 800, pl2 + ((n >> 3) & 63), 800); printf("  sad %5.1f", (now_ns() - t) / 2e6);
    t = now_ns(); for (n = 0; n < 1000000; n++) { k->pd8(s + 128 * (n & 63), r + 128 * (n & 63), 64, 3, out); sink += out[3]; } printf("  pd8 %5.1f", (now_ns() - t) / 1e6);
    t = now_ns(); for (n = 0; n < 1000000; n++) { k->fwd8(s + 128 * (n & 63), r + 128 * (n & 63), T); sink += (uint32_t)T[5]; } printf("  fwd8 %5.1f", (now_ns() - t) / 1e6);
    { int16_t L[128]; t = now_ns(); for (n = 0; n < 1000000; n++) { T[n & 127] ^= 3; sink += (uint32_t)k->quant8(T, 32768, 7208, 17, L); } printf("  quant8 %5.1f", (now_ns() - t) / 1e6); }
    t = now_ns(); for (n = 0; n < 2000000; n++) { c[n & 15] ^= 1; k->iwht(c, o); sink += (uint32_t)o[7]; } printf("  iwht %5.1f", (now_ns() - t) / 2e6);
    t = now_ns(); for (n = 0; n < 1000000; n++) { k->addclamp(s + 128 * (n & 63), R, 0x5A | (n & 0xA5), rec); sink += rec[9]; } printf("  addcl %5.1f", (now_ns() - t) / 1e6);
    t = now_ns(); for (n = 0; n < 20000; n++) k->halfpel_row(pl + 800 * (n & 31), pl + 800 * ((n & 31) + 1), 800, h, v, hv); printf("  hpel %5.3f ns/px", (now_ns() - t) / (20000.0 * 800));
    t = now_ns(); for (n = 0; n < 2000; n++) { memset(rows, 0, sizeof rows); memset(cols, 0, sizeof cols); k->proj_rows(pl, 800, 800, 60, 200, rows, cols); } printf("  proj %5.3f ns/px\n", (now_ns() - t) / (2000.0 * 48000));
}

int main(void) {
    rnv2_csc_init();
    printf("bench_kernels: ns per call unless noted (csc per pixel pair, hpel/proj per pixel)\n");
    run(&rnv2_kern_scalar);
#ifdef RNV2_HAVE_NEON
    run(&rnv2_kern_neon);
#endif
    return (int)(sink & 0);
}
