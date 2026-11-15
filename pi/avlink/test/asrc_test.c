/* asrc_test.c -- quality of the Pi ASRC against exact references (tones: the ideal output is known in closed form).
 *   cc -O2 -std=c11 -I.. asrc_test.c ../asrc.c -lm -o asrc_test && ./asrc_test
 *   ./asrc_test wav IN.wav OUT.f32 STEP [TAPS PHASES BETA CUT]   (music: stereo s16 wav -> float32 interleaved, for an offline comparison) */
#define _POSIX_C_SOURCE 200809L
#include "asrc.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static int g_fail;
/* run a tone through the converter; step(t) = step0 * (1 + dev * sin(2 pi fm t)); returns error rms re amplitude, dB */
static double tone(double fs_in, double fs_out, int taps, int ph, double beta, double cut, double f, double step0, double dev, double fm,
                   double *gain_db)
{
    asrc_t r;
    if (asrc_init(&r, fs_in, fs_out, taps, ph, beta, cut)) { puts("init failed"); exit(1); }
    const int NIN = 800, TOT = (int)fs_in * 4;
    static float in[2 * 800], out[2 * 2000];
    double A = 0.5, se = 0, ss = 0, sy = 0;
    long double t_out = 0;                   /* the reference time needs more than double: 2e5 additions drift ~1e-6 samples */
    long k = 0, skip = taps * 4;
    for (int j0 = 0; j0 < TOT; j0 += NIN) {
        for (int j = 0; j < NIN; j++) {
            double v = A * sin(2 * M_PI * f * (j0 + j) / fs_in);
            in[2 * j] = (float)v; in[2 * j + 1] = (float)(-v);
        }
        double st = step0 * (1 + dev * sin(2 * M_PI * fm * j0 / fs_in));
        asrc_set_step(&r, st);
        int n = asrc_process(&r, in, NIN, out, 2000);
        for (int i = 0; i < n; i++, k++) {
            double ideal = A * sin(2 * M_PI * f * (double)fmodl(t_out, (long double)fs_in) / fs_in);
            if (k >= skip) { double e = out[2 * i] - ideal; se += e * e; ss += ideal * ideal; sy += (double)out[2 * i] * out[2 * i];
                             double e1 = out[2 * i + 1] + ideal; se += e1 * e1; ss += ideal * ideal; sy += (double)out[2 * i + 1] * out[2 * i + 1]; }
            t_out += st;
        }
    }
    asrc_free(&r);
    if (gain_db) *gain_db = 10 * log10(sy / ss);
    return 10 * log10(se / ss);
}

static void suite(const char *name, double fs_in, double fs_out, int taps, int ph, double beta, double cut, double limit_db)
{
    double fr[] = {100, 1000, 5000, 10000, 15000, 18000, 20000, 21000, 22000};
    printf("%s (taps %d, phases %d, beta %.1f, cut %.2f): error re signal, dB [gain dB]\n", name, taps, ph, beta, cut);
    for (unsigned i = 0; i < sizeof fr / sizeof fr[0]; i++) {
        if (fr[i] >= fs_out / 2) continue;
        double g, e0 = tone(fs_in, fs_out, taps, ph, beta, cut, fr[i], fs_in / fs_out * (48000.0 / 47994.0), 0, 0, &g);
        double e1 = tone(fs_in, fs_out, taps, ph, beta, cut, fr[i], fs_in / fs_out * (48000.0 / 47994.0), 300e-6, 0.5, NULL);
        int chk = fr[i] <= 20000;
        printf("  %6.0f Hz: fixed %7.1f [%+.4f]  swept +-300ppm %7.1f%s\n", fr[i], e0, g, e1, chk && (e0 > limit_db || e1 > limit_db) ? "  FAIL" : "");
        if (chk && (e0 > limit_db || e1 > limit_db)) g_fail = 1;
    }
}

static int bypass(void)
{
    asrc_t r;
    asrc_init(&r, 48000, 48000, 96, 2048, 14.5, 1.0);
    asrc_set_step(&r, 1.0);
    static float in[2 * 48000], out[2 * 48200];
    srand(1);
    for (int i = 0; i < 2 * 48000; i++) in[i] = (float)((rand() % 65536) - 32768) / 32768.0f;
    int got = 0;
    for (int j = 0; j < 48000; j += 800) got += asrc_process(&r, in + 2 * j, 800, out + 2 * got, 48200 - got);
    int lat = r.taps / 2, bad = 0;
    for (int i = 0; i < got && i < 48000; i++) if (out[2 * i] != in[2 * i] || out[2 * i + 1] != in[2 * i + 1]) bad++;
    printf("bypass (step 1): %d frames out of %d in (latency %d), %d differ\n", got, 48000, lat, bad);
    asrc_free(&r);
    return bad;
}

static void speed(int taps, double step)
{
    asrc_t r;
    asrc_init(&r, 48000, 48000 / step, taps, 2048, 14.5, 1.0);
    asrc_set_step(&r, step);
    static float in[2 * 800], out[2 * 2000];
    for (int i = 0; i < 1600; i++) in[i] = (float)sin(i * 0.01);
    struct timespec a, b;
    clock_gettime(CLOCK_MONOTONIC, &a);
    long n = 0;
    for (int k = 0; k < 600; k++) n += asrc_process(&r, in, 800, out, 2000);   /* 10 s of audio */
    clock_gettime(CLOCK_MONOTONIC, &b);
    double s = (b.tv_sec - a.tv_sec) + (b.tv_nsec - a.tv_nsec) * 1e-9;
    printf("speed, %d taps: %.1f ms CPU per s of audio (%ld frames)\n", taps, s * 100, n);
    asrc_free(&r);
}

static int wav(int argc, char **argv)
{
    FILE *f = fopen(argv[2], "rb"), *o = fopen(argv[3], "wb");
    if (!f || !o) { perror("open"); return 1; }
    double step = atof(argv[4]);
    int taps = argc > 5 ? atoi(argv[5]) : 96, ph = argc > 6 ? atoi(argv[6]) : 2048;
    double beta = argc > 7 ? atof(argv[7]) : 14.5, cut = argc > 8 ? atof(argv[8]) : 1.0;
    char id[4]; unsigned sz;
    fseek(f, 12, SEEK_SET);
    for (;;) {                                              /* find the data chunk */
        if (fread(id, 1, 4, f) != 4 || fread(&sz, 4, 1, f) != 1) return 1;
        if (!memcmp(id, "data", 4)) break;
        fseek(f, sz, SEEK_CUR);
    }
    asrc_t r;
    asrc_init(&r, 48000, 48000 / step, taps, ph, beta, cut);
    asrc_set_step(&r, step);
    static short raw[2 * 800]; static float in[2 * 800], out[2 * 2000];
    size_t n;
    while ((n = fread(raw, 4, 800, f)) > 0) {
        for (size_t i = 0; i < 2 * n; i++) in[i] = raw[i] / 32768.0f;
        int m = asrc_process(&r, in, (int)n, out, 2000);
        fwrite(out, 8, (size_t)m, o);
    }
    fclose(o); fclose(f); asrc_free(&r);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc > 4 && !strcmp(argv[1], "wav")) return wav(argc, argv);
    suite("48k -> 48k (x 48000/47994)", 48000, 48000, 96, 2048, 14.5, 1.0, -135);
    suite("48k -> 44.1k", 48000, 44100, 128, 2048, 14.5, 1.0, -125);
    if (bypass()) g_fail = 1;
    speed(96, 48000.0 / 47994.0);
    speed(128, 48000.0 / 44100.0);
    puts(g_fail ? "FAIL" : "PASS");
    return g_fail;
}
