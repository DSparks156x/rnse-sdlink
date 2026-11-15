/* asrc.c -- see asrc.h */
#include "asrc.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <math.h>
#include <stdlib.h>
#include <string.h>

static double bessel_i0(double x)
{
    double s = 1, t = 1, q = x * x / 4;
    for (int k = 1; k < 200; k++) {
        t *= q / ((double)k * k);
        s += t;
        if (t < s * 1e-17) break;
    }
    return s;
}

int asrc_init(asrc_t *r, double fs_in, double fs_out, int taps, int phases, double beta, double cut)
{
    memset(r, 0, sizeof *r);
    if (taps < 8 || (taps & 1) || phases < 2 || fs_in <= 0 || fs_out <= 0 || cut <= 0 || cut > 1) return -1;
    r->taps = taps; r->phases = phases; r->fs_in = fs_in; r->fs_out = fs_out;
    double c = cut * (fs_out < fs_in ? fs_out / fs_in : 1.0);   /* cutoff relative to the input Nyquist */
    double half = taps / 2.0, i0b = bessel_i0(beta);
    r->tab = malloc(sizeof(float) * (size_t)(phases + 1) * (size_t)taps);
    if (!r->tab) return -1;
    for (int p = 0; p <= phases; p++) {
        double f = (double)p / phases;
        float *row = r->tab + (size_t)p * taps;
        for (int m = 0; m < taps; m++) {
            double t = f + (half - 1) - m;                       /* input sample m of the window sits t samples before the output time */
            double w = fabs(t) >= half ? 0 : bessel_i0(beta * sqrt(1 - (t / half) * (t / half))) / i0b;
            double x = M_PI * c * t, s = fabs(x) < 1e-12 ? 1 : sin(x) / x;
            row[m] = (float)(c * s * w);
        }
        if (c == 1.0 && (p == 0 || p == phases)) {               /* integer positions: an exact copy (the step-1 bypass is bit-transparent) */
            memset(row, 0, sizeof(float) * (size_t)taps);
            row[p == 0 ? taps / 2 - 1 : taps / 2] = 1.0f;
        }
    }
    r->cap = 8192 + taps;
    for (int ch = 0; ch < 2; ch++) {
        r->h[ch] = calloc((size_t)r->cap, sizeof(float));
        if (!r->h[ch]) { asrc_free(r); return -1; }
    }
    r->have = taps / 2 - 1;                                      /* zeros before the first sample */
    r->pos = taps / 2 - 1;
    r->step = fs_in / fs_out;
    return 0;
}

void asrc_free(asrc_t *r)
{
    free(r->tab); free(r->h[0]); free(r->h[1]);
    r->tab = NULL; r->h[0] = r->h[1] = NULL;
}

void asrc_set_step(asrc_t *r, double step) { r->step = step; }
void asrc_snap(asrc_t *r) { r->pos = floor(r->pos + 0.5); }

int asrc_process(asrc_t *r, const float *in, int n, float *out, int out_cap)
{
    const int N = r->taps, back = N / 2 - 1, L = r->phases;
    int produced = 0;
    while (n > 0 || produced < out_cap) {
        /* make room: drop history the next output no longer needs */
        int drop = (int)floor(r->pos) - back;
        if (drop > 0 && (r->have + n > r->cap || drop > r->cap / 2)) {
            if (drop > r->have) drop = r->have;
            for (int ch = 0; ch < 2; ch++) memmove(r->h[ch], r->h[ch] + drop, sizeof(float) * (size_t)(r->have - drop));
            r->have -= drop;
            r->pos -= drop;
        }
        int take = r->cap - r->have;
        if (take > n) take = n;
        for (int i = 0; i < take; i++) {
            r->h[0][r->have + i] = in[2 * i];
            r->h[1][r->have + i] = in[2 * i + 1];
        }
        r->have += take; in += 2 * take; n -= take; r->in_total += (uint64_t)take;
        /* outputs while the window [i - back, i + N/2] is inside the history */
        int made = 0;
        while (produced < out_cap) {
            double fl = floor(r->pos);
            int i = (int)fl;
            if (i + N / 2 > r->have - 1) break;
            double fp = (r->pos - fl) * L;
            int p0 = (int)fp;
            if (p0 >= L) p0 = L - 1;
            float a = (float)(fp - p0);
            const float *ra = r->tab + (size_t)p0 * N, *rb = ra + N;
            const float *x0 = r->h[0] + (i - back), *x1 = r->h[1] + (i - back);
            double s0 = 0, s1 = 0;
            for (int m = 0; m < N; m++) {
                float cm = ra[m] + a * (rb[m] - ra[m]);
                s0 += (double)cm * x0[m];
                s1 += (double)cm * x1[m];
            }
            out[2 * produced] = (float)s0;
            out[2 * produced + 1] = (float)s1;
            produced++; made++;
            r->pos += r->step;
        }
        r->out_total += (uint64_t)made;
        if (n == 0) break;
        if (!take && !made) break;                               /* output full and no room: caller gave too little out_cap */
    }
    return produced;
}

/* ------------------------------------------------------------------ rate controller */
#define RC_TP_S     20.0        /* proportional time constant: an error of E frames is worked off in ~20 s */
#define RC_TI_S     80.0        /* integrator: 4 x Tp (phase margin ~75 deg on the pure-integrator plant) */
#define RC_EF_S     2.0         /* error filter: the reports carry reader-timing jitter */
#define RC_CLAMP    1500e-6     /* crystals are +-100 ppm each; anything near this is a fault, not drift */
#define RC_SLEW     0.5e-6      /* per call (tick, 60 Hz): <= 30 ppm/s */
#define RC_BIG      2400        /* 50 ms off target: a stall or a cut, not a rate error -- the unit handles those itself */
#define RC_FF_MIN_S 30.0        /* rate-seed window */
#define RC_FS       48000.0

void ratectl_init(ratectl_t *c, double step0)
{
    memset(c, 0, sizeof *c);
    c->step0 = step0;
    c->kp = 1.0 / (RC_FS * RC_TP_S);
    c->ki = c->kp / RC_TI_S;
}

void ratectl_reset(ratectl_t *c)
{
    c->have_e = 0; c->e_f = 0; c->cnt_have = 0; c->ff_have = 0;
}

void ratectl_fill(ratectl_t *c, int err, uint64_t t_us, int frozen)
{
    double dt = c->have_e && t_us > c->t_last_us ? (t_us - c->t_last_us) * 1e-6 : 0;
    if (dt > 2.0) { c->have_e = 0; dt = 0; }
    c->t_last_us = t_us;
    c->e_last = err;
    c->n_fill++;
    if (frozen || err > RC_BIG || err < -RC_BIG) { c->n_frozen++; return; }
    if (!c->have_e) { c->e_f = err; c->have_e = 1; return; }
    double a = dt / (RC_EF_S + dt);
    c->e_f += a * (err - c->e_f);
    c->integ += c->ki * c->e_f * dt;
    if (c->integ > RC_CLAMP) c->integ = RC_CLAMP;
    if (c->integ < -RC_CLAMP) c->integ = -RC_CLAMP;
}

/* Least-squares slope of the unit's consumed frames over Pi time = its sample rate on the Pi clock. The input comes in on the Pi clock at
 * RC_FS (avsd_clock drives the PipeWire graph), so step = RC_FS / rate. Seeds the integrator once, after RC_FF_MIN_S of clean reports. */
static double ls_n, ls_t, ls_c, ls_tt, ls_tc;
void ratectl_count(ratectl_t *c, uint16_t cnt, uint64_t t_us, uint64_t in_total)
{
    (void)in_total;
    c->n_cnt++;
    if (c->cnt_have) {
        double dt = (t_us - c->cnt_t) * 1e-6;
        uint32_t d = (uint16_t)(cnt - c->cnt_last);
        if (t_us < c->cnt_t || dt > 1.2 || fabs(d - RC_FS * dt) > 4800) c->cnt_have = 0;   /* gap or a session restart: new window */
        else c->cnt_un += d;
    }
    if (!c->cnt_have) {
        c->cnt_have = 1; c->cnt_un = 0; c->ff_t0 = t_us;
        ls_n = ls_t = ls_c = ls_tt = ls_tc = 0;
    }
    c->cnt_last = cnt; c->cnt_t = t_us;
    double t = (t_us - c->ff_t0) * 1e-6, y = (double)c->cnt_un;
    ls_n++; ls_t += t; ls_c += y; ls_tt += t * t; ls_tc += t * y;
    double span = t, den = ls_n * ls_tt - ls_t * ls_t;
    if (span < RC_FF_MIN_S || den <= 0) return;
    double rate = (ls_n * ls_tc - ls_t * ls_c) / den;
    double rel = (RC_FS / rate) - 1.0;
    c->ff_ratio = rel;
    if (!c->seeded && fabs(rel) < RC_CLAMP) { c->integ = rel; c->seeded = 1; }
}

double ratectl_step(ratectl_t *c)
{
    double want = c->integ + (c->have_e ? c->kp * c->e_f : 0);
    if (want > RC_CLAMP) want = RC_CLAMP;
    if (want < -RC_CLAMP) want = -RC_CLAMP;
    if (want > c->out + RC_SLEW) c->out += RC_SLEW;
    else if (want < c->out - RC_SLEW) c->out -= RC_SLEW;
    else c->out = want;
    return c->step0 * (1.0 + c->out);
}
