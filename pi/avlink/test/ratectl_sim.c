/* ratectl_sim.c -- offline long run of the audio rate loop (asrc.c ratectl) against a model of the unit, in simulated time.
 *   cc -O2 -std=c11 -I.. ratectl_sim.c ../asrc.c -lm -o ratectl_sim && ./ratectl_sim [ppm] [minutes]
 * Model: the Pi takes 800 input frames per 60 Hz tick (input exactly 48 kHz on the Pi clock) and sends 800 / step output frames (fraction carried);
 * the unit's sink plays at 48 kHz * (1 - ppm) plus a slow drift (+-20 ppm, 10 min period), in 960-frame blocks; the reported fill uses the
 * credit-time correction, so it is continuous up to the TMU/credit jitter (+-1 ms here). A status read every 8 superframes, alternating fill /
 * counter, with 1-20 ms of delay to the Pi. A 300 ms reader stall every 60 s (its audio arrives late, in one burst). Prints fill error and
 * pitch statistics per minute and PASS/FAIL. */
#include "asrc.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double urand(void) { return rand() / (RAND_MAX + 1.0); }

int main(int argc, char **argv)
{
    double ppm = argc > 1 ? atof(argv[1]) : 125, minutes = argc > 2 ? atof(argv[2]) : 30;
    ratectl_t c;
    ratectl_init(&c, 1.0);
    srand(7);
    const double dt = 1.0 / 60, tgt = 3840 + 15 * 960;
    double t = 0, carry = 0, queued = 0, consumed = 0, pending = 0, in_total = 0;
    int playing = 0, k = 0, fail = 0;
    long unders = 0;
    /* report queue (delay) */
    struct { double t; int kind; double v; } q[64]; int qh = 0, qn = 0;
    double emin = 1e9, emax = -1e9, es = 0, ess = 0; long en = 0;
    double smin = 1e9, smax = -1e9, slast = 0, slew_max = 0;
    int st_n = 0, st_k = 0, minute = 0;
    printf("sink %.0f ppm slow, drift +-20 ppm / 10 min, %.0f min\n", ppm, minutes);
    printf(" min  fill_err mean   sd    min    max | step ppm min   max   (sink ppm now)\n");
    while (t < minutes * 60) {
        double rate = 48000 * (1 - ppm * 1e-6 + 20e-6 * sin(2 * M_PI * t / 600));
        int stall = fmod(t, 60) > 30 && fmod(t, 60) < 30.3;
        /* Pi tick */
        double step = ratectl_step(&c);
        double out = 800 / step + carry;
        int n = (int)out; carry = out - n;
        in_total += 800;
        pending += n;                                       /* on its way (Teensy / reader) */
        if (!stall) { queued += pending; pending = 0; }     /* the reader takes it */
        /* sink */
        if (!playing && queued >= tgt) playing = 1;
        if (playing) {
            double c1 = rate * dt;
            if (queued < c1) { unders++; playing = 0; queued = 0; }
            else { queued -= c1; consumed += c1; }
        }
        t += dt;
        /* status read after the push (st_poll at the top of the next read) */
        if (playing && !stall && ++st_n >= 8) {
            st_n = 0;
            double jit = (urand() - 0.5) * 2e-3 * rate;     /* credit / TMU timing error, frames */
            double v = st_k ? fmod(consumed + jit, 65536) : queued - tgt + jit;
            q[(qh + qn) % 64].t = t + 0.001 + urand() * 0.019; q[(qh + qn) % 64].kind = st_k; q[(qh + qn) % 64].v = v; qn++;
            st_k ^= 1;
        }
        while (qn && q[qh].t <= t) {
            uint64_t tu = (uint64_t)(q[qh].t * 1e6);
            if (q[qh].kind == 0) ratectl_fill(&c, (int)lround(q[qh].v), tu, 0);
            else ratectl_count(&c, (uint16_t)(long)q[qh].v, tu, (uint64_t)in_total);
            qh = (qh + 1) % 64; qn--;
        }
        /* stats from minute 2 on */
        if (t > 120 && playing) {
            double e = queued - tgt;
            if (e < emin) emin = e;
            if (e > emax) emax = e;
            es += e; ess += e * e; en++;
            double sp = c.out * 1e6;
            if (sp < smin) smin = sp;
            if (sp > smax) smax = sp;
            if (k++ && fabs(sp - slast) > slew_max) slew_max = fabs(sp - slast);
            slast = sp;
        }
        if ((int)(t / 60) != minute) {
            minute = (int)(t / 60);
            if (en) printf("%4d  %8.0f %6.0f %6.0f %6.0f | %8.1f %6.1f   (%.1f)\n", minute, es / en, sqrt(ess / en - (es / en) * (es / en)), emin, emax, smin, smax,
                           (1 - rate / 48000) * 1e6);
            emin = 1e9; emax = -1e9; es = ess = 0; en = 0; smin = 1e9; smax = -1e9;
        }
    }
    printf("underruns %ld, max step change per tick %.3f ppm, seeded %d (ff %.1f ppm)\n", unders, slew_max, c.seeded, c.ff_ratio * 1e6);
    if (unders || slew_max > 0.51) fail = 1;
    puts(fail ? "FAIL" : "PASS");
    return fail;
}
