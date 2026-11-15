/* asrc.h -- the one sample-rate conversion in the SDLink audio path (docs/PROTOCOL.md 3.5 / 4.6).
 *
 * The phone audio arrives on the Pi's clock (pw-record of avsd_clock, nominal 48 kHz); the RNS-E's SSI plays it on its own crystal (~47994 Hz
 * measured). Before 2026-11-14 the RNS-E bridged the two with a 24-tap fixed-point resampler (rnse/src/rs_fir.h: -36 dB alias at 18 kHz).
 * Now the Pi does it here, in float, steered by the unit's status reports, and the unit copies the samples unchanged.
 *
 * Converter: polyphase windowed sinc (Kaiser), `taps` taps, `phases` table rows with linear interpolation between rows, float table, double
 * accumulators. The output position advances by `step` input samples per output sample (= fs_in / fs_out, steered). 48 -> 48 kHz: cutoff at the
 * input Nyquist, so phase 0 is an exact copy (the step-1 bypass is the same code). Alias/image products of a tone at f land near f - (fs_in -
 * fs_out); everything at or below 20 kHz is kept >= 120 dB clean (asrc_test.c measures it).
 *
 * Rate controller (ratectl): PI on the unit's reported fill error, plus a one-time seed from the measured rates (unit consumed-frame counter
 * against the frames this converter took in, both on the Pi's clock). Slow on purpose: the step may change by at most RC_SLEW per output
 * block, so the pitch never moves by more than a few ppm per second (inaudible: vibrato is noticed from ~0.2 % = 2000 ppm). */
#ifndef AVL_ASRC_H
#define AVL_ASRC_H
#include <stdint.h>

typedef struct {
    int taps, phases;           /* taps even; table rows phases + 1 */
    float *tab;                 /* (phases + 1) x taps */
    float *h[2];                /* input history per channel */
    int cap, have;              /* history capacity / frames held */
    double pos;                 /* next output time, in input samples from h[*][0] */
    double step;                /* input samples per output sample */
    double fs_in, fs_out;
    uint64_t in_total, out_total;
} asrc_t;

/* fs_out < fs_in lowers the cutoff (to 0.5 * fs_out * cut); cut <= 1: fraction of the lower Nyquist. 48k->48k: taps 64, cut 1. */
int asrc_init(asrc_t *r, double fs_in, double fs_out, int taps, int phases, double beta, double cut);
void asrc_free(asrc_t *r);
void asrc_set_step(asrc_t *r, double step);
/* integer output position (bypass at step 1): the next output is an input sample exactly */
void asrc_snap(asrc_t *r);
/* n interleaved stereo float frames in; up to out_cap frames out (interleaved). Returns frames produced. */
int asrc_process(asrc_t *r, const float *in, int n, float *out, int out_cap);
/* group delay in input samples (output time t reads input around t + taps/2 - 1 ahead of it) */
static inline double asrc_delay(const asrc_t *r) { return r->taps / 2 - 1; }

/* ------------------------------------------------------------------ rate controller */
typedef struct {
    double step0;               /* nominal fs_in / fs_out */
    double ki, kp;              /* per frame of error (step units) */
    double e_f;                 /* filtered fill error, frames */
    double integ;               /* step offset from the integrator (relative, e.g. 125e-6) */
    double out;                 /* current relative step offset (after slew) */
    int have_e;
    uint64_t t_last_us;         /* newest report (Pi clock) */
    /* feed-forward: rates over a long window */
    int seeded;
    uint64_t ff_t0, ff_in0; uint32_t ff_c0; int ff_have;
    uint64_t cnt_un;            /* unwrapped consumed counter */
    uint32_t cnt_last; int cnt_have; uint64_t cnt_t;
    double ff_ratio;            /* last measured in/out rate ratio - 1 (0 if none) */
    uint32_t n_fill, n_cnt, n_frozen;
    double e_last;              /* last raw report */
} ratectl_t;

void ratectl_init(ratectl_t *c, double step0);
void ratectl_reset(ratectl_t *c);   /* reports stopped: keep integ (it is the clock ratio), drop the rest */
/* one fill report: err = frames over the unit's target. frozen: the reader is stalled (error is not a rate error) */
void ratectl_fill(ratectl_t *c, int err, uint64_t t_us, int frozen);
/* one counter report: low 16 bits of frames the unit's sink consumed; in_total = frames this converter has taken in by now */
void ratectl_count(ratectl_t *c, uint16_t cnt, uint64_t t_us, uint64_t in_total);
/* once per output block (tick): slewed step to use */
double ratectl_step(ratectl_t *c);

#endif
