/* tone48: 48 kHz stereo test tone into the DSP_DEC PCM sink (Snd_opn/Snd_dyD), as task 159.
 *
 * Param block (doom_audio_v2 + SD_LOSSLESS_STUDY B4): {cb, 0x100, rate code (9 = 48 kHz), 0 (PCM), 0, 1,
 * +18 = 0 (stereo), +1C, 0}. +1C = 1: the worker passes 16-bit words unchanged, so the client supplies native
 * big-endian halfwords, one u32 per frame. +1C = 0: the worker byte-swaps every halfword, so this program
 * pre-swaps (same sound if the study is right; noise if the study has the sense of +1C backwards).
 *
 * Flow control: the callback counts block credits (event 0x70000, 2 right after open, then one per played block);
 * a block is submitted only when credits > submitted. Ring of 4 buffers is safe: at most 2 blocks outstanding.
 *
 * CTL params: p0 seconds (1..120), p1 mode, p2 amplitude (0..32767), p3 +1C value (0/1), p4 rate code (8 or 9),
 *             p5 lr_swap (0: L in the high halfword), p6 block frames (256..4096, multiple of 32)
 * modes: 0 1 kHz both, 1 alternate L-only / R-only every 1 s, 2 L 1 kHz + R 1.5 kHz, 3 digital silence,
 *        4 L only 1 kHz, 5 R only 1 kHz
 * STAT words: see T_* below (mirrored in tone48.py). */
#include "rnse.h"
#include "sine_tab.h"

#define PROG_TONE  1
#define NBUF       4
#define MAXFR      4096
#define CREDIT_EV  0x70000u
#define EOS_EV     0xB0000u

enum { T_HANDLE = 8, T_OPN_RET, T_SINK_AT_START, T_CREDITS, T_SUBMITTED, T_DYD_FULL, T_DYD_ERR, T_EOS_EV,
       T_UNDERRUN_EV, T_OTHER_EV, T_LAST_EV, T_LAST_A, T_LAST_B, T_CB_CALLS, T_MAX_WAIT_MS, T_STALLS,
       T_SSI_CR, T_SSI_SR, T_SSI_CLKW, T_STP_RET, T_CLS_RET, T_EOS_RET, T_DRAIN_MS, T_FRAMES, T_LATE,
       T_FIRST_CREDIT_MS, T_LAST_DYD_RET, T_PARAM_MODE, T_PARAM_AMP, T_PARAM_1C, T_PARAM_RATE, T_PARAM_SECS,
       T_PARAM_BLOCK, T_FIRST_EV, T_FIRST_A, T_FIRST_B };

enum { E_OK, E_NOCTL, E_SINK_BUSY, E_OPN, E_NO_CREDIT, E_STALL, E_DYD, E_PARAM };

#define SND_OPN(p)          ((int (*)(u32 *))FW_SND_OPN)(p)
#define SND_DYD(h, b, l, a) ((int (*)(int, const void *, int, int))FW_SND_DYD)(h, b, l, a)
#define SND_STP(h)          ((int (*)(int))FW_SND_STP)(h)
#define SND_CLS(h)          ((int (*)(int))FW_SND_CLS)(h)

static u32 pblock[9];
static s16 t1k[48], t1k5[32];
static u32 n_frame;                 /* global frame index (48000 per second) */
static u32 ph1, ph2;                /* table phases */
static u32 alt_cnt, alt_side;       /* mode 1: frames into the current 1 s half, which side */
static u32 mode, p1c, lrswap, nfr;

static void cb(int h, int ev, int a, int b)
{
    u32 e = (u32)ev;
    (void)h;
    if (ST(T_CB_CALLS) == 0) { ST(T_FIRST_EV) = e; ST(T_FIRST_A) = (u32)a; ST(T_FIRST_B) = (u32)b; }
    ST(T_CB_CALLS)++;
    ST(T_LAST_EV) = e; ST(T_LAST_A) = (u32)a; ST(T_LAST_B) = (u32)b;
    if ((e & ~0x100u) == CREDIT_EV) ST(T_CREDITS)++;
    else if ((e & ~0x100u) == EOS_EV) ST(T_EOS_EV)++;
    else if (e == 1 || e == 2) ST(T_UNDERRUN_EV)++;
    else ST(T_OTHER_EV)++;
}

static inline u32 bswap16x2(u32 w) { return ((w & 0x00FF00FFu) << 8) | ((w >> 8) & 0x00FF00FFu); }

static inline u32 pack(s32 l, s32 r)
{
    u32 w = lrswap ? (((u32)r << 16) | ((u32)l & 0xFFFFu)) : (((u32)l << 16) | ((u32)r & 0xFFFFu));
    return p1c ? w : bswap16x2(w);
}

/* fade: 0 none, 1 ramp in over the first 512 frames of the stream, 2 ramp out over this block's first 512 frames */
static void fill(u32 *dst, int fade)
{
    u32 i;
    for (i = 0; i < nfr; i++) {
        s32 a = t1k[ph1], l = 0, r = 0;
        switch (mode) {
        case 0: l = r = a; break;
        case 1: if (alt_side) r = a; else l = a; break;
        case 2: l = a; r = t1k5[ph2]; break;
        case 4: l = a; break;
        case 5: r = a; break;
        default: break;
        }
        if (fade == 1 && n_frame < 512) { l = (l * (s32)n_frame) >> 9; r = (r * (s32)n_frame) >> 9; }
        if (fade == 2) {
            s32 k = i < 512 ? 512 - (s32)i : 0;
            l = (l * k) >> 9; r = (r * k) >> 9;
        }
        dst[i] = pack(l, r);
        if (++ph1 == 48) ph1 = 0;
        if (++ph2 == 32) ph2 = 0;
        n_frame++;
        if (++alt_cnt == 48000) { alt_cnt = 0; alt_side ^= 1; }
    }
    ST(T_FRAMES) = n_frame;
}

/* wait until credits > submitted, up to ms; returns 1 if a credit is available */
static int wait_credit(u32 ms)
{
    u32 t0 = tick(), w;
    while (ST(T_CREDITS) <= ST(T_SUBMITTED)) {
        w = tick() - t0;
        if (w >= ms) return 0;
        dly_tsk(1);
        ST(ST_HB) = tick();
    }
    w = tick() - t0;
    if (w > ST(T_MAX_WAIT_MS)) ST(T_MAX_WAIT_MS) = w;
    return 1;
}

static int submit(u32 *buf)
{
    int r, tries = 0;
    for (;;) {
        r = SND_DYD((int)ST(T_HANDLE), buf, (int)(nfr * 4), 0);
        ST(T_LAST_DYD_RET) = (u32)r;
        if (r == 0) { ST(T_SUBMITTED)++; return 0; }
        if (r == -2 && ++tries < 50) { ST(T_DYD_FULL)++; dly_tsk(1); continue; }
        ST(T_DYD_ERR)++;
        return -1;
    }
}

int main(void)
{
    u32 secs, amp, rate, i, k, t0, dur, err = E_OK, last_credits, last_credit_tick, ssi_done = 0;
    u32 *ring = (u32 *)TONE_BUF;
    int h;

    st_begin(PROG_TONE);
    if (!ctl_ok()) { st_end(PROG_TONE, E_NOCTL); return 0; }
    secs = CTL_P(0); mode = CTL_P(1); amp = CTL_P(2); p1c = CTL_P(3); rate = CTL_P(4); lrswap = CTL_P(5);
    nfr = CTL_P(6);
    ST(T_PARAM_MODE) = mode; ST(T_PARAM_AMP) = amp; ST(T_PARAM_1C) = p1c; ST(T_PARAM_RATE) = rate;
    ST(T_PARAM_SECS) = secs; ST(T_PARAM_BLOCK) = nfr;
    if (secs < 1 || secs > 120 || mode > 5 || amp > 32767 || p1c > 1 || (rate != 8 && rate != 9) || lrswap > 1 ||
        nfr < 256 || nfr > MAXFR || (nfr & 31)) { st_end(PROG_TONE, E_PARAM); return 0; }

    for (i = 0; i < 48; i++) t1k[i] = (s16)(((s32)sin1k[i] * (s32)amp) >> 15);
    for (i = 0; i < 32; i++) t1k5[i] = (s16)(((s32)sin1k5[i] * (s32)amp) >> 15);

    ST(T_SINK_AT_START) = R32(DMDL_HANDLE);
    if (ST(T_SINK_AT_START) != 0) { st_end(PROG_TONE, E_SINK_BUSY); return 0; }

    ST(ST_STAGE) = 2;
    pblock[0] = (u32)cb; pblock[1] = 0x100; pblock[2] = rate; pblock[3] = 0; pblock[4] = 0; pblock[5] = 1;
    pblock[6] = 0; pblock[7] = p1c; pblock[8] = 0;
    t0 = tick();
    h = SND_OPN(pblock);
    ST(T_OPN_RET) = (u32)h;
    if (h <= 0) { st_end(PROG_TONE, E_OPN); return 0; }
    ST(T_HANDLE) = (u32)h;

    /* the 2 initial credits */
    while (ST(T_CREDITS) < 2 && tick() - t0 < 300) dly_tsk(1);
    ST(T_FIRST_CREDIT_MS) = tick() - t0;
    if (ST(T_CREDITS) < 2) { err = E_NO_CREDIT; goto close; }

    ST(ST_STAGE) = 3;
    dur = secs * 1000u;
    t0 = tick();
    k = 0;
    last_credits = ST(T_CREDITS); last_credit_tick = t0;
    fill(ring, 1);                                  /* pre-fill the first block (with ramp-in) */
    while (tick() - t0 < dur && !stop_req()) {
        ST(ST_HB) = tick();
        if (ST(T_CREDITS) != last_credits) { last_credits = ST(T_CREDITS); last_credit_tick = tick(); }
        if (ST(T_CREDITS) > ST(T_SUBMITTED)) {
            if (submit(ring + k * MAXFR) < 0) { err = E_DYD; break; }
            k = (k + 1) & (NBUF - 1);
            fill(ring + k * MAXFR, 1);              /* prepare the next one while the sink plays */
        } else {
            if (tick() - last_credit_tick > 500) { ST(T_STALLS)++; err = E_STALL; break; }
            dly_tsk(1);
        }
        if (!ssi_done && tick() - t0 > 500) {      /* SSI snapshot while streaming (read-only) */
            ST(T_SSI_CR) = R32(SSI_CR); ST(T_SSI_SR) = R32(SSI_SR); ST(T_SSI_CLKW) = R32(SSI_CLKW);
            ssi_done = 1;
        }
    }
    if (err == E_OK && wait_credit(300)) {          /* one ramp-out block so the stop does not click */
        fill(ring + k * MAXFR, 2);
        submit(ring + k * MAXFR);
    }

close:
    ST(ST_STAGE) = 4;
    ST(T_EOS_RET) = (u32)SND_DYD(h, (const void *)-1, 0, 0);
    t0 = tick();
    while (ST(T_EOS_EV) == 0 && ST(T_CREDITS) < ST(T_SUBMITTED) + 2 && tick() - t0 < 1000) dly_tsk(1);
    ST(T_DRAIN_MS) = tick() - t0;
    ST(T_STP_RET) = (u32)SND_STP(h);
    ST(T_CLS_RET) = (u32)SND_CLS(h);
    st_end(PROG_TONE, err);
    return 0;
}
