/* sdstream: the full A/V-over-SD loop as a RAM program (task 159, @8F760000). link/SPEC.md mode A:
 *   per tick: read superframe seq from LBA 0x10000 + (seq mod 256) * 256 with the stock raw-read API 80653598
 *   (task context, 32-B aligned buffer SF_BUF -> zero-copy DMA, same driver lock as the file system, PATCH_LIST D1),
 *   check the header CRC, push PCM into a jitter FIFO that feeds the DSP_DEC sink (Snd_opn/Snd_dyD/Snd_stp/Snd_cls,
 *   PATCH_LIST A2, +1C = parameter), decode RNV2 video into 2-3 YUY2 buffers, flip on DU array 0 (Doom-style
 *   takeover, restore on exit). A buffer is rewritten only >= 17 ms after the flip that replaced it (DU_DESIGN 3b).
 * The TV-source audio switch (PATCH_LIST A1) is NOT here: select the MP3 source by hand, as for tone48.
 *
 * CTL: p0 slot, p1 card-detect gate (0 none, 1 high, 2 low), p2 flags (F_*), p3 max seconds (1..3600),
 *      p4 wrap K (0 = live stream; K = the card holds superframes 0..K-1 in slots 0..K-1, played in a loop),
 *      p5 DSP_DEC +1C (0/1, tone48 decides), p6 lr swap, p7 audio block frames (256..4096, multiple of 32),
 *      p8 audio target fill (frames, prebuffer before the first submit; >= block + 1600 recommended), p9 frame buffers (2/3), p10 first read
 *      hint (sectors 1..256), p11 start seq, p12 max superframes (0 = no limit); p13 RAM patch arm (0/5A), p14/p15 RAM window start/end (exclusive),
 *      only with F_MEM (128), which requires wrap=0; all memory commands default off
 * STAT: words S_* (mirrored in sdstream.py); log ring at RES_BASE: 1024 x 8 words (see L_*). */
#include "avsd.h"
#include "rnv2sh.h"
#include "sdmem_target.h"

#define PROG_SDSTREAM 6

enum { F_DECODE = 1, F_AUDIO = 2, F_DISPLAY = 4, F_TLVCRC = 8, F_PAYCRC = 16, F_PACE = 32, F_PREF = 64, F_MEM = 128, F_STANDBY = 512 };
/* superframe header flags (SPEC 3.2) */
enum { H_KEY = 1, H_REPEAT = 2, H_IDLE = 4, H_DISCONT = 8, H_ADISCONT = 16, H_SYNTH = 32, H_NOTREADY = 64,
       H_EXPIRED = 128, H_EOS = 256 };
enum { S_SEQ = 8, S_SF_OK, S_HDR_BAD, S_RETRY, S_EXPIRED, S_SEQ_MISM, S_RD_CALLS, S_RD_ERR, S_RD_SECT, S_RD_CNT_LO,
       S_RD_CNT_MAX, S_V_DEC, S_V_DEC_ERR, S_V_CRC_BAD, S_V_DROP, S_V_REPEAT, S_V_LATE, S_DEC_MAX, S_DEC_LO, S_DEC_HI,
       S_V_SHOWN, S_TLV_OK, S_TLV_BAD, S_A_CRC_BAD, S_A_IN, S_A_UNDER, S_A_BLOCKS, S_A_CREDITS, S_A_FILL, S_A_MIN,
       S_A_MAX, S_A_OVER, S_A_HANDLE, S_A_STATE, S_A_PLAYED, S_DISCONT, S_EOS, S_SKIPPED, S_DU, S_LOOP_MAX,
       S_LAST_FLAGS, S_LAST_FID, S_SINK0, S_CLOSE, S_EOS_RET, S_DRAIN_MS, S_RD_CNT_HI, S_LAST_RD_RET, S_PACE_WAITS,
       S_FIRST_ERR_SEQ, S_V_ERR_CODE, S_CD_RAW, S_A_FMT_BAD, S_WALL_MS, S_SF_TOTAL, S_REUSE_WAITS, S_A_EV_UNDER,
       S_A_EV_OTHER, S_FRAG_IN, S_FRAG_ERR, S_SPAN_MAX, S_VFRAME_MAX, S_SPAN1, S_SPAN2, S_SPAN3, S_SPAN4, S_SPAN5,
       S_SPAN6, S_SPAN7, S_SPAN8P, S_LOG_N, S_MEM_CMD, S_MEM_BAD, S_MEM_TX, S_MEM_RETRY, S_MEM_IO,
       S_IIS_RET, S_IIS_SAVED, S_IIS_MS, S_IIS_REISSUE, S_IIS_END,
       S_SSI_CR, S_SSI_SR, S_E219_A, S_E219_B, S_P904_A, S_P904_B,
       S_STUCK_N, S_STUCK_MS, S_STUCK_LBA, S_STUCK_CNT, S_SDR0, S_SDR1, S_SDR2, S_SDR3, S_SDR4, S_SDR5, S_SDR6, S_SDR7,
       S_FL_N, S_FL_WAIT, S_FL_F14, S_FL_F20, S_FL_F34, S_FL_F50, S_FL_MAX, S_FL_BLK,
       S_SESS, S_R_MIN, S_R_ZERO, S_RATE_MHZ, S_CTX_CHG, S_CTX_A, S_CTX_B, S_DU_FIX, S_A_RESYNC, S_MUTE_A, S_KEYS, S_RESTARTS };
enum { E_OK, E_NOCTL, E_PARAM, E_NOCARD, E_SINK_BUSY, E_OPN, E_NO_CREDIT, E_READ, E_STOPPED, E_DYD, E_SRC_LEFT };
enum { L_SEQ, L_FLAGS, L_EV, L_RD, L_DEC, L_FILL, L_TOTAL, L_TICK };
/* L_EV bits */
enum { EV_HDR_BAD = 1, EV_RETRY = 2, EV_VDEC = 4, EV_VERR = 8, EV_VCRC = 16, EV_VDROP = 32, EV_LATE = 64,
       EV_ACRC = 128, EV_UNDER = 256, EV_DISCONT = 512, EV_TLV_BAD = 1024, EV_SHOWN = 2048 };
#define LOGN 1024
#define NBLK 4
#define MAXBLK 4096
#define REUSE_MS 17u
#define SLOT_COUNTS 208125u          /* 16.667 ms at 12.4875 MHz */
#define CREDIT_EV 0x70000u
#define EOS_EV 0xB0000u

static sdmem memory_service;
static rnv_state vs;
static u32 pblock[9];
static u32 flags, p1c, lrswap, blk, target;
static volatile u32 a_wr, a_rd;      /* FIFO indices (frames), monotonic. a_wr: reader only, a_rd: audio task only */
static volatile int a_state;         /* 0 off, 1 prebuffer, 2 running, 3 dead */
static u32 a_k;                      /* next block buffer */
static u32 replaced[3], front;
static volatile u32 credits;
/* video frame reassembly (frames spanning several superframes): two 768 KiB buffers, ping-pong */
#define ASM0 0x8F800000u
#define ASM1 0x8F8C0000u   /* ends 8F980000; firmware sink ring starts 8F9DA320 (bench 2026-11-05) */
#define ASM_MAX 0xC0000u
static u32 asm_on, asm_tot, asm_have, asm_span, asm_key, asm_buf;

/* DSP input select through the V850 (AUDIO_ROUTE_RE.txt, untested until 2026-11-05): idx 3 = controller source 4 = IIS */
#define FW_SRCSEL   0x805F3A94u
#define FW_SRCIDX   0x805F782Eu
#define SRC_CACHE   (*(volatile u8 *)0x8E9F08C4u)
#define SRC_STATUS  (*(volatile u8 *)0x8E9F08CCu)
#define SRC_OVERLAY (*(volatile u8 *)0x8E9DFF54u)
#define SRCSEL(i)   ((int (*)(int))FW_SRCSEL)(i)
#define SRCIDX(s)   ((int (*)(int))FW_SRCIDX)(s)
static u32 iis_req, iis_nohop, iis_idx, iis_t0, iis_check, src_saved, iis_st, ctx_t;
static volatile u32 iis_on, src_left;
#define SRC_CTX     (*(volatile u8 *)0x8E9DFF52u)   /* 07 = TV, 01 = FM, 06 = MP3 */
#define CTX_WORD    (*(volatile u32 *)0x8E9DFF50u)
#define IIS_LEAD_FRAMES 4800u                        /* SSI must run first: an idle SSI makes the DSP fall back */

/* Route state machine, never blocks: it runs in the audio task, so the sink and the reader keep going during the switch. A blocking hop used to
 * stop the reader for 1.3 s, which backed up the whole chain (bench 2026-11-05). */
static void iis_service(void)
{
    u32 now;
    if (!iis_req) return;
    now = tick();
    switch (iis_st) {
    case 0:
        if (ST(S_A_BLOCKS) * blk < IIS_LEAD_FRAMES) return;
        ST(S_IIS_SAVED) = SRC_CACHE;
        src_saved = SRC_CTX;
        ST(S_CTX_A) = CTX_WORD;
        iis_idx = (u32)SRCIDX((int)ST(S_IIS_SAVED)) & 0xFF;
        if (ST(S_IIS_SAVED) == 4 && SRC_CTX == 7) { iis_idx = 6; ST(S_IIS_SAVED) = 8; }   /* a dead program left IIS selected on the TV source: restore means TV */
        ST(S_SSI_CR) = R32(SSI_CR); ST(S_SSI_SR) = R32(SSI_SR);
        ST(S_E219_A) = *(volatile u8 *)0x8E9E0219u;
        ST(S_MUTE_A) = ((u32)*(volatile u8 *)0x8E9E0216u << 8) | *(volatile u8 *)0x8E9E0217u;
        if (ST(S_IIS_SAVED) != 3 && !iis_nohop) { SRCSEL(2); iis_t0 = now; iis_st = 1; return; }   /* hop through the FM input first (see BENCH_HANDOFF) */
        iis_st = 3;
        return;
    case 1:
        if (SRC_STATUS == 3 || now - iis_t0 >= 1500) { iis_t0 = now; iis_st = 2; }
        return;
    case 2:
        if (now - iis_t0 >= 300) iis_st = 3;
        return;
    case 3:
        ST(S_IIS_RET) = (u32)SRCSEL(3);
        iis_on = 1; iis_t0 = iis_check = ctx_t = now; iis_st = 4;
        ST(S_E219_B) = *(volatile u8 *)0x8E9E0219u;
        return;
    }
    if (iis_st != 4) return;
    if (!ST(S_IIS_MS) && SRC_STATUS == 4) ST(S_IIS_MS) = now - iis_t0 + 1;
    if (now - iis_check >= 5) {                        /* user left the source: stop at once, never fight it */
        iis_check = now;
        if (SRC_CTX != src_saved) {
            ST(S_CTX_B) = CTX_WORD;
            if (SRC_OVERLAY == 0) { if (!src_left) ST(S_CTX_CHG)++; src_left = 1; }
            return;
        }
        ctx_t = now;
        if (SRC_CACHE != 4 && SRC_OVERLAY == 0 && now - iis_t0 >= 500) { SRCSEL(3); iis_t0 = now; ST(S_IIS_REISSUE)++; }   /* firmware re-selected, same source: re-assert */
    }
}

/* bench only: host writes KEYREQ. 0xAC000100|mask = press + release a panel key through queue 6 (r31 design A1), 0xAC000200 = select TV */
#define KEYREQ   (*(volatile u32 *)0x8F72FFF0u)
#define FW_QPOST 0x8054BAC6u
struct kslot { unsigned short busy; u8 bm[6]; };
#define key_slot (*(volatile struct kslot *)0x8F72FFF4u)   /* fixed, so the host can see whether ACON consumed it */
static u32 key_msg[2], key_t, key_on;
#define FW_PANEL 0x805FB0CCu                            /* the panel driver's own callback: bitmap6 */
static u8 key_bm[8] __attribute__((aligned(4)));
static void key_service(void)
{
    u32 v = KEYREQ;
    if (key_on == 3 && tick() - key_t >= 120) {
        key_on = 0; key_bm[0] = 0;
        ((void (*)(u8 *))FW_PANEL)(key_bm);
    }
    if (key_on == 1 && tick() - key_t >= 100 && !key_slot.busy) {
        key_on = 0; key_slot.busy = 1; key_slot.bm[0] = 0;
        ((int (*)(int, void *))FW_QPOST)(6, key_msg);
    }
    if ((v >> 24) != 0xACu || key_on || key_slot.busy) return;
    KEYREQ = 0; ST(S_KEYS)++;
    if ((v & 0xF00u) == 0x100u) {
        u32 i; for (i = 0; i < 6; i++) key_slot.bm[i] = 0;
        key_slot.busy = 1; key_slot.bm[0] = (u8)v;
        key_msg[0] = 0x00010001u; key_msg[1] = (u32)&key_slot;
        ((int (*)(int, void *))FW_QPOST)(6, key_msg);
        key_on = 1; key_t = tick();
    } else if ((v & 0xF00u) == 0x200u) {
        ((int (*)(int, int))0x80594D26u)((int)SRC_CTX, 13);
    } else if ((v & 0xF00u) == 0x300u) {                /* press through the driver callback, release 120 ms later */
        u32 i; for (i = 0; i < 8; i++) key_bm[i] = 0;
        key_bm[0] = (u8)v;
        ((void (*)(u8 *))FW_PANEL)(key_bm);
        key_on = 3; key_t = tick();
    } else if ((v & 0xF00u) == 0x400u) {                /* simulate: source byte (session logic only) */
        SRC_CTX = (u8)v;
    } else if ((v & 0xF00u) == 0x500u) {                /* simulate: firmware takes the plane (shadow only, no commit) */
        R32(0x8E8E7D14u) = 0x04004031u;
    }
}

static void cb(int h, int ev, int a, int b)
{
    u32 e = (u32)ev & ~0x100u;
    (void)h; (void)a; (void)b;
    if (e == CREDIT_EV) credits++;
    else if (e == EOS_EV) ST(S_EOS)++;
    else if (e == 1 || e == 2) ST(S_A_EV_UNDER)++;
    else ST(S_A_EV_OTHER)++;
}

static inline u32 be32(const u8 *p) { return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3]; }
static inline u32 be16(const u8 *p) { return ((u32)p[0] << 8) | p[1]; }
static inline u32 swap16x2(u32 w) { return ((w & 0x00FF00FFu) << 8) | ((w >> 8) & 0x00FF00FFu); }

/* ---------------------------------------------------------------- audio */
static u32 g_scan;
static void a_push(const u8 *p, u32 frames)
{
    u32 *fifo = (u32 *)AFIFO, i, room = AFIFO_FRAMES - (a_wr - a_rd);
    const u32 *s = (const u32 *)p;
    if (frames > room) { ST(S_A_OVER) += frames - room; frames = room; }
    for (i = 0; i < frames; i++) {
        u32 w = s[i];                                   /* BE s16 L, s16 R -> L in the high half */
        if (lrswap) w = (w << 16) | (w >> 16);
        if (!p1c) w = swap16x2(w);                      /* +1C = 0: the worker swaps bytes, pre-swap here */
        fifo[(a_wr + i) & (AFIFO_FRAMES - 1)] = w;
    }
    if (g_scan) {                                       /* bench: a steady tone must not jump at the input */
        volatile u32 *d = (volatile u32 *)0x8F72FFD0u;
        static s32 prev;
        for (i = 0; i < frames; i++) {
            s32 v = (s16)(fifo[(a_wr + i) & (AFIFO_FRAMES - 1)] >> 16), j = v - prev;
            if (j > 3000 || j < -3000) { if (!d[0]) d[1] = (a_wr + i) & (AFIFO_FRAMES - 1); d[0]++; d[2] = (a_wr + i) & (AFIFO_FRAMES - 1); }
            prev = v;
        }
    }
    a_wr += frames;
    ST(S_A_IN) += frames;
}

static u32 a_rebuf;
static volatile u32 a_flush, a_stop;

/* Sink ring (DSP_DEC, RE 2026-11-05, read live: element size 16384, n 16): Snd_dyD posts one element per block, and a credit comes back while the ring has room,
 * so feeding by credit alone keeps the ring FULL: 16 blocks + byte FIFO. That was the ~0.65 s of audio delay, and swallowing those blocks at start also emptied our FIFO,
 * which made the rate loop run at its -0.4% clamp for the first ~40 s (mic: 1 kHz -> 995.86 Hz). Now a block goes in only while the ring holds fewer than sink_max blocks. */
#define RING_RD   (*(volatile u32 *)0x8EA1987Cu)
#define RING_WR   (*(volatile u32 *)0x8EA19880u)
#define RING_N    (*(volatile u32 *)0x8EA19878u)
static u32 sink_max;

/* Rate match. Real sink rate is about 47994 Hz against 48000 from the Pi (mic, tone, clamp known; the old 48.11 kHz came from blocks vanishing into the ring). A cubic read of the FIFO,
 * step = 1 + PI on fill, clamp +-500 ppm (inaudible). Q16, 1 unit = 15.3 ppm. Bigger errors are faults, not drift: a backlog is cut (cross-fade), an empty FIFO rebuffers (fade). */
#define RS_MAX 33
#define RESYNC_OVER 3600u                   /* 75 ms over target: drop the excess */
#define FADE 64u
static u32 rs_frac, a_fadein;
static s32 rs_step, rs_ema, rs_int, a_last_l, a_last_r;
static void rs_steer(u32 fill)
{
    s32 err = (s32)fill - (s32)target, x;
    rs_ema += ((err << 8) - rs_ema) >> 6;
    if (err > -2400 && err < 2400) rs_int += err * (s32)(blk / 32u);
    if (rs_int > RS_MAX << 24) rs_int = RS_MAX << 24;
    if (rs_int < -(RS_MAX << 24)) rs_int = -(RS_MAX << 24);
    x = (rs_ema >> 8) / 64 + (rs_int >> 24);
    if (x > RS_MAX) x = RS_MAX;
    if (x < -RS_MAX) x = -RS_MAX;
    rs_step = 65536 + x;
    ST(S_FL_BLK) = (u32)x;
}
static inline s32 cub(s32 y0, s32 y1, s32 y2, s32 y3, s32 f)      /* Catmull-Rom, f Q11 */
{
    s32 a = 3 * (y1 - y2) + y3 - y0, b = 2 * y0 - 5 * y1 + 4 * y2 - y3, c = y2 - y0, v;
    v = (((((((a * f) >> 11) + b) * f) >> 11) + c) * f) >> 12;
    v += y1;
    return v > 32767 ? 32767 : v < -32768 ? -32768 : v;
}
static u32 rs_fill_block(u32 *dst, const u32 *fifo)  /* blk frames out; returns frames consumed */
{
    u32 i, pos = rs_frac;
    const u32 m = AFIFO_FRAMES - 1, rd = a_rd;
    for (i = 0; i < blk; i++) {
        u32 k = rd + (pos >> 16);
        u32 w0 = fifo[(k - 1) & m], w1 = fifo[k & m], w2 = fifo[(k + 1) & m], w3 = fifo[(k + 2) & m];
        s32 f = (s32)((pos & 0xFFFF) >> 5);
        s32 l = cub((s16)(w0 >> 16), (s16)(w1 >> 16), (s16)(w2 >> 16), (s16)(w3 >> 16), f);
        s32 r = cub((s16)w0, (s16)w1, (s16)w2, (s16)w3, f);
        dst[i] = ((u32)l << 16) | ((u32)r & 0xFFFF);
        pos += (u32)rs_step;
    }
    rs_frac = pos & 0xFFFF;
    return pos >> 16;
}
static inline u32 mix2(u32 a, u32 b, s32 g)          /* g/FADE of b, the rest of a */
{
    s32 n = (s32)FADE, l = ((s16)(a >> 16) * (n - g) + (s16)(b >> 16) * g) >> 6, r = ((s16)a * (n - g) + (s16)b * g) >> 6;
    return ((u32)l << 16) | ((u32)r & 0xFFFF);
}

static u32 rate_b0, rate_c0;
static const short tone_tab[48] = {0,2088,4141,6123,8000,9740,11314,12694,13856,14782,15455,15863,16000,15863,15455,14782,13856,12694,11314,9740,8000,6123,4141,2088,0,-2088,-4141,-6123,-8000,-9740,-11314,-12694,-13856,-14782,-15455,-15863,-16000,-15863,-15455,-14782,-13856,-12694,-11314,-9740,-8000,-6123,-4141,-2088};   /* bench: --sinktone replaces the audio with a steady 1 kHz at the sink, to tell sink gaps from gaps upstream or in the mic */
static u32 g_tone, tone_i;
static void a_service(void)
{
    u32 fill, rf;
    if (a_state != 1 && a_state != 2) return;
    if (a_flush) { a_flush = 0; a_rd = a_wr; }
    fill = a_wr - a_rd;
    if (a_state == 1 && fill >= target + sink_max * blk) { a_state = 2; a_fadein = 1; }   /* the ring takes its blocks first, then the FIFO sits at target */
    while (a_state == 2 && credits > ST(S_A_BLOCKS)) {
        u32 *dst = (u32 *)(TONE_BUF + a_k * MAXBLK * 4), n, i;
        const u32 *fifo = (const u32 *)AFIFO;
        u32 rs = p1c != 0, need = rs ? ((rs_frac + blk * (u32)rs_step) >> 16) + 3 : blk, used, keep_frac = rs_frac, old_rd = 0, cut = 0;
        int r;
        rf = RING_WR - RING_RD;
        if (rf >= sink_max) break;
        if (ST(S_A_BLOCKS) > 8) {                       /* stats: only once the ring has been filled */
            if (rf < ST(S_R_MIN)) ST(S_R_MIN) = rf;
            if (rf == 0) ST(S_R_ZERO)++;
        }
        fill = a_wr - a_rd;
        if (a_rebuf && fill >= target) { a_rebuf = 0; a_fadein = 1; }   /* the ring is already full of silence here */   /* jitter buffer full again: play */
        n = a_rebuf ? 0 : (fill < need ? fill : blk);
        if (n < blk && rf >= 2) break;                  /* sink still holds two blocks: wait for more data */
        if (n == blk && rs && fill > target + RESYNC_OVER) {   /* backlog after a stall: latency matters more than these frames */
            old_rd = a_rd; cut = 1;
            a_rd = a_wr - target; fill = target; rs_frac = 0;
            ST(S_A_RESYNC)++;
        }
        if (n == blk && rs) {
            used = rs_fill_block(dst, fifo);
            if (cut) for (i = 0; i < FADE; i++) dst[i] = mix2(fifo[(old_rd + i) & (AFIFO_FRAMES - 1)], dst[i], (s32)i);
            if (a_fadein) { a_fadein = 0; for (i = 0; i < FADE; i++) dst[i] = mix2(0, dst[i], (s32)i); }
            a_last_l = (s16)(dst[blk - 1] >> 16); a_last_r = (s16)dst[blk - 1];
        } else {
            for (i = 0; i < n; i++) dst[i] = fifo[(a_rd + i) & (AFIFO_FRAMES - 1)];
            if (n < blk) {                              /* sink about to run dry: fade the last sample out, silence, keep the SSI running */
                u32 w = n ? dst[n - 1] : (((u32)a_last_l << 16) | ((u32)a_last_r & 0xFFFF));
                for (i = n; i < blk; i++) { u32 g = i - n; dst[i] = g < FADE ? mix2(w, 0, (s32)g) : 0; }
                a_last_l = a_last_r = 0;
            }
            used = n;
        }
        if (n < blk && !a_rebuf) { ST(S_A_UNDER)++; a_rebuf = 1; rs_frac = 0; }   /* rebuffer behind silence, or supply = demand keeps it empty */
        if (g_tone) for (i = 0; i < blk; i++) { u32 v = (u16)tone_tab[tone_i]; dst[i] = (v << 16) | v; tone_i = (tone_i + 1) % 48; }
        r = SND_DYD((int)ST(S_A_HANDLE), dst, (int)(blk * 4), 0);   /* (h, buf, len, aux): arg 2 is alignment-checked + memcpy src */
        if (r != 0) { rs_frac = keep_frac; if (r == -2) break; a_state = 3; break; }
        a_rd += used;
        fill = a_wr - a_rd;
        if (rs && !a_rebuf) rs_steer(fill);
        ST(S_A_BLOCKS)++;
        a_k = (a_k + 1) & (NBLK - 1);
        if (ST(S_A_BLOCKS) == 200) { rate_b0 = 200; rate_c0 = now_counts(); }
        else if (rate_c0 && now_counts() - rate_c0 > 125000000u) {    /* every 10 s: real sink rate, mHz on the TMU clock */
            u32 c = now_counts();
            ST(S_RATE_MHZ) = (u32)((unsigned long long)(ST(S_A_BLOCKS) - rate_b0) * blk * 12487500000ull / (c - rate_c0));
            rate_b0 = ST(S_A_BLOCKS); rate_c0 = c;
        }
    }
    fill = a_wr - a_rd;
    ST(S_A_FILL) = fill;
    if (a_state == 2) {
        if (fill < ST(S_A_MIN)) ST(S_A_MIN) = fill;
        if (fill > ST(S_A_MAX)) ST(S_A_MAX) = fill;
    }
    ST(S_A_CREDITS) = credits;
    ST(S_A_PLAYED) = credits >= 2 ? (credits - 2) * blk : 0;
    ST(S_A_STATE) = (u32)a_state;
}

#ifdef SD_FASTSTUCK
static void fs_watch(void);
#endif

/* ---------------------------------------------------------------- leaving TV: -DSD_LEAVETRACE, -DSD_LEAVEFIX (leave_tv_glitch.md). Neither is in the plain build.
 * TRACE: exit stage times and a log of the array-0 registers at 8F7FF800..8F7FFFFF (free: ring end to ASM0). No change to behaviour, but main() returns 1.5 s late.
 * FIX: du_restore wrote a snapshot from TV entry back even when the firmware had already moved on. By then the snapshot's DSA points at capture buffers the firmware has freed
 * (8050ACD4 -> 80519E4A), and it turns P1 on again, on top, with MR 04004031. So: the display is given back from the audio task, and the snapshot is only written if the firmware has not touched array 0. */
#ifdef SD_LEAVEFIX
#define SD_LEAVETRACE
#endif
#if defined(SD_LEAVETRACE)
#define SH0_MR    0x8E8E7D14u
#define SH0_DSA0  0x8E8E7E24u
#define SH0_DPPR  0x8E8E7FCCu
#define SH0_MASK  0x8E8E97D4u
#define SH0_LAYER 0x8E8E9804u
/* 0 = array 0 is still as we left it, 1 = the firmware registered a surface on it (80523C3E and friends write MR), 2 = it released the layer (8052322C / 80520D7C clear layer state and the DPPR bit, MR stays ours) */
static inline u32 lf_fw_state(void)
{
    if (R32(SH0_MR) != DU_MR_YUY2) return 1;
    if (R32(SH0_LAYER) != 1 || !(R32(SH0_DPPR) & 8u)) return 2;
    return 0;
}
#define LT ((volatile u32 *)0x8F7FF800u)
#define LT_RING 32u   /* 60 entries of 8 words: tick, sh MR, sh DSA0, sh DPPR, layer | mask << 8 | src << 16 | screen << 24, hw MR, hw DSA0, hw DPPR. Only on a change; our own flips don't count */
#define LT_NENT 60u
enum { LT_MAGIC, LT_T_SESS, LT_T_SRC, LT_T_STOP, LT_T_MOVED, LT_SEEN, LT_T_BACK, LT_DONE, LT_T_RDDONE, LT_T_LOOP, LT_T_ROUTE, LT_T_AUDIO, LT_T_SINK, LT_T_END,
       LT_SV_MR, LT_SV_DSA, LT_SV_DPPR, LT_SV_LAYER, LT_SV_MASK, LT_DROPPED, LT_SKIPFLIP, LT_N };
#define LTS(i) do { if (!LT[i]) LT[i] = tick(); } while (0)
static u32 lt_last[10];
#ifdef SD_LEAVEFIX
static volatile u32 du_up;
#else
static u32 du_up;
#endif
static inline u32 lt_ours(u32 d) { return d >= DSA_PHYS(FB0) && d < DSA_PHYS(FB2) + FB_BYTES ? 1u : d; }
static void lt_begin(void)
{
    u32 i;
    for (i = 0; i < 0x800u / 4; i++) LT[i] = 0;
    for (i = 0; i < 10; i++) lt_last[i] = 0xFFFFFFFFu;
    LT[LT_MAGIC] = 0x4C565431u;
    LT[LT_T_SESS] = tick();
    LT[LT_SV_MR] = R32(SH0_MR); LT[LT_SV_DSA] = R32(SH0_DSA0); LT[LT_SV_DPPR] = R32(SH0_DPPR); LT[LT_SV_LAYER] = R32(SH0_LAYER); LT[LT_SV_MASK] = R32(SH0_MASK);
}
static void lt_sample(void)
{
    u32 k[8], i, ch = 0;
    k[0] = R32(SH0_MR); k[1] = R32(SH0_DSA0); k[2] = R32(SH0_DPPR);
    k[3] = (R32(SH0_LAYER) & 255u) | ((R32(SH0_MASK) & 255u) << 8) | ((u32)SRC_CTX << 16) | ((u32)*(volatile u8 *)0x8E9E083Du << 24);
    k[4] = R32(0xFFF80100u); k[5] = R32(0xFFF80120u); k[6] = R32(0xFFF80018u);
    for (i = 0; i < 7; i++) {
        u32 v = (i == 1 || i == 5) ? lt_ours(k[i]) : k[i];
        if (v != lt_last[i]) { lt_last[i] = v; ch = 1; }
    }
    if (ch) {
        volatile u32 *e = LT + LT_RING + (LT[LT_N] % LT_NENT) * 8u;
        e[0] = tick();
        for (i = 0; i < 7; i++) e[i + 1] = k[i];
        LT[LT_N]++;
    }
    if (src_left) LTS(LT_T_SRC);
    if (stop_req()) LTS(LT_T_STOP);
    if (du_up && lf_fw_state() && !LT[LT_T_MOVED]) { LT[LT_T_MOVED] = tick(); LT[LT_SEEN] = lf_fw_state(); }
}
#else
#define LTS(i) ((void)0)
#endif
#ifdef SD_LEAVEFIX
#define LF_MR      0x8E8E7D14u
#define LF_DPPR    0x8E8E7FCCu
#define LF_DIRTY   0x8E8E97D8u
#define LF_COMMIT  0x80520908u
static const u32 lf_addr[] = {   /* same words as avlib.c save_addr, in the same order; DPPR is handled apart: only our P1 bit goes back, the other planes' bits are the firmware's */
    0x8E8E7D14u, 0x8E8E7D3Cu, 0x8E8E7D64u, 0x8E8E7D84u, 0x8E8E7DACu, 0x8E8E7DD4u, 0x8E8E7DFCu, 0x8E8E7E24u, 0x8E8E7E4Cu, 0x8E8E7E74u, 0x8E8E7E9Cu, 0x8E8E7EC4u,
    0x8E8E7EECu, 0x8E8E7F14u, 0x8E8E7F3Cu, 0x8E8E7F64u, 0x8E8E7F84u, 0x8E8E7FA4u, 0x8E8E97D4u, 0x8E8E9804u,
};
#define LF_NSAVE (sizeof lf_addr / sizeof lf_addr[0])
static u32 lf_saved[LF_NSAVE], lf_saved_p1;
static inline u32 lf_cli(void) { u32 sr, n; __asm__ volatile("stc sr,%0" : "=r"(sr)); n = sr | 0xF0u; __asm__ volatile("ldc %0,sr" : : "r"(n)); return sr; }
static inline void lf_sti(u32 sr) { __asm__ volatile("ldc %0,sr" : : "r"(sr)); }
static inline void lf_commit(void) { R32(LF_DIRTY) |= 3u; ((int (*)(int))LF_COMMIT)(3); }
static int lf_takeover(u32 phys)   /* avlib du_takeover, except it can be taken again without du_restore having run */
{
    u32 i;
    for (i = 0; i < LF_NSAVE; i++) lf_saved[i] = R32(lf_addr[i]);
    lf_saved_p1 = R32(LF_DPPR) & 8u;
    R32(0x8E8E7D14u) = DU_MR_YUY2; R32(0x8E8E7D3Cu) = 800; R32(0x8E8E7D64u) = 0xFF; R32(0x8E8E7D84u) = 800; R32(0x8E8E7DACu) = 480; R32(0x8E8E7DD4u) = 0; R32(0x8E8E7DFCu) = 0;
    R32(0x8E8E7E24u) = phys; R32(0x8E8E7E4Cu) = phys; R32(0x8E8E7E74u) = phys;
    R32(0x8E8E7EECu) = 0x3FF0; R32(0x8E8E7F14u) = 0xFFF; R32(0x8E8E7F84u) = 0x7C1F; R32(0x8E8E7FA4u) = 480;
    R32(LF_DPPR) |= 8u; R32(0x8E8E9804u) = 1; R32(0x8E8E97D4u) = 0;
    lf_commit();
    return 0;
}
/* the check and the writes can't be split by the firmware's graphics task: IRQs off, about 20 instructions */
static void lf_flip(u32 phys)
{
    u32 sr = lf_cli();
    if (du_up && !lf_fw_state()) {
        R32(0x8E8E7E24u) = phys; R32(0x8E8E7E4Cu) = phys; R32(0x8E8E7E74u) = phys;
        R32(0xFFF80000u) |= 0x10000u;
        R32(0xFFF80120u) = phys; R32(0xFFF80124u) = phys; R32(0xFFF80128u) = phys;
        R32(0xFFF80000u) &= ~0x10000u;
        lf_sti(sr);
    } else { lf_sti(sr); LT[LT_SKIPFLIP]++; }
}
#define du_takeover lf_takeover
#define du_flip lf_flip
static void du_give_back(void);
static inline void lf_watch(void) { if (du_up && (src_left || stop_req() || lf_fw_state())) du_give_back(); }
#endif

/* Audio task: the sink never depends on how long a read or a hold takes (read max 59 ms, ring ~30 ms). Also owns the route. */
void audio_main(void)
{
    while (!a_stop) {
        a_service();
        iis_service();
        key_service();
#ifdef SD_FASTSTUCK
        fs_watch();
#endif
#ifdef SD_LEAVEFIX
        lf_watch();
#endif
#ifdef SD_LEAVETRACE
        lt_sample();
#endif
        dly_tsk(1);
    }
    a_stop = 2;
    ((void (*)(void))FW_EXT_TSK)();
}
__asm__(".text\n.align 2\n_audio_entry:\n  mov.l 1f,r15\n  mov.l 2f,r0\n  jmp @r0\n  nop\n  .align 2\n1: .long 0x8F9B6000\n2: .long _audio_main\n");
extern void audio_entry(void);

static void sleep_ms(u32 ms) { u32 t0 = tick(); while (tick() - t0 < ms) dly_tsk(1); }

/* ---------------------------------------------------------------- video buffers */
/* Fixed-rate presentation. The DU latches DSA at frame start, so two flips inside one refresh lose a frame and the next refresh repeats:
 * that is judder on an otherwise 60 fps stream. A decoded frame waits until FLIP_GAP after the previous flip. GAP is a little under the
 * 16.67 ms period so a late frame catches up over a few frames rather than leaving the phase stuck late. */
static volatile u32 r_w, r_r;
#define FLIP_GAP (SLOT_COUNTS - SLOT_COUNTS / 40)   /* 16.25 ms */
static u32 fl_last, fl_have;
static void flip_paced(u32 phys)
{
    if (fl_have) {
        u32 d = now_counts() - fl_last;
        if (d < FLIP_GAP && r_w - r_r < 3u) {   /* frames queued behind this one: no pacing until caught up. Pacing here held the decoder near real time, which held the reader, which starved the audio */
            ST(S_FL_WAIT)++;
            while (FLIP_GAP - (now_counts() - fl_last) > 2u * COUNTS_PER_MS && (s32)(FLIP_GAP - (now_counts() - fl_last)) > 0) dly_tsk(1);
            while ((now_counts() - fl_last) < FLIP_GAP) {}   /* last ms: dly_tsk overshoots */
        }
    }
    du_flip(phys);
    {
        u32 t = now_counts();
        if (fl_have) {
            u32 d = t - fl_last;
            ST(S_FL_N)++;
            if (d > ST(S_FL_MAX)) ST(S_FL_MAX) = d;
            if (d < 14u * COUNTS_PER_MS) ST(S_FL_F14)++;
            else if (d > 50u * COUNTS_PER_MS) ST(S_FL_F50)++;
            else if (d > 34u * COUNTS_PER_MS) ST(S_FL_F34)++;
            else if (d > 20u * COUNTS_PER_MS) ST(S_FL_F20)++;
        }
        fl_last = t; fl_have = 1;
    }
}

/* The firmware takes array 0 back when it leaves the TV screen: it rewrites MR while DSA still points at our YUY2 buffer, which shows as trash until it draws (owner 2026-11-05: about 1 s).
 * So the first sign of that, or of the source changing, gives the plane back at once. No flip after that. If the source is still ours and the TV screen is up 500 ms later (a re-init, not a leave), take it again. */
#define SCREEN_GROUP (*(volatile unsigned short *)0x8E9E083Cu)
#ifdef SD_LEAVEFIX
static u32 du_lost_t, first_shown_t, g_sess_t0;
/* Any task. Kind 1, 2: the firmware has moved on, its registers stay as they are, and the last frame stays up until it commits. Kind 0: stock TV state back, as before. */
static void du_give_back(void)
{
    u32 sr = lf_cli(), kind, i;
    if (!du_up) { lf_sti(sr); return; }
    du_up = 0; du_lost_t = tick();
    kind = lf_fw_state();
    if (!kind) {
        for (i = 0; i < LF_NSAVE; i++) R32(lf_addr[i]) = lf_saved[i];
        R32(LF_DPPR) = (R32(LF_DPPR) & ~8u) | lf_saved_p1;
    }
    lf_sti(sr);
    if (!kind) lf_commit();
    ST(S_DU) |= 0x100;
    LT[LT_T_BACK] = tick(); LT[LT_DONE] = kind + 1;
}
#else
#ifndef SD_LEAVETRACE
static u32 du_up, du_lost_t, first_shown_t, g_sess_t0;
#else
static u32 du_lost_t, first_shown_t, g_sess_t0;
#endif
static void du_give_back(void)
{
    if (!du_up) return;
    du_up = 0; du_lost_t = tick();
    du_restore();
    ST(S_DU) |= 0x100;
#ifdef SD_LEAVETRACE
    LT[LT_T_BACK] = tick(); LT[LT_DONE] = 4;
#endif
}
#endif
static void du_guard(void)
{
    if (!(flags & F_DISPLAY)) return;
#ifdef SD_LEAVEFIX
    if (du_up) {
        if (src_left || stop_req() || lf_fw_state()) { if (!src_left && !stop_req()) ST(S_DU_FIX)++; du_give_back(); }
        return;
    }
    {   /* take it again only over the stock TV plane: a TV re-init, not the next screen's UI */
        u32 mr = R32(LF_MR);
        if ((mr != 0x04004031u && mr != 0x04014031u) || R32(0x8E8E9804u) != 1 || stop_req()) return;
    }
#else
    if (du_up) {
        if (src_left || du_mr() != DU_MR_YUY2) { if (!src_left) ST(S_DU_FIX)++; du_give_back(); }
        return;
    }
#endif
    if (!src_left && iis_st == 4 && SRC_CTX == src_saved && !SRC_OVERLAY && SCREEN_GROUP == 9 && tick() - du_lost_t >= 500) {
        u32 t = tick();
        if (t - du_lost_t >= 500) { du_takeover(DSA_PHYS(front)); du_up = 1; ST(S_DU) &= ~0x100u; }
    }
}

static int fbi(u32 p) { return p == FB0 ? 0 : p == FB1 ? 1 : 2; }
static void wait_reuse(u32 p)
{
    u32 i = (u32)fbi(p);
    if (tick() - replaced[i] < REUSE_MS) {
        ST(S_REUSE_WAITS)++;
        while (tick() - replaced[i] < REUSE_MS) dly_tsk(1);   /* decoder task: audio belongs to the reader */
    }
}

/* down-channel TLVs {u8 type, u8 len, data} (sdgen.py writes them):
 *   0x10 len 8 : {u32 frame index, u32 expected CRC-32 of the decoded YUY2 frame}   (test streams only)
 *   0x11 len 12: video fragment {u32 frame_total_bytes, u32 offset of this fragment, u32 CRC-32 of the whole frame}
 *                (a frame larger than one superframe's video area is split over consecutive superframes; each
 *                fragment keeps video_bytes / video_crc32 for its own bytes; no 0x11 = the frame is complete here) */
static const u8 *tlv_find(const u8 *ctrl, u32 n, u32 type, u32 len)
{
    u32 i = 0;
    while (i + 2 <= n) {
        u32 t = ctrl[i], l = ctrl[i + 1];
        if (i + 2 + l > n) break;
        if (t == type && l == len) return ctrl + i + 2;
        i += 2 + l;
    }
    return 0;
}

static void logent(u32 seq, u32 hf, u32 ev, u32 rd, u32 dec, u32 total)
{
    u32 k = ST(S_LOG_N) & (LOGN - 1);
    volatile u32 *e = (volatile u32 *)(RES_BASE + 32 * k);
    e[L_SEQ] = seq; e[L_FLAGS] = hf; e[L_EV] = ev; e[L_RD] = rd; e[L_DEC] = dec; e[L_FILL] = a_wr - a_rd;
    e[L_TOTAL] = total; e[L_TICK] = tick();
    ST(S_LOG_N)++;
}

static u32 g_slot, g_wrap;
static volatile u32 rd_in, rd_lba, rd_cnt, rd_t0, stuck_seen;
/* sd_raw_read waits 50 s for a completion that sometimes never comes (bench 2026-11-05: BRE set, ILR set, no end, no error, cause open; it does
 * return -1 and the next read works). Same call it makes (806536C8) with a short timeout: a hang costs one retry, not 50 s of silence.
 * Normal reads stay under 60 ms (30 min soak, holds included). */
#define FW_SD_RDCORE 0x8067D108u
#define SD_BUSY(s)   (*(volatile u32 *)((s) == 1 ? 0x8EAAC96Cu : 0x8EAAC970u))
#define READ_TO_MS   65   /* normal reads under 31 ms in a 10 min soak; FIFO + ring hold about 90 ms */
static u32 g_to_test;                                   /* bench: every Nth read gets a 1 ms timeout, to exercise the timeout path */
#ifdef SD_TRACE
static u32 g_last_raw, g_last_cnt;
#endif
static int sd_read_to(u32 slot, u32 lba, u32 buf, u32 n)
{
    volatile int cnt = (int)n;
    int r, to = READ_TO_MS;
    if (g_to_test && ST(S_RD_CALLS) % g_to_test == g_to_test - 1) to = 1;
    SD_BUSY(slot) = 1;
    r = ((int (*)(u32, void *, volatile int *, u32, int))FW_SD_RDCORE)(slot, (void *)buf, &cnt, lba, to);
    SD_BUSY(slot) = 0;
#ifdef SD_TRACE
    g_last_raw = (u32)r; g_last_cnt = (u32)cnt;
#endif
    return ((short)r != 0 || cnt != (int)n) ? -1 : 0;
}
#define SD_VARIANT0 0x8EAD1254u                         /* slot 0 clock: byte +2 = 1 -> /4 = 12.5 MHz, bench-proven */
#define SDH16(o) (*(volatile unsigned short *)(0xFF90A000u + (o)))

#ifdef SD_FASTSTUCK
/* Early end of stuck live reads, slot 0 only. Offline RE 2026-11-05 on r57, see stuck_reads.md. Runs in the audio task (or the decoder with no audio), never in the reader.
 * Type A, DMA stopped with data waiting: BRE set, the empty-port bit latched, DMA remaining count unchanged for FS_FROZEN_MS, driver asleep in its data wait.
 *   A Teensy hold cannot match: during a hold the host buffer drains and BRE is clear. Action: put a data-timeout bit in the driver's status image and wake the reader.
 *   The driver then takes its own stock error path (8068340A), which stops the host and retries inside the call. No driver code or lock is touched.
 * Type B, the reader asleep in the 100 ms CBSY wait of the command routine (8068638E) after a CMD12, CBSY already clear: move its wake tick to the next tick with the kernel's own
 *   timeout-list helpers, interrupts masked. The driver loop then re-reads CBSY and carries on.
 * The wake count is cleared before each read: nobody may wake the reader then, and a left-over wake would end the next command wait early, which is what Type B looks like.
 * Host: ST(127) holds the address of fs (struct below). Nothing acts until the DMA count has been seen moving (fs.live), so a count that does not read back leaves stock behaviour. */
#define FS_TASK       0x2Fu
#define FS_TCB        (0x8E8D53ACu + FS_TASK * 0x40u)
#define FS_TCB16(o)   (*(volatile unsigned short *)(FS_TCB + (o)))   /* +4 state, +0x14 wake count */
#define FS_ST_TSLP    0x0804u                                        /* timed sleep (804CD9BC); wup_tsk wakes it, and in other states only counts */
#define FS_ST_DLY     0x1004u                                        /* dly_tsk (804CE8FE); wup_tsk does not wake it */
#define FS_TICK       (*(volatile u32 *)0x8E8DA8B0u)
#define FS_OWNER      (*(volatile unsigned short *)0x8EAD13A0u)      /* slot 0: task the SD ISR wakes, 0 when the driver is free */
#define FS_DRVSTAT    (*(volatile u32 *)0x8EAD125Cu)                 /* slot 0 INFO image, tested with 0x807F at 8068338C */
#define FS_DMA        (*(volatile u32 *)0x8EAAC93Cu)                 /* SD DMA channel base (FF801200) */
#define FS_FROZEN_MS  2u
#define FS_INJECT     0x0008u                                        /* in the 0x807F set, same bit a real data timeout would give */
#define FS_DIS_DSP()  ((void (*)(void))0x804D0470u)()
#define FS_ENA_DSP()  ((void (*)(void))0x804D0480u)()
#define FS_WUP(id)    ((int (*)(int))0x804D031Cu)((int)(id))
#define FS_TO_DEL(t)  ((void (*)(u32))0x804CFB1Eu)(t)                /* leaf, list remove */
#define FS_TO_ADD(t)  ((void (*)(u32))0x804CFACCu)(t)                /* leaf, sorted insert by +0x18 */
static volatile struct {
    u32 live, b5_reads, wup_stale, wup_race, nob5, frozen_dead;      /* learned count; reads with the bit latched; left-over wakes cleared; our wake lost to the ISR; frozen without the bit, not acted on; frozen before live, not acted on */
    u32 a_n, a_ms, a_tcr, a_dsts, a_ddar, a_info, a_sec, a_buf0, a_buf1; /* last Type A: count, ms into the read, DMA left / status / current dest, INFO1<<16|INFO2, SECCNT, first 8 bytes of the buffer (uncached) */
    u32 b_n, b_ms, b_rem, b_info;                                    /* last Type B: count, ms into the read, sleep left when cut, INFO1<<16|INFO2 */
} fs;
static u32 fs_buf;

static inline u32 fs_cli(void)
{
    u32 sr, n;
    __asm__ volatile("stc sr,%0" : "=r"(sr));
    n = sr | 0xF0u;
    __asm__ volatile("ldc %0,sr" : : "r"(n));
    return sr;
}
static inline void fs_sti(u32 sr) { __asm__ volatile("ldc %0,sr" : : "r"(sr)); }

/* reader, driver not entered */
static inline void fs_preread(u32 buf)
{
    ST(127) = (u32)&fs;
    fs_buf = buf;
    if (!g_wrap && !g_slot && FS_TCB16(0x14)) { FS_TCB16(0x14) = 0; fs.wup_stale++; }
}

static void fs_watch(void)
{
    static u32 t_same, tcr_last, t0_last, b5_t0;
    u32 t0, now, st, i2, m1, m2, cmd, dma, tcr;
    if (!rd_in || g_wrap || g_slot || FS_OWNER != FS_TASK) { t_same = 0; return; }
    t0 = rd_t0; now = tick(); st = FS_TCB16(4);
    cmd = SDH16(0x00); i2 = SDH16(0x1E); m1 = SDH16(0x20); m2 = SDH16(0x22);

    if (cmd == 0x000Cu && m1 == 0x0305u && m2 == 0x837Fu && st == FS_ST_DLY && (SDH16(0x24) & 0x100u) && !(i2 & 0x4000u)) {
        u32 sr = fs_cli(), rem = R32(FS_TCB + 0x18) - FS_TICK, cut = 0;
        if (FS_TCB16(4) == FS_ST_DLY && rem > 2u && rem <= 100u) {
            FS_TO_DEL(FS_TCB);
            R32(FS_TCB + 0x18) = FS_TICK + 1u;
            FS_TO_ADD(FS_TCB);
            cut = 1;
        }
        fs_sti(sr);
        if (cut) { fs.b_n++; fs.b_ms = now - t0; fs.b_rem = rem; fs.b_info = ((u32)SDH16(0x1C) << 16) | i2; }
        t_same = 0;
        return;
    }
    if (cmd != 0x0012u || m1 != 0x0301u || m2 != 0x0330u || st != FS_ST_TSLP) { t_same = 0; return; }

    dma = FS_DMA;
    tcr = R32(dma + 0x20);                                 /* remaining count, read back by sdtime's rate test */
    if (tcr && tcr != rd_cnt << 8) fs.live = 1;
    if ((i2 & 0x20u) && b5_t0 != t0) { b5_t0 = t0; fs.b5_reads++; }
    if (!(i2 & 0x100u) || !(i2 & 0x4000u)) { t_same = 0; return; }   /* BRE clear: draining done or a hold; CBSY clear: not a transfer */
    if (!t_same || t0_last != t0 || tcr != tcr_last) { t_same = now; t0_last = t0; tcr_last = tcr; return; }
    if (now - t_same < FS_FROZEN_MS) return;
    t_same = 0;
    if (!(i2 & 0x20u)) { fs.nob5++; return; }
    if (!fs.live) { fs.frozen_dead++; return; }
    fs.a_n++; fs.a_ms = now - t0; fs.a_tcr = tcr; fs.a_dsts = R32(dma + 0x34); fs.a_ddar = R32(dma + 0x1C);
    fs.a_info = ((u32)SDH16(0x1C) << 16) | i2; fs.a_sec = SDH16(0x0A);
    fs.a_buf0 = *(volatile u32 *)(fs_buf | 0x20000000u); fs.a_buf1 = *(volatile u32 *)((fs_buf | 0x20000000u) + 4);
    FS_DIS_DSP();                                          /* the reader stays off the CPU until the count is checked */
    if (FS_TCB16(4) == FS_ST_TSLP && SDH16(0x20) == 0x0301u) {
        FS_DRVSTAT |= FS_INJECT;                           /* a wake with a clean image would be taken as a good read */
        FS_WUP(FS_TASK);
        if (FS_TCB16(0x14)) { FS_TCB16(0x14) = 0; fs.wup_race++; }   /* the SD ISR woke it first: ours only counted, and would end the next wait early */
    }
    FS_ENA_DSP();
}
#endif

static int timed_read(u32 slot, u32 lba, u32 buf, u32 n)
{
    u32 c0 = now_counts(), c1, d;
    int r;
    if (!g_wrap && !g_slot) R32(SD_VARIANT0) = 0x00000100u;   /* live stream, /4. The driver does variant = max(variant, retries), cleared only on reinit: one error would otherwise leave the slot at /8 (2.8 MB/s) for good */
#ifdef SD_FASTSTUCK
    fs_preread(buf);
#endif
    rd_in = 1; rd_lba = lba; rd_cnt = n; rd_t0 = tick();
    r = g_wrap ? SD_READ((int)slot, (int)lba, (void *)buf, (int)n) : sd_read_to(slot, lba, buf, n);
    rd_in = 0;
    c1 = now_counts(); d = c1 - c0;
    ST(S_RD_CALLS)++;
    ST(S_LAST_RD_RET) = (u32)r;
    if (ST(S_RD_CNT_LO) + d < ST(S_RD_CNT_LO)) ST(S_RD_CNT_HI)++;
    ST(S_RD_CNT_LO) += d;
    if (d > ST(S_RD_CNT_MAX)) ST(S_RD_CNT_MAX) = d;
    if (r) ST(S_RD_ERR)++;
#ifdef SD_TRACE
    if (r) {                                            /* bench: what a failed read looked like, first 8 */
        volatile u32 *z = (volatile u32 *)0x8F75B000u;
        u32 k = z[0]++;
        if (k < 8) {
            z[4 + k * 6] = lba; z[5 + k * 6] = n; z[6 + k * 6] = d;
            z[7 + k * 6] = g_last_raw; z[8 + k * 6] = g_last_cnt;
            z[9 + k * 6] = ((u32)SDH16(0x1E) << 16) | SDH16(0x2C);
        }
    }
#endif
    else { ST(S_RD_SECT) += n; oc_inval(buf, n * 512); }
    return r;
}

/* ---------------------------------------------------------------- two tasks (bench 2026-11-05)
 * The CPU idles during SD DMA, and read + decode in one loop did not fit 16.67 ms (map3d 7.6 + 12.6 + 2 ms). Reader task: SD, header,
 * audio and the sink, in a free slot (id 0x2F, prio 0x30, above the decoder) on its own stack. Decoder: task 159, video only. */
#define RD_TASK     0x2Fu
#define TCB_OF(id)  (0x8E8D53ACu + (u32)(id) * 0x40u)
#define FW_STA_TSK  0x804D026Cu
#define SFRING      0x8F780000u
#define NSF         6u
#ifndef SF_SECT
#define SF_SECT     170u        /* 170 x 6 ends 8F7FF800, below ASM0. 160 x 6 was the r61 ring */
#endif
#define SF_SIZE     (SF_SECT * 512u)    /* 160 sectors: hud2 --maxvid 150 + audio + header. Six of them keep audio coming through two decode-heavy frames */
struct sfe { u32 hf, sseq, total, c_rd, lost, seq; };
static volatile struct sfe r_e[NSF];
static volatile u32 rd_done, rd_err;
static u32 g_secs, g_hint, g_seq, g_maxsf, g_t_start, g_steal, g_t_all0;
static u32 need_key, g_sess_seq;
static inline u32 secs_up(void) { return g_secs && tick() - g_t_all0 >= g_secs * 1000u; }   /* secs 0 = forever */
static int (*g_dec)(rnv_state *, const rnv_u8 *, rnv_u32);
static u32 g_nbuf;

#ifdef SD_TRACE
/* bench only, not in the flash build: per-iteration traces in the free gap SFRING end..ASM0, frozen shortly after the first expired or CRC-bad event */
#define TR_N 256u
#define TR_R ((volatile u32 *)0x8F758000u)
#define TR_D ((volatile u32 *)0x8F759000u)
#define TR_H ((volatile u32 *)0x8F75A000u)   /* 0 rn, 1 dn, 2 trig type, 3 trig seq, 4 frozen, 5 ncrc; CRC captures from word 16, 16 words each */
static u32 tr_left, tr_pw;
static void tr_trig(u32 type, u32 seq)
{
    if (TR_H[2] || tick() - g_t_start < 3000u) return;
    TR_H[2] = type; TR_H[3] = seq; tr_left = 48;
}
static void tr_r(u32 ev, u32 seq, u32 hint, u32 total, u32 c0, u32 c_rd)
{
    volatile u32 *e;
    if (TR_H[4]) return;
    e = TR_R + 4 * (TR_H[0] % TR_N);
    e[0] = c0; e[1] = c_rd; e[2] = (seq << 20) | ((hint & 1023) << 10) | (total & 1023);
    e[3] = (ev << 24) | (((r_w - r_r) & 255) << 16) | ((tr_pw > 255 ? 255 : tr_pw) << 8) | (((a_wr - a_rd) >> 6) & 255);
    tr_pw = 0; TR_H[0]++;
    if (tr_left && --tr_left == 0) TR_H[4] = 1;
}
static void tr_d(u32 c0, u32 lc, u32 cdec, u32 seq, u32 ev)
{
    volatile u32 *e;
    if (TR_H[4]) return;
    e = TR_D + 4 * (TR_H[1] % TR_N);
    e[0] = c0; e[1] = lc; e[2] = cdec; e[3] = (seq << 16) | (ev & 0xFFFF);
    TR_H[1]++;
}
static void tr_crc(const u8 *sf, u32 seq, u32 hf, u32 total, u32 vsec, u32 vb)
{
    u32 n = TR_H[5]++, i, magic = 0, mfirst = 0, zero = 0, zfirst = 0;
    volatile u32 *c;
    tr_trig(2, seq);
    if (n >= 16) return;
    c = TR_H + 16 + n * 16;
    for (i = 1; i < total; i++) {
        const u32 *w = (const u32 *)(sf + i * 512);
        if (be32((const u8 *)w) == 0x52534156u) { if (!magic++) mfirst = i; }
        if (!w[0] && !w[1] && !w[2] && !w[3] && !w[64] && !w[127]) { if (!zero++) zfirst = i; }
    }
    c[0] = seq; c[1] = hf; c[2] = total; c[3] = vsec; c[4] = vb; c[5] = magic; c[6] = mfirst; c[7] = zero; c[8] = zfirst;
    c[9] = r_w; c[10] = r_r; c[11] = now_counts(); c[12] = be32(sf + 44); c[13] = crc32_upd(0, sf + vsec * 512, vb);
    c[14] = be32(sf + 8); c[15] = be16(sf + 42);
}
#define TR_READER(ev) tr_r((ev), seq, hint, total, c0, c_rd)
#else
#define tr_trig(a, b) ((void)0)
#define TR_READER(ev) ((void)0)
#endif

void reader_main(void)
{
    u32 rdfail = 0, retry = 0, lost = 0, due = 0, have_due = 0, seq = g_sess_seq ? g_sess_seq : g_seq, hint = g_hint;
    while (1) {
        u32 w = r_w, buf = SFRING + (w % NSF) * SF_SIZE, slotseq = g_wrap ? seq % g_wrap : seq, lba = 0x10000u + (slotseq & 255u) * 256u, hf, total = 0, sseq;
        const u8 *sf = (const u8 *)buf;
        u32 c0 = now_counts(), c_rd = 0;
        int r;
        if (stop_req()) { rd_err = E_STOPPED; break; }
        if (src_left) { rd_err = E_SRC_LEFT; break; }
        if (secs_up()) break;
        if (g_maxsf && ST(S_SF_TOTAL) >= g_maxsf) break;
        ST(ST_HB) = tick();
        ST(S_SEQ) = seq; g_sess_seq = seq;
        if (w - r_r >= NSF) { ST(S_PACE_WAITS)++;
#ifdef SD_TRACE
            tr_pw++;
#endif
            dly_tsk(1); continue; }  /* decoder behind: hold the read, audio still services */
        r = timed_read(g_slot, lba, buf, hint);
        c_rd = now_counts() - c0;
        if (r) {
            if (++rdfail >= (g_wrap ? 5u : 40u)) { rd_err = E_READ; break; }   /* live: a hang costs 150 ms, don't drop the stream over a few. The retry reads the same seq, so nothing is lost if it is still resident. */
            sleep_ms(1);
            continue;
        }
        rdfail = 0;
        {
            u32 crc = crc32_upd(crc32_upd(0, sf, 60), sf + 64, 448);
            if (be32(sf) != 0x52534156u || crc != be32(sf + 60) || be16(sf + 6) != 64) {
                ST(S_HDR_BAD)++;
                if (!ST(S_FIRST_ERR_SEQ)) ST(S_FIRST_ERR_SEQ) = seq | 0x80000000u;
                logent(seq, 0, EV_HDR_BAD, c_rd, 0, 0);
                if (++retry < 3) { sleep_ms(2); continue; }
                retry = 0; seq++; ST(S_SKIPPED)++; lost = 1;   /* a lost superframe loses video */
                continue;
            }
        }
        hf = be32(sf + 12); sseq = be32(sf + 8); total = be16(sf + 40);
        if (tlv_find(sf + 64, be16(sf + 52) > 448 ? 448 : be16(sf + 52), 0x12, 4)) CTL_W(CTL_STOP) = 1;   /* STOP from the Pi (avlink sdctl.h): same as the CAN stop word, no CAN */
        ST(S_LAST_FLAGS) = hf; ST(S_LAST_FID) = be16(sf + 54);
        if (hf & H_EXPIRED) { ST(S_EXPIRED)++; TR_READER(2); tr_trig(1, seq); seq = be32(sf + 64); lost = 1; continue; }
        if (hf & (H_IDLE | H_NOTREADY)) {
            ST(S_RETRY)++; TR_READER(3); logent(seq, hf, EV_RETRY, c_rd, 0, 0);
            if (!g_wrap) {                               /* Teensy reports its newest complete seq (NOTREADY: bytes 64..67, IDLE: the header seq): resync at once, also after a feeder restart */
                u32 nw = (hf & H_NOTREADY) ? be32(sf + 64) : sseq;
                if (nw && ((s32)(nw - seq) > 4 || (s32)(seq - nw) > 64)) { seq = nw; lost = 1; continue; }
            }
            sleep_ms(2);
            continue;
        }
        if (sseq != slotseq) {
            ST(S_SEQ_MISM)++;
            if (!g_wrap && (s32)(sseq - seq) > 0) { seq = sseq; lost = 1; }   /* we are behind: jump forward */
            else { ST(S_RETRY)++; sleep_ms(2); continue; }                    /* slot not rewritten yet: retry */
        }
        retry = 0;
        if (total < 1 || total > SF_SIZE / 512u) { ST(S_HDR_BAD)++; seq++; lost = 1; continue; }
        if (total > hint) {
            r = timed_read(g_slot, lba + hint, buf + hint * 512, total - hint);
            c_rd = now_counts() - c0;
            if (r) { if (++rdfail >= 5) { rd_err = E_READ; break; } continue; }
        }
        {
            u32 nh = be16(sf + 42);
            hint = nh ? nh : total;
            if (hint > SF_SIZE / 512u) hint = SF_SIZE / 512u;
        }
        if (hf & H_ADISCONT) a_flush = 1;
        if (flags & F_MEM) sdmem_control(&memory_service, sf + 64, be16(sf + 52));
        {   /* audio goes straight to the FIFO, so it never waits for video */
            u32 ab = be32(sf + 32), afr = be16(sf + 28), asec = be16(sf + 36), fmt = sf[30];
            if (ab && asec >= 1 && asec * 512 + ab <= total * 512) {
                const u8 *ap = sf + asec * 512;
                if ((flags & F_PAYCRC) && crc32_upd(0, ap, ab) != be32(sf + 48)) ST(S_A_CRC_BAD)++;
                else if (fmt == 1 && afr * 4 <= ab) { if (flags & F_AUDIO) a_push(ap, afr); else ST(S_A_IN) += afr; }
                else ST(S_A_FMT_BAD)++;
            }
        }
        if (flags & F_MEM) {
            sdmem_pump(&memory_service, ST(S_SF_TOTAL));
            ST(S_MEM_CMD)=memory_service.commands; ST(S_MEM_BAD)=memory_service.bad;
            ST(S_MEM_TX)=memory_service.tx; ST(S_MEM_RETRY)=memory_service.retries; ST(S_MEM_IO)=memory_service.io_errors;
        }
        {
            volatile struct sfe *e = &r_e[w % NSF];
            e->hf = hf; e->sseq = sseq; e->total = total; e->c_rd = c_rd; e->lost = lost; e->seq = seq;
            lost = 0;
            TR_READER(1);
            r_w = w + 1;
        }
        if (hf & H_EOS) break;
        seq++;
        if (flags & F_PACE) {                           /* card images: one superframe per 16.667 ms, nudged by the FIFO */
            if (!have_due) { due = c0; have_due = 1; }
            due += SLOT_COUNTS;
            if (a_state == 2) {
                u32 fill = a_wr - a_rd;
                if (fill > target + 1600) due += COUNTS_PER_MS;
                else if (fill + 1600 < target) due -= COUNTS_PER_MS;
            }
            if ((s32)(now_counts() - due) < 0) ST(S_PACE_WAITS)++;
            while ((s32)(now_counts() - due) < 0) dly_tsk(1);
        } else {
#ifdef SD_TRACE
            /* TR_H: 6 n, 7 sum read-end to sleep (counts), 8 sum sleep, 9 max sleep, 10..14 sleep hist <1.5/2.5/3.5/5.5/more ms, 15 sum whole loop */
            u32 c2 = now_counts(), c3;
#ifdef SD_SLEEP
            dly_tsk(1);
#endif
            c3 = now_counts();
            {
                u32 ds = c3 - c2, ms = ds / 1249u;      /* 0.1 ms */
                TR_H[6]++; TR_H[7] += c2 - c0 - c_rd; TR_H[8] += ds; if (ds > TR_H[9]) TR_H[9] = ds;
                TR_H[ms < 15 ? 10 : ms < 25 ? 11 : ms < 35 ? 12 : ms < 55 ? 13 : 14]++;
                TR_H[15] += c3 - c0;
            }
#else
            /* no sleep after a superframe: dly_tsk(1) measured 2.8-3.0 ms, ~18% of each frame. -DSD_SLEEP restores it (A/B) */
#ifdef SD_SLEEP
            dly_tsk(1);
#endif
#endif
        }
    }
    rd_done = 1;
    ((void (*)(void))FW_EXT_TSK)();
}

/* entered by the kernel on the small slot stack: move to our own 64 KiB, between AFIFO and the firmware sink buffers (8F9B6320..) */
__asm__(".text\n.align 2\n_reader_entry:\n  mov.l 1f,r15\n  mov.l 2f,r0\n  jmp @r0\n  nop\n  .align 2\n1: .long 0x8F9B0000\n2: .long _reader_main\n");
extern void reader_entry(void);

static void decode_slot(const u8 *sf, u32 hf, u32 c_rd, u32 total, u32 seq)
{
    u32 i, ev = 0, c_dec = 0, c0_loop = now_counts();
    if (hf & H_DISCONT) {
        ST(S_DISCONT)++; ev |= EV_DISCONT;
        if (flags & F_DECODE) {
            if (flags & F_DISPLAY) for (i = 0; i < g_nbuf; i++) if ((u32)vs.buf[i] != front) wait_reuse((u32)vs.buf[i]);
            if (flags & F_DISPLAY) rnv_state_reset_keep(&vs, (const u8 *)front);   /* shown frame stays up */
            else rnv_state_reset(&vs);
        }
        need_key = 0;
    }
    {
        u32 vb = be32(sf + 24), vsec = be16(sf + 38), fmt = sf[31], cn = be16(sf + 52) > 448 ? 448 : be16(sf + 52);
        const u8 *vp = 0, *fr = tlv_find(sf + 64, cn, 0x11, 12);
        u32 vlen = 0, key = hf & H_KEY;
        if (hf & H_REPEAT) ST(S_V_REPEAT)++;
        else if (vb && fmt == 2 && vsec >= 1 && vsec * 512 + vb <= total * 512) {
            const u8 *pp = sf + vsec * 512;
            if ((flags & F_PAYCRC) && !(flags & 256u) && crc32_upd(0, pp, vb) != be32(sf + 44)) {
                ST(S_V_CRC_BAD)++; ev |= EV_VCRC; need_key = 1; asm_on = 0;
#ifdef SD_TRACE
                tr_crc(sf, seq, hf, total, vsec, vb);
#endif
            } else if (fr) {
                u32 tot = be32(fr), off = be32(fr + 4);
                if (off == 0) {
                    asm_on = 0;
                    if (tot > ASM_MAX || tot <= vb) { ST(S_FRAG_ERR)++; need_key = 1; }
                    else { asm_on = 1; asm_tot = tot; asm_have = 0; asm_span = 0; asm_key = key;
                           asm_buf = asm_buf == ASM0 ? ASM1 : ASM0; }
                }
                if (asm_on && off == asm_have && tot == asm_tot && vb <= tot - off) {
                    memcpy((void *)(asm_buf + off), pp, vb);
                    asm_have += vb; asm_span++;
                    ST(S_FRAG_IN)++;
                    if (asm_have == tot) {
                        asm_on = 0;
                        if ((flags & F_PAYCRC) && !(flags & 256u) && crc32_upd(0, (const u8 *)asm_buf, tot) != be32(fr + 8)) {
                            ST(S_FRAG_ERR)++; need_key = 1; ev |= EV_VCRC;
                        } else { vp = (const u8 *)asm_buf; vlen = tot; key = asm_key; }
                    }
                } else if (off) {                      /* lost/out-of-order fragment: drop the frame */
                    if (asm_on || !need_key) { ST(S_FRAG_ERR)++; need_key = 1; }
                    asm_on = 0;
                }
            } else {
                if (asm_on) { ST(S_FRAG_ERR)++; asm_on = 0; need_key = 1; }   /* unfinished frame abandoned */
                vp = pp; vlen = vb; asm_span = 1;
            }
        }
        if (vp) {
            ST(S_SPAN1 + (asm_span > 8 ? 7 : asm_span - 1))++;
            if (asm_span > ST(S_SPAN_MAX)) ST(S_SPAN_MAX) = asm_span;
            if (vlen > ST(S_VFRAME_MAX)) ST(S_VFRAME_MAX) = vlen;
            if (need_key && !key) {
                ST(S_V_DROP)++; ev |= EV_VDROP;
            } else if (flags & F_DECODE) {
                u32 c0, d;
                int e;
                if (flags & F_DISPLAY) wait_reuse((u32)vs.buf[vs.cur]);
                c0 = now_counts();
                e = g_dec(&vs, vp, vlen);
                d = now_counts() - c0; c_dec = d;
                if (ST(S_DEC_LO) + d < ST(S_DEC_LO)) ST(S_DEC_HI)++;
                ST(S_DEC_LO) += d;
                if (d > ST(S_DEC_MAX)) ST(S_DEC_MAX) = d;
                if (e) {
                    ST(S_V_DEC_ERR)++; ST(S_V_ERR_CODE) = (u32)e | (vs.err_blk << 8); ev |= EV_VERR; need_key = 1;
                } else {
                    u32 out = (u32)vs.buf[vs.last];
                    const u8 *t = tlv_find(sf + 64, cn, 0x10, 8);
                    ST(S_V_DEC)++; ev |= EV_VDEC; need_key = 0;
                    if ((flags & F_TLVCRC) && t) {
                        if (crc32_upd(0, (const u8 *)out, FB_BYTES) == be32(t + 4)) ST(S_TLV_OK)++;
                        else { ST(S_TLV_BAD)++; ev |= EV_TLV_BAD; }
                    }
                    if (flags & F_DISPLAY) {
                        wb_drain(out + FB_BYTES - 4);
                        du_guard();
                        if (du_up) flip_paced(DSA_PHYS(out));
                        if (!first_shown_t) { first_shown_t = tick(); ST(S_STUCK_MS) = first_shown_t - g_sess_t0; }
                        replaced[fbi(front)] = tick();
                        front = out;
                        ST(S_V_SHOWN)++; ev |= EV_SHOWN;
                    }
                }
            } else need_key = 0;
        }
    }
    {
        u32 lc = now_counts() - c0_loop;               /* decoder time only; the reader is concurrent */
        if (lc > ST(S_LOOP_MAX)) ST(S_LOOP_MAX) = lc;
        if (lc > SLOT_COUNTS) { ST(S_V_LATE)++; ev |= EV_LATE; }
#ifdef SD_TRACE
        tr_d(c0_loop, lc, c_dec, seq, ev);
#endif
    }
    ST(S_SF_OK)++; ST(S_SF_TOTAL)++;
    logent(seq, hf, ev, c_rd, c_dec, total);
}

/* One TV visit: open sink, take the display, stream until the time is up, the user leaves the source, or a stop. */
#define AU_TASK 0x2Eu
static int start_task(u32 id, void (*entry)(void), u32 *saved)
{
    u32 tcb = TCB_OF(id);
    if ((R32(tcb + 4) >> 16) != 0x10) return -1;       /* slot not dormant: leave it alone */
    *saved = R32(tcb + 0x20);
    R32(tcb + 0x20) = (u32)entry;
    if (((int (*)(int, int))FW_STA_TSK)((int)id, 0)) { R32(tcb + 0x20) = *saved; return -1; }
    return 0;
}

static u32 session(void)
{
    u32 err = E_OK, i, t_start, tcb_r = TCB_OF(RD_TASK), tcb_a = TCB_OF(AU_TASK), saved_r = 0, saved_a = 0, au_up = 0;
    /* per-visit state */
    a_wr = a_rd = 0; a_state = 0; a_k = 0; credits = 0; a_rebuf = 0; a_flush = 0; a_stop = 0; a_fadein = 0; a_last_l = a_last_r = 0;
    rs_step = 65536; rs_ema = 0; rs_int = 0; rs_frac = 0; rate_c0 = 0;
    iis_on = 0; iis_st = 0; src_left = 0;
#ifdef SD_TRACE
    { u32 z; for (z = 0; z < 16; z++) TR_H[z] = 0; tr_left = 0; tr_pw = 0; *(volatile u32 *)0x8F75B000u = 0; }
#endif
    r_w = r_r = 0; rd_done = 0; rd_err = 0; rd_in = 0; stuck_seen = 0;
    asm_on = 0; fl_have = 0; need_key = g_wrap ? 0 : 1;     /* mid-stream start: wait for a key instead of decoding deltas on black */
    ST(S_A_BLOCKS) = 0; ST(S_A_HANDLE) = 0; du_up = 0; first_shown_t = 0; g_sess_t0 = tick();
    ST(S_SESS)++;
    t_start = tick();
#ifdef SD_LEAVETRACE
    lt_begin();
#endif

    /* ---- audio sink (tone48's sequence) */
    if (flags & F_AUDIO) {
        int h;
        u32 t0;
        ST(S_SINK0) = R32(DMDL_HANDLE);
        ST(S_P904_A) = R32(0xFF909004u);
        if (iis_req) ((void (*)(void))0x8062D848u)();   /* stock player's once-per-boot enable (FF909004 bit 2); read from code, see AUDIO_ROUTE_RE (c) */
        ST(S_P904_B) = R32(0xFF909004u);
        /* Snd_opn fails while the player holds the stream (bench 2026-11-05): --steal closes the player's handle first */
        if (g_steal && (ST(S_SINK0) >> 24) == 2u) {
            SND_STP((int)ST(S_SINK0));
            SND_CLS((int)ST(S_SINK0));
            dly_tsk(5);
        }
        pblock[0] = (u32)cb; pblock[1] = 0x100; pblock[2] = 9; pblock[3] = 0; pblock[4] = 0; pblock[5] = 1;
        pblock[6] = 0; pblock[7] = p1c; pblock[8] = 0;
        h = SND_OPN(pblock);
        if (h <= 0) { ST(S_A_HANDLE) = (u32)h; return E_OPN; }
        ST(S_A_HANDLE) = (u32)h;
        t0 = tick();
        while (credits < 2 && tick() - t0 < 300) dly_tsk(1);
        if (credits < 2) { err = E_NO_CREDIT; goto close; }
        a_state = 1;
    }

    /* ---- video */
    if (flags & F_DECODE) {
        rnv_state_init(&vs, (u8 *)FB0, (u8 *)FB1, (u8 *)FB2, (int)g_nbuf);
        if (flags & F_DISPLAY) {
            front = (u32)vs.buf[vs.last];
            wb_drain(front + FB_BYTES - 4);
            ST(S_DU) = 1 + (u32)du_takeover(DSA_PHYS(front));
            du_up = 1;
            replaced[0] = replaced[1] = replaced[2] = tick() - REUSE_MS;
        }
    }

    ST(ST_STAGE) = 2;
    g_t_start = t_start = tick();
    if (flags & F_AUDIO) {
        if (start_task(AU_TASK, audio_entry, &saved_a)) { err = E_READ; goto close; }
        au_up = 1;
    }
    if (start_task(RD_TASK, reader_entry, &saved_r)) { err = E_READ; goto close; }
    {
        while (1) {
            u32 rr = r_r;
            du_guard();
            if (rr == r_w) {
                if (rd_done) break;
                if (rd_in && tick() - rd_t0 > (g_wrap ? 2000u : READ_TO_MS - 30u) && stuck_seen != rd_t0) {   /* read stuck: snapshot the SD host once per stall */
                    stuck_seen = rd_t0;
                    ST(S_STUCK_N)++; ST(S_STUCK_LBA) = rd_lba; ST(S_STUCK_CNT) = rd_cnt;
                    ST(S_SDR0) = ((u32)SDH16(0x00) << 16) | SDH16(0x1C);   /* CMD | INFO1 */
                    ST(S_SDR1) = ((u32)SDH16(0x1E) << 16) | SDH16(0x20);   /* INFO2 | INFO1_MASK */
                    ST(S_SDR2) = ((u32)SDH16(0x22) << 16) | SDH16(0x24);   /* INFO2_MASK | CLK_CTRL */
                    ST(S_SDR3) = ((u32)SDH16(0x26) << 16) | SDH16(0x28);   /* SIZE | OPTION */
                    ST(S_SDR4) = ((u32)SDH16(0x2C) << 16) | SDH16(0x2E);   /* ERR_STS1 | ERR_STS2 */
                    ST(S_SDR5) = ((u32)SDH16(0x08) << 16) | SDH16(0x0A);   /* STOP | SECCNT */
                    ST(S_SDR6) = ((u32)SDH16(0x0C) << 16) | SDH16(0x0E);   /* RSP0 | RSP1 */
                    ST(S_SDR7) = R32(0x8EAD13A0u);   /* driver lock owner words, AUDIO_ROUTE_RE sec 6 */
                }
#ifdef SD_FASTSTUCK
                if (!au_up) fs_watch();                /* one caller only: with audio up, the audio task runs it */
#endif
                dly_tsk(1);
                continue;
            }
#ifdef SD_LEAVEFIX
            if (src_left || stop_req()) { LT[LT_DROPPED]++; r_r = rr + 1; continue; }   /* leaving: no more decoding; the reader sees the same words and exits by itself */
#endif
            {
                volatile struct sfe *e = &r_e[rr % NSF];
                if (e->lost) { need_key = 1; asm_on = 0; }
                decode_slot((const u8 *)(SFRING + (rr % NSF) * SF_SIZE), e->hf, e->c_rd, e->total, e->seq);
            }
            r_r = rr + 1;
        }
        LTS(LT_T_LOOP);
        while ((R32(tcb_r + 4) >> 16) != 0x10) dly_tsk(1);       /* reader fully exited */
        LTS(LT_T_RDDONE);
        R32(tcb_r + 0x20) = saved_r;
        if (rd_err) err = rd_err;
    }

close:
    ST(ST_STAGE) = 3;
    ST(S_WALL_MS) += tick() - t_start;
    if (flags & F_DISPLAY) du_give_back();
    if (CTL_W(CTL_STOP) == 2) src_left = 1;            /* the car hook says the user left the source: stock owns the route */
    if (iis_on && iis_idx != 0xFF && !src_left) {      /* route back while the audio task still feeds the sink (fades to silence); if the user left, stock already chose */
        u32 t0 = tick();
        iis_st = 5;                                    /* the audio task must not re-assert IIS over this */
        SRCSEL((int)iis_idx);
        while (SRC_STATUS != ST(S_IIS_SAVED) && tick() - t0 < 2000) dly_tsk(10);
        ST(S_IIS_END) = (tick() - t0 + 1) | ((u32)SRC_STATUS << 24);
    }
    LTS(LT_T_ROUTE);
    if (g_scan) {                                       /* bench: does the FIFO still hold it? finds memory overwritten after the push */
        volatile u32 *d = (volatile u32 *)0x8F72FFD0u;
        const u32 *f = (const u32 *)AFIFO;
        s32 prev = (s16)(f[0] >> 16);
        for (i = 1; i < AFIFO_FRAMES; i++) {
            s32 v = (s16)(f[i] >> 16), j = v - prev;
            if (j > 3000 || j < -3000) { if (!d[3]) { d[4] = i; d[6] = f[i]; d[7] = f[i + 1]; } d[3]++; d[5] = i; }
            prev = v;
        }
        d[8] = a_wr & (AFIFO_FRAMES - 1);
    }
    if (au_up) {
        a_stop = 1;
        while ((R32(tcb_a + 4) >> 16) != 0x10) dly_tsk(1);
        R32(tcb_a + 0x20) = saved_a;
    }
    LTS(LT_T_AUDIO);
    if (ST(S_A_HANDLE) && (s32)ST(S_A_HANDLE) > 0) {
        int h = (int)ST(S_A_HANDLE);
        u32 t0, sub = ST(S_A_BLOCKS);
        ST(S_EOS_RET) = (u32)SND_DYD(h, (const void *)-1, 0, 0);
        t0 = tick();
        while (ST(S_EOS) == 0 && credits < sub + 2 && tick() - t0 < 1000) dly_tsk(1);
        ST(S_DRAIN_MS) = tick() - t0;
        ST(S_CLOSE) = ((u32)SND_STP(h) << 16) | ((u32)SND_CLS(h) & 0xFFFF);
    }
    LTS(LT_T_SINK);
    if (a_state == 3 && err == E_OK) err = E_DYD;
    (void)i;
    return err;
}

int main(void)
{
    u32 slot, gate, secs, wrap, nbuf, hint, seq, maxsf, err = E_OK, i, t_all = tick(), tv_ctx;
    (void)secs;

    st_begin(PROG_SDSTREAM);
    if (!ctl_ok()) { st_end(PROG_SDSTREAM, E_NOCTL); return 0; }
    slot = CTL_P(0); gate = CTL_P(1) & 15; g_steal = (CTL_P(1) >> 4) & 1; iis_req = (CTL_P(1) >> 5) & 1; iis_nohop = (CTL_P(1) >> 6) & 1; flags = CTL_P(2); secs = CTL_P(3); wrap = CTL_P(4); p1c = CTL_P(5);
    lrswap = CTL_P(6) & 255; sink_max = (CTL_P(6) >> 8) & 255; g_tone = (CTL_P(6) >> 16) & 1; g_scan = (CTL_P(6) >> 17) & 1; if (g_scan) { volatile u32 *d = (volatile u32 *)0x8F72FFD0u; for (i = 0; i < 12; i++) d[i] = 0; } blk = CTL_P(7); target = CTL_P(8); nbuf = CTL_P(9); hint = CTL_P(10); seq = CTL_P(11);
    maxsf = CTL_P(12);
    g_to_test = CTL_P(1) >> 8;
    if (!sink_max) sink_max = 3;
    if (slot > 1 || gate > 2 || flags > 1023 || wrap > 256 || p1c > 1 || lrswap > 1 || sink_max > 16 ||
        blk < 256 || blk > MAXBLK || (blk & 31) || target > AFIFO_FRAMES / 2 || (nbuf != 2 && nbuf != 3) ||
        hint < 1 || hint > 256 || ((flags & F_DISPLAY) && !(flags & F_DECODE))) {
        st_end(PROG_SDSTREAM, E_PARAM); return 0;
    }
    /* Memory replies are card writes: live emulator only, opt-in, never a card-image loop. */
    if ((flags & F_MEM) && (wrap || sdmem_target_init(&memory_service, slot, CTL_P(13), CTL_P(14), CTL_P(15)))) {
        st_end(PROG_SDSTREAM, E_PARAM); return 0;
    }
    g_dec = (flags & F_PREF) ? rnvo_decode : rnvf_decode;   /* --pref: the old decoder, A/B on the bench */
    for (i = 64; i < 128; i++) ST(i) = 0;           /* st_begin clears 0..63 only */
    ST(S_A_MIN) = 0xFFFFFFFFu; ST(S_R_MIN) = 0xFFFFFFFFu;
    { typedef char chk[(S_RESTARTS < 128) ? 1 : -1]; (void)sizeof(chk); }
    for (i = 0; i < LOGN * 8; i++) R32(RES_BASE + 4 * i) = 0;
    KEYREQ = 0; key_on = 0; key_slot.busy = 0;
    crc32_init();
    ST(S_CD_RAW) = R32(CD_GPIO);
    {
        u32 bit = (ST(S_CD_RAW) & (slot ? 0x800u : 0x400u)) ? 1 : 0;
        if ((gate == 1 && !bit) || (gate == 2 && bit)) { st_end(PROG_SDSTREAM, E_NOCARD); return 0; }
    }
    if (flags & F_DECODE) rnv_tables_init();
    g_slot = slot; g_wrap = wrap; g_secs = secs; g_hint = hint; g_seq = seq; g_maxsf = maxsf; g_nbuf = nbuf; g_t_all0 = t_all;
    tv_ctx = ((flags & F_STANDBY) && !((CTL_P(6) >> 18) & 1)) ? 7u : SRC_CTX;     /* standby = the car: runs from boot, plays only while the TV source is up */
    while (1) {
        if (flags & F_STANDBY) {
            /* the source is someone else's until the user comes to TV. Nothing here touches the sink, the display or the SD card. */
            u32 ok_t = tick(), done = 0;
            ST(ST_STAGE) = 4;
            while (!done) {
                if (stop_req()) { err = E_STOPPED; done = 2; break; }
                if (secs_up()) { err = E_OK; done = 2; break; }
                key_service();
                ST(ST_HB) = tick();
                if (SRC_CTX != tv_ctx || SRC_OVERLAY) ok_t = tick();
                else if (tick() - ok_t >= 800) done = 1;       /* let the TV screen finish its own setup first */
                dly_tsk(10);
            }
            if (done == 2) break;
        }
        err = session();
        if (!(flags & F_STANDBY) || err == E_OK || err == E_STOPPED) break;
        /* left the source, or a fault (sink busy, reads failing): try again. */
        if (err != E_SRC_LEFT) { ST(S_RESTARTS)++; sleep_ms(1000); }
    }
#ifdef SD_LEAVETRACE
    {   /* what the firmware does to array 0 after we let go */
        u32 t0 = tick();
        while (tick() - t0 < 1500) { lt_sample(); dly_tsk(2); }
        LT[LT_T_END] = tick();
    }
#endif
    st_end(PROG_SDSTREAM, err);
    return 0;
}
