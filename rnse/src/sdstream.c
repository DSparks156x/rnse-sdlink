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

enum { F_DECODE = 1, F_AUDIO = 2, F_DISPLAY = 4, F_TLVCRC = 8, F_PAYCRC = 16, F_PACE = 32, F_PREF = 64, F_MEM = 128 };
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
       S_FL_N, S_FL_WAIT, S_FL_F14, S_FL_F20, S_FL_F34, S_FL_F50, S_FL_MAX, S_FL_BLK };
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
static u32 a_wr, a_rd;               /* FIFO indices (frames), monotonic */
static int a_state;                  /* 0 off, 1 prebuffer, 2 running, 3 dead */
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
static u32 iis_req, iis_on, iis_idx, iis_t0, iis_check, src_left, src_saved;
#define SRC_CTX     (*(volatile u8 *)0x8E9DFF52u)   /* 07 = TV, 01 = FM, 06 = MP3 */

static void a_service(void);

static void iis_service(void)
{
    if (!iis_req) return;
    if (!iis_on) {
        if (ST(S_A_BLOCKS) < 15) return;               /* SSI must run first: an idle SSI makes the DSP fall back */
        ST(S_IIS_SAVED) = SRC_CACHE;
        src_saved = SRC_CTX;
        iis_idx = (u32)SRCIDX((int)ST(S_IIS_SAVED)) & 0xFF;
        ST(S_SSI_CR) = R32(SSI_CR); ST(S_SSI_SR) = R32(SSI_SR);
        ST(S_E219_A) = *(volatile u8 *)0x8E9E0219u;
        if (ST(S_IIS_SAVED) != 3) {                    /* hop through FM's input first: silent if going straight from TV (bench 2026-11-05) */
            SRCSEL(2);
            for (iis_t0 = tick(); SRC_STATUS != 3 && tick() - iis_t0 < 1500; dly_tsk(10)) a_service();
            for (iis_t0 = tick(); tick() - iis_t0 < 300; dly_tsk(10)) a_service();
        }
        ST(S_IIS_RET) = (u32)SRCSEL(3);
        iis_on = 1; iis_t0 = iis_check = tick();
        ST(S_E219_B) = *(volatile u8 *)0x8E9E0219u;
        return;
    }
    if (!ST(S_IIS_MS) && SRC_STATUS == 4) ST(S_IIS_MS) = tick() - iis_t0 + 1;
    if (tick() - iis_check >= 1000) {                  /* the firmware re-selects on overlays, TV re-entry: re-assert */
        iis_check = tick();
        if (SRC_CTX != src_saved) { if (SRC_OVERLAY == 0) src_left = 1; return; }   /* user changed source: stop, never fight it */
        if (SRC_CACHE != 4 && SRC_OVERLAY == 0) { SRCSEL(3); ST(S_IIS_REISSUE)++; }
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
    a_wr += frames;
    ST(S_A_IN) += frames;
}

static u32 a_rebuf;

/* Rate match. The sink's SSI runs at ~48.11 kHz, the Pi sends 48.00 (bench 2026-11-05: 90211 blocks x 960 in 1800 s, and 41 underruns =
 * one per 44 s = a 100 ms gap each time, which is what the FIFO drains at). So the FIFO is read through a linear interpolator whose step
 * is steered by the fill level: step = 1 + (P on filtered error + I), Q16, 1 unit = 15.3 ppm. Clamped to +-0.4%. I learns only near the target:
 * after a stall the error is transient, not a clock offset. */
#define RS_MAX 262   /* 0.4%: at 1% a refill is audibly slow */
static u32 rs_frac;                  /* position between a_rd and a_rd + 1, Q16 */
static s32 rs_step, rs_ema, rs_int;   /* set in main: initialised statics would change the image in RAM, and the upload skip check would then fail */
static void rs_steer(u32 fill)
{
    s32 err = (s32)fill - (s32)target, x;
    rs_ema += ((err << 8) - rs_ema) >> 6;            /* tau ~64 blocks = 1.3 s; arrival jitter is several ms */
    if (err > -1600 && err < 1600) rs_int += err;
    if (rs_int > RS_MAX * 4096) rs_int = RS_MAX * 4096;
    if (rs_int < -RS_MAX * 4096) rs_int = -RS_MAX * 4096;
    x = (rs_ema >> 8) / 3 + (rs_int >> 12);
    if (x > RS_MAX) x = RS_MAX;
    if (x < -RS_MAX) x = -RS_MAX;
    rs_step = 65536 + x;
    ST(S_FL_BLK) = (u32)x;
}
static inline s32 lerp16(s32 a, s32 b, s32 f) { return a + (((b - a) * f) >> 15); }
static u32 rs_fill_block(u32 *dst, const u32 *fifo)  /* blk frames out; returns frames consumed */
{
    u32 i, pos = rs_frac;
    for (i = 0; i < blk; i++) {
        u32 k = a_rd + (pos >> 16);
        u32 a = fifo[k & (AFIFO_FRAMES - 1)], b = fifo[(k + 1) & (AFIFO_FRAMES - 1)];
        s32 f = (s32)((pos & 0xFFFF) >> 1);
        s32 l = lerp16((s16)(a >> 16), (s16)(b >> 16), f), r = lerp16((s16)a, (s16)b, f);
        dst[i] = ((u32)l << 16) | ((u32)r & 0xFFFF);
        pos += (u32)rs_step;
    }
    rs_frac = pos & 0xFFFF;
    return pos >> 16;
}

static void a_service(void)
{
    u32 fill;
    if (a_state != 1 && a_state != 2) return;
    fill = a_wr - a_rd;
    if (a_state == 1 && fill >= target) a_state = 2;
    while (a_state == 2 && credits > ST(S_A_BLOCKS)) {
        u32 *dst = (u32 *)(TONE_BUF + a_k * MAXBLK * 4), n, i;
        const u32 *fifo = (const u32 *)AFIFO;
        u32 rs = p1c != 0, need = rs ? ((rs_frac + blk * (u32)rs_step) >> 16) + 2 : blk, used, keep_frac = rs_frac;
        int r;
        if (a_rebuf && fill >= target) a_rebuf = 0;    /* jitter buffer full again: play */
        n = a_rebuf ? 0 : (fill < need ? fill : blk);
        if (n < blk && ST(S_A_BLOCKS) + 2 > credits) break;  /* sink still holds a block: wait for more data */
        if (n == blk && rs) used = rs_fill_block(dst, fifo);
        else {
            for (i = 0; i < n; i++) dst[i] = fifo[(a_rd + i) & (AFIFO_FRAMES - 1)];
            for (; i < blk; i++) dst[i] = 0;           /* sink about to run dry: pad with silence, keep the SSI running */
            used = n;
        }
        if (n < blk && !a_rebuf) { ST(S_A_UNDER)++; a_rebuf = 1; rs_frac = 0; }   /* rebuffer behind silence, or supply = demand keeps it empty */
        r = SND_DYD((int)ST(S_A_HANDLE), dst, (int)(blk * 4), 0);   /* (h, buf, len, aux): arg 2 is alignment-checked + memcpy src */
        if (r != 0) { rs_frac = keep_frac; if (r == -2) break; a_state = 3; break; }
        a_rd += used; fill -= used;
        if (rs && !a_rebuf) rs_steer(fill);
        ST(S_A_BLOCKS)++;
        a_k = (a_k + 1) & (NBLK - 1);
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

static void sleep_ms(u32 ms) { u32 t0 = tick(); while (tick() - t0 < ms) { a_service(); dly_tsk(1); } }

/* ---------------------------------------------------------------- video buffers */
/* Fixed-rate presentation. The DU latches DSA at frame start, so two flips inside one refresh lose a frame and the next refresh repeats:
 * that is judder on an otherwise 60 fps stream. A decoded frame waits until FLIP_GAP after the previous flip. GAP is a little under the
 * 16.67 ms period so a late frame catches up over a few frames rather than leaving the phase stuck late. */
#define FLIP_GAP (SLOT_COUNTS - SLOT_COUNTS / 40)   /* 16.25 ms */
static u32 fl_last, fl_have;
static void flip_paced(u32 phys)
{
    if (fl_have) {
        u32 d = now_counts() - fl_last;
        if (d < FLIP_GAP) {
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
#define READ_TO_MS   150
static u32 g_to_test;                                   /* bench: every Nth read gets a 1 ms timeout, to exercise the timeout path */
static int sd_read_to(u32 slot, u32 lba, u32 buf, u32 n)
{
    volatile int cnt = (int)n;
    int r, to = READ_TO_MS;
    if (g_to_test && ST(S_RD_CALLS) % g_to_test == g_to_test - 1) to = 1;
    SD_BUSY(slot) = 1;
    r = ((int (*)(u32, void *, volatile int *, u32, int))FW_SD_RDCORE)(slot, (void *)buf, &cnt, lba, to);
    SD_BUSY(slot) = 0;
    return ((short)r != 0 || cnt != (int)n) ? -1 : 0;
}
#define SD_VARIANT0 0x8EAD1254u                         /* slot 0 clock: byte +2 = 1 -> /4 = 12.5 MHz, bench-proven */
#define SDH16(o) (*(volatile unsigned short *)(0xFF90A000u + (o)))

static int timed_read(u32 slot, u32 lba, u32 buf, u32 n)
{
    u32 c0 = now_counts(), c1, d;
    int r;
    if (!g_wrap && !g_slot) R32(SD_VARIANT0) = 0x00000100u;   /* live stream, /4. The driver does variant = max(variant, retries), cleared only on reinit: one error would otherwise leave the slot at /8 (2.8 MB/s) for good */
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
#define NSF         4u
#define SF_SIZE     0x20000u
struct sfe { u32 hf, sseq, total, c_rd, lost, seq; };
static volatile struct sfe r_e[NSF];
static volatile u32 r_w, r_r, rd_done, rd_err;
static u32 g_secs, g_hint, g_seq, g_maxsf, g_t_start;
static u32 need_key;
static int (*g_dec)(rnv_state *, const rnv_u8 *, rnv_u32);
static u32 g_nbuf;

void reader_main(void)
{
    u32 rdfail = 0, retry = 0, lost = 0, due = 0, have_due = 0, seq = g_seq, hint = g_hint;
    while (1) {
        u32 w = r_w, buf = SFRING + (w % NSF) * SF_SIZE, slotseq = g_wrap ? seq % g_wrap : seq, lba = 0x10000u + (slotseq & 255u) * 256u, hf, total, sseq;
        const u8 *sf = (const u8 *)buf;
        u32 c0 = now_counts(), c_rd = 0;
        int r;
        if (stop_req()) { rd_err = E_STOPPED; break; }
        if (src_left) { rd_err = E_SRC_LEFT; break; }
        if (tick() - g_t_start >= g_secs * 1000u) break;
        if (g_maxsf && ST(S_SF_TOTAL) >= g_maxsf) break;
        ST(ST_HB) = tick();
        ST(S_SEQ) = seq;
        a_service();
        iis_service();
        if (w - r_r >= NSF) { dly_tsk(1); continue; }  /* decoder behind: hold the read, audio still services */
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
        ST(S_LAST_FLAGS) = hf; ST(S_LAST_FID) = be16(sf + 54);
        if (hf & H_EXPIRED) { ST(S_EXPIRED)++; seq = be32(sf + 64); lost = 1; continue; }
        if (hf & (H_IDLE | H_NOTREADY)) {
            ST(S_RETRY)++; logent(seq, hf, EV_RETRY, c_rd, 0, 0);
            if (!g_wrap && (hf & H_NOTREADY)) {          /* Teensy reports its newest complete seq: resync at once, also after a feeder restart */
                u32 nw = be32(sf + 64);
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
        if (total < 1 || total > 256) { ST(S_HDR_BAD)++; seq++; lost = 1; continue; }
        if (total > hint) {
            r = timed_read(g_slot, lba + hint, buf + hint * 512, total - hint);
            c_rd = now_counts() - c0;
            if (r) { if (++rdfail >= 5) { rd_err = E_READ; break; } continue; }
        }
        {
            u32 nh = be16(sf + 42);
            hint = nh ? nh : total;
            if (hint > 256) hint = 256;
        }
        if (hf & H_ADISCONT) a_rd = a_wr;
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
        a_service();
        if (flags & F_MEM) {
            sdmem_pump(&memory_service, ST(S_SF_TOTAL));
            ST(S_MEM_CMD)=memory_service.commands; ST(S_MEM_BAD)=memory_service.bad;
            ST(S_MEM_TX)=memory_service.tx; ST(S_MEM_RETRY)=memory_service.retries; ST(S_MEM_IO)=memory_service.io_errors;
        }
        {
            volatile struct sfe *e = &r_e[w % NSF];
            e->hf = hf; e->sseq = sseq; e->total = total; e->c_rd = c_rd; e->lost = lost; e->seq = seq;
            lost = 0;
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
            while ((s32)(now_counts() - due) < 0) { a_service(); dly_tsk(1); }
        } else dly_tsk(1);
    }
    rd_done = 1;
    ((void (*)(void))FW_EXT_TSK)();
}

/* entered by the kernel on the slot's small stack: move to our own 64 KiB, under the sink ring and above AFIFO */
__asm__(".text\n.align 2\n_reader_entry:\n  mov.l 1f,r15\n  mov.l 2f,r0\n  jmp @r0\n  nop\n  .align 2\n1: .long 0x8F9D0000\n2: .long _reader_main\n");
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
                        flip_paced(DSA_PHYS(out));
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
    }
    ST(S_SF_OK)++; ST(S_SF_TOTAL)++;
    logent(seq, hf, ev, c_rd, c_dec, total);
}

int main(void)
{
    u32 slot, gate, secs, wrap, nbuf, hint, seq, maxsf, err = E_OK, i, t_start = tick();
    u32 steal;
    int (*dec)(rnv_state *, const rnv_u8 *, rnv_u32);

    st_begin(PROG_SDSTREAM);
    if (!ctl_ok()) { st_end(PROG_SDSTREAM, E_NOCTL); return 0; }
    slot = CTL_P(0); gate = CTL_P(1) & 15; steal = (CTL_P(1) >> 4) & 1; iis_req = (CTL_P(1) >> 5) & 1; flags = CTL_P(2); secs = CTL_P(3); wrap = CTL_P(4); p1c = CTL_P(5);
    lrswap = CTL_P(6); blk = CTL_P(7); target = CTL_P(8); nbuf = CTL_P(9); hint = CTL_P(10); seq = CTL_P(11);
    maxsf = CTL_P(12);
    g_to_test = CTL_P(1) >> 8;
    rs_step = 65536; rs_ema = 0; rs_int = 0; rs_frac = 0;
    if (slot > 1 || gate > 2 || flags > 511 || secs < 1 || secs > 3600 || wrap > 256 || p1c > 1 || lrswap > 1 ||
        blk < 256 || blk > MAXBLK || (blk & 31) || target > AFIFO_FRAMES / 2 || (nbuf != 2 && nbuf != 3) ||
        hint < 1 || hint > 256 || ((flags & F_DISPLAY) && !(flags & F_DECODE))) {
        st_end(PROG_SDSTREAM, E_PARAM); return 0;
    }
    /* Memory replies are card writes: live emulator only, opt-in, never a card-image loop. */
    if ((flags & F_MEM) && (wrap || sdmem_target_init(&memory_service, slot, CTL_P(13), CTL_P(14), CTL_P(15)))) {
        st_end(PROG_SDSTREAM, E_PARAM); return 0;
    }
    dec = (flags & F_PREF) ? rnvp_decode : rnvo_decode;
    for (i = 64; i < 128; i++) ST(i) = 0;           /* st_begin clears 0..63 only */
    ST(S_A_MIN) = 0xFFFFFFFFu;
    { typedef char chk[(S_MEM_IO < 128) ? 1 : -1]; (void)sizeof(chk); }
    for (i = 0; i < LOGN * 8; i++) R32(RES_BASE + 4 * i) = 0;
    crc32_init();
    ST(S_CD_RAW) = R32(CD_GPIO);
    {
        u32 bit = (ST(S_CD_RAW) & (slot ? 0x800u : 0x400u)) ? 1 : 0;
        if ((gate == 1 && !bit) || (gate == 2 && bit)) { st_end(PROG_SDSTREAM, E_NOCARD); return 0; }
    }

    /* ---- audio sink (tone48's sequence) */
    if (flags & F_AUDIO) {
        int h;
        u32 t0;
        ST(S_SINK0) = R32(DMDL_HANDLE);
        ST(S_P904_A) = R32(0xFF909004u);
        if (iis_req) ((void (*)(void))0x8062D848u)();   /* stock player's once-per-boot enable (FF909004 bit 2); read from code, see AUDIO_ROUTE_RE (c) */
        ST(S_P904_B) = R32(0xFF909004u);
        /* Snd_opn fails while the player holds the stream (bench 2026-11-05): --steal closes the player's handle first */
        if (steal && (ST(S_SINK0) >> 24) == 2u) {
            SND_STP((int)ST(S_SINK0));
            SND_CLS((int)ST(S_SINK0));
            dly_tsk(5);
        }
        pblock[0] = (u32)cb; pblock[1] = 0x100; pblock[2] = 9; pblock[3] = 0; pblock[4] = 0; pblock[5] = 1;
        pblock[6] = 0; pblock[7] = p1c; pblock[8] = 0;
        h = SND_OPN(pblock);
        if (h <= 0) { ST(S_A_HANDLE) = (u32)h; st_end(PROG_SDSTREAM, E_OPN); return 0; }
        ST(S_A_HANDLE) = (u32)h;
        t0 = tick();
        while (credits < 2 && tick() - t0 < 300) dly_tsk(1);
        if (credits < 2) { err = E_NO_CREDIT; goto close; }
        a_state = 1;
    }

    /* ---- video */
    if (flags & F_DECODE) {
        rnv_tables_init();
        rnv_state_init(&vs, (u8 *)FB0, (u8 *)FB1, (u8 *)FB2, (int)nbuf);
        if (flags & F_DISPLAY) {
            front = (u32)vs.buf[vs.last];
            wb_drain(front + FB_BYTES - 4);
            ST(S_DU) = 1 + (u32)du_takeover(DSA_PHYS(front));
            replaced[0] = replaced[1] = replaced[2] = tick() - REUSE_MS;
        }
    }

    ST(ST_STAGE) = 2;
    g_slot = slot; g_wrap = wrap; g_secs = secs; g_hint = hint; g_seq = seq; g_maxsf = maxsf; g_dec = dec; g_nbuf = nbuf;
    g_t_start = t_start = tick();
    {
        u32 tcb = TCB_OF(RD_TASK), saved_ent = R32(tcb + 0x20);
        int sr;
        if ((R32(tcb + 4) >> 16) != 0x10) { err = E_READ; goto close; }   /* slot not dormant: leave it alone */
        R32(tcb + 0x20) = (u32)reader_entry;
        sr = ((int (*)(int, int))FW_STA_TSK)((int)RD_TASK, 0);
        if (sr) { R32(tcb + 0x20) = saved_ent; err = E_READ; goto close; }
        while (1) {
            u32 rr = r_r;
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
                dly_tsk(1);
                continue;
            }
            {
                volatile struct sfe *e = &r_e[rr % NSF];
                if (e->lost) { need_key = 1; asm_on = 0; }
                decode_slot((const u8 *)(SFRING + (rr % NSF) * SF_SIZE), e->hf, e->c_rd, e->total, e->seq);
            }
            r_r = rr + 1;
        }
        while ((R32(tcb + 4) >> 16) != 0x10) dly_tsk(1);       /* reader fully exited */
        R32(tcb + 0x20) = saved_ent;
        if (rd_err) err = rd_err;
    }

close:
    ST(ST_STAGE) = 3;
    ST(S_WALL_MS) = tick() - t_start;
    if (flags & F_DISPLAY) { du_restore(); ST(S_DU) |= 0x100; }
    if (iis_on && iis_idx != 0xFF && !src_left) {      /* route back before the sink stops; if the user left, stock already chose */
        u32 t0 = tick();
        SRCSEL((int)iis_idx);
        while (SRC_STATUS != ST(S_IIS_SAVED) && tick() - t0 < 2000) dly_tsk(10);
        ST(S_IIS_END) = (tick() - t0 + 1) | ((u32)SRC_STATUS << 24);
    }
    if (ST(S_A_HANDLE) && (s32)ST(S_A_HANDLE) > 0) {
        int h = (int)ST(S_A_HANDLE);
        u32 t0, sub = ST(S_A_BLOCKS);
        ST(S_EOS_RET) = (u32)SND_DYD(h, (const void *)-1, 0, 0);
        t0 = tick();
        while (ST(S_EOS) == 0 && credits < sub + 2 && tick() - t0 < 1000) dly_tsk(1);
        ST(S_DRAIN_MS) = tick() - t0;
        ST(S_CLOSE) = ((u32)SND_STP(h) << 16) | ((u32)SND_CLS(h) & 0xFFFF);
    }
    if (a_state == 3 && err == E_OK) err = E_DYD;
    st_end(PROG_SDSTREAM, err);
    return 0;
}
