/* decbench: RNV2 decode timing + correctness on the real SH-4A, as task 159 (RAM program @8F740000).
 *
 * Input: an RNVS test-stream blob (mkdecstream.py) at DEC_STREAM 8F780000, either written there over CAN by the
 * host (src 0) or read from an SD card with the stock raw-read API 80653598 (src 1; the card is only READ).
 * For each loop the decoder is reset to black and every frame decoded; the decode call alone is timed with TMU0
 * (read-only: the kernel's 1 ms tick timer, 12.4875 MHz, via now_counts(); no timer is started or changed).
 * Per frame: first-loop time, min and max over loops, CRC-32 (zlib) of the decoded 768000-byte YUY2 frame. The CRC
 * is computed OUTSIDE the timed region and compared on the unit against the expected table in the blob (made by
 * the PC decoder). The task yields (dly_tsk(1)) after every frame.
 * Optional display (p8 = 1): Doom-style array-0 takeover in YUY2, buffers FB0..FB2, flip after each frame, a
 * buffer is rewritten only >= 17 ms after the flip that replaced it (DU_DESIGN 3b: one vblank passed), full
 * restore on exit. Default OFF.
 *
 * CTL: p0 src (0 RAM blob, 1 SD), p1 slot, p2 lba, p3 sectors, p4 loops (1..100), p5 variant (0 opt, 1 opt+PREF,
 *      2 reference C), p6 nbuf (2/3; reference = 2), p7 crc (0 none, 1 every loop, 2 first loop only),
 *      p8 display (0/1), p9 pace ms (display, 0..1000), p10 hold ms at end (display, 0..30000), p11 max frames
 *      (0 = all), p12 SD card-detect gate (0 none, 1 slot bit high, 2 low)
 * STAT: see D_* (mirrored in decbench.py); per frame 4 words at RES_BASE: t_first, t_min, t_max (TMU counts), crc */
#include "avsd.h"
#include "rnv2sh.h"

#define PROG_DECBENCH 5
#define MAXFR 2048
#define NBIN 64
#define BIN_COUNTS 3122u            /* 0.25 ms */
#define REUSE_MS 17u

enum { D_NFRAMES = 8, D_BLOB_LEN, D_CRC_CALC, D_CRC_EXP, D_BLOB_OK, D_DECODES, D_LOOPS_DONE, D_DEC_ERR, D_FIRST_ERR,
       D_CRC_MISM, D_FIRST_MISM, D_LOOP_MISM, D_TMIN, D_TMAX, D_TSUM_LO, D_TSUM_HI, D_NTIMED, D_VARIANT, D_NBUF,
       D_SD_RET, D_SD_MS, D_DU, D_FLIPS, D_REUSE_WAITS, D_MS_TOTAL, D_HAVE_TABLE, D_CD_RAW, D_PARAM_LOOPS,
       D_PARAM_CRC, D_PARAM_DISPLAY, D_ERR_BLK, D_LAST_FRAME, D_BLACK_MS };
enum { E_OK, E_NOCTL, E_PARAM, E_NOCARD, E_SDREAD, E_BLOB, E_BLOBCRC, E_STOPPED };
#define HIST 64                       /* STAT words 64..127: histogram of all timed decodes, 0.25 ms bins */

static u32 fptr[MAXFR], flen[MAXFR];
static rnv_state st;
static rnvref_dec rd;
static u32 replaced[3];               /* tick when buffer i stopped being the front buffer */

static u32 be32(const u8 *p) { return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3]; }

static int sd_load(u32 slot, u32 lba, u32 nsect)
{
    u32 done = 0, t0 = tick();
    int r = 0;
    while (done < nsect) {
        u32 n = nsect - done > 4096 ? 4096 : nsect - done;
        r = SD_READ((int)slot, (int)(lba + done), (void *)(DEC_STREAM + done * 512), (int)n);
        if (r) break;
        done += n;
        ST(ST_HB) = tick();
    }
    ST(D_SD_RET) = (u32)r;
    ST(D_SD_MS) = tick() - t0;
    oc_inval(DEC_STREAM, nsect * 512);
    return r;
}

/* CCR 00000101 on this unit (read 2026-11-05): WT=0, CB=0: P1 write-through, P0 copy-back; MMUCR.AT=0, so P0 = phys. Variant 3 decodes through P0 and writes the frame back. */
#define P0A(x) ((x) & 0x1FFFFFFFu)
static void oc_wb(u32 a, u32 n)
{
    u32 e = a + n;
    for (a &= ~31u; a < e; a += 32) __asm__ volatile("ocbwb @%0" : : "r"(a) : "memory");
}

static int fb_index(const u8 *p) { return p == (u8 *)FB0 ? 0 : p == (u8 *)FB1 ? 1 : 2; }

/* display: before writing buffer p, make sure >= 17 ms passed since it stopped being displayed */
static void wait_reuse(const u8 *p, int disp)
{
    u32 i = (u32)fb_index(p);
    if (!disp) return;
    if (tick() - replaced[i] < REUSE_MS) {
        ST(D_REUSE_WAITS)++;
        while (tick() - replaced[i] < REUSE_MS) dly_tsk(1);
    }
}

/* ---- p13 = 1: what a whole 6000-block frame costs on each memory path, min of 5, ms*1000 at RES_BASE+0xD040. No display. */
#define MT_R 8
#define WD(p) (*(volatile u32 *)(p))
static volatile u32 mt_sink;
static void __attribute__((noinline)) mt_write_wt(u32 d)
{ u32 y, x, r; for (y = 0; y < 480; y += 8) for (x = 0; x < 1600; x += 16) { u32 *q = (u32 *)(d + y * 1600 + x); for (r = 0; r < MT_R; r++) { q[0] = r; q[1] = r; q[2] = r; q[3] = r; q += 400; } } }
static void __attribute__((noinline)) mt_read(u32 s)
{ u32 y, x, r, a = 0; for (y = 0; y < 480; y += 8) for (x = 0; x < 1600; x += 16) { const u32 *q = (const u32 *)(s + y * 1600 + x); for (r = 0; r < MT_R; r++) { a += q[0] + q[1] + q[2] + q[3]; q += 400; } } mt_sink = a; }
static void __attribute__((noinline)) mt_copy(u32 d, u32 s)
{ u32 y, x, r; for (y = 0; y < 480; y += 8) for (x = 0; x < 1600; x += 16) { u32 *q = (u32 *)(d + y * 1600 + x); const u32 *p = (const u32 *)(s + y * 1600 + x); for (r = 0; r < MT_R; r++) { u32 a = p[0], b = p[1], c = p[2], e = p[3]; q[0] = a; q[1] = b; q[2] = c; q[3] = e; q += 400; p += 400; } } }
static void __attribute__((noinline)) mt_copy_line_movca(u32 d0, u32 s)   /* P0: 32-B line copy, movca + ocbwb per line */
{ u32 y, x; for (y = 0; y < 480; y++) for (x = 0; x < 1600; x += 32) { u32 *q = (u32 *)(d0 + y * 1600 + x); const u32 *p = (const u32 *)(s + y * 1600 + x); u32 a = p[0];
    __asm__ volatile("mov %1,r0\n\tmovca.l r0,@%0" : : "r"(q), "r"(a) : "r0", "memory");
    q[1] = p[1]; q[2] = p[2]; q[3] = p[3]; q[4] = p[4]; q[5] = p[5]; q[6] = p[6]; q[7] = p[7];
    __asm__ volatile("ocbwb @%0" : : "r"(q) : "memory"); } }
static void __attribute__((noinline)) mt_copy_line_sq(u32 d, u32 s)        /* store queue: 32-B burst per line */
{ u32 y, x; u32 phys = d & 0x1FFFFFFFu; R32(0xFF000038u) = (phys >> 24) & 0x1Cu; R32(0xFF00003Cu) = (phys >> 24) & 0x1Cu;
  for (y = 0; y < 480; y++) for (x = 0; x < 1600; x += 32) { u32 o = (phys + y * 1600 + x) & 0x03FFFFE0u; volatile u32 *q = (volatile u32 *)(0xE0000000u | o); const u32 *p = (const u32 *)(s + y * 1600 + x);
    q[0] = p[0]; q[1] = p[1]; q[2] = p[2]; q[3] = p[3]; q[4] = p[4]; q[5] = p[5]; q[6] = p[6]; q[7] = p[7];
    __asm__ volatile("pref @%0" : : "r"(q) : "memory"); }
  R32(0xFF000038u) = 0; R32(0xFF00003Cu) = 0; }
static void __attribute__((noinline)) mt_copy_line_wt(u32 d, u32 s)        /* P1 write-through, whole lines */
{ u32 y, x; for (y = 0; y < 480; y++) for (x = 0; x < 1600; x += 32) { u32 *q = (u32 *)(d + y * 1600 + x); const u32 *p = (const u32 *)(s + y * 1600 + x);
    u32 a = p[0], b = p[1], c = p[2], e = p[3], f = p[4], g = p[5], h = p[6], i = p[7]; q[0] = a; q[1] = b; q[2] = c; q[3] = e; q[4] = f; q[5] = g; q[6] = h; q[7] = i; } }
static void __attribute__((noinline)) mt_flush_only(u32 a) { oc_wb(a, FB_BYTES); }
static void __attribute__((noinline)) mt_alu(void) { u32 i, a = 1; for (i = 0; i < 1000000u; i++) { a += a >> 1; __asm__ volatile("" : "+r"(a)); } mt_sink = a; }
static void __attribute__((noinline)) mt_pref(u32 s)   /* PREF every line only: does it block? */
{ u32 y, x; for (y = 0; y < 480; y++) for (x = 0; x < 1600; x += 32) __builtin_prefetch((const void *)(s + y * 1600 + x)); }
static void __attribute__((noinline)) mt_copy_pref(u32 d, u32 s)   /* decoder copy, PREF the next pair's 8 rows before each pair */
{ u32 y, x, r; for (y = 0; y < 480; y += 8) for (x = 0; x < 1600; x += 16) { u32 *q = (u32 *)(d + y * 1600 + x); const u32 *p = (const u32 *)(s + y * 1600 + x);
    if (!(x & 16)) { const u8 *n = (const u8 *)p + 32; for (r = 0; r < MT_R; r++) __builtin_prefetch(n + r * 1600); }
    for (r = 0; r < MT_R; r++) { u32 a = p[0], b = p[1], c = p[2], e = p[3]; q[0] = a; q[1] = b; q[2] = c; q[3] = e; q += 400; p += 400; } } }
static void __attribute__((noinline)) mt_copy_pref1(u32 d, u32 s)  /* copy rows, PREF next row of this block before each row */
{ u32 y, x, r; for (y = 0; y < 480; y += 8) for (x = 0; x < 1600; x += 16) { u32 *q = (u32 *)(d + y * 1600 + x); const u32 *p = (const u32 *)(s + y * 1600 + x);
    for (r = 0; r < MT_R; r++) { u32 a, b, c, e; __builtin_prefetch(p + 400); a = p[0]; b = p[1]; c = p[2]; e = p[3]; q[0] = a; q[1] = b; q[2] = c; q[3] = e; q += 400; p += 400; } } }
static void __attribute__((noinline)) mt_compute(void) { volatile u32 sink; u32 i, a = 1; for (i = 0; i < 24000u; i++) { u32 k; for (k = 0; k < 90u; k++) { a += a >> 1; __asm__ volatile("" : "+r"(a)); } } sink = a; (void)sink; }
static void __attribute__((noinline)) mt_compute_pref(u32 s)  /* 90-loop compute per line, with a PREF of a new line each time: full overlap? */
{ volatile u32 sink; u32 i, a = 1; const u8 *p = (const u8 *)s; for (i = 0; i < 24000u; i++) { u32 k; __builtin_prefetch(p); p += 32; for (k = 0; k < 90u; k++) { a += a >> 1; __asm__ volatile("" : "+r"(a)); } } sink = a; (void)sink; }
#define MT_RUN(k, call) do { u32 n, best = ~0u; for (n = 0; n < 5; n++) { u32 c0; dly_tsk(1); c0 = now_counts(); call; c0 = now_counts() - c0; if (c0 < best) best = c0; } mt[k] = best; } while (0)
static void memtest(void)
{
    volatile u32 *mt = (volatile u32 *)(RES_BASE + 0xD040u);
    u32 i; for (i = 0; i < 16; i++) mt[i] = 0;
    MT_RUN(0, mt_alu());
    MT_RUN(1, mt_write_wt(FB2));
    MT_RUN(2, mt_read(FB0));
    MT_RUN(3, mt_copy(FB2, FB0));
    MT_RUN(4, (mt_copy(P0A(FB2), FB0), oc_wb(P0A(FB2), FB_BYTES)));
    MT_RUN(5, mt_flush_only(P0A(FB2)));
    MT_RUN(6, mt_copy_line_wt(FB2, FB0));
    MT_RUN(7, mt_copy_line_movca(P0A(FB2), FB0));
    MT_RUN(8, mt_copy_line_sq(FB2, FB0));
    MT_RUN(9, mt_read(FB0));
    MT_RUN(10, mt_pref(FB0));
    MT_RUN(11, mt_copy_pref(FB2, FB0));
    MT_RUN(12, mt_copy_pref1(FB2, FB0));
    MT_RUN(13, mt_compute());
    MT_RUN(14, mt_compute_pref(FB0));
}

/* ---- p13 & 2: SD reads running while decode is timed, as sdstream's reader does: is the in-stream decode time the bus, or something else? */
#define RD_TASK 0x2Fu
#define TCB_OF(id) (0x8E8D53ACu + (u32)(id) * 0x40u)
#define FW_STA_TSK 0x804D026Cu
#define FW_SD_RDCORE 0x8067D108u
#define SD_BUSY0 (*(volatile u32 *)0x8EAAC970u)
#define RBUF 0x8F900000u
static volatile u32 rd_stop, rd_done, rd_n, rd_bad, rd_counts;
void rd_main(void)
{
    u32 seq = 0, due = now_counts();
    while (!rd_stop) {
        volatile int cnt = 130;
        int r; u32 c0 = now_counts();
        R32(0x8EAD1254u) = 0x00000100u;
        SD_BUSY0 = 1;
        r = ((int (*)(u32, void *, volatile int *, u32, int))FW_SD_RDCORE)(0, (void *)RBUF, &cnt, 0x10000u + (seq & 255u) * 256u, 65);
        SD_BUSY0 = 0;
        rd_counts += now_counts() - c0;
        if ((short)r != 0 || cnt != 130) rd_bad++;
        oc_inval(RBUF, 130 * 512);
        rd_n++; seq++;
        due += 208125u;                       /* 16.667 ms */
        while ((s32)(now_counts() - due) < 0) dly_tsk(1);
    }
    rd_done = 1;
    ((void (*)(void))0x804D027Cu)();
}
__asm__(".text\n.align 2\n_rd_entry:\n  mov.l 1f,r15\n  mov.l 2f,r0\n  jmp @r0\n  nop\n  .align 2\n1: .long 0x8F9B0000\n2: .long _rd_main\n");
extern void rd_entry(void);
static u32 rd_saved;
static int rd_start(void)
{
    u32 tcb = TCB_OF(RD_TASK);
    if ((R32(tcb + 4) >> 16) != 0x10) return -1;
    rd_saved = R32(tcb + 0x20);
    R32(tcb + 0x20) = (u32)rd_entry;
    if (((int (*)(int, int))FW_STA_TSK)((int)RD_TASK, 0)) { R32(tcb + 0x20) = rd_saved; return -1; }
    return 0;
}
static void rd_finish(void)
{
    u32 tcb = TCB_OF(RD_TASK);
    volatile u32 *pr = (volatile u32 *)(RES_BASE + 0xD000u);
    rd_stop = 1;
    while ((R32(tcb + 4) >> 16) != 0x10) dly_tsk(1);
    R32(tcb + 0x20) = rd_saved;
    pr[5] = rd_n; pr[6] = rd_bad; pr[7] = rd_counts;
}

int main(void)
{
    u32 src, slot, lba, nsect, loops, variant, nbuf, crcm, disp, pace, hold, maxf, gate;
    u32 nf, dlen, doff, toff, i, lp, err = E_OK, last_flip = 0, front = 0, t_start;
    u32 sum_lo = 0, sum_hi = 0, tmin = 0xFFFFFFFFu, tmax = 0;
    const u8 *blob = (const u8 *)DEC_STREAM;
    volatile u32 *res = (volatile u32 *)RES_BASE;
    const u32 *exp = 0;

    st_begin(PROG_DECBENCH);
    for (i = HIST; i < HIST + NBIN; i++) ST(i) = 0;
    if (!ctl_ok()) { st_end(PROG_DECBENCH, E_NOCTL); return 0; }
    src = CTL_P(0); slot = CTL_P(1); lba = CTL_P(2); nsect = CTL_P(3); loops = CTL_P(4); variant = CTL_P(5);
    nbuf = CTL_P(6); crcm = CTL_P(7); disp = CTL_P(8); pace = CTL_P(9); hold = CTL_P(10); maxf = CTL_P(11);
    gate = CTL_P(12);
    if (variant == 2) nbuf = 2;
    ST(D_VARIANT) = variant; ST(D_NBUF) = nbuf; ST(D_PARAM_LOOPS) = loops; ST(D_PARAM_CRC) = crcm;
    ST(D_PARAM_DISPLAY) = disp;
    if (src > 1 || slot > 1 || (s32)lba < 0 || nsect > DEC_STREAM_MAX / 512 || (src == 1 && nsect < 1) ||
        loops < 1 || loops > 100 || variant > 4 || (nbuf != 2 && nbuf != 3) || crcm > 2 || disp > 1 ||
        pace > 1000 || hold > 30000 || maxf > MAXFR || gate > 2) { st_end(PROG_DECBENCH, E_PARAM); return 0; }

    crc32_init();
    if (src == 1) {
        u32 cd = R32(CD_GPIO), bit = (cd & (slot ? 0x800u : 0x400u)) ? 1 : 0;
        ST(D_CD_RAW) = cd;
        if ((gate == 1 && !bit) || (gate == 2 && bit)) { st_end(PROG_DECBENCH, E_NOCARD); return 0; }
        ST(ST_STAGE) = 2;
        if (sd_load(slot, lba, nsect)) { st_end(PROG_DECBENCH, E_SDREAD); return 0; }
    }

    /* ---- blob: 'RNVS' ver nframes data_off data_len data_crc table_off */
    ST(ST_STAGE) = 3;
    nf = be32(blob + 8); doff = be32(blob + 12); dlen = be32(blob + 16); toff = be32(blob + 24);
    ST(D_NFRAMES) = nf; ST(D_BLOB_LEN) = dlen; ST(D_CRC_EXP) = be32(blob + 20);
    if (be32(blob) != 0x524E5653u || nf < 1 || nf > MAXFR || doff < 32 || doff > 4096 || (doff & 3) ||
        dlen > DEC_STREAM_MAX - doff - 64 || (toff && (toff & 3 || toff + 4 * nf > doff))) {
        st_end(PROG_DECBENCH, E_BLOB); return 0;
    }
    {
        u32 c = 0, k;
        for (k = 0; k < dlen; k += 65536) {
            c = crc32_upd(c, blob + doff + k, dlen - k > 65536 ? 65536 : dlen - k);
            ST(ST_HB) = tick();
            dly_tsk(1);
        }
        ST(D_CRC_CALC) = c;
        if (c != be32(blob + 20)) { ST(D_BLOB_OK) = 0; st_end(PROG_DECBENCH, E_BLOBCRC); return 0; }
        ST(D_BLOB_OK) = 1;
    }
    {
        u32 pos = doff, end = doff + dlen;
        for (i = 0; i < nf; i++) {
            u32 n;
            if (pos + 4 > end) break;
            n = be32(blob + pos);
            if (n > end - pos - 4) break;
            fptr[i] = DEC_STREAM + pos + 4; flen[i] = n;
            pos += 4 + ((n + 3) & ~3u);
        }
        if (i != nf) { st_end(PROG_DECBENCH, E_BLOB); return 0; }
    }
    if (maxf && maxf < nf) nf = maxf;
    if (toff) { exp = (const u32 *)(blob + toff); ST(D_HAVE_TABLE) = 1; }
    for (i = 0; i < nf; i++) { res[4 * i] = 0; res[4 * i + 1] = 0xFFFFFFFFu; res[4 * i + 2] = 0; res[4 * i + 3] = 0; }

    /* ---- decoder */
    ST(ST_STAGE) = 4;
    rnv_tables_init();
    t_start = tick();
    if (variant == 2) rnvref_init(&rd, (u8 *)FB0, (u8 *)FB1);
    else if (variant == 3) rnv_state_init(&st, (u8 *)P0A(FB0), (u8 *)P0A(FB1), (u8 *)P0A(FB2), (int)nbuf);
    else rnv_state_init(&st, (u8 *)FB0, (u8 *)FB1, (u8 *)FB2, (int)nbuf);
    ST(D_BLACK_MS) = tick() - t_start;
    if (disp) {
        wb_drain(FB0 + FB_BYTES - 4); wb_drain(FB1 + FB_BYTES - 4); wb_drain(FB2 + FB_BYTES - 4);
        front = variant == 2 ? FB1 : (u32)st.buf[st.last];
        ST(D_DU) = 1 + (u32)du_takeover(DSA_PHYS(front));
        replaced[0] = replaced[1] = replaced[2] = tick() - REUSE_MS;
        last_flip = tick();
    }

    { volatile u32 *pr = (volatile u32 *)(RES_BASE + 0xD000u);   /* read-only register snapshot, read by decbench.py */
      pr[0] = R32(0xFF00001Cu); pr[1] = R32(0xFF000010u); pr[2] = R32(0xFF000038u); pr[3] = R32(0xFF00003Cu); pr[4] = R32(0xFF000024u); }
    if (CTL_P(13) & 1) memtest();
    if (CTL_P(13) & 2) { if (rd_start()) { st_end(PROG_DECBENCH, E_PARAM); return 0; } dly_tsk(30); }
    ST(ST_STAGE) = 5;
    t_start = tick();
    for (lp = 0; lp < loops && err == E_OK; lp++) {
        if (lp) {
            if (variant == 2) {
                if (disp) { wait_reuse((u8 *)FB0, 1); wait_reuse((u8 *)FB1, 1); }
                rnvref_init(&rd, (u8 *)FB0, (u8 *)FB1);
            } else {
                for (i = 0; i < nbuf; i++) if (disp && (u32)st.buf[i] != front) wait_reuse(st.buf[i], 1);
                /* black reset; with display on, the shown buffer turns black for one frame between loops */
                if (variant == 3) rnv_state_init(&st, (u8 *)P0A(FB0), (u8 *)P0A(FB1), (u8 *)P0A(FB2), (int)nbuf);
                else rnv_state_init(&st, (u8 *)FB0, (u8 *)FB1, (u8 *)FB2, (int)nbuf);
            }
        }
        for (i = 0; i < nf; i++) {
            u32 c0, c1, dt, b;
            const u8 *out;
            int e = 0;
            if (stop_req()) { err = E_STOPPED; break; }
            if (disp) {
                const u8 *nxt = variant == 2 ? rd.buf[rd.cur] : st.buf[st.cur];
                wait_reuse(nxt, 1);
            }
            c0 = now_counts();
            if (variant == 2) out = rnvref_decode(&rd, (const u8 *)fptr[i], flen[i]);
            else {
                e = variant == 4 ? rnvf_decode(&st, (const u8 *)fptr[i], flen[i]) : variant == 1 ? rnvp_decode(&st, (const u8 *)fptr[i], flen[i])
                                 : rnvo_decode(&st, (const u8 *)fptr[i], flen[i]);
                out = st.buf[st.last];
                if (variant == 3) oc_wb((u32)out, FB_BYTES);
            }
            c1 = now_counts();
            dt = c1 - c0;
            ST(ST_HB) = tick();
            ST(D_LAST_FRAME) = i | (lp << 16);
            ST(D_DECODES)++;
            if (e) {
                ST(D_DEC_ERR)++;
                if (ST(D_DEC_ERR) == 1) { ST(D_FIRST_ERR) = (i << 8) | (u32)e | (lp << 24); ST(D_ERR_BLK) = st.err_blk; }
            }
            if (dt < tmin) tmin = dt;
            if (dt > tmax) tmax = dt;
            if (sum_lo + dt < sum_lo) sum_hi++;
            sum_lo += dt;
            b = dt / BIN_COUNTS; if (b >= NBIN) b = NBIN - 1;
            ST(HIST + b)++;
            if (lp == 0) res[4 * i] = dt;
            if (dt < res[4 * i + 1]) res[4 * i + 1] = dt;
            if (dt > res[4 * i + 2]) res[4 * i + 2] = dt;
            if (crcm == 1 || (crcm == 2 && lp == 0)) {
                u32 c = crc32_upd(0, out, FB_BYTES);
                if (lp == 0) {
                    res[4 * i + 3] = c;
                    if (exp && c != exp[i]) { if (!ST(D_CRC_MISM)) ST(D_FIRST_MISM) = i; ST(D_CRC_MISM)++; }
                } else if (c != res[4 * i + 3]) ST(D_LOOP_MISM)++;
            }
            if (disp) {
                u32 fr = (u32)out;
                if (pace) while (tick() - last_flip < pace) dly_tsk(1);
                wb_drain(fr + FB_BYTES - 4);
                du_flip(DSA_PHYS(fr));
                replaced[fb_index((const u8 *)front)] = tick();
                front = fr;
                last_flip = tick();
                ST(D_FLIPS)++;
            }
            dly_tsk(1);
        }
        ST(D_LOOPS_DONE) = lp + (err == E_OK ? 1 : 0);
    }
    if (CTL_P(13) & 2) rd_finish();
    ST(D_MS_TOTAL) = tick() - t_start;
    ST(D_TMIN) = tmin; ST(D_TMAX) = tmax; ST(D_TSUM_LO) = sum_lo; ST(D_TSUM_HI) = sum_hi;
    ST(D_NTIMED) = ST(D_DECODES);
    if (disp) {
        u32 t0 = tick();
        while (tick() - t0 < hold && !stop_req()) { dly_tsk(10); ST(ST_HB) = tick(); }
        du_restore();
        ST(D_DU) |= 0x100;
    }
    st_end(PROG_DECBENCH, err);
    return 0;
}
