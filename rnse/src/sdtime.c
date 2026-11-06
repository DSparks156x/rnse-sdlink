/* sdtime: where does the 92 us/sector SD read time go? RAM only, nothing in flash changes, MMIO is read-only.
 *
 * mode bits (p0):
 *   1  timing  time CPU reads of RAM, SD host, DMAC and TMU registers (no SD traffic)
 *   2  mem     RAM bandwidth seen by the CPU, in SD_BUF
 *   4  dump    read registers the firmware itself uses
 *   8  port    also time reads of the SD data port +0x30. OPT-IN: an idle read may set the host's illegal access flag
 *  16  extra   also dump registers the firmware never reads (+E2/E4/E6, FF800018..20). OPT-IN
 *  64  periph  also time other peripherals, for comparison
 * 128  wide    also time 32-bit reads of SD registers. OPT-IN: this host's registers are 16-bit; may fault
 * 256  dma     run the SD DMA channel alone, auto-request, from harmless registers to RAM: DMA speed with no SD controller pacing
 *  32  read    timed reads: flash driver first, then a RAM copy of the driver for each DCR value in p8..p13
 *
 * The RAM copy: 8067B400..80684C00 is copied whole to DRV_COPY, so its relative branches and literal loads still work.
 * Only these literals change: the read core's address (8067D8DC, 8067E458) and the DMA control value (806834D8, 80683818,
 * 80684484). All are checked against the stock value first. Absolute calls still go to flash and the driver's state
 * and lock are shared, so the copy cooperates with the file system exactly as the flash driver does.
 *
 * Reads call 8067D108(slot, buf, &count, lba, timeout_ms), the function sd_raw_read (80653598) calls with 50000.
 *
 * CTL: p0 mode, p1 slot, p2 lba, p3 sectors per pass (3..4096), p4 sectors per call (3..4096; 1 and 2 use another path),
 *      p5 timeout ms, p6 value for word 8EAD1254 before each pass (FFFFFFFF = leave it; restored at the end),
 *      p7 number of DCR values (0..6) | 0x100 = one attempt: the copy gives up on the first error instead of retrying
 *      (8068348A mov #3,r2 -> mov #1,r2), so the error bits and the divider byte survive for the report,
 *      0x200 = 32-bit transfers: the copy programs the DMA count as n << 7, not n << 8 (80682FC2 and 80682FC6),
 *      0x400 = also set SD +0xE4 to 0101 during the copy's passes (32-bit port on later SDHI, unknown here; restored),
 *      0x800 = delayed DMA start: the copy does not start the DMA itself (806832D4 mov #1,r5 -> #0); a hook in the wait call
 *      (literal 806834E8) waits for the first data, then starts it. For auto-request DCR values, which have no pacing,
 *      0x1000 = CPU copy: with 0x800, the hook reads the port itself instead of starting the DMA (measurement; uses all the CPU),
 *      0x2000 = trace: with 0x800, the hook times every DMA count change during the read (last call of the run is kept),
 *      p8..p13 DCR values.
 * STAT: 8..15 header, 16.. timing (min, max per row), 40..47 mem, 64.. passes (R_N words each), 280.. dump (addr, value). */
#include "rnse.h"

#define PROG_SDTIME 7
#define SD_BASE     0xFF90A000u
#define DMA_CH      0xFF801200u
#define DMA_COM     0xFF801400u
#define MARK        0xA5C3A5C3u
#define GUARD       0x5AA5C33Cu
#define GUARD_N     64u

#define DRV_SRC     0x8067B400u
#define DRV_END     0x80684C00u
#define DRV_COPY    0x8F7D0000u                       /* 64 KiB free while sdstream is not running (same task, exclusive) */
#define COPY_OF(a)  ((u32)(a) - DRV_SRC + DRV_COPY)
#define FW_CORE     0x80682F92u
#define FW_RD108    0x8067D108u
#define DCR_STOCK   0x00402112u
#define CLK_WORD    0x8EAD1254u

enum { SB_ERRDETAIL = 8, SB_NTIME, SB_NPASS, SB_NDUMP, SB_CLK_WORD0, SB_INFO2_0, SB_INFO2_1, SB_CLK_WORD1 };
enum { ST_TIME = 16, ST_MEM = 56, ST_PASS = 64, ST_DUMP = 300 };
enum { R_DCR, R_PATH, R_CHUNK, R_SECT, R_CALLS, R_ERRS, R_COUNTS, R_SUM, R_W0, R_W1, R_UNTOUCHED, R_GUARD, R_CLK, R_DRVINFO, R_ERRSTS, R_LIVE, R_N };
#define DRV_INFO    0x8EAD125Cu                       /* the driver's INFO1/INFO2 image, slot stride 0xA4; tested with 0x807F at 8068338C */
enum { E_OK, E_NOCTL, E_PARAM, E_FW, E_STOPPED, E_READ };
enum { TM_R32, TM_R16, TM_W32 };

typedef int (*rd108_t)(u32 slot, void *buf, u32 *count, u32 lba, s32 timeout_ms);

static inline void ocbp(u32 a) { __asm__ volatile("ocbp @%0" : : "r"(a) : "memory"); }

static void sync_code(u32 a, u32 n)                   /* write the data cache back, drop the instruction cache */
{
    u32 e = a + n;
    for (a &= ~31u; a < e; a += 32) {
        register u32 r1 __asm__("r1") = a;
        __asm__ volatile("ocbwb @%0\n\t.word 0x01E3" : : "r"(r1) : "memory");   /* 01E3 = icbi @r1 */
    }
}

/* ---------------------------------------------------------------- timing */
struct trow { u32 addr; u32 kind; };
static const struct trow TIMED[] = {
    { 0x8F7CF000u, TM_R32 },        /* 0 cached RAM, hit: the loop's own cost */
    { 0xAF7CF000u, TM_R32 },        /* 1 uncached RAM */
    { 0xAF7CF000u, TM_R16 },        /* 2 uncached RAM, 16-bit */
    { 0xAF7CF000u, TM_W32 },        /* 3 uncached RAM, write */
    { SD_BASE + 0x24, TM_R16 },     /* 4 SD CLK_CTRL */
    { SD_BASE + 0x1E, TM_R16 },     /* 5 SD INFO2 */
    { SD_BASE + 0xD8, TM_R16 },     /* 6 SD CC_EXT_MODE */
    { DMA_CH + 0x34, TM_R32 },      /* 7 DMA DSTSR */
    { DMA_COM + 0x10, TM_R32 },     /* 8 DMA DINTSR */
    { TMU_TCNT0, TM_R32 },          /* 9 TMU */
    { SD_BASE + 0x30, TM_R16 },     /* 10 SD data port: only with mode bit 8 */
    { SSI_CR, TM_R32 },             /* 11 other peripherals, for comparison: only with mode bit 64 */
    { SSI_CLKW, TM_R32 },           /* 12 */
    { 0xFF93E00Cu, TM_R32 },        /* 13 GPIO */
    { 0xFFF80008u, TM_R32 },        /* 14 DU DSSR */
    { 0xFF903024u, TM_R32 },        /* 15 DMA request select */
    { 0xFF900864u, TM_R32 },        /* 16 */
    { 0xFF80001Cu, TM_R32 },        /* 17 */
    { SD_BASE + 0x24, TM_R32 },     /* 18 SD CLK_CTRL as a 32-bit read: only with mode bit 128 */
    { SD_BASE + 0x30, TM_R32 },     /* 19 SD data port as a 32-bit read: only with mode bit 128 */
};
#define NTIMED 10u
#define TN 1024u

static u32 time_once(u32 a, u32 kind)
{
    u32 c0, c1, i;
    volatile u32 *p32 = (volatile u32 *)a;
    volatile u16 *p16 = (volatile u16 *)a;
    c0 = now_counts();
    if (kind == TM_R32)      for (i = 0; i < TN; i++) (void)*p32;
    else if (kind == TM_R16) for (i = 0; i < TN; i++) (void)*p16;
    else                     for (i = 0; i < TN; i++) *p32 = i;
    c1 = now_counts();
    return c1 - c0;
}

static void do_timing(u32 mode)
{
    u32 r, k;
    for (r = 0; r < sizeof TIMED / sizeof TIMED[0]; r++) {
        u32 lo = 0xFFFFFFFFu, hi = 0;
        if (r == 10 && !(mode & 8)) continue;
        if (r >= 11 && r <= 17 && !(mode & 64)) continue;
        if (r >= 18 && !(mode & 128)) continue;
        for (k = 0; k < 16; k++) {
            u32 d = time_once(TIMED[r].addr, TIMED[r].kind);
            if (d < lo) lo = d;
            if (d > hi) hi = d;
        }
        ST(ST_TIME + 2 * r) = lo;
        ST(ST_TIME + 2 * r + 1) = hi;
        ST(SB_NTIME) = r + 1;
        dly_tsk(1);
    }
}

/* ---------------------------------------------------------------- RAM bandwidth (bytes over counts; host converts) */
#define MB (64u * 1024u)
#define UB (8u * 1024u)
static void do_mem(void)
{
    volatile u32 *c = (volatile u32 *)SD_BUF, *u = (volatile u32 *)P2(SD_BUF);
    volatile u32 *c2 = (volatile u32 *)(SD_BUF + MB);
    u32 i, t, s = 0;

    for (i = 0; i < MB / 4; i++) c[i] = i;                          /* make it dirty */
    t = now_counts(); for (i = 0; i < MB; i += 32) ocbp(SD_BUF + i);
    ST(ST_MEM + 0) = now_counts() - t;                              /* write-back of 64 KiB of dirty lines */
    dly_tsk(1);
    t = now_counts(); for (i = 0; i < MB / 4; i++) s += c[i];
    ST(ST_MEM + 1) = now_counts() - t;                              /* cached read, every line a miss */
    dly_tsk(1);
    for (i = 0; i < MB; i += 32) ocbp(SD_BUF + i);
    t = now_counts(); for (i = 0; i < MB / 4; i++) c[i] = s + i;
    ST(ST_MEM + 2) = now_counts() - t;                              /* cached write */
    for (i = 0; i < MB; i += 32) ocbp(SD_BUF + i);
    dly_tsk(1);
    t = now_counts(); for (i = 0; i < MB / 4; i++) c2[i] = c[i];
    ST(ST_MEM + 3) = now_counts() - t;                              /* cached copy, word loop */
    for (i = 0; i < MB; i += 32) { ocbp(SD_BUF + i); ocbp(SD_BUF + MB + i); }
    dly_tsk(1);
    t = now_counts(); for (i = 0; i < UB / 4; i++) s += u[i];
    ST(ST_MEM + 4) = now_counts() - t;                              /* uncached read, 8 KiB */
    dly_tsk(1);
    t = now_counts(); for (i = 0; i < UB / 4; i++) u[i] = s;
    ST(ST_MEM + 5) = now_counts() - t;                              /* uncached write, 8 KiB */
}

/* ---------------------------------------------------------------- register dump */
static u32 nd;
static void d16(u32 a) { ST(ST_DUMP + 2 * nd) = a; ST(ST_DUMP + 2 * nd + 1) = R16(a); ST(SB_NDUMP) = ++nd; }
static void d32(u32 a) { ST(ST_DUMP + 2 * nd) = a; ST(ST_DUMP + 2 * nd + 1) = R32(a); ST(SB_NDUMP) = ++nd; }

static void do_dump(u32 extra)
{
    static const u16 sd[] = { 0x1C, 0x1E, 0x20, 0x22, 0x24, 0x26, 0x28, 0xD8, 0xE0 };
    static const u8 ch[] = { 0x00, 0x04, 0x08, 0x18, 0x1C, 0x20, 0x24, 0x28, 0x34 };
    static const u8 com[] = { 0x00, 0x04, 0x0C, 0x10, 0x18, 0x20, 0x80, 0x84, 0x90 };
    u32 i;
    if (!(extra & 4)) { if (extra & 128) { d32(SD_BASE + 0x24); d32(SD_BASE + 0x1C); } return; }
    for (i = 0; i < sizeof sd / sizeof sd[0]; i++) d16(SD_BASE + sd[i]);
    for (i = 0; i < sizeof ch; i++) d32(DMA_CH + ch[i]);
    for (i = 0; i < sizeof com; i++) d32(DMA_COM + com[i]);
    d32(0xFF903024u);                                  /* DMA request select */
    d32(0xFF900804u);
    d32(0xFF900864u);
    if (extra & 128) { d32(SD_BASE + 0x24); d32(SD_BASE + 0x1C); }
    if (extra & 16) {
        d16(SD_BASE + 0xE2); d16(SD_BASE + 0xE4); d16(SD_BASE + 0xE6);
        d32(0xFF800018u); d32(0xFF80001Cu); d32(0xFF800020u);
    }
}

/* ---------------------------------------------------------------- the DMA channel alone */
#define ST_RATE 430u
#define RATE_N  4096u
#define CH(n)   (0xFF801000u + 0x40u * (n))
/* ch, source, DCR, flags (1 = do not poll the DMA while it runs), register to change during this row (0 = none), value */
struct rrow { u32 ch, src, dcr, flags, reg, val; };
static const struct rrow RATE[] = {       /* channels 4, 5, 6 are audio and have interrupts enabled: left alone */
    { 8, SD_BASE + 0x24, 0x00402512u, 1, 0, 0 },            /* 0 baseline */
    { 0, SD_BASE + 0x24, 0x00402512u, 1, 0, 0 },            /* 1 */
    { 1, SD_BASE + 0x24, 0x00402512u, 1, 0, 0 },            /* 2 */
    { 2, SD_BASE + 0x24, 0x00402512u, 1, 0, 0 },            /* 3 */
    { 3, SD_BASE + 0x24, 0x00402512u, 1, 0, 0 },            /* 4 */
    { 7, SD_BASE + 0x24, 0x00402512u, 1, 0, 0 },            /* 5 */
    { 10, SD_BASE + 0x24, 0x00402512u, 1, 0, 0 },           /* 6 */
    { 11, SD_BASE + 0x24, 0x00402512u, 1, 0, 0 },           /* 7 */
    { 13, SD_BASE + 0x24, 0x00402512u, 1, 0, 0 },           /* 8 */
    { 14, SD_BASE + 0x24, 0x00402512u, 1, 0, 0 },           /* 9 */
    { 0, SD_BASE + 0x24, 0x00602512u, 1, 0, 0 },            /* 10 burst bit: does it stick, is it faster */
    { 1, SD_BASE + 0x24, 0x00602512u, 1, 0, 0 },            /* 11 */
    { 2, SD_BASE + 0x24, 0x00602512u, 1, 0, 0 },            /* 12 */
    { 3, SD_BASE + 0x24, 0x00602512u, 1, 0, 0 },            /* 13 */
    { 8, SD_BASE + 0x24, 0x00402512u, 3, 0, 0 },            /* 14 channels 8 and 9 together: shared engine, or parallel? */
    { 12, SD_BASE + 0x24, 0x00402512u, 3, 0, 0 },           /* 15 channels 12 and 13 together */
};
#define NRATE (sizeof RATE / sizeof RATE[0])
static void do_rate(void)
{
    u32 i, id = R16(0x8EAD1118u + 0x2AEu);
    typedef int (*lk_t)(int, int);
    typedef int (*ul_t)(int);
    int lk = ((lk_t)0x80653B44u)((int)id, 5000);
    ST(ST_RATE - 1) = (u32)lk;
    if (lk) return;
    for (i = 0; i < NRATE; i++) {
        const struct rrow *q = &RATE[i];
        u32 c = CH(q->ch), t0, t1, st, left, old = 0, done = 0;
        volatile u32 spin;
        if (R32(c + 0x34) & 1u) { ST(ST_RATE + 4 * i + 1) = 0xEE00u; continue; }   /* channel in use: leave it alone */
        if (q->reg) { old = R32(q->reg); R32(q->reg) = q->val; }
        R32(c + 0x00) = q->src;
        R32(c + 0x04) = SD_BUF;
        R32(c + 0x08) = RATE_N;
        R32(c + 0x28) = q->dcr;
        R32(P2(SD_BUF)) = MARK;
        if (q->flags & 2) {
            if (R32(c + 0x40 + 0x34) & 1u) continue;
            R32(c + 0x40 + 0x00) = q->src; R32(c + 0x40 + 0x04) = SD_BUF + 0x10000u; R32(c + 0x40 + 0x08) = RATE_N; R32(c + 0x40 + 0x28) = q->dcr;
        }
        t0 = now_counts();
        R32(c + 0x2C) = 1;
        if (q->flags & 2) R32(c + 0x40 + 0x2C) = 1;
        if (q->flags & 1) {
            for (spin = 0; spin < 40000u; spin++) { }       /* cached loop, about 300 us, no peripheral access */
        } else {
            while ((s32)(now_counts() - t0) < 12488 * 20) if (!(R32(c + 0x34) & 1u)) { done = 1; break; }
        }
        t1 = now_counts();
        left = R32(c + 0x20);
        st = (q->flags & 2) ? RATE_N - R32(c + 0x40 + 0x20) : R32(c + 0x34);
        if (q->flags & 1) {                                 /* now let it finish */
            u32 t2 = now_counts();
            while ((s32)(now_counts() - t2) < 12488 * 20) if (!(R32(c + 0x34) & 1u)) { done = 1; break; }
        }
        if (!done) R32(c + 0x30) = 1;
        if (q->flags & 2) {
            u32 t2 = now_counts();
            while ((s32)(now_counts() - t2) < 12488 * 20) if (!(R32(c + 0x40 + 0x34) & 1u)) break;
            if (R32(c + 0x40 + 0x34) & 1u) R32(c + 0x40 + 0x30) = 1;
            R32(DMA_COM + 0x14) = 2u << q->ch;
        }
        R32(DMA_COM + 0x14) = 1u << q->ch;
        if (q->reg) R32(q->reg) = old;
        ST(ST_RATE + 4 * i) = t1 - t0;
        ST(ST_RATE + 4 * i + 1) = done | (st << 8);
        ST(ST_RATE + 4 * i + 2) = (q->flags & 1) ? RATE_N - left : RATE_N;
        ST(ST_RATE + 4 * i + 3) = R32(c + 0x28);          /* DCR readback: an unimplemented bit reads 0 */
        dly_tsk(1);
    }
    ((ul_t)0x80653B10u)((int)id);
}

/* ---------------------------------------------------------------- the driver copy */
static int copy_driver(void)
{
    volatile u32 *s = (volatile u32 *)DRV_SRC, *d = (volatile u32 *)DRV_COPY;
    u32 i, n = (DRV_END - DRV_SRC) / 4;
    /* stock bytes at everything we rely on, or no copy at all */
    if (R32(FW_RD108) != 0x4F226263u || R32(FW_RD108 + 4) != 0x66537FFCu) return 1;
    if (R32(0x8067D8DCu) != FW_CORE || R32(0x8067E458u) != FW_CORE) return 2;
    if (R32(0x806834D8u) != DCR_STOCK || R32(0x80683818u) != DCR_STOCK || R32(0x80684484u) != DCR_STOCK) return 3;
    for (i = 0; i < n; i++) d[i] = s[i];
    R32(COPY_OF(0x8067D8DCu)) = COPY_OF(FW_CORE);
    R32(COPY_OF(0x8067E458u)) = COPY_OF(FW_CORE);
    sync_code(DRV_COPY, DRV_END - DRV_SRC);
    return 0;
}

static void set_dcr(u32 v)
{
    static const u32 at[3] = { 0x806834D8u, 0x80683818u, 0x80684484u };
    u32 i;
    for (i = 0; i < 3; i++) { R32(COPY_OF(at[i])) = v; sync_code(COPY_OF(at[i]), 4); }
}

/* ---------------------------------------------------------------- delayed start */
static volatile u32 arm, hook_seen, hook_miss, pio_on, pio_buf, pio_n;
static void pio_read(void)
{
    volatile u16 *port = (volatile u16 *)(SD_BASE + 0x30), *inf2 = (volatile u16 *)(SD_BASE + 0x1E);
    volatile u16 *d = (volatile u16 *)P2(pio_buf);
    u32 n, i;
    R16(SD_BASE + 0xD8) = (u16)(R16(SD_BASE + 0xD8) & ~2u);      /* no DMA requests from the host */
    for (n = 0; n < pio_n; n++) {
        u32 t0 = now_counts();
        while (!(*inf2 & 0x100u)) if ((s32)(now_counts() - t0) > 12488 * 20) { hook_miss++; return; }
        *inf2 = (u16)~0x100u;
        for (i = 0; i < 256; i++) *d++ = *port;
        hook_seen++;
    }
}
#define ST_TR 520u                                  /* 32 histogram words, 2 summary, then 32 gaps (position, ticks) */
static volatile u32 tr_on;
static void trace_dma(u32 total)
{
    u32 i, last = R32(DMA_CH + 0x20), tl = now_counts(), t0 = tl, big = 0, ng = 0;
    for (i = 0; i < 32; i++) ST(ST_TR + i) = 0;
    while ((s32)(now_counts() - t0) < 12488 * 60) {
        u32 v = R32(DMA_CH + 0x20);
        if (v != last) {
            u32 t = now_counts(), dt = t - tl, m = last - v, b = m ? dt / m : 31;
            ST(ST_TR + (b > 31 ? 31 : b)) += m;
            if (dt > 12u) {                             /* 12 ticks, about 1 us: a pause, not a slow word */
                big += dt;
                if (ng < 32) { ST(ST_TR + 34 + 2 * ng) = total - v; ST(ST_TR + 35 + 2 * ng) = dt; ng++; }
            }
            last = v; tl = t;
            if (!v) break;
        }
    }
    ST(ST_TR + 32) = now_counts() - t0;
    ST(ST_TR + 33) = big | (ng << 24);
}
static int start_hook(int tmo)
{
    if (arm && pio_on) { arm = 0; pio_read(); }
    if (arm) {
        u32 t0 = now_counts(), ok = 0;
        arm = 0;
        while ((s32)(now_counts() - t0) < 12488 * 5) if (R16(SD_BASE + 0x1E) & 0x100u) { ok = 1; break; }
        if (ok) hook_seen++; else hook_miss++;
        R32(DMA_CH + 0x2C) = 1;
        if (tr_on) trace_dma(pio_n << 8);
    }
    return ((int (*)(int))0x806539FAu)(tmo);
}

/* ---------------------------------------------------------------- timed reads */
static u32 npass, use_hook;
static int one_pass(rd108_t rd, u32 path, u32 dcr, u32 rep, u32 slot, u32 lba, u32 total, u32 chunk, s32 tmo, u32 clk)
{
    volatile u32 *r = &ST(ST_PASS + R_N * npass);
    volatile u32 *mk = (volatile u32 *)P2(SD_BUF);
    volatile u32 *busy = (volatile u32 *)(slot == 1 ? 0x8EAAC96Cu : 0x8EAAC970u);
    u32 k, done = 0, calls = 0, c0, c1, s = 0, unt = 0, gb = 0, j;
    int err = 0;

    r[R_ERRS] = 0;
    for (k = 0; k < total; k++) mk[k * 128] = MARK ^ k;
    for (k = 0; k < GUARD_N; k++) mk[total * 128 + k] = GUARD ^ k;
    if (clk != 0xFFFFFFFFu) R32(CLK_WORD) = clk;       /* the driver lowers it after an error: start each pass equal */
    *busy = 1;
    c0 = now_counts();
    while (done < total) {
        u32 n = total - done < chunk ? total - done : chunk, cnt = n;
        arm = use_hook && path;
        pio_buf = SD_BUF + done * 512; pio_n = n;
        int ret = (short)rd(slot, (void *)(SD_BUF + done * 512), &cnt, lba + done, tmo);
        calls++;
        if (ret != 0 || cnt != n) { err = 1; r[R_ERRS] = 1u | ((u32)ret << 16); break; }
        done += n;
        ST(ST_HB) = tick();
    }
    c1 = now_counts();
    *busy = 0;
    r[R_CLK] = (u32)R16(SD_BASE + 0x24) | (R32(CLK_WORD) << 16);
    r[R_LIVE] = R32(DMA_CH + 0x28);                    /* DCR actually in the channel: proves the patch reached the hardware */
    r[R_DRVINFO] = R32(DRV_INFO + slot * 0xA4u);
    r[R_ERRSTS] = ((u32)R16(SD_BASE + 0x2C) << 16) | R16(SD_BASE + 0x2E);
    r[R_DCR] = dcr; r[R_PATH] = path | (rep << 8); r[R_CHUNK] = chunk; r[R_SECT] = done; r[R_CALLS] = calls;
    r[R_COUNTS] = c1 - c0;
    r[R_W0] = mk[0]; r[R_W1] = mk[1];
    for (k = 0; k < done; k++) {
        if (mk[k * 128] == (MARK ^ k)) unt++;
        for (j = 0; j < 128; j++) s += mk[k * 128 + j] ^ j;
        if ((k & 127) == 127) dly_tsk(1);
    }
    for (k = 0; k < GUARD_N; k++) if (mk[total * 128 + k] != (GUARD ^ k)) gb++;
    r[R_SUM] = s; r[R_UNTOUCHED] = unt; r[R_GUARD] = gb;
    npass++;
    ST(SB_NPASS) = npass;
    dly_tsk(20);
    return err || gb;
}

int main(void)
{
    u32 mode, slot, lba, total, chunk, clk, nvar, once, i, j, err = E_OK, clk0;
    s32 tmo;

    st_begin(PROG_SDTIME);
    if (!ctl_ok()) { st_end(PROG_SDTIME, E_NOCTL); return 0; }
    mode = CTL_P(0); slot = CTL_P(1); lba = CTL_P(2); total = CTL_P(3); chunk = CTL_P(4); tmo = (s32)CTL_P(5);
    clk = CTL_P(6); nvar = CTL_P(7) & 0xFF; once = CTL_P(7) & 0x100;
    clk0 = R32(CLK_WORD);
    ST(SB_CLK_WORD0) = clk0;
    ST(SB_INFO2_0) = R16(SD_BASE + 0x1E);

    if (mode & 1) { ST(ST_STAGE) = 2; do_timing(mode); }
    if (mode & 2) { ST(ST_STAGE) = 3; do_mem(); }
    if (mode & (4 | 128)) { ST(ST_STAGE) = 4; do_dump(mode); }

    if (mode & 256) { ST(ST_STAGE) = 6; do_rate(); }
    if (mode & 32) {
        rd108_t flash = (rd108_t)FW_RD108, copy = (rd108_t)COPY_OF(FW_RD108);
        if (slot > 1 || total < 3 || total > 4096 || chunk < 3 || chunk > 4096 || nvar > 6 || tmo < 10 ||
            (s32)lba <= 0) { st_end(PROG_SDTIME, E_PARAM); return 0; }
        ST(ST_STAGE) = 5;
        if (nvar) {
            int c = copy_driver();
            if (c) { ST(SB_ERRDETAIL) = (u32)c; st_end(PROG_SDTIME, E_FW); return 0; }
            if (once) {
                if (R16(COPY_OF(0x8068348Au)) != 0xE203u) { ST(SB_ERRDETAIL) = 4; st_end(PROG_SDTIME, E_FW); return 0; }
                R16(COPY_OF(0x8068348Au)) = 0xE201u;
                sync_code(COPY_OF(0x8068348Au), 2);
            }
        }
        if (nvar && (CTL_P(7) & 0x200)) {
            if (R16(COPY_OF(0x80682FC2u)) != 0x75FFu || R16(COPY_OF(0x80682FC6u)) != 0x4D18u) { ST(SB_ERRDETAIL) = 5; st_end(PROG_SDTIME, E_FW); return 0; }
            R16(COPY_OF(0x80682FC2u)) = 0x4D18u;       /* shll8 r13, replaces add #-1,r5 (ocbp then covers one more line) */
            R16(COPY_OF(0x80682FC6u)) = 0x4D01u;       /* shlr r13: count = n << 7 */
            sync_code(COPY_OF(0x80682FC2u), 8);
        }
        for (j = 0; j < 2 && err == E_OK && !once; j++)  /* flash retries and hides the first error: skip it in this mode */
            if (one_pass(flash, 0, DCR_STOCK, j, slot, lba, total, chunk, tmo, clk)) err = E_READ;
        if (nvar && (CTL_P(7) & 0x800)) {
            if (R16(COPY_OF(0x806832D4u)) != 0xE501u || R32(COPY_OF(0x806834E8u)) != 0x806539FAu) { ST(SB_ERRDETAIL) = 6; st_end(PROG_SDTIME, E_FW); return 0; }
            R16(COPY_OF(0x806832D4u)) = 0xE500u;
            R32(COPY_OF(0x806834E8u)) = (u32)start_hook;
            sync_code(COPY_OF(0x806832D4u), 2); sync_code(COPY_OF(0x806834E8u), 4);
            use_hook = 1;
            pio_on = (CTL_P(7) & 0x1000) ? 1 : 0;
            tr_on = (CTL_P(7) & 0x2000) ? 1 : 0;
        }
        if (CTL_P(7) & 0x400) R16(SD_BASE + 0xE4) = 0x0101u;
        ST(SB_ERRDETAIL) = (u32)R16(SD_BASE + 0xE4) << 16;
        for (i = 0; i < nvar && err == E_OK; i++) {
            u32 v = CTL_P(8 + i);
            set_dcr(v);
            for (j = 0; j < 2 && err == E_OK; j++) {
                if (stop_req()) { err = E_STOPPED; break; }
                if (one_pass(copy, 1, v, j, slot, lba, total, chunk, tmo, clk)) err = E_READ;
            }
        }
        if (CTL_P(7) & 0x400) R16(SD_BASE + 0xE4) = 0;
        R32(CLK_WORD) = clk0;
    }
    if (use_hook) ST(SB_ERRDETAIL) = (hook_seen << 16) | hook_miss;
    ST(SB_INFO2_1) = R16(SD_BASE + 0x1E);
    ST(SB_CLK_WORD1) = R32(CLK_WORD);
    st_end(PROG_SDTIME, err);
    return 0;
}
