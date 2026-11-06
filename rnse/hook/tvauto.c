/* tvauto.c -- r61 SDST flash module: start sdstream when the TV screen is entered, stop it on exit, no CAN, no PC.
 *
 * Two entry points, both called through the table in hdr.S:
 *   tva_poll()  every 100 ms from the ACON task (OSM cave tvhook_r61.s, after PWM main 8060125A). Never blocks: reads RAM, may
 *               call sta_tsk once, may set the stop word. State lives in RAM at TVA_BASE (no .data/.bss in this module).
 *   tva_task()  task 159 body (entry swapped in for one sta_tsk, as the r60 cave and rnse_bench do; rest state stays the Doom entry).
 *               Waits for a live stream on the card, copies the UNCHANGED RAM build of sdstream (out/sdstream.bin, embedded
 *               in the module by hdr.S) to its link address, writes the same CTL block "benchd.py sdstream run" writes, and
 *               calls its main(). So the code that runs in the car is byte-for-byte the binary tested on the bench, at the same
 *               address, with the same timing. 8062D848 + 805F3A94(3) and the /4 clock poke are already inside it (--iis, timed_read).
 *
 * Addresses: r59 OSM (= r57 for all of these). See R61.md for the evidence of each.
 *   src  u8 8E9DFF52 (ctx[2], 7 = TV)     screen group u16 8E9E083C (9 = TV, NOT yet read live)     pwm u8 8E9F3564 (3 = MMI on)
 *
 * RAM host pokes: TVA(W_OFF) = 'OFF!' disables auto start (bench: rnse_bench.start would otherwise find task 159 busy on TV).
 */
typedef unsigned int u32;
typedef int s32;
typedef unsigned short u16;
typedef unsigned char u8;
#define R32(a) (*(volatile u32 *)(a))
#define R16(a) (*(volatile u16 *)(a))
#define R8(a)  (*(volatile u8 *)(a))

#include "gen_cfg.h"   /* IMG_BASE IMG_MAIN IMG_BSS_S IMG_BSS_E, TVA_CTL_WORDS, TVA_CTL_FLAGS, TVA_USE_SCREEN, TVA_BUILD_ID */

/* ---- firmware ---- */
#define TICK_MS     0x8E8DA8B0u
#define FW_DLY_TSK  0x804D04D8u
#define FW_EXT_TSK  0x804D027Cu
#define FW_STA_TSK  0x804D026Cu
#define FW_SRCSEL   0x805F3A94u
#define FW_SD_RDCORE 0x8067D108u
#define TCB159      0x8E8D7B6Cu
#define DOOM_ENTRY  0x80B06000u
#define SRC_CTX     0x8E9DFF52u
#define SRC_OVERLAY 0x8E9DFF54u
#define SRC_CACHE   0x8E9F08C4u
#define SCR_GROUP   0x8E9E083Cu
#define PWM_STATE   0x8E9F3564u
#define DMDL_HANDLE 0x8FCE6294u
#define SD_VARIANT0 0x8EAD1254u
#define SD_BUSY(s)  (*(volatile u32 *)((s) == 1 ? 0x8EAAC96Cu : 0x8EAAC970u))
#define SH_MR0      0x8E8E7D14u
#define DU_MR_YUY2  0x04104033u
#define SRC_TV 7u
#define SCR_TV 9u
#define PWM_ON 3u

/* ---- sdstream's own blocks (rnse.h) ---- */
#define CTL_BASE   0x8F720000u
#define STAT_BASE  0x8F720100u
#define STAT_MAGIC 0x41565344u
#define CTL_STOP   R32(CTL_BASE + 4)
#define ST(i)      R32(STAT_BASE + 4 * (i))
#define PROG_SDSTREAM 6u
#define PROBE_BUF  0x8F780000u   /* SFRING: 32-B aligned, unused until sdstream runs */
#define WIN_LBA    0x10000u
#define H_IDLE     4u
#define H_NOTREADY 64u

/* ---- our state ---- */
#define TVA_BASE   0x8F72F000u   /* above sdstream's log ring (8F721000..8F729000), below TONE_BUF; see R61.md RAM map */
#define TVA(i)     R32(TVA_BASE + 4 * (i))
enum { W_MAGIC, W_OFF, W_STATE, W_STARTS, W_T_WANT, W_T_NOWANT, W_T_NEXT, W_ERR, W_FAILS, W_T_BOOT, W_REASON, W_HEALS, W_STAGE,
       W_PROBES, W_PROBE_LAST, W_ROUTE, W_T_START, W_SAMPLE, W_TRN, W_CRASH, W_STA_RET, W_MRBAD, W_T_STOP, W_BUILD, W_T_LAST };
#define TVA_MAGIC 0x54564131u
#define OFF_MAGIC 0x4F464621u
#define TR_BASE   (TVA_BASE + 0x100)
#define TR_N      128u
enum { TS_IDLE, TS_RUN, TS_STOP };
enum { TR_INIT = 1, TR_SAMPLE, TR_START, TR_STARTFAIL, TR_STOP, TR_EXIT, TR_HEAL, TR_LOADER, TR_ROUTE, TR_CRASHOFF };
enum { WHY_SRC = 1, WHY_SCREEN, WHY_POWER, WHY_DU, WHY_OFF };

#define GRACE_MS    10000u   /* nothing after boot: time to pick another source if a build misbehaves; the card mounts meanwhile */
#define SETTLE_MS   1000u    /* TV entry finished re-registering its plane (DU_DESIGN 4B) before we take array 0 */
#define LEAVE_SRC_MS 1500u   /* sdstream sees a source change itself within 1 s (err 10, skips the route restore): let it, then force */
#define LEAVE_SCR_MS 300u
#define CRASH_MAX   3u

static inline u32 tick(void) { return R32(TICK_MS); }
static inline u32 be32(const u8 *p) { return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3]; }
#define dly_tsk(ms) ((int (*)(int))FW_DLY_TSK)(ms)

static void trace(u32 code, u32 a, u32 b)
{
    u32 k = TVA(W_TRN)++ % TR_N, p = TR_BASE + k * 16;
    R32(p) = tick(); R32(p + 4) = code | (TVA(W_STATE) << 16); R32(p + 8) = a; R32(p + 12) = b;
}

static inline u32 dormant159(void) { return R16(TCB159 + 4) == 0x0010; }

static inline u32 want_now(void)
{
#if TVA_USE_SCREEN
    if (R16(SCR_GROUP) != SCR_TV) return 0;
#endif
    return R8(SRC_CTX) == SRC_TV && R8(PWM_STATE) == PWM_ON;
}

extern void tva_task(void);

/* ------------------------------------------------------------------ ACON task, 100 ms. Must not block. */
void tva_poll(void)
{
    u32 now = tick(), want, sample, st;
    if (TVA(W_MAGIC) != TVA_MAGIC || now < TVA(W_T_LAST)) {   /* first call after power-up, or after a reset that kept RAM */
        u32 i, off = 0, crash = 0;
        if (TVA(W_MAGIC) == TVA_MAGIC) {
            off = TVA(W_OFF) == OFF_MAGIC ? OFF_MAGIC : 0;
            crash = TVA(W_CRASH);
            if (TVA(W_STATE) != TS_IDLE) crash++;             /* the unit reset while a stream ran */
        }
        for (i = 0; i < 0x100 / 4 + TR_N * 4; i++) TVA(i) = 0;
        TVA(W_OFF) = off; TVA(W_CRASH) = crash; TVA(W_T_BOOT) = now; TVA(W_BUILD) = TVA_BUILD_ID;
        TVA(W_MAGIC) = TVA_MAGIC;
        trace(TR_INIT, crash, off);
        if (crash >= CRASH_MAX) { TVA(W_OFF) = OFF_MAGIC; trace(TR_CRASHOFF, crash, 0); }
    }
    TVA(W_T_LAST) = now;
    sample = ((u32)R8(SRC_CTX) << 24) | (((u32)R16(SCR_GROUP) << 8) & 0xFFFF00u) | R8(PWM_STATE);
    if (sample != TVA(W_SAMPLE)) { TVA(W_SAMPLE) = sample; trace(TR_SAMPLE, sample, R8(SRC_OVERLAY)); }
    if (now - TVA(W_T_BOOT) < GRACE_MS) return;

#if TVA_STANDBY   /* sdstream --standby: it watches the source itself, we only keep it alive */
    want = TVA(W_OFF) != OFF_MAGIC && R8(PWM_STATE) == PWM_ON;
#else
    want = TVA(W_OFF) != OFF_MAGIC && want_now();
#endif
    st = TVA(W_STATE);
    if (st == TS_IDLE) {
        if (!want) { TVA(W_T_WANT) = 0; return; }
        if (!TVA(W_T_WANT)) TVA(W_T_WANT) = now;
        if (now - TVA(W_T_WANT) < SETTLE_MS || (s32)(now - TVA(W_T_NEXT)) < 0) return;
        if (!dormant159()) return;                            /* Doom or a bench program */
        {
            int r;
            CTL_STOP = 0;
            TVA(W_STAGE) = 0;
            R32(TCB159 + 0x20) = (u32)tva_task;
            r = ((int (*)(int, int))FW_STA_TSK)(159, 0);
            R32(TCB159 + 0x20) = DOOM_ENTRY;                  /* sta_tsk copied it into the launch frame (R60.md, 804CF3D2) */
            TVA(W_STA_RET) = (u32)r;
            if (r) { TVA(W_T_NEXT) = now + 2000; trace(TR_STARTFAIL, (u32)r, 0); return; }
            TVA(W_STATE) = TS_RUN; TVA(W_T_START) = now; TVA(W_T_NOWANT) = 0; TVA(W_MRBAD) = 0; TVA(W_STARTS)++;
            trace(TR_START, TVA(W_STARTS), 0);
        }
        return;
    }
    if (dormant159()) {                                       /* task ended: asked to, or by itself */
        u32 ran = now - TVA(W_T_START);
        if (st == TS_RUN) {
            u32 f = ran < 5000 ? TVA(W_FAILS) + 1 : 0, d = f > 4 ? 30000u : 2000u << f;
            TVA(W_FAILS) = f;
            TVA(W_T_NEXT) = now + (f ? d : 1000u);
        } else TVA(W_T_NEXT) = now + 300u;
        trace(TR_EXIT, TVA(W_ERR), ran);
        TVA(W_STATE) = TS_IDLE; TVA(W_T_WANT) = 0; TVA(W_CRASH) = 0;
        return;
    }
    if (st == TS_STOP) {
        if (now - TVA(W_T_STOP) > 5000u) { CTL_STOP = 1; TVA(W_T_STOP) = now; trace(TR_STOP, 0xEE, 0); }
        return;
    }
    {   /* TS_RUN */
        u32 why = 0;
        if (want) TVA(W_T_NOWANT) = 0;
        else {
            if (!TVA(W_T_NOWANT)) TVA(W_T_NOWANT) = now;
            if (TVA(W_OFF) == OFF_MAGIC) why = WHY_OFF;
            else if (R8(PWM_STATE) != PWM_ON) why = WHY_POWER;
            else if (R8(SRC_CTX) != SRC_TV) { if (now - TVA(W_T_NOWANT) >= LEAVE_SRC_MS) why = WHY_SRC; }
            else if (now - TVA(W_T_NOWANT) >= LEAVE_SCR_MS) why = WHY_SCREEN;
        }
        /* TV (re)entry or a TV re-init rewrites array 0 under us (MR 04104033 -> 04004031, DU_DESIGN 4B): take it again */
        if (!why && ST(0) == STAT_MAGIC && ST(1) == PROG_SDSTREAM && ST(2) == 2 && (TVA_CTL_FLAGS & 4u)) {
            if (R32(SH_MR0) != DU_MR_YUY2) { if (++TVA(W_MRBAD) >= 2) { why = WHY_DU; TVA(W_HEALS)++; trace(TR_HEAL, R32(SH_MR0), 0); } }
            else TVA(W_MRBAD) = 0;
        }
        if (why) {
            CTL_STOP = why == WHY_SRC ? 2u : 1u;              /* 2 = the user left: any nonzero stops; a sdstream that knows 2 can skip its route restore */
            TVA(W_REASON) = why; TVA(W_STATE) = TS_STOP; TVA(W_T_STOP) = now;
            trace(TR_STOP, why, sample);
        }
    }
}

/* ------------------------------------------------------------------ task 159 */
static inline u32 stop_req(void) { return CTL_STOP != 0; }
#if !TVA_STANDBY
static inline void oc_inval(u32 a, u32 n) { u32 e = a + n; for (a &= ~31u; a < e; a += 32) __asm__ volatile("ocbi @%0" : : "r"(a) : "memory"); }

/* 0 = stream alive, 1 = card answers but no feed yet (or not our card), 2 = read failed. The read is the call sdstream's sd_read_to makes, 150 ms bound. */
static u32 probe(void)
{
    volatile int cnt = 1;
    const u8 *sf = (const u8 *)PROBE_BUF;
    u32 hf;
    int r;
    R32(SD_VARIANT0) = 0x00000100u;
    SD_BUSY(0) = 1;
    r = ((int (*)(u32, void *, volatile int *, u32, int))FW_SD_RDCORE)(0, (void *)PROBE_BUF, &cnt, WIN_LBA, 150);
    SD_BUSY(0) = 0;
    TVA(W_PROBES)++;
    if ((short)r != 0 || cnt != 1) return 2;
    oc_inval(PROBE_BUF, 512);
    hf = be32(sf + 12);
    TVA(W_PROBE_LAST) = hf;
    if (be32(sf) != 0x52534156u) return 1;                    /* 'RSAV': a normal SD card never reaches the takeover */
    if (hf & H_IDLE) return 1;
    if ((hf & H_NOTREADY) && !be32(sf + 64)) return 1;        /* Teensy up, no complete superframe from the Pi yet */
    return 0;
}

#endif

static inline void icbi_range(u32 a, u32 e)
{
    register u32 p __asm__("r1") = a & ~31u;
    for (; p < e; p += 32) __asm__ volatile(".word 0x01E3" : : "r"(p) : "memory");   /* icbi @r1, as tramp.S */
}

/* Send the controller the input of the source that is selected now. Stock chose it when the user changed source, but sdstream's stop path can
 * send TV after that (its restore runs unless it saw the change itself). 805F3A94 is any-task, draws nothing (AUDIO_ROUTE_RE 2.2). */
static void route_fixup(void)
{
    static const signed char idx_of[16] = { -1, 2, 2, 2, 0, 1, 3, 6, -1, -1, -1, -1, -1, 10, 7, 7 };
    static const u8 ctrl_of[11] = { 1, 2, 3, 4, 5, 6, 8, 9, 0x80, 3, 10 };   /* 8047C8E4 */
    u32 s = R8(SRC_CTX);
    int idx = s < 16 ? idx_of[s] : -1;
    if (idx < 0 || R8(SRC_OVERLAY) || R8(PWM_STATE) != PWM_ON) return;
    if (R8(SRC_CACHE) == ctrl_of[idx]) return;
    TVA(W_ROUTE) = (u32)idx + 1;
    trace(TR_ROUTE, s, R8(SRC_CACHE));
    ((int (*)(int))FW_SRCSEL)(idx);
}

void tva_task(void)
{
    static const u32 ctl[] = TVA_CTL_WORDS;
    extern const u32 tva_image[], tva_image_end[];
    u32 i;
    TVA(W_STAGE) = 1;
#if !TVA_STANDBY
    for (;;) {                                                /* no takeover until the Pi really feeds: until then the unit stays stock */
        u32 r;
        if (stop_req()) goto out;
        r = probe();
        if (!r) break;
        dly_tsk(r == 2 ? 1000 : 250);
    }
#endif
    TVA(W_STAGE) = 2;
    {   /* MP3 player closing its handle after a source change */
        u32 t0 = tick();
        while ((R32(DMDL_HANDLE) >> 24) == 2u && tick() - t0 < 1500) dly_tsk(10);
    }
    if (stop_req()) goto out;
#ifndef TVA_NOCOPY
    {
        volatile u32 *d = (volatile u32 *)IMG_BASE;
        const u32 *s = tva_image;
        while (s < tva_image_end) *d++ = *s++;
    }
#endif
    for (i = IMG_BSS_S; i < IMG_BSS_E; i += 4) R32(i) = 0;
    (void)R32(((IMG_BSS_E - 4) & 0x1FFFFFFCu) | 0xA0000000u);  /* write-through drain, as wb_drain */
    icbi_range(IMG_BASE, IMG_BSS_E);
    for (i = 0; i < sizeof ctl / sizeof ctl[0]; i++) if (i != 1) R32(CTL_BASE + 4 * i) = ctl[i];   /* word 1 = stop word: keep */
    TVA(W_STAGE) = 3;
    trace(TR_LOADER, tick(), 0);
    ((int (*)(void))IMG_MAIN)();
    TVA(W_ERR) = ST(3);
    TVA(W_STAGE) = 4;
    route_fixup();
out:
    TVA(W_STAGE) = 0xFE;
    ((void (*)(void))FW_EXT_TSK)();
}

#ifdef TVA_RAM
/* RAM equivalent of the OSM hook, for the bench (no flash): started as task 159 by the bench launcher; moves itself to free task id 0x2D (sdstream takes 0x2E, 0x2F; 0x30 is past the task table: not a record)
 * with its own stack (empty TCB, as sdstream's reader on 0x2F) and calls tva_poll every 100 ms. W_OFF = 'QUIT' ends it. */
#define HR_TASK 0x2Du
#define HR_TCB  (0x8E8D53ACu + HR_TASK * 0x40u)
void tva_ram_loop(void)
{
    while (TVA(W_OFF) != 0x51554954u) { tva_poll(); dly_tsk(100); }
    TVA(W_OFF) = 0;
    ((void (*)(void))FW_EXT_TSK)();
}
__asm__(".text\n.align 2\n_tva_ram_stk:\n  mov.l 1f,r15\n  mov.l 2f,r0\n  jmp @r0\n  nop\n  .align 2\n1: .long 0x8F72E000\n2: .long _tva_ram_loop\n");
extern void tva_ram_stk(void);
void tva_ram_start(void)
{
    u32 saved = R32(HR_TCB + 0x20);
    if ((R32(HR_TCB + 4) >> 16) == 0x10) {
        R32(HR_TCB + 0x20) = (u32)tva_ram_stk;
        ((int (*)(int, int))FW_STA_TSK)((int)HR_TASK, 0);
        R32(HR_TCB + 0x20) = saved;
    }
    ((void (*)(void))FW_EXT_TSK)();
}
#endif
