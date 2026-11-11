/* tvauto.c -- SDLink v2 SDST flash module (sector 96): start sdstream when the TV screen is entered, stop it on exit, no CAN, no PC.
 * Same logic as the r61 (v1) module; v2 = every firmware address / firmware-specific constant comes from the firmware table
 * (fwtab.h) that the firmware's own hook v2 passes in r4. Nothing here is bound to one firmware build.
 *
 * Two entry points, both called through the table in hdr.S (magic 'TVA2'):
 *   tva_poll(fwt) every 100 ms from the ACON task (hook v2, after PWM main). Validates the table on EVERY call (magic, version,
 *               nwords, ram map, CRC, every SIGNED entry's first 4 bytes against flash; ~120 words, a few us). Fail: the reason goes
 *               to TVA(W_FWT_ST) (only if the table at least claims ram_map 1, else nothing is written anywhere) and nothing starts.
 *               Pass: TVA(W_FWT) = the table address, then the r61 logic. Never blocks.
 *   tva_task()  task body (entry swapped in for one sta_tsk). Hands the table to sdstream in CTL words 62 (address) and 63
 *               (complement); sdstream copies it to its .bss (sdstream.c fwt_load) and checks it again.
 *
 * RAM host pokes: TVA(W_OFF) = 'OFF!' disables auto start. */
typedef unsigned int u32;
typedef int s32;
typedef unsigned short u16;
typedef unsigned char u8;
#define R32(a) (*(volatile u32 *)(a))
#define R16(a) (*(volatile u16 *)(a))
#define R8(a)  (*(volatile u8 *)(a))

#include "gen_cfg.h"   /* IMG_BASE IMG_MAIN IMG_BSS_S IMG_BSS_E, TVA_CTL_WORDS, TVA_CTL_FLAGS, TVA_USE_SCREEN, TVA_BUILD_ID */

#ifdef TVA_RAM
#error "the r61 RAM bench harness (TVA_RAM) is not table-aware (v1 only)"
#endif

/* ---- our state ---- */
#include "sdl_ram.h"
#define TVA_BASE   ((u32)SDL_TVA_BASE)   /* CTL page +0xF000 (sdl_ram.h): above sdstream's log ring, below TONE_BUF */
#define TVA(i)     R32(TVA_BASE + 4 * (i))
enum { W_MAGIC, W_OFF, W_STATE, W_STARTS, W_T_WANT, W_T_NOWANT, W_T_NEXT, W_ERR, W_FAILS, W_T_BOOT, W_REASON, W_HEALS, W_STAGE,
       W_PROBES, W_PROBE_LAST, W_ROUTE, W_T_START, W_SAMPLE, W_TRN, W_CRASH, W_STA_RET, W_MRBAD, W_T_STOP, W_BUILD, W_T_LAST,
       W_FWT, W_FWT_ST,     /* v2: validated table address (never anything else), table status (FWT_OK or FV_*) */
       W_HS_ST, W_HS_K, W_HS_T0, W_HS_N, W_HS_ACK,     /* version handshake (appended; words 0..26 unchanged), see hs_gate */
       W_TN_ST, W_TN_SNAP, W_TN_CUR, W_TN_N, W_TN_EXT }; /* tone (appended): see tone_poll */
#define TVA_MAGIC 0x54564132u    /* 'TVA2' state block (v1 used 'TVA1': a v1 -> v2 change re-inits) */
#define OFF_MAGIC 0x4F464621u
#define TR_BASE   (TVA_BASE + 0x100)
#define TR_N      128u
enum { TS_IDLE, TS_RUN, TS_STOP };
enum { TR_INIT = 1, TR_SAMPLE, TR_START, TR_STARTFAIL, TR_STOP, TR_EXIT, TR_HEAL, TR_LOADER, TR_ROUTE, TR_CRASHOFF, TR_TONE };
enum { WHY_SRC = 1, WHY_SCREEN, WHY_POWER, WHY_DU, WHY_OFF };

/* ---- firmware table ---- */
#include "fwtab.h"
#include "sdl_ver.h"   /* SDL_STREAM_FMT and the handshake LBA window */
#define FWT_BASE_PTR ((const u32 *)TVA(W_FWT))   /* set before any FW() use, only to a table that passed fwt_check */
#define FWT_OK     0x4F4B3246u                    /* 'OK2F' */
/* W_FWT_ST on failure: 0xFA000000 | reason << 16 | detail */
enum { FV_ADDR = 1, FV_MAGIC, FV_VERSION, FV_NWORDS, FV_RAMMAP, FV_CRC, FV_SIGADDR, FV_SIG };
#define FLASH_LO   0x80000000u                    /* P1 view of the 16 MiB NOR: table and every SIGNED entry must be in it */
#define FLASH_HI   0x81000000u

#define TICK_MS     FW(FWE_TICK_MS)
#define FW_DLY_TSK  FW(FWE_DLY_TSK)
#define FW_EXT_TSK  FW(FWE_EXT_TSK)
#define FW_STA_TSK  FW(FWE_STA_TSK)
#define FW_SRCSEL   FW(FWE_SRCSEL)
#define FW_SD_RDCORE FW(FWE_SD_RDCORE)
#define TASK_TV     FW(FWE_TASK_TV)
#define TCB_TV      (FW(FWE_TCB_BASE) + TASK_TV * 0x40u)
#define REST_ENTRY  FW(FWE_TASK_REST)
#define SRC_CTX     FW(FWE_SRC_CTX)
#define SRC_OVERLAY FW(FWE_SRC_OVERLAY)
#define SRC_CACHE   FW(FWE_SRC_CACHE)
#define SCR_GROUP   FW(FWE_SCR_GROUP)
#define PWM_STATE   FW(FWE_PWM_STATE)
#define DMDL_HANDLE FW(FWE_DMDL_HANDLE)
#define SD_VARIANT0 FW(FWE_SD_VARIANT0)
#define SD_BUSY0    R32(FW(FWE_SD_BUSY0))
#define SH_MR0      FW(FWE_SH_MR)
#define SRC_TV      FW(FWE_SRC_TV)
#define SCR_TV      FW(FWE_SCR_TV)
#define PWM_ON      FW(FWE_PWM_ON)
#define DU_MR_YUY2  0x04104033u   /* our MR value (DU hardware format), not the firmware's */

/* ---- sdstream's own blocks (rnse.h) ---- */
#define CTL_BASE   ((u32)SDL_CTL(0))
#define STAT_BASE  ((u32)SDL_CTL(0x100))
#define STAT_MAGIC 0x41565344u
#define CTL_STOP   R32(CTL_BASE + 4)
#define CTL_FWT    62u            /* sdstream.c CTL_FWT: table address, word 63 = complement */
#define ST(i)      R32(STAT_BASE + 4 * (i))
#define PROG_SDSTREAM 6u
#define PROBE_BUF  ((u32)SDL_SFRING)   /* SFRING: 32-B aligned, unused until sdstream runs */
#define WIN_LBA    0x10000u
#define H_IDLE     4u
#define H_NOTREADY 64u

/* No post-boot grace (was 10 s): the run itself probes until the card mounts and the Pi feeds (250 ms, 1 s after a read error),
 * and the crash counter turns auto start off after CRASH_MAX resets during a stream. */
#define SETTLE_MS   1000u    /* TV entry finished re-registering its plane (DU_DESIGN 4B) before we take array 0 */
#define LEAVE_SRC_MS 1500u   /* sdstream sees a source change itself within 1 s (err 10, skips the route restore): let it, then force */
#define LEAVE_SCR_MS 300u
#define CRASH_MAX   3u

static inline u32 tick(void) { return R32(TICK_MS); }
static inline u32 be32(const u8 *p) { return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3]; }
#define dly_tsk(ms) ((int (*)(int))FW_DLY_TSK)(ms)

/* ------------------------------------------------------------------ table check: reads flash only, writes nothing */
static u32 crc32_bits(const u8 *p, u32 n)
{
    u32 c = 0xFFFFFFFFu, k;
    while (n--) {
        c ^= *p++;
        for (k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

#define FV(r, d) (0xFA000000u | ((u32)(r) << 16) | ((u32)(d) & 0xFFFFu))
static u32 fwt_check(const u32 *t)
{
    u32 a = (u32)t, i;
    if ((a & 3u) || a < FLASH_LO || a + FWT_NWORDS * 4u > FLASH_HI) return FV(FV_ADDR, 0);
    if (t[FWT_W_MAGIC] != FWT_MAGIC) return FV(FV_MAGIC, 0);
    if (t[FWT_W_VERSION] != FWT_VERSION && t[FWT_W_VERSION] != FWT_VERSION_TONE) return FV(FV_VERSION, t[FWT_W_VERSION]);
    if (t[FWT_W_NWORDS] != FWT_NWORDS) return FV(FV_NWORDS, t[FWT_W_NWORDS]);
    if (t[FWT_W_RAMMAP] != SDL_RAMMAP) return FV(FV_RAMMAP, t[FWT_W_RAMMAP]);
    if (crc32_bits((const u8 *)t, (FWT_NWORDS - 1u) * 4u) != t[FWT_NWORDS - 1u]) return FV(FV_CRC, 0);
    for (i = 0; i < FWE_NSIG; i++) {
        u32 f = t[FWT_HDR + i];
        if ((f & 1u) || f < FLASH_LO || f + 4u > FLASH_HI) return FV(FV_SIGADDR, i);
        if (be32((const u8 *)f) != t[FWT_SIG0 + i]) return FV(FV_SIG, i);
    }
    return 0;
}

/* the TVA block is ours only on a firmware that guarantees the RAM map: say why only if the table at least claims it */
static void fwt_fail(const u32 *t, u32 why)
{
    u32 a = (u32)t;
    if (!(a & 3u) && a >= FLASH_LO && a + FWT_NWORDS * 4u <= FLASH_HI && t[FWT_W_MAGIC] == FWT_MAGIC && t[FWT_W_RAMMAP] == SDL_RAMMAP)
        TVA(W_FWT_ST) = why;
}

/* ------------------------------------------------------------------ r61 logic, firmware through the table */
static void trace(u32 code, u32 a, u32 b)
{
    u32 k = TVA(W_TRN)++ % TR_N, p = TR_BASE + k * 16;
    R32(p) = tick(); R32(p + 4) = code | (TVA(W_STATE) << 16); R32(p + 8) = a; R32(p + 12) = b;
}

static inline u32 dormant_tv(void) { return R16(TCB_TV + 4) == 0x0010; }

static inline u32 want_now(void)
{
#if TVA_USE_SCREEN
    if (R16(SCR_GROUP) != SCR_TV) return 0;
#endif
    return R8(SRC_CTX) == SRC_TV && R8(PWM_STATE) == PWM_ON;
}

extern void tva_task(void);

/* ------------------------------------------------------------------ tone: SDLink on TV uses the TV source's bass/treble
 * (analysis/tone_source_v1/IMPLEMENTATION.md). The firmware keeps bass/treble per audio input in a battery-backed cache and picks the
 * slot by the input the audio controller REPORTS; SDLink plays through the IIS (SD/MP3) input, so stock would play and edit the MP3
 * player's tone while the TV page shows TV's. While SDLink plays on TV (state RUN, TV source, MMI on, reported input IIS, no overlay):
 *   enter: the IIS pair (sampled while SDLink did not hold the slot) is kept in W_TN_SNAP; TV's pair goes into the IIS slot through
 *          the firmware's own tone_set (slot store + checksum + backup record + UART to the controller);
 *   run:   a knob edit lands in the IIS slot (stock): copy it to the TV slot, save, write the TV pair of the HMI mirror, notify the HMI;
 *   leave: the IIS slot gets the snapshot back (saved; pushed to the controller when the MP3 player is the live input now).
 * Version-1 table, or an extension that does not check out: nothing here runs and nothing outside our RAM is written. */
#define TN_TAG    0x544E0000u                             /* 'TN' << 16 | bass << 8 | treble */
#define FWX_OK    0x544E4F4Bu                             /* 'TNOK' */
#define FX(r, d)  (0xFB000000u | ((u32)(r) << 16) | ((u32)(d) & 0xFFFFu))
enum { TN_OFF, TN_IDLE, TN_ACTIVE };
enum { FX_ADDR = 1, FX_MAGIC, FX_NWORDS, FX_CRC, FX_SIGADDR, FX_SIG, FX_LIT, FX_LAYOUT };
#define SRC_STATUS FW(FWE_SRC_STATUS)
#define CTRL_IIS   FW(FWE_CTRL_IIS)

/* table already passed fwt_check. -> extension address, or 0 (W_TN_EXT says why). Reads flash only. */
static const u32 *fwx_check(const u32 *t)
{
    const u32 *x = t + FWT_NWORDS;
    u32 a = (u32)x, i, lay, sl, mi;
    if (t[FWT_W_VERSION] != FWT_VERSION_TONE) { TVA(W_TN_EXT) = 0; return 0; }
    if (a + FWX_NWORDS * 4u > FLASH_HI) { TVA(W_TN_EXT) = FX(FX_ADDR, 0); return 0; }
    if (x[0] != FWX_MAGIC) { TVA(W_TN_EXT) = FX(FX_MAGIC, 0); return 0; }
    if (x[1] != FWX_NWORDS) { TVA(W_TN_EXT) = FX(FX_NWORDS, x[1]); return 0; }
    if (crc32_bits((const u8 *)x, (FWX_NWORDS - 1u) * 4u) != x[FWX_NWORDS - 1u]) { TVA(W_TN_EXT) = FX(FX_CRC, 0); return 0; }
    for (i = 0; i < FWX_NSIG; i++) {
        u32 f = x[FWX_HDR + i];
        if ((f & 1u) || f < FLASH_LO || f + 4u > FLASH_HI) { TVA(W_TN_EXT) = FX(FX_SIGADDR, i); return 0; }
        if (be32((const u8 *)f) != x[FWX_SIG0 + i]) { TVA(W_TN_EXT) = FX(FX_SIG, i); return 0; }
    }
    /* the data addresses must be the ones the firmware's own code loads */
    if (x[FWX_SIG0 + FWX_TN_CACHE_LIT] != x[FWX_HDR + FWX_TN_CACHE] || x[FWX_SIG0 + FWX_TN_MIRROR_LIT] != x[FWX_HDR + FWX_TN_MIRROR]) {
        TVA(W_TN_EXT) = FX(FX_LIT, 0); return 0;
    }
    lay = x[FWX_HDR + FWX_TN_LAYOUT]; sl = x[FWX_HDR + FWX_TN_SLOTS]; mi = x[FWX_HDR + FWX_TN_MIR];
    i = (sl >> 8) > (sl & 255u) ? sl >> 8 : sl & 255u;        /* highest slot used */
    if ((sl >> 8) > 15u || (lay >> 24) + i >= ((lay >> 16) & 255u) || ((lay >> 16) & 255u) + i >= ((lay >> 8) & 255u) ||
        ((lay >> 8) & 255u) > 0x40u || (lay & 3u) || (mi >> 8) > 0x40u || (mi & 255u) > 0x40u || (x[FWX_HDR + FWX_TN_CACHE] & 3u)) {
        TVA(W_TN_EXT) = FX(FX_LAYOUT, 0); return 0;
    }
    TVA(W_TN_EXT) = FWX_OK;
    return x;
}

#define XV(e) (x[FWX_HDR + FWX_##e])
static u32 tn_pair(const u32 *x, u32 slot)
{
    const volatile u8 *c = (const volatile u8 *)XV(TN_CACHE);
    u32 lay = XV(TN_LAYOUT);
    return ((u32)c[(lay >> 24) + slot] << 8) | c[((lay >> 16) & 255u) + slot];
}
static void tn_put(const u32 *x, u32 slot, u32 p)          /* raw slot write; the caller saves */
{
    volatile u8 *c = (volatile u8 *)XV(TN_CACHE);
    u32 lay = XV(TN_LAYOUT);
    c[(lay >> 24) + slot] = (u8)(p >> 8);
    c[((lay >> 16) & 255u) + slot] = (u8)p;
}
static void tn_save(const u32 *x) { ((void (*)(int))XV(TN_SAVE))(0); }
static void tn_set(const u32 *x, u32 p)                   /* both values into the REPORTED input's slot + controller */
{
    ((int (*)(int, int))XV(TN_SET))(2, (int)(p >> 8));
    ((int (*)(int, int))XV(TN_SET))(3, (int)(p & 255u));
}
static void tn_mirror(const u32 *x, u32 off, u32 p)        /* HMI mirror pair: treble, bass (805BCA1E order) */
{
    volatile u8 *m = (volatile u8 *)XV(TN_MIRROR);
    m[off] = (u8)p;
    m[off + 1u] = (u8)(p >> 8);
    ((void (*)(int, int))XV(TN_NOTIFY))((int)(XV(TN_NOTE) >> 16), (int)(XV(TN_NOTE) & 0xFFFFu));
}
static inline void tn_count(u32 shift) { TVA(W_TN_N) += 1u << shift; }   /* bytes: enters 24, syncs 16, restores 8, pushes 0 */

static void tone_poll(const u32 *x)
{
    u32 si = XV(TN_SLOTS) >> 8, sv = XV(TN_SLOTS) & 255u, iis, on, held;
    iis = tn_pair(x, si);
    held = R8(SRC_CTX) == SRC_TV && R8(SRC_STATUS) == CTRL_IIS;      /* the only state in which SDLink can hold the IIS slot */
    on = held && TVA(W_STATE) == TS_RUN && R8(PWM_STATE) == PWM_ON && !R8(SRC_OVERLAY);
    if (TVA(W_TN_ST) != TN_ACTIVE) {
        TVA(W_TN_ST) = TN_IDLE;
        if (!held || (TVA(W_TN_SNAP) >> 16) != (TN_TAG >> 16)) TVA(W_TN_SNAP) = TN_TAG | iis;   /* the MP3 player's own pair */
        if (!on) return;
        {
            u32 tv = tn_pair(x, sv);
            TVA(W_TN_ST) = TN_ACTIVE; TVA(W_TN_CUR) = TN_TAG | tv; tn_count(24);
            trace(TR_TONE, 1, (TVA(W_TN_SNAP) << 16) | tv);
            if (iis != tv) {
                tn_set(x, tv); tn_count(0);
                if (tn_pair(x, si) != tv) {                   /* reported input moved under us: undo, retry next tick */
                    trace(TR_TONE, 4, tn_pair(x, si));
                    tn_put(x, si, TVA(W_TN_SNAP) & 0xFFFFu); tn_save(x);
                    TVA(W_TN_ST) = TN_IDLE;
                }
            }
        }
        return;
    }
    if ((TN_TAG | iis) != TVA(W_TN_CUR)) {                   /* a knob edit landed in the IIS slot: it is TV's now */
        tn_put(x, sv, iis);
        tn_save(x);
        tn_mirror(x, XV(TN_MIR) & 255u, iis);
        TVA(W_TN_CUR) = TN_TAG | iis; tn_count(16);
        trace(TR_TONE, 2, iis);
    }
    if (on) return;
    {   /* leave: the MP3 player's pair back */
        u32 s = TVA(W_TN_SNAP) & 0xFFFFu;
        tn_put(x, si, s);
        if (R8(SRC_STATUS) == CTRL_IIS && R8(SRC_CTX) != SRC_TV && R8(PWM_STATE) == PWM_ON) { tn_set(x, s); tn_count(0); }   /* MP3 plays now */
        else tn_save(x);
        tn_mirror(x, XV(TN_MIR) >> 8, s);
        TVA(W_TN_ST) = TN_IDLE; tn_count(8);
        trace(TR_TONE, 3, s);
    }
}

static void sm_poll(u32 now);

/* ------------------------------------------------------------------ ACON task, 100 ms. Must not block. */
void tva_poll(const u32 *fwt)
{
    u32 now, v;
    const u32 *x;
    v = fwt_check(fwt);
    if (v) { fwt_fail(fwt, v); return; }                      /* never starts on a table that does not check out */
    TVA(W_FWT) = (u32)fwt;
    now = tick();
    if (TVA(W_MAGIC) != TVA_MAGIC || now < TVA(W_T_LAST)) {   /* first call after power-up, or after a reset that kept RAM */
        u32 i, off = 0, crash = 0, keep = 0;
        if (TVA(W_MAGIC) == TVA_MAGIC) {
            off = TVA(W_OFF) == OFF_MAGIC ? OFF_MAGIC : 0;   /* only traced now */
            crash = TVA(W_CRASH);
            if (TVA(W_STATE) != TS_IDLE) crash++;             /* the unit reset while a stream ran */
            keep = 1;                                         /* RAM kept: so is the tone state (a session cut by the reset is undone below) */
        }
        for (i = 0; i < 0x100 / 4 + TR_N * 4; i++) if (i != W_FWT && !(keep && i >= W_TN_ST && i <= W_TN_N)) TVA(i) = 0;
        TVA(W_OFF) = 0;   /* any reset re-arms auto start (off word: flash tools, bench); the crash guard below re-sets it */
        TVA(W_CRASH) = crash; TVA(W_T_BOOT) = now; TVA(W_BUILD) = TVA_BUILD_ID;
        TVA(W_MAGIC) = TVA_MAGIC;
        trace(TR_INIT, crash, off);
        if (crash >= CRASH_MAX) { TVA(W_OFF) = OFF_MAGIC; trace(TR_CRASHOFF, crash, 0); }
    }
    TVA(W_FWT_ST) = FWT_OK;
    TVA(W_T_LAST) = now;
    sm_poll(now);
    x = fwx_check(fwt);
    if (x) tone_poll(x);
    else TVA(W_TN_ST) = TN_OFF;
}

static void sm_poll(u32 now)
{
    u32 want, sample, st;
    sample = ((u32)R8(SRC_CTX) << 24) | (((u32)R16(SCR_GROUP) << 8) & 0xFFFF00u) | R8(PWM_STATE);
    if (sample != TVA(W_SAMPLE)) { TVA(W_SAMPLE) = sample; trace(TR_SAMPLE, sample, R8(SRC_OVERLAY)); }

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
        if (!dormant_tv()) return;                            /* Doom or a bench program */
        {
            int r;
            u32 tcb = TCB_TV;
            CTL_STOP = 0;
            TVA(W_STAGE) = 0;
            R32(tcb + 0x20) = (u32)tva_task;
            r = ((int (*)(int, int))FW_STA_TSK)((int)TASK_TV, 0);
            R32(tcb + 0x20) = REST_ENTRY;                     /* sta_tsk copied it into the launch frame (R60.md, 804CF3D2) */
            TVA(W_STA_RET) = (u32)r;
            if (r) { TVA(W_T_NEXT) = now + 2000; trace(TR_STARTFAIL, (u32)r, 0); return; }
            TVA(W_STATE) = TS_RUN; TVA(W_T_START) = now; TVA(W_T_NOWANT) = 0; TVA(W_MRBAD) = 0; TVA(W_STARTS)++;
            trace(TR_START, TVA(W_STARTS), 0);
        }
        return;
    }
    if (dormant_tv()) {                                       /* task ended: asked to, or by itself */
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
        /* TV (re)entry or a TV re-init rewrites array 0 under us (MR 04104033 -> stock TV MR, DU_DESIGN 4B): take it again */
        /* REMOVED 2026-11-10: the "DU heal" (stop + restart the stream when array 0's MR is not ours for 200 ms). sdstream (SD_LEAVEFIX) gives
         * the plane back and retakes it itself; the heal instead restarted the stream while a quadrant menu was open, and the new session's
         * takeover covered the menu (bench tvauto trace: heal MR 04001002 -> stop -> start 3.4 s later). W_MRBAD/W_HEALS stay for the layout. */
        TVA(W_MRBAD) = 0;
        if (why) {
            CTL_STOP = why == WHY_SRC ? 2u : 1u;              /* 2 = the user left: any nonzero stops; a sdstream that knows 2 can skip its route restore */
            TVA(W_REASON) = why; TVA(W_STATE) = TS_STOP; TVA(W_T_STOP) = now;
            trace(TR_STOP, why, sample);
        }
    }
}

/* ------------------------------------------------------------------ task (TASK_TV) */
static inline u32 stop_req(void) { return CTL_STOP != 0; }
#if !TVA_STANDBY
static inline void oc_inval(u32 a, u32 n) { u32 e = a + n; for (a &= ~31u; a < e; a += 32) __asm__ volatile("ocbi @%0" : : "r"(a) : "memory"); }

/* One sector at lba into PROBE_BUF: the call sdstream's sd_read_to makes, 150 ms bound. 0 = read ok. */
static u32 sd_read1(u32 lba)
{
    volatile int cnt = 1;
    int r;
    R32(SD_VARIANT0) = 0x00000100u;
    SD_BUSY0 = 1;
    r = ((int (*)(u32, void *, volatile int *, u32, int))FW_SD_RDCORE)(0, (void *)PROBE_BUF, &cnt, lba, 150);
    SD_BUSY0 = 0;
    return (short)r != 0 || cnt != 1;
}

/* 0 = stream alive, 1 = our card (RSAV) but no feed yet, 2 = read failed, 3 = not our card (no RSAV: the loop treats it as 1). */
static u32 probe(void)
{
    const u8 *sf = (const u8 *)PROBE_BUF;
    u32 hf, bad = sd_read1(WIN_LBA);
    TVA(W_PROBES)++;
    if (bad) return 2;
    oc_inval(PROBE_BUF, 512);
    hf = be32(sf + 12);
    TVA(W_PROBE_LAST) = hf;
    if (be32(sf) != 0x52534156u) return 3;                    /* 'RSAV': a normal SD card never reaches the takeover (nor the handshake) */
    if (hf & H_IDLE) return 1;
    if ((hf & H_NOTREADY) && !be32(sf + 64)) return 1;        /* Teensy up, no complete superframe from the Pi yet */
    return 0;
}

/* ---- version handshake (docs/PROTOCOL.md 4.5). Only after a probe saw 'RSAV', so a normal card never sees these reads.
 * announce(): 8 one-sector reads at SDL_HS_LBA + (i << 16) + w[i]; the Teensy decodes the LBAs (no card write, no CAN). A Teensy that
 * knows the handshake answers read 7 with 'SDHA', an echo of w[0..7] and the Pi's verdict; an older Teensy returns its usual data. */
enum { HS_NONE, HS_PENDING, HS_ACCEPT, HS_REJECT, HS_TIMEOUT };
#define HS_WAIT_MS 1000u   /* acked but no verdict from the Pi this long (old avlink, Pi not up): go on as before */

static inline u32 be16(const u8 *p) { return ((u32)p[0] << 8) | p[1]; }

static u32 announce(void)
{
    const u32 *t = (const u32 *)TVA(W_FWT);
    const u8 *s = (const u8 *)PROBE_BUF;
    u32 w[SDL_HS_N], i, sum = 0, v;
    w[0] = SDL_HS_W0 | SDL_HS_VER;
    w[1] = TVA_BUILD_ID >> 16;
    w[2] = TVA_BUILD_ID & 0xFFFFu;
    w[3] = (SDL_STREAM_FMT_MIN << 8) | SDL_STREAM_FMT;
    w[4] = t[FWT_W_FWID] >> 16;
    w[5] = t[FWT_W_FWID] & 0xFFFFu;
    w[6] = t[FWT_W_VERSION] & 0xFFFFu;
    for (i = 0; i < SDL_HS_N - 1; i++) sum += w[i];
    w[SDL_HS_N - 1] = ~sum & 0xFFFFu;
    TVA(W_HS_N)++;
    for (i = 0; i < SDL_HS_N; i++) if (sd_read1(SDL_HS_LBA + (i << 16) + w[i])) return HS_NONE;
    oc_inval(PROBE_BUF, 512);
    if (be32(s) != SDL_HS_ACK) return HS_NONE;
    for (i = 0; i < SDL_HS_N; i++) if (be16(s + 4 + 2 * i) != w[i]) return HS_NONE;
    TVA(W_HS_ACK) = be32(s + 20);                              /* verdict, chosen format, 0, 0 */
    v = s[20];
    return v == SDL_HS_ACCEPT ? HS_ACCEPT : v == SDL_HS_REJECT ? HS_REJECT : HS_PENDING;
}

/* r = probe() result for an RSAV card. Returns what the probe loop acts on: r, or 1 (wait) while the Pi has not accepted.
 * Old Teensy (no ack): r, re-announced every 8th probe (~2 s). Acked: announce on every probe until a verdict, at most HS_WAIT_MS,
 * then r (today's behaviour). Accept: r, no more reads this run. Reject: 1 (stay stock), re-announced every 4th probe (~1 s). */
static u32 hs_gate(u32 r)
{
    u32 st = TVA(W_HS_ST), k = TVA(W_HS_K)++, v;
    if (st == HS_ACCEPT) return r;
    if ((st == HS_NONE || st == HS_TIMEOUT) && (k & 7u)) return r;
    if (st == HS_REJECT && (k & 3u)) return 1;
    v = announce();
    if (v == HS_PENDING) {
        if (!TVA(W_HS_T0)) TVA(W_HS_T0) = tick() | 1u;
        if (st == HS_TIMEOUT || tick() - TVA(W_HS_T0) >= HS_WAIT_MS) v = HS_TIMEOUT;
    }
    TVA(W_HS_ST) = v;
    return v == HS_PENDING || v == HS_REJECT ? 1u : r;
}

#endif

static inline void icbi_range(u32 a, u32 e)
{
    register u32 p __asm__("r1") = a & ~31u;
    for (; p < e; p += 32) __asm__ volatile(".word 0x01E3" : : "r"(p) : "memory");   /* icbi @r1, as tramp.S */
}

/* Send the controller the input of the source that is selected now. Stock chose it when the user changed source, but sdstream's stop path can
 * send TV after that (its restore runs unless it saw the change itself). FW_SRCSEL is any-task, draws nothing (AUDIO_ROUTE_RE 2.2).
 * ctx -> srcsel idx: table nibbles IDX_LO (ctx 0..7) / IDX_HI (ctx 8..15), F = none; idx -> controller code: the firmware's own CTRL_TAB. */
static void route_fixup(void)
{
    u32 s = R8(SRC_CTX), idx;
    if (s >= 16) return;
    idx = ((s < 8 ? FW(FWE_IDX_LO) : FW(FWE_IDX_HI)) >> (4u * (s & 7u))) & 15u;
    if (idx == 15u || idx > 10u || R8(SRC_OVERLAY) || R8(PWM_STATE) != PWM_ON) return;
    if (R8(SRC_CACHE) == ((const volatile u8 *)FW(FWE_CTRL_TAB))[idx]) return;
    TVA(W_ROUTE) = idx + 1;
    trace(TR_ROUTE, s, R8(SRC_CACHE));
    ((int (*)(int))FW_SRCSEL)((int)idx);
}

void tva_task(void)
{
    static const u32 ctl[] = TVA_CTL_WORDS;
    extern const u32 tva_image[], tva_image_end[];
    u32 i;
    TVA(W_STAGE) = 1;
#if !TVA_STANDBY
    TVA(W_HS_ST) = HS_NONE; TVA(W_HS_K) = 0; TVA(W_HS_T0) = 0;   /* the handshake starts over with every run */
    for (;;) {                                                /* no takeover until the Pi really feeds: until then the unit stays stock */
        u32 r;
        if (stop_req()) goto out;
        r = probe();
        if (r < 2) r = hs_gate(r);                            /* our card: announce the versions, the Pi may veto */
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
    R32(CTL_BASE + 4 * CTL_FWT) = TVA(W_FWT);                 /* the table, for sdstream's fwt_load */
    R32(CTL_BASE + 4 * (CTL_FWT + 1)) = ~TVA(W_FWT);
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
