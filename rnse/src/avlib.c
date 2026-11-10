/* avlib.c -- small runtime for decbench/sdstream: memcpy/memset (the reference decoder and gcc may call them),
 * CRC-32 (zlib), DU array-0 YUY2 takeover / flip / restore.
 * DU sequence = doom/port/rnse_video.c (proven on the bench 2026-10-28: save array-0 shadows + DPPR + mask mode +
 * layer state, write shadows, commit 80520908(3); flip = DSA0-2 shadows + MMIO under the DSYSR bit16 hold; restore
 * = write the saved shadows back + commit(3)). Only the MR differs: 04104033 = YUY2 Y0 Cb Y1 Cr (proven live on the
 * TV plane 2026-10-29, DU_DESIGN.txt section 1). Stride MWR = 800 pixels. */
#include "avsd.h"

void *memcpy(void *d, const void *s, unsigned long n)
{
    u8 *dd = d; const u8 *ss = s;
    if (!(((u32)dd | (u32)ss | n) & 3)) {
        u32 *dw = (u32 *)dd; const u32 *sw = (const u32 *)ss;
        for (n >>= 2; n; n--) *dw++ = *sw++;
        return d;
    }
    while (n--) *dd++ = *ss++;
    return d;
}

void *memset(void *d, int c, unsigned long n)
{
    u8 *dd = d;
    while (n--) *dd++ = (u8)c;
    return d;
}

/* ---------------------------------------------------------------- CRC-32 */
static u32 ct[4][256];

void crc32_init(void)
{
    u32 i, k, v;
    for (i = 0; i < 256; i++) {
        v = i;
        for (k = 0; k < 8; k++) v = (v >> 1) ^ (0xEDB88320u & (0u - (v & 1)));
        ct[0][i] = v;
    }
    for (i = 0; i < 256; i++) {
        v = ct[0][i];
        for (k = 1; k < 4; k++) { v = ct[0][v & 0xFF] ^ (v >> 8); ct[k][i] = v; }
    }
}

u32 crc32_upd(u32 crc, const u8 *p, u32 n)
{
    u32 c = ~crc;
    while (n && ((u32)p & 3)) { c = ct[0][(c ^ *p++) & 0xFF] ^ (c >> 8); n--; }
    while (n >= 4) {
        u32 w = *(const u32 *)p;                      /* big-endian word: bytes b0 b1 b2 b3 */
        w = (w >> 24) | ((w >> 8) & 0xFF00u) | ((w << 8) & 0xFF0000u) | (w << 24);   /* -> b0 in the low byte */
        c ^= w;
        c = ct[3][c & 0xFF] ^ ct[2][(c >> 8) & 0xFF] ^ ct[1][(c >> 16) & 0xFF] ^ ct[0][c >> 24];
        p += 4; n -= 4;
    }
    while (n--) c = ct[0][(c ^ *p++) & 0xFF] ^ (c >> 8);
    return ~c;
}

/* ---------------------------------------------------------------- DU (firmware shadow words through the table, fwtab.h; values = doom/port/rnse_hw.h) */
#ifndef FWTAB
#error "avlib.c is table-only now (build -DFWTAB); the literal r57 addresses are in avlib.c.before_fwtab"
#endif
#define SH_MR      FW(FWE_SH_MR)
#define SH_MWR     FW(FWE_SH_MWR)
#define SH_ALPHAR  FW(FWE_SH_ALPHAR)
#define SH_DSXR    FW(FWE_SH_DSXR)
#define SH_DSYR    FW(FWE_SH_DSYR)
#define SH_DPXR    FW(FWE_SH_DPXR)
#define SH_DPYR    FW(FWE_SH_DPYR)
#define SH_DSA0    FW(FWE_SH_DSA0)
#define SH_DSA1    FW(FWE_SH_DSA1)
#define SH_DSA2    FW(FWE_SH_DSA2)
#define SH_WASPR   FW(FWE_SH_WASPR)
#define SH_WAMWR   FW(FWE_SH_WAMWR)
#define SH_TC2R    FW(FWE_SH_TC2R)
#define SH_MLR     FW(FWE_SH_MLR)
#define SH_DPPR    FW(FWE_SH_DPPR)
#define MASK_MODE  FW(FWE_SH_MASK)
#define DIRTY      FW(FWE_SH_DIRTY)
#define LAYER0     FW(FWE_SH_LAYER0)
#define COMMIT_FN  FW(FWE_DU_COMMIT)
#define DU_DSYSR   0xFFF80000u
#define DU_DSSR    0xFFF80008u
#define DSYSR_HOLD 0x00010000u
#define DU_P1_DSA0 0xFFF80120u

static const u8 save_idx[] = {
    FWE_SH_MR, FWE_SH_MWR, FWE_SH_ALPHAR, FWE_SH_DSXR, FWE_SH_DSYR, FWE_SH_DPXR, FWE_SH_DPYR, FWE_SH_DSA0, FWE_SH_DSA1, FWE_SH_DSA2,
    FWE_SH_SPXR, FWE_SH_SPYR, FWE_SH_WASPR, FWE_SH_WAMWR, FWE_SH_BTR, FWE_SH_TC1R, FWE_SH_TC2R, FWE_SH_MLR, FWE_SH_DPPR, FWE_SH_MASK,
    FWE_SH_LAYER0,
};
#define NSAVE (sizeof save_idx / sizeof save_idx[0])
static u32 saved[NSAVE];
static int du_on;

static void commit(u32 flags)
{
    R32(DIRTY) |= flags;
    ((int (*)(int))COMMIT_FN)((int)flags);
}

int du_takeover(u32 phys)
{
    u32 i;
    if (du_on) return 1;
    for (i = 0; i < NSAVE; i++) saved[i] = R32(FW(save_idx[i]));
    R32(SH_MR) = DU_MR_YUY2;
    R32(SH_MWR) = 800;
    R32(SH_ALPHAR) = 0xFF;
    R32(SH_DSXR) = 800;
    R32(SH_DSYR) = 480;
    R32(SH_DPXR) = 0;
    R32(SH_DPYR) = 0;
    R32(SH_DSA0) = phys; R32(SH_DSA1) = phys; R32(SH_DSA2) = phys;
    R32(SH_WASPR) = 0x3FF0;
    R32(SH_WAMWR) = 0xFFF;
    R32(SH_TC2R) = 0x7C1F;
    R32(SH_MLR) = 480;
    R32(SH_DPPR) |= 8;
    R32(LAYER0) = 1;
    R32(MASK_MODE) = 0;
    commit(3);
    du_on = 1;
    return 0;
}

void du_flip(u32 phys)
{
    if (!du_on) return;
    R32(SH_DSA0) = phys; R32(SH_DSA1) = phys; R32(SH_DSA2) = phys;
    R32(DU_DSYSR) |= DSYSR_HOLD;
    R32(DU_P1_DSA0) = phys; R32(DU_P1_DSA0 + 4) = phys; R32(DU_P1_DSA0 + 8) = phys;
    R32(DU_DSYSR) &= ~DSYSR_HOLD;
}

void du_restore(void)
{
    u32 i;
    if (!du_on) return;
    du_on = 0;
    for (i = 0; i < NSAVE; i++) R32(FW(save_idx[i])) = saved[i];
    commit(3);
}

u32 du_dssr(void) { return R32(DU_DSSR); }

/* array-0 mode word as the firmware last left it: anything but ours means it is taking the plane back */
u32 du_mr(void) { return R32(SH_MR); }
