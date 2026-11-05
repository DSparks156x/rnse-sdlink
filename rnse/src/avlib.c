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

/* ---------------------------------------------------------------- DU (addresses = doom/port/rnse_hw.h) */
#define SH_MR      0x8E8E7D14u
#define SH_MWR     0x8E8E7D3Cu
#define SH_ALPHAR  0x8E8E7D64u
#define SH_DSXR    0x8E8E7D84u
#define SH_DSYR    0x8E8E7DACu
#define SH_DPXR    0x8E8E7DD4u
#define SH_DPYR    0x8E8E7DFCu
#define SH_DSA0    0x8E8E7E24u
#define SH_DSA1    0x8E8E7E4Cu
#define SH_DSA2    0x8E8E7E74u
#define SH_SPXR    0x8E8E7E9Cu
#define SH_SPYR    0x8E8E7EC4u
#define SH_WASPR   0x8E8E7EECu
#define SH_WAMWR   0x8E8E7F14u
#define SH_BTR     0x8E8E7F3Cu
#define SH_TC1R    0x8E8E7F64u
#define SH_TC2R    0x8E8E7F84u
#define SH_MLR     0x8E8E7FA4u
#define SH_DPPR    0x8E8E7FCCu
#define MASK_MODE  0x8E8E97D4u
#define DIRTY      0x8E8E97D8u
#define LAYER0     0x8E8E9804u
#define COMMIT_FN  0x80520908u
#define DU_DSYSR   0xFFF80000u
#define DU_DSSR    0xFFF80008u
#define DSYSR_HOLD 0x00010000u
#define DU_P1_DSA0 0xFFF80120u

static const u32 save_addr[] = {
    SH_MR, SH_MWR, SH_ALPHAR, SH_DSXR, SH_DSYR, SH_DPXR, SH_DPYR, SH_DSA0, SH_DSA1, SH_DSA2, SH_SPXR, SH_SPYR,
    SH_WASPR, SH_WAMWR, SH_BTR, SH_TC1R, SH_TC2R, SH_MLR, SH_DPPR, MASK_MODE, LAYER0,
};
#define NSAVE (sizeof save_addr / sizeof save_addr[0])
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
    for (i = 0; i < NSAVE; i++) saved[i] = R32(save_addr[i]);
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
    for (i = 0; i < NSAVE; i++) R32(save_addr[i]) = saved[i];
    commit(3);
}

u32 du_dssr(void) { return R32(DU_DSSR); }
