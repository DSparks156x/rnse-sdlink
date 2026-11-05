/* rnv2_int.h -- internals of the RNV2 decoder (rnv2sh.c / rnv2tab.c). Pixel words are always handled in
 * BIG-ENDIAN byte order (Y0 Cb Y1 Cr = bits 31..0): native on the SH, byte-swapped on little-endian hosts by
 * LDW/STW, so the host harness executes exactly the SH arithmetic. */
#ifndef RNV2_INT_H
#define RNV2_INT_H
#include "rnv2sh.h"

#if defined(RNV_HOST) && defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
static inline rnv_u32 LDW(const void *p) { rnv_u32 v; __builtin_memcpy(&v, p, 4); return __builtin_bswap32(v); }
static inline void STW(void *p, rnv_u32 v) { v = __builtin_bswap32(v); __builtin_memcpy(p, &v, 4); }
#else
#define LDW(p) (*(const rnv_u32 *)(const void *)(p))
#define STW(p, v) (*(rnv_u32 *)(void *)(p) = (v))
#endif

#define RNV_PACK(y0, cb, y1, cr) (((rnv_u32)(y0) << 24) | ((rnv_u32)(cb) << 16) | ((rnv_u32)(y1) << 8) | (rnv_u32)(cr))
/* odd shift: w0=[Ya Cb Yb Cr] w1=[Yc ..] -> [Yb Cb Yc Cr] */
#define ODDW(w0, w1) ((((w0) & 0xFF00u) << 16) | ((w0) & 0x00FF00FFu) | (((w1) >> 16) & 0xFF00u))
#define AVG4(a, b) (((a) | (b)) - ((((a) ^ (b)) >> 1) & 0x7F7F7F7Fu))

#define RNV_RUNMAX 16
#define RNV_LVMAX 13
#define RNV_NSYM (2 * RNV_RUNMAX * RNV_LVMAX)
#define RNV_L1BITS 10

typedef struct {
    rnv_u16 l1[1 << RNV_L1BITS];   /* (info << 5) | code length, 0 = longer than 10 bits */
    int first[18], cnt[18], off[18];
    rnv_u16 info[RNV_NSYM];        /* coef: last<<8 | run<<4 | (level-1); cbp: symbol */
} rnv_huff;

extern rnv_huff rnv_hc, rnv_hbl, rnv_hbc;
extern rnv_u8 rnv_clamp[1024];
extern rnv_u8 rnv_clz8[256];
const int *rnv_qstep(void);

#endif
