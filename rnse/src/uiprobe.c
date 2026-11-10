/* uiprobe: READ-ONLY. Which DU planes does the stock UI scan out (array/plane, address, format, stride), and how often
 * does the picture actually change? (CAPTURE.txt, for the channel-2 screen capture.)
 *
 * Reads: the firmware's DU shadows in OSM .bss (doom_display_v1/FINDINGS.md: MR 8E8E7D14 + 4a, MWR 7D3C, ALPHAR 7D64,
 * DSXR 7D84, DSYR 7DAC, DPXR 7DD4, DPYR 7DFC, DSA0-2 7E24/7E4C/7E74, SPXR 7E9C, SPYR 7EC4, DPPR 8E8E7FCC, mask mode
 * 8E8E97D4, layer state 8E8E9804 + 0x38a), MMIO DSYSR FFF80000 and DPPR FFF80018 (both read live before by
 * duprobe/vin888), optionally (p2 bit0) plane registers Pn +00..+34 at FFF80100 + 0x100(n-1).
 * NO DU WRITE, no DSRCR access, no MR probing. Frame buffers are only read: through P1 after OCBP of each line
 * (write-back + invalidate; never discards data), or through P2 uncached (p2 bit1).
 *
 * Plane selection: enabled entries of DPPR (MMIO; if it reads 0, the shadow), colour arrays 0..5 only, up to 4, top
 * priority first; or an explicit array mask p1. Array a = plane P(a+1). Sanity: DSA0 phys in [p4, p5), size fits.
 * Per sample: every 64x16 tile of each plane is hashed (rotl5 ^ word); changed tiles vs the previous sample are
 * counted -> histogram, per-tile heat map, per-sample log. Samples are paced by the 1 ms tick (16.67 ms mean, or p3
 * ms) and the task always yields (dly_tsk >= 2 ms per sample) so the UI keeps running.
 *
 * CTL: p0 seconds (1..120, default 10), p1 array mask (0 = auto), p2 flags (bit0 MMIO plane snapshot, bit1 P2 reads),
 *      p3 interval ms (0 = vsync rate 16.67), p4/p5 phys window (default 08000000/10000000).
 * STAT (word index): 8 nplanes 9 samples 10 overruns 11 secs 12 flags 13 DPPR mmio 14 DPPR shadow 15 mask mode
 *      16 period us 17 DSYSR 19 selection source (0 mmio DPPR, 1 shadow DPPR, 2 p1 mask) 20 nevents 21 logged samples
 *      23/24 phys window; 32 + 14a (a = 0..9): MR MWR ALPHAR DSXR DSYR DPXR DPYR DSA0 DSA1 DSA2 SPXR SPYR layerstate sel
 *      (sel: bit31 in DPPR, bits 0-3 priority, bit8 measured, bits 16-23 skip reason);
 *      176 + 16k (measured plane k): array bpp w h stride ntx nty ntiles base_phys cnt_min cnt_max cnt_sum
 *      changed_samples changed_tiles max_changed 0;  240 + 11n: MMIO Pn +00 04 10 14 18 1C 20 24 28 30 34 (if bit0);
 *      336 + 3e: events (tick, plane<<8 | kind [1 DSA0, 2 MR, 3 DPPR, 4 SP], new value).
 * RES 8F721000: hist[4][512] u32 (bucket 511 = >= 511); +2000 heat[4][1024] u16; +4000 log, 4 words per sample:
 *      tick, ch0<<16|ch1, ch2<<16|ch3, flags (bits 0-3 DSA0 changed, bit8 overrun) | hash time in 10 us << 16. */
#include "rnse.h"

#define PROG_UIPROBE 0x20
#define SHB(off, a)  R32(0x8E8E0000u + (off) + 4u * (a))
#define DPPR_SH      0x8E8E7FCCu
#define MASK_MODE    0x8E8E97D4u
#define LAYER_ST(a)  R32(0x8E8E9804u + 0x38u * (a))
#define DU_DSYSR     0xFFF80000u
#define DU_DPPR      0xFFF80018u
#define RES          0x8F721000u
#define HIST(k)      ((volatile u32 *)(RES + 0x800u * (k)))
#define HEAT(k)      ((volatile u16 *)(RES + 0x2000u + 0x800u * (k)))
#define LOGW(i)      R32(RES + 0x4000u + 4u * (i))
#define LOG_MAX      ((0x8F730000u - RES - 0x4000u) / 16u)
#define MAXP         4
#define MAXT         1024

static const u16 shoff[12] = { 0x7D14, 0x7D3C, 0x7D64, 0x7D84, 0x7DAC, 0x7DD4, 0x7DFC, 0x7E24, 0x7E4C, 0x7E74,
                               0x7E9C, 0x7EC4 };
static const u8 mmoff[11] = { 0x00, 0x04, 0x10, 0x14, 0x18, 0x1C, 0x20, 0x24, 0x28, 0x30, 0x34 };

typedef struct { u32 a, bpp, w, h, stride, ntx, nty, nt, dsa, mr, sp, base; } plane;
static plane P[MAXP];
static u32 prevh[MAXP][MAXT];
static u32 rowh[64];
static u32 nev;

static void event(u32 code, u32 v)
{
    if (nev < 120) { ST(336 + 3 * nev) = tick(); ST(337 + 3 * nev) = code; ST(338 + 3 * nev) = v; nev++; ST(20) = nev; }
}

static u32 bpp_of(u32 mr) { return (mr & 3) == 0 ? 1 : 2; }   /* DDDF 0 PAL8, 1 RGB565, 2 ARGB1555, 3 YC */

/* hash all tiles of plane p into h[]; returns 0 */
static void hash_plane(const plane *p, u32 *h, u32 uncached)
{
    u32 ty, r, tx, y;
    u32 base = uncached ? ((p->base & 0x1FFFFFFFu) | 0xA0000000u) : ((p->base & 0x1FFFFFFFu) | 0x80000000u);
    u32 rowb = p->w * p->bpp, tb = 64 * p->bpp;
    for (ty = 0; ty < p->nty; ty++) {
        for (tx = 0; tx < p->ntx; tx++) rowh[tx] = 0x9E3779B9u ^ (ty * p->ntx + tx);
        for (r = 0; r < 16; r++) {
            u32 a;
            y = ty * 16 + r;
            if (y >= p->h) break;
            a = base + y * p->stride;
            if (!uncached) {
                u32 l, e = a + rowb;
                for (l = a & ~31u; l < e; l += 32) __asm__ volatile("ocbp @%0" : : "r"(l) : "memory");
            }
            for (tx = 0; tx < p->ntx; tx++) {
                u32 x0 = tx * tb, n = rowb - x0 < tb ? rowb - x0 : tb, hv = rowh[tx], k;
                const volatile u32 *w = (const volatile u32 *)(a + x0);
                for (k = 0; k + 4 <= n; k += 4, w++) hv = ((hv << 5) | (hv >> 27)) ^ *w;
                for (; k < n; k++) hv = ((hv << 5) | (hv >> 27)) ^ *(const volatile u8 *)(a + x0 + k);
                rowh[tx] = hv;
            }
        }
        for (tx = 0; tx < p->ntx; tx++) h[ty * p->ntx + tx] = rowh[tx];
    }
}

int main(void)
{
    u32 i, k, a, secs, mask, flags, period_us, lo, hi, dppr, dppr_sh, src, np = 0, ns = 0, over = 0, t0, nlog = 0;
    static u32 cur[MAXT];
    st_begin(PROG_UIPROBE);
    for (i = 64; i < 960; i++) ST(i) = 0;
    if (!ctl_ok()) { st_end(PROG_UIPROBE, 1); return 0; }
    secs = CTL_P(0); if (secs < 1 || secs > 120) secs = 10;
    mask = CTL_P(1) & 0xFF; flags = CTL_P(2); period_us = CTL_P(3) ? CTL_P(3) * 1000 : 16667;
    lo = CTL_P(4) ? CTL_P(4) : 0x08000000u; hi = CTL_P(5) ? CTL_P(5) : 0x10000000u;
    ST(11) = secs; ST(12) = flags; ST(16) = period_us; ST(23) = lo; ST(24) = hi;
    dppr = R32(DU_DPPR); dppr_sh = R32(DPPR_SH);
    ST(13) = dppr; ST(14) = dppr_sh; ST(15) = R32(MASK_MODE); ST(17) = R32(DU_DSYSR);
    src = mask ? 2 : dppr ? 0 : 1;
    ST(19) = src;
    if (src == 1) dppr = dppr_sh;
    /* snapshot of all 10 shadow arrays */
    for (a = 0; a < 10; a++) {
        for (i = 0; i < 12; i++) ST(32 + 14 * a + i) = SHB(shoff[i], a);
        ST(32 + 14 * a + 12) = LAYER_ST(a);
    }
    if (flags & 1)
        for (a = 0; a < 8; a++) for (i = 0; i < 11; i++) ST(240 + 11 * a + i) = R32(0xFFF80100u + 0x100u * a + mmoff[i]);
    /* selection in priority order (DPPR nibble n-1 = priority n: bit3 enable, bits 2-0 plane-1) */
    for (k = 0; k < 8; k++) {
        u32 nib = dppr >> (4 * k) & 0xF, sel;
        a = nib & 7;
        sel = (nib & 8) ? (0x80000000u | (k + 1)) : 0;
        if (nib & 8) ST(32 + 14 * a + 13) |= sel;
    }
    for (k = 0; k < 8 && np < MAXP; k++) {
        u32 nib = dppr >> (4 * k) & 0xF, why = 0, mr, mwr, w, h, bpp, dsa, spx, spy;
        plane *p = &P[np];
        a = nib & 7;
        if (mask) { if (k >= 8) break; a = k; if (!(mask >> a & 1)) continue; }
        else if (!(nib & 8)) continue;
        mr = SHB(0x7D14, a); mwr = SHB(0x7D3C, a); w = SHB(0x7D84, a); h = SHB(0x7DAC, a);
        dsa = SHB(0x7E24, a) & 0x1FFFFFF0u; spx = SHB(0x7E9C, a); spy = SHB(0x7EC4, a);
        bpp = bpp_of(mr);
        if (!mask && a > 5) why = 1;                                    /* alpha planes P7/P8 */
        else if (w < 16 || w > 2048 || h < 16 || h > 2048) why = 2;
        else if (mwr < w || mwr > 4096) why = 3;
        else if (spx > 4096 || spy > 4096) why = 4;
        else if (((w + 63) / 64) * ((h + 15) / 16) > MAXT) why = 5;
        else {
            u32 base = dsa + (spy * mwr + spx) * bpp, end = base + (h - 1) * mwr * bpp + w * bpp;
            if (base < lo || end > hi || end < base) why = 6;
            else if ((mwr * bpp) & 3) why = 7;
            p->base = base;
        }
        if (why) { ST(32 + 14 * a + 13) |= why << 16; continue; }
        p->a = a; p->bpp = bpp; p->w = w; p->h = h; p->stride = mwr * bpp; p->ntx = (w + 63) / 64; p->nty = (h + 15) / 16;
        p->nt = p->ntx * p->nty; p->dsa = dsa; p->mr = mr; p->sp = spx << 16 | spy;
        ST(32 + 14 * a + 13) |= 0x100;
        { u32 *s = (u32 *)&ST(176 + 16 * np); s[0] = a; s[1] = bpp; s[2] = w; s[3] = h; s[4] = p->stride; s[5] = p->ntx;
          s[6] = p->nty; s[7] = p->nt; s[8] = p->base; s[9] = 0xFFFFFFFFu; }
        np++;
    }
    ST(8) = np;
    for (k = 0; k < MAXP; k++) { for (i = 0; i < 512; i++) HIST(k)[i] = 0; for (i = 0; i < MAXT; i++) HEAT(k)[i] = 0; }
    if (!np) { st_end(PROG_UIPROBE, 2); return 0; }
    ST(ST_STAGE) = 2;
    t0 = tick();
    for (ns = 0; !stop_req(); ns++) {
        u32 ch[MAXP] = { 0, 0, 0, 0 }, fl = 0, c0, c1, due, now;
        c0 = now_counts();
        for (k = 0; k < np; k++) {
            plane *p = &P[k];
            u32 mr = SHB(0x7D14, p->a), dsa = SHB(0x7E24, p->a) & 0x1FFFFFF0u, sp = SHB(0x7E9C, p->a) << 16 | SHB(0x7EC4, p->a);
            u32 c, d, *s = (u32 *)&ST(176 + 16 * k);
            if (dsa != p->dsa) { event(k << 8 | 1, dsa); fl |= 1u << k;
                p->base = dsa + ((sp & 0xFFFF) * (p->stride / p->bpp) + (sp >> 16)) * p->bpp;
                if (p->base < lo || p->base + p->h * p->stride > hi) { st_end(PROG_UIPROBE, 3); return 0; }
                p->dsa = dsa; }
            if (mr != p->mr) { event(k << 8 | 2, mr); p->mr = mr; }
            if (sp != p->sp) { event(k << 8 | 4, sp); p->sp = sp; }
            c = now_counts();
            hash_plane(p, cur, flags & 2);
            d = now_counts() - c;
            if (d < s[9]) s[9] = d;
            if (d > s[10]) s[10] = d;
            s[11] += d;
            if (ns) {
                for (i = 0; i < p->nt; i++) if (cur[i] != prevh[k][i]) { ch[k]++; if (HEAT(k)[i] < 0xFFFF) HEAT(k)[i]++; }
                HIST(k)[ch[k] < 511 ? ch[k] : 511]++;
                if (ch[k]) { s[12]++; s[13] += ch[k]; if (ch[k] > s[14]) s[14] = ch[k]; }
            }
            for (i = 0; i < p->nt; i++) prevh[k][i] = cur[i];
        }
        { u32 v = R32(DU_DPPR); if (v != dppr && src == 0) { event(3, v); dppr = v; } }
        c1 = now_counts();
        now = tick();
        due = t0 + (ns + 1) * period_us / 1000;
        if ((s32)(due - now) < 2) { over++; fl |= 0x100; due = now + 2; }
        if (nlog < LOG_MAX) {
            u32 *l = (u32 *)&LOGW(4 * nlog);
            l[0] = now; l[1] = ch[0] << 16 | ch[1]; l[2] = ch[2] << 16 | ch[3];
            l[3] = fl | (((c1 - c0) / 125u > 0xFFFF ? 0xFFFF : (c1 - c0) / 125u) << 16);
            nlog++;
        }
        ST(9) = ns + 1; ST(10) = over; ST(21) = nlog; ST(ST_HB) = ns;
        dly_tsk(due - now);
        if (tick() - t0 >= secs * 1000) { ns++; break; }
    }
    ST(9) = ns;
    st_end(PROG_UIPROBE, 0);
    return 0;
}
