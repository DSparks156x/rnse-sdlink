/* avsd.h -- shared definitions for the A/V-over-SD RAM prototypes decbench and sdstream (in addition to rnse.h).
 * RAM map (all inside the doom_ram_v1 free block 0ED20000-0FAC0000; P1 cached write-through):
 *   8F4C0000 / 8F57B800 / 8F637000   three 800x480 YUY2 frame buffers (768000 B each, DU_DESIGN.txt section 2;
 *                                    overlaps Doom's heap -> Doom and these programs are exclusive, same task 159)
 *   8F700000-8F70FFFF  existing probe windows (tone48/sdprobe/sdbench/duprobe)
 *   8F710000           launchers, 8F720000 CTL, 8F720100 STAT (as before)
 *   8F721000-8F72FFFF  per-frame / per-superframe result arrays (RES_BASE)
 *   8F730000-8F73FFFF  PCM block buffers (TONE_BUF, shared with tone48)
 *   8F740000-8F75FFFF  decbench image window (0x20000)
 *   8F760000-8F77FFFF  sdstream image window (0x20000)
 *   8F780000-8F9FFFFF  decbench: test stream blob (RNVS, 2.5 MiB max)
 *   8F800000-8F83FFFF  sdstream: superframe read buffer (256 KiB, 32-B aligned -> zero-copy DMA)
 *   8F840000-8F8FFFFF  sdstream: video frame reassembly buffer 0 (768 KiB, frames spanning superframes)
 *   8F900000-8F91FFFF  sdstream: audio jitter FIFO (32768 frames x 4 B)
 *   8F920000-8F9DFFFF  sdstream: video frame reassembly buffer 1 (768 KiB)
 * decbench and sdstream are never resident at the same time (both run as task 159). */
#ifndef AVSD_H
#define AVSD_H
#include "rnse.h"

#define FB0          ((u32)SDL_FB_BASE)
#define FB1          ((u32)SDL_FB_BASE + 0xBB800u)
#define FB2          ((u32)SDL_FB_BASE + 0x177000u)
#define FB_BYTES     768000u
#define RES_BASE     ((u32)SDL_CTL(0x1000))
#define RES_END      ((u32)SDL_CTL(0x10000))
#define DEC_STREAM   ((u32)SDL_SFRING)
#define DEC_STREAM_MAX 0x280000u
#define SF_BUF       ((u32)SDL_ASM0)
#define SF_BUF_BYTES 0x40000u
#define AFIFO        ((u32)SDL_AFIFO)   /* ends 8F9A0000. Firmware sink: temp+byte FIFO 8F9B6320..8F9DA320 (read at 8EA19824), ring 8F9DA320.. : stay out, it silently overwrote our FIFO tail and stacks */
#define AFIFO_FRAMES 32768u

#define CD_GPIO      0xFF93E00Cu
#define SD_READ(s, l, b, n) ((int (*)(int, int, void *, int))FW_SD_READ)(s, l, b, n)
#define SND_OPN(p)          ((int (*)(u32 *))FW_SND_OPN)(p)
#define SND_DYD(h, b, l, a) ((int (*)(int, const void *, int, int))FW_SND_DYD)(h, b, l, a)
#define SND_STP(h)          ((int (*)(int))FW_SND_STP)(h)
#define SND_CLS(h)          ((int (*)(int))FW_SND_CLS)(h)

/* TMU0 counts per ms (Pck/4 = 12.4875 MHz) */
#define COUNTS_PER_MS 12488u

/* invalidate the operand cache over [a, a+n) (after DMA into RAM that may have been cached) */
static inline void oc_inval(u32 a, u32 n)
{
    u32 e = a + n;
    for (a &= ~31u; a < e; a += 32) __asm__ volatile("ocbi @%0" : : "r"(a) : "memory");
}
/* make write-through stores visible before the DU / DMA sees them: a read through the uncached alias */
static inline void wb_drain(u32 a) { (void)R32(P2(a & ~3u)); }

/* CRC-32 (zlib polynomial, reflected), slice-by-4 */
void crc32_init(void);
u32 crc32_upd(u32 crc, const u8 *p, u32 n);   /* crc starts at 0; returns the zlib value */

/* DU array-0 takeover in YUY2 (du_yuy2.c, Doom's rnse_video.c sequence with MR 04104033) */
int du_takeover(u32 phys_front);   /* returns 0, or 1 if already taken */
void du_flip(u32 phys);
void du_restore(void);
u32 du_mr(void);
u32 du_dssr(void);
#define DU_MR_YUY2   0x04104033u
#define DSA_PHYS(a)  ((u32)(a) & 0x1FFFFFF0u)

void *memcpy(void *d, const void *s, unsigned long n);
void *memset(void *d, int c, unsigned long n);
#endif
