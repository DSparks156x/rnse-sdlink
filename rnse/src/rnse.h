/* rnse_proto shared definitions: firmware entry points (r56b == r57, see refind_out.txt), RAM layout, status block.
 * All programs run as kernel task 159 (entered through tramp.S), never return, end with ext_tsk.
 * Big-endian SH-4A, built -m4-single but no floating point is used anywhere. */
#ifndef RNSE_H
#define RNSE_H
typedef unsigned int u32;
typedef int s32;
typedef unsigned short u16;
typedef short s16;
typedef unsigned char u8;

#define R32(a) (*(volatile u32 *)(a))
#define R16(a) (*(volatile u16 *)(a))

#ifdef FWTAB
/* ---- firmware through the table (fwtab.h): the RAM program copies it to g_fwt at start (sdstream.c fwt_load) ---- */
#include "fwtab.h"
extern u32 g_fwt[];
#define FWT_BASE_PTR g_fwt
#define FW_DLY_TSK   FW(FWE_DLY_TSK)
#define FW_EXT_TSK   FW(FWE_EXT_TSK)
#define FW_SND_OPN   FW(FWE_SND_OPN)
#define FW_SND_DYD   FW(FWE_SND_DYD)
#define FW_SND_STP   FW(FWE_SND_STP)
#define FW_SND_CLS   FW(FWE_SND_CLS)
#define FW_SD_READ   FW(FWE_SD_READ)
#define TICK_MS      FW(FWE_TICK_MS)
#define DMDL_HANDLE  FW(FWE_DMDL_HANDLE)
#else
/* ---- firmware (OSM r56b/r57; identical bytes at the r19 addresses, refind.py) ---- */
#define FW_DLY_TSK   0x804D04D8u   /* int dly_tsk(ms)  yields */
#define FW_EXT_TSK   0x804D027Cu   /* void ext_tsk(void) */
#define FW_SND_OPN   0x8062D654u   /* int Snd_opn(u32 param[9]) -> handle 0x02xxxxxx, synchronous */
#define FW_SND_DYD   0x8062D6CCu   /* int Snd_dyD(h, buf, len, aux): 0 ok, -2 queue full, -1 error; buf -1, len 0 = EOS. CORRECTED 2026-11-05 (swapped args = 2 GB runaway memcpy) */
#define FW_SND_STP   0x8062D71Au   /* int Snd_stp(h) */
#define FW_SND_CLS   0x8062D742u   /* int Snd_cls(h) */
#define FW_SD_READ   0x80653598u   /* int sd_raw_read(u16 slot, s32 lba, void *buf, s32 nsect): 0 ok, -1 error */

#define TICK_MS      0x8E8DA8B0u   /* kernel 1 ms tick counter (u32) */
#define DMDL_HANDLE  0x8FCE6294u   /* MP3 player's sink handle; 0 = PCM sink free */
#endif
#define dly_tsk(ms)  ((int (*)(int))FW_DLY_TSK)(ms)

/* ---- RAM / MMIO ---- */
#define TMU_TCNT0    0xFFD8000Cu   /* counts down 12487..0 at Pck/4 = 12.4875 MHz, one period per tick */
#define TMU_PER      12488u
#define SSI_CR       0xFF984000u
#define SSI_SR       0xFF984004u
#define SSI_CLKW     0xFF972000u

/* ---- our RAM region (inside the doom_ram_v1 free block 0ED20000-0FAC0000, outside Doom's 0EE00000-0F600000,
 *      outside the VIN redirect buffers 0F000000-0F4B9000, below OSM buffers 0FACA800) ---- */
#include "sdl_ram.h"
#define CTL_BASE     ((u32)SDL_CTL(0))       /* host-written parameters */
#define STAT_BASE    ((u32)SDL_CTL(0x100))   /* program status, read by the host with B1 */
#define TONE_BUF     ((u32)SDL_CTL(0x10000)) /* 4 x 16 KiB PCM block buffers (max 4096 frames each) */
#define SD_BUF       ((u32)SDL_ASM0)         /* 2 MiB SD read buffer (4096 sectors; bench probes only), 32-byte aligned -> zero-copy DMA */
#define P2(a)        (((u32)(a) & 0x1FFFFFFFu) | 0xA0000000u)

#define CTL_MAGIC    0x43544C31u   /* 'CTL1' */
#define STAT_MAGIC   0x41565344u   /* 'AVSD' */
#define DONE_MAGIC   0xD0DE0000u

/* CTL words */
#define CTL_W(i)     R32(CTL_BASE + 4 * (i))
#define CTL_MAGIC_W  0
#define CTL_STOP     1
#define CTL_P(i)     CTL_W(2 + (i))

/* STAT words (common header 0..7, then program-specific) */
#define ST(i)        R32(STAT_BASE + 4 * (i))
#define ST_MAGIC     0
#define ST_PROG      1
#define ST_STAGE     2
#define ST_ERR       3
#define ST_HB        4
#define ST_T0        5
#define ST_T1        6
#define ST_DONE      7

#define STAGE_ERR    0xEE
#define STAGE_DONE   0xFE

static inline u32 tick(void) { return R32(TICK_MS); }

/* TMU-resolution timestamp: ticks * 12488 + elapsed counts in the current tick (wraps every ~344 s) */
static inline u32 now_counts(void)
{
    u32 a, c, b;
    do { a = R32(TICK_MS); c = R32(TMU_TCNT0); b = R32(TICK_MS); } while (a != b);
    if (c > TMU_PER - 1) c = TMU_PER - 1;
    return a * TMU_PER + (TMU_PER - 1 - c);
}

static inline void st_begin(u32 prog)
{
    u32 i;
    for (i = 0; i < 64; i++) ST(i) = 0;
    ST(ST_PROG) = prog;
    ST(ST_STAGE) = 1;
    ST(ST_T0) = tick();
    ST(ST_MAGIC) = STAT_MAGIC;
}

static inline int ctl_ok(void) { return CTL_W(CTL_MAGIC_W) == CTL_MAGIC; }
static inline int stop_req(void) { return CTL_W(CTL_STOP) != 0; }

static inline void st_end(u32 prog, u32 err)
{
    ST(ST_ERR) = err;
    ST(ST_T1) = tick();
    ST(ST_STAGE) = err ? STAGE_ERR : STAGE_DONE;
    ST(ST_DONE) = DONE_MAGIC | prog;
}
#endif
