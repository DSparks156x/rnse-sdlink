/* sdl_ram.h -- the SDLink RAM map in ONE place (sdstream, avlib, tvauto, hook v3 via build_fw.py, sims). Three relocatable regions;
 * everything else is a fixed offset inside one of them. The firmware table's ram_map word promises that these regions are free on
 * that firmware (fwtab.h FWT_RAMMAP_*). Defaults = the r57..r64 map (ram_map 1); a build may -D the three bases.
 *
 *   FB   3 x 768000 B YUY2 frame buffers (DU scan-out), 32-B aligned. May overlap Doom (task 159 is exclusive).
 *   CTL  0x20000: +0 CTL words, +0x100 STAT, +0x1000..+0x10000 RES log ring, +0xF000 TVA state (tvauto, 0x900), +0xFF00 hook v3 cache,
 *        +0xFFD0 scan debug words, +0xFFF0 KEYREQ, +0xFFF4 key slot, +0x10000..+0x20000 TONE_BUF (4 x 16 KiB PCM blocks).
 *        Touched by tvauto every 100 ms even while Doom runs: must never overlap Doom's RAM.
 *   WORK 0x25E000: +0 trace (TR_R/D/H, +0x3000 word), +0x8000 sdstream image window (0x20000, link base), +0x28000 SF ring,
 *        +0xA7800 LEAVETRACE ring (0x800), +0xA8000 reassembly 0 (= SF_BUF), +0x168000 reassembly 1, +0x228000 audio FIFO (0x20000),
 *        +0x248000..+0x258000 reader stack, +0x258000..+0x25E000 audio stack. May overlap Doom. */
#ifndef SDL_RAM_H
#define SDL_RAM_H
#ifndef SDL_FB_BASE
#define SDL_FB_BASE    0x8F4C0000
#endif
#ifndef SDL_CTL_BASE
#define SDL_CTL_BASE   0x8F720000
#endif
#ifndef SDL_WORK_BASE
#define SDL_WORK_BASE  0x8F758000
#endif
#define SDL_FB_BYTES   0x232800
#define SDL_CTL_BYTES  0x20000
#define SDL_WORK_BYTES 0x25E000
/* the range a RAM-resident firmware table (bench loader) may live in */
#ifndef SDL_RAM_LO
#define SDL_RAM_LO     0x8F700000
#define SDL_RAM_HI     0x8FA00000
#endif

/* the map version a firmware table must promise (fwtab.h word 4): 1 = this default map (r57..r64 NA free block) */
#ifndef SDL_RAMMAP
#define SDL_RAMMAP     1
#endif
#define SDL_STR_(x)    #x
#define SDL_XSTR(x)    SDL_STR_(x)
#define SDL_CTL(o)     (SDL_CTL_BASE + (o))
#define SDL_WORK(o)    (SDL_WORK_BASE + (o))
#define SDL_TVA_BASE   SDL_CTL(0xF000)
#define SDL_HOOKCACHE  SDL_CTL(0xFF00)
#define SDL_SCANW      SDL_CTL(0xFFD0)
#define SDL_KEYREQ     SDL_CTL(0xFFF0)
#define SDL_KSLOT      SDL_CTL(0xFFF4)
#define SDL_TR_R       SDL_WORK(0x0000)
#define SDL_TR_D       SDL_WORK(0x1000)
#define SDL_TR_H       SDL_WORK(0x2000)
#define SDL_TR_Z       SDL_WORK(0x3000)
#define SDL_IMG        SDL_WORK(0x8000)
#define SDL_SFRING     SDL_WORK(0x28000)
#define SDL_LT         SDL_WORK(0xA7800)
#define SDL_ASM0       SDL_WORK(0xA8000)
#define SDL_ASM1       SDL_WORK(0x168000)
#define SDL_AFIFO      SDL_WORK(0x228000)
#define SDL_RD_STACK   SDL_WORK(0x258000)   /* top */
#define SDL_AU_STACK   SDL_WORK(0x25E000)   /* top */
#endif
