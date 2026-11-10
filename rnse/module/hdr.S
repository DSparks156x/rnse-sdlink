/* SDLink v2 SDST payload head (XIP at BASE: phys 00D60080 / P1 80D60080 since 2026-11-10, was 80C00080). Hook v2 checks the magic 'TVA2', then calls +4 (tva_poll) with r4 = firmware table.
 * A v1 hook ('TVA1') never calls this module; this hook never calls a v1 module.
 * IMAGE_BIN = the sdstream RAM build, embedded unchanged. TVA_NOCOPY (RAM bench harness): no embedded image. */
#include "sdl_ver.h"
#ifndef HDR_BUILD_ID
#define HDR_BUILD_ID 0
#endif
#ifndef HDR_FWT_VER
#define HDR_FWT_VER 0
#endif
    .section .text.start,"ax"
    .global _tva_hdr
_tva_hdr:
    .long 0x54564132            /* 'TVA2' */
    .long _tva_poll
    .long _tva_task
    .long _tva_image
    .long _tva_image_end - _tva_image
    .long HDR_BUILD_ID          /* +0x14: build id (sha8 of the sdstream image), for the Pi updater */
    .long HDR_FWT_VER           /* +0x18: firmware-table version this module needs (fwtab.h FWT_VERSION); updater refuses if the unit's table is older */
    .long (SDL_STREAM_FMT_MIN << 8) | SDL_STREAM_FMT   /* +0x1C: stream formats the decoder plays (sdl_ver.h): oldest << 8 | newest */
    .section .rodata.image,"a"
    .balign 32
    .global _tva_image
    .global _tva_image_end
_tva_image:
#ifndef TVA_NOCOPY
    .incbin IMAGE_BIN
    .balign 4
#endif
_tva_image_end:
