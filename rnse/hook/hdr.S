/* r61 SDST payload head (phys 00C00080 / P1 80C00080). The OSM cave checks the magic, then calls +4 from the ACON task.
 * IMAGE_BIN = the sdstream RAM build, embedded unchanged. TVA_NOCOPY (RAM bench harness): no embedded image. */
    .section .text.start,"ax"
    .global _tva_hdr
_tva_hdr:
    .long 0x54564131            /* 'TVA1' */
    .long _tva_poll
    .long _tva_task
    .long _tva_image
    .long _tva_image_end - _tva_image
    .long 0
    .long 0
    .long 0
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
