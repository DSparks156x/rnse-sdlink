/* tramp.S: position-independent 48-byte launcher, written to a FRESH slot for every run (the host patches the
 * three literal words). It invalidates the I-cache over the program image (ICBI per 32-byte line, SH-4A) so a
 * rebuilt image at the same fixed address can never execute stale lines, then jumps to the image entry.
 * Literal words at offset 0x20: image start, image end, entry. */
    .text
    .global _tramp
_tramp:
    mova    L_lit,r0
    mov.l   @r0,r1          /* start (32-byte aligned) */
    mov.l   @(4,r0),r2      /* end */
    mov.l   @(8,r0),r3      /* entry */
1:  .word   0x01E3          /* icbi @r1 */
    add     #32,r1
    cmp/hs  r2,r1
    bf      1b
    nop
    nop
    jmp     @r3
    nop
    .align 5
L_lit:
    .long   0
    .long   0
    .long   0
    .long   0
