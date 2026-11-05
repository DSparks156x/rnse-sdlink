/* crt0 for rnse_proto RAM programs: zero .bss, call main(), then ext_tsk (task 159 never returns). */
    .section .text.start,"ax"
    .global _start
_start:
    mov.l   L_bss_s,r1
    mov.l   L_bss_e,r2
    mov     #0,r0
1:  cmp/hs  r2,r1
    bt      2f
    mov.l   r0,@r1
    bra     1b
    add     #4,r1
2:  mov.l   L_main,r0
    jsr     @r0
    nop
    mov.l   L_ext,r0
    jmp     @r0
    nop
    .align 2
L_bss_s: .long __bss_start
L_bss_e: .long __bss_end
L_main:  .long _main
L_ext:   .long 0x804D027C
