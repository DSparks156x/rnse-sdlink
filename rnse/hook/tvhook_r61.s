; r61 ACON 100 ms hook (base 0x806AA780; free OSM zone audited by rebuild.py).
; Literal 0x80590798 (one use, at 0x80590536: jsr PWM main, inside tick dispatcher 805904E4, ACON task, driven by the 100 ms cyclic
; handler 805909A8) is repointed here. Calls PWM main exactly as before, then, only if the SDST module is really in flash
; (word at 0x80C00080 = 'TVA1'), calls its tva_poll. The stock caller re-masks FPSCR after the call and keeps nothing in scratch regs.
tvhook:
    sts.l pr,@-r15
    mov.l lit_pwm,r1
    jsr @r1
    nop
    mov.l lit_hdr,r1
    mov.l @r1,r2
    mov.l lit_magic,r3
    cmp/eq r3,r2
    bf out
    mov.l @(4,r1),r1
    .word 0x026A         ; sts fpscr,r2
    mov.l lit_fpm,r3
    and r3,r2
    jsr @r1
    .word 0x426A         ; lds r2,fpscr  (delay slot; SZ/PR clear, as the stock tick dispatcher does)
out:
    lds.l @r15+,pr
    rts
    nop
    .align 4
lit_pwm:   .long 0x8060125A
lit_hdr:   .long 0x80C00080
lit_magic: .long 0x54564131
lit_fpm:   .long 0xFFE7FFFF
