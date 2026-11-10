#include "avsd.h"
#include "sdmem_target.h"
#ifndef FWTAB
#error "sdmem_target.c is table-only now (build -DFWTAB); r57 literal: FW_SD_WRITE 0x80653776"
#endif
#define FW_SD_WRITE FW(FWE_SD_WRITE)
static int target_read(void *ctx,u32 a,u8 *dst,u32 n) {
    const volatile u8 *p=(const volatile u8 *)P2(a);
    (void)ctx;while(n--) *dst++=*p++;
    return 0;
}
static int target_write(void *ctx,u32 a,const u8 *src,u32 n) {
    volatile u8 *p=(volatile u8 *)P2(a);
    u32 i,base=(a&0x1FFFFFFFu)|0x80000000u;
    (void)ctx;
    for(i=0;i<n;i++) p[i]=src[i];
    /* P2 stores + uncached read drain; discard stale P1 operand and instruction lines.
     * P1 is write-through on this target. Matches the launcher's SH-4A ICBI sequence. */
    wb_drain(P2(a+n-1));
    for(i=base&~31u;i<base+n;i+=32) {
        __asm__ volatile("ocbi @%0"::"r"(i):"memory");
        __asm__ volatile("mov %0,r0\n\t.word 0x00E3"::"r"(i):"r0","memory");
    }
    __asm__ volatile("nop\n\tnop":::"memory");
    return 0;
}
static int target_send(void *ctx,u32 lba,const u8 *sector) {
    wb_drain((u32)sector+508);
    return ((int (*)(int,int,const void *,int))FW_SD_WRITE)((int)(u32)ctx,(int)lba,sector,1);
}
int sdmem_target_init(sdmem *s,u32 slot,u32 arm,u32 lo,u32 hi) {
    sdmem_io io;
    if(slot>1 || (arm!=0 && arm!=0x5A)) return -1;
    if(arm && (hi<=lo || (lo>>26)!=((hi-1)>>26) || ((lo>>26)!=0x23 && (lo>>26)!=0x2B))) return -1;
    io.opaque=(void *)slot;io.read=target_read;io.write=target_write;io.send=target_send;
    io.write_lo=arm?(lo&0x1FFFFFFFu):0;io.write_hi=arm?(hi&0x1FFFFFFFu):0;
    io.protect_lo=0x0F4C0000u;io.protect_hi=0x0F9E0000u;
    sdmem_init(s,&io);return 0;
}
