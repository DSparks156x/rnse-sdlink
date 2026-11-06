/* sdboot: resident SD bootstrap for RAM programs (SDBOOT.txt). Uploaded once over CAN, stays at 8F71F000 until power-off.
 * Runs as task 159: reads a header sector + image from the Teensy's superframe window (LBA 0x10000 + 0xBE * 256, or the LBA
 * the host poked into BL_ARG), checks the live firmware bytes the host listed, checks both CRCs, copies CTL, clears the status
 * block and jumps to the image exactly as tramp.S does. Any failure: result code in RES, ext_tsk, RAM only, never flash.
 * All state is on the stack: no .data, no .bss (the image is one flat blob with a stamp in its last 16 bytes of code area). */
#include "rnse.h"

#define BL_BASE   0x8F71F000u
#define BL_ARG    0x8F720FD0u          /* host-poked: LBA of the header sector; 0 = default */
#define BL_RES    0x8F720FE0u          /* 8 words, read by the host */
#define HDR_BUF   0x8F71FE00u          /* 512 B DMA buffer, 32-B aligned */
#define DEF_LBA   (0x10000u + 0xBEu * 256u)
#define REGION_LO 0x8F700000u
#define REGION_HI 0x8FA00000u
#define KEEP_LO   BL_BASE              /* never overwrite the bootstrap, CTL, status */
#define KEEP_HI   0x8F721000u
#define TCB159_ENTRY (0x8E8D7B6Cu + 0x20u)
#define DOOM_ENTRY   0x80B06000u
#define FW_SD_RDCORE 0x8067D108u       /* same call sd_raw_read makes (806536C8), with our timeout, as sdstream.c */
#define SD_BUSY0  (*(volatile u32 *)0x8EAAC970u)
#define SD_VARIANT0 0x8EAD1254u
#define READ_TO_MS 200

enum { R_MAGIC, R_ID, R_CODE, R_MS, R_TRIES, R_CRC, R_NSECT, R_ENTRY };
enum { C_RUN = 1, C_HDR, C_RANGE, C_READ, C_CRC, C_FW, C_LOADED, C_JUMP };
#define RES(i) R32(BL_RES + 4 * (i))
#define W(i)   R32(HDR_BUF + 4 * (i))    /* read through P1 after ocbi */
#define TICK   R32(TICK_MS)

static u32 crc(const u8 *p, u32 n)
{
    u32 c = ~0u, k;
    while (n--) {
        c ^= *p++;
        for (k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1)));
    }
    return ~c;
}

static void oc_inv(u32 a, u32 n)
{
    u32 e = a + n;
    for (a &= ~31u; a < e; a += 32) __asm__ volatile("ocbi @%0" : : "r"(a) : "memory");
}

static int rd(u32 lba, u32 buf, u32 n)
{
    volatile int cnt = (int)n;
    int r, t;
    for (t = 0; t < 5; t++) {
        R32(SD_VARIANT0) = 0x00000100u;   /* /4: the driver leaves a raised variant behind after any error */
        cnt = (int)n;
        SD_BUSY0 = 1;
        r = ((int (*)(u32, void *, volatile int *, u32, int))FW_SD_RDCORE)(0, (void *)buf, &cnt, lba, READ_TO_MS);
        SD_BUSY0 = 0;
        RES(R_TRIES)++;
        if ((short)r == 0 && cnt == (int)n) { oc_inv(buf, n * 512); return 0; }
        dly_tsk(2);
    }
    return -1;
}

static void fail(u32 code)
{
    RES(R_CODE) = code;
    ((void (*)(void))FW_EXT_TSK)();
}

__attribute__((section(".text.start"))) void boot_entry(void)
{
    u32 t0, lba, base, len, entry, win, ncrc, nsect, nctl, flags, nchk, i;
    if (R32(TCB159_ENTRY) == BL_BASE) R32(TCB159_ENTRY) = DOOM_ENTRY;   /* the host restores it too; this covers a host that died */
    t0 = TICK;
    lba = R32(BL_ARG);
    R32(BL_ARG) = 0;                    /* a poked LBA is good for one start */
    if (!lba) lba = DEF_LBA;
    RES(R_MAGIC) = 0; RES(R_ID) = 0; RES(R_TRIES) = 0; RES(R_MS) = 0; RES(R_CRC) = 0; RES(R_NSECT) = 0; RES(R_ENTRY) = 0;
    RES(R_CODE) = C_RUN;
    RES(R_MAGIC) = 0x52534252u;         /* 'RSBR' */
    if (rd(lba, HDR_BUF, 1)) fail(C_READ);
    if (W(0) != 0x5253424Cu || crc((const u8 *)HDR_BUF, 508) != W(127)) fail(C_HDR);
    RES(R_ID) = W(1);
    base = W(2); len = W(3); entry = W(4); win = W(5); ncrc = W(6); nsect = W(7); nctl = W(8); flags = W(9); nchk = W(10);
    RES(R_NSECT) = nsect;
    RES(R_ENTRY) = entry;
    if ((base & 31) || !len || len > nsect * 512 || nsect > 255 || nsect * 512 > win || (win & 31) ||
        base < REGION_LO || base + win > REGION_HI || base >= base + win ||
        (base < KEEP_HI && base + win > KEEP_LO) || entry < base || entry >= base + len || nctl > 64 || nchk > 15)
        fail(C_RANGE);
    for (i = 0; i < nchk; i++) {        /* live firmware bytes, the same gate the host used to read over CAN */
        const u8 *h = (const u8 *)(HDR_BUF + 320 + i * 12 + 4), *f = (const u8 *)R32(HDR_BUF + 320 + i * 12);
        u32 k;
        for (k = 0; k < 8; k++) if (h[k] != f[k]) { RES(R_CRC) = (u32)f; fail(C_FW); }
    }
    if (rd(lba + 1, base, nsect)) fail(C_READ);
    RES(R_CRC) = crc((const u8 *)base, len);
    RES(R_MS) = TICK - t0;
    if (RES(R_CRC) != ncrc) fail(C_CRC);
    if (!(flags & 1)) fail(C_LOADED);   /* load only: the program stays resident, the host starts it later */
    for (i = 0; i < nctl; i++) CTL_W(i) = W(16 + i);
    ST(ST_MAGIC) = 0;
    R32(STAT_BASE + 28) = 0;
    RES(R_CODE) = C_JUMP;
    for (i = base & ~31u; i < base + win; i += 32)
        __asm__ volatile("mov %0,r0\n\t.word 0x00E3" : : "r"(i) : "r0", "memory");   /* icbi @r0, SH-4A, as tramp.S / sdmem_target.c */
    __asm__ volatile("nop\n\tnop" : : : "memory");
    ((void (*)(void))entry)();
    ((void (*)(void))FW_EXT_TSK)();
}
