/* sdprobe: READ-ONLY snapshot of the SD host (TMIO/SDHI @FF90A000, 16-bit registers), card-detect GPIO,
 * SD power, SD DMA channel and SD interrupt registers. No register is written. The DATA port (+0x30) is never read
 * (a read would pop the FIFO). Every access uses the width the firmware uses (refind_out.txt / README).
 * CTL p0 = group mask: bit0 SD regs the driver uses, bit1 extra SD regs the driver never touches (+2C +2E +36 +E2;
 *          opt-in), bit2 CD GPIO + power, bit3 SD DMA channel (opt-in: base = *(u32*)0x8EAAC93C, which must lie in
 *          FF800000..FF80FFFF; reads +0 +4 +8 (SAR/DAR/TCR, inferred) and +0x34 (read by the driver at 80688B70)) +
 *          DMA request select FF903024, bit4 SD IRQ regs FF804254/FF80410C/FF804108 (32-bit, as the driver).
 *          p1 = number of extra CD GPIO samples (0..32, 10 ms apart; stored at words 32..63).
 * STAT: word 8 = entry count, 9 = group mask, 10/11 = samples asked/done, 12 = DMA base read; entries from word 64:
 * [addr | 1 if 16-bit] [value]. */
#include "rnse.h"

#define PROG_SDPROBE 2
#define SD_BASE 0xFF90A000u

static const u8 sd_used[] = { 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12, 0x14, 0x16, 0x18, 0x1A,
                              0x1C, 0x1E, 0x20, 0x22, 0x24, 0x26, 0x28, 0xB2, 0xB8, 0xD8, 0xE0, 0xF0, 0xFC };
static const u8 sd_extra[] = { 0x2C, 0x2E, 0x36, 0xE2 };
static const u32 gpio32[] = { 0xFF93E00C, 0xFF93F008 };
static const u8 dma_off[] = { 0x00, 0x04, 0x08, 0x34 };
static const u32 irq32[] = { 0xFF804254, 0xFF80410C, 0xFF804108 };

static u32 n;

static void put(u32 a, u32 w16)
{
    u32 v = w16 ? (u32)R16(a) : R32(a);
    ST(64 + 2 * n) = a | (w16 ? 1u : 0u);
    ST(64 + 2 * n + 1) = v;
    n++;
    ST(8) = n;
}

int main(void)
{
    u32 m, i, ns;
    st_begin(PROG_SDPROBE);
    if (!ctl_ok()) { st_end(PROG_SDPROBE, 1); return 0; }
    m = CTL_P(0); ns = CTL_P(1);
    if (ns > 32) ns = 32;
    ST(9) = m;
    n = 0;
    if (m & 1) for (i = 0; i < sizeof sd_used; i++) put(SD_BASE + sd_used[i], 1);
    if (m & 2) for (i = 0; i < sizeof sd_extra; i++) put(SD_BASE + sd_extra[i], 1);
    if (m & 4) for (i = 0; i < 2; i++) put(gpio32[i], 0);
    if (m & 8) {
        u32 p = R32(0x8EAAC93C);
        ST(12) = p;
        if (p >= 0xFF800000u && p < 0xFF810000u && !(p & 3))
            for (i = 0; i < sizeof dma_off; i++) put(p + dma_off[i], 0);
        put(0xFF903024, 0);
    }
    if (m & 16) for (i = 0; i < sizeof irq32 / 4; i++) put(irq32[i], 0);
    ST(10) = ns;
    for (i = 0; i < ns; i++) {               /* CD GPIO over time (e.g. while a card is inserted / removed) */
        ST(32 + i) = R32(0xFF93E00C);
        ST(11) = i + 1;
        ST(ST_HB) = tick();
        dly_tsk(10);
    }
    st_end(PROG_SDPROBE, 0);
    return 0;
}
