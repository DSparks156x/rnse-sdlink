/* duprobe: READ-ONLY DU snapshot + DSSR (display status) transition log, for the YUY2 double-buffer design (DU_DESIGN.txt).
 * Reads (32-bit, all previously read live by vin888.py mm): DSYSR FFF80000, DSMR +04, DSSR +08, DIER +10, DPPR +18,
 * DEFR +20, plane P1 (TV plane, array 0) FFF80100..FFF80150. Never touches DSRCR (+0C, write-to-clear) or any write.
 * Then polls DSSR for p0 ms (10..200, default 60) without yielding and logs every change with a TMU timestamp:
 * tells whether bit 11 (VBK, the bit 805192xx polls) is visible to a polling task and at what period.
 * STAT: 8 = snapshot count, 9 = transitions, 10 = polls, 11 = poll ms; snapshot from word 16 (addr, value) x 27;
 *       transitions from word 128: (DSSR value, counts) x up to 64. */
#include "rnse.h"

#define PROG_DUPROBE 4
static const u8 glob[] = { 0x00, 0x04, 0x08, 0x10, 0x18, 0x20 };

int main(void)
{
    u32 i, n = 0, ms, t0, last, nt = 0, polls = 0;
    st_begin(PROG_DUPROBE);
    if (!ctl_ok()) { st_end(PROG_DUPROBE, 1); return 0; }
    ms = CTL_P(0);
    if (ms < 10 || ms > 200) ms = 60;
    ST(11) = ms;
    for (i = 0; i < sizeof glob; i++, n++) { ST(16 + 2 * n) = 0xFFF80000u + glob[i]; ST(17 + 2 * n) = R32(0xFFF80000u + glob[i]); }
    for (i = 0; i <= 0x50; i += 4, n++) { ST(16 + 2 * n) = 0xFFF80100u + i; ST(17 + 2 * n) = R32(0xFFF80100u + i); }
    ST(8) = n;
    last = R32(0xFFF80008u);
    ST(128) = last; ST(129) = now_counts(); nt = 1;
    t0 = tick();
    while (tick() - t0 < ms) {
        u32 v = R32(0xFFF80008u);
        polls++;
        if (v != last && nt < 64) { ST(128 + 2 * nt) = v; ST(129 + 2 * nt) = now_counts(); nt++; last = v; }
    }
    ST(9) = nt; ST(10) = polls;
    st_end(PROG_DUPROBE, 0);
    return 0;
}
