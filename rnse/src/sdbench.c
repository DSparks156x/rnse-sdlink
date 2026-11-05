/* sdbench: timed raw SD reads through the firmware driver, int 80653598(u16 slot, s32 lba, void *buf, s32 nsect)
 * (task context; same driver lock as the file system, so it may interleave with AFFM safely). READ-ONLY on the card.
 * Buffer: SD_BUF (P1, 32-byte aligned -> the driver DMAs straight into it, up to 0x1000 sectors per inner call).
 * Data checks go through the P2 (uncached) alias.
 *
 * CTL: p0 slot (0/1), p1 start LBA, p2 sectors per pass (1..4096), p3 card-detect gate (0 none (the driver itself
 *      returns -1 with no card), 1 require GPIO slot bit = 1, 2 require it = 0), p4 number of chunk sizes (1..6),
 *      p5..p10 chunk sizes (sectors per call, 1..4096), p11 passes per chunk size (1..4).
 * Timing: TMU0 counts (12.4875 MHz) via now_counts(), plus the 1 ms tick.
 * STAT: words 8..27 header (see SB_*), results from word 64, 10 words per pass (see R_*). */
#include "rnse.h"

#define PROG_SDBENCH 3
#define SD_BASE   0xFF90A000u
#define CD_GPIO   0xFF93E00Cu
#define MARK      0xA5C3A5C3u
#define MAXPASS   24
#define RW        10

enum { SB_CD_RAW = 8, SB_CD_BIT, SB_CLK0, SB_INFO1_0, SB_INFO2_0, SB_DMDL, SB_NRES, SB_SUM_LAST, SB_S0_0, SB_S0_1,
       SB_S0_2, SB_S0_3, SB_S0_TAIL, SB_SUM_MISMATCH, SB_CLK1, SB_OPT1, SB_INFO1_1, SB_INFO2_1, SB_BUSY0, SB_BUSY1 };
enum { R_CHUNK, R_SECT_OK, R_CALLS, R_ERRS, R_COUNTS, R_MS, R_SUM, R_UNTOUCHED, R_FIRST_ERR, R_CLK_INFO2 };
enum { E_OK, E_NOCTL, E_PARAM, E_NOCARD, E_STOPPED, E_READ };

#define SD_READ(s, l, b, n) ((int (*)(int, int, void *, int))FW_SD_READ)(s, l, b, n)

static u32 sum_p2(u32 nsect, u32 *untouched)
{
    volatile u32 *p = (volatile u32 *)P2(SD_BUF);
    u32 s = 0, i, j, u = 0;
    for (i = 0; i < nsect; i++) {
        if (p[i * 128] == (MARK ^ i)) u++;
        for (j = 0; j < 128; j++) s += p[i * 128 + j] ^ j;
        if ((i & 127) == 127) dly_tsk(1);
    }
    *untouched = u;
    return s;
}

int main(void)
{
    u32 slot, lba, total, gate, nsz, passes, sz[6], i, j, np = 0, err = E_OK, cdbit, prev_sum = 0, have_prev = 0;
    volatile u32 *mk = (volatile u32 *)P2(SD_BUF);

    st_begin(PROG_SDBENCH);
    if (!ctl_ok()) { st_end(PROG_SDBENCH, E_NOCTL); return 0; }
    slot = CTL_P(0); lba = CTL_P(1); total = CTL_P(2); gate = CTL_P(3); nsz = CTL_P(4); passes = CTL_P(11);
    for (i = 0; i < 6; i++) sz[i] = CTL_P(5 + i);
    if (slot > 1 || (s32)lba < 0 || total < 1 || total > 4096 || gate > 2 || nsz < 1 || nsz > 6 || passes < 1 ||
        passes > 4) { st_end(PROG_SDBENCH, E_PARAM); return 0; }
    for (i = 0; i < nsz; i++) if (sz[i] < 1 || sz[i] > 4096) { st_end(PROG_SDBENCH, E_PARAM); return 0; }

    ST(SB_CD_RAW) = R32(CD_GPIO);                       /* same test as 80653560: slot1 bit 0x800, slot0 0x400 */
    cdbit = (ST(SB_CD_RAW) & (slot ? 0x800u : 0x400u)) ? 1 : 0;
    ST(SB_CD_BIT) = cdbit;
    ST(SB_CLK0) = R16(SD_BASE + 0x24); ST(SB_INFO1_0) = R16(SD_BASE + 0x1C); ST(SB_INFO2_0) = R16(SD_BASE + 0x1E);
    ST(SB_DMDL) = R32(DMDL_HANDLE);
    ST(SB_BUSY0) = R32(0x8EAAC970); ST(SB_BUSY1) = R32(0x8EAAC96C);
    if ((gate == 1 && !cdbit) || (gate == 2 && cdbit)) { st_end(PROG_SDBENCH, E_NOCARD); return 0; }

    ST(ST_STAGE) = 2;
    for (i = 0; i < nsz && err == E_OK; i++) {
        for (j = 0; j < passes && err == E_OK; j++) {
            u32 chunk = sz[i], done = 0, calls = 0, errs = 0, first_err = 0xFFFFFFFFu, c0, c1, t0, k, unt;
            volatile u32 *r = &ST(64 + RW * np);
            if (stop_req()) { err = E_STOPPED; break; }
            for (k = 0; k < total; k++) mk[k * 128] = MARK ^ k;    /* per-sector marker: detects "0 but no data" */
            t0 = tick();
            c0 = now_counts();
            while (done < total) {
                u32 n = total - done < chunk ? total - done : chunk;
                int ret = SD_READ((int)slot, (int)(lba + done), (void *)(SD_BUF + done * 512), (int)n);
                calls++;
                if (ret != 0) { errs++; if (first_err == 0xFFFFFFFFu) first_err = done; break; }
                done += n;
                ST(ST_HB) = tick();
            }
            c1 = now_counts();
            r[R_CHUNK] = chunk | (j << 16);
            r[R_SECT_OK] = done; r[R_CALLS] = calls; r[R_ERRS] = errs;
            r[R_COUNTS] = c1 - c0; r[R_MS] = tick() - t0; r[R_FIRST_ERR] = first_err;
            r[R_CLK_INFO2] = (u32)R16(SD_BASE + 0x24) | ((u32)R16(SD_BASE + 0x1E) << 16);
            r[R_SUM] = sum_p2(done, &unt);
            r[R_UNTOUCHED] = unt;
            if (done == total) {
                if (have_prev && r[R_SUM] != prev_sum) ST(SB_SUM_MISMATCH)++;
                prev_sum = r[R_SUM]; have_prev = 1;
            }
            np++;
            ST(SB_NRES) = np;
            if (errs) break;                        /* one failing pass is enough: no retry storm */
            dly_tsk(20);
        }
        if (np && ST(64 + RW * (np - 1) + R_ERRS)) { err = E_READ; break; }
    }
    ST(SB_SUM_LAST) = prev_sum;
    ST(SB_S0_0) = mk[0]; ST(SB_S0_1) = mk[1]; ST(SB_S0_2) = mk[2]; ST(SB_S0_3) = mk[3]; ST(SB_S0_TAIL) = mk[127];
    ST(SB_CLK1) = R16(SD_BASE + 0x24); ST(SB_OPT1) = R16(SD_BASE + 0x28);
    ST(SB_INFO1_1) = R16(SD_BASE + 0x1C); ST(SB_INFO2_1) = R16(SD_BASE + 0x1E);
    st_end(PROG_SDBENCH, err);
    return 0;
}
