/* rnv2tab.c -- tables shared by the rnvo_/rnvp_ decoder variants (built once by rnv_tables_init) and the black
 * fill. Canonical Huffman construction is the reference's (video/dec/rnv2dec.c huff_build), but the fast-table
 * entry carries the pre-split coefficient fields instead of the symbol number (no division in the decoder). */
#include "rnv2sh.h"
#include "rnv2_int.h"
#include "rnv2_tables.h"

rnv_huff rnv_hc, rnv_hbl, rnv_hbc;
rnv_u8 rnv_clamp[1024];
rnv_u8 rnv_clz8[256];

static void huff_build(rnv_huff *h, const rnv_u8 *len, int n, int coef)
{
    int l, i, k = 0, code = 0, cnt[18], m;
    rnv_u16 sym[RNV_NSYM];
    for (i = 0; i < (int)(sizeof h->l1 / sizeof h->l1[0]); i++) h->l1[i] = 0;
    for (i = 0; i < 18; i++) { cnt[i] = 0; h->first[i] = 0; h->cnt[i] = 0; h->off[i] = 0; }
    for (i = 0; i < n; i++) cnt[len[i]]++;
    for (l = 1; l <= 16; l++) {
        h->off[l] = k;
        for (i = 0; i < n; i++) if (len[i] == l) sym[k++] = (rnv_u16)i;
    }
    cnt[0] = 0;
    for (l = 1; l <= 16; l++) { code = (code + cnt[l - 1]) << 1; if (l == 1) code = 0; h->first[l] = code; h->cnt[l] = cnt[l]; }
    for (i = 0; i < k; i++) {
        int s = sym[i], info;
        if (coef) {
            int last = s / (RNV_RUNMAX * RNV_LVMAX), run = (s / RNV_LVMAX) % RNV_RUNMAX, lvm1 = s % RNV_LVMAX;
            info = (last << 8) | (run << 4) | lvm1;
        } else info = s;
        h->info[i] = (rnv_u16)info;
    }
    for (l = 1; l <= RNV_L1BITS; l++)
        for (i = 0; i < cnt[l]; i++) {
            int c = (h->first[l] + i) << (RNV_L1BITS - l);
            for (m = 0; m < (1 << (RNV_L1BITS - l)); m++) h->l1[c + m] = (rnv_u16)((h->info[h->off[l] + i] << 5) | l);
        }
}

void rnv_tables_init(void)
{
    int i;
    for (i = 0; i < 1024; i++) rnv_clamp[i] = (rnv_u8)(i - 384 < 0 ? 0 : i - 384 > 255 ? 255 : i - 384);
    for (i = 0; i < 256; i++) { int z = 0; while (z < 8 && !(i & (0x80 >> z))) z++; rnv_clz8[i] = (rnv_u8)z; }
    huff_build(&rnv_hc, COEF_LEN, RNV_NSYM, 1);
    huff_build(&rnv_hbl, CBPL_LEN, 16, 0);
    huff_build(&rnv_hbc, CBPC_LEN, 16, 0);
}

const int *rnv_qstep(void) { return QSTEP2; }

static void black(rnv_u8 *b)
{
    rnv_u32 *p = (rnv_u32 *)b, *e = (rnv_u32 *)(b + RNV_FRAME_BYTES);
    const rnv_u32 v = RNV_PACK(0, 128, 0, 128);
    while (p < e) { STW(p, v); STW(p + 1, v); STW(p + 2, v); STW(p + 3, v); p += 4; }
}

void rnv_state_reset(rnv_state *s)
{
    int i;
    for (i = 0; i < s->nbuf; i++) black(s->buf[i]);
    for (i = 0; i < RNV_NB; i++) s->age[i] = 0;     /* reference starts with prevmode = INTRA: copy on first skip */
    s->cur = 0; s->last = s->nbuf - 1;
    s->frames = 0; s->last_err = 0; s->err_blk = 0;
}

/* key frame during display: the shown buffer stays up until the new frame flips in, so no black flash */
void rnv_state_reset_keep(rnv_state *s, const rnv_u8 *shown)
{
    int i, k = 0;
    for (i = 0; i < s->nbuf; i++) if (s->buf[i] == shown) k = i; else black(s->buf[i]);
    for (i = 0; i < RNV_NB; i++) s->age[i] = 0;
    s->cur = (k + 1) % s->nbuf;
    s->last = s->nbuf == 3 ? (k + 2) % 3 : s->cur;
    s->frames = 0; s->last_err = 0; s->err_blk = 0;
}

void rnv_state_init(rnv_state *s, rnv_u8 *b0, rnv_u8 *b1, rnv_u8 *b2, int nbuf)
{
    s->buf[0] = b0; s->buf[1] = b1; s->buf[2] = b2;
    s->nbuf = nbuf == 3 ? 3 : 2;
    rnv_state_reset(s);
}
