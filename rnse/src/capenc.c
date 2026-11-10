/* capenc.c -- dirty-tile + lossless run-op screen-capture encoder (format: capenc.h, design: CAPTURE.txt).
 * Portable C (SH-4A big-endian target, also built on x86 for the host fuzz test). Pixel bytes are copied in memory
 * order, all header fields are written big-endian byte by byte, so the output is identical on both. */
#include "capenc.h"

#define NONE 4u
#define FAIL 0xFFFFFFFFu
#define END_BYTES 28u
#define HDR_BYTES 32u
#define TREC_BYTES 24u

typedef unsigned long uptr;

static inline void p16(u8 *p, u32 v) { p[0] = (u8)(v >> 8); p[1] = (u8)v; }
static inline void p32(u8 *p, u32 v) { p[0] = (u8)(v >> 24); p[1] = (u8)(v >> 16); p[2] = (u8)(v >> 8); p[3] = (u8)v; }

/* make the live lines fresh before reading them through a cached alias: OCBP writes a dirty line back and
 * invalidates it, so the next load fetches what the GE/DU-side writer put in RAM. It never discards data. */
static void live_prep(const capenc *e, const u8 *p, u32 n)
{
#ifdef __sh__
    if (e->c.live_prep) {
        uptr a = (uptr)p & ~31ul, end = (uptr)p + n;
        for (; a < end; a += 32) __asm__ volatile("ocbp @%0" : : "r"(a) : "memory");
    }
#else
    (void)e; (void)p; (void)n;
#endif
}

/* ------------------------------------------------------------------ CRC-32 (zlib), slice-by-4 with byte loads */
static u32 crct[4][256];
static u32 crc_ready;

static void crc_init(void)
{
    u32 i, k, v;
    for (i = 0; i < 256; i++) {
        v = i;
        for (k = 0; k < 8; k++) v = (v >> 1) ^ (0xEDB88320u & (0u - (v & 1)));
        crct[0][i] = v;
    }
    for (i = 0; i < 256; i++)
        for (v = crct[0][i], k = 1; k < 4; k++) { v = crct[0][v & 0xFF] ^ (v >> 8); crct[k][i] = v; }
    crc_ready = 1;
}

u32 cap_crc32(u32 crc, const u8 *p, u32 n)
{
    u32 c;
    if (!crc_ready) crc_init();
    c = ~crc;
    while (n >= 4) {
        c ^= (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
        c = crct[3][c & 0xFF] ^ crct[2][(c >> 8) & 0xFF] ^ crct[1][(c >> 16) & 0xFF] ^ crct[0][c >> 24];
        p += 4; n -= 4;
    }
    while (n--) c = crct[0][(c ^ *p++) & 0xFF] ^ (c >> 8);
    return ~c;
}

/* ------------------------------------------------------------------ tile op encoder, one copy per pixel size
 * nw = new tile pixels (tw x th, contiguous), oldb = shadow at the tile origin (stride ss bytes) = the Pi's pixels.
 * Returns the op-stream length, or FAIL when it would not be shorter than RAW (cap = raw bytes). */
#define ENC_OPS(NAME, T)                                                                                        \
static u32 NAME(u8 *out, u32 cap, const T *nw, const u8 *oldb, u32 ss, u32 tw, u32 th, u32 intra)            \
{                                                                                                               \
    u32 pos = 0, x, y, kind = NONE, cnt = 0, lp = 0, b;                                                         \
    T p = 0;                                                                                                    \
    for (y = 0; y < th; y++) {                                                                                  \
        const T *o = (const T *)(oldb + y * ss), *n = nw + y * tw, *u = y ? n - tw : n;                         \
        for (x = 0; x < tw; x++) {                                                                              \
            T v = n[x];                                                                                         \
            u32 mo = !intra && o[x] == v, mr = p == v, mu = y && u[x] == v;                                     \
            if (pos + sizeof(T) + 2 > cap) return FAIL;                                                         \
            if ((kind == 0 && mr) || (kind == 1 && mu) || (kind == 3 && mo)) {                                  \
                if (++cnt == 64) { out[pos++] = (u8)(kind << 6 | 63); kind = NONE; }                            \
            } else {                                                                                            \
                if (kind != 2 || mo || mr || mu) {                                                              \
                    if (kind == 2) out[lp] = (u8)(0x80 | (cnt - 1));                                            \
                    else if (kind != NONE) out[pos++] = (u8)(kind << 6 | (cnt - 1));                            \
                    kind = mo ? 3 : mr ? 0 : mu ? 1 : 2;                                                        \
                    if (kind == 2) { lp = pos++; cnt = 0; } else cnt = 1;                                       \
                }                                                                                               \
                if (kind == 2) {                                                                                \
                    const u8 *s = (const u8 *)&n[x];                                                            \
                    for (b = 0; b < sizeof(T); b++) out[pos++] = s[b];                                          \
                    if (++cnt == 64) { out[lp] = 0xBF; kind = NONE; }                                           \
                }                                                                                               \
            }                                                                                                   \
            p = v;                                                                                              \
        }                                                                                                       \
    }                                                                                                           \
    if (kind == 2) out[lp] = (u8)(0x80 | (cnt - 1));                                                            \
    else if (kind != NONE) out[pos++] = (u8)(kind << 6 | (cnt - 1));                                            \
    return pos;                                                                                                 \
}                                                                                                               \
static u32 NAME##_solid(const T *nw, u32 n)                                                                     \
{                                                                                                               \
    T v = nw[0];                                                                                                \
    u32 i;                                                                                                      \
    for (i = 1; i < n; i++) if (nw[i] != v) return 0;                                                           \
    return 1;                                                                                                   \
}
ENC_OPS(ops8, u8)
ENC_OPS(ops16, u16)
ENC_OPS(ops32, u32)

/* ------------------------------------------------------------------ helpers */
static void geom(const capenc *e, u32 t, u32 *x0, u32 *y0, u32 *tw, u32 *th)
{
    u32 tx = t % e->ntx, ty = t / e->ntx;
    *x0 = tx * CAP_TW; *y0 = ty * CAP_TH;
    *tw = e->c.w - *x0 < CAP_TW ? e->c.w - *x0 : CAP_TW;
    *th = e->c.h - *y0 < CAP_TH ? e->c.h - *y0 : CAP_TH;
}

static void cpy(u8 *d, const u8 *s, u32 n)
{
    if (!(((uptr)d | (uptr)s | n) & 3)) {
        u32 *dw = (u32 *)d; const u32 *sw = (const u32 *)s;
        for (n >>= 2; n >= 4; n -= 4, dw += 4, sw += 4) { dw[0] = sw[0]; dw[1] = sw[1]; dw[2] = sw[2]; dw[3] = sw[3]; }
        while (n--) *dw++ = *sw++;
    } else while (n--) *d++ = *s++;
}

/* rows r = phase, phase+div, ... of tile t: does the live buffer differ from the shadow? */
static u32 tile_differs(const capenc *e, const u8 *live, u32 t, u32 phase, u32 div)
{
    u32 x0, y0, tw, th, r, nb, ls = e->c.live_stride, ss = e->sstride;
    const u8 *lp, *sp;
    geom(e, t, &x0, &y0, &tw, &th);
    nb = tw * e->c.bpp;
    lp = live + y0 * ls + x0 * e->c.bpp;
    sp = e->c.shadow + y0 * ss + x0 * e->c.bpp;
    for (r = phase; r < th; r += div) {
        const u8 *a = lp + r * ls, *b = sp + r * ss;
        live_prep(e, a, nb);
        if (!(((uptr)a | (uptr)b | nb) & 3)) {
            const u32 *aw = (const u32 *)a, *bw = (const u32 *)b;
            u32 k = nb >> 2;
            for (; k >= 4; k -= 4, aw += 4, bw += 4)
                if ((aw[0] ^ bw[0]) | (aw[1] ^ bw[1]) | (aw[2] ^ bw[2]) | (aw[3] ^ bw[3])) return 1;
            for (; k; k--) if (*aw++ != *bw++) return 1;
        } else {
            u32 k;
            for (k = 0; k < nb; k++) if (a[k] != b[k]) return 1;
        }
    }
    return 0;
}

/* encode tile t at out (room >= 5 + raw). Snapshots the live tile into e->tmp first, so what is sent is exactly
 * what capenc_commit copies to the shadow, even if the live buffer changes meanwhile. Returns bytes written. */
static u32 enc_tile(capenc *e, u8 *out, u32 t, const u8 *live, u32 intra)
{
    u32 x0, y0, tw, th, r, B = e->c.bpp, raw, n, len;
    u8 *tmp = (u8 *)e->tmp;
    const u8 *oldb;
    geom(e, t, &x0, &y0, &tw, &th);
    n = tw * th; raw = n * B;
    for (r = 0; r < th; r++) {
        const u8 *a = live + (y0 + r) * e->c.live_stride + x0 * B;
        live_prep(e, a, tw * B);
        cpy(tmp + r * tw * B, a, tw * B);
    }
    oldb = e->c.shadow + y0 * e->sstride + x0 * B;
    p16(out, t);
    if (B == 1 ? ops8_solid(tmp, n) : B == 2 ? ops16_solid((const u16 *)tmp, n) : ops32_solid((const u32 *)tmp, n)) {
        out[2] = CAP_M_SOLID;
        cpy(out + 3, tmp, B);
        e->st.solid++;
        return 3 + B;
    }
    len = B == 1 ? ops8(out + 5, raw, tmp, oldb, e->sstride, tw, th, intra)
        : B == 2 ? ops16(out + 5, raw, (const u16 *)tmp, oldb, e->sstride, tw, th, intra)
        : ops32(out + 5, raw, (const u32 *)tmp, oldb, e->sstride, tw, th, intra);
    if (len != FAIL && len + 2 < raw) {
        out[2] = CAP_M_OPS;
        p16(out + 3, len);
        e->st.ops++;
        return 5 + len;
    }
    out[2] = CAP_M_RAW;
    cpy(out + 3, tmp, raw);
    e->st.raw++;
    return 3 + raw;
}

static void commit_tile(capenc *e, u32 t)
{
    u32 x0, y0, tw, th, r, B = e->c.bpp;
    const u8 *tmp = (const u8 *)e->tmp;
    geom(e, t, &x0, &y0, &tw, &th);
    for (r = 0; r < th; r++) cpy(e->c.shadow + (y0 + r) * e->sstride + x0 * B, tmp + r * tw * B, tw * B);
}

static void rec_hdr(u8 *p, u32 type, u32 layer, u32 len) { p[0] = (u8)type; p[1] = (u8)layer; p16(p + 2, len); }

static void close_tiles(capenc *e)
{
    if (e->mtiles_rec) {
        u8 *p = e->msg + e->mtiles_rec - 1;
        p16(p + 2, (u32)(e->msg + e->mlen - p));
        p16(p + 20, e->mtiles);
        e->mtiles_rec = 0;
    }
}

/* hand the message to the transport. On refusal the tiles in it become "intra required" (their shadow already holds
 * the new pixels, the Pi does not), HDR/PAL become pending again. Returns accepted bytes. */
static u32 flush(capenc *e)
{
    u32 len, i;
    close_tiles(e);
    len = e->mlen;
    if (!len) return 0;
    if (uplink_put(CAP_CHAN, e->msg, len) < 0) {
        for (i = 0; i < e->mtiles; i++) e->tflag[e->mlist[i]] |= 1;
        if (e->mhas_hdr) e->hdr_pending = 1;
        if (e->mhas_pal) e->pal_pending = 1;
        e->st.putfail++;
        e->refused = 1;
        len = 0;
    } else {
        e->st.msgs++;
        e->st.bytes += len;
    }
    e->mlen = e->mtiles = e->mhas_hdr = e->mhas_pal = 0;
    return len;
}

/* ------------------------------------------------------------------ API */
int capenc_init(capenc *e, const capenc_cfg *cfg)
{
    u32 i, n;
    u8 *z = (u8 *)e;
    for (i = 0; i < sizeof *e; i++) z[i] = 0;
    e->c = *cfg;
    if (!(cfg->bpp == 1 || cfg->bpp == 2 || cfg->bpp == 4) || cfg->w < 1 || cfg->h < 1 || cfg->w > 4096 ||
        cfg->h > 4096 || cfg->live_stride < cfg->w * cfg->bpp || !cfg->shadow)
        return -1;
    if (!e->c.scan_div || (e->c.scan_div & (e->c.scan_div - 1)) || e->c.scan_div > CAP_TH) e->c.scan_div = 1;
    e->sstride = (cfg->w * cfg->bpp + 3) & ~3u;
    e->ntx = (cfg->w + CAP_TW - 1) / CAP_TW;
    e->nty = (cfg->h + CAP_TH - 1) / CAP_TH;
    e->ntiles = e->ntx * e->nty;
    if (e->ntiles > CAP_MAX_TILES) return -1;
    n = e->sstride * cfg->h;
    for (i = 0; i < n; i += 4) *(u32 *)(cfg->shadow + i) = 0;
    if (!crc_ready) crc_init();
    e->epoch = 0;
    capenc_refresh(e);
    return 0;
}

void capenc_refresh(capenc *e)
{
    u32 i;
    e->epoch++;
    for (i = 0; i < e->ntiles; i++) e->tflag[i] = 1;
    e->hdr_pending = 1;
    e->pal_pending = e->c.pal != 0;
    e->crc_ctr = 0;
    e->crc_dirty = 1;
}

u32 capenc_frame(capenc *e, const u8 *live, u32 budget)
{
    u32 cap = uplink_space(CAP_CHAN), acc = 0, i, t, dirty = 0, sent = 0, pending = 0, div = e->c.scan_div;
    u32 B = e->c.bpp, stopped = 0;
    if (budget < cap) cap = budget;
    e->refused = 0;
    e->frame_id++;
    e->st.calls++;
    /* palette change? (PAL8 only; 256-word compare) */
    if (e->c.pal) {
        for (i = 0; i < 256; i++) if (e->pal[i] != e->c.pal[i]) break;
        if (i < 256) { for (i = 0; i < 256; i++) e->pal[i] = e->c.pal[i]; e->pal_pending = 1; }
    }
    if (e->hdr_pending && HDR_BYTES + END_BYTES <= cap) {
        u8 *p = e->msg + e->mlen;
        rec_hdr(p, CAP_R_HDR, e->c.layer, HDR_BYTES);
        p32(p + 4, e->epoch); p16(p + 8, e->c.w); p16(p + 10, e->c.h);
        p[12] = (u8)e->c.fmt; p[13] = (u8)B; p[14] = CAP_TW; p[15] = CAP_TH;
        p16(p + 16, (u32)e->c.x); p16(p + 18, (u32)e->c.y); p[20] = (u8)e->c.z; p[21] = e->c.key_valid ? 1 : 0;
        p16(p + 22, 0); p32(p + 24, e->c.key); p32(p + 28, e->c.du_mr);
        e->mlen += HDR_BYTES; e->mhas_hdr = 1; e->hdr_pending = 0;
    }
    if (e->pal_pending && e->mlen + 12 + 1024 + END_BYTES <= cap) {
        u8 *p = e->msg + e->mlen;
        rec_hdr(p, CAP_R_PAL, e->c.layer, 12 + 1024);
        p32(p + 4, e->epoch); p16(p + 8, 256); p16(p + 10, 0);
        for (i = 0; i < 256; i++) p32(p + 12 + 4 * i, e->pal[i]);
        e->mlen += 12 + 1024; e->mhas_pal = 1; e->pal_pending = 0;
    }
    /* 1. detection */
    for (t = 0; t < e->ntiles; t++) {
        if ((e->tflag[t] & 1) || tile_differs(e, live, t, e->scan_phase, div)) { e->tflag[t] |= 2; dirty++; }
    }
    e->scan_phase = (e->scan_phase + 1) & (div - 1);
    /* 2. encode dirty tiles round-robin from the cursor until the budget is used */
    for (i = 0; i < e->ntiles && dirty; i++) {
        u32 hneed, worst, len, x0, y0, tw, th;
        t = e->cursor + i;
        if (t >= e->ntiles) t -= e->ntiles;
        if (!(e->tflag[t] & 3)) continue;
        geom(e, t, &x0, &y0, &tw, &th);
        worst = 5 + tw * th * B;
        hneed = e->mtiles_rec ? 0 : TREC_BYTES;
        if (e->mlen + hneed + worst + END_BYTES > CAP_MSG_MAX || e->mtiles + 1 > sizeof e->mlist / sizeof e->mlist[0]) {
            acc += flush(e);
            hneed = TREC_BYTES;
            if (e->refused) { e->cursor = t; stopped = 1; break; }
        }
        len = enc_tile(e, e->msg + e->mlen + hneed, t, live, e->tflag[t] & 1);
        if (acc + e->mlen + hneed + len + END_BYTES > cap) { e->cursor = t; stopped = 1; break; }
        if (hneed) {
            u8 *p = e->msg + e->mlen;
            rec_hdr(p, CAP_R_TILES, e->c.layer, 0);
            p32(p + 4, e->epoch); p32(p + 8, e->frame_id); p16(p + 12, e->c.w); p16(p + 14, e->c.h);
            p[16] = (u8)e->c.fmt; p[17] = (u8)B; p[18] = CAP_TW; p[19] = CAP_TH; p16(p + 20, 0); p16(p + 22, 0);
            e->mtiles_rec = e->mlen + 1;
        }
        e->mlen += hneed + len;
        e->mlist[e->mtiles++] = (u16)t;
        commit_tile(e, t);
        e->tflag[t] = 0;
        sent++;
    }
    if (!stopped) e->cursor = 0;
    close_tiles(e);
    for (t = 0; t < e->ntiles; t++) { if (e->tflag[t] & 3) pending++; e->tflag[t] &= 1; }
    /* 3. END record (pending counts what a refused message pushes back too) */
    if (acc + e->mlen + END_BYTES <= cap) {
        u8 *p;
        u32 fl = div > 1 ? 2 : 0, crc = 0;
        if (e->mlen + END_BYTES > CAP_MSG_MAX) acc += flush(e);
        if (sent) e->crc_dirty = 1;
        if (e->crc_ctr < 0xFFFFFFFFu) e->crc_ctr++;
        if (!pending && e->c.crc_every && e->crc_dirty && e->crc_ctr >= e->c.crc_every) {
            for (i = 0; i < e->c.h; i++) crc = cap_crc32(crc, e->c.shadow + i * e->sstride, e->c.w * B);
            fl |= 1; e->crc_ctr = 0; e->crc_dirty = 0; e->st.crcs++;
        }
        p = e->msg + e->mlen;
        rec_hdr(p, CAP_R_END, e->c.layer, END_BYTES);
        p32(p + 4, e->epoch); p32(p + 8, e->frame_id); p32(p + 12, pending); p32(p + 16, dirty);
        p32(p + 20, fl); p32(p + 24, crc);
        e->mlen += END_BYTES;
    }
    acc += flush(e);
    /* tiles of a message refused by the last flush were marked intra after the count: include them */
    if (e->refused) { pending = 0; for (t = 0; t < e->ntiles; t++) if (e->tflag[t] & 1) pending++; }
    e->st.dirty += dirty; e->st.sent += sent; e->st.pending = pending;
    e->last_dirty = dirty; e->last_sent = sent; e->last_pending = pending; e->last_bytes = acc;
    return acc;
}
