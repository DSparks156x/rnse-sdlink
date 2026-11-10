/* rnse_ver.h -- what avlink does with the RNS-E module's version announcement (docs/PROTOCOL.md 4.5). Header only, no globals:
 * avlink.c and test/test_rnsever.c include it.
 *
 * The unit's tvauto announces {module build id, stream formats its decoder plays (fmt_min..fmt), firmware table fw_id/version} in the
 * LBAs of its start-handshake reads; the Teensy decodes them and sends RNSE_INFO. avlink then:
 *   - picks the newest stream format both ends support (the unit's fmt_min..fmt and this chain's AVL_FMT_MIN..AVL_FMT_MAX);
 *   - logs "RNS-E module build XXXXXXXX fmt N table fw_id/ver", or "RNS-E module too old: ..." / "too new" when there is none;
 *   - answers HS_VERDICT (accept + format, or reject: the unit then stays stock) and stops sending superframes on a reject;
 *   - writes the result to --rnse-file (shell NAME=value lines, see rnse_ver_write) for pi/run and the encoder.
 * No announcement (a Teensy or module from before the handshake): today's format, AVL_FMT_DEFAULT, and "no version announced".
 *
 * AVL_FMT_* = the stream formats this avlink + encoder produce (rnse/src/sdl_ver.h SDL_STREAM_FMT rule: same numbers). */
#ifndef AVL_RNSE_VER_H
#define AVL_RNSE_VER_H
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "tlink.h"

#define AVL_FMT_MIN     1   /* oldest stream format this chain can send */
#define AVL_FMT_MAX     1   /* newest */
#define AVL_FMT_DEFAULT 1   /* assumed when the unit announces nothing: the format of every unit before the handshake */

typedef struct {
    int announced;          /* an RNSE_INFO arrived */
    tl_rnse_info info;
    int fmt;                /* the format avlink sends; 0 = none in common (not streaming) */
} rnse_ver_t;

/* the newest format in both ranges, 0 = none */
static int rnse_ver_choose(const tl_rnse_info *r, int pmin, int pmax)
{
    int lo = r->fmt_min > pmin ? r->fmt_min : pmin, hi = r->fmt < pmax ? r->fmt : pmax;
    if (!r->fmt || r->fmt_min > r->fmt) return 0;          /* a malformed announcement plays nothing we know */
    return lo <= hi ? hi : 0;
}

static void rnse_ver_set(rnse_ver_t *v, const tl_rnse_info *r, int pmin, int pmax)
{
    v->announced = 1; v->info = *r; v->fmt = rnse_ver_choose(r, pmin, pmax);
}

static void rnse_ver_none(rnse_ver_t *v)
{
    memset(v, 0, sizeof *v); v->fmt = AVL_FMT_DEFAULT;
}

/* "build F46C419D fmt 1 table 54B/1" (fmt shows the range when the unit plays more than one) */
static int rnse_ver_id(const rnse_ver_t *v, char *b, size_t n)
{
    const tl_rnse_info *r = &v->info;
    if (r->fmt_min != r->fmt)
        return snprintf(b, n, "build %08X fmt %u-%u table %X/%u", (unsigned)r->build_id, r->fmt_min, r->fmt, (unsigned)r->fw_id, r->table_ver);
    return snprintf(b, n, "build %08X fmt %u table %X/%u", (unsigned)r->build_id, r->fmt, (unsigned)r->fw_id, r->table_ver);
}

/* the log line for the current state (no newline) */
static int rnse_ver_line(const rnse_ver_t *v, int pmin, int pmax, char *b, size_t n)
{
    char id[96];
    if (!v->announced)
        return snprintf(b, n, "RNS-E module: no version announced (Teensy or module from before the handshake): assuming stream fmt %d", v->fmt);
    rnse_ver_id(v, id, sizeof id);
    if (v->fmt) return snprintf(b, n, "RNS-E module %s: streaming fmt %d", id, v->fmt);
    return snprintf(b, n, "RNS-E module too %s: %s; this chain sends fmt %d-%d: not streaming", v->info.fmt < pmin ? "old" : "new", id, pmin, pmax);
}

/* shell-sourceable state, written atomically (tmp + rename). 0 = written. */
static int rnse_ver_write(const char *path, const rnse_ver_t *v)
{
    char tmp[512];
    if (!path || snprintf(tmp, sizeof tmp, "%s.tmp", path) >= (int)sizeof tmp) return -1;
    FILE *f = fopen(tmp, "w");
    if (!f) return -1;
    const tl_rnse_info *r = &v->info;
    fprintf(f, "# avlink: the RNS-E module's version announcement (docs/PROTOCOL.md 4.5)\n");
    fprintf(f, "RNSE_ANNOUNCED=%d\n", v->announced);
    if (v->announced)
        fprintf(f, "RNSE_BUILD=%08X\nRNSE_FMT=%u\nRNSE_FMT_MIN=%u\nRNSE_FW_ID=%X\nRNSE_TABLE=%u\nRNSE_HS=%u\n",
                (unsigned)r->build_id, r->fmt, r->fmt_min, (unsigned)r->fw_id, r->table_ver, r->hs_ver);
    fprintf(f, "STREAM_FMT=%d\n", v->fmt);
    if (fclose(f)) return -1;
    return rename(tmp, path);
}

/* fill an HS_VERDICT for the current announcement */
static void rnse_ver_verdict(const rnse_ver_t *v, tl_hs_verdict *o)
{
    memset(o, 0, sizeof *o);
    o->build_id = v->info.build_id; o->check = v->info.check;
    o->verdict = v->fmt ? TL_HS_ACCEPT : TL_HS_REJECT;
    o->fmt = (uint8_t)v->fmt;
}
#endif
