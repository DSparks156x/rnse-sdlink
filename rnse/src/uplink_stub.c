/* uplink_stub.c -- reference/sim implementation of the uplink transport interface used by capenc (capenc.h):
 * fragments each message into RSUP sectors exactly as link/SPEC.md 4.1 and appends them to a RAM ring
 * (= the sectors the real transport would write with CMD24/25 at LBA 0x28000 + position for channel 2).
 * The real SD write path (other agent) replaces this file; only uplink_put/uplink_space are the contract.
 * Sector: +0 'RSUP' +4 u8 channel +5 u8 flags (bit0 first, bit1 last) +6 u16 payload len (<= 496) +8 u32 message
 * seq (per channel) +12 u32 CRC-32 (zlib) of the payload bytes +16 payload, zero padded to 512.
 * Test knobs: a per-frame payload budget (uplink_stub_tick() starts a frame) and "lose every Nth message"
 * (accepted, seq consumed, never written: what an optimistic CRC token + bad block looks like to the Pi);
 * drop_every = N | limit << 16: lose message N, 2N, ... up to message number limit (0 = forever). */
#include "capenc.h"
#include "uplink_stub.h"

#define RSUP_MAGIC 0x52535550u
#define RSUP_PAY   496u

static struct { u8 *ring; u32 nsect, wpos, space, used, drop_every, nput, lost, refuse, nref, seq[8]; } U;

static inline void p16(u8 *p, u32 v) { p[0] = (u8)(v >> 8); p[1] = (u8)v; }
static inline void p32(u8 *p, u32 v) { p[0] = (u8)(v >> 24); p[1] = (u8)(v >> 16); p[2] = (u8)(v >> 8); p[3] = (u8)v; }

void uplink_stub_init(u8 *ring, u32 nsect, u32 space_per_frame, u32 drop_every)
{
    u32 i;
    U.ring = ring; U.nsect = nsect; U.wpos = 0; U.space = space_per_frame; U.used = 0;
    U.drop_every = drop_every; U.nput = 0; U.lost = 0; U.refuse = 0; U.nref = 0;
    for (i = 0; i < 8; i++) U.seq[i] = 0;
}

void uplink_stub_tick(u32 space_per_frame) { U.used = 0; U.space = space_per_frame; }
u32 uplink_stub_sectors(void) { return U.wpos; }
u32 uplink_stub_lost(void) { return U.lost; }
void uplink_stub_refuse(u32 every) { U.refuse = every; }   /* refuse every Nth put (transport busy) */

u32 uplink_space(u32 chan)
{
    u32 s = U.space > U.used ? U.space - U.used : 0, fr = (U.nsect - U.wpos) * RSUP_PAY;
    (void)chan;
    return s < fr ? s : fr;
}

int uplink_put(u32 chan, const void *msg, u32 len)
{
    const u8 *m = (const u8 *)msg;
    u32 off, seq;
    if (!len || chan > 7 || len > uplink_space(chan)) return -1;
    if (U.refuse && ++U.nref % U.refuse == 0) return -1;
    U.used += len;
    seq = U.seq[chan]++;
    U.nput++;
    if ((U.drop_every & 0xFFFF) && U.nput % (U.drop_every & 0xFFFF) == 0 &&
        (!(U.drop_every >> 16) || U.nput <= (U.drop_every >> 16))) { U.lost++; return 0; }
    for (off = 0; off < len; off += RSUP_PAY) {
        u8 *s = U.ring + 512 * U.wpos++;
        u32 n = len - off < RSUP_PAY ? len - off : RSUP_PAY, i;
        p32(s, RSUP_MAGIC);
        s[4] = (u8)chan;
        s[5] = (u8)((off == 0 ? 1 : 0) | (off + n == len ? 2 : 0));
        p16(s + 6, n);
        p32(s + 8, seq);
        p32(s + 12, cap_crc32(0, m + off, n));
        for (i = 0; i < n; i++) s[16 + i] = m[off + i];
        for (; i < RSUP_PAY; i++) s[16 + i] = 0;
    }
    return 0;
}
