// avstream.cpp -- see avstream.h. No hardware access.
#include "avstream.h"
#include "sdproto.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#if defined(__arm__) && !defined(AVS_HOST)
// Main loop only. Masks the read engine (AVS.get runs in pump, prio 64) but not the CMD top half (prio 0, 640 ns deadline per
// word at 25 MHz) or bottom half (prio 16). A full cpsid here lost CMD words: bad-CRC host command, 50 s host stall (bench 2026-11-05).
#define AVS_LOCK()   __asm__ volatile("msr basepri, %0" :: "r"(64u) : "memory")
#define AVS_UNLOCK() __asm__ volatile("msr basepri, %0" :: "r"(0u) : "memory")
// Pin commit only: prevent the prio-16 command bottom half changing the command generation between its
// check and the one-store pin update. No table scan, image copy or sector freeing runs inside this region.
// CMD top half (prio 0) remains enabled; restore any incoming mask rather than lowering it.
#define AVS_PIN_LOCK(v) __asm__ volatile("mrs %0, basepri\n\tmsr basepri_max, %1" : "=&r"(v) : "r"(16u) : "memory")
#define AVS_PIN_UNLOCK(v) __asm__ volatile("msr basepri, %0" :: "r"(v) : "memory")
#elif defined(AVS_TEST_INTERLEAVE)
// Native regression hook: let a simulated read IRQ land after victim selection, before the lock.
extern void avs_test_before_lock(void);
#define AVS_LOCK()   avs_test_before_lock()
#define AVS_UNLOCK() do {} while (0)
#define AVS_PIN_LOCK(v) do { (v) = 0; } while (0)
#define AVS_PIN_UNLOCK(v) do { (void)(v); } while (0)
#else
#define AVS_LOCK()   do {} while (0)
#define AVS_UNLOCK() do {} while (0)
#define AVS_PIN_LOCK(v) do { (v) = 0; } while (0)
#define AVS_PIN_UNLOCK(v) do { (void)(v); } while (0)
#endif

AvStream AVS;
#define HS_BARRIER() __asm__ volatile("" ::: "memory")   // handshake seqlock: keep the copies between the hs_gen loads/stores

static const uint8_t MAGIC[4] = {'T', 'L', 'N', 'K'};
static inline uint32_t le32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static inline uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline void wle32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static inline void wle16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void wbe32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; }
static inline void wbe16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static inline int seq_after(uint32_t a, uint32_t b) { return (int32_t)(a - b) > 0; }   // a newer than b

static uint8_t bitrev8(uint8_t b) {
  b = (uint8_t)(((b & 0xF0) >> 4) | ((b & 0x0F) << 4));
  b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2));
  return (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1));
}

// wire image (start word first) -> the 512 data bytes. fmt4: 4-bit image, else 1-bit.
void avs_unwire(const uint32_t *w, int fmt4, uint8_t *out) {
  for (int i = 0; i < 128; i++) {
    uint32_t v = w[1 + i];
    for (int j = 0; j < 4; j++) {
      uint8_t b = (uint8_t)(v >> (8 * j));
      out[4 * i + j] = fmt4 ? (uint8_t)((b >> 4) | (b << 4)) : bitrev8(b);
    }
  }
}

// ---------------------------------------------------------------- pool
void AvStream::init(AvsSector *const *b, const uint16_t *bn, int nb, now_fn nowf, tx_fn txf) {
  memset(slots, 0, sizeof slots);
  read_slot = -1;
  banks = b; bank_n = bn; nbanks = nb; now = nowf; tx = txf;
  pool_n = 0;
  for (int i = 0; i < nb; i++) pool_n += bn[i];
  if (pool_n > 65534) pool_n = 65534;
  free_list = (uint16_t *)malloc(sizeof(uint16_t) * (size_t)(pool_n ? pool_n : 1));
  free_n = 0;
  for (int i = pool_n - 1; i >= 0; i--) free_list[free_n++] = (uint16_t)i;
  cfg.hold_cut_ms = 30; cfg.hold_future_ms = 20; cfg.silent_ms = 100; cfg.stale_ms = 200; cfg.status_hz = 20; cfg.whole_sf = 0;
  memset(&st, 0, sizeof st);
  st.ver = 1; st.size = sizeof(AvsStatus); st.pool_sectors = (uint32_t)pool_n;
  have_begun = have_complete = have_served = 0; newest_begun = newest_complete = 0;
  ps = 0; mpos = 0; link_active = 0; rx_msgs = 0; last_rx_us = 0;
  wslot = -1; cur_sec = -1; cur_got = 0; cur_dst = nullptr;
  read_gen = 0; read_win = -1; my_gen = 0xFFFFFFFFu; holding = 0; sticky_win = -1; degraded = 0; syn_valid = 0;
  srv_head = srv_tail = 0; last_status_us = 0; last_sent_free = 0xFFFFFFFFu; last_sent_slots = 0xFFFFFFFFu; tx_seq = 0;
  card_state = 0; bus_width = 1; mode = 0; crc7_rejects = 0;
  hs_n = 0; hs_gen = 0; hs_count = 0; hs_t = 0; hs_ack_lba = 0; hs_have = 0; hs_verdict = 0; hs_sent_count = 0; hs_sent_link = 0;
  memset(hs_w, 0, sizeof hs_w); memset(hs_ann, 0, sizeof hs_ann);
  uint8_t z[512];
  memset(z, 0, sizeof z);
  zero_n4 = (uint16_t)wire4(z, 512, zero_w4);
  zero_n1 = (uint16_t)wire1(z, 512, zero_w1);
}

AvsSector *AvStream::sec_ptr(uint16_t i) const {
  for (int b = 0; b < nbanks; b++) {
    if (i < bank_n[b]) return &banks[b][i];
    i = (uint16_t)(i - bank_n[b]);
  }
  return nullptr;
}

int AvStream::alloc_sector(void) {
  if (free_n == 0) {
    // Credit overrun: only unpinned slots other than the writer can be sacrificed. Selection is advisory:
    // a read IRQ may pin that victim before free_slot atomically checks and unpublishes it.
    for (int tries = 0; tries < AVS_NSLOT && free_n == 0; tries++) {
      int victim = -1;
      for (int s = 0; s < AVS_NSLOT; s++)
        if (slots[s].used && s != wslot && s != read_slot &&
            (victim < 0 || seq_after(slots[victim].seq, slots[s].seq))) victim = s;
      if (victim < 0) return -1;
      free_slot(victim, 0, 1);
    }
  }
  if (free_n == 0) return -1;
  int r;
  AVS_LOCK();
  r = free_list[--free_n];
  AVS_UNLOCK();
  return r;
}

void AvStream::free_sector(uint16_t i) { free_list[free_n++] = i; }   // main loop owns pool allocation

// ---------------------------------------------------------------- slots
int AvStream::used_slots(void) const {
  int n = 0;
  for (int s = 0; s < AVS_NSLOT; s++) n += slots[s].used;
  return n;
}

int AvStream::free_slot(int s, int force, int evict) {
  AVS_LOCK();
  AvsSlot &S = slots[s];
  if (!S.used || (!force && s == read_slot)) { AVS_UNLOCK(); return 0; }
  if (evict && !S.served) st.evicted++;
  if (s == read_slot) read_slot = -1;              // only explicit FLUSH/reset can invalidate a retry pin
  S.used = 0;                                    // get can no longer acquire this slot
  AVS_UNLOCK();
  // Main owns the detached slot. A read IRQ cannot see it, so walking up to 256 indices needs no mask.
  for (int i = 0; i < S.total; i++)
    if (S.idx[i] != 0xFFFF) { free_sector(S.idx[i]); S.idx[i] = 0xFFFF; }
  return 1;
}

int AvStream::find_slot(uint32_t seq) const {
  for (int s = 0; s < AVS_NSLOT; s++) if (slots[s].used && slots[s].seq == seq) return s;
  return -1;
}

int AvStream::find_slot_idx(uint32_t win) const {
  for (int s = 0; s < AVS_NSLOT; s++) if (slots[s].used && (slots[s].seq & 0xFF) == win) return s;
  return -1;
}

// More than 128 superframes behind the newest: outside the window's +-128 seq range, so its window number now means a newer
// superframe. get() no longer answers from it and it cannot keep a pin: the reader must always make progress (review finding 1).
int AvStream::slot_stale(uint32_t seq) const { return have_begun && (int32_t)(newest_begun - seq) > 128; }

int AvStream::new_slot(uint32_t seq, uint16_t total) {
  int s;
  if (slot_stale(seq)) {                                           // seq only ever rises in one session: the sender restarted and DTR was not seen
    for (s = 0; s < AVS_NSLOT; s++) if (slots[s].used) free_slot(s, 1);
    AVS_LOCK();
    have_begun = have_complete = 0;
    syn_valid = 0;
    AVS_UNLOCK();
  }
  // An admitted resend/wrap must never replace the host's retry image. Fail admission instead. A stale alias is not a retry image.
  while ((s = find_slot_idx(seq & 0xFF)) >= 0) if (!free_slot(s, slot_stale(slots[s].seq))) return -1;
  for (s = 0; s < AVS_NSLOT && slots[s].used; s++) {}
  if (s == AVS_NSLOT) {                                            // table full: oldest served first, else oldest
    for (int tries = 0; tries < AVS_NSLOT; tries++) {
      int v = -1;
      for (int i = 0; i < AVS_NSLOT; i++)
        if (slots[i].used && i != read_slot && slots[i].served &&
            (v < 0 || seq_after(slots[v].seq, slots[i].seq))) v = i;
      if (v < 0)
        for (int i = 0; i < AVS_NSLOT; i++)
          if (slots[i].used && i != read_slot && (v < 0 || seq_after(slots[v].seq, slots[i].seq))) v = i;
      if (v < 0) return -1;
      if (free_slot(v, 0, 1)) { s = v; break; }
    }
    if (s == AVS_NSLOT) return -1;
  }
  uint32_t t = now();
  AvsSlot &S = slots[s];
  S.seq = seq; S.total = total; S.have = 0; S.ended = 0; S.served = 0;
  S.t_begin = t; S.t_end = t; S.t_read = t;
  memset(S.idx, 0xFF, sizeof S.idx);
  memset(S.fmt1, 0, sizeof S.fmt1);
  AVS_LOCK();                                  // initialize privately; publish only after all indices are ready
  S.used = 1;
  if (!have_begun || seq_after(seq, newest_begun)) newest_begun = seq;
  have_begun = 1;
  AVS_UNLOCK();
  return s;
}

void AvStream::reset_link(void) {
  for (int s = 0; s < AVS_NSLOT; s++) if (slots[s].used) free_slot(s, 1);
  AVS_LOCK();
  have_begun = have_complete = have_served = 0;
  syn_valid = 0;
  AVS_UNLOCK();
  link_active = 0;
  hs_verdict = 0;          // that Pi's verdict goes with it; the announcement stays and goes to the next sender
  hs_sent_link = 0;
  // The sender may have died inside a message. A parser left mid-WIRE would swallow the next sender's CONFIGs as sector
  // data, up to 100 s at the 200 ms resend. Hunt for the magic again and give back the sector being copied.
  if (ps == 4 && cur_sec >= 0) free_sector((uint16_t)cur_sec);
  ps = 0; mpos = 0; hpos = 0; mgot = 0; mlen = 0; ppos = 0; wpos = 0;
  wslot = -1; cur_sec = -1; cur_got = 0; cur_dst = nullptr; cur_drop = 0;
}

void AvStream::reclaim(uint32_t t) {
  for (int s = 0; s < AVS_NSLOT; s++) {
    AvsSlot &X = slots[s];
    if (!X.used) continue;
    if (slot_stale(X.seq)) { free_slot(s, 1, 1); continue; }   // includes an old pin: get() cannot answer from it any more
    if (X.served) {                                  // mode A: free once a newer superframe was served. The reader's command for
      int newer = 0;                                 // that newer window moved the pin first, and a retry only ever re-reads the pinned one
      for (int i = 0; i < AVS_NSLOT && !newer; i++)
        if (slots[i].used && slots[i].served && seq_after(slots[i].seq, X.seq)) newer = 1;
      if (newer) free_slot(s);
      continue;
    }
    uint32_t quiet = t - X.t_read;
    int complete = X.ended && X.have == X.total;
    if (complete) {                                  // never read: stale after stale_ms unless it is the newest
      if (have_complete && X.seq == newest_complete) continue;
      if (t - X.t_end > (uint32_t)cfg.stale_ms * 1000u && quiet > 50000u) free_slot(s, 0, 1);
    } else if (X.seq != newest_begun && t - X.t_begin > 1000000u && quiet > 50000u) {   // abandoned
      free_slot(s, 0, 1);
    }
  }
}

// ---------------------------------------------------------------- USB RX parser
// states: 0 hunt magic, 1 header, 2 small payload, 3 WIRE sub-header, 4 WIRE sector bytes, 5 skip
void AvStream::feed(const uint8_t *p, int n, console_fn console) {
  while (n > 0) {
    switch (ps) {
      case 0: {
        uint8_t b = *p++; n--;
        if (b == MAGIC[mpos]) {
          if (++mpos == 4) { memcpy(hdr, MAGIC, 4); hpos = 4; ps = 1; mpos = 0; }
        } else {
          if (console) for (int i = 0; i < mpos; i++) console(MAGIC[i]);
          mpos = 0;
          if (b == MAGIC[0]) mpos = 1;
          else if (console) console(b);
        }
        break;
      }
      case 1: {
        int k = 16 - hpos; if (k > n) k = n;
        memcpy(hdr + hpos, p, (size_t)k); hpos += k; p += k; n -= k;
        if (hpos == 16) on_header();
        break;
      }
      case 2: {
        uint32_t k = mlen - mgot; if (k > (uint32_t)n) k = (uint32_t)n;
        for (uint32_t i = 0; i < k; i++) if (ppos < (int)sizeof pbuf) pbuf[ppos++] = p[i];
        mgot += k; p += k; n -= (int)k;
        if (mgot == mlen) { on_small(); ps = 0; }
        break;
      }
      case 3: {
        int k = 12 - wpos; if (k > n) k = n;
        memcpy(wsub + wpos, p, (size_t)k); wpos += k; p += k; n -= k; mgot += (uint32_t)k;
        if (wpos < 12) break;
        wseq = le32(wsub); widx = le16(wsub + 4); wcount = le16(wsub + 6);
        uint32_t fmt = le32(wsub + 8);
        wfmt1 = fmt == 1; wbytes = wfmt1 ? 520 : 528;
        wslot = find_slot(wseq);
        if ((fmt != 1 && fmt != 4) || mlen != 12u + (uint32_t)wcount * (uint32_t)wbytes) {
          st.rx_errors++; ps = 5; break;
        }
        if (wslot >= 0 && (uint32_t)widx + wcount > slots[wslot].total) wslot = -1;
        if (wslot < 0) st.rx_errors++;              // no SF_BEGIN for it: sectors are counted, then dropped
        wdone = 0; cur_sec = -1; cur_got = 0;
        ps = wcount ? 4 : 0;
        break;
      }
      case 4: {
        if (cur_sec < 0 && !cur_got) {                // start of a sector
          cur_drop = 1;
          if (wslot >= 0 && !(wslot == read_slot && slots[wslot].idx[widx + wdone] != 0xFFFF)) {
            int a = alloc_sector();
            if (a >= 0) { cur_sec = a; cur_drop = 0; }
            else st.rx_errors++;                   // all storage is owned by the reader/writer: count and drop
          }
          cur_dst = cur_drop ? nullptr : (uint8_t *)sec_ptr((uint16_t)cur_sec)->w;
        }
        uint32_t k = (uint32_t)wbytes - cur_got; if (k > (uint32_t)n) k = (uint32_t)n;
        if (cur_dst) memcpy(cur_dst + cur_got, p, k);
        cur_got += k; p += k; n -= (int)k; mgot += k;
        if (cur_got == (uint32_t)wbytes) sector_done();
        break;
      }
      case 5: {
        uint32_t k = mlen - mgot; if (k > (uint32_t)n) k = (uint32_t)n;
        mgot += k; p += k; n -= (int)k;
        if (mgot == mlen) ps = 0;
        break;
      }
    }
  }
}

void AvStream::on_header(void) {
  mtype = le16(hdr + 4); mlen = le32(hdr + 8); mgot = 0; ppos = 0; wpos = 0;
  if (mlen > 12u + 64u * 528u) { st.rx_errors++; ps = 0; return; }   // not a message: resync on the next magic
  if (!link_active) { link_active = 1; last_status_us = now() - 1000000u; }   // first STATUS goes out at once
  last_rx_us = now();
  rx_msgs++;
  if (mtype == TL_WIRE) { ps = mlen >= 12 ? 3 : 5; if (mlen < 12) st.rx_errors++; return; }
  if (mlen <= sizeof pbuf) { ps = 2; if (mlen == 0) { on_small(); ps = 0; } return; }
  ps = 5;                                            // META / DOWNLINK / unknown large: not handled here
}

void AvStream::sector_done(void) {
  st.rx_sectors++;
  if (!cur_drop) {
    AvsSector *sp = sec_ptr((uint16_t)cur_sec);
    uint32_t w0 = sp->w[0];
    int ok = wfmt1 ? (w0 == 0x7FFFFFFFu && sp->w[129] >> 16 == 0xFFFFu) : (w0 == 0x0FFFFFFFu && sp->w[131] == 0xFFFFFFFFu);
    AVS_LOCK();
    if (!ok || !slots[wslot].used || slots[wslot].seq != wseq) { free_sector((uint16_t)cur_sec); if (!ok) st.rx_errors++; }
    else {
      AvsSlot &S = slots[wslot];
      uint16_t i = (uint16_t)(widx + wdone);
      // A read may have pinned this slot while USB copied the image. Existing retry sectors stay immutable;
      // missing sectors may still be published for cut-through reads.
      if (wslot == read_slot && S.idx[i] != 0xFFFF) free_sector((uint16_t)cur_sec);
      else {
        if (S.idx[i] != 0xFFFF) free_sector(S.idx[i]); else S.have++;
        S.idx[i] = (uint16_t)cur_sec;
        if (wfmt1) S.fmt1[i >> 5] |= 1u << (i & 31); else S.fmt1[i >> 5] &= ~(1u << (i & 31));
      }
    }
    AVS_UNLOCK();
  }
  cur_sec = -1; cur_got = 0; cur_dst = nullptr;
  if (++wdone == wcount) ps = 0;
}

void AvStream::on_small(void) {
  const uint8_t *q = pbuf;
  switch (mtype) {
    case TL_SF_BEGIN:
      if (mlen >= 8) {
        uint16_t total = le16(q + 4);
        if (total >= 1 && total <= 256) { if (new_slot(le32(q), total) < 0) st.rx_errors++; st.begun++; }
        else { st.rx_errors++; st.begun++; }
      }
      break;
    case TL_SF_END:
      if (mlen >= 4) {
        int s = find_slot(le32(q));
        if (s < 0) { st.rx_errors++; break; }
        AVS_LOCK();
        AvsSlot &S = slots[s];
        S.ended = 1; S.t_end = now();
        if (S.have == S.total && (!have_complete || seq_after(S.seq, newest_complete))) { newest_complete = S.seq; have_complete = 1; }
        AVS_UNLOCK();
      }
      break;
    case TL_CONFIG:
      if (mlen >= 9) {
        if (le16(q)) cfg.hold_cut_ms = le16(q);
        if (le16(q + 2)) cfg.hold_future_ms = le16(q + 2);
        if (le16(q + 4)) cfg.silent_ms = le16(q + 4);
        if (le16(q + 6)) cfg.stale_ms = le16(q + 6);
        if (q[8]) cfg.status_hz = q[8];
        if (mlen >= 10 && q[9]) cfg.whole_sf = q[9] == 2;   // 0 = leave as it is (every older sender), 1 = off, 2 = on
      }
      break;
    case TL_HS_VERDICT:     // AvsHsVerdict: kept only if it answers the newest announcement
      if (mlen >= 8 && hs_have) {
        uint16_t ann[AVS_HS_N];
        uint32_t g = hs_gen;
        HS_BARRIER();
        memcpy(ann, hs_ann, sizeof ann);
        HS_BARRIER();
        if ((g & 1) || g != hs_gen) break;           // the IRQ is mid-update: the Pi resends on the next INFO
        if (le32(q) == ((uint32_t)ann[1] << 16 | ann[2]) && le16(q + 4) == ann[7] && q[6] <= AVS_HS_REJECT)
          hs_verdict = (uint32_t)ann[7] << 16 | (uint32_t)q[7] << 8 | q[6];
      }
      break;
    case TL_FLUSH:
      if (mlen >= 4) {
        uint32_t f = le32(q);
        for (int s = 0; s < AVS_NSLOT; s++) if (slots[s].used && !seq_after(f, slots[s].seq)) free_slot(s, 1);
        AVS_LOCK();
        if (have_begun && !seq_after(f, newest_begun)) newest_begun = f - 1;
        if (have_complete && !seq_after(f, newest_complete)) newest_complete = f - 1;
        syn_valid = 0;
        AVS_UNLOCK();
      }
      break;
    default:                                         // META / DOWNLINK (uplink agent), unknown: ignored
      break;
  }
}

// ---------------------------------------------------------------- read side (IRQ)
void AvStream::read_begin(uint32_t lba) {
  uint32_t rel = lba - AVS_WIN_LBA;
  read_win = rel < AVS_WIN_SECT ? (int)(rel >> 8) : -1;
  read_gen++;
  if (lba - AVS_HS_LBA < AVS_HS_SECT) hs_read(lba);
}

// ---------------------------------------------------------------- version handshake (IRQ side, docs/PROTOCOL.md 4.5)
// Reads must come in order 0..7; other LBAs in between (FS, probe) do not disturb it, an out-of-order handshake read restarts it.
void AvStream::hs_read(uint32_t lba) {
  uint32_t rel = lba - AVS_HS_LBA, i = rel >> 16, v = rel & 0xFFFFu;
  if (i == 0) {
    hs_n = (v & 0xFF00u) == AVS_HS_W0 ? 1 : 0;
    hs_w[0] = (uint16_t)v;
    return;
  }
  if (i != hs_n) { hs_n = 0; return; }
  hs_w[i] = (uint16_t)v;
  hs_n++;
  if (hs_n < AVS_HS_N) return;
  hs_n = 0;
  uint32_t sum = 0;
  for (int k = 0; k < AVS_HS_N - 1; k++) sum += hs_w[k];
  if (((~sum) & 0xFFFFu) != hs_w[AVS_HS_N - 1]) return;
  hs_gen++;                                          // odd: main loop readers retry
  HS_BARRIER();
  memcpy(hs_ann, hs_w, sizeof hs_ann);
  hs_count++; hs_t = now ? now() : 0; hs_ack_lba = lba; hs_have = 1;
  HS_BARRIER();
  hs_gen++;
}

int AvStream::hs_patch(uint32_t lba, uint8_t *sec) {
  if (!hs_have || lba != hs_ack_lba || hs_n != 0) return 0;
  uint32_t v = hs_verdict;
  memset(sec, 0, 512);
  wbe32(sec, AVS_HS_ACK);
  for (int k = 0; k < AVS_HS_N; k++) wbe16(sec + 4 + 2 * k, hs_ann[k]);
  if ((v >> 16) == hs_ann[AVS_HS_N - 1]) { sec[20] = (uint8_t)v; sec[21] = (uint8_t)(v >> 8); }   // else pending, no format
  wbe32(sec + 24, hs_count);
  return 1;
}

int AvStream::hs_info(AvsRnseInfo *o) const {
  uint16_t a[AVS_HS_N];
  uint32_t g, c, t;
  do {
    g = hs_gen;
    HS_BARRIER();
    if (!hs_have) return 0;
    memcpy(a, hs_ann, sizeof a); c = hs_count; t = hs_t;
    HS_BARRIER();
  } while ((g & 1) || g != hs_gen);
  memset(o, 0, sizeof *o);
  o->ver = 1; o->size = sizeof *o;
  o->build_id = (uint32_t)a[1] << 16 | a[2];
  o->fmt = (uint8_t)a[3]; o->fmt_min = (uint8_t)(a[3] >> 8); o->hs_ver = (uint8_t)a[0];
  o->fw_id = (uint32_t)a[4] << 16 | a[5];
  o->table_ver = a[6]; o->check = a[7];
  o->count = c;
  o->age_ms = now ? (now() - t) / 1000u : 0;
  return 1;
}

// main loop: every new announcement goes to the Pi, and the newest one again to every new sender (link_active edge)
void AvStream::hs_poll(uint32_t t) {
  (void)t;
  AvsRnseInfo r;
  if (!hs_info(&r)) return;
  if (hs_sent_link && r.count == hs_sent_count) return;
  send(TL_RNSE_INFO, &r, sizeof r);
  hs_sent_count = r.count; hs_sent_link = 1;
}

void AvStream::synth(uint8_t kind, uint32_t seq, uint32_t arg, int bus4, uint32_t *out, uint16_t *n) {
  if (!(syn_valid && syn_kind == kind && syn_seq == seq && syn_arg == arg && syn_bus4 == (uint8_t)bus4)) {
    uint8_t s[512];
    memset(s, 0, sizeof s);
    wbe32(s, 0x52534156u); wbe16(s + 4, 1); wbe16(s + 6, 64);
    wbe32(s + 8, seq);
    wbe32(s + 12, (uint32_t)(SFF_SYNTH | kind));
    wbe16(s + 36, 1); wbe16(s + 38, 1); wbe16(s + 40, 1);   // audio_sec, video_sec, total_sec = 1
    if (kind == SFF_EXPIRED || kind == SFF_NOTREADY) wbe32(s + 64, arg);              // SPEC 3.2: bytes 64..67 = newest seq
    wbe32(s + 60, crc32z(s + 64, 448, crc32z(s, 60)));
    syn_n = (uint16_t)(bus4 ? wire4(s, 512, syn_w) : wire1(s, 512, syn_w));
    syn_kind = kind; syn_seq = seq; syn_arg = arg; syn_bus4 = (uint8_t)bus4; syn_valid = 1;
  }
  memcpy(out, syn_w, (size_t)syn_n * 4);
  *n = syn_n;
}

int AvStream::copy_image(const AvsSlot &S, int off, int bus4, uint32_t *out, uint16_t *n) {
  const AvsSector *sp = sec_ptr(S.idx[off]);
  int img4 = !((S.fmt1[off >> 5] >> (off & 31)) & 1);
  if (img4 == bus4) {
    int nw = img4 ? 132 : 130;
    memcpy(out, sp->w, (size_t)nw * 4);
    out[nw] = 0xFFFFFFFFu;                          // pad word: the engine releases DAT when it loads
    *n = (uint16_t)(nw + 1);
    return 0;
  }
  uint8_t b[512];                                    // width mismatch (host switched width): re-encode
  avs_unwire(sp->w, img4, b);
  *n = (uint16_t)(bus4 ? wire4(b, 512, out) : wire1(b, 512, out));
  st.converted++;
  return 1;
}

int AvStream::get(uint32_t lba, int bus4, uint32_t *out, uint16_t *nw) {
  uint32_t rel = lba - AVS_WIN_LBA;
  if (rel >= AVS_WIN_SECT) return AVS_NOTWIN;
  uint32_t g = read_gen;
  if (g != my_gen) { my_gen = g; holding = 0; sticky_win = -1; degraded = 0; }
  int win = (int)(rel >> 8), off = (int)(rel & 0xFF);
  if (win != read_win) {                          // speculative queue crossing a Mode-A command's window boundary
    memcpy(out, bus4 ? zero_w4 : zero_w1, (size_t)(bus4 ? zero_n4 : zero_n1) * 4);
    *nw = bus4 ? zero_n4 : zero_n1;
    return AVS_READY;
  }
  uint32_t t = now();
  if (sticky_win == win) { synth(sticky_kind, sticky_seq, sticky_arg, bus4, out, nw); return AVS_READY; }
  int s = find_slot_idx((uint32_t)win);
  if (s >= 0 && slot_stale(slots[s].seq)) s = -1;   // a stale alias never answers for the window and is never pinned again
  if (s >= 0) {
    AvsSlot &S = slots[s];
    uint32_t pin_mask;
    AVS_PIN_LOCK(pin_mask);
    int command_slot = read_gen == g && read_win == win;
    if (command_slot) read_slot = s;
    AVS_PIN_UNLOCK(pin_mask);
    if (!command_slot) {                          // a newer CMD17/18 preempted this build; pump will discard it
      memcpy(out, bus4 ? zero_w4 : zero_w1, (size_t)(bus4 ? zero_n4 : zero_n1) * 4);
      *nw = bus4 ? zero_n4 : zero_n1;
      return AVS_READY;
    }
    S.t_read = t;
    if (off == 0 && cfg.whole_sf && S.have != S.total && !degraded) {   // hold before the first block instead of inside the read
      if (!holding || hold_lba != lba) { holding = 1; hold_lba = lba; hold_t0 = t; st.holds++; }
      uint32_t lim = cfg.hold_cut_ms < AVS_MAX_HOLD_MS ? cfg.hold_cut_ms : AVS_MAX_HOLD_MS;
      if (t - hold_t0 < lim * 1000u) return AVS_HOLD;
      holding = 0; degraded = 1; st.hold_timeouts++; st.notready++;   // not marked served: nothing of it was delivered
      synth(SFF_NOTREADY, S.seq, have_complete ? newest_complete : 0, bus4, out, nw);
      return AVS_READY;
    }
    if (off == 0 && !S.served) {
      S.served = 1; st.served_seq = S.seq; have_served = 1;
      uint8_t h = srv_head, nh = (uint8_t)((h + 1) & 15);
      if (nh != srv_tail) { srv_seq[h] = S.seq; srv_t[h] = t; srv_head = nh; }
    }
    if (off >= S.total) { memcpy(out, bus4 ? zero_w4 : zero_w1, (size_t)(bus4 ? zero_n4 : zero_n1) * 4); *nw = bus4 ? zero_n4 : zero_n1; return AVS_READY; }
    if (S.idx[off] != 0xFFFF) { holding = 0; copy_image(S, off, bus4, out, nw); st.sectors_served++; return AVS_READY; }
    if (!degraded) {                                 // cut-through: hold the start bit until it arrives
      if (!holding || hold_lba != lba) { holding = 1; hold_lba = lba; hold_t0 = t; st.holds++; }
      uint32_t lim = cfg.hold_cut_ms < AVS_MAX_HOLD_MS ? cfg.hold_cut_ms : AVS_MAX_HOLD_MS;
      if (t - hold_t0 < lim * 1000u) return AVS_HOLD;
      holding = 0; degraded = 1; st.hold_timeouts++; st.notready++;
    }
    synth(SFF_NOTREADY, S.seq, have_complete ? newest_complete : 0, bus4, out, nw); // the payload CRC catches it; the RNS-E drops this superframe
    return AVS_READY;
  }
  // not resident: past (evicted) or future (not begun yet)
  uint32_t req = (uint32_t)win;
  if (have_begun) {
    int d = (int8_t)(uint8_t)((uint32_t)win - (newest_begun & 0xFF));
    req = newest_begun + (uint32_t)d;
    if (d <= 0) {
      sticky_win = win; sticky_kind = SFF_EXPIRED; sticky_seq = req; st.expired++; holding = 0;
      sticky_arg = have_complete ? newest_complete : newest_begun;
      synth(SFF_EXPIRED, req, sticky_arg, bus4, out, nw);
      return AVS_READY;
    }
  }
  if (!holding || hold_lba != lba) { holding = 1; hold_lba = lba; hold_t0 = t; st.holds++; }
  uint32_t lim = cfg.hold_future_ms < AVS_MAX_HOLD_MS ? cfg.hold_future_ms : AVS_MAX_HOLD_MS;
  if (t - hold_t0 < lim * 1000u) return AVS_HOLD;
  holding = 0; st.hold_timeouts++;
  int silent = !link_active || (t - last_rx_us) > (uint32_t)cfg.silent_ms * 1000u;
  sticky_win = win; sticky_arg = 0;
  if (silent) { sticky_kind = SFF_IDLE; sticky_seq = have_complete ? newest_complete : 0; st.idle++; }
  else { sticky_kind = SFF_NOTREADY; sticky_seq = req; st.notready++; sticky_arg = have_complete ? newest_complete : 0; }
  synth(sticky_kind, sticky_seq, sticky_arg, bus4, out, nw);
  return AVS_READY;
}

// ---------------------------------------------------------------- TX (main loop)
void AvStream::send(uint16_t type, const void *pl, uint32_t len) {
  uint8_t b[16 + 96];
  if (!tx || len > 96) return;
  wle32(b, TL_MAGIC); wle16(b + 4, type); wle16(b + 6, 0); wle32(b + 8, len); wle32(b + 12, tx_seq++);
  memcpy(b + 16, pl, len);
  tx(b, (int)(16 + len));
}

void AvStream::send_status(uint32_t t) {
  AvsStatus s = st;
  s.free_sectors = (uint32_t)free_n;
  s.free_slots = (uint32_t)(AVS_NSLOT - used_slots());
  s.newest_seq = newest_complete;
  s.card_state = card_state; s.bus_width = bus_width; s.mode = mode; s.crc7_rejects = crc7_rejects;
  s.flags = (uint8_t)((have_complete ? 1 : 0) | (have_served ? 2 : 0) | (have_begun ? 4 : 0));
  send(TL_STATUS, &s, sizeof s);
  last_status_us = t; last_sent_free = s.free_sectors; last_sent_slots = s.free_slots;
}

void AvStream::poll(void) {
  uint32_t t = now();
  reclaim(t);
  if (!link_active) { srv_tail = srv_head; return; }
  while (srv_tail != srv_head) {
    uint8_t p[8];
    wle32(p, srv_seq[srv_tail]); wle32(p + 4, srv_t[srv_tail]);
    srv_tail = (uint8_t)((srv_tail + 1) & 15);
    send(TL_SERVED, p, 8);
  }
  hs_poll(t);
  uint32_t period = 1000000u / (cfg.status_hz ? cfg.status_hz : 20);
  uint32_t fs = (uint32_t)(AVS_NSLOT - used_slots());
  if (t - last_status_us >= period ||
      ((last_sent_free != (uint32_t)free_n || last_sent_slots != fs) && t - last_status_us >= 2000u))
    send_status(t);
}

void AvStream::info(uint8_t *p, int n) {
  if (n < 40) return;
  wbe32(p, newest_complete); wbe32(p + 4, newest_begun); wbe32(p + 8, st.served_seq);
  wbe32(p + 12, (uint32_t)free_n); wbe32(p + 16, (uint32_t)pool_n); wbe32(p + 20, (uint32_t)used_slots());
  wbe32(p + 24, st.sectors_served); wbe32(p + 28, st.notready); wbe32(p + 32, st.expired); wbe32(p + 36, st.evicted);
}

// ---------------------------------------------------------------- unit test (private instance, fake clock)
static uint32_t t_now;
static uint32_t t_clock(void) { return t_now; }
static uint8_t t_tx[4096]; static int t_txn;
static void t_txf(const uint8_t *p, int n) { if (t_txn + n <= (int)sizeof t_tx) { memcpy(t_tx + t_txn, p, (size_t)n); t_txn += n; } }
static char t_con[64]; static int t_conn;
static void t_cons(uint8_t c) { if (t_conn < 63) t_con[t_conn++] = (char)c; }
static uint8_t t_msg[16 + 12 + 4 * 528];
static int t_mk(uint16_t type, const uint8_t *pl, uint32_t len) {
  wle32(t_msg, TL_MAGIC); wle16(t_msg + 4, type); wle16(t_msg + 6, 0); wle32(t_msg + 8, len); wle32(t_msg + 12, 0);
  memcpy(t_msg + 16, pl, len);
  return (int)(16 + len);
}
static void t_feed(AvStream &a, int n, int step) {     // feed t_msg in pieces of `step` bytes
  for (int i = 0; i < n; i += step) a.feed(t_msg + i, n - i < step ? n - i : step, t_cons);
}
static void t_sector(uint8_t *b, uint32_t seq, int i) { for (int k = 0; k < 512; k++) b[k] = (uint8_t)(seq * 31 + i * 7 + k * 13); }
static void t_send_sf(AvStream &a, uint32_t seq, int total, int nsend, int step) {
  uint8_t pl[12 + 4 * 528];
  wle32(pl, seq); wle16(pl + 4, (uint16_t)total); wle16(pl + 6, 0);
  t_feed(a, t_mk(TL_SF_BEGIN, pl, 8), step);
  for (int i = 0; i < nsend; i++) {
    uint8_t b[512]; uint32_t w[WIRE_MAX_WORDS];
    t_sector(b, seq, i);
    wire4(b, 512, w);
    wle32(pl, seq); wle16(pl + 4, (uint16_t)i); wle16(pl + 6, 1); wle32(pl + 8, 4);
    memcpy(pl + 12, w, 528);
    t_feed(a, t_mk(TL_WIRE, pl, 12 + 528), step);
  }
  if (nsend == total) { wle32(pl, seq); t_feed(a, t_mk(TL_SF_END, pl, 4), step); }
}
static int t_get(AvStream &a, uint32_t lba, int bus4, uint8_t *bytes) {   // READY -> decoded bytes
  uint32_t w[WIRE_MAX_WORDS]; uint16_t n = 0;
  int r = a.get(lba, bus4, w, &n);
  if (r == AVS_READY && bytes) {
    avs_unwire(w, bus4, bytes);
    uint32_t chk[WIRE_MAX_WORDS];                    // the served image must be a valid, CRC-correct image
    int m = bus4 ? wire4(bytes, 512, chk) : wire1(bytes, 512, chk);
    if (m != n || memcmp(chk, w, (size_t)n * 4)) return -1;
  }
  return r;
}
static uint32_t be32r(const uint8_t *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }

int avs_selftest(char *msg, int ml) {
#define T_FAIL(...) do { snprintf(msg, (size_t)ml, __VA_ARGS__); free(pool); return 1; } while (0)
  enum { NP = 24 };
  AvsSector *pool = (AvsSector *)malloc(sizeof(AvsSector) * NP);
  if (!pool) { snprintf(msg, (size_t)ml, "FAIL no heap"); return 1; }
  AvStream a;                                         // 11 KB on the stack: console only, not during streaming; a static would cost pool
  memset((void *)&a, 0, sizeof a);
  AvsSector *banks[2] = {pool, pool + 16};
  uint16_t bn[2] = {16, 8};
  t_now = 1000000; t_txn = 0; t_conn = 0;
  a.init(banks, bn, 2, t_clock, t_txf);
  const uint8_t txt[] = "hi TL ok\n";              // console text, incl. a partial magic, before any message
  a.feed(txt, (int)sizeof txt - 1, t_cons);
  if (t_conn != 9 || memcmp(t_con, txt, 9) || a.link_active) T_FAIL("FAIL console passthrough");
  uint8_t b[512], e[512];
  t_send_sf(a, 5, 3, 3, 7);                          // seq 5, 3 sectors, fed 7 bytes at a time
  if (a.free_sectors() != NP - 3 || !a.link_active) T_FAIL("FAIL pool after sf5 free=%d", a.free_sectors());
  uint32_t L5 = AVS_WIN_LBA + 5 * 256;
  a.read_begin(L5);
  for (int i = 0; i < 3; i++) {
    t_sector(e, 5, i);
    if (t_get(a, L5 + (uint32_t)i, 1, b) != AVS_READY || memcmp(b, e, 512)) T_FAIL("FAIL serve sf5 sector %d", i);
  }
  if (t_get(a, L5 + 3, 1, b) != AVS_READY) T_FAIL("FAIL beyond total");
  for (int k = 0; k < 512; k++) if (b[k]) T_FAIL("FAIL beyond total not zero");
  a.read_begin(L5 + 1);
  t_sector(e, 5, 1);                                  // 1-bit bus: the Teensy re-encodes the 4-bit image
  if (t_get(a, L5 + 1, 0, b) != AVS_READY || memcmp(b, e, 512) || a.st.converted != 1) T_FAIL("FAIL 1-bit convert");
  if (t_get(a, 0x40100, 1, b) != AVS_NOTWIN) T_FAIL("FAIL notwin");
  // future slot (seq 6, nothing sent): hold, then NOT_READY, then the same answer at once for the rest
  uint32_t L6 = AVS_WIN_LBA + 6 * 256;
  a.read_begin(L6);
  if (t_get(a, L6, 1, b) != AVS_HOLD) T_FAIL("FAIL future hold");
  t_now += 10000;
  if (t_get(a, L6, 1, b) != AVS_HOLD) T_FAIL("FAIL future hold 10ms");
  t_now += 15000;
  if (t_get(a, L6, 1, b) != AVS_READY || be32r(b + 8) != 6 || be32r(b + 12) != (SFF_SYNTH | SFF_NOTREADY))
    T_FAIL("FAIL NOT_READY seq=%lu flags=%lx", (unsigned long)be32r(b + 8), (unsigned long)be32r(b + 12));
  if (crc32z(b + 64, 448, crc32z(b, 60)) != be32r(b + 60)) T_FAIL("FAIL NOT_READY hdr crc");
  if (t_get(a, L6 + 1, 1, b) != AVS_READY || be32r(b + 12) != (SFF_SYNTH | SFF_NOTREADY)) T_FAIL("FAIL sticky");
  // past slot (seq 2 < newest 5, not resident): EXPIRED naming the newest seq
  a.read_begin(AVS_WIN_LBA + 2 * 256);
  if (t_get(a, AVS_WIN_LBA + 2 * 256, 1, b) != AVS_READY || be32r(b + 12) != (SFF_SYNTH | SFF_EXPIRED) ||
      be32r(b + 8) != 2 || be32r(b + 64) != 5) T_FAIL("FAIL EXPIRED");
  // cut-through: seq 6 begun with 1 of 3 sectors; sector 1 holds, arrives during the hold -> served
  t_send_sf(a, 6, 3, 1, 600);
  a.read_begin(L6);
  t_sector(e, 6, 0);
  if (t_get(a, L6, 1, b) != AVS_READY || memcmp(b, e, 512)) T_FAIL("FAIL cut sector 0");
  if (t_get(a, L6 + 1, 1, b) != AVS_HOLD) T_FAIL("FAIL cut hold");
  {
    uint8_t pl[12 + 528]; uint32_t w[WIRE_MAX_WORDS];
    for (int i = 1; i < 3; i++) {
      t_sector(e, 6, i); wire4(e, 512, w);
      wle32(pl, 6); wle16(pl + 4, (uint16_t)i); wle16(pl + 6, 1); wle32(pl + 8, 4); memcpy(pl + 12, w, 528);
      t_feed(a, t_mk(TL_WIRE, pl, 12 + 528), 1000);
      t_now += 5000;
      if (t_get(a, L6 + (uint32_t)i, 1, b) != AVS_READY || memcmp(b, e, 512)) T_FAIL("FAIL cut sector %d", i);
    }
    wle32(pl, 6); t_feed(a, t_mk(TL_SF_END, pl, 4), 3);
  }
  // cut-through timeout: seq 7 with 1 of 2 sectors, never completed -> NOT_READY after 30 ms
  t_send_sf(a, 7, 2, 1, 64);
  uint32_t L7 = AVS_WIN_LBA + 7 * 256;
  a.read_begin(L7);
  t_get(a, L7, 1, b);
  if (t_get(a, L7 + 1, 1, b) != AVS_HOLD) T_FAIL("FAIL cut2 hold");
  t_now += 31000;
  if (t_get(a, L7 + 1, 1, b) != AVS_READY || be32r(b + 12) != (SFF_SYNTH | SFF_NOTREADY)) T_FAIL("FAIL cut timeout");
  // reclaim: 5, 6, 7 served -> 5 and 6 are freed (a newer one served), 7 stays (newest, pinned); STATUS reports it
  t_txn = 0;
  a.poll();
  if (a.slot(0)->used && a.slot(0)->seq == 5) T_FAIL("FAIL reclaim sf5");
  if (a.used_slots() != 1) T_FAIL("FAIL reclaim keep one, used=%d", a.used_slots());
  int fs = -1, srv = 0;
  for (int i = 0; i + 16 <= t_txn;) {
    uint32_t len = le32(t_tx + i + 8);
    if (le32(t_tx + i) != TL_MAGIC) T_FAIL("FAIL tx framing");
    if (le16(t_tx + i + 4) == TL_STATUS) fs = (int)le32(t_tx + i + 16 + 4);
    if (le16(t_tx + i + 4) == TL_SERVED) srv++;
    i += 16 + (int)len;
  }
  if (fs != a.free_sectors() || fs != NP - 1 || srv != 3) T_FAIL("FAIL status free=%d srv=%d", fs, srv);
  // silent Pi: future slot -> IDLE (seq = newest complete)
  t_now += 300000;
  a.read_begin(AVS_WIN_LBA + 9 * 256);
  t_get(a, AVS_WIN_LBA + 9 * 256, 1, b);
  t_now += 21000;
  if (t_get(a, AVS_WIN_LBA + 9 * 256, 1, b) != AVS_READY || be32r(b + 12) != (SFF_SYNTH | SFF_IDLE) || be32r(b + 8) != 6)
    T_FAIL("FAIL IDLE flags=%lx seq=%lu", (unsigned long)be32r(b + 12), (unsigned long)be32r(b + 8));
  // FLUSH >= 6: slots 6 and 7 dropped, pool back to full
  { uint8_t pl[4]; wle32(pl, 6); t_feed(a, t_mk(TL_FLUSH, pl, 4), 2); }
  if (a.used_slots() != 0 || a.free_sectors() != NP) T_FAIL("FAIL flush used=%d free=%d", a.used_slots(), a.free_sectors());
  // credit overrun: 30 sectors into a 24-sector pool -> oldest slot evicted, nothing corrupt
  t_send_sf(a, 10, 20, 20, 528);
  t_send_sf(a, 11, 10, 10, 528);
  a.read_begin(AVS_WIN_LBA + 11 * 256 + 9);
  t_sector(e, 11, 9);
  if (t_get(a, AVS_WIN_LBA + 11 * 256 + 9, 1, b) != AVS_READY || memcmp(b, e, 512)) T_FAIL("FAIL overrun serve");
  // version handshake (PROTOCOL 4.5): build F46C419D, fmt 1 (min 1), fw_id 0x54B, table 1
  {
    uint16_t w[AVS_HS_N] = {(uint16_t)(AVS_HS_W0 | AVS_HS_VER), 0xF46C, 0x419D, 0x0101, 0x0000, 0x054B, 0x0001, 0};
    uint32_t sum = 0;
    for (int k = 0; k < AVS_HS_N - 1; k++) sum += w[k];
    w[7] = (uint16_t)~sum;
    uint32_t L7 = AVS_HS_LBA + (7u << 16) + w[7];
    AvsRnseInfo ri;
    if (a.hs_info(&ri) || a.hs_patch(L7, b)) T_FAIL("FAIL hs before any announcement");
    for (int k = 0; k < AVS_HS_N; k++) {
      a.read_begin(AVS_HS_LBA + ((uint32_t)k << 16) + w[k]);
      if (k == 3) a.read_begin(AVS_WIN_LBA);            // a probe / FS read in between does not disturb it
    }
    if (!a.hs_info(&ri) || ri.build_id != 0xF46C419Du || ri.fmt != 1 || ri.fmt_min != 1 || ri.hs_ver != 1 || ri.fw_id != 0x54Bu ||
        ri.table_ver != 1 || ri.check != w[7] || ri.count != 1 || ri.size != 32 || sizeof(AvsRnseInfo) != 32)
      T_FAIL("FAIL hs decode");
    memset(b, 0xEE, sizeof b);
    if (!a.hs_patch(L7, b) || be32r(b) != AVS_HS_ACK || b[20] != AVS_HS_PENDING || b[21] || be32r(b + 24) != 1) T_FAIL("FAIL hs ack");
    for (int k = 0; k < AVS_HS_N; k++) if (((b[4 + 2 * k] << 8) | b[5 + 2 * k]) != w[k]) T_FAIL("FAIL hs ack echo %d", k);
    if (a.hs_patch(L7 - 1, b) || a.hs_patch(AVS_HS_LBA, b)) T_FAIL("FAIL hs patch of another LBA");
    t_txn = 0;
    a.poll();
    int seen = 0;
    for (int i = 0; i + 16 <= t_txn; i += 16 + (int)le32(t_tx + i + 8))
      if (le16(t_tx + i + 4) == TL_RNSE_INFO && le32(t_tx + i + 8) == 32 && !memcmp(t_tx + i + 16, &ri, 8)) seen++;
    if (seen != 1) T_FAIL("FAIL hs INFO to the Pi (%d)", seen);
    t_txn = 0; a.poll();
    for (int i = 0; i + 16 <= t_txn; i += 16 + (int)le32(t_tx + i + 8)) if (le16(t_tx + i + 4) == TL_RNSE_INFO) T_FAIL("FAIL hs INFO resent");
    uint8_t vp[8];
    wle32(vp, 0xF46C419Eu); wle16(vp + 4, w[7]); vp[6] = AVS_HS_ACCEPT; vp[7] = 1;      // another build: ignored
    t_feed(a, t_mk(TL_HS_VERDICT, vp, 8), 3);
    if (a.hs_patch(L7, b) != 1 || b[20] != AVS_HS_PENDING) T_FAIL("FAIL hs verdict for another build");
    wle32(vp, 0xF46C419Du);
    t_feed(a, t_mk(TL_HS_VERDICT, vp, 8), 5);
    if (a.hs_patch(L7, b) != 1 || b[20] != AVS_HS_ACCEPT || b[21] != 1) T_FAIL("FAIL hs accept");
    vp[6] = AVS_HS_REJECT; vp[7] = 0;
    t_feed(a, t_mk(TL_HS_VERDICT, vp, 8), 8);
    if (a.hs_patch(L7, b) != 1 || b[20] != AVS_HS_REJECT || b[21] != 0) T_FAIL("FAIL hs reject");
    // a bad check word, an out-of-order sequence and a missing start decode nothing
    for (int k = 0; k < AVS_HS_N; k++) a.read_begin(AVS_HS_LBA + ((uint32_t)k << 16) + (k == 7 ? (uint16_t)(w[7] ^ 1) : w[k]));
    for (int k = 0; k < AVS_HS_N; k++) a.read_begin(AVS_HS_LBA + ((uint32_t)(k == 2 ? 3 : k) << 16) + w[k]);
    for (int k = 1; k < AVS_HS_N; k++) a.read_begin(AVS_HS_LBA + ((uint32_t)k << 16) + w[k]);
    if (!a.hs_info(&ri) || ri.count != 1) T_FAIL("FAIL hs accepted a bad announcement (count %u)", (unsigned)ri.count);
    // the same announcement again: same verdict, count 2, INFO again; a new sender gets it too, without the old verdict
    for (int k = 0; k < AVS_HS_N; k++) a.read_begin(AVS_HS_LBA + ((uint32_t)k << 16) + w[k]);
    if (a.hs_patch(L7, b) != 1 || b[20] != AVS_HS_REJECT || be32r(b + 24) != 2) T_FAIL("FAIL hs repeat");
    a.reset_link();
    if (a.hs_patch(L7, b) != 1 || b[20] != AVS_HS_PENDING) T_FAIL("FAIL hs verdict kept over a link reset");
    { uint8_t cp[16]; memset(cp, 0, sizeof cp); t_feed(a, t_mk(TL_CONFIG, cp, 16), 16); }
    t_txn = 0; a.poll();
    seen = 0;
    for (int i = 0; i + 16 <= t_txn; i += 16 + (int)le32(t_tx + i + 8)) if (le16(t_tx + i + 4) == TL_RNSE_INFO) seen++;
    if (seen != 1) T_FAIL("FAIL hs INFO to a new sender (%d)", seen);
  }
  free(pool);
  snprintf(msg, (size_t)ml, "PASS console,serve,beyond,1bit-convert,future-hold,NOT_READY,EXPIRED,cut-through,"
           "cut-timeout,reclaim,STATUS,IDLE,FLUSH,overrun,handshake");
  return 0;
#undef T_FAIL
}
