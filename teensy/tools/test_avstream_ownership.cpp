// Native regressions against AvStream itself, including an IRQ between victim selection and retirement.
#include "avstream.h"
#include "config.h"
#include "sdproto.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

static uint32_t clock_us = 1000000;
static AvStream *clock_preempt_stream;
static uint32_t clock_preempt_lba;
static uint32_t clock_now() {
  if (clock_preempt_stream) {
    AvStream *a = clock_preempt_stream; clock_preempt_stream = nullptr;
    a->read_begin(clock_preempt_lba); // simulate prio-16 command arriving inside an old get()
  }
  return clock_us;
}
static void require(bool ok, const char *message) {
  if (!ok) { std::fprintf(stderr, "FAIL %s\n", message); std::exit(1); }
}
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16)); }
static uint32_t lba(uint32_t seq, int off = 0) { return AVS_WIN_LBA + (seq & 255u) * 256u + (uint32_t)off; }
static void data(uint8_t *p, uint32_t seq, int off, int variant = 0) {
  for (int i = 0; i < 512; i++) p[i] = (uint8_t)(seq * 31u + (seq >> 8) * 83u + off * 7 + i * 13 + variant);
}
static AvStream *irq_stream;
static uint32_t irq_lba;
static int irq_count;
void avs_test_before_lock() {
  if (!irq_stream) return;
  AvStream *a = irq_stream;
  irq_stream = nullptr; // get() itself does not lock; arm only one simulated interrupt
  uint32_t w[WIRE_MAX_WORDS]; uint16_t n;
  a->read_begin(irq_lba);
  require(a->get(irq_lba, 1, w, &n) == AVS_READY, "interleaved IRQ read");
  irq_count++;
}
static void interrupt_next_lock(AvStream &a, uint32_t seq) { irq_stream = &a; irq_lba = lba(seq); }

struct Fixture {
  std::vector<AvsSector> pool;
  AvsSector *bank;
  uint16_t bank_n;
  AvStream a;
  explicit Fixture(int n) : pool((size_t)n), bank(pool.data()), bank_n((uint16_t)n) {
    a.init(&bank, &bank_n, 1, clock_now, nullptr);
  }
  void message(uint16_t type, const uint8_t *p, int n, int step = 17) {
    std::vector<uint8_t> m((size_t)n + 16);
    put32(m.data(), TL_MAGIC); put16(m.data() + 4, type); put32(m.data() + 8, (uint32_t)n);
    if (n) std::memcpy(m.data() + 16, p, (size_t)n);
    for (int pos = 0; pos < n + 16; pos += step)
      a.feed(m.data() + pos, n + 16 - pos < step ? n + 16 - pos : step, nullptr);
  }
  void begin(uint32_t seq, int total) {
    uint8_t p[8] = {}; put32(p, seq); put16(p + 4, (uint16_t)total); message(TL_SF_BEGIN, p, 8);
  }
  std::vector<uint8_t> wire_message(uint32_t seq, int off, int variant = 0) {
    uint8_t b[512]; uint32_t w[WIRE_MAX_WORDS]; data(b, seq, off, variant); wire4(b, 512, w);
    std::vector<uint8_t> m(16 + 12 + 528);
    put32(m.data(), TL_MAGIC); put16(m.data() + 4, TL_WIRE); put32(m.data() + 8, 12 + 528);
    put32(m.data() + 16, seq); put16(m.data() + 20, (uint16_t)off);
    put16(m.data() + 22, 1); put32(m.data() + 24, 4); std::memcpy(m.data() + 28, w, 528);
    return m;
  }
  void wire(uint32_t seq, int off, int variant = 0) {
    auto m = wire_message(seq, off, variant);
    for (size_t pos = 0; pos < m.size(); pos += 19)
      a.feed(m.data() + pos, (int)(m.size() - pos < 19 ? m.size() - pos : 19), nullptr);
  }
  void end(uint32_t seq) { uint8_t p[4]; put32(p, seq); message(TL_SF_END, p, 4); }
  void send(uint32_t seq, int total, int nsend = -1) {
    if (nsend < 0) nsend = total;
    begin(seq, total); for (int i = 0; i < nsend; i++) wire(seq, i); if (nsend == total) end(seq);
  }
  const AvsSlot *find(uint32_t seq) const {
    for (int i = 0; i < AVS_NSLOT; i++) if (a.slot(i)->used && a.slot(i)->seq == seq) return a.slot(i);
    return nullptr;
  }
  int get(uint32_t seq, int off = 0, int bus4 = 1) {
    uint32_t w[WIRE_MAX_WORDS], expected[WIRE_MAX_WORDS]; uint16_t n = 0;
    int result = a.get(lba(seq, off), bus4, w, &n);
    if (result == AVS_READY) {
      uint8_t decoded[512]; avs_unwire(w, bus4, decoded);
      int nw = bus4 ? wire4(decoded, 512, expected) : wire1(decoded, 512, expected);
      require(n == nw && !std::memcmp(w, expected, (size_t)nw * 4), "CRC-correct served wire image");
    }
    return result;
  }
  void exact(uint32_t seq, int off = 0, int bus4 = 1, int variant = 0) {
    uint32_t w[WIRE_MAX_WORDS]; uint16_t n = 0; uint8_t b[512], expected[512];
    a.read_begin(lba(seq, off));
    require(get(seq, off, bus4) == AVS_READY, "resident sector ready");
    require(a.get(lba(seq, off), bus4, w, &n) == AVS_READY, "repeat resident sector ready");
    avs_unwire(w, bus4, b); data(expected, seq, off, variant);
    require(!std::memcmp(b, expected, 512), "resident/retry bytes preserved");
  }
  void storage() const {
    std::vector<int> owners(pool.size(), 0); int count = 0, used = 0;
    for (int s = 0; s < AVS_NSLOT; s++) {
      const AvsSlot &x = *a.slot(s); if (!x.used) continue;
      used++; int have = 0;
      for (int off = 0; off < x.total; off++) if (x.idx[off] != 0xffff) {
        require(x.idx[off] < pool.size(), "pool index bounds");
        require(++owners[x.idx[off]] == 1, "unique pool ownership"); count++; have++;
      }
      require(have == x.have, "slot have accounting");
    }
    require(count + a.free_sectors() == (int)pool.size(), "pool conservation");
    require(used == a.used_slots(), "table accounting");
  }
};

static void pool_split_retry_progress() {
  Fixture f(8); f.send(10, 4); f.exact(10);
  f.send(11, 8); // writer and pin own all eight sectors; remaining sectors must be dropped
  require(f.find(10) && f.find(11) && f.find(11)->have == 4, "pool pressure keeps pinned frame");
  require(f.a.st.rx_sectors == 12 && f.a.st.rx_errors == 4, "dropped WIRE acknowledged and counted");
  f.exact(10, 1); // split command
  uint32_t w[WIRE_MAX_WORDS]; uint16_t nw;
  require(f.a.get(0x40100, 1, w, &nw) == AVS_NOTWIN, "metadata does not release retry pin");
  f.a.read_begin(0x40100); f.exact(10, 2, 0); // retry after metadata command in another width
  clock_us += 2000000; f.a.poll(); f.exact(10, 3); f.storage();
  f.exact(11); // host progress releases frame 10
  for (int i = 4; i < 8; i++) f.wire(11, i);
  f.end(11); require(!f.find(10) && f.find(11)->have == 8, "progress releases old pin and writer resumes");
  f.exact(11, 7); f.storage();
  uint8_t p[4]; put32(p, 11); f.message(TL_FLUSH, p, 4);
  require(f.a.used_slots() == 0 && f.a.free_sectors() == 8, "explicit FLUSH invalidates active pin");
  f.send(12, 8); f.exact(12); f.a.reset_link(); f.storage();
  require(f.a.used_slots() == 0 && f.a.free_sectors() == 8, "reset invalidates active pin");
}

static void collision_and_duplicates() {
  Fixture f(12); f.send(5, 3); f.exact(5);
  uint32_t begun = f.a.st.begun, errors = f.a.st.rx_errors;
  f.send(261, 3);
  require(f.find(5) && !f.find(261), "same-window wrap collision rejected while pinned");
  require(f.a.st.begun == begun + 1 && f.a.st.rx_errors > errors, "rejected BEGIN counted for credits");
  f.begin(5, 1); f.wire(5, 1, 77); f.exact(5, 1); // same-sequence resend cannot mutate retained bytes
  f.send(6, 1); f.exact(6); // release pin 5
  auto m = f.wire_message(5, 1, 99);
  f.a.feed(m.data(), (int)m.size() - 1, nullptr); // copy replacement image while 5 is unpinned
  interrupt_next_lock(f.a, 5);
  f.a.feed(m.data() + m.size() - 1, 1, nullptr); // pin just before sector_done publishes
  require(!irq_stream, "duplicate publication interleave exercised"); f.exact(5, 1); f.storage();
  f.exact(6); f.send(261, 3); // collision is now allowed and storage makes forward progress
  require(!f.find(5) && f.find(261), "same-window replacement allowed after host progress");
  f.exact(261, 2); f.storage();
}

static void table_pressure_and_race() {
  Fixture f(32); for (uint32_t i = 0; i < AVS_NSLOT; i++) f.send(i, 1);
  f.exact(0); f.exact(15);
  interrupt_next_lock(f.a, 0); f.begin(16, 1); // oldest served victim becomes pinned after selection
  require(!irq_stream && f.find(0) && f.find(16), "table retirement rechecks pin atomically");
  f.wire(16, 0); f.end(16);
  for (uint32_t i = 17; i < 50; i++) f.send(i, 1);
  require(f.a.used_slots() == AVS_NSLOT && f.find(0), "table pressure cannot evict current retry frame");
  f.exact(0); f.storage();
}

static void pool_retirement_race() {
  Fixture f(4); f.send(0, 2); f.send(1, 2); f.exact(1); f.begin(2, 1);
  interrupt_next_lock(f.a, 0); f.wire(2, 0); f.end(2);
  require(!irq_stream && f.find(0) && !f.find(1) && f.find(2), "pool retirement rechecks pin atomically");
  f.exact(0, 1); f.storage();
}

static void reclaim_race_and_cutthrough() {
  Fixture f(8); f.send(0, 2); f.send(1, 1); f.send(2, 1);
  f.exact(0); f.exact(1); f.exact(2);
  interrupt_next_lock(f.a, 0); f.a.poll();
  require(!irq_stream && f.find(0), "reclaim cannot retire newly pinned retry");
  clock_us += 3000000; f.a.poll(); f.exact(0, 1); f.storage();
  f.exact(2); f.a.poll(); require(!f.find(0), "mode A reclaim resumes after pin progress");
  f.send(3, 2, 1); f.exact(3); require(f.get(3, 1) == AVS_HOLD, "pinned cut-through still holds");
  f.wire(3, 1); f.end(3); f.exact(3, 1); f.storage();
}

static void future_read_does_not_release() {
  Fixture f(4); f.send(1, 2); f.exact(1); f.a.read_begin(lba(2));
  require(f.get(2) == AVS_HOLD, "future request holds");
  clock_us += 60000; require(f.get(2) == AVS_READY, "future request times out synthetically");
  f.send(3, 4); require(f.find(1) && f.find(3)->have == 2, "nonresident synthetic read retains real pin");
  f.exact(1, 1); f.storage();
  f.exact(3); f.wire(3, 2); f.wire(3, 3); f.exact(3, 3); f.storage();
}

static void command_anchor_prefetch_and_abort() {
  Fixture f(260); f.send(7, 256); f.send(8, 1);
  f.exact(7, 254); // CMD18 starting near the end of a full window
  require(f.get(7, 255) == AVS_READY, "last anchored sector available to prefetch");
  uint32_t served = f.a.st.sectors_served;
  require(f.get(8) == AVS_READY, "cross-window prefetch returns filler");
  require(!f.find(8)->served && f.a.st.sectors_served == served, "cross-window prefetch has no served side effects");
  uint32_t w[WIRE_MAX_WORDS]; uint16_t nw; uint8_t b[512];
  f.a.get(lba(8), 1, w, &nw); avs_unwire(w, 1, b);
  for (uint8_t v : b) require(v == 0, "cross-window filler is zero");
  f.begin(7 + 256, 1); require(f.find(7) && !f.find(7 + 256), "discarded prefetch cannot release original retry pin");
  f.exact(7, 255); f.storage();
  f.a.read_begin(lba(8));
  clock_preempt_stream = &f.a; clock_preempt_lba = 0x40100; // CMD12 + metadata command supersedes speculative build
  require(f.get(8) == AVS_READY && !clock_preempt_stream, "old generation get preempted before commit");
  require(!f.find(8)->served, "aborted old get cannot announce served header");
  f.begin(7 + 256, 1); require(f.find(7), "aborted old get cannot transfer pin");
  f.exact(7, 255); f.exact(8);
  f.send(7 + 256, 1); require(!f.find(7) && f.find(7 + 256), "new anchored command allows progress");
  f.storage();
}

// ---- review finding 1: the reader must always make progress across a 256-superframe alias of its old pin.
// Reader = rnse_proto/src/sdstream.c reader_main branch logic (EXPIRED jump, NOTREADY/IDLE resync, "older seq = retry").
static uint32_t rd32be(const uint8_t *p) { return (uint32_t)p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3]; }
struct AliasStream : Fixture {
  explicit AliasStream(int n) : Fixture(n) {}
  void header_wire(uint32_t seq, int off, int total) { // a real superframe header, so the reader can parse sector 0
    uint8_t d[512] = {}; uint32_t w[WIRE_MAX_WORDS];
    if (off == 0) {
      d[0] = 'R'; d[1] = 'S'; d[2] = 'A'; d[3] = 'V'; d[5] = 1; d[6] = 0; d[7] = 64;
      d[8] = (uint8_t)(seq >> 24); d[9] = (uint8_t)(seq >> 16); d[10] = (uint8_t)(seq >> 8); d[11] = (uint8_t)seq;
      d[41] = (uint8_t)total;
    }
    wire4(d, 512, w);
    std::vector<uint8_t> m(16 + 12 + 528);
    put32(m.data(), TL_MAGIC); put16(m.data() + 4, TL_WIRE); put32(m.data() + 8, 12 + 528);
    put32(m.data() + 16, seq); put16(m.data() + 20, (uint16_t)off); put16(m.data() + 22, 1); put32(m.data() + 24, 4);
    std::memcpy(m.data() + 28, w, 528);
    a.feed(m.data(), (int)m.size(), nullptr);
  }
  void push(uint32_t seq, int total) {
    begin(seq, total); for (int i = 0; i < total; i++) header_wire(seq, i, total); end(seq);
  }
  uint32_t synth_flags(uint32_t seq) {                  // header flags the reader would see for this window
    uint32_t l = lba(seq), w[WIRE_MAX_WORDS]; uint16_t n = 0; int r; uint8_t h[512];
    a.read_begin(l);
    while ((r = a.get(l, 1, w, &n)) == AVS_HOLD) clock_us += 1000;
    avs_unwire(w, 1, h);
    return rd32be(h + 12);
  }
  // one reader_main pass: returns the next seq, sets accepted. Holds and the reader's sleeps advance the clock.
  uint32_t reader(uint32_t seq, bool *accepted) {
    *accepted = false;
    uint32_t l = lba(seq), w[WIRE_MAX_WORDS]; uint16_t n = 0; int r; uint8_t h[512];
    a.read_begin(l);
    while ((r = a.get(l, 1, w, &n)) == AVS_HOLD) { clock_us += 1000; a.poll(); }
    avs_unwire(w, 1, h);
    uint32_t hf = rd32be(h + 12), ss = rd32be(h + 8);
    if (hf & SFF_EXPIRED) return rd32be(h + 64);
    if (hf & (SFF_IDLE | SFF_NOTREADY)) {
      uint32_t nw = (hf & SFF_NOTREADY) ? rd32be(h + 64) : ss;
      if (nw && ((int32_t)(nw - seq) > 4 || (int32_t)(seq - nw) > 64)) return nw;
      clock_us += 2000; return seq;
    }
    if (ss != seq) { if ((int32_t)(ss - seq) > 0) seq = ss; else { clock_us += 2000; return seq; } }
    *accepted = true; return seq + 1;
  }
};

// Reader pins a slot and leaves, the feeder runs to pin + 256 - offset, the reader returns.
// poll_during_absence false: the main loop did not reclaim while the alias arrived, so new_slot and get must cope alone.
static void alias_case(int offset, bool poll_during_absence) {
  AliasStream s(1500); uint32_t seq = 1, fs = 1; bool acc;
  for (; fs <= 11; fs++) {
    s.push(fs, 8); s.a.poll();
    while ((int32_t)(fs - seq) >= 0) { seq = s.reader(seq, &acc); s.a.poll(); }
    clock_us += 16667;
  }
  uint32_t pin = seq - 1, last = pin + 256 - (uint32_t)offset;
  for (; fs <= last; fs++) { s.push(fs, 8); clock_us += 100000; if (poll_during_absence) s.a.poll(); }
  uint32_t accepted = 0, t0 = clock_us, next_feed = clock_us, last_acc = 0;
  while (clock_us - t0 < 10000000u) {                    // 10 s simulated: feeder 60/s, reader follows
    if ((int32_t)(clock_us - next_feed) >= 0) { s.push(fs, 8); fs++; next_feed += 16667; }
    uint32_t before = clock_us;
    seq = s.reader(seq, &acc); s.a.poll();
    if (acc) { accepted++; last_acc = seq - 1; }
    if (clock_us == before) clock_us += 1000;
    require(s.a.free_sectors() >= 0, "pool state");
  }
  s.storage();
  if (accepted < 550 || (int32_t)(fs - 1 - last_acc) > 16) {
    std::fprintf(stderr, "FAIL window alias: offset %d poll %d: accepted %u of ~600, last accepted %u, feeder %u, reader asks %u\n",
                 offset, poll_during_absence, accepted, last_acc, fs - 1, seq);
    std::exit(1);
  }
}

static void window_alias_never_freezes() {
  for (int poll_abs = 1; poll_abs >= 0; poll_abs--)
    for (int offset : {-2, -1, 0, 1, 2, 3, 5, 8, 16, 40, 100}) alias_case(offset, poll_abs != 0);
}

// A resident slot answers only for its own superframe: an old alias never pins, and never answers for the new window.
static void stale_alias_never_answers() {
  AliasStream s(64); s.push(5, 2);
  s.a.read_begin(lba(5)); require(s.get(5) == AVS_READY && s.find(5)->served, "pin seq 5");
  for (uint32_t q = 6; q <= 5 + 200; q++) { s.push(q, 1); clock_us += 100; } // no poll: reclaim has not run
  uint32_t pinned_alias = 5 + 256;
  require(s.find(5) != nullptr, "stale slot still in the table before reclaim");
  require(s.synth_flags(pinned_alias) & SFF_SYNTH, "window of a stale slot answers synthetically, never with old data");
  for (uint32_t q = 206; q <= 261; q++) { s.push(q, 1); clock_us += 100; }
  const AvsSlot *n = s.find(261);
  require(n != nullptr && !s.find(5) && n->have == 1, "newer alias BEGIN replaces a stale pin");
  s.a.read_begin(lba(261)); require(s.get(261) == AVS_READY && s.find(261)->served, "new window serves new data");
  s.storage();
}

// ---- review finding 4: a sender that died inside a WIRE message must not delay the next one's link.
static void parser_reset_after_partial_message() {
  for (int cut : {5, 16 + 6, 16 + 12, 16 + 12 + 100, 16 + 12 + 528 + 10, 16 + 12 + 528 * 3 - 1}) {
    Fixture f(8);
    f.begin(1, 4);
    std::vector<uint8_t> m(16 + 12 + 4 * 528);
    put32(m.data(), TL_MAGIC); put16(m.data() + 4, TL_WIRE); put32(m.data() + 8, 12 + 4 * 528);
    put32(m.data() + 16, 1); put16(m.data() + 20, 0); put16(m.data() + 22, 4); put32(m.data() + 24, 4);
    for (int i = 0; i < 4; i++) {
      uint8_t b[512]; uint32_t w[WIRE_MAX_WORDS]; data(b, 1, i); wire4(b, 512, w);
      std::memcpy(m.data() + 28 + i * 528, w, 528);
    }
    f.a.feed(m.data(), cut, nullptr);
    f.a.reset_link();
    require(f.a.free_sectors() == 8 && f.a.used_slots() == 0, "reset gives back the sector being copied");
    uint8_t cfg[16] = {}; put16(cfg + 6, 400); cfg[8] = 20;
    f.message(TL_CONFIG, cfg, 16);
    require(f.a.link_active && f.a.cfg.stale_ms == 400, "first CONFIG after reset is parsed");
    f.send(2, 4); f.exact(2, 3); f.storage();
  }
}

// ---- keep one served slot, not two: everything a reader can do next finds its data.
static void served_slot_kept_until_one_newer() {
  Fixture f(32);
  f.send(1, 4); f.send(2, 4); f.send(3, 4); f.send(4, 1);
  f.exact(1); f.a.poll(); require(f.find(1), "served, nothing newer: stays");
  f.exact(1, 1); f.a.read_begin(0x40100); f.a.poll(); f.exact(1, 2); // split read and retry across a non-window command
  f.exact(2); f.a.poll(); require(!f.find(1) && f.find(2), "newer served: previous goes, new is pinned");
  f.exact(2); f.exact(2, 3); f.a.poll();                             // header retry, second read of the same superframe
  f.a.read_begin(lba(3)); require(f.get(3) == AVS_READY, "next superframe");
  require(f.get(4) == AVS_READY && !f.find(4)->served, "speculative queue past the window");
  f.a.poll(); require(!f.find(2) && f.find(3) && f.find(4), "prefetch does not count as served");
  f.a.read_begin(lba(9)); require(f.get(9) == AVS_HOLD, "future window: hold");
  clock_us += 60000; require(f.get(9) == AVS_READY, "future window: synthetic");
  f.a.poll(); f.exact(3, 3); f.storage();                            // a future read keeps the pin
}

// ---- the pool at 155 and 170 sectors: sender with credits and a window of 8, reader at 100 us per sector, stuck reads of 122 ms.
// Nothing evicted, nothing expired, every superframe read in order, reader stays within reach of the feeder.
static void pool_rate(int n, int pool, uint32_t *accepted_out) {
  AliasStream s(pool);
  s.a.cfg.stale_ms = 400;                               // what live avlink sends in CONFIG
  uint32_t fs = 1, seq = 1, acc = 0, t0 = clock_us, next_feed = clock_us, next_stall = clock_us + 1000000;
  uint32_t read_due = clock_us;
  while (clock_us - t0 < 5000000u) {
    if ((int32_t)(clock_us - next_feed) >= 0) {
      if (s.a.free_sectors() >= n && s.a.used_slots() < AVS_NSLOT && (int32_t)(fs - seq) <= 8) { s.push(fs++, n); next_feed += 16667; }
      else next_feed = clock_us + 1000;
      if ((int32_t)(clock_us - next_feed) > 100000) next_feed = clock_us;
      s.a.poll();
    }
    if ((int32_t)(clock_us - read_due) >= 0 && (int32_t)(fs - seq) > 0) {
      if ((int32_t)(clock_us - next_stall) >= 0) { read_due = clock_us + 122000; next_stall += 3000000; continue; }
      uint32_t before = seq; bool ok;
      uint32_t c = clock_us;
      seq = s.reader(seq, &ok); s.a.poll();
      require(ok && seq == before + 1, "every superframe read in order");
      for (int i = 1; i < n; i += 37) require(s.get(before, i) == AVS_READY, "whole superframe resident while read");
      acc++;
      read_due = c + 420 + (uint32_t)n * 100;
    }
    clock_us += 100;
  }
  require(s.a.st.evicted == 0 && s.a.st.expired == 0 && s.a.st.rx_errors == 0, "no eviction, expiry or dropped sector at this rate");
  s.storage();
  *accepted_out = acc;
}
static void pool_supports_155_and_170() {
  uint32_t a155, a170;
  pool_rate(155, AVS_POOL_OCRAM + AVS_POOL_DTCM, &a155);
  pool_rate(170, AVS_POOL_OCRAM + AVS_POOL_DTCM, &a170);
  require(a155 >= 280, "155 sectors: 60 superframes/s less the stuck reads, 100 us per sector");
  require(a170 >= 260, "170 sectors: as fast as the reader goes");
  std::printf("pool %d: 155 sectors %u superframes in 5 s, 170 sectors %u\n", AVS_POOL_OCRAM + AVS_POOL_DTCM, a155, a170);
}

// ---- whole-superframe hold (stuck_reads.md): off = cut-through as always; on = hold before sector 0 only, with the same bound.
static void set_whole(Fixture &f, uint8_t v) { uint8_t c[16] = {}; c[9] = v; f.message(TL_CONFIG, c, 16); }
static uint32_t header_flags_now(Fixture &f, uint32_t seq, int off) {
  uint32_t w[WIRE_MAX_WORDS]; uint16_t n = 0; uint8_t h[512];
  require(f.a.get(lba(seq, off), 1, w, &n) == AVS_READY, "ready");
  avs_unwire(w, 1, h);
  return rd32be(h + 12);
}
static void whole_superframe_hold() {
  { // default off, and CONFIG byte 0 leaves it: cut-through serves sector 0, marks served, holds mid-read
    Fixture f(16);
    require(!f.a.cfg.whole_sf, "off by default");
    set_whole(f, 0); require(!f.a.cfg.whole_sf, "0 leaves it");
    f.send(3, 3, 1); f.a.read_begin(lba(3));
    require(f.get(3, 0) == AVS_READY && f.find(3)->served && f.a.st.holds == 0, "off: sector 0 of a partial superframe goes at once");
    require(f.get(3, 1) == AVS_HOLD && f.a.st.holds == 1, "off: mid-read hold");
    clock_us += 31000; require(f.get(3, 1) == AVS_READY, "off: bound");
    require(f.a.st.hold_timeouts == 1 && f.a.st.notready == 1, "off: counted");
  }
  { // on: arrival during the hold
    Fixture f(16); set_whole(f, 2); require(f.a.cfg.whole_sf, "2 sets it");
    f.send(5, 3, 1); f.a.read_begin(lba(5));
    require(f.get(5, 0) == AVS_HOLD && !f.find(5)->served && f.a.st.holds == 1, "on: holds before sector 0, not served");
    clock_us += 29000; require(f.get(5, 0) == AVS_HOLD && f.a.st.holds == 1, "on: within the bound, one hold");
    f.wire(5, 1); require(f.get(5, 0) == AVS_HOLD, "on: still a sector short");
    f.wire(5, 2); f.end(5);
    require(f.get(5, 0) == AVS_READY && f.find(5)->served, "on: whole superframe: sector 0 goes, served");
    require(f.get(5, 1) == AVS_READY && f.get(5, 2) == AVS_READY && f.a.st.holds == 1 && !f.a.st.hold_timeouts, "on: no mid-read hold");
    f.send(6, 2); f.a.read_begin(lba(6));
    require(f.get(6, 0) == AVS_READY && f.a.st.holds == 1, "on: complete superframe: no hold at all");
    f.storage();
  }
  { // on: the tail never arrives. Bound, NOT_READY as today, not served, each new command bound again
    Fixture f(16); set_whole(f, 2);
    f.send(7, 3, 2); f.a.read_begin(lba(7));
    require(f.get(7, 0) == AVS_HOLD, "tail: hold");
    clock_us += 29999; require(f.get(7, 0) == AVS_HOLD, "tail: just inside hold_cut_ms");
    clock_us += 2; uint32_t fl = header_flags_now(f, 7, 0);
    require((fl & SFF_SYNTH) && (fl & SFF_NOTREADY), "tail: NOT_READY in sector 0 at the bound");
    require(f.a.st.hold_timeouts == 1 && f.a.st.notready == 1 && !f.find(7)->served, "tail: counted, not served");
    require(f.get(7, 1) == AVS_READY && f.get(7, 2) == AVS_READY, "tail: rest of that command does not hold");
    f.a.read_begin(lba(7)); require(f.get(7, 0) == AVS_HOLD, "tail: retry holds again");
    clock_us += 30001; require(f.get(7, 0) == AVS_READY && f.a.st.hold_timeouts == 2, "tail: retry bound");
    // the bound is capped, as a cut-through hold
    uint8_t c[16] = {}; put16(c, 200); f.message(TL_CONFIG, c, 16);
    f.a.read_begin(lba(7)); require(f.get(7, 0) == AVS_HOLD, "cap: hold");
    clock_us += 49000; require(f.get(7, 0) == AVS_HOLD, "cap: inside 50 ms");
    clock_us += 1001; require(f.get(7, 0) == AVS_READY, "cap: AVS_MAX_HOLD_MS");
    // late tail: delivered whole, and the slot was never lost
    f.wire(7, 2); f.end(7); f.exact(7, 0); require(f.find(7)->served, "late tail: served"); f.storage();
    // off again at run time
    set_whole(f, 1); require(!f.a.cfg.whole_sf, "1 clears it");
    f.send(8, 2, 1); f.a.read_begin(lba(8)); require(f.get(8, 0) == AVS_READY && f.find(8)->served, "off again: cut-through");
  }
  for (int on = 0; on <= 1; on++) { // a superframe whose tail never comes must not stop the reader, on or off: it moves on when newer ones complete
    AliasStream s(1500); s.a.cfg.stale_ms = 400; set_whole(s, on ? 2 : 1);
    uint32_t seq = 1, fs = 1, acc = 0, skipped = 0, next = clock_us, t0 = clock_us; bool ok;
    while (clock_us - t0 < 3000000u) {
      if ((int32_t)(clock_us - next) >= 0) {
        if (fs == 30) { s.begin(fs, 8); for (int i = 0; i < 5; i++) s.header_wire(fs, i, 8); fs++; }   // 3 sectors and the END never come
        else s.push(fs++, 8);
        next += 16667; s.a.poll();
      }
      uint32_t before = seq, c0 = clock_us;
      seq = s.reader(seq, &ok); s.a.poll();
      if (ok) acc++;
      if (seq != before && !ok) skipped++;
      if (clock_us == c0) clock_us += 500;
    }
    if (acc < 150 || (int32_t)(fs - seq) > 6) { std::fprintf(stderr, "FAIL tail never comes, whole_sf %d: accepted %u, reader %u, feeder %u\n", on, acc, seq, fs); std::exit(1); }
    require(skipped >= 1 || !on, "whole_sf: reader jumped past the incomplete superframe");
    s.storage();
  }
}

int main() {
  pool_split_retry_progress(); collision_and_duplicates(); table_pressure_and_race();
  pool_retirement_race(); reclaim_race_and_cutthrough(); future_read_does_not_release();
  command_anchor_prefetch_and_abort();
  require(irq_count == 4, "all ISR/main interleavings exercised");
  stale_alias_never_answers();
  window_alias_never_freezes();
  parser_reset_after_partial_message();
  served_slot_kept_until_one_newer();
  pool_supports_155_and_170();
  whole_superframe_hold();
  char msg[512]; require(avs_selftest(msg, sizeof msg) == 0, msg);
  std::puts("PASS AvStream pool/table pressure, split/retry, wrap/resend, duplicate immutability, progress, FLUSH/reset,");
  std::puts("     command-anchored prefetch/abort, cut-through, synthetic reads, pool accounting, four ISR/main interleavings, existing avs_selftest");
  return 0;
}
