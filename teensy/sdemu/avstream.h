// avstream.h -- live superframe store for card mode (link/SPEC.md 2.A.3 window, 3.4 synthetic sectors, 5 USB).
// Hardware-independent (compiles on the Teensy and natively on a PC for tools/host_link and the self-test):
//   USB RX bytes -> TLNK parser -> wire-sector pool (528 B images from the Pi, stored as received)
//   slot table by seq (16 resident superframes; LBA 0x10000 + (seq mod 256) * 256)
//   get(lba): the read engine's sector source for the window: pool image / hold / NOT_READY / EXPIRED / IDLE / zeros
//   poll(): slot reclaim (mode A: freed once a newer superframe was served; stale unread slots after stale_ms),
//           STATUS (credits) and SERVED messages to the Pi.
// Concurrency on the Teensy: get() runs in the read-engine interrupt (prio 64); feed()/poll() run in the main loop
// and change shared state only inside AVS_LOCK. get() never blocks: AVS_HOLD = "not yet, ask again".
// The resident superframe targeted by the last read command is pinned across CMD12/CMD17/18 and non-window
// reads, for split reads and retries. A command targeting another resident superframe transfers the pin.
// Speculative get() calls beyond the command's starting window cannot transfer it. Pressure/resends cannot discard it;
// FLUSH and reset_link deliberately can. At most one slot is pinned, with no timeout while the host is paused, until the
// stream is more than 128 superframes past it: then its window number means a newer superframe, so it stops answering and
// is released (slot_stale). A retry never needs that old: the reader treats it as expired and moves on.
// Mode A reads each superframe with a separate command. Cross-window get() returns zero filler without read/served
// side effects, so the engine's speculative queue cannot advance ownership. Cross-superframe CMD18 is unsupported.
// Must match ../../pi/avlink/tlink.h (message layout).
#pragma once
#include <stdint.h>
#include <stddef.h>

#define AVS_WIN_LBA     0x10000u
#define AVS_WIN_SECT    0x10000u     // 256 slots x 256 sectors
#define AVS_NSLOT       16
#define AVS_IMG_WORDS   132          // 4-bit image; 1-bit images (130 words) use the same pool entry
#define AVS_MAX_HOLD_MS 50           // SPEC 3.4 hard rule: a start-bit hold never exceeds 50 ms

#define TL_MAGIC 0x4B4E4C54u
enum {
  TL_SF_BEGIN = 0x01, TL_WIRE = 0x02, TL_SF_END = 0x03, TL_META = 0x04, TL_CONFIG = 0x05, TL_DOWNLINK = 0x06,
  TL_FLUSH = 0x07, TL_HOST_WRITE = 0x81, TL_UPLINK = 0x82, TL_STATUS = 0x83, TL_CMDLOG = 0x84, TL_SERVED = 0x85
};
enum { SFF_KEY = 1, SFF_REPEAT = 2, SFF_IDLE = 4, SFF_DISCONT = 8, SFF_ADISCONT = 16, SFF_SYNTH = 32,
       SFF_NOTREADY = 64, SFF_EXPIRED = 128, SFF_EOS = 256 };
enum { AVS_NOTWIN = 0, AVS_READY = 1, AVS_HOLD = 2 };

struct AvsSector { uint32_t w[AVS_IMG_WORDS]; };

struct AvsStatus {                   // = tl_status in pi/avlink/tlink.h (80 B, little-endian)
  uint16_t ver, size;
  uint32_t free_sectors, rx_sectors, free_slots, begun, newest_seq, served_seq, sectors_served;
  uint32_t holds, hold_timeouts, notready, expired, idle, evicted, converted, rx_errors, crc7_rejects;
  uint32_t write_crc_errors;
  uint8_t card_state, bus_width, mode, flags;
  uint32_t pool_sectors;
};

// whole_sf: 1 = hold only before sector 0, until the whole superframe has arrived, so a read has no mid-read holds. Off by default.
// Same bound as a cut-through hold (hold_cut_ms, at most AVS_MAX_HOLD_MS), then NOT_READY. Set by CONFIG or the console (cfg wholesf).
struct AvsConfig { uint16_t hold_cut_ms, hold_future_ms, silent_ms, stale_ms; uint8_t status_hz; uint8_t whole_sf; };

struct AvsSlot {
  uint8_t used, ended, served;
  uint32_t seq;
  uint16_t total, have;
  uint32_t t_begin, t_end, t_read;   // us
  uint16_t idx[256];                 // pool index per sector, 0xFFFF = not received
  uint32_t fmt1[8];                  // bit set = that sector's image is 1-bit (520 B)
};

class AvStream {
 public:
  typedef uint32_t (*now_fn)(void);                      // microseconds, wraps
  typedef void (*tx_fn)(const uint8_t *p, int n);        // write bytes to the Pi (main loop only)
  typedef void (*console_fn)(uint8_t c);                 // non-TLNK bytes (console text)

  // pool = nbanks arrays of sectors (e.g. OCRAM + DTCM); total <= 65534
  void init(AvsSector *const *banks, const uint16_t *bank_n, int nbanks, now_fn now, tx_fn tx);
  void feed(const uint8_t *p, int n, console_fn console);   // USB RX bytes
  void poll(void);                                          // main loop
  // read engine (IRQ): window sector -> wire image (+1 pad word). bus4 = current width. Returns AVS_*.
  int get(uint32_t lba, int bus4, uint32_t *out, uint16_t *nwords);
  void read_begin(uint32_t lba);    // command start LBA; resets hold state and anchors split-read/retry pin
  void info(uint8_t *p, int n);    // big-endian summary for the 0x40000 info sector (n >= 40)

  // card-side values reported in STATUS (set by the sketch)
  uint8_t card_state, bus_width, mode;
  uint32_t crc7_rejects;

  AvsConfig cfg;
  AvsStatus st;                    // live counters (st.free_* filled when sent)
  uint8_t link_active;             // set by the first TLNK message (until reset_link())
  uint32_t rx_msgs, last_rx_us;
  void reset_link(void);           // forget all slots (Pi gone / FLUSH all)
  int free_sectors(void) const { return free_n; }
  int used_slots(void) const;
  const AvsSlot *slot(int i) const { return &slots[i]; }

 private:
  // pool
  AvsSector *const *banks; const uint16_t *bank_n; int nbanks; int pool_n;
  uint16_t *free_list; int free_n;
  AvsSector *sec_ptr(uint16_t i) const;
  int alloc_sector(void);
  void free_sector(uint16_t i);
  // slots
  AvsSlot slots[AVS_NSLOT];
  volatile int read_slot;            // IRQ-owned retry pin; main tests it while holding AVS_LOCK
  uint8_t have_begun, have_complete, have_served;
  uint32_t newest_begun, newest_complete;
  int free_slot(int s, int force = 0, int evict = 0); // atomically unpublish, then release storage outside lock
  int find_slot(uint32_t seq) const;
  int find_slot_idx(uint32_t win_idx) const;   // resident slot with seq % 256 == win_idx
  int new_slot(uint32_t seq, uint16_t total);
  int slot_stale(uint32_t seq) const;          // > 128 superframes behind the newest: its window now means a newer one
  void reclaim(uint32_t now);
  // parser
  uint8_t ps, mpos;
  uint8_t hdr[16]; int hpos;
  uint16_t mtype; uint32_t mlen, mgot;
  uint8_t pbuf[64]; int ppos;
  uint8_t wsub[12]; int wpos;
  int wslot; uint32_t wseq; uint16_t widx, wcount, wdone; uint8_t wfmt1; int wbytes;
  int cur_sec; uint32_t cur_got; uint8_t *cur_dst; uint8_t cur_drop;
  void on_header(void);
  void on_small(void);
  void wire_byte_chunk(const uint8_t *p, int n);
  void sector_done(void);
  // hold / synthetic state (IRQ side)
  volatile uint32_t read_gen; volatile int read_win; uint32_t my_gen;
  uint32_t hold_lba, hold_t0; uint8_t holding;
  int sticky_win; uint8_t sticky_kind; uint32_t sticky_seq, sticky_arg;    // per-command synthetic answer
  uint8_t degraded;                                            // cut-through timed out in this command
  uint32_t syn_w[AVS_IMG_WORDS + 1]; uint16_t syn_n; uint8_t syn_kind, syn_bus4, syn_valid; uint32_t syn_seq, syn_arg;
  uint32_t zero_w4[AVS_IMG_WORDS + 1], zero_w1[AVS_IMG_WORDS + 1];
  uint16_t zero_n4, zero_n1;
  void synth(uint8_t kind, uint32_t seq, uint32_t arg, int bus4, uint32_t *out, uint16_t *n);
  int copy_image(const AvsSlot &s, int off, int bus4, uint32_t *out, uint16_t *n);
  // served notifications (IRQ -> main loop)
  volatile uint32_t srv_seq[16], srv_t[16]; volatile uint8_t srv_head, srv_tail;
  // tx
  now_fn now; tx_fn tx;
  uint32_t last_status_us, last_sent_free, last_sent_slots;
  void send(uint16_t type, const void *pl, uint32_t len);
  void send_status(uint32_t t);
  uint32_t tx_seq;
};

extern AvStream AVS;                 // the firmware's instance (fxsd.cpp / sdemu.ino)
int avs_selftest(char *msg, int ml);   // unit tests on a private instance; 0 = pass
void avs_unwire(const uint32_t *w, int fmt4, uint8_t *out);   // wire image -> 512 data bytes (no CRC check)
