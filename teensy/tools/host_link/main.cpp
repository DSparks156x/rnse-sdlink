// host_link -- the Teensy's live-stream path on a Linux PC (WSL), for the end-to-end test with pi/avlink.
//   pty master  <->  avlink --dev <pty slave>   (the pty stands in for /dev/ttyACM0)
//   bytes -> AvStream::feed() (sdemu/avstream.cpp, the firmware's own file) ; AvStream::poll() -> STATUS/SERVED
//   simulated RNS-E, mode A recipe (SPEC 2.A.4): read(slot_lba(seq), hint) through AvStream::get() exactly as the
//   firmware read engine calls it (HOLD = start bit held: try again later), second read if total_sec > hint,
//   NOT_READY/IDLE -> retry after 2 ms, EXPIRED -> jump to the seq in bytes 64..67 (like rnse_proto sdstream.c).
//   Every served image is checked: decode it, re-encode with sdproto wire4/wire1, must be word-identical
//   (start word, data, all CRC16 lines, end word, pad).
// Usage: host_link --ptyfile F --k K --out served.rsav [--bus 4|1] [--start-delay ms] [--stale-ms ms] [--timeout s]
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include "../../sdemu/avstream.h"
#include "../../sdemu/sdproto.h"

static int mfd = -1;
static uint64_t t0_us;
static uint64_t mono_us(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (uint64_t)t.tv_sec * 1000000u + (uint64_t)t.tv_nsec / 1000u;
}
static uint32_t now32(void) { return (uint32_t)(mono_us() - t0_us + 5000000u); }
static unsigned long tx_dropped;
static void tx(const uint8_t *p, int n) {
  int tries = 0;
  while (n > 0) {
    ssize_t w = write(mfd, p, (size_t)n);
    if (w > 0) { p += w; n -= (int)w; continue; }
    if (w < 0 && errno != EAGAIN && errno != EINTR) { tx_dropped++; return; }
    if (++tries > 20) { tx_dropped++; return; }        // nobody reads the slave any more (avlink exited)
    struct pollfd q = {mfd, POLLOUT, 0};
    poll(&q, 1, 5);
  }
}
static unsigned long console_bytes;
static void console(uint8_t c) { (void)c; console_bytes++; }

#include "../../sdemu/config.h"
#define POOL_A AVS_POOL_OCRAM                         // same pool as the firmware
#define POOL_B AVS_POOL_DTCM
static AvsSector poolA[POOL_A], poolB[POOL_B];

static void service(int wait_ms) {
  struct pollfd q = {mfd, POLLIN, 0};
  if (poll(&q, 1, wait_ms) > 0 && (q.revents & POLLIN)) {
    static uint8_t buf[65536];
    for (;;) {
      ssize_t r = read(mfd, buf, sizeof buf);
      if (r <= 0) break;
      AVS.feed(buf, (int)r, console);
    }
  }
  AVS.poll();
}

static uint32_t be32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
static uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

static int bus4 = 1;
static unsigned long wire_bad, holds_seen, reads, zero_bad, hdr_bad, notready_seen, idle_seen, expired_seen, retries,
    seq_mism;
static uint8_t rbuf[256 * 512];

// one CMD18: count sectors from lba into dst; services the pty while a sector is held
static void read_cmd(uint32_t lba, int count, uint8_t *dst) {
  AVS.read_begin(lba);
  reads++;
  for (int i = 0; i < count; i++) {
    uint32_t w[WIRE_MAX_WORDS], chk[WIRE_MAX_WORDS];
    uint16_t n = 0;
    int r;
    while ((r = AVS.get(lba + (uint32_t)i, bus4, w, &n)) == AVS_HOLD) { holds_seen++; service(1); }
    avs_unwire(w, bus4, dst + 512 * i);
    int m = bus4 ? wire4(dst + 512 * i, 512, chk) : wire1(dst + 512 * i, 512, chk);
    if (r != AVS_READY || m != n || memcmp(chk, w, (size_t)n * 4)) wire_bad++;
  }
}

int main(int argc, char **argv) {
  const char *ptyfile = nullptr, *outp = nullptr;
  int k = 0, start_delay = 0, stale = 0, timeout_s = 120;
  for (int i = 1; i + 1 < argc; i += 2) {
    if (!strcmp(argv[i], "--ptyfile")) ptyfile = argv[i + 1];
    else if (!strcmp(argv[i], "--k")) k = atoi(argv[i + 1]);
    else if (!strcmp(argv[i], "--out")) outp = argv[i + 1];
    else if (!strcmp(argv[i], "--bus")) bus4 = atoi(argv[i + 1]) == 4;
    else if (!strcmp(argv[i], "--start-delay")) start_delay = atoi(argv[i + 1]);
    else if (!strcmp(argv[i], "--stale-ms")) stale = atoi(argv[i + 1]);
    else if (!strcmp(argv[i], "--timeout")) timeout_s = atoi(argv[i + 1]);
  }
  if (!ptyfile || !outp || k < 1) { fprintf(stderr, "usage: host_link --ptyfile F --k K --out F [--bus 4|1] ...\n"); return 2; }
  t0_us = mono_us();
  sdp_init();
  mfd = posix_openpt(O_RDWR | O_NOCTTY);
  if (mfd < 0 || grantpt(mfd) || unlockpt(mfd)) { perror("pty"); return 1; }
  const char *sname = ptsname(mfd);
  int sfd = open(sname, O_RDWR | O_NOCTTY);          // keep the slave open: no EIO when avlink exits
  struct termios t;
  tcgetattr(sfd, &t); cfmakeraw(&t); tcsetattr(sfd, TCSANOW, &t);
  fcntl(mfd, F_SETFL, fcntl(mfd, F_GETFL) | O_NONBLOCK);
  AvsSector *banks[2] = {poolA, poolB};
  uint16_t bn[2] = {POOL_A, POOL_B};
  AVS.init(banks, bn, 2, now32, tx);
  if (stale) AVS.cfg.stale_ms = (uint16_t)stale;
  FILE *pf = fopen(ptyfile, "w");
  fprintf(pf, "%s\n", sname);
  fclose(pf);
  FILE *out = fopen(outp, "wb");
  uint64_t tstart = mono_us();
  while (mono_us() - tstart < (uint64_t)start_delay * 1000u) service(1);   // RNS-E not reading yet
  // ---- mode A reader
  uint32_t seq = 0, hint = 178, done = 0;
  int tries = 0, rc = 0;
  unsigned long served = 0;
  while (!done) {
    if (mono_us() - tstart > (uint64_t)timeout_s * 1000000u) { fprintf(stderr, "host_link: TIMEOUT at seq %u\n", seq); rc = 3; break; }
    service(0);
    uint32_t base = AVS_WIN_LBA + (seq & 0xFF) * 256u;
    read_cmd(base, (int)hint, rbuf);
    const uint8_t *h = rbuf;
    int ok = be32(h) == 0x52534156u && be16(h + 6) == 64 && crc32z(h + 64, 448, crc32z(h, 60)) == be32(h + 60);
    if (!ok) { hdr_bad++; if (++tries >= 3) { seq++; tries = 0; } continue; }
    uint32_t flags = be32(h + 12);
    if (flags & SFF_EXPIRED) { expired_seen++; seq = be32(h + 64); tries = 0; continue; }
    if (flags & (SFF_IDLE | SFF_NOTREADY)) {
      if (flags & SFF_IDLE) idle_seen++; else notready_seen++;
      retries++;
      uint64_t tw = mono_us();
      while (mono_us() - tw < 2000) service(1);       // sdstream: retry after 2 ms
      continue;
    }
    if (be32(h + 8) != seq) { seq_mism++; seq = be32(h + 8); }
    uint32_t total = be16(h + 40);
    if (total > hint) read_cmd(base + hint, (int)(total - hint), rbuf + 512 * hint);
    for (uint32_t i = total; i < hint; i++)          // sectors past total_sec: zeros (like the sdgen card image)
      for (int j = 0; j < 512; j++) if (rbuf[512 * i + j]) { zero_bad++; break; }
    fwrite(rbuf, 512, total, out);
    served++;
    if (seq == (uint32_t)k - 1) done = 1;
    uint16_t nh = be16(h + 42);
    hint = nh ? nh : 178;
    if (hint > 256) hint = 256;
    seq++; tries = 0;
  }
  fclose(out);
  for (int i = 0; i < 20; i++) service(1);           // let the last STATUS out
  const AvsStatus &s = AVS.st;
  printf("HOSTLINK rc=%d served_sf=%lu reads=%lu wire_crc_bad=%lu hdr_bad=%lu seq_mism=%lu zero_bad=%lu "
         "notready_seen=%lu idle_seen=%lu expired_seen=%lu retries=%lu holds=%lu teensy_holds=%u hold_timeouts=%u "
         "converted=%u evicted=%u rx_sectors=%u rx_errors=%u rx_msgs=%u console_bytes=%lu tx_dropped=%lu bus=%d\n",
         rc, served, reads, wire_bad, hdr_bad, seq_mism, zero_bad, notready_seen, idle_seen, expired_seen, retries,
         holds_seen, s.holds, s.hold_timeouts, s.converted, s.evicted, s.rx_sectors, s.rx_errors, AVS.rx_msgs,
         console_bytes, tx_dropped, bus4 ? 4 : 1);
  if (wire_bad || zero_bad) rc = rc ? rc : 4;
  close(sfd);
  close(mfd);
  return rc;
}
