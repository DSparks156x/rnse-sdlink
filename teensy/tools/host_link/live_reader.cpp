// live_reader -- the Teensy's real AvStream on a pty, read by a model of the RNS-E reader, for live (not --k) tests with the
// real avlink: real tick, real credits, a reader that leaves and returns. Driven by pi/avlink/test/test_live.py.
//   Reader: rnse_proto/src/sdstream.c reader_main branch logic, retyped, with a time model of 0.42 ms per read + us_per_sector.
//   Decoder model: need_key as in decode_slot (start, lost -> wait for a key, DISCONT clears it). Each test frame carries a
//   frame number, so a delta decoded on the wrong reference is counted (chain_breaks): after a discard this must stay 0.
//   Measures per superframe: age of the newest complete video frame and of the audio. No USB, no SD bus, no decoder time.
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <algorithm>
#include <vector>
#include "../../sdemu/avstream.h"
#include "../../sdemu/config.h"
#include "../../sdemu/sdproto.h"

static int mfd;
static uint64_t mono() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (uint64_t)t.tv_sec * 1000000u + (uint64_t)t.tv_nsec / 1000; }
static uint64_t t0;
static uint32_t now32() { return (uint32_t)(mono() - t0 + 5000000u); }
static void tx(const uint8_t *p, int n) {
  int tries = 0;
  while (n > 0) {
    ssize_t w = write(mfd, p, (size_t)n);
    if (w > 0) { p += w; n -= (int)w; continue; }
    if (++tries > 20) return;
    pollfd q = {mfd, POLLOUT, 0}; poll(&q, 1, 5);
  }
}
static void con(uint8_t) {}
#ifndef POOL_A
#define POOL_A AVS_POOL_OCRAM
#define POOL_B AVS_POOL_DTCM
#endif
static AvsSector A[POOL_A], B[POOL_B];
static void service(int ms) {
  pollfd q = {mfd, POLLIN, 0};
  if (poll(&q, 1, ms) > 0 && (q.revents & POLLIN)) {
    static uint8_t b[65536];
    for (;;) { ssize_t r = read(mfd, b, sizeof b); if (r <= 0) break; AVS.feed(b, (int)r, con); }
  }
  AVS.poll();
}
static void wait_us(uint64_t us) { uint64_t e = mono() + us; while (mono() < e) { service(0); uint64_t n = mono(); if (n < e && e - n > 1000) service(1); } }
static uint32_t be32(const uint8_t *p) { return (uint32_t)p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3]; }
static uint16_t be16(const uint8_t *p) { return (uint16_t)(p[0] << 8 | p[1]); }
static uint8_t rb[256 * 512];
static double us_per_sector = 100, us_per_read = 420, sleep_us = 0;
static unsigned max_sf_sectors = 170;
static unsigned long mid_holds;                         // start-bit holds inside a read: sector > 0 (one count per sector held)
static void rd(uint32_t lba, int n, uint8_t *d) {
  AVS.read_begin(lba);
  uint64_t st = mono();
  for (int i = 0; i < n; i++) {
    uint32_t w[WIRE_MAX_WORDS]; uint16_t k = 0; int r;
    int held = 0;
    while ((r = AVS.get(lba + (uint32_t)i, 1, w, &k)) == AVS_HOLD) { if (i && !held) { mid_holds++; held = 1; } service(1); }
    avs_unwire(w, 1, d + 512 * i);
  }
  uint64_t need = (uint64_t)(us_per_read + us_per_sector * n), el = mono() - st;
  if (el < need) wait_us(need - el);
}
static const uint8_t *tlv(const uint8_t *p, unsigned n, uint8_t type) {
  for (unsigned i = 0; i + 2 <= n;) { if (p[i] == type) return p + i + 2; i += 2u + p[i + 1]; }
  return nullptr;
}
struct Sample { double e, v; };
static double pct(std::vector<double> v, double q) { if (v.empty()) return 0; std::sort(v.begin(), v.end()); return v[(size_t)(q * (double)(v.size() - 1))]; }
static double mean(const std::vector<double> &v) { double s = 0; for (double x : v) s += x; return v.empty() ? 0 : s / (double)v.size(); }

int main(int c, char **v) {
  const char *pf = nullptr; double total = 20; uint32_t preset = 0; double settle = 1.0;
  std::vector<double> abs_at, abs_for;
  for (int i = 1; i + 1 < c; i += 2) {
    if (!strcmp(v[i], "--ptyfile")) pf = v[i + 1];
    else if (!strcmp(v[i], "--absent-at")) abs_at.push_back(atof(v[i + 1]));
    else if (!strcmp(v[i], "--absent-for")) abs_for.push_back(atof(v[i + 1]));
    else if (!strcmp(v[i], "--total")) total = atof(v[i + 1]);
    else if (!strcmp(v[i], "--preset")) preset = (uint32_t)atoi(v[i + 1]);
    else if (!strcmp(v[i], "--us-per-sector")) us_per_sector = atof(v[i + 1]);
    else if (!strcmp(v[i], "--sleep-us")) sleep_us = atof(v[i + 1]);
    else if (!strcmp(v[i], "--max-sf")) max_sf_sectors = (unsigned)atoi(v[i + 1]);
    else if (!strcmp(v[i], "--settle")) settle = atof(v[i + 1]);
  }
  t0 = mono(); sdp_init();
  mfd = posix_openpt(O_RDWR | O_NOCTTY); grantpt(mfd); unlockpt(mfd);
  const char *sn = ptsname(mfd);
  int sfd = open(sn, O_RDWR | O_NOCTTY);
  termios t; tcgetattr(sfd, &t); cfmakeraw(&t); tcsetattr(sfd, TCSANOW, &t);
  fcntl(mfd, F_SETFL, fcntl(mfd, F_GETFL) | O_NONBLOCK);
  AvsSector *bk[2] = {A, B}; uint16_t bn[2] = {POOL_A, POOL_B};
  AVS.init(bk, bn, 2, now32, tx);
  AVS.st.rx_sectors = preset; AVS.st.begun = preset;   // lifetime counters as after an earlier chain run
  FILE *f = fopen(pf, "w"); fprintf(f, "%s\n", sn); fclose(f);

  uint64_t T = mono(), ret_t = 0;
  uint32_t seq = 0, hint = 30, seq_max = 0;
  size_t next_abs = 0;
  int bin = -1; unsigned nsf = 0, nexp = 0, nnr = 0, nvid = 0; double vmax = 0, vsum = 0, amax = 0, asum = 0; unsigned na = 0; uint64_t maxgap = 0, last_sf = 0;
  std::vector<Sample> vs, as;
  int need_key = 1, have_no = 0, lost = 0; uint32_t last_no = 0;
  unsigned long chain_breaks = 0, vdrop = 0, frames_shown = 0, keys_shown = 0, sf_total = 0, retries = 0, hdr_bad = 0;
  double first_sf = -1, first_vid = -1, first_key = -1;  // ms after the last return (or after start, without an absence)
  double f_ts = 0; uint32_t f_no = 0, f_key = 0; int f_on = 0;
  auto flush = [&](int b) {
    printf("%3d %4u v_mean %6.0f v_max %6.0f a_mean %6.0f a_max %6.0f shown %3u exp %3u nr %3u gap %5.0f seq %6u free %5d slots %2d\n", b, nsf, nvid ? vsum / nvid : 0, vmax, na ? asum / na : 0, amax, nvid, nexp, nnr, (double)maxgap / 1000.0, seq, AVS.free_sectors(), AVS_NSLOT - AVS.used_slots());
    fflush(stdout);
    nsf = nexp = nnr = nvid = na = 0; vmax = vsum = amax = asum = 0; maxgap = 0;
  };
  for (;;) {
    double e = (double)(mono() - T) / 1e6;
    if (e > total) break;
    int b = (int)e;
    if (b != bin) { if (bin >= 0) flush(bin); bin = b; }
    if (next_abs < abs_at.size() && e >= abs_at[next_abs]) {
      uint64_t en = mono() + (uint64_t)(abs_for[next_abs] * 1e6);
      while (mono() < en) { service(1); int b2 = (int)((double)(mono() - T) / 1e6); if (b2 != bin) { flush(bin); bin = b2; } }
      next_abs++; ret_t = mono(); last_sf = 0;
      first_sf = first_vid = first_key = -1;
      printf("--- reader back at %.3f\n", (double)(ret_t - T) / 1e6);
    }
    service(0);
    uint32_t lba = 0x10000u + (seq & 255) * 256u;
    uint64_t c0 = mono();
    rd(lba, (int)hint, rb);
    const uint8_t *h = rb;
    if (be32(h) != 0x52534156u || be16(h + 6) != 64 || crc32z(h + 64, 448, crc32z(h, 60)) != be32(h + 60)) { hdr_bad++; wait_us(2000); continue; }
    uint32_t hf = be32(h + 12), ss = be32(h + 8), tot = be16(h + 40);
    if (hf & SFF_EXPIRED) { nexp++; seq = be32(h + 64); lost = 1; continue; }
    if (hf & (SFF_IDLE | SFF_NOTREADY)) {
      nnr++; retries++;
      uint32_t nw = (hf & SFF_NOTREADY) ? be32(h + 64) : ss;
      if (nw && ((int32_t)(nw - seq) > 4 || (int32_t)(seq - nw) > 64)) { seq = nw; lost = 1; continue; }
      wait_us(2000); continue;
    }
    if (ss != seq) { if ((int32_t)(ss - seq) > 0) { seq = ss; lost = 1; } else { retries++; wait_us(2000); continue; } }
    if (tot < 1 || tot > max_sf_sectors) { seq++; lost = 1; continue; }
    if (tot > hint) rd(lba + hint, (int)(tot - hint), rb + hint * 512);
    uint16_t nh = be16(h + 42); hint = nh ? nh : tot;
    uint64_t n = mono();
    double es = (double)(n - T) / 1e6, since = ret_t ? (double)(n - ret_t) / 1e3 : (double)(n - T) / 1e3;
    (void)c0;
    nsf++; sf_total++; seq_max = ss;
    if (first_sf < 0) first_sf = since;
    if (last_sf && n - last_sf > maxgap) maxgap = n - last_sf;
    last_sf = n;
    uint32_t ab = be32(h + 32), vb = be32(h + 24); uint16_t asec = be16(h + 36), vsec = be16(h + 38);
    if (ab >= 4) {
      const uint8_t *a = rb + asec * 512;
      uint32_t idx = (uint32_t)be16(a) | ((uint32_t)be16(a + 2) << 16);
      if (idx) { uint32_t nowi = (uint32_t)(n * 48 / 1000); double l = (double)(int32_t)(nowi - idx) / 48.0; asum += l; na++; if (l > amax) amax = l; as.push_back({es, l}); }
    }
    // decoder model
    if (lost) { need_key = 1; f_on = 0; lost = 0; }
    if (hf & SFF_DISCONT) need_key = 0;
    int done = 0;
    if (vb) {
      const uint8_t *p = rb + vsec * 512;
      unsigned cn = be16(h + 52) > 448 ? 448 : be16(h + 52);
      const uint8_t *fr = tlv(h + 64, cn, 0x11);
      if (fr) {
        uint32_t fx = be32(fr), off = be32(fr + 4);
        if (off == 0) { f_on = 1; f_ts = (double)(((uint64_t)be32(p) << 32) | be32(p + 4)); f_no = be32(p + 8); f_key = be32(p + 12); }
        if (f_on && off + vb == fx) { f_on = 0; done = 1; }
      } else if (vb >= 16) {
        f_on = 0; f_ts = (double)(((uint64_t)be32(p) << 32) | be32(p + 4)); f_no = be32(p + 8); f_key = be32(p + 12); done = 1;
      }
    }
    if (done) {
      if (need_key && !f_key) vdrop++;
      else {
        if (!f_key && have_no && f_no != last_no + 1) chain_breaks++;
        last_no = f_no; have_no = 1; need_key = 0;
        double l = ((double)n - f_ts) / 1000.0;
        vsum += l; nvid++; if (l > vmax) vmax = l;
        vs.push_back({es, l});
        frames_shown++;
        if (first_vid < 0) first_vid = since;
        if (f_key) { keys_shown++; if (first_key < 0) first_key = since; }
      }
    }
    seq++;
    if (sleep_us > 0) wait_us((uint64_t)sleep_us);
  }
  flush(bin);
  const AvsStatus &s = AVS.st;
  double from = ret_t ? (double)(ret_t - T) / 1e6 + settle : settle + 2.0;
  std::vector<double> vw, aw; for (auto &x : vs) if (x.e >= from) vw.push_back(x.v); for (auto &x : as) if (x.e >= from) aw.push_back(x.v);
  // hold on the screen: the longest stretch without a new picture after `from`
  double maxhold = 0, prev = from; for (auto &x : vs) if (x.e >= from) { if (x.e - prev > maxhold) maxhold = x.e - prev; prev = x.e; }
  printf("JSON {\"v_mean\":%.1f,\"v_p95\":%.1f,\"v_max\":%.1f,\"a_mean\":%.1f,\"a_p95\":%.1f,\"a_max\":%.1f,\"nv\":%zu,\"na\":%zu,"
         "\"first_sf_ms\":%.1f,\"first_vid_ms\":%.1f,\"first_key_ms\":%.1f,\"chain_breaks\":%lu,\"vdrop\":%lu,\"frames\":%lu,\"keys\":%lu,\"sf\":%lu,\"seq_max\":%u,"
         "\"mid_holds\":%lu,\"retries\":%lu,\"hdr_bad\":%lu,\"maxhold_ms\":%.0f,\"evicted\":%u,\"expired\":%u,\"notready\":%u,\"idle\":%u,\"rx_errors\":%u,\"pool\":%d,\"free\":%d}\n",
         mean(vw), pct(vw, .95), pct(vw, 1), mean(aw), pct(aw, .95), pct(aw, 1), vw.size(), aw.size(),
         first_sf, first_vid, first_key, chain_breaks, vdrop, frames_shown, keys_shown, sf_total, seq_max,
         mid_holds, retries, hdr_bad, maxhold * 1000, s.evicted, s.expired, s.notready, s.idle, s.rx_errors, (int)s.pool_sectors, AVS.free_sectors());
  return 0;
}
