// sdemu.ino -- Teensy 4.1 SD-card emulator, bring-up firmware for the RNS-E AV-over-SD link.
// Spec: ../../link/SPEC.md (+ SPEC_VERIFY.md). Wiring, build and checklist: ../README.txt.
//
// Stages (USB serial console, 115200 nominal -- the port is USB High Speed, baud is ignored):
//   S0  usbtx N / usbrx N        USB throughput (host script measures MB/s)
//   S1  mode mon                 passive monitor: SDCLK frequency, CMD decode + CRC7, DAT block CRC16,
//                                slot VDD, line levels. Never drives a pin.
//   S2  mode card (1-bit)        card emulation, stock init, CMD17/18/12, FAT16 1 GiB test volume
//   S3  mode card (4-bit)        same; ACMD6 arg 2 switches the DAT path to 4-bit FlexIO DMA
//   selftest                     CRC/response/wire-image/volume values for tools/selftest.py
//   touch                        GT911 (I2C 18/19) forwarded as a USB HID touchscreen (always on)
//
// Pin safety: nothing is ever driven unless mode=card AND slot VDD is sensed on pin 14.
#include <Arduino.h>
#include "config.h"
#include "sdproto.h"
#include "fxsd.h"
#include "selftest.h"
#include "touch.h"
#include "avstream.h"

static const char *const STATE_NAMES[8] = {"idle", "ready", "ident", "stby", "tran", "data", "rcv", "prg"};
static const char *const MODE_NAMES[3] = {"off", "mon", "card"};
static uint8_t perstat = 0;

// ---------------------------------------------------------------- printing helpers
static void phex(const uint8_t *b, int n) {
  for (int i = 0; i < n; i++) { if (b[i] < 16) Serial.print('0'); Serial.print(b[i], HEX); }
}
static void phex32(uint32_t v) {
  char s[9];
  snprintf(s, sizeof s, "%08lX", (unsigned long)v);
  Serial.print(s);
}

static void print_log(const LogRec &r) {
  char line[160];
  switch (r.kind) {
    case 'C':
      snprintf(line, sizeof line, "C t=%lu %s%u arg=%08lX crc=%s resp=%u", (unsigned long)r.t_us,
               r.app ? "ACMD" : "CMD", r.idx, (unsigned long)r.arg, r.crc == 1 ? "ok" : "BAD", r.responded);
      Serial.println(line);
      break;
    case 'R':
      if (r.nbytes == 17) {
        snprintf(line, sizeof line, "R t=%lu R2 crc=%s bytes=", (unsigned long)r.t_us, r.crc == 1 ? "ok" : "BAD");
        Serial.print(line); phex(r.b, 17); Serial.println();
      } else {
        const char *crc = r.crc == 1 ? "ok" : (r.crc == 2 ? "na(R3)" : "BAD");
        snprintf(line, sizeof line, "R t=%lu idx=%u arg=%08lX crc=%s st=%s%s", (unsigned long)r.t_us, r.idx,
                 (unsigned long)r.arg, crc, r.crc == 2 ? "-" : STATE_NAMES[(r.arg >> 9) & 7],
                 (r.crc != 2 && (r.arg & 0xFFF80000u)) ? " ERRBITS" : "");
        Serial.println(line);
      }
      break;
    case 'D':
      snprintf(line, sizeof line, "D t=%lu w=%d len=%d crc=%s first=", (unsigned long)r.t_us, r.width, r.len,
               r.crc ? "ok" : "BAD");
      Serial.print(line); phex(r.b, 8); Serial.println();
      break;
    case 'E':
      if (r.idx == 'V' || r.idx == 'v')
        snprintf(line, sizeof line, "E t=%lu %s %lumV", (unsigned long)r.t_us, r.idx == 'V' ? "VDD_ON" : "VDD_OFF",
                 (unsigned long)r.arg);
      else if (r.idx == 6)
        snprintf(line, sizeof line, "E t=%lu ACMD6 width=%s", (unsigned long)r.t_us, r.arg == 2 ? "4bit" : "1bit");
      else
        snprintf(line, sizeof line, "E t=%lu code=%u arg=%08lX", (unsigned long)r.t_us, r.idx, (unsigned long)r.arg);
      Serial.println(line);
      break;
  }
}

static void print_status(const char *tag) {
  uint32_t p = fx_pins();
  char line[260];
  snprintf(line, sizeof line,
           "%s mode=%s vdd=%u vddv=%.2f clk_avg=%lu clk_peak=%lu clk_min=%lu CLK=%lu CMD=%lu DAT=%lu%lu%lu%lu "
           "state=%s bus=%d cmds=%lu crcbad=%lu resp=%lu rcrcbad=%lu sectors=%lu dat_ok=%lu dat_bad=%lu "
           "retries=%lu illegal=%lu ovr=%lu",
           tag, MODE_NAMES[S.mode], S.vdd_ok, (double)S.vdd_volts, (unsigned long)S.clk_hz_avg,
           (unsigned long)S.clk_hz_peak, (unsigned long)S.clk_hz_min, (unsigned long)(p & 1),
           (unsigned long)((p >> 1) & 1), (unsigned long)((p >> 2) & 1), (unsigned long)((p >> 3) & 1),
           (unsigned long)((p >> 4) & 1), (unsigned long)((p >> 5) & 1), STATE_NAMES[S.card_state & 7],
           S.bus4 ? 4 : 1, (unsigned long)C.host_cmds, (unsigned long)C.host_crc_bad, (unsigned long)C.resp_seen,
           (unsigned long)C.resp_crc_bad, (unsigned long)C.sectors_served, (unsigned long)C.dat_ok,
           (unsigned long)C.dat_bad, (unsigned long)C.retries, (unsigned long)C.illegal,
           (unsigned long)(C.rx_overrun + C.ring_overrun_cmd + C.ring_overrun_dat));
  Serial.print(line);
  if (K.cdpin) { Serial.print(" cd="); Serial.print(digitalReadFast(PIN_CDSENSE)); }
  Serial.println();
}

#define CTR(name) do { Serial.print(" " #name "="); Serial.print((unsigned long)C.name); } while (0)
static void print_counters(void) {
  Serial.print("CTR"); CTR(host_cmds); CTR(host_crc_bad); CTR(resp_seen); CTR(resp_crc_bad); CTR(responded);
  CTR(unanswered); CTR(illegal); CTR(suppressed_novdd); Serial.println();
  Serial.print("CTR"); CTR(cmd0); CTR(cmd8); CTR(acmd41); CTR(cmd2); CTR(cmd3); CTR(cmd9); CTR(cmd7); CTR(cmd13);
  CTR(cmd16); CTR(acmd6); CTR(acmd13); CTR(acmd51); Serial.println();
  Serial.print("CTR"); CTR(cmd17); CTR(cmd18); CTR(cmd12); CTR(writes_refused); CTR(retries); CTR(other);
  CTR(sectors_served); CTR(small_blocks); Serial.println();
  Serial.print("CTR"); CTR(dat_ok); CTR(dat_bad); CTR(dat_w1); CTR(dat_w4); CTR(rx_overrun); CTR(dat_underrun); CTR(tx_underrun_resp); CTR(ring_overrun_cmd);
  CTR(ring_overrun_dat); CTR(vdd_on_events); CTR(vdd_off_events);
  Serial.print(" log_dropped="); Serial.println((unsigned long)fx_log_dropped);
}

// ---------------------------------------------------------------- knobs
struct KnobDef { const char *name; uint8_t *p; uint8_t maxv; uint8_t apply; const char *help; };  // apply: 0 none, 1 live, 2 engine restart
static KnobDef KNOBS[] = {
  {"samp", &K.samp, 1, 2, "RX sample edge 0=rising 1=falling"},
  {"launch", &K.launch, 1, 1, "TX launch edge 0=rising 1=falling"},
  {"wphase", &K.wphase, 2, 2, "word timer enable 0=rising 1=falling 2=free"},
  {"dse", &K.dse, 7, 1, "CMD/DAT drive strength 1..7"},
  {"sre", &K.sre, 1, 1, "fast slew"},
  {"speed", &K.speed, 3, 1, "pad SPEED 0..3"},
  {"hys", &K.hys, 1, 1, "CMD/DAT hysteresis"},
  {"pull", &K.pull, 1, 1, "47k pull-ups while VDD present (card)"},
  {"acmd41busy", &K.acmd41busy, 50, 0, "ACMD41 busy polls"},
  {"illegal", &K.illegal, 1, 0, "report ILLEGAL_COMMAND bit"},
  {"blockaddr", &K.blockaddr, 1, 0, "0=byte address (SDSC) 1=block"},
  {"pattern", &K.pattern, 1, 0, "data area pattern 1 / zeros 0"},
  {"lbk", &K.lbk, 1, 2, "card: loopback DAT capture + CRC"},
  {"log", &K.log, 2, 0, "0 off, 1 cmds+bad blocks, 2 every block"},
  {"dat", &K.dat, 1, 2, "monitor: sniff DAT"},
  {"cdpin", &K.cdpin, 1, 0, "report pin 15 level"},
  {"stream", &K.stream, 1, 0, "window 0x10000.. = Pi stream 1 / pattern 0"},
  {"vddgate", &K.vddgate, 1, 0, "1 = drive only with slot VDD sensed, 0 = ignore VDD (bench)"},
  {"fastpat", &K.fastpat, 1, 0, "pattern area = one pre-encoded sector (bench timing test)"},
  {"dlaunch", &K.dlaunch, 1, 1, "DAT TX launch edge 0=rising 1=falling"},
  {"mp3", &vol_mp3, 2, 0, "0 none, 1 SILENCE.MP3, 2 TONE.MP3 (RNS-E must remount)"},
};
static void print_knobs(void) {
  Serial.print("CFG");
  for (auto &k : KNOBS) { Serial.print(' '); Serial.print(k.name); Serial.print('='); Serial.print(*k.p); }
  Serial.print(" perstat="); Serial.println(perstat);
}

// ---------------------------------------------------------------- self-test (values for tools/selftest.py)
static void st_emit(const char *key, const char *val) {
  Serial.print("ST "); Serial.print(key); Serial.print(' '); Serial.println(val);
}
static void selftest(void) {
  Serial.println("ST begin " FW_VERSION);
  sdp_selftest(st_emit);
  Serial.println("ST end");
}

// ---------------------------------------------------------------- S0 USB throughput
static uint8_t usbbuf[65536] __attribute__((aligned(32)));
static void usb_tx_test(uint32_t mb) {
  uint32_t total = mb * 1048576u, sent = 0, ctr = 0;
  Serial.print("USBTX BEGIN "); Serial.println((unsigned long)total);
  Serial.flush();
  uint32_t t0 = micros();
  while (sent < total) {
    uint32_t n = total - sent; if (n > sizeof usbbuf) n = sizeof usbbuf;
    uint32_t *p = (uint32_t *)usbbuf;
    for (uint32_t i = 0; i < n / 4; i++) p[i] = ctr++;
    Serial.write(usbbuf, n);
    sent += n;
  }
  Serial.flush();
  uint32_t dt = micros() - t0;
  Serial.print("\nUSBTX END us="); Serial.println((unsigned long)dt);
}
static void usb_rx_test(uint32_t mb) {
  uint32_t total = mb * 1048576u, got = 0, ctr = 0, errs = 0;
  Serial.print("USBRX READY "); Serial.println((unsigned long)total);
  Serial.setTimeout(3000);
  uint32_t t0 = 0;
  while (got < total) {
    uint32_t want = total - got; if (want > sizeof usbbuf) want = sizeof usbbuf;
    size_t n = Serial.readBytes((char *)usbbuf, want);
    if (n == 0) break;
    if (!t0) t0 = micros();
    // pattern: little-endian u32 counter; n is a multiple of 4 except possibly at the tail
    uint32_t i = 0;
    while (i + 4 <= n) { uint32_t v; memcpy(&v, usbbuf + i, 4); if (v != ctr) { errs++; ctr = v; } ctr++; i += 4; }
    got += n;
    if (n & 3) { errs++; break; }
  }
  uint32_t dt = micros() - t0;
  Serial.setTimeout(1000);
  char line[96];
  snprintf(line, sizeof line, "USBRX DONE bytes=%lu us=%lu errors=%lu", (unsigned long)got, (unsigned long)dt,
           (unsigned long)errs);
  Serial.println(line);
}

// ---------------------------------------------------------------- console
static void help(void) {
  Serial.println("HELP " FW_VERSION);
  Serial.println("  ver | help | status | ctr | clr | cfg [key val] | phase 0..3 | perstat 0|1");
  Serial.println("  mode off|mon|card    (card drives only while slot VDD is sensed)");
  Serial.println("  pins | duty [samples] | selftest | sec <lba> | wire <lba> <1|4>");
  Serial.println("  usbtx <MB> | usbrx <MB>");
  Serial.println("  touch | touch on|swap|invx|invy|period <v> | touch save");
  Serial.println("  link | link reset   (live stream from pi/avlink: TLNK messages on this same port)");
}

static void print_link(void) {
  const AvsStatus &s = AVS.st;
  char line[300];
  snprintf(line, sizeof line,
           "LINK active=%u msgs=%lu free=%d/%lu slots=%d rx_sect=%lu begun=%lu served_seq=%lu srv_sect=%lu holds=%lu "
           "timeouts=%lu notready=%lu expired=%lu idle=%lu evicted=%lu conv=%lu rxerr=%lu",
           AVS.link_active, (unsigned long)AVS.rx_msgs, AVS.free_sectors(), (unsigned long)s.pool_sectors,
           AVS_NSLOT - AVS.used_slots(), (unsigned long)s.rx_sectors, (unsigned long)s.begun,
           (unsigned long)s.served_seq, (unsigned long)s.sectors_served, (unsigned long)s.holds,
           (unsigned long)s.hold_timeouts, (unsigned long)s.notready, (unsigned long)s.expired, (unsigned long)s.idle,
           (unsigned long)s.evicted, (unsigned long)s.converted, (unsigned long)s.rx_errors);
  Serial.println(line);
  for (int i = 0; i < AVS_NSLOT; i++) {
    const AvsSlot *t = AVS.slot(i);
    if (!t->used) continue;
    snprintf(line, sizeof line, "  slot%d seq=%lu total=%u have=%u ended=%u served=%u", i, (unsigned long)t->seq,
             t->total, t->have, t->ended, t->served);
    Serial.println(line);
  }
}

static void cmd_line(char *s) {
  char *argv[4] = {0}; int argc = 0;
  for (char *t = strtok(s, " \t"); t && argc < 4; t = strtok(nullptr, " \t")) argv[argc++] = t;
  if (!argc) return;
  const char *c = argv[0];
  if (!strcmp(c, "help")) help();
  else if (!strcmp(c, "ver")) {
    Serial.print("VER " FW_VERSION " flexio2_param="); phex32(S.param);
    Serial.print(" f_cpu="); Serial.println(F_CPU_ACTUAL);
  }
  else if (!strcmp(c, "status")) print_status("STAT");
  else if (!strcmp(c, "ctr")) print_counters();
  else if (!strcmp(c, "clr")) { fx_clear_counters(); Serial.println("OK clr"); }
  else if (!strcmp(c, "perstat") && argc > 1) { perstat = atoi(argv[1]) ? 1 : 0; Serial.println("OK"); }
  else if (!strcmp(c, "mode") && argc > 1) {
    FxMode m = !strcmp(argv[1], "mon") ? FX_MON : !strcmp(argv[1], "card") ? FX_CARD : FX_OFF;
    fx_set_mode(m);
    Serial.print("OK mode="); Serial.println(MODE_NAMES[S.mode]);
  }
  else if (!strcmp(c, "cfg")) {
    if (argc >= 3) {
      for (auto &k : KNOBS) if (!strcmp(k.name, argv[1])) {
        int v = atoi(argv[2]);
        if (v < 0 || v > k.maxv) { Serial.println("ERR range"); return; }
        *k.p = (uint8_t)v;
        if (k.apply == 2) fx_apply_knobs();      // timers/rings: restarts the engine, resets the card state
        else if (k.apply == 1) fx_update_live(); // pads / TX launch edge: live, no reset
        print_knobs();
        return;
      }
      Serial.println("ERR unknown knob");
    } else {
      print_knobs();
      for (auto &k : KNOBS) { Serial.print("  "); Serial.print(k.name); Serial.print(": "); Serial.println(k.help); }
    }
  }
  else if (!strcmp(c, "phase") && argc > 1) {
    // presets for eye/phase sweeps. 0: rising enable; 1: falling enable (inverted timer pin);
    // 2: free-running word phase; 3: deliberately wrong sample edge (sanity: must fail)
    int p = atoi(argv[1]);
    K.samp = 0;                                 // launch stays as set: 1 is what the RNS-E needs at 25 MHz
    if (p == 0) K.wphase = 0; else if (p == 1) K.wphase = 1; else if (p == 2) K.wphase = 2;
    else { K.wphase = 0; K.samp = 1; }
    fx_apply_knobs(); print_knobs();
  }
  else if (!strcmp(c, "pins")) {
    uint32_t p = fx_pins();
    char line[96];
    snprintf(line, sizeof line, "PINS CLK=%lu CMD=%lu DAT0=%lu DAT1=%lu DAT2=%lu DAT3=%lu vddv=%.2f cd=%d",
             (unsigned long)(p & 1), (unsigned long)((p >> 1) & 1), (unsigned long)((p >> 2) & 1),
             (unsigned long)((p >> 3) & 1), (unsigned long)((p >> 4) & 1), (unsigned long)((p >> 5) & 1),
             (double)S.vdd_volts, digitalReadFast(PIN_CDSENSE));
    Serial.println(line);
  }
  else if (!strcmp(c, "duty")) {
    uint32_t n = argc > 1 ? (uint32_t)atol(argv[1]) : 2000000u, edges = 0;
    float d = fx_duty(n, &edges);
    char line[96];
    snprintf(line, sizeof line, "DUTY high=%.2f%% samples=%lu clk_edges_during=%lu", (double)(d * 100.0f),
             (unsigned long)n, (unsigned long)edges);
    Serial.println(line);
  }
  else if (!strcmp(c, "trace")) {             // 'trace arm' then 'trace': CMD7 word trace (dcyc @600 MHz per 16 SDCLK)
    if (argc > 1) { fx_trace_arm(); Serial.println("OK trace armed"); }
    else {
      uint32_t cyc, last = 0; uint16_t w; uint8_t ss, txi; int r = 0;
      for (uint32_t i = 0; (r = fx_trace_get(i, &cyc, &w, &ss, &txi)) == 1; i++) {
        char tl[64];
        snprintf(tl, sizeof tl, "TR %3lu dcyc=%6lu w=%04X ss=%02X txi=%u", (unsigned long)i, (unsigned long)(i ? cyc - last : 0), w, ss, txi);
        Serial.println(tl);
        last = cyc;
      }
      Serial.println(r < 0 ? "TR not frozen" : "TR end");
    }
  }
  else if (!strcmp(c, "cmdlog")) {             // last host commands and the read engine, for stalls; flg 1 app 2 crc ok 4 rd_active 8 dma_busy 16 hold
    char tl[140]; uint32_t t, arg; uint8_t idx, st, q, flg;
    fx_read_state(tl, sizeof tl); Serial.println(tl);
    for (uint32_t i = 0; fx_cmdlog_get(i, &t, &arg, &idx, &st, &q, &flg); i++) {
      snprintf(tl, sizeof tl, "CL t=%lu cmd=%u arg=%08lX st=%u q=%u flg=%02X", (unsigned long)t, idx, (unsigned long)arg, st, q, flg);
      Serial.println(tl);
    }
  }
  else if (!strcmp(c, "selftest")) selftest();
  else if (!strcmp(c, "sec") && argc > 1) {
    uint8_t sec[512];
    uint32_t L = strtoul(argv[1], nullptr, 0);
    gen_sector(L, sec, K.pattern, fx_info_sector);
    Serial.print("SEC "); Serial.print((unsigned long)L); Serial.print(' '); phex(sec, 512); Serial.println();
  }
  else if (!strcmp(c, "wire") && argc > 2) {
    uint8_t sec[512]; static uint32_t w[WIRE_MAX_WORDS];
    uint32_t L = strtoul(argv[1], nullptr, 0);
    gen_sector(L, sec, K.pattern, fx_info_sector);
    int n = atoi(argv[2]) == 4 ? wire4(sec, 512, w) : wire1(sec, 512, w);
    Serial.print("WIRE "); Serial.print((unsigned long)L); Serial.print(' ');
    for (int i = 0; i < n - 1; i++) phex32(w[i]);
    Serial.println();
  }
  else if ((!strcmp(c, "usbtx") || !strcmp(c, "usbrx")) && argc > 1) {
    if (S.mode != FX_OFF) { Serial.println("ERR usb tests block the loop (no VDD watch): mode off first"); return; }
    if (!strcmp(c, "usbtx")) usb_tx_test((uint32_t)atol(argv[1])); else usb_rx_test((uint32_t)atol(argv[1]));
  }
  else if (!strcmp(c, "touch")) {
    if (argc == 2 && !strcmp(argv[1], "save")) { touch_save(); Serial.println("OK saved"); }
    else if (argc >= 3 && !touch_cfg(argv[1], atoi(argv[2]))) Serial.println("ERR touch key/value");
    else touch_print();
  }
  else if (!strcmp(c, "link")) {
    if (argc > 1 && !strcmp(argv[1], "reset")) { AVS.reset_link(); Serial.println("OK link reset"); }
    else print_link();
  }
  else Serial.println("ERR unknown command (help)");
}

static char linebuf[128];
static int linelen;
// bytes that are not part of a TLNK message (the console shares the CDC port with the binary link)
static void console_byte(uint8_t ch) {
  if (ch == '\r') return;
  if (ch == '\n') { linebuf[linelen] = 0; cmd_line(linebuf); linelen = 0; }
  else if (linelen < (int)sizeof(linebuf) - 1) linebuf[linelen++] = (char)ch;
}
static uint8_t rxbuf[4096];

void setup() {
  fx_begin();                                // pads high-Z first thing
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}
  touch_begin();                             // ~75 ms GT911 reset + probe; SD pads already high-Z
  Serial.println("BOOT " FW_VERSION);
  help();
  touch_print();
  fx_set_mode(FX_CARD);                      // card from power-up: drives only while slot VDD is sensed
}

void loop() {
  fx_poll();
  touch_poll();
  for (int rounds = 0; rounds < 16; rounds++) {      // USB RX: TLNK messages -> stream pool, the rest -> console
    int n = Serial.available();
    if (n <= 0) break;
    if (n > (int)sizeof rxbuf) n = sizeof rxbuf;
    n = Serial.readBytes((char *)rxbuf, (size_t)n);
    AVS.feed(rxbuf, n, console_byte);
    fx_poll();                                        // keep VDD watch / hold retries going during long bursts
  }
  if (AVS.link_active && !Serial) AVS.reset_link();  // Pi closed the port (DTR low): drop its superframes
  AVS.poll();                                        // slot reclaim, STATUS credits, SERVED
  LogRec r;
  int n = 0;
  while (n < 32 && fx_log_pop(r)) { print_log(r); n++; }
  static uint32_t last_stat;
  if (perstat && S.mode != FX_OFF && millis() - last_stat >= 1000) {
    last_stat = millis();
    print_status("STAT");
  }
}
