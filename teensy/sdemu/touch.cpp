// touch.cpp -- GT911 over Wire (LPI2C1, pins 18/19) forwarded as the core's USB multitouch device.
// Register map follows the GT911 programming guide / Linux goodix.c. Read-only towards the panel: the only
// write is clearing the 0x814E buffer flag. The config area (0x8047..0x8100) is never written.
//
// Runs in loop() at thread level: Wire is polled, so it cannot delay the FlexIO2 ISR (priority 0) or the
// DMA pump. One status poll at 400 kHz is ~0.1 ms; a 5-point report ~1 ms.
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "config.h"
#include "touch.h"

#define GT_STATUS  0x814E
#define GT_POINTS  0x814F
#define GT_ID      0x8140
#define GT_MAXPTS  5
#define EE_ADDR    0       // EEPROM: 'T', cfg bytes
#define EE_MAGIC   0x54

#ifndef MULTITOUCH_INTERFACE  // plain USB Type "Serial": panel is still read and reported, nothing goes to USB
#define MULTITOUCH_FINGERS 10
static void usb_touchscreen_press(uint8_t, uint32_t, uint32_t, uint32_t) {}
static void usb_touchscreen_release(uint8_t) {}
#endif

TouchCfg TC = {1, 0, 0, 0, 5};
TouchStat TS;
static uint8_t down[MULTITOUCH_FINGERS];

static bool gt_read(uint16_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(TS.addr);
  Wire.write(reg >> 8); Wire.write(reg & 0xFF);
  if (Wire.endTransmission(false) != 0) { TS.i2c_err++; return false; }
  if (Wire.requestFrom(TS.addr, n) != n) { TS.i2c_err++; return false; }
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static bool gt_write8(uint16_t reg, uint8_t v) {
  Wire.beginTransmission(TS.addr);
  Wire.write(reg >> 8); Wire.write(reg & 0xFF); Wire.write(v);
  if (Wire.endTransmission() != 0) { TS.i2c_err++; return false; }
  return true;
}

// GT911 address strap: INT level while RST rises. INT low -> 0x5D (Linux default).
static void gt_reset(void) {
  pinMode(PIN_TRST, OUTPUT); pinMode(PIN_TINT, OUTPUT);
  digitalWriteFast(PIN_TRST, LOW); digitalWriteFast(PIN_TINT, LOW);
  delay(11);
  digitalWriteFast(PIN_TRST, HIGH);
  delay(6);
  digitalWriteFast(PIN_TINT, LOW);
  delay(55);
  pinMode(PIN_TINT, INPUT);  // hand INT back to the panel
}

static bool probe(uint8_t a) {
  Wire.beginTransmission(a);
  return Wire.endTransmission() == 0;
}

void touch_begin(void) {
  memset(&TS, 0, sizeof TS);
  if (EEPROM.read(EE_ADDR) == EE_MAGIC)
    for (unsigned i = 0; i < sizeof TC; i++) ((uint8_t *)&TC)[i] = EEPROM.read(EE_ADDR + 1 + i);
  if (TC.period < 1) TC.period = 5;
  gt_reset();
  Wire.begin();
  Wire.setClock(400000);
  if (probe(0x5D)) TS.addr = 0x5D;
  else if (probe(0x14)) TS.addr = 0x14;
  else return;
  uint8_t b[11];
  if (!gt_read(GT_ID, b, 11)) { TS.addr = 0; return; }
  memcpy(TS.id, b, 4); TS.id[4] = 0;
  TS.fw = b[4] | b[5] << 8;
  TS.xres = b[6] | b[7] << 8;
  TS.yres = b[8] | b[9] << 8;
  if (TS.xres < 2 || TS.yres < 2) { TS.xres = 800; TS.yres = 480; }  // blank config: assume panel native
}

static uint32_t map_axis(uint32_t v, uint32_t res, uint8_t inv) {
  if (v >= res) v = res - 1;
  uint32_t m = v * 32767u / (res - 1);
  return inv ? 32767u - m : m;
}

void touch_poll(void) {
  static uint32_t last, wait;
  if (!TC.on || !TS.addr || millis() - last < wait) return;
  last = millis();
  wait = TC.period;
  TS.polls++;
  uint8_t st;
  if (!gt_read(GT_STATUS, &st, 1)) { wait = 250; return; }  // bus trouble: back off, Wire can stall up to 50 ms
  if (!(st & 0x80)) return;                                  // bit7 = new report ready
  uint8_t n = st & 0x0F;
  if (n > GT_MAXPTS) n = GT_MAXPTS;
  uint8_t p[GT_MAXPTS * 8];
  if (n && !gt_read(GT_POINTS, p, n * 8)) { gt_write8(GT_STATUS, 0); return; }
  gt_write8(GT_STATUS, 0);
  TS.reports++;
  TS.fingers = n;
  uint8_t seen[MULTITOUCH_FINGERS] = {0};
  for (uint8_t i = 0; i < n; i++) {
    const uint8_t *q = p + i * 8;  // track id, x lo/hi, y lo/hi, size lo/hi, reserved
    uint8_t f = q[0] % MULTITOUCH_FINGERS;
    uint32_t x = q[1] | q[2] << 8, y = q[3] | q[4] << 8;
    uint32_t xr = TS.xres, yr = TS.yres;
    if (TC.swap) { uint32_t t = x; x = y; y = t; t = xr; xr = yr; yr = t; }
    TS.lastx = x; TS.lasty = y;
    uint32_t size = q[5] | q[6] << 8;
    usb_touchscreen_press(f, map_axis(x, xr, TC.invx), map_axis(y, yr, TC.invy), size ? (size > 255 ? 255 : size) : 128);
    seen[f] = 1;
  }
  for (uint8_t f = 0; f < MULTITOUCH_FINGERS; f++) {
    if (down[f] && !seen[f]) usb_touchscreen_release(f);
    down[f] = seen[f];
  }
}

void touch_print(void) {
  char line[200];
  snprintf(line, sizeof line,
           "TOUCH addr=%02X id=%s fw=%04X res=%ux%u on=%u swap=%u invx=%u invy=%u period=%u fingers=%u "
           "last=%u,%u reports=%lu polls=%lu i2c_err=%lu",
           TS.addr, TS.addr ? TS.id : "-", TS.fw, TS.xres, TS.yres, TC.on, TC.swap, TC.invx, TC.invy, TC.period,
           TS.fingers, TS.lastx, TS.lasty, (unsigned long)TS.reports, (unsigned long)TS.polls,
           (unsigned long)TS.i2c_err);
  Serial.println(line);
}

bool touch_cfg(const char *key, int v) {
  if (v < 0 || v > 255) return false;
  if (!strcmp(key, "on")) TC.on = v ? 1 : 0;
  else if (!strcmp(key, "swap")) TC.swap = v ? 1 : 0;
  else if (!strcmp(key, "invx")) TC.invx = v ? 1 : 0;
  else if (!strcmp(key, "invy")) TC.invy = v ? 1 : 0;
  else if (!strcmp(key, "period") && v >= 1) TC.period = v;
  else return false;
  if (!TC.on)
    for (uint8_t f = 0; f < MULTITOUCH_FINGERS; f++) if (down[f]) { usb_touchscreen_release(f); down[f] = 0; }
  return true;
}

void touch_save(void) {
  EEPROM.update(EE_ADDR, EE_MAGIC);
  for (unsigned i = 0; i < sizeof TC; i++) EEPROM.update(EE_ADDR + 1 + i, ((uint8_t *)&TC)[i]);
}
