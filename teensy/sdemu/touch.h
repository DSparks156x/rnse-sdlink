// touch.h -- GT911 I2C touch panel -> USB HID multitouch (Linux hid-multitouch, no Pi driver needed).
#pragma once
#include <stdint.h>

struct TouchCfg {
  uint8_t on;      // 1 = poll the panel and forward reports
  uint8_t swap;    // swap X/Y before mapping
  uint8_t invx;    // mirror X (after swap)
  uint8_t invy;    // mirror Y (after swap)
  uint8_t period;  // poll period, ms
};

struct TouchStat {
  uint8_t addr;        // 0x5D / 0x14, 0 = not found
  char id[5];          // product ID, "911"
  uint16_t fw;         // firmware version
  uint16_t xres, yres; // panel output range (0x8146/0x8148)
  uint8_t fingers;     // contacts in the last report
  uint16_t lastx, lasty;
  uint32_t reports, polls, i2c_err;
};

extern TouchCfg TC;
extern TouchStat TS;

void touch_begin(void);         // reset (if RST wired), probe, read ID/resolution, load saved cfg
void touch_poll(void);          // call from loop(); never blocks longer than one I2C report read
void touch_print(void);         // "TOUCH ..." status line
bool touch_cfg(const char *key, int v);
void touch_save(void);          // cfg -> EEPROM
