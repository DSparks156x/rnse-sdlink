// selftest.h -- hardware-independent self-test. Emits "key value" pairs that tools/selftest.py compares
// against ../link/sd_wire.py. Runs on the Teensy ('selftest' command) and on a PC (tools/host_selftest).
#pragma once
typedef void (*st_emit_fn)(const char *key, const char *value);
int sdp_selftest(st_emit_fn emit);   // returns number of internal failures (0 = pass)
