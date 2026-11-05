// host_selftest: compiles the firmware's hardware-independent code (sdproto.cpp, selftest.cpp) natively
// and prints the same "ST key value" lines as the Teensy 'selftest' command. tools/selftest.py --host
// builds and runs it (zig c++ from the local venv) and compares against sd_wire.py.
#include <stdio.h>
#include "../../sdemu/sdproto.h"
#include "../../sdemu/selftest.h"
static void emit(const char *k, const char *v) { printf("ST %s %s\n", k, v); }
int main(void) {
  sdp_init();
  printf("ST begin host\n");
  int f = sdp_selftest(emit);
  printf("ST end\n");
  return f ? 1 : 0;
}
