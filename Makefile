# sdlink: Pi-side binaries. On a Raspberry Pi (aarch64):
#   make            -> bin/rnv3enc, bin/avlink, bin/wlcap (same flags the Pi installs with)
#                      plus bin/rnv3enc-{a76,a72,a53}: the encoder tuned for the Pi 5 / Pi 4 / Pi 3 and Zero 2 cores.
#                      bin/rnv3enc is plain ARMv8-A and runs on any aarch64 CPU; pi/run/hud2.sh picks the variant
#                      for the CPU it runs on (/proc/cpuinfo "CPU part") and falls back to bin/rnv3enc.
#   make test       -> encoder kernel + closed-loop checks (pienc/), avlink C unit tests
#   make clean
# Teensy firmware: cd teensy && pio run.  RNS-E program: cd rnse && bash build.sh (sh-elf toolchain).
CC      ?= gcc
BIN     ?= bin
ARCH    := $(shell uname -m)
ENC_CPUS :=
ifeq ($(ARCH),aarch64)
  ENCARCH ?= -march=armv8-a -mtune=generic
  ENC_CPUS := a76 a72 a53
endif

all: $(BIN)/rnv3enc $(BIN)/avlink $(BIN)/wlcap $(ENC_CPUS:%=$(BIN)/rnv3enc-%)

$(BIN):
	mkdir -p $@

ENC_SRC := pienc/src/main.c pienc/src/enc.c pienc/src/kern_scalar.c pienc/src/kern_neon.c
$(BIN)/rnv3enc: $(ENC_SRC) $(wildcard pienc/src/*.h pienc/src/*.inc pienc/include/*.h) | $(BIN)
	$(CC) -O3 $(ENCARCH) $(ENC_FLAGS) $(ENC_SRC) -o $@ -lpthread -lm

ENC_FLAGS := -std=c11 -ffp-contract=off -D_POSIX_C_SOURCE=200809L -Ipienc/include -Ipienc/src -w
# Same source, one per Pi core: bin/rnv3enc-a76 is -mcpu=cortex-a76, and so on.
$(BIN)/rnv3enc-%: $(ENC_SRC) $(wildcard pienc/src/*.h pienc/src/*.inc pienc/include/*.h) | $(BIN)
	$(CC) -O3 -mcpu=cortex-$* $(ENC_FLAGS) $(ENC_SRC) -o $@ -lpthread -lm

$(BIN)/avlink: $(wildcard pi/avlink/*.c pi/avlink/*.h) | $(BIN)
	$(MAKE) -s -C pi/avlink
	mv pi/avlink/avlink $@

$(BIN)/wlcap: pi/wlcap/wlcap.c | $(BIN)
	$(CC) -O2 -pthread -o $@ $<

test:
	$(MAKE) -C pienc test

clean:
	rm -rf $(BIN)
	$(MAKE) -s -C pienc clean 2>/dev/null || true
	$(MAKE) -s -C pi/avlink clean 2>/dev/null || true

.PHONY: all test clean
