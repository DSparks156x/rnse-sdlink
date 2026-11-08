# sdlink: Pi-side binaries. On a Raspberry Pi 5 (aarch64):
#   make            -> bin/rnv3enc, bin/avlink, bin/wlcap (same flags the Pi installs with)
#   make test       -> encoder kernel + closed-loop checks (pienc/), avlink C unit tests
#   make clean
# Teensy firmware: cd teensy && pio run.  RNS-E program: cd rnse && bash build.sh (sh-elf toolchain).
CC      ?= gcc
BIN     ?= bin
ARCH    := $(shell uname -m)
ifeq ($(ARCH),aarch64)
  ENCARCH ?= -mcpu=cortex-a76
endif

all: $(BIN)/rnv3enc $(BIN)/avlink $(BIN)/wlcap

$(BIN):
	mkdir -p $@

ENC_SRC := pienc/src/main.c pienc/src/enc.c pienc/src/kern_scalar.c pienc/src/kern_neon.c
$(BIN)/rnv3enc: $(ENC_SRC) $(wildcard pienc/src/*.h pienc/src/*.inc pienc/include/*.h) | $(BIN)
	$(CC) -O3 $(ENCARCH) -std=c11 -ffp-contract=off -D_POSIX_C_SOURCE=200809L -Ipienc/include -Ipienc/src -w $(ENC_SRC) -o $@ -lpthread -lm

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
