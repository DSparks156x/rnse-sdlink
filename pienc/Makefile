# rnv2enc - build on Raspberry Pi 5 (aarch64, NEON) or x86-64 Linux (scalar; NEON logic testable via emulation)
#   make            -> rnv2enc (CLI), librnv2enc.a, rnv2dec_ref (unchanged reference decoder copy), test_kernels
#   make test       -> NEON-vs-scalar kernel self-check + closed-loop encode/decode check on a synthetic clip
#   make emutest    -> x86 only: compile the NEON kernels against src/neon_emu.h and run the same self-check
#   make neoncheck  -> x86 only: compile-check the NEON kernels for aarch64 with clang (object only)
CC      ?= gcc
ARCH    := $(shell uname -m)
OPT     ?= -O3
ifeq ($(ARCH),aarch64)
  MARCH ?= -mcpu=cortex-a76
endif
ifeq ($(ARCH),x86_64)
  MARCH ?= -march=x86-64-v2
endif
CFLAGS  ?= $(OPT) $(MARCH) -g
CFLAGS  += -std=c11 -Wall -Wextra -Wno-unused-parameter -Wno-misleading-indentation -ffp-contract=off -Iinclude -Isrc -D_POSIX_C_SOURCE=200809L
LDLIBS  += -lpthread -lm

SRC = src/enc.c src/kern_scalar.c src/kern_neon.c
OBJ = $(SRC:.c=.o)

all: rnv2enc rnv2dec_ref rnv3dec_ref test_kernels bench_kernels

librnv2enc.a: $(OBJ)
	ar rcs $@ $^

src/%.o: src/%.c src/kern.h src/csc_common.h include/rnv2enc.h src/rnv2_tables.h
	$(CC) $(CFLAGS) -c $< -o $@

rnv2enc: src/main.o librnv2enc.a
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

test_kernels: src/test_kernels.o librnv2enc.a
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

bench_kernels: tools/bench_kernels.c src/kern_scalar.o src/kern_neon.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

# per-frame kernel work counts for the speed model (single thread)
rnv2enc_count: src/main.c src/enc.c src/kern_scalar.c src/kern_neon.c
	$(CC) $(CFLAGS) -DRNV_COUNT src/main.c src/enc.c src/kern_scalar.c src/kern_neon.c -o $@ $(LDLIBS)

synth: tools/synth.c
	$(CC) $(CFLAGS) $< -o $@ $(LDLIBS)

# the reference decoder, compiled unchanged from ref/rnv2dec.c (byte-identical copy of video/dec/rnv2dec.c)
rnv2dec_ref: ref/rnv2dec.c ref/rnv2_tables.h
	$(CC) -O2 -DRNV_MAIN ref/rnv2dec.c -o $@

# RNV3 reference decoder (dec/): what the closed-loop test checks against
rnv3dec_ref: dec/rnv3dec.c dec/rnv2_tables.h
	$(CC) -O2 -DRNV_MAIN dec/rnv3dec.c -o $@ -lm

ifeq ($(ARCH),aarch64)
test: rnv2enc rnv3dec_ref test_kernels synth
	./test_kernels
	sh tools/closed_loop_test.sh
else
# x86: no real NEON; check the NEON logic through src/neon_emu.h (kernels + whole encoder), then the closed loop
test: rnv2enc rnv3dec_ref synth emutest rnv2enc_emu
	sh tools/closed_loop_test.sh
endif

rnv2enc_emu: src/main.c src/enc.c src/kern_scalar.c src/kern_neon.c src/neon_emu.h
	$(CC) $(CFLAGS) -DRNV_NEON_EMU src/main.c src/enc.c src/kern_scalar.c src/kern_neon.c -o $@ $(LDLIBS)

emutest:
	$(CC) $(OPT) -g -std=c11 -Wall -Iinclude -Isrc -DRNV_NEON_EMU src/test_kernels.c src/kern_scalar.c src/kern_neon.c -o test_kernels_emu -lm
	./test_kernels_emu

neoncheck:
	clang --target=aarch64-linux-gnu -O3 -mcpu=cortex-a76 -std=c11 -Wall -Wextra -Wno-unused-parameter -Iinclude -Isrc -ffreestanding -c src/kern_neon.c -o /tmp/kern_neon_a64.o
	clang --target=aarch64-linux-gnu -O3 -mcpu=cortex-a76 -std=c11 -Wall -Iinclude -Isrc -ffreestanding -fsyntax-only src/kern_scalar.c
	@echo "aarch64 NEON object OK: /tmp/kern_neon_a64.o"

clean:
	rm -f src/*.o librnv2enc.a rnv2enc rnv2dec_ref rnv3dec_ref test_kernels test_kernels_emu rnv2enc_emu rnv2enc_count bench_kernels synth headroom_eval

.PHONY: headroom_eval all test emutest neoncheck clean

headroom_eval: tools/headroom_eval.c
	$(CC) $(CFLAGS) $< -o $@ -lm
