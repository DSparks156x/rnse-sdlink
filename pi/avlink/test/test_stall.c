/* test_stall.c -- what a stalled link does to avlink's inputs, no device needed. Build from pi/avlink:
 *   cc -O2 -std=c11 -Dmain=avlink_main -c avlink.c -o /tmp/avlink_t.o && cc -O2 -std=c11 test/test_stall.c /tmp/avlink_t.o sf.c wire.c crc.c audio.c link.c -I. -o /tmp/test_stall
 * Checks: 1) a full audio buffer keeps draining its FIFO, so poll() goes quiet instead of spinning; 2) video input stops being requested above the limit,
 * so the pipe fills and the encoder blocks; a single frame larger than the limit still completes. */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include "audio.h"
#include "sf.h"

/* from avlink.c, exported for this test only */
int avlink_test_wants(int fd, int eof, uint8_t *buf, size_t have);

static int fails;
#define CHECK(c, m) do { if (c) printf("ok   %s\n", m); else { printf("FAIL %s\n", m); fails++; } } while (0)

int main(void)
{
    char path[] = "/tmp/test_stall_XXXXXX";
    if (!mkdtemp(path)) return 2;
    char fifo[64];
    snprintf(fifo, sizeof fifo, "%s/f", path);
    mkfifo(fifo, 0600);
    int w = open(fifo, O_RDWR | O_NONBLOCK);           /* keeps a writer so the reader sees no EOF */
    audio_stage a;
    if (audio_open(&a, fifo, 0, 1)) return 2;
    static uint8_t junk[32768];
    long total = 0;
    for (int round = 0; round < 8; round++) {          /* more than the 64 KiB buffer holds, with no audio_take: a stalled link */
        ssize_t n = write(w, junk, sizeof junk);
        if (n > 0) total += n;
        audio_read_avail(&a);
    }
    struct pollfd p = {a.fd, POLLIN, 0};
    CHECK(total > (long)a.cap, "wrote more audio than the buffer holds");
    CHECK(poll(&p, 1, 0) == 0, "FIFO empty after audio_read_avail, poll() is quiet");
    CHECK(a.have <= a.cap, "buffer within bounds");
    CHECK(a.frames_dropped > 0, "oldest audio counted as dropped");
    CHECK((a.have & 3) == 0 || a.have % 4 == (size_t)(total % 4), "dropping keeps frame alignment");
    audio_close(&a);
    close(w);
    unlink(fifo);
    rmdir(path);

    static uint8_t b[2 << 20];
    memset(b, 0, sizeof b);
    put_be32(b, 1000);                                 /* first frame 1000 B, complete */
    CHECK(avlink_test_wants(9, 0, b, 1000) == 1, "little buffered: wants input");
    CHECK(avlink_test_wants(9, 0, b, 256u << 10) == 0, "at the limit with a complete first frame: no more input");
    put_be32(b, (uint32_t)(1 << 20));                  /* first frame larger than the limit, incomplete */
    CHECK(avlink_test_wants(9, 0, b, 300u << 10) == 1, "oversized frame still incomplete: wants input");
    put_be32(b, 0x80000000u | (uint32_t)(1 << 20));    /* same, key flag set */
    CHECK(avlink_test_wants(9, 0, b, 300u << 10) == 1, "key flag does not disturb the length");
    CHECK(avlink_test_wants(9, 0, b, (1u << 20) + 4) == 0, "oversized frame complete: no more input");
    CHECK(avlink_test_wants(9, 1, b, 0) == 0, "EOF: no input");
    CHECK(avlink_test_wants(-1, 0, b, 0) == 0, "no video source: no input");
    printf(fails ? "FAILED %d\n" : "all passed\n", fails);
    return fails != 0;
}
