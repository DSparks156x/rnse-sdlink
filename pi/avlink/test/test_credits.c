/* Offline transport regression: cc -std=c11 -Wall -Wextra -Werror test/test_credits.c link.c -o /tmp/avlink-credit-test
 * /tmp/avlink-credit-test
 * Uses an ordered local socket instead of a tty; no hardware or firmware is required.
 * Protocol v1 has no epoch/challenge: the first STATUS must be a current connection snapshot. Input flush
 * reduces old tty data, but cannot authenticate that snapshot. A reset exactly resembling valid u32 wrap
 * is likewise indistinguishable. Automatic live reboot recovery requires a protocol epoch; it is not added here.
 */
#define _DEFAULT_SOURCE
#include "../link.h"
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static int peer;
static void open_test(link_t *l)
{
    int fd[2];
    assert(link_open(l, "/dev/null") == 0);
    close(l->fd);
    assert(socketpair(AF_UNIX, SOCK_STREAM, 0, fd) == 0);
    l->fd = l->rfd = fd[0]; peer = fd[1];
    assert(fcntl(l->rfd, F_SETFL, O_NONBLOCK) == 0);
    assert(link_credits(l) == -1 && link_slots(l) == -1);
}
static void close_test(link_t *l) { close(peer); link_close(l); }
static tl_status status(uint32_t rx, uint32_t begun, uint32_t free_sectors, uint32_t free_slots)
{
    tl_status s;
    memset(&s, 0, sizeof s);
    s.ver = 1; s.size = sizeof s; s.pool_sectors = 1072;
    s.rx_sectors = rx; s.begun = begun; s.free_sectors = free_sectors; s.free_slots = free_slots;
    return s;
}
static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static int deliver(link_t *l, const tl_status *s, uint32_t len)
{
    uint8_t b[TL_HDR + sizeof *s];
    memset(b, 0, sizeof b);
    put32(b, TL_MAGIC); b[4] = TL_STATUS; put32(b + 8, len);
    assert(len <= sizeof *s);
    memcpy(b + TL_HDR, s, len);
    assert(write(peer, b, TL_HDR + len) == (ssize_t)(TL_HDR + len));
    return link_read(l);
}
static void fault_is_sticky(link_t *l)
{
    assert(l->credit_error && l->hup);
    assert(link_credits(l) == -1 && link_slots(l) == -1);
    errno = 0; assert(link_read(l) == -1 && errno == EPROTO);
    errno = 0; assert(link_write(l) == -1 && errno == EPROTO);
    errno = 0; assert(link_drain(l, 10) == -1 && errno == EPROTO);
    uint8_t byte;
    assert(recv(peer, &byte, 1, MSG_DONTWAIT) == -1 && (errno == EAGAIN || errno == EWOULDBLOCK));
}
int main(void)
{
    link_t l;
    tl_status s;
    open_test(&l);
    s = status(10000, 300, 100, 2);       /* reopened host, receiver lifetime counters nonzero */
    assert(deliver(&l, &s, sizeof s) == 1);
    assert(link_credits(&l) == 100 && link_slots(&l) == 2);
    l.sent_sectors += 70; l.sent_begins++;
    assert(link_credits(&l) == 30 && link_slots(&l) == 1);
    s = status(10040, 301, 60, 1);       /* 30 sectors remain queued/in USB */
    assert(deliver(&l, &s, sizeof s) == 1);
    assert(link_credits(&l) == 30 && link_slots(&l) == 1);
    assert(deliver(&l, &s, sizeof s) == 1);  /* duplicate snapshot */
    s = status(10070, 301, 0, 0);
    assert(deliver(&l, &s, sizeof s) == 1);
    assert(link_credits(&l) == 0 && link_slots(&l) == 0); /* absent reader really backpressures */
    s.free_sectors = 80; s.free_slots = 2;
    assert(deliver(&l, &s, sizeof s) == 1);
    assert(link_credits(&l) == 80 && link_slots(&l) == 2);
    close_test(&l);

    open_test(&l);
    s = status(UINT32_MAX - 2, UINT32_MAX - 1, 100, 4);
    assert(deliver(&l, &s, sizeof s) == 1);
    l.sent_sectors = 5; l.sent_begins = 3;
    s = status(2, 1, 95, 1);            /* both receiver counters wrap */
    assert(deliver(&l, &s, sizeof s) == 1);
    assert(link_credits(&l) == 95 && link_slots(&l) == 1);
    l.sent_sectors = l.ack_sectors = UINT32_MAX - 1; /* long connection: local totals also wrap */
    l.sent_begins = l.ack_begins = UINT32_MAX;
    l.sent_sectors += 5; l.sent_begins += 2;
    s.rx_sectors += 5; s.begun += 2;
    assert(deliver(&l, &s, sizeof s) == 1);
    assert(link_credits(&l) == 95 && link_slots(&l) == 1);
    close_test(&l);

    for (int kind = 0; kind < 5; kind++) {
        open_test(&l); s = status(10000, 300, 100, 2);
        assert(deliver(&l, &s, sizeof s) == 1);
        l.sent_sectors = 10; l.sent_begins = 1;
        (void)link_msg(&l, TL_CONFIG, 0); /* must never flush even preexisting output after fault */
        if (kind == 0) { s.rx_sectors = 0; s.begun = 0; } /* receiver reset while in flight */
        if (kind == 1) s.rx_sectors += 11;              /* ack exceeds queued sectors */
        if (kind == 2) s.begun += 2;                    /* ack exceeds queued begins */
        if (kind == 3) s.rx_sectors--;                   /* old/backward STATUS */
        if (kind == 4) s.pool_sectors++;                /* receiver geometry changed */
        assert(deliver(&l, &s, sizeof s) == -1);
        assert(l.st.rx_sectors == 10000 && l.ack_sectors == 0); /* no partial acceptance */
        fault_is_sticky(&l); close_test(&l);
    }
    for (int kind = 0; kind < 7; kind++) {
        open_test(&l); s = status(10000, 300, 100, 2);
        uint32_t len = sizeof s;
        if (kind == 0) len = 8;
        if (kind == 1) s.ver++;
        if (kind == 2) s.size--;
        if (kind == 3) s.free_sectors = s.pool_sectors + 1;
        if (kind == 4) s.free_slots = 257;
        if (kind == 5) s.pool_sectors = 0;
        if (kind == 6) l.sent_begins = 1; /* cannot baseline pre-STATUS no-credit traffic */
        assert(deliver(&l, &s, len) == -1);
        fault_is_sticky(&l); close_test(&l);
    }
    /* review finding 6: device gone with part of a message buffered must still be a hangup */
    open_test(&l); s = status(10000, 300, 100, 2);
    assert(deliver(&l, &s, sizeof s) == 1);
    {
        uint8_t partial[TL_HDR + 4];
        memset(partial, 0, sizeof partial);
        put32(partial, TL_MAGIC); partial[4] = TL_STATUS; put32(partial + 8, sizeof s);
        assert(write(peer, partial, sizeof partial) == (ssize_t)sizeof partial);
        assert(link_read(&l) == 0 && l.ilen == sizeof partial && !l.hup);   /* still waiting for the rest */
        close(peer);
        assert(link_read(&l) == -1 && l.hup && !l.credit_error);            /* EOF behind it */
        link_close(&l);
    }
    open_test(&l); close(peer);                      /* and with nothing buffered, as before */
    assert(link_read(&l) == -1 && l.hup);
    link_close(&l);
    puts("PASS credit origins, backpressure, receiver/local wrap, invalid/reset acknowledgements, sticky fail-closed I/O, hangup with a partial message");
    return 0;
}
