/* Exercise the production sender pump with --nocredits, using a local socket and one sector only.
 * cc -std=c11 -Wall -Wextra -Werror -O2 test/test_nocredits.c sf.c wire.c crc.c audio.c link.c -o test/nocredits-test
 * Including the implementation gives this test access to the actual static pump without production test hooks. */
#define main avlink_program_main
#include "../avlink.c"
#undef main
#include <assert.h>
#include <sys/socket.h>

static void queue_one_test_sf(void)
{
    txsf *t = &tq[(tq_head + tq_n++) % TQ_N];
    memset(t, 0, sizeof *t);
    t->seq = 7; t->total = 1; t->body = calloc(1, 512);
    assert(t->body);
}
int main(void)
{
    int sockets[2];
    assert(link_open(&g_link, "/dev/null") == 0);
    close(g_link.fd);
    assert(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) == 0);
    g_link.fd = g_link.rfd = sockets[0];
    assert(fcntl(g_link.rfd, F_SETFL, O_NONBLOCK) == 0);
    O.nocredits = 1;
    queue_one_test_sf();
    send_config();
    tx_pump();
    assert(!g_link.have_status && tq_n == 1 && tq[tq_head].state == 0);
    assert(g_link.sent_begins == 0 && g_link.sent_sectors == 0);
    assert(link_write(&g_link) == 0);
    uint8_t bytes[2048];
    ssize_t n = recv(sockets[1], bytes, sizeof bytes, MSG_DONTWAIT);
    assert(n == TL_HDR + sizeof(tl_config));
    assert(bytes[4] == TL_CONFIG);             /* handshake still flows, no SF_BEGIN/WIRE escaped */
    assert(recv(sockets[1], bytes, sizeof bytes, MSG_DONTWAIT) == -1 && errno == EAGAIN);

    uint8_t reply[TL_HDR + sizeof(tl_status)] = {0};
    put32(reply, TL_MAGIC); put16(reply + 4, TL_STATUS); put32(reply + 8, sizeof(tl_status));
    tl_status s = {0};
    s.ver = 1; s.size = sizeof s; s.pool_sectors = 1072;
    s.rx_sectors = 10000; s.begun = 300;        /* receiver retains old-session totals */
    s.free_sectors = s.free_slots = 0;         /* diagnostic bypasses capacity only after origin is known */
    memcpy(reply + TL_HDR, &s, sizeof s);
    assert(write(sockets[1], reply, sizeof reply) == sizeof reply);
    assert(link_read(&g_link) == 1 && g_link.have_status && !g_link.credit_error);
    tx_pump();
    assert(tq_n == 0 && g_sent_sf == 1 && g_link.sent_begins == 1 && g_link.sent_sectors == 1);
    assert(link_write(&g_link) == 0);
    n = recv(sockets[1], bytes, sizeof bytes, MSG_DONTWAIT);
    const uint16_t expected[] = {TL_SF_BEGIN, TL_WIRE, TL_SF_END};
    size_t offset = 0;
    for (size_t i = 0; i < sizeof expected / sizeof expected[0]; i++) {
        assert(offset + TL_HDR <= (size_t)n);
        uint16_t type; uint32_t length;
        memcpy(&type, bytes + offset + 4, sizeof type);
        memcpy(&length, bytes + offset + 8, sizeof length);
        assert(type == expected[i]);
        offset += TL_HDR + length;
    }
    assert(offset == (size_t)n);
    queue_one_test_sf();
    g_link.credit_error = 1;
    tx_pump();
    assert(tq_n == 1 && !link_pending(&g_link));
    free(tq[tq_head].body);
    close(sockets[1]); link_close(&g_link);
    puts("PASS --nocredits waits for STATUS, permits CONFIG, then sends bounded SF traffic; credit error stops pump");
    return 0;
}
