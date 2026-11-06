/* Unit checks of avlink's live latency limits: discard, key gate, reader tracking, reader window, bootstrap image hold. No device, no timing.
 * cc -std=c11 -Wall -Wextra -Werror -O2 test/test_discard.c sf.c wire.c crc.c audio.c link.c -o test/discard-test
 * Including the implementation gives access to the static functions, as test_nocredits.c. */
#define main avlink_program_main
#include "../avlink.c"
#undef main
#include <assert.h>

static int wfd;
static uint32_t next_no;
static void put_frame(uint32_t len, int key, uint32_t nbytes)   /* nbytes of the payload now, the rest later with put_rest */
{
    uint8_t b[4]; put_be32(b, len | (key ? 0x80000000u : 0));
    assert(write(wfd, b, 4) == 4);
    static uint8_t z[1 << 16];
    memset(z, (int)(next_no++ & 0x7F), nbytes);
    assert(write(wfd, z, nbytes) == (ssize_t)nbytes);
}
static void put_rest(uint32_t nbytes) { static uint8_t z[1 << 16]; assert(write(wfd, z, nbytes) == (ssize_t)nbytes); }
static void fill(void) { for (int i = 0; i < 8; i++) plan_fill(0); }
static void clear_fq(void) { while (fq_n) { frag_t *f = &fq[fq_head]; fq_head = (fq_head + 1) % FQ_N; fq_n--; frame_unref(f->f); } }

int main(void)
{
    char dir[] = "/tmp/avl_disc_XXXXXX", path[64];
    assert(mkdtemp(dir));
    snprintf(path, sizeof path, "%s/v", dir);
    assert(mkfifo(path, 0600) == 0);
    wfd = open(path, O_RDWR | O_NONBLOCK);
    assert(wfd >= 0 && vsrc_open(&g_vid, path, 1) == 0 && g_vid.nonblock);
    long pipesz = fcntl(g_vid.fd, F_GETPIPE_SZ);
    assert(pipesz > 0 && pipesz <= 65536);
    assert(link_open(&g_link, "/dev/null") == 0);
    g_have_link = 1; g_can_discard = 1;
    g_keyfd = open("/dev/null", O_WRONLY);

    /* the frame limit, not bytes: two small complete frames stop the reads, so the encoder blocks */
    put_frame(100, 0, 100); put_frame(100, 0, 100);
    assert(vsrc_wants_input(&g_vid));                  /* still in the pipe */
    vsrc_read(&g_vid, 65536);
    assert(!vsrc_wants_input(&g_vid) && g_vid.have == 208);
    assert(backlog_ticks() == 2);
    fill();                                            /* fq takes up to 2 */
    assert(fq_n == 2 && backlog_ticks() == 2);
    clear_fq();

    /* discard: complete frames go, a partial one survives and stays framed */
    put_frame(100, 0, 100); put_frame(100, 1, 100); put_frame(100, 0, 100);
    put_frame(100, 0, 50);
    uint32_t before = g_vdiscarded;
    video_discard();
    assert(g_wait_key && g_vdiscarded == before + 3 && g_vid.have == 54 && fq_n == 0);

    /* gate: the end of the partial delta and more deltas do not pass, the first key does, deltas after it do */
    put_rest(50); put_frame(100, 0, 100);
    fill();
    assert(fq_n == 0 && g_wait_key && g_vid.have == 0);
    uint32_t idx0 = g_frames_in;
    put_frame(300, 1, 300); put_frame(100, 0, 100);
    fill();
    assert(!g_wait_key && fq_n == 2 && fq[fq_head].f->key && fq[fq_head].f->index == idx0);
    assert(!fq[(fq_head + 1) % FQ_N].f->key);

    /* discard drops queued fragments too, a frame in pieces included */
    O.maxvid = 1;                                      /* 512 B per fragment */
    put_frame(1500, 0, 1500);
    clear_fq();
    fill();
    assert(fq_n == 3);
    before = g_vdiscarded;
    video_discard();
    assert(fq_n == 0 && g_vdiscarded == before + 1 && g_wait_key);
    O.maxvid = 140;

    /* reader tracking: lifetime counters are only a baseline; no reader until a SERVED; keepalive state; return asks for a key at once */
    tl_status *s = &g_link.st;
    memset(s, 0, sizeof *s);
    s->served_seq = 5000; s->sectors_served = 777; s->holds = 3;
    g_link.have_status = 1; g_link.n_status = 1;
    uint64_t t = 1000000;
    g_key_t = 0; g_link.n_served = 0;
    reader_update(t);
    assert(g_rd.stalled && !g_rd.t_act);
    key_feedback(t); key_feedback(t);                  /* first call is the baseline */
    assert(g_key_reqs == 0);                           /* no reader: no key */
    s->holds++;                                        /* something polls the window */
    reader_update(t += 1000);
    assert(g_rd.stalled && g_rd.t_act == t);
    g_link.n_served++; g_link.last_served_seq = 5001; s->served_seq = 5001;
    reader_update(t += 1000);
    assert(!g_rd.stalled && g_rd.returned);
    key_feedback(t);
    assert(g_key_reqs == 1 && !g_rd.returned);
    assert(g_wait_key);
    key_feedback(t + 1000);
    assert(g_key_reqs == 1);                           /* rate limit */
    key_feedback(t + 600000);
    assert(g_key_reqs == 2);                           /* the key has not come: ask again */
    /* reader leaves */
    put_frame(100, 1, 100);
    g_wait_key = 0;
    uint32_t stalls = g_stalls;
    reader_update(t += RD_STALL_US - 1000);
    assert(!g_rd.stalled);
    reader_update(t += 2000);
    assert(g_rd.stalled && g_stalls == stalls + 1 && g_wait_key && g_vid.have == 0);
    /* frames arriving while away go at once, a key too: the decoder cannot use it yet */
    put_frame(100, 1, 100);
    reader_update(t += 1000);
    assert(g_vid.have == 0);

    /* window: not more than RD_WINDOW ahead while the reader works, no limit while it makes no progress */
    s->free_sectors = 1000; s->free_slots = 16; s->pool_sectors = 1500; s->ver = 1; s->size = sizeof *s;
    g_link.last_served_seq = 100; g_link.n_served = 1;
    txsf *q = &tq[(tq_head + tq_n++) % TQ_N];
    memset(q, 0, sizeof *q);
    q->seq = 100 + RD_WINDOW + 1; q->total = 1; q->body = calloc(1, 512);
    g_rd.stalled = 0;
    tx_pump();
    assert(tq_n == 1 && g_window_waits == 1 && !g_link.sent_begins);
    g_link.last_served_seq = 101;
    tx_pump();
    assert(tq_n == 0 && g_link.sent_begins == 1);
    q = &tq[(tq_head + tq_n++) % TQ_N];
    memset(q, 0, sizeof *q);
    q->seq = 1000; q->total = 1; q->body = calloc(1, 512);
    g_rd.stalled = 1;
    tx_pump();
    assert(tq_n == 0);
    /* a backward seq (reader ahead) is never held */
    q = &tq[(tq_head + tq_n++) % TQ_N];
    memset(q, 0, sizeof *q);
    q->seq = 50; q->total = 1; q->body = calloc(1, 512);
    g_rd.stalled = 0;
    tx_pump();
    assert(tq_n == 0);

    /* CONFIG: live stale limit, longer for a bootstrap image and for batch */
    g_link.olen = g_link.opos = 0;
    send_config();
    tl_config c; memcpy(&c, g_link.out + TL_HDR, sizeof c);
    assert(c.stale_ms == STALE_LIVE_MS);
    g_link.olen = g_link.opos = 0; g_img_stale = 1;
    send_config(); memcpy(&c, g_link.out + TL_HDR, sizeof c);
    assert(c.stale_ms == STALE_BATCH_MS);
    g_link.olen = g_link.opos = 0; g_img_stale = 0; O.k = 48;
    send_config(); memcpy(&c, g_link.out + TL_HDR, sizeof c);
    assert(c.stale_ms == STALE_BATCH_MS);
    O.k = 0;

    /* whole_sf: every CONFIG byte stays zero unless asked; the control FIFO switches it live */
    memcpy(&c, g_link.out + TL_HDR, sizeof c);
    assert(c.whole_sf == 0 && !c.rsv[0]);
    char cp[80]; snprintf(cp, sizeof cp, "%s/c", dir);
    assert(mkfifo(cp, 0600) == 0);
    g_ctl = open(cp, O_RDWR | O_NONBLOCK);
    assert(g_ctl >= 0 && write(g_ctl, "WSF1", 4) == 4);
    sdctl_poll();
    assert(g_whole_sf == 2 && g_cfg_dirty);
    g_link.olen = g_link.opos = 0;
    send_config(); memcpy(&c, g_link.out + TL_HDR, sizeof c);
    assert(c.whole_sf == 2);
    assert(write(g_ctl, "WSF0", 4) == 4);
    sdctl_poll();
    assert(g_whole_sf == 1);
    close(g_ctl); g_ctl = -1; unlink(cp);

    close(wfd); unlink(path); rmdir(dir);
    puts("PASS frame limit, discard keeps framing, key gate, fragments, reader tracking and return key, away drop, reader window, stale limits");
    return 0;
}
