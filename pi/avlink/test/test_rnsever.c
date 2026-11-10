/* test_rnsever.c -- the RNS-E version announcement on the Pi side (docs/PROTOCOL.md 4.5), no device needed. Build from pi/avlink:
 *   cc -std=c11 -Wall -Wextra -Werror -O2 test/test_rnsever.c link.c -o /tmp/rnsever-test
 * Checks: RNSE_INFO parsing in link.c (valid, short, wrong version, a longer future layout), format choice (rnse_ver.h) for an equal,
 * older, newer and overlapping unit, the log lines, the verdict, and the --rnse-file contents with and without an announcement. */
#define _DEFAULT_SOURCE
#include "../link.h"
#include "../rnse_ver.h"
#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static int fails, peer;
#define CHECK(c, m) do { if (c) printf("ok   %s\n", m); else { printf("FAIL %s\n", m); fails++; } } while (0)

static void put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

static tl_rnse_info info(uint32_t build, int fmt_min, int fmt, uint32_t fw_id, int table)
{
    tl_rnse_info r;
    memset(&r, 0, sizeof r);
    r.ver = 1; r.size = sizeof r; r.build_id = build; r.fmt = (uint8_t)fmt; r.fmt_min = (uint8_t)fmt_min; r.hs_ver = 1;
    r.fw_id = fw_id; r.table_ver = (uint16_t)table; r.check = 0x1234; r.count = 1;
    return r;
}

static void deliver(const void *pl, uint32_t len)
{
    uint8_t b[TL_HDR + 64];
    memset(b, 0, sizeof b);
    put32(b, TL_MAGIC); put16(b + 4, TL_RNSE_INFO); put32(b + 8, len);
    memcpy(b + TL_HDR, pl, len);
    assert(write(peer, b, TL_HDR + len) == (ssize_t)(TL_HDR + len));
}

static int file_has(const char *path, const char *line)
{
    char b[1024];
    FILE *f = fopen(path, "r");
    if (!f) return 0;
    size_t n = fread(b, 1, sizeof b - 1, f);
    fclose(f);
    b[n] = 0;
    return strstr(b, line) != NULL;
}

int main(void)
{
    assert(sizeof(tl_rnse_info) == 32 && sizeof(tl_hs_verdict) == 8);
    /* 1. link.c parses RNSE_INFO */
    link_t l;
    int fd[2];
    assert(link_open(&l, "/dev/null") == 0);
    close(l.fd);
    assert(socketpair(AF_UNIX, SOCK_STREAM, 0, fd) == 0);
    l.fd = l.rfd = fd[0]; peer = fd[1];
    assert(fcntl(l.rfd, F_SETFL, O_NONBLOCK) == 0);
    tl_rnse_info r = info(0xF46C419Du, 1, 1, 0x54B, 1);
    deliver(&r, sizeof r);
    link_read(&l);
    CHECK(l.n_rnse == 1 && !memcmp(&l.rnse, &r, sizeof r), "RNSE_INFO parsed");
    deliver(&r, 20);
    link_read(&l);
    CHECK(l.n_rnse == 1, "short RNSE_INFO ignored");
    tl_rnse_info bad = r; bad.ver = 2;
    deliver(&bad, sizeof bad);
    link_read(&l);
    CHECK(l.n_rnse == 1, "unknown RNSE_INFO version ignored");
    uint8_t big[40];
    memset(big, 0xAB, sizeof big);
    tl_rnse_info nx = info(0x01020304u, 1, 2, 0x64, 1); nx.size = 40;
    memcpy(big, &nx, sizeof nx);
    deliver(big, sizeof big);
    link_read(&l);
    CHECK(l.n_rnse == 2 && l.rnse.build_id == 0x01020304u && l.rnse.fmt == 2, "longer layout with the same 32-B head accepted");
    CHECK(!l.credit_error && !l.hup, "RNSE_INFO never touches credits");
    close(peer); link_close(&l);

    /* 2. format choice and log lines */
    rnse_ver_t v;
    char line[256];
    rnse_ver_set(&v, &r, 1, 1);
    rnse_ver_line(&v, 1, 1, line, sizeof line);
    CHECK(v.fmt == 1 && strstr(line, "RNS-E module build F46C419D fmt 1 table 54B/1") == line, "same format: streaming, owner's log line");
    rnse_ver_set(&v, &r, 2, 3);
    rnse_ver_line(&v, 2, 3, line, sizeof line);
    CHECK(v.fmt == 0 && strstr(line, "RNS-E module too old: build F46C419D fmt 1") == line, "older unit: none, 'too old'");
    tl_rnse_info n3 = info(0xAAAA0001u, 3, 4, 0x310, 1);
    rnse_ver_set(&v, &n3, 1, 2);
    rnse_ver_line(&v, 1, 2, line, sizeof line);
    CHECK(v.fmt == 0 && strstr(line, "RNS-E module too new: build AAAA0001 fmt 3-4 table 310/1") == line, "newer unit: none, 'too new'");
    tl_rnse_info n13 = info(0xAAAA0002u, 1, 3, 0x64, 1);
    rnse_ver_set(&v, &n13, 2, 4);
    CHECK(v.fmt == 3, "overlap 1-3 / 2-4: newest common = 3");
    tl_rnse_info z = info(1, 0, 0, 0, 1);
    CHECK(rnse_ver_choose(&z, 1, 1) == 0, "fmt 0 plays nothing");
    tl_rnse_info inv = info(1, 3, 2, 0, 1);
    CHECK(rnse_ver_choose(&inv, 1, 4) == 0, "fmt_min > fmt plays nothing");

    /* 3. verdicts */
    tl_hs_verdict vd;
    rnse_ver_set(&v, &r, 1, 1);
    rnse_ver_verdict(&v, &vd);
    CHECK(vd.build_id == 0xF46C419Du && vd.check == 0x1234 && vd.verdict == TL_HS_ACCEPT && vd.fmt == 1, "accept verdict");
    rnse_ver_set(&v, &r, 2, 2);
    rnse_ver_verdict(&v, &vd);
    CHECK(vd.verdict == TL_HS_REJECT && vd.fmt == 0, "reject verdict");

    /* 4. --rnse-file */
    char dir[] = "/tmp/rnsever_XXXXXX", path[64];
    assert(mkdtemp(dir));
    snprintf(path, sizeof path, "%s/rnse.txt", dir);
    rnse_ver_none(&v);
    rnse_ver_line(&v, 1, 1, line, sizeof line);
    CHECK(strstr(line, "no version announced") && v.fmt == AVL_FMT_DEFAULT, "nothing announced: today's format, logged");
    CHECK(rnse_ver_write(path, &v, NULL) == 0 && file_has(path, "RNSE_ANNOUNCED=0\n") && file_has(path, "STREAM_FMT=1\n") &&
          !file_has(path, "RNSE_BUILD=") && !file_has(path, "TEENSY_VERSION="), "file without an announcement");
    rnse_ver_set(&v, &r, 1, 1);
    CHECK(rnse_ver_write(path, &v, "") == 0 && file_has(path, "RNSE_ANNOUNCED=1\n") && file_has(path, "RNSE_BUILD=F46C419D\n") &&
          file_has(path, "RNSE_FMT=1\n") && file_has(path, "RNSE_FW_ID=54B\n") && file_has(path, "RNSE_TABLE=1\n") &&
          file_has(path, "STREAM_FMT=1\n") && !file_has(path, "TEENSY_VERSION="), "file with an announcement");
    rnse_ver_set(&v, &r, 2, 2);
    CHECK(rnse_ver_write(path, &v, NULL) == 0 && file_has(path, "STREAM_FMT=0\n"), "file after a reject: STREAM_FMT=0");
    CHECK(rnse_ver_write(path, &v, "sdemu 0.5 (2026-11-10, it's)") == 0 &&
          file_has(path, "TEENSY_VERSION='sdemu 0.5 (2026-11-10, its)'\n"), "Teensy version, single-quoted, quote dropped");
    unlink(path); rmdir(dir);

    /* 5. the Teensy's version from its console lines */
    static link_t lk;
    CHECK(!link_console_line(&lk, "LINK active=1 msgs=3") && !lk.teensy_ver[0], "other console lines carry no version");
    CHECK(link_console_line(&lk, "VER sdemu 0.5 (2026-11-10, touch) flexio2_param=00000000 f_cpu=600000000\r") &&
          !strcmp(lk.teensy_ver, "sdemu 0.5 (2026-11-10, touch)") && lk.n_teensy_ver == 1, "VER line: the version, up to its params");
    CHECK(link_console_line(&lk, "BOOT sdemu 0.5 (2026-11-10, touch)") && lk.n_teensy_ver == 1, "same version again: no change counted");
    CHECK(link_console_line(&lk, "HELP sdemu 0.6 (2026-12-01)") && !strcmp(lk.teensy_ver, "sdemu 0.6 (2026-12-01)") &&
          lk.n_teensy_ver == 2, "HELP line: a new version is counted");
    CHECK(!link_console_line(&lk, "VER garbage") && lk.n_teensy_ver == 2, "a VER line that is not sdemu is ignored");

    printf("%s: rnse version handshake (Pi side), %d failure(s)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
