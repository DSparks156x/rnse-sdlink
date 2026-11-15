/* audio.c -- see audio.h */
#include "audio.h"
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

int audio_open(audio_stage *a, const char *path, int be, int f32, int live)
{
    memset(a, 0, sizeof *a);
    a->fd = -1;
    a->be = be;
    a->f32 = f32;
    a->fb = f32 ? 8 : 4;
    a->gain = 1.0f;
    a->dither = 1;
    a->rng = 0x9E3779B97F4A7C15ull;
    a->cap = (size_t)32768 * (size_t)a->fb;      /* 683 ms: must stay above the HIGH mark in audio_take */
    a->buf = malloc(a->cap);
    if (!a->buf) return -1;
    if (!path) return 0;
    a->fd = strcmp(path, "-") ? open(path, O_RDONLY) : 0;
    if (a->fd < 0) return -1;
    struct stat st;
    if (live && fstat(a->fd, &st) == 0 && !S_ISREG(st.st_mode)) {
        fcntl(a->fd, F_SETFL, fcntl(a->fd, F_GETFL) | O_NONBLOCK);
        a->nonblock = 1;
    }
    return 0;
}

void audio_set_gain_db(audio_stage *a, double db) { a->gain = (float)pow(10.0, db / 20.0); }

int audio_poll_fd(const audio_stage *a) { return (a->fd >= 0 && a->nonblock && !a->eof) ? a->fd : -1; }

static int read_some(audio_stage *a, size_t want)
{
    if (a->fd < 0 || a->eof) return 0;
    if (a->have + want > a->cap) want = a->cap - a->have;
    if (!want) return 0;
    ssize_t r = read(a->fd, a->buf + a->have, want);
    if (r > 0) { a->have += (size_t)r; a->raw_bytes += (uint64_t)r; a->last_arrival = 1; return (int)r; }
    if (r == 0 || (errno != EAGAIN && errno != EINTR)) a->eof = 1;
    return 0;
}

void audio_read_avail(audio_stage *a)
{
    while (a->nonblock) {
        if (a->have == a->cap) {                 /* no superframes going out (reader away, or USB behind): keep the newest half. Leaving the FIFO unread */
            size_t drop = (a->cap / 2) / (size_t)a->fb * (size_t)a->fb;   /* here would make poll() return at once, forever: 75% of a core, measured */
            memmove(a->buf, a->buf + drop, a->have - drop);
            a->have -= drop;
            a->frames_dropped += drop / (size_t)a->fb;
        }
        if (read_some(a, a->cap - a->have) <= 0) break;
    }
}

static inline float s16f(int v) { return (float)v * (1.0f / 32768.0f); }

int audio_take(audio_stage *a, int frames, float *out, int blocking)
{
    const size_t fb = (size_t)a->fb, need = (size_t)frames * fb;
    if (a->fd >= 0) {
        if (blocking || !a->nonblock) while (a->have < need && !a->eof) read_some(a, need - a->have);
        else {
            audio_read_avail(a);
            /* PipeWire hands audio over in bursts (256-1440 frames), the tick takes a fixed 800. With no margin the tick found too little, padded the rest
             * with silence, and later dropped the backlog: ~20 audible gaps per 30 s, in multiples of 16 frames (bench 2026-11-05, 1 kHz through the chain).
             * So: hold a margin (PRIME). Only a real shortage re-primes. Over HIGH the excess goes, back to the margin: ticks are skipped while the
             * reader is away, and the delay must not grow.
             * HIGH was 60 ms, which threw away 60-280 ms of music, with a click, each time a busy screen held the reader back (car log 2026-11-06). The reader's own FIFO rides out such a stall
             * if the audio is kept: under 250 ms it stays and the tick clock drains it. */
            size_t prime = 48 * AUDIO_PRIME_MS * fb, high = prime + 48 * 250 * fb;
            if (!a->primed) {
                if (a->have >= need + prime) a->primed = 1;
                else {
                    memset(out, 0, (size_t)frames * 2 * sizeof(float));
                    a->frames_silence += (uint64_t)frames;
                    return 0;
                }
            }
            if (a->have > need + high) {
                size_t drop = (a->have - need - prime) / fb * fb;
                memmove(a->buf, a->buf + drop, a->have - drop);
                a->have -= drop;
                a->frames_dropped += drop / fb;
            }
            if (a->have < need) a->primed = 0;
        }
    }
    size_t got = (a->have < need ? a->have : need) / fb;   /* frames */
    const uint8_t *b = a->buf;
    if (a->f32) memcpy(out, b, got * 8);                    /* LE float, host LE */
    else if (a->be) for (size_t i = 0; i < got * 2; i++) out[i] = s16f((int16_t)((uint16_t)b[2 * i] << 8 | b[2 * i + 1]));
    else for (size_t i = 0; i < got * 2; i++) out[i] = s16f((int16_t)((uint16_t)b[2 * i + 1] << 8 | b[2 * i]));
    if (a->f32)                                              /* a NaN or Inf from upstream must not reach the converter's history */
        for (size_t i = 0; i < got * 2; i++) if (!isfinite(out[i])) out[i] = 0;
    memset(out + got * 2, 0, ((size_t)frames - got) * 2 * sizeof(float));
    memmove(a->buf, a->buf + got * fb, a->have - got * fb);
    a->have -= got * fb;
    a->frames_in += got;
    a->frames_silence += (uint64_t)frames - got;
    return (int)got;
}

/* xorshift64*: uniform in [0, 1) with 24 bits */
static inline float urand(uint64_t *s)
{
    uint64_t x = *s;
    x ^= x >> 12; x ^= x << 25; x ^= x >> 27;
    *s = x;
    return (float)((x * 0x2545F4914F6CDD1Dull) >> 40) * (1.0f / 16777216.0f);
}

void audio_quantize(audio_stage *a, const float *in, int n, uint8_t *out)
{
    const float g = a->gain * 32768.0f;
    for (int i = 0; i < 2 * n; i++) {
        float x = in[i];
        int v = 0;
        if (x != 0.0f) {                                     /* exact digital silence stays silent: no dither hiss between tracks */
            float y = x * g, r = floorf(y + 0.5f);
            if (a->dither && r != y) r = floorf(y + urand(&a->rng) - urand(&a->rng) + 0.5f);   /* TPDF, +-1 LSB peak; an exact s16 value (unity gain) needs none */
            if (r > 32767.0f) { r = 32767.0f; a->clipped++; }
            else if (r < -32768.0f) { r = -32768.0f; a->clipped++; }
            v = (int)r;
        }
        out[2 * i] = (uint8_t)((uint16_t)v >> 8);
        out[2 * i + 1] = (uint8_t)v;
    }
}

void audio_close(audio_stage *a)
{
    if (a->fd > 0) close(a->fd);
    free(a->buf);
    a->buf = NULL;
}
