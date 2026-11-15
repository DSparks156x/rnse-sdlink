/* audio.h -- the audio stage: the only code that produces audio sector bytes (SPEC 3.2 audio_fmt 1 = PCM s16
 * big-endian stereo, 48 kHz). Input: 48 kHz stereo PCM on a pipe or file: s16 little-endian (ALSA S16_LE, default), s16 big-endian
 * (--audio-be) or float32 little-endian (--audio-f32, PipeWire's own format: no conversion before ours). Missing input -> silence (counted).
 *
 * Since 2026-11-14 the stage works in float: audio_take() hands out float frames, the converter (asrc.c) runs on them, and
 * audio_quantize() is the single step to s16: gain, TPDF dither (+-1 LSB triangular; none on exact digital silence or on a value that is already an exact s16 step, e.g. unity gain), saturation
 * (the old Q15 gain wrapped around above 0 dBFS).
 *
 * HOOK: the Teensy will later be a UAC2 sound card and insert audio itself. Then this stage is replaced by
 * "reserve N audio sectors and let the Teensy fill them + patch the header" (audio_stage_take() becomes a
 * no-op and sf_build() is fed audio_bytes with a NULL buffer). Nothing else in avlink touches audio bytes. */
#ifndef AVL_AUDIO_H
#define AVL_AUDIO_H
#include <stdint.h>
#include <stddef.h>

#define AUDIO_PRIME_MS 50
#define AUDIO_GAIN_MAX_DB (-3.0)   /* the DSP's IIS input scaler may add ~+2.5 dB (DEPLOYMENT.md "Audio gain"): keep this much headroom */
typedef struct {
    int fd;              /* -1: no input (silence) */
    int be;              /* s16 input is big-endian */
    int f32;             /* float32 LE input */
    int fb;              /* input bytes per frame (4 or 8) */
    int eof;
    int nonblock;        /* pipe in live mode */
    uint8_t *buf;        /* raw input bytes not consumed yet */
    size_t have, cap;
    uint64_t frames_in, frames_silence, frames_dropped;
    float gain;          /* linear, applied in audio_quantize */
    int dither;          /* TPDF on (default) */
    uint64_t rng;
    uint64_t clipped;    /* samples saturated in audio_quantize */
    int primed;     /* live: margin reached, see audio_take */
    uint64_t raw_bytes;  /* everything read from the input */
    int last_arrival;    /* set by a read, cleared by the tick clock: audio is flowing */
} audio_stage;

int audio_open(audio_stage *a, const char *path, int be, int f32, int live);   /* path NULL = silence */
int audio_poll_fd(const audio_stage *a);        /* fd to poll for POLLIN in live mode, or -1 */
void audio_read_avail(audio_stage *a);          /* live: read what the pipe has (nonblocking) */
static inline size_t audio_have_frames(const audio_stage *a) { return a->have / (size_t)a->fb; }
/* Produce exactly `frames` stereo float frames (interleaved, full scale +-1.0) into out. blocking: wait for the input
 * (batch); else take what is buffered and pad with silence. Returns frames that came from the input. */
int audio_take(audio_stage *a, int frames, float *out, int blocking);
/* gain + TPDF dither + round + saturate: n float frames -> s16 big-endian stereo (n * 4 bytes) */
void audio_quantize(audio_stage *a, const float *in, int n, uint8_t *out);
void audio_set_gain_db(audio_stage *a, double db);
void audio_close(audio_stage *a);

#endif
