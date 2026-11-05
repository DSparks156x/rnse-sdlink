/* Deterministic synthetic 800x480 RGB24 test clip for `make test` (no test data needed on the Pi):
 * static rail + scrolling list (vertical scroll, varying speed) + panning "map" (diagonal pan, half-pel-ish
 * content) + a fading card + a noise patch (natural-video-like) + a hard cut at frame N/2.
 * Usage: synth N [bgr0] > clip.raw   (bgr0: XRGB8888 little-endian byte order instead of RGB24) */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#define W 800
#define H 480
static uint32_t rs = 1;
static uint32_t rnd(void) { rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5; return rs; }

static void map_px(int x, int y, uint8_t *p) { /* procedural map: park blocks, roads, labels */
    int gx = ((x % 97) + 97) % 97, gy = ((y % 71) + 71) % 71;
    uint8_t r = 236, g = 232, b = 224;
    if (((x / 97 + y / 71) & 3) == 0) { r = 170; g = 214; b = 160; }
    if (gx < 5 || gy < 4) { r = 255; g = 255; b = 255; }
    if (gx < 2) { r = 250; g = 190; b = 60; }
    if (((x * 7 + y * 3) / 13) % 41 == 0) { r = 70; g = 120; b = 230; }
    p[0] = r; p[1] = g; p[2] = b;
}
int main(int argc, char **argv) {
    int n = argc > 1 ? atoi(argv[1]) : 60, bgr0 = argc > 2, f, x, y;
    static uint8_t fr[W * H * 3], noise[200 * 120 * 3], list[2400 * 560 * 3];
    for (y = 0; y < 2400; y++) for (x = 0; x < 560; x++) { /* list rows with "text" */
        uint8_t *p = list + (y * 560 + x) * 3; int row = y / 80, ry = y % 80, v = 20;
        if (ry > 20 && ry < 34 && x > 90 && x < 90 + 40 * (row % 9 + 3) && ((x * 13 + ry * 7 + row) % 5) < 3) v = 230;
        if (ry > 44 && ry < 54 && x > 90 && x < 300 && ((x * 11 + ry * 5) % 4) < 2) v = 150;
        if (x > 20 && x < 70 && ry > 15 && ry < 65) { p[0] = (uint8_t)(row * 37); p[1] = (uint8_t)(200 - row * 23); p[2] = (uint8_t)(row * 61); continue; }
        p[0] = p[1] = p[2] = (uint8_t)v;
    }
    for (f = 0; f < n; f++) {
        double t = f / 60.0; int scroll = (int)(600 * (1 - exp(-3 * t))) + (f > n / 2 ? 200 : 0);
        int panx = (int)(f * 3.5), pany = (int)(f * 1.5), cut = f >= n / 2;
        for (y = 0; y < H; y++) for (x = 0; x < W; x++) {
            uint8_t *p = fr + (y * W + x) * 3;
            if (x < 72) { p[0] = p[1] = 16; p[2] = 18; if ((x - 36) * (x - 36) + ((y % 80) - 40) * ((y % 80) - 40) < 300) { p[0] = 138; p[1] = 180; p[2] = 248; } }
            else if (!cut && x < 72 + 560 / 2 + 140) { int ly = (y + scroll) % 2400, lx = (x - 72) % 560; uint8_t *q = list + (ly * 560 + lx) * 3; p[0] = q[0]; p[1] = q[1]; p[2] = q[2]; }
            else map_px(x + panx + (cut ? 333 : 0), y + pany, p);
        }
        { /* fading card */
            int a = (int)(255 * (0.5 + 0.5 * sin(t * 3)));
            for (y = 300; y < 420; y++) for (x = 450; x < 760; x++) { uint8_t *p = fr + (y * W + x) * 3; int k; for (k = 0; k < 3; k++) p[k] = (uint8_t)((p[k] * (255 - a) + 48 * a) / 255); }
        }
        if (f % 20 < 10) { /* noise patch for 10 of every 20 frames */
            for (y = 0; y < 120 * 200 * 3; y++) noise[y] = (uint8_t)(rnd() >> 24);
            for (y = 0; y < 120; y++) for (x = 0; x < 200; x++) { uint8_t *p = fr + ((y + 20) * W + x + 560) * 3, *q = noise + (y * 200 + x) * 3; p[0] = q[0]; p[1] = q[1]; p[2] = q[2]; }
        }
        if (bgr0) { static uint8_t o[W * H * 4]; int i; for (i = 0; i < W * H; i++) { o[4 * i] = fr[3 * i + 2]; o[4 * i + 1] = fr[3 * i + 1]; o[4 * i + 2] = fr[3 * i]; o[4 * i + 3] = 0xAA; } fwrite(o, 1, sizeof o, stdout); }
        else fwrite(fr, 1, sizeof fr, stdout);
    }
    return 0;
}
