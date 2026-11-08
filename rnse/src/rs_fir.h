/* Rate match: windowed-sinc interpolation.  Table in rs_fir_tab.h (enc3/tools/gen_fir.py).  Replaces a Catmull-Rom whose false tones sat only 15-25 dB below the signal
 * at 12-16 kHz, which is cymbals.  Coefficients are blended between adjacent table rows, once per frame.  Reads FIR_TAPS/2 - 1 frames behind and FIR_TAPS/2 ahead of the read position. */
#ifndef RS_FIR_H
#define RS_FIR_H
#include "rs_fir_tab.h"
#define FIR_BACK (FIR_TAPS / 2 - 1)
#define FIR_AHEAD (FIR_TAPS / 2 + 1)                     /* frames the FIFO must hold past the last position */
#define FIR_SH (16 - FIR_LGPH)
static inline u32 fir_frame(const u32 *fifo, u32 m, u32 k, u32 f)
{
    const short *a = fir_tab[f >> FIR_SH], *b = a + FIR_TAPS;
    s32 bl = (s32)(f & ((1u << FIR_SH) - 1)), al = (1 << FIR_SH) - bl, l = 16384, r = 16384, sum = 0;
    u32 p = k - FIR_BACK, w;
    int j;
    for (j = 0; j < FIR_TAPS; j++, p++) {
        s32 c = (a[j] * al + b[j] * bl + (1 << (FIR_SH - 1))) >> FIR_SH;
        w = fifo[p & m];
        sum += c;
        l += c * (s16)(w >> 16);
        r += c * (s16)w;
    }
    w = fifo[k & m];                                     /* what rounding left over goes on the nearest frame: gain stays exactly 1, no level ripple */
    l += (32768 - sum) * (s16)(w >> 16);
    r += (32768 - sum) * (s16)w;
    l >>= 15; r >>= 15;
    if (l > 32767) l = 32767; else if (l < -32768) l = -32768;
    if (r > 32767) r = 32767; else if (r < -32768) r = -32768;
    return ((u32)l << 16) | ((u32)r & 0xFFFF);
}
#endif
