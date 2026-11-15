r"""ab_asrc.py -- music through the Pi ASRC (and the full Pi audio stage) against an ideal resample. Offline, no hardware.

  <python> -I ab_asrc.py SRC.wav ASRC_OUT.f32 [OLD_RNSE.wav]

SRC.wav: 48 kHz s16 stereo music (the parent session's B_after_pi_gain.wav: phone capture after the old Pi gain).
ASRC_OUT.f32: test/asrc_test wav SRC.wav OUT.f32 0.99987793 (step 8191/8192: 122 ppm, the same ratio as the old RNS-E resampler test).
The reference is scipy resample_poly(8192, 8191) with a 64-zero-crossing kaiser-sinc (beta 14), float64. Reports the in-band error of
  1. the ASRC alone (float out),
  2. the Pi's whole stage: ASRC -> -3 dB gain -> TPDF dither -> s16 (re the same gain applied to the reference, in float),
  3. optionally the old RNS-E resampler capture (C_after_rnse_resampler.wav) by the same metric.
"""
import sys
import numpy as np
import wave
from scipy.signal import resample_poly, butter, sosfiltfilt, welch


def rd(p):
    w = wave.open(p)
    return np.frombuffer(w.readframes(w.getnframes()), '<i2').reshape(-1, 2).astype(np.float64) / 32768.0


def align(ref, out, maxlag=200):
    """integer lag of out against ref (out[k] ~ ref[k + lag])"""
    n = min(len(ref), len(out), 48000 * 20)
    a, b = ref[48000:n, 0], out[48000:n, 0]
    best, lag = None, 0
    for L in range(-maxlag, maxlag + 1):
        x = a[max(0, L):len(a) + min(0, L)]
        y = b[max(0, -L):len(b) - max(0, L)]
        m = min(len(x), len(y))
        e = np.mean((x[:m] - y[:m]) ** 2)
        if best is None or e < best:
            best, lag = e, L
    return lag


def report(name, ref, out, gain=1.0):
    lag = align(ref, out)
    if lag >= 0:
        r, o = ref[lag:], out
    else:
        r, o = ref, out[-lag:]
    m = min(len(r), len(o)) - 200
    r, o = r[2000:m] * gain, o[2000:m]
    sos = butter(10, 20000, fs=48000, output='sos')
    e = sosfiltfilt(sos, o - r, axis=0)[2000:-2000]
    s = sosfiltfilt(sos, r, axis=0)[2000:-2000]
    pe, ps = (e ** 2).mean(), (s ** 2).mean()
    print('%s (lag %d): in-band (<20 kHz) error %.1f dBFS rms, signal %.1f dBFS -> %.1f dB below the signal'
          % (name, lag, 10 * np.log10(pe), 10 * np.log10(ps), 10 * np.log10(pe / ps)))
    fq, pee = welch(e[:, 0], 48000, nperseg=8192)
    _, pss = welch(s[:, 0], 48000, nperseg=8192)
    bands = [(20, 500), (500, 2000), (2000, 6000), (6000, 10000), (10000, 14000), (14000, 18000), (18000, 20000)]
    print('   ' + '  '.join('%d-%dk %.0f' % (lo // 1000, hi // 1000, 10 * np.log10(pee[(fq >= lo) & (fq < hi)].sum() / pss[(fq >= lo) & (fq < hi)].sum()))
                          for lo, hi in bands) + '  (dB below signal per band)')
    return 10 * np.log10(pe)


def main():
    src = rd(sys.argv[1])
    # reference: resample_poly with a long kaiser-sinc (64 zero crossings per side, cutoff at the input Nyquist): flat to ~23 kHz.
    # (scipy's default, 10 zero crossings, already rolls off from ~16 kHz and made the old comparison stop at 18 kHz.)
    from scipy.signal import firwin
    L = 8192
    h = firwin(2 * 64 * L + 1, 1.0 / L, window=('kaiser', 14.0))   # resample_poly scales it by up itself
    ref = np.stack([resample_poly(src[:, ch], L, L - 1, window=h) for ch in (0, 1)], 1)
    a = np.fromfile(sys.argv[2], '<f4').reshape(-1, 2).astype(np.float64)
    print('source %.1f s' % (len(src) / 48000))
    report('1. Pi ASRC alone, float', ref, a)
    # 2. the Pi stage as avlink runs it: gain -3 dB in float, TPDF +-1 LSB, round, saturate
    g = 10 ** (-3 / 20)
    rng = np.random.default_rng(1)
    y = a * g * 32768
    d = rng.random(y.shape) - rng.random(y.shape)
    q = np.where(y == 0, 0, np.clip(np.floor(y + d + 0.5), -32768, 32767)) / 32768
    e2 = report('2. Pi stage: ASRC + -3 dB + TPDF + s16', ref, q, g)
    print('   (TPDF +-1 LSB dither + rounding alone: %.1f dBFS rms; white, so in band 20k/24k of it)' % (10 * np.log10(3 / 12 / 32768 ** 2 * 20 / 24)))
    if len(sys.argv) > 3:
        c = rd(sys.argv[3])
        report('3. old RNS-E resampler capture (rs_fir, 24 taps fixed point)', ref, c)


if __name__ == '__main__':
    main()
