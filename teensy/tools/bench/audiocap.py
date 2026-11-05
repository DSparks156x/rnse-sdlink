r"""audiocap.py -- record from a Windows audio input (winmm, no packages) and report tone levels.

    audiocap.py list
    audiocap.py rec --dev 0 --secs 10 --out cap.wav [--tones 1000,1500] [--rate 48000]
    audiocap.py wav cap.wav [--tones 1000,1500]

Prints RMS, peak and Goertzel levels (dB relative to full scale of the capture) per second, plus a noise floor at
off-tone probe frequencies. A tone is called present when it is >= 15 dB above the off-tone probes. Writes a mono WAV.
"""
import argparse, ctypes, math, struct, sys, time, wave
from ctypes import wintypes

winmm = ctypes.WinDLL('winmm')
WAVE_FORMAT_PCM, WHDR_DONE, CALLBACK_NULL = 1, 1, 0


class WAVEFORMATEX(ctypes.Structure):
    _fields_ = [('wFormatTag', wintypes.WORD), ('nChannels', wintypes.WORD), ('nSamplesPerSec', wintypes.DWORD),
                ('nAvgBytesPerSec', wintypes.DWORD), ('nBlockAlign', wintypes.WORD), ('wBitsPerSample', wintypes.WORD),
                ('cbSize', wintypes.WORD)]


class WAVEHDR(ctypes.Structure):
    _fields_ = [('lpData', ctypes.c_void_p), ('dwBufferLength', wintypes.DWORD), ('dwBytesRecorded', wintypes.DWORD),
                ('dwUser', ctypes.c_size_t), ('dwFlags', wintypes.DWORD), ('dwLoops', wintypes.DWORD),
                ('lpNext', ctypes.c_void_p), ('reserved', ctypes.c_size_t)]


class WAVEINCAPSW(ctypes.Structure):
    _fields_ = [('wMid', wintypes.WORD), ('wPid', wintypes.WORD), ('vDriverVersion', wintypes.UINT),
                ('szPname', ctypes.c_wchar * 32), ('dwFormats', wintypes.DWORD), ('wChannels', wintypes.WORD),
                ('wReserved1', wintypes.WORD)]


def devices():
    out = []
    for i in range(winmm.waveInGetNumDevs()):
        c = WAVEINCAPSW()
        if winmm.waveInGetDevCapsW(i, ctypes.byref(c), ctypes.sizeof(c)) == 0:
            out.append((i, c.szPname, c.wChannels))
    return out


def record(dev, secs, rate):
    fmt = WAVEFORMATEX(WAVE_FORMAT_PCM, 1, rate, rate * 2, 2, 16, 0)
    h = wintypes.HANDLE()
    r = winmm.waveInOpen(ctypes.byref(h), dev, ctypes.byref(fmt), 0, 0, CALLBACK_NULL)
    if r:
        sys.exit('waveInOpen failed, error %d (is the device busy, or mic access off in Windows privacy settings?)' % r)
    n = int(secs * rate) * 2
    buf = ctypes.create_string_buffer(n)
    hdr = WAVEHDR(ctypes.cast(buf, ctypes.c_void_p), n, 0, 0, 0, 0, None, 0)
    sz = ctypes.sizeof(hdr)
    winmm.waveInPrepareHeader(h, ctypes.byref(hdr), sz)
    winmm.waveInAddBuffer(h, ctypes.byref(hdr), sz)
    winmm.waveInStart(h)
    end = time.time() + secs + 2
    while not hdr.dwFlags & WHDR_DONE and time.time() < end:
        time.sleep(0.05)
    winmm.waveInReset(h)
    winmm.waveInUnprepareHeader(h, ctypes.byref(hdr), sz)
    winmm.waveInClose(h)
    return buf.raw[:hdr.dwBytesRecorded]


def goertzel(x, f, rate):
    w = 2 * math.pi * f / rate
    c = 2 * math.cos(w)
    a = b = 0.0
    for v in x:
        a, b = v + c * a - b, a
    return 2 * math.sqrt(max(a * a + b * b - c * a * b, 0.0)) / max(len(x), 1)


def db(v):
    return 20 * math.log10(max(v, 1e-7))


def analyse(pcm, rate, tones):
    x = [s / 32768 for s in struct.unpack('<%dh' % (len(pcm) // 2), pcm[:len(pcm) // 2 * 2])]
    probes = [f + d for f in tones for d in (-130, 170)]
    verdict = {f: 0 for f in tones}
    secs = len(x) // rate
    for s in range(secs):
        p = x[s * rate:(s + 1) * rate]
        dc = sum(p) / len(p)
        p = [v - dc for v in p]
        rms = math.sqrt(sum(v * v for v in p) / len(p))
        floor = max(goertzel(p, f, rate) for f in probes)
        lv = {f: goertzel(p, f, rate) for f in tones}
        for f in tones:
            if db(lv[f]) - db(floor) >= 15:
                verdict[f] += 1
        print('  %3d s: rms %6.1f dB peak %6.1f dB | %s | off-tone %6.1f dB' % (
            s, db(rms), db(max(abs(v) for v in p)), ' '.join('%d Hz %6.1f' % (f, db(lv[f])) for f in tones), db(floor)))
    print('TONES: ' + ', '.join('%d Hz present in %d/%d s' % (f, verdict[f], secs) for f in tones))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=('list', 'rec', 'wav'))
    ap.add_argument('path', nargs='?')
    ap.add_argument('--dev', type=int, default=0)
    ap.add_argument('--secs', type=float, default=10)
    ap.add_argument('--rate', type=int, default=48000)
    ap.add_argument('--out', default='cap.wav')
    ap.add_argument('--tones', default='1000,1500')
    a = ap.parse_args()
    tones = [int(t) for t in a.tones.split(',')]
    if a.cmd == 'list':
        for d in devices():
            print('%d: %s (%d ch)' % d)
        return
    if a.cmd == 'wav':
        with wave.open(a.path) as w:
            analyse(w.readframes(w.getnframes()), w.getframerate(), tones)
        return
    pcm = record(a.dev, a.secs, a.rate)
    with wave.open(a.out, 'wb') as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(a.rate); w.writeframes(pcm)
    print('recorded %.1f s to %s' % (len(pcm) / 2 / a.rate, a.out))
    analyse(pcm, a.rate, tones)


if __name__ == '__main__':
    main()
