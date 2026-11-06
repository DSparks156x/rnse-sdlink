#!/usr/bin/env python3
"""live_gen.py -- test sources for test_live.py, no encoder needed.
  live_gen.py video SIZE FPS KEYSIZE KEYFIFO [KEYEVERY_S] > pipe    .rnv stream like rnv2enc: length (top bit = key) + frame. Each frame starts with its encode time and
      frame number. The write blocks when the pipe is full and the next frame is taken after that, like wlcap's newest-wins buffer. "key" on KEYFIFO:
      the next frame is a key, as rnv2enc --linkctl. Frame 0 is not flagged, as rnv2enc. Every frame is numbered, so a missing delta is visible at the reader.
  live_gen.py audio FIFO                               48 kHz s16 stereo in 5 ms bursts, each sample pair holds its own sample clock.
"""
import os, struct, sys, time
CL = time.CLOCK_MONOTONIC
def now(): return time.clock_gettime_ns(CL) // 1000
if sys.argv[1] == 'video':
    size, fps, keysize = int(sys.argv[2]), float(sys.argv[3]), int(sys.argv[4])
    kf = os.open(sys.argv[5], os.O_RDONLY | os.O_NONBLOCK)
    out = sys.stdout.buffer.raw
    per = 1e6 / fps; nx = now(); no = 0; key = False
    every = float(sys.argv[6]) * 1e6 if len(sys.argv) > 6 else 0; lastk = now()   # periodic key, as rnv2enc --keyint
    while True:
        t = now()
        if t < nx:
            time.sleep((nx - t) / 1e6); t = now()
        try:
            if b'key' in os.read(kf, 256): key = True
        except BlockingIOError:
            pass
        if every and t - lastk >= every: key = True
        if key: lastk = t
        n = keysize if key else size
        hdr = struct.pack('>IQII', n | (0x80000000 if key else 0), t, no, 1 if key else 0)
        m = memoryview(hdr + bytes(n - 16))
        while len(m):
            m = m[out.write(m):]
        no += 1; key = False
        nx += per
        if now() > nx + per: nx = now()
else:
    f = open(sys.argv[2], 'wb', buffering=0); last = now() * 48 // 1000
    while True:
        time.sleep(0.005); n = now() * 48 // 1000; buf = bytearray()
        for i in range(last, n):
            j = i & 0xffffffff
            buf += struct.pack('<HH', j & 0xffff, j >> 16)
        last = n; f.write(buf)
