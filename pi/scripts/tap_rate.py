#!/usr/bin/env python3
"""tap_rate.py -- the rate PipeWire really delivers at the chain's audio tap, against the Pi clock, independent of content.
   python3 tap_rate.py [seconds=60]
Same pw-record command as hud2.sh. Prints frames/s (least-squares slope after a 5 s warm-up), ppm from 48000, the driver node, and
reads that stalled over 40 ms. avlink's tick can follow +-0.72% (+-7200 ppm); beyond that the chain pads silence or drops audio."""
import os, subprocess, sys, time
sec = float(sys.argv[1]) if len(sys.argv) > 1 else 60
sinks = subprocess.run(['pactl', 'list', 'short', 'sinks'], capture_output=True, text=True).stdout.split()
cap = 'echo_cancel_sink' if 'echo_cancel_sink' in sinks else 'hudiy_equalizer_sink'
p = subprocess.Popen(['pw-record', '--target', cap, '-P', '{stream.capture.sink=true}', '--rate', '48000', '--channels', '2', '--format', 's16',
                      '--latency', '20ms', '--raw', '-'], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
fd = p.stdout.fileno()
t0 = time.monotonic(); total = 0; pts = []; last = None; stalls = []
while True:
    b = os.read(fd, 65536)
    now = time.monotonic()
    if not b: break
    if last is not None and now - last > 0.040 and now - t0 > 5: stalls.append((round(now - t0, 2), round((now - last) * 1000)))
    last = now; total += len(b) // 4
    if now - t0 > 5: pts.append((now, total))
    if now - t0 > sec: break
p.terminate()
n = len(pts); mt = sum(t for t, _ in pts) / n; mf = sum(f for _, f in pts) / n
slope = sum((t - mt) * (f - mf) for t, f in pts) / sum((t - mt) ** 2 for t, _ in pts)
print('tap %s: %.1f frames/s, %+.0f ppm from 48000, over %.0f s' % (cap, slope, (slope / 48000 - 1) * 1e6, pts[-1][0] - pts[0][0]))
print('reads stalled > 40 ms: %d %s' % (len(stalls), stalls[:10]))
out = subprocess.run(['pw-top', '-b', '-n', '2'], capture_output=True, text=True, timeout=10).stdout.split('\n')
blk = [i for i, l in enumerate(out) if l.lstrip().startswith('S') and 'ID' in l]
drivers = [l for l in out[blk[-1]:] if l.startswith('R') and '+' not in l]
print('drivers now:'); print('\n'.join(l[:110] for l in drivers))
