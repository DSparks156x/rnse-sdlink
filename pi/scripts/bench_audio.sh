#!/bin/bash
# bench_audio.sh -- the car chain with no head unit: wlcap + encoder + avlink at its real tick (superframes to /dev/null), a steady tone played
# into Hudiy's sink, and a second recording of what the chain captures, checked for clock rate, gaps and clicks.  Starts nothing permanent,
# restarts nothing.  Only while the RNS-E is off (the real chain is then waiting for the Teensy).
#   bash bench_audio.sh [seconds, default 45] [tag]
SEC=${1:-45}; TAG=${2:-run}
export XDG_RUNTIME_DIR=/run/user/$(id -u) WAYLAND_DISPLAY=wayland-0
pgrep -x avlink >/dev/null && { echo "avlink already running: the real chain is up, not testing"; exit 1; }
W=$XDG_RUNTIME_DIR/benchaudio; rm -rf "$W"; mkdir -p "$W"; cd "$W" || exit 1
python3 - <<'E'
import math, struct, wave
f = wave.open('tone.wav', 'wb'); f.setnchannels(2); f.setsampwidth(2); f.setframerate(48000)
n = 48000 * 120; w = 2 * math.pi * 997 / 48000; b = bytearray()
for i in range(n):
    v = int(8000 * math.sin(w * i)); b += struct.pack('<hh', v, v)
f.writeframes(bytes(b)); f.close()
E
CAPSINK=hudiy_equalizer_sink; pactl list short sinks | awk '{print $2}' | grep -Fxq echo_cancel_sink && CAPSINK=echo_cancel_sink
pw-play --target hudiy_equalizer_sink tone.wav & PLAY=$!
sleep 1
timeout $SEC pw-record --target "$CAPSINK" -P '{stream.capture.sink=true}' --rate 48000 --channels 2 --format s16 --latency 20ms --raw - > tap.raw 2>/dev/null &
T0=$(date +%s.%N)
LINKDEV="--dump-sf /dev/null" CHAINDIR=$W NORANDR=1 STATS=5 timeout $((SEC + 2)) bash $HOME/avsd/current/chain/hud2.sh $SEC &
HUD=$!
sleep 3
top -b -d $((SEC - 8)) -n 2 -H -o %CPU | awk '/^top -/{n++} n==2' | sed -n 1,25p | cut -c1-120 > top.txt &
wait $HUD; T1=$(date +%s.%N)
kill $PLAY 2>/dev/null; sleep 1
echo "== $TAG: driver and xruns at end"
pw-top -b -n 2 2>/dev/null | awk '/^S +ID/{n++} n==2' | awk '$1=="R" || NR==1' | cut -c1-110
echo "== avlink"
grep -E '^(STAT|END)' live.log | tail -2 | grep -oE '(late|adrop|silence|adj|audio_in|raw)=-?[0-9]+' | tr '\n' ' '; echo
echo "== encoder"; grep -E 'SUMMARY|enc ms' enc.log | tail -1 | cut -c1-200
echo "== top threads"; cat top.txt
echo "== tap recording"
python3 - tap.raw "$T0" "$T1" <<'E'
import sys, struct, math
d = open(sys.argv[1], 'rb').read(); n = len(d) // 4
x = struct.unpack('<%dh' % (2 * n), d[:4 * n])[0::2]
c = 2 * math.cos(2 * math.pi * 997 / 48000)
# a pure tone obeys x[i+1] + x[i-1] = c x[i]; a dropped, repeated or padded stretch breaks it
ev, last, zr, z = [], -10**9, 0, 0
for i in range(1, n - 1):
    r = x[i + 1] + x[i - 1] - c * x[i]
    if abs(r) > 400 and i - last > 480: ev.append(i); last = i
    if x[i] == 0 and x[i - 1] == 0: z += 1
    elif z: zr = max(zr, z); z = 0
first = next((i for i in range(n) if abs(x[i]) > 1000), 0)
dur = n / 48000
print('frames %d (%.2f s at 48 kHz), tone from %.2f s, discontinuities %d, longest zero run %.1f ms' % (n, dur, first / 48000, len([e for e in ev if e > first + 480]), zr / 48))
print('discontinuities at s:', ' '.join('%.2f' % (e / 48000) for e in ev if e > first + 480)[:600])
E
