#!/bin/bash
# Hudiy screen + Hudiy audio -> RNS-E
# Env: KEYINT (periodic key safety net, frames), LINKCAP (B/s), MAXVID (video sectors per superframe: must read in under a tick, so audio
# keeps 60 superframes/s even when video lags), GAIN (dB, -18 = the stock MP3 level), ENCTHREADS.
# CAPMODE: dmabuf (default: labwc hands over its buffer, no readback on the labwc thread) or shm (labwc reads back).
# RTPRIO: SCHED_FIFO priority of the encoder, 0 = off. wlcap gets one less, avlink one more. Needs passwordless sudo; without it the chain just runs at normal priority.
# Test without the Teensy: LINKDEV="--dump-sf /dev/null" CHAINDIR=/some/dir NORANDR=1 WLCAP_BIN=... AVLINK_BIN=... hud2.sh SECS
export XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0
A=$HOME/avsd
cd "${CHAINDIR:-$A/clips}" || exit 1
[ -n "$NORANDR" ] || wlr-randr --output NOOP-1 --custom-mode 800x480@60Hz 2>/dev/null
rm -f pcm.fifo key.fifo cap.pid enc.pid link.pid; mkfifo pcm.fifo key.fifo
if [ "${CAPMODE:-dmabuf}" = shm ]; then CAPARG=; FMT=rgb0; else CAPARG=--dmabuf; FMT=bgr0; fi
R=${RTPRIO:-5}
if [ "$R" -gt 0 ]; then
  (   # the chain stays owned by pi; only setting its priority needs root. -a: every thread.
    sleep 1
    sudo -n chrt -a -f -p $((R - 1)) "$(cat cap.pid)"
    sudo -n chrt -a -f -p "$R" "$(cat enc.pid)"
    sudo -n chrt -a -f -p $((R + 1)) "$(cat link.pid)"
    # Audio above the chain. PipeWire gets no RT here by itself (data-loop.0 ran SCHED_OTHER under a FIFO encoder, 2026-11-05).
    pgrep -x pw-record | while read -r p; do sudo -n chrt -a -f -p $((R + 2)) "$p"; done
    for p in $(pgrep -x pipewire) $(pgrep -x pipewire-pulse); do
      for t in /proc/$p/task/*; do
        [ "$(cat "$t/comm" 2>/dev/null)" = data-loop.0 ] && sudo -n chrt -f -p $((R + 15)) "${t##*/}"
      done
    done
  ) > rt.log 2>&1 &
fi
withpid() { f=$1; shift; bash -c 'echo $$ > "$0"; exec "$@"' "$f" "$@"; }
# AUDIOCAP=parec reverts. parec goes through pipewire-pulse, which loses buffers ("out of buffers" in its log): 47724-47931 frames/s against pw-record's 48004 on the same monitor, 2026-11-05.
if [ "${AUDIOCAP:-pw}" = parec ]; then
  timeout $1 parec -d hudiy_equalizer_sink.monitor --rate=48000 --format=s16le --channels=2 --raw --latency-msec=20 > pcm.fifo 2>parec.log &
else
  timeout $1 pw-record --target hudiy_equalizer_sink -P '{stream.capture.sink=true}' --rate 48000 --channels 2 --format s16 --latency 20ms --raw - > pcm.fifo 2>parec.log &
fi
timeout $1 bash -c 'echo $$ > cap.pid; exec "$0" "$@"' "${WLCAP_BIN:-$A/pi/wlcap/wlcap}" $CAPARG 2>cap.log | withpid enc.pid "${ENC_BIN:-$A/pienc_budget/rnv2enc}" -i - --fmt $FMT -o - --keyint ${KEYINT:-240} --linkcap ${LINKCAP:-4.8e6} --linkctl key.fifo --stats st.txt --threads ${ENCTHREADS:-2} --maxvid ${MAXVID:-161} ${ENCFLAGS:---cyc 4.05e6 --refill wall --dwell 0} 2>enc.log | withpid link.pid "${AVLINK_BIN:-$A/pi/avlink/avlink}" --video - --audio pcm.fifo ${LINKDEV:---dev /dev/ttyACM0} --keyctl key.fifo --maxvid ${MAXVID:-161} --whole-sf ${WHOLESF:-1} --gain-db ${GAIN:--18} --stats ${STATS:-5} --console > live.log 2>&1
