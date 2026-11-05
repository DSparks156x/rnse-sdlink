#!/bin/bash
# Hudiy screen + Hudiy audio -> RNS-E
# Env: KEYINT (periodic key safety net, frames), LINKCAP (B/s), MAXVID (video sectors per superframe: must read in under a tick, so audio
# keeps 60 superframes/s even when video lags), GAIN (dB, -18 = the stock MP3 level).
export XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0
cd ~/avsd/clips
wlr-randr --output NOOP-1 --custom-mode 800x480@60Hz 2>/dev/null
rm -f pcm.fifo key.fifo; mkfifo pcm.fifo key.fifo
timeout $1 parec -d hudiy_equalizer_sink.monitor --rate=48000 --format=s16le --channels=2 --raw --latency-msec=20 > pcm.fifo 2>parec.log &
timeout $1 ../pi/wlcap/wlcap 2>cap.log | ../pienc/rnv2enc -i - --fmt rgb0 -o - --keyint ${KEYINT:-240} --linkcap ${LINKCAP:-4.4e6} --linkctl key.fifo --stats st.txt --auto-damage --threads ${ENCTHREADS:-2} 2>enc.log | ../pi/avlink/avlink --video - --audio pcm.fifo --dev /dev/ttyACM0 --keyctl key.fifo --maxvid ${MAXVID:-150} --gain-db ${GAIN:--18} --stats 5 --console > live.log 2>&1
