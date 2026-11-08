#!/bin/bash
# audio_diag.sh -- one pass over everything that can make audio crackle or gap on the car Pi. Read only: changes nothing, restarts nothing.
# Run on the Pi while music plays on the RNS-E TV source:  bash audio_diag.sh [seconds, default 20] > diag.txt 2>&1
# UNIT=1 also reads the unit's own audio counters once over CAN (one read; CAN reads briefly stall the unit, so it is last and optional).
S=${1:-20}
H=$HOME/avsd
sec() { printf '\n===== %s\n' "$*"; }

sec "time, load, throttling"
date; uptime
vcgencmd get_throttled 2>/dev/null; vcgencmd measure_temp 2>/dev/null; vcgencmd measure_clock arm 2>/dev/null
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null | sort | uniq -c

sec "release, env"
readlink $H/current; grep -vE '^#|^$' $H/avsd.env; cat ${XDG_RUNTIME_DIR:-/run/user/1000}/avsd-trial.env 2>/dev/null

sec "audio graph: modules, sinks, sources, defaults"
pactl info | grep -E 'Default|Server Name'
pactl list short modules
pactl list short sinks; pactl list short sources
pw-metadata -n settings 2>/dev/null | grep -E 'quantum|rate'

sec "who changed the mic chain: config files newer than 2026-11-01"
find ~/.config/pipewire ~/.config/wireplumber ~/.hudiy /etc/pipewire /usr/share/pipewire/pipewire.conf.d ~/.config/systemd/user ~/.config/autostart \
     -newermt 2026-11-01 -type f -printf '%TY-%Tm-%Td %TH:%TM  %p\n' 2>/dev/null | sort
for f in ~/.hudiy/share/hudiy_startup.sh $(ls ~/.config/pipewire/pipewire.conf.d/*.conf ~/.config/pipewire/filter-chain.conf.d/*.conf 2>/dev/null); do
  echo "--- $f"; cat "$f"
done

sec "processes: mic and audio"
ps -eo pid,ni,rtprio,cls,psr,pcpu,rss,args --sort=-pcpu | grep -Ei 'pipewire|wireplumber|noisetorch|deepfilter|rnnoise|easyeffects|hudiy|avlink|rnv3enc|wlcap|pw-record|filter|PID' | grep -v grep | cut -c1-200

sec "pw-top: per node ERR (xruns) and timing, ${S}s"
B=$(mktemp); timeout $((S + 3)) pw-top -b -n $S > $B 2>&1
awk 'NR<=60' $B; echo ...; tail -45 $B; rm -f $B

sec "per-thread CPU over 5 s (top 30)"
top -H -b -d 5 -n 2 -o %CPU | awk '/^top -/{n++} n==2' | sed -n 1,38p | cut -c1-150

sec "pipewire log: xruns, errors, last boot"
journalctl --user -b --no-pager -u pipewire -u pipewire-pulse -u wireplumber 2>/dev/null | grep -Ei 'xrun|underrun|overrun|error|warn|timeout|late' | tail -40

sec "link counters: latest session, per 5 s while playing"
L=$(ls -t $H/logs/*-live.log 2>/dev/null | head -1); echo "$L"
python3 - "$L" <<'E'
import re, sys
K = ['built', 'late', 'adrop', 'silence', 'catchups', 'backoffs', 'cap_kb', 'window_waits', 'away_ticks', 'adj', 'served', 'free']
rows = []
for l in open(sys.argv[1], errors='replace'):
    if l.startswith('STAT') or l.startswith('END'):
        rows.append({k: int(m.group(1)) if (m := re.search(r'\b%s=(-?\d+)' % k, l)) else 0 for k in K})
p = rows[0] if rows else None
for i, r in enumerate(rows[1:], 1):
    if r['away_ticks'] - p['away_ticks'] < 20:
        print('t=%4ds late+%3d adrop+%5.0fms sil+%4.0fms catch+%3d back+%3d cap=%5dkB win+%5d adj=%4d free=%d' % (i * 5, r['late'] - p['late'], (r['adrop'] - p['adrop']) / 48,
              (r['silence'] - p['silence']) / 48, r['catchups'] - p['catchups'], r['backoffs'] - p['backoffs'], r['cap_kb'], r['window_waits'] - p['window_waits'], r['adj'], r['free']))
    p = r
E
tail -3 $H/logs/$(basename "$L" -live.log)-enc.log 2>/dev/null | cut -c1-160
tail -2 "$(dirname "$L")/$(basename "$L" -live.log)-cap.log" 2>/dev/null | cut -c1-200
ls $H/current/chain >/dev/null && grep -h parec -A0 $H/logs/*parec* 2>/dev/null | tail -3

if [ "${UNIT:-0}" = 1 ]; then
sec "unit audio counters (one CAN read)"
cd $H/current && python3 - <<'E'
import struct
from app import can_link
names = ('SEQ SF_OK HDR_BAD RETRY EXPIRED SEQ_MISM RD_CALLS RD_ERR RD_SECT RD_CNT_LO RD_CNT_MAX V_DEC V_DEC_ERR V_CRC_BAD V_DROP V_REPEAT V_LATE DEC_MAX DEC_LO DEC_HI '
         'V_SHOWN TLV_OK TLV_BAD A_CRC_BAD A_IN A_UNDER A_BLOCKS A_CREDITS A_FILL A_MIN A_MAX A_OVER A_HANDLE A_STATE A_PLAYED DISCONT EOS SKIPPED DU LOOP_MAX '
         'LAST_FLAGS LAST_FID SINK0 CLOSE EOS_RET DRAIN_MS RD_CNT_HI LAST_RD_RET PACE_WAITS FIRST_ERR_SEQ V_ERR_CODE CD_RAW A_FMT_BAD WALL_MS SF_TOTAL REUSE_WAITS A_EV_UNDER '
         'A_EV_OTHER FRAG_IN FRAG_ERR SPAN_MAX VFRAME_MAX SPAN1 SPAN2 SPAN3 SPAN4 SPAN5 SPAN6 SPAN7 SPAN8P LOG_N MEM_CMD MEM_BAD MEM_TX MEM_RETRY MEM_IO '
         'IIS_RET IIS_SAVED IIS_MS IIS_REISSUE IIS_END SSI_CR SSI_SR E219_A E219_B P904_A P904_B STUCK_N STUCK_MS STUCK_LBA STUCK_CNT SDR0 SDR1 SDR2 SDR3 SDR4 SDR5 SDR6 SDR7 '
         'FL_N FL_WAIT FL_F14 FL_F20 FL_F34 FL_F50 FL_MAX FL_BLK SESS R_MIN R_ZERO RATE_MHZ CTX_CHG CTX_A CTX_B DU_FIX A_RESYNC MUTE_A KEYS RESTARTS').split()
u = can_link.Unit(can_link.SocketCanBus())
try:
    w = struct.unpack('>%dI' % (8 + len(names)), u.read(0x8F720100, (8 + len(names)) * 4))
finally:
    u.close()
print('magic %08X (expect 41565344)' % w[0])
d = dict(zip(names, w[8:]))
for k in ('A_IN A_UNDER A_OVER A_RESYNC A_CRC_BAD A_FMT_BAD A_EV_UNDER A_EV_OTHER R_MIN R_ZERO A_FILL A_MIN A_MAX FL_BLK MUTE_A DISCONT SF_OK HDR_BAD RETRY EXPIRED SEQ_MISM '
          'RD_ERR V_LATE DEC_MAX STUCK_N STUCK_MS SESS RESTARTS').split():
    v = d[k]
    print('%-11s %10d' % (k, v - (1 << 32) if v >= 1 << 31 else v))
E
fi
