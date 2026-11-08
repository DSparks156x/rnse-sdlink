#!/bin/bash
# xrun_delta.sh SECONDS -- per-node xrun (ERR) increase and mean busy time over SECONDS, from pw-top. Read only.
S=${1:-30}
snap() { pw-top -b -n 2 2>/dev/null | awk '/^S +ID/{n++} n==2 && $1=="R"' | awk '{name=$NF; for(i=11;i<NF;i++) if($i=="+"){name=""; for(j=i+1;j<=NF;j++) name=name $j " "; break} print $2 "|" $9 "|" $6 "|" $3 "|" name}'; }
snap > /tmp/xa; sleep "$S"; snap > /tmp/xb
echo "node | quantum | busy now | ERR +$S s"
join -t'|' -j1 <(sort /tmp/xa) <(sort /tmp/xb) | awk -F'|' '{d=$6-$2; printf "%-45s q=%-5s busy=%-8s ERR+%d\n", $9, $8, $7, d}'
