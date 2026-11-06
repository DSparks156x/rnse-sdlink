#!/bin/bash
# Installs the car setup on the Pi. Run on the Pi, as pi, from ~/avsd/car: ./install_car.sh
# Takes effect at the next reboot; nothing running is stopped or restarted. Undo: ./uninstall_car.sh
set -e
A=$HOME/avsd
C=$A/car
B=$C/backup
[ "$(id -un)" = pi ] || { echo "run as pi"; exit 1; }
[ -d "$C" ] && [ -f "$C/avsd_run.sh" ] || { echo "copy pi/scripts/car to ~/avsd/car first"; exit 1; }
for f in "$A/hud2.sh" "$A/pi/wlcap/wlcap" "$A/pienc/rnv2enc" "$A/pi/avlink/avlink"; do
  [ -x "$f" ] || { echo "missing or not executable: $f"; exit 1; }
done
grep -q 'CHAINDIR' "$A/hud2.sh" && grep -q 'LINKDEV' "$A/hud2.sh" || { echo "$A/hud2.sh is too old (no CHAINDIR/LINKDEV): deploy pi/scripts/hud2.sh first"; exit 1; }
command -v fuser >/dev/null || { echo "fuser missing (package psmisc)"; exit 1; }
sudo -n chrt --help >/dev/null 2>&1 || echo "note: no passwordless sudo for chrt: the chain will run without RT priority"
chmod +x "$C/avsd_run.sh" "$C/uninstall_car.sh" "$C/avsd_status.sh"

# ---- saved once, never overwritten
mkdir -p "$B"
E=$HOME/.config/labwc/environment
[ -e "$B/labwc-environment" ] || cp -p "$E" "$B/labwc-environment"
[ -e "$B/governor" ] || cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor > "$B/governor"
[ -e "$B/wayvnc-enabled" ] || systemctl is-enabled wayvnc > "$B/wayvnc-enabled" 2>/dev/null || true

# ---- invisible cursor: only that line changes (same result as ~/.config/labwc/environment.bak-avsd)
sed -i 's/^XCURSOR_THEME=.*/XCURSOR_THEME=invisible/' "$E"
grep -q '^XCURSOR_THEME=invisible$' "$E" || echo 'XCURSOR_THEME=invisible' >> "$E"

# ---- wayvnc stays off at boot (a running one started by hand ends with the session)
[ -n "$KEEP_VNC" ] || sudo systemctl disable wayvnc 2>/dev/null || true   # KEEP_VNC=1: bench, wayvnc stays as it is

# ---- units
mkdir -p "$HOME/.config/systemd/user"
install -m 644 "$C/avsd-chain.service" "$HOME/.config/systemd/user/avsd-chain.service"
systemctl --user daemon-reload
systemctl --user enable avsd-chain.service
sudo install -m 644 "$C/avsd-governor.service" /etc/systemd/system/avsd-governor.service
sudo systemctl daemon-reload
sudo systemctl enable avsd-governor.service
loginctl show-user "$(id -un)" -p Linger | grep -q yes || sudo loginctl enable-linger "$(id -un)"

echo "Installed. Reboot to start. Check with ~/avsd/car/avsd_status.sh. Undo: ~/avsd/car/uninstall_car.sh"
