#!/bin/bash
# Removes the car setup and restores what install_car.sh saved in ~/avsd/car/backup. Run on the Pi, as pi.
# Stops the chain now; the cursor and wayvnc settings return at the next reboot. No Hudiy, labwc or pipewire restart.
C=$HOME/avsd/car
B=$C/backup
systemctl --user disable --now avsd-chain.service 2>/dev/null
rm -f "$HOME/.config/systemd/user/avsd-chain.service"
systemctl --user daemon-reload
sudo systemctl disable --now avsd-governor.service 2>/dev/null
sudo rm -f /etc/systemd/system/avsd-governor.service
sudo systemctl daemon-reload
if [ -e "$B/labwc-environment" ]; then cp -p "$B/labwc-environment" "$HOME/.config/labwc/environment"; else echo "no saved labwc environment"; fi
if [ -e "$B/governor" ]; then
  G=$(cat "$B/governor")
  for f in /sys/devices/system/cpu/cpufreq/policy*/scaling_governor; do echo "$G" | sudo tee "$f" >/dev/null; done
fi
[ "$(cat "$B/wayvnc-enabled" 2>/dev/null)" = enabled ] && sudo systemctl enable wayvnc
rm -rf "$XDG_RUNTIME_DIR/avsd" 2>/dev/null
echo "Removed. Backup left in $B. Reboot for the cursor setting."
