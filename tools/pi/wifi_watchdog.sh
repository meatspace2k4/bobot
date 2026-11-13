#!/bin/bash
# Root, every 60 s via beni-wifi-watchdog.timer. wlan0 is a FAILOVER uplink only:
#  - eth0 (Asus) healthy  -> wlan0 stays off home Wi-Fi (and free for Beni)
#  - eth0 down/unusable   -> bring home Wi-Fi up on wlan0
# Never touches wlan0 while a Beni relay (beni-bridge) is running.
systemctl is-active --quiet beni-bridge && exit 0
HOME_NET="${HOME_SSID:-Beeman}"
eth_ok() {
  [ "$(cat /sys/class/net/eth0/carrier 2>/dev/null)" = 1 ] || return 1
  ping -c1 -W2 -I eth0 1.1.1.1 >/dev/null 2>&1 || ping -c1 -W2 -I eth0 10.0.0.1 >/dev/null 2>&1
}
wl=$(nmcli -t -f DEVICE,STATE,CONNECTION dev | awk -F: '$1=="wlan0"{print $2":"$3}')
nmcli con delete "${BENI_SSID:-Beni-206391}" >/dev/null 2>&1
if eth_ok; then
  if [ "$wl" = "connected:$HOME_NET" ]; then nmcli con down "$HOME_NET" >/dev/null 2>&1 && echo "watchdog: eth0 healthy, released wlan0"; fi
else
  if [ "${wl%%:*}" != "connected" ]; then
    nmcli dev wifi rescan >/dev/null 2>&1; sleep 3
    nmcli con up "$HOME_NET" ifname wlan0 >/dev/null 2>&1 && echo "watchdog: eth0 down, failover to $HOME_NET" || echo "watchdog: eth0 down, failover failed"
  fi
fi
