#!/bin/bash
# Root, every 60 s via beni-wifi-watchdog.timer. If no relay is running and wlan0 isn't connected,
# bring home Wi-Fi back. Manual activation also clears NetworkManager's sticky 'no-secrets' autoconnect block.
systemctl is-active --quiet beni-bridge && exit 0
state=$(nmcli -t -f DEVICE,STATE dev | awk -F: '$1=="wlan0"{print $2}')
[ "$state" = "connected" ] && exit 0
nmcli con delete "${BENI_SSID:-Beni-206391}" >/dev/null 2>&1
nmcli dev wifi rescan >/dev/null 2>&1; sleep 3
nmcli con up "${HOME_SSID:-Beeman}" ifname wlan0 >/dev/null 2>&1 && echo "watchdog: restored ${HOME_SSID:-Beeman}" || echo "watchdog: reconnect failed (state=$state)"
