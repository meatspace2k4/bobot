#!/bin/bash
# Run as root by beni_start.sh (systemd unit beni-bridge). Join Beni on wlan0, run the bridge, restore home Wi-Fi on exit.
SSID="${BENI_SSID:-Beni-206391}"; HOME_NET="${HOME_SSID:-Beeman}"
: "${BENI_PSK:?BENI_PSK missing from /opt/beni/beni.env}"
cleanup(){
  nmcli con delete "$SSID" >/dev/null 2>&1
  for i in 1 2 3 4 5; do
    nmcli dev wifi rescan >/dev/null 2>&1; sleep 3
    nmcli con up "$HOME_NET" ifname wlan0 >/dev/null 2>&1 && { echo "restored $HOME_NET"; return; }
    echo "restore attempt $i failed"; sleep 7
  done
  echo "restore failed; beni-wifi-watchdog.timer will retry"
}
trap cleanup EXIT
trap 'exit 0' TERM INT
nmcli con delete "$SSID" >/dev/null 2>&1
for try in 1 2 3; do
  nmcli dev wifi rescan >/dev/null 2>&1; sleep 3
  nmcli dev wifi connect "$SSID" password "$BENI_PSK" ifname wlan0 && break
  echo "join attempt $try failed"
done
nmcli con modify "$SSID" connection.autoconnect no >/dev/null 2>&1
for i in $(seq 1 25); do
  BIP=$(nmcli -g IP4.ADDRESS dev show wlan0 | cut -d/ -f1 | head -1)
  case "$BIP" in 192.168.7.*) break;; esac; sleep 1
done
case "$BIP" in 192.168.7.*) echo "wlan0=$BIP";; *) echo "no Beni IP"; exit 3;; esac
python3 -I /opt/beni/beni_bridge.py --bind "$BIP" --max-cap "${MAX_CAP:-0.5}" &
wait $!
