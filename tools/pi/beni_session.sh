#!/bin/bash
# Run as root by beni_start.sh (systemd unit beni-bridge). Keeps wlan0 on Beni and the bridge running;
# rejoins Beni's Wi-Fi if it drops (power cycle / battery swap). The bridge itself only holds Beni's
# UCP session while a pilot is connected, so the app can take Beni whenever no pilot is open.
SSID="${BENI_SSID:-Beni-206391}"
: "${BENI_PSK:?BENI_PSK missing from /opt/beni/beni.env}"
cleanup(){ nmcli con delete "$SSID" >/dev/null 2>&1; echo "left Beni; wlan0 failover is beni-wifi-watchdog.timer's job"; }
trap cleanup EXIT
trap 'exit 0' TERM INT
beni_ip(){ nmcli -g IP4.ADDRESS dev show wlan0 2>/dev/null | cut -d/ -f1 | head -1; }
join(){
  nmcli con delete "$SSID" >/dev/null 2>&1
  nmcli dev wifi rescan >/dev/null 2>&1; sleep 3
  nmcli dev wifi connect "$SSID" password "$BENI_PSK" ifname wlan0 >/dev/null 2>&1 || return 1
  nmcli con modify "$SSID" connection.autoconnect no >/dev/null 2>&1
  for i in $(seq 1 25); do BIP=$(beni_ip); case "$BIP" in 192.168.7.*) echo "wlan0=$BIP"; return 0;; esac; sleep 1; done
  return 1
}
until join; do echo "Beni not reachable; retrying in 10 s"; sleep 10; done
while :; do
  python3 -I /opt/beni/beni_bridge.py --bind "$BIP" --max-cap "${MAX_CAP:-1.0}" &
  PID=$!; lost=0
  while kill -0 $PID 2>/dev/null; do
    sleep 5
    case "$(beni_ip)" in 192.168.7.*) ;; *) echo "lost Beni Wi-Fi; rejoining"; lost=1; kill $PID; break;; esac
  done
  wait $PID; rc=$?
  [ $lost = 0 ] && [ $rc = 0 ] && break      # /stop requested -> exit for real
  until join; do echo "Beni not reachable; retrying in 10 s"; sleep 10; done
done
