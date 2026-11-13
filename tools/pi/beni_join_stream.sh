#!/bin/bash
# Run as root via systemd-run. Join hidden Beni on wlan0, serve H.265 as HTTP-TS on :8780.
PORT=8780
SSID="${BENI_SSID:-Beni-206391}"
: "${BENI_PSK:?set BENI_PSK (see tools/pi/beni.env.example)}"
cleanup(){ nmcli con delete $SSID >/dev/null 2>&1; nmcli con up "${HOME_SSID:-Beeman}" >/dev/null 2>&1; }
trap cleanup EXIT
nmcli con delete $SSID >/dev/null 2>&1
nmcli dev wifi connect $SSID password "$BENI_PSK" ifname wlan0
nmcli con modify $SSID connection.autoconnect no
for i in $(seq 1 25); do
  BIP=$(nmcli -g IP4.ADDRESS dev show wlan0 | cut -d/ -f1 | head -1)
  case "$BIP" in 192.168.7.*) break;; esac; sleep 1
done
case "$BIP" in 192.168.7.*) echo "wlan0=$BIP";; *) echo "no Beni IP"; exit 3;; esac
python3 -I /opt/beni/beni_video.py --live --out - --secs 590 --bind "$BIP" \
 | ffmpeg -hide_banner -loglevel info -fflags nobuffer -probesize 2M -analyzeduration 1000000 -f hevc -i - -c copy \
     -f mpegts -listen 1 "http://0.0.0.0:$PORT"
