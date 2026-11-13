#!/bin/bash
# Run on the Pi:  sudo bash /opt/beni/beni_start.sh
set -a; . /opt/beni/beni.env; set +a
systemctl stop beni-relay 2>/dev/null; systemctl reset-failed beni-relay 2>/dev/null
if ! systemctl is-active --quiet beni-cf; then
  systemctl reset-failed beni-cf 2>/dev/null
  systemd-run --unit beni-cf -E HOME=/tmp /usr/local/bin/cloudflared --config /opt/beni/cf-empty.yml tunnel --no-autoupdate --url http://localhost:8780
  sleep 12
fi
URL=$(journalctl -u beni-cf -n 200 --no-pager | grep -oE 'https://[a-z0-9-]+\.trycloudflare\.com' | tail -1)
echo "URL:  $URL"
systemd-run --unit beni-relay -p EnvironmentFile=/opt/beni/beni.env -p RuntimeMaxSec=600 \
  -p ExecStopPost="-/usr/bin/nmcli con delete ${BENI_SSID:-Beni-206391}" -p ExecStopPost="/usr/bin/nmcli con up ${HOME_SSID:-Beeman}" \
  /opt/beni/beni_join_stream.sh && echo "RELAY STARTED - SSH will drop"
