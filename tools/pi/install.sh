#!/bin/bash
# From the Mac: tools/pi/install.sh jpark@aircheck-pi.local
# Installs to /opt/beni, prompts once for Beni's Wi-Fi password (stored root-only), deletes old /tmp copies.
set -e
HOST="${1:?usage: install.sh user@pi}"
D="$(cd "$(dirname "$0")" && pwd)"
scp -q "$D/beni_start.sh" "$D/beni_session.sh" "$D/beni_bridge.py" "$D/cf-empty.yml" "$D/beni.env.example" "$HOST:/tmp/"
ssh -t "$HOST" 'set -e
sudo install -d -m 755 /opt/beni
cd /tmp && sudo install -m 755 beni_start.sh beni_session.sh /opt/beni/ && sudo install -m 644 beni_bridge.py cf-empty.yml beni.env.example /opt/beni/
if ! sudo test -f /opt/beni/beni.env; then
  read -rsp "Beni Wi-Fi password: " PSK; echo
  printf "BENI_SSID=Beni-206391\nBENI_PSK=%s\nHOME_SSID=Beeman\nMAX_CAP=1.0\n" "$PSK" | sudo install -m 600 /dev/stdin /opt/beni/beni.env
  unset PSK
fi
rm -f /tmp/beni_start.sh /tmp/beni_session.sh /tmp/beni_bridge.py /tmp/cf-empty.yml /tmp/beni.env.example /tmp/beni_join_stream.sh /tmp/pi_tunnel_relay.sh /tmp/pi_relay.sh /tmp/beni_video.py /tmp/beni_url.txt
sudo rm -f /tmp/pi_tunnel_relay.log /tmp/cf_quick.log /tmp/beni_tunnel_url.txt
ls -l /opt/beni; sudo stat -c "%a %U %n" /opt/beni/beni.env
echo INSTALL-OK'
