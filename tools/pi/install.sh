#!/bin/bash
# From the Mac: tools/pi/install.sh jpark@aircheck-pi.local
# Copies the relay to /opt/beni on the Pi. Create /opt/beni/beni.env from beni.env.example there (root-only, 600).
set -e
HOST="${1:?usage: install.sh user@pi}"
D="$(cd "$(dirname "$0")" && pwd)"
scp "$D/beni_start.sh" "$D/beni_join_stream.sh" "$D/cf-empty.yml" "$D/beni.env.example" "$D/../beni_video.py" "$HOST:/tmp/"
ssh -t "$HOST" 'sudo install -d /opt/beni && cd /tmp && sudo install -m 755 beni_start.sh beni_join_stream.sh /opt/beni/ && sudo install -m 644 cf-empty.yml beni_video.py beni.env.example /opt/beni/ && { [ -f /opt/beni/beni.env ] || echo "now create /opt/beni/beni.env (chmod 600)"; }'
