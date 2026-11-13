#!/bin/sh
# Fly over the LAN instead of Cloudflare: SSH local-forward through the Asus port-forward to the Pi's bridge.
# Measured 2026-11-12: p50 21 ms vs 115 ms through the quick tunnel.
PI="${PI:-jpark@192.168.10.218}"; LPORT="${LPORT:-18780}"
pgrep -f "L $LPORT:127.0.0.1:8780" >/dev/null || \
  ssh -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -f -N -L "$LPORT:127.0.0.1:8780" "$PI" || exit 1
exec python3 -I "$(dirname "$0")/beni_pilot.py" "http://127.0.0.1:$LPORT" --buffer-ms "${BUFFER_MS:-60}"
