#!/bin/sh
# One app: starts the Pi bridge, opens the LAN forward, runs the pilot. When the pilot exits,
# the bridge exits ~3 s later and the Pi leaves Beni's Wi-Fi, so the phone app can connect.
PI="${PI:-jpark@192.168.10.218}"; LPORT="${LPORT:-18780}"
# CH5=547 tools/mac/beni_lan.sh  -> bridge sends that raw ch5 instead of the template's 560.
if [ -n "$CH5" ]; then SET="echo $CH5 > ~/.beni_ch5"; else SET="rm -f ~/.beni_ch5"; fi
ssh -o BatchMode=yes "$PI" "$SET" || exit 1
ssh -o BatchMode=yes "$PI" 'sudo -n /usr/bin/bash /opt/beni/beni_start.sh' >/dev/null || { echo "could not start bridge on the Pi"; exit 1; }
pgrep -f "L $LPORT:127.0.0.1:8780" >/dev/null || \
  ssh -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -f -N -L "$LPORT:127.0.0.1:8780" "$PI" || exit 1
exec python3 -I "$(dirname "$0")/beni_pilot.py" "http://127.0.0.1:$LPORT" --buffer-ms "${BUFFER_MS:-60}"
