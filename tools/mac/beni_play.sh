#!/bin/sh
URL="${1:?usage: beni_play.sh <trycloudflare url>}"
for i in $(seq 1 200); do
  echo "$(date +%T) ffplay attempt $i" >> /tmp/beni_play.log
  /opt/homebrew/bin/ffplay -hide_banner -loglevel warning -fflags nobuffer -flags low_delay \
    -window_title "Beni live" "$URL" >> /tmp/beni_play.log 2>&1
  sleep 3
done
