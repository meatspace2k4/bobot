#!/bin/sh
# Waits for the Mac to reach Beni, then runs a capped stick test. Log: captures/drive-log.txt
cd /Users/johusha/bobot || exit 1
LOG=captures/drive-log.txt
echo "armed $(date +%T)" > $LOG
i=0
until ipconfig getifaddr en0 2>/dev/null | grep -q '^192\.168\.7\.'; do
  sleep 1; i=$((i+1)); [ $i -gt 900 ] && { echo "gave up waiting $(date +%T)" >> $LOG; exit 1; }
done
echo "on Beni AP as $(ipconfig getifaddr en0) $(date +%T); starting in 10 s" >> $LOG
sleep 10
run() { echo "--- $(date +%T.%N | cut -c1-12) $*" >> $LOG; python3 -I tools/beni_drive.py "$@" >> $LOG 2>&1; sleep 2; }
run --throttle 0.1 --secs 1.0
run --throttle -0.1 --secs 1.0
run --yaw -0.1 --secs 0.5
run --yaw 0.1 --secs 0.5
echo "finished $(date +%T)" >> $LOG
