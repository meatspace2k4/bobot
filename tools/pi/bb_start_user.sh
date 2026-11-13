#!/bin/bash
# start root Wi-Fi session + my bridge (ch5 546) on 8781
# A root bridge whose pilot just left shuts down (and drops Beni's Wi-Fi) within ~10 s: wait that out first.
for i in $(seq 1 12); do systemctl is-active --quiet beni-bridge || break; sleep 1; done
systemctl is-active --quiet beni-bridge && ip -br addr show wlan0 | grep -q 192.168.7. || sudo -n /usr/bin/bash /opt/beni/beni_start.sh >/dev/null
for i in $(seq 1 60); do ip -br addr show wlan0 | grep -q 192.168.7. && break; sleep 1; done
IP=$(ip -br addr show wlan0 | grep -oE "192\.168\.7\.[0-9]+")
echo "wlan0=$IP"
for p in $(pgrep -u jpark -f "python3 -I /tmp/bb[.]py"); do kill $p; done
nohup python3 -I /tmp/bb.py --bind "$IP" --port 8781 --ch5 546 > /tmp/bb.log 2>&1 < /dev/null &
sleep 1; tail -1 /tmp/bb.log
