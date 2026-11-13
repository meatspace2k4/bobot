# bobot: Beni (Mondo Robotics) protocol RE

Protocol notes are in [protocol.md](protocol.md), and the topic ID table is in [topics.md](topics.md).

## Tools
| path | what |
|---|---|
| `tools/pktap_summary.py` | Summarize an rvictl pcap/pcapng by interface, process and flow |
| `tools/ucp_timeline.py` | UCP (UDP 9007) message timeline, without video |
| `tools/beni_video.py` | H.265 depacketizer: `--from-pcap` or `--live` (50 Hz ack) |
| `tools/beni_drive.py` | Capped stick-frame client (11-bit RC channels) |
| `tools/pi/` | Live video relay: Pi joins Beni and serves the stream over a cloudflared quick tunnel |
| `tools/mac/beni_play.sh` | ffplay retry loop for the tunnel URL |
| `tools/mac/beni_pilot.py` | Video + DualSense control window (WebSocket to the bridge) |
| `tools/sim/fake_beni.py` | Fake Beni for bench-testing bridge + pilot |

## Control path to the Pi
`ssh jpark@192.168.10.218`: the Asus WAN forwards port 22 to the Pi's eth0. This works wherever wlan0 is. Replies from 10.0.0.2 are pinned to eth0 by a source-routing rule (table 100) on "Wired connection 1".
wlan0 is failover only: `beni-wifi-watchdog.timer` (60 s) brings up home Wi-Fi only when eth0 is down or can't reach out, and releases it when eth0 is healthy. Beeman is `autoconnect no`.

## Fly Beni from the Mac
Order matters: Beni serves one UCP peer at a time.
1. Stand Beni with the remote. The phone must not be on Beni's Wi-Fi (Beni serves one peer).
2. On the Pi, run `sudo bash /opt/beni/beni_start.sh`. It prints the tunnel URL. The bridge takes over the app's session.
3. On the Mac, run `tools/mac/beni_lan.sh` (LAN path via SSH forward: p50 21 ms; preferred). From off-LAN, use `tools/mac/beni_pilot.py <url>` through the tunnel (p50 115 ms). Controls: left stick drives, right stick pans the head, Cross jumps (2nd press within 1.5 s = jump toward the stick), D-pad flips, touchpad e-stop.
4. Stop with `curl <url>/stop`.

## Live video only
One-time setup:
```bash
tools/pi/install.sh jpark@aircheck-pi.local
```
Then on the Pi, create `/opt/beni/beni.env` from `beni.env.example` (`chmod 600`).

Each session:
1. On the Pi, run `sudo bash /opt/beni/beni_start.sh`. It prints the trycloudflare URL and starts a 10-minute relay. Your SSH drops when wlan0 joins Beni.
2. On the Mac, run `tools/mac/beni_play.sh <url>`.

The Pi returns to the home Wi-Fi when the relay stops. Rerunning `beni_start.sh` reuses the tunnel and its URL.

Captures (`captures/`, ~2 GB) and `beni.env` are not tracked.
