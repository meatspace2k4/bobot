# Beni protocol notes (working draft)

Captures: `captures/beni-rvi-20261112-2012.pcap` (rvictl, iPhone 12, phone on Beni's AP).
- 20:12–20:26: an existing session (session 4 clean run at 20:20:02–20:20:54).
- 20:27:44: a fresh connect after a power cycle (full handshake).

Tools: `tools/pktap_summary.py` (flows by interface and process), `tools/ucp_timeline.py` (UCP messages without video).
Topic IDs: [topics.md](topics.md).

## Topology
- App: process `Runner`, so a **Flutter** app. On home Wi-Fi with the proxy configured, its HTTPS went **direct** (to 54.241.19.94, 47.88.x, 52.52.x, 43.130.8.48, 18.154.132.25), never through mitmproxy. Confirmed: it ignores the system proxy.
- Beni AP: robot `192.168.7.1` (DHCP server), phone `192.168.7.137`.
- Robot ports:
  - UDP 9007: everything (UCP transport)
  - TCP 8084: TLS 1.3. Its identity is likely served via `app/cmd/security/local_tls_identity_req`.

## Layer 1: UCP over UDP 9007
Connect: the phone sends ASCII `ucp:sync` and the robot replies `ucp:ack` (twice), then the frames below start.
```
off size field (LE)
0   1   magic 0x51
1   1   type: 0x00 keepalive "UCP:", 0x10 unreliable/stream, 0x0d reliable message, 0x07/0x05 video frag (more/last)
2   2   ack (peer's last seq)
4   2   own seq
6   2   datagram length
8   4   sender timestamp (µs, sender clock)
12  ..  body
```
- **Video** (0x07/0x05): cleartext H.265 Annex-B, ~11 Mbit/s.
- **Stick** (0x10, 31 B, 50 Hz, cleartext): body[0:9] holds LE-packed 11-bit channels, centered on 1000 with a range of 1..1999.
  - ch0 throttle: 1999 full forward, 1 full reverse, proportional
  - ch1 yaw: 1 full left, 1999 full right
  - ch2 = 1000, ch3 = 2015, ch4 = 32
  - ch5 mode: 545/560 idle, 801/816 pilot
  - body[9:11] = video ack
- Before pilot mode starts, the phone sends 24 B 0x10 frames with body `fe05 <ack>` instead.

## Action pulses (cleartext, ch4) — mapped 2026-11-12 22:54, `captures/beni-actions.pcap` (untracked)
Jump and flips are not encrypted. They ride in the 50 Hz stick frame as a ~190 ms pulse (about 10 frames) on **ch4**, which rests at 32:
`ch4 = 32 | code << 6`.

| action | code | ch4 |
|---|---|---|
| jump | 1 | 96 |
| flip forward | 4 | 288 |
| flip back | 6 | 416 |
| flip left | 8 | 544 |
| flip right | 2 | 160 |

- The first jump drew a robot `device/status/rc/fsm_state` push (tag 0x5a) about 50 ms later.
- No encrypted 0x0d message accompanied any action; only the periodic ping and timer were seen.
- ch5 was 546 in this session (545 in session 4); driving works with either.
- Codes 3, 5, 7 and 9–15 are unexplored, and may be the other trained skills.

## Layer 2: pub/sub messages (UCP type 0x0d)
Body: `a7 00 01 <len varint> 00 25 <msg seq varint> <op> ...`
- At connect, each side declares key expressions, Zenoh-style: `1e 20 <id> 00 <len> <name>`, plus subscriber decls `1e 62 <id>` for `app/**` and `device/**`. The robot declares 99 topics and the phone 83. Both lists are in [topics.md](topics.md).
- Data message: `5d <topic id> 41 1a <len> 18 01 22 <len> <E2E envelope>`. The **topic ID is the sender's own declared ID**.
- Confirmed request/response pairs:

| phone id | topic | robot id | topic | seen |
|---|---|---|---|---|
| 0x19 | app/cmd/common/ping_req | 0x15 | device/cmd/common/ping_rsp | every ~5 s |
| 0x31 | app/cmd/parameter/query_req | 0x28 | device/cmd/parameter/query_rsp | 20:20:02 burst |
| 0x32 | app/cmd/parameter/update_req | 0x29 | device/cmd/parameter/update_rsp | 20:20:53 ×2 (sit / speed write) |
| 0x28 | app/cmd/mission/request | 0x1f | device/cmd/mission/request_ack | 20:20:02 (stand / mode) |
| 0x25 | app/cmd/mic/saved_list_req | 0x1c | device/cmd/mic/saved_list_rsp | connect |
| 0x02 | app/cmd/audio/op_req | 0x02 | device/cmd/audio/op_rsp | — |
| 0x1f | app/cmd/gateway/analytics_event_receipt | 0x51 | device/status/gateway/analytics_event_transfer | the 500–1100 B bursts |
| 0x52 | app/security/session | 0x46 | device/security/session | handshake |
| 0x42 | app/cmd/snapback/capture_control | — | — | every 10.000 s |

- Robot pushes:
  - 0x4c, 1 Hz (ID-table entry is `device/status/capability/voice_language`; at 1 Hz that is suspect, so verify)
  - 0x5a `device/status/rc/fsm_state`, on change (stand/sit state)
- Cleartext heartbeat `a7 00 01 01 00 04`, every 3 s both ways.

## Layer 3: E2E encryption
Handshake (topic `security/session`, 20:27:44.755):
```
phone 0x52: 2a 37 01 01 04 | keyid[20]=4c812c7f3b3dfc4057f42bb79f3d3a1d71fcdec8 | rand_c[32]
robot 0x46: 2a 3f 01 01 05 | keyid[20] (same)                                   | rand_s[32] | session_id[8]=cb240936bdd11c57
then: phone 0x52 and robot 0x46 each send one confirm message (encrypted, counter 0)
```
Correction from the re-pair capture (`captures/beni-rvi-20261112-2037.pcap`, UCP sync at 20:37:23):
- The fixed key ID is **16 bytes**: `4c812c7f3b3dfc4057f42bb79f3d3a1d`.
- The next 4 bytes are a per-handshake value the robot echoes back (`71fcdec8`, then `c77de916`).
- The key ID is **unchanged after re-pairing**, so it is bound to the robot or account, not to the pairing session.
- No `e2e_provision_req` (0x40) or `device_activate_req` (0x3c) crossed Wi-Fi. Provisioning happened over BLE or via the cloud.
- Right after the session is up, the phone sends `local_tls_identity_req` (0x41) over the E2E channel, then opens TLS to **8083** and **8084**. The local TLS identity is fetched through the encrypted channel.

Every later envelope: `session_id[8] ‖ counter[8, BE, per direction] ‖ ciphertext ‖ tag`.
- The 20-byte `keyid` is identical on both sides and survives power cycles. It names a **long-term pairing key**, probably provisioned during pairing via `app/cmd/security/e2e_provision_req` and/or the cloud.
- The session key is likely KDF(pairing key, rand_c, rand_s). No key material crosses the LAN, so **passive decryption needs the pairing key**.
- The previous session's id was `0f6211a55074197a`, so a new id is issued on every connect.

## Active test from the Mac (20:50:46–20:51:05, `tools/beni_drive.py`, log `captures/drive-log.txt`)
- `ucp:sync` from a new host (Mac, 192.168.7.64) gets `ucp:ack`, and video streams to it right away with no E2E session.
- Four ±10% stick pulses were sent. Per the operator, Beni **twitched to the pulses, then sat**.
- Reading: the stick frames reached the balance/drive loop (the twitch), so the format and host are accepted. But with no E2E session and no pilot-mode mission active, a watchdog drops it to the sit posture within ~1 s. Driving needs a live pilot-mode session held open (encrypted mission), not just stick frames.

## Live video relay (working, 2026-11-12 22:00)
- Any host that sends `ucp:sync` to 192.168.7.1:9007 gets `ucp:ack` and the video stream. No E2E session is needed for video.
- **The receiver must ack at about 50 Hz**: 0x10 frames, with body[9:11] set to the last video seq. At about 1 Hz, Beni sends a burst and then stalls (126 fragments in 10 min). At 50 Hz it delivers the full ~650 pps stream.
- Resolution varies with mode: 960×720 @ 25 fps in the first capture, 1920×1440 @ 60 fps via the relay.
- Path: Pi wlan0 joins Beni (ch 153, 5 GHz, SSID broadcast). `beni_video.py --live` feeds `ffmpeg -c copy -f mpegts -listen 1 :8780`. A cloudflared quick tunnel serves it over eth0. The Mac plays it with `ffplay <trycloudflare url>`.
- The link is single-peer: the relay displaces the phone app's session.
- **Session lock:** Beni serves one UCP peer and refuses others (ICMP port unreachable) until that peer's session expires. Take the phone off Beni's Wi-Fi, then sync. Stick frames alone drive Beni once the session is held; the encrypted pilot-mode command is not needed (verified 22:41).
- **Tunnel delivery:** the quick tunnel delivers the stream in 128 KiB blocks every 200–400 ms, whatever the Content-Type (A/B tested `video/hevc` vs `text/event-stream`). Beni→Pi is a steady 60 fps. The pilot absorbs this with a jitter buffer (`--buffer-ms`, default 100).

## Open
- [ ] Pairing key: capture a re-pair (BLE + `e2e_provision` + cloud) to see where it originates
- [ ] Session KDF and AEAD (guess HKDF-SHA256 + AES-GCM or ChaCha20-Poly1305; verify once the key is in hand)
- [ ] Discrete commands: jump, photo, record (session 5) — map them to topic IDs even while encrypted
- [ ] BLE provisioning
