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
| flip back | 4 | 288 |
| flip left | 6 | 416 |
| flip right | 8 | 544 |
| flip forward | 2 | 160 |

Order confirmed by the operator: jump, then back, left, right, forward.

**Mid-air direction jump works** (confirmed live 2026-11-12): a flip code sent within 1.5 s of a jump code fires in the air. This gives Rocket League-style jump, then a jump toward the stick.

**ch2 is head pan** (confirmed live 2026-11-12): it rests at 1000, and 1..1999 pans the head across its range. **Head tilt = low 6 bits of ch3** (capture 2026-11-12 23:55 plus decoded video): ch3 rests at 2015 (`0b11111_011111`, so tilt 31). 0 (ch3 = 1984) looks down and 62 (ch3 = 2046) looks up. The upper 5 bits stay constant. The app's head control is a 2D pad: X → ch2, Y → ch3 tilt.

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

## Reference: open-beni model and policy (read 2026-11-13, open-beni @ 4545b86)
These are the public sim and policy, not measured robot values.
- **Head:**
  - `head_yaw_joint` range ±5.20 rad (±298°). We observed ch2 = 1/1999 at about ±180° on the robot, so ch2's span sits inside the joint range.
  - `head_roll_joint` ±0.05 rad, which "follows roll mechanically".
  - **The model has no head tilt joint.** ch3 tilt drives a camera axis the model does not include.
- **Policy command inputs:**
  - lin_vel_x ±2.0 m/s, ang_vel_z ±5.0 rad/s (lin_vel_y fixed at 0)
  - base pitch and roll ±0.1 rad, height 0.10–0.12 m (nominal 0.11)
  - gimbal yaw ±5 rad plus an enable flag. No reward term uses the gimbal input, so the public policy ignores it.
  - jump: phase, height 0.18 m
- **Policy loop:** 100 Hz (2 ms physics × 5), 512-wide recurrent state, 2–5 substep command delay. Our stick frames are 50 Hz.
- **Skills in the public repo:** drive, jump, recovery (getting up after a fall). **There are no flips.** Flip codes 2/4/6/8 run firmware skills outside this policy.
- **Candidates for unmapped ch4 codes 3, 5, 7, 9–15:**
  - recovery
  - jump at the second reference height (0.25 m motion in `assets/motions`)
  - the pose inputs (pitch, roll, height) could instead sit in the ch5 or ch3 upper bits

  None is verified; each needs a capture or live test.
- **Open:** the physical scale of ch0/ch1. Measure speed and yaw rate at fixed channel values on the robot (video odometry) to fill this in.

## Bench sim (`tools/sim/beni_sim.py`)
The open-beni policy runs in MuJoCo behind a UCP endpoint, so `beni_bridge.py --robot 127.0.0.1:19007` drives it unchanged.
- ch0/ch1 → velocity commands; scales are flags (`--vx-max`, `--wz-max`), not measured.
- ch2 → head yaw (`--pan-max`, default ±π).
- ch3 tilt → render camera pitch only.
- ch4 code 1 → jump. Flip codes are logged only.
- Video: a head-camera render → VideoToolbox HEVC → UCP 0x07/0x05 fragments.
- Checked 2026-11-13: full ch0 holds 2.06 m/s at the default scale, ch2 = 1999 puts the head at +3.14 rad, and a jump reaches base z 0.226 m.

## Open
- [ ] Pairing key: capture a re-pair (BLE + `e2e_provision` + cloud) to see where it originates
- [ ] Session KDF and AEAD (guess HKDF-SHA256 + AES-GCM or ChaCha20-Poly1305; verify once the key is in hand)
- [ ] Discrete commands: jump, photo, record (session 5) — map them to topic IDs even while encrypted
- [ ] BLE provisioning
