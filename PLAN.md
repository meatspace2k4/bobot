# Beni (Mondo Robotics) — protocol RE plan

## System model (what we expect to find)
Beni is a two-wheel self-balancing robot (inverted pendulum) with a 4K camera, jump actuator, self-righting, person/dog tracking, and a wrist controller. The balance loop, tracking, and recording all run on the robot. The phone only sends high-level commands and receives video and telemetry. Expected links:

| Link | Likely transport | What rides on it | Capture tool |
|---|---|---|---|
| Phone ↔ cloud | HTTPS/WSS (AWS) | account, pairing tokens, OTA firmware, AI-edit uploads | mitmproxy (running, :8080) |
| Phone ↔ robot, setup | BLE GATT | discovery, Wi-Fi provisioning, maybe keys | iOS Bluetooth logging profile + PacketLogger |
| Phone ↔ robot, live | Wi-Fi (robot AP or LAN): UDP/TCP, likely RTSP/WebRTC/custom | drive setpoints (v, ω), jump/photo triggers, mode, telemetry (IMU, battery, wheel state), live view | rvictl `rvi0` + tcpdump/Wireshark |
| Wrist controller ↔ robot | BLE or 2.4 GHz proprietary | joystick axes, triggers | PacketLogger if it pairs through the phone; otherwise SDR (out of scope for now) |
| File transfer | Wi-Fi HTTP/FTP-like | media pull | rvictl / mitmproxy if it's HTTP |

Look for: a control packet sent at a fixed rate (20–100 Hz) with two signed axes; a telemetry stream with pitch/roll and battery; a heartbeat or watchdog (the robot should stop if commands stop).

## Capture setup
- **Mac**: mitmweb on `192.168.10.165:8080` (UI at http://127.0.0.1:8081), flows saved to `captures/*.flows`. Its CA is already in `~/.mitmproxy`.
- **iPhone 12** (`00008101-0010555E0A84001E`, the one attached over USB): HTTP proxy pointed at the Mac, mitmproxy CA installed and fully trusted. Install the Bluetooth logging profile for BLE.
- **Beni hosts its own Wi-Fi (confirmed)**: while the phone is joined to Beni, its internet goes over cellular and the Mac proxy can't be reached.
  - Live robot link: captured by `rvi0` over USB (`captures/beni-rvi-*.pcap`), which sees the Wi-Fi and cellular interfaces with no proxy needed.
  - Cloud API: captured in a separate session on home Wi-Fi with the proxy (login, device list, OTA, media upload). Cellular TLS seen on rvi0 is ciphertext only.
  - Optional: the Mac's Wi-Fi (en0) can also join Beni's AP so that robot traffic can be proxied. This drops the Mac off the LAN while it's joined.

## Sessions (one action per capture, timestamped)
1. Cold start the app, log in, view the device list (cloud API, auth scheme)
2. Power on Beni and connect (BLE discovery, then the Wi-Fi handoff)
3. Live view while idle (video transport and codec)
4. Pilot mode: forward only, then turn only, then stop (isolate the control axes)
5. Jump, photo, record start/stop (discrete command opcodes)
6. Follow mode on/off (mode switch)
7. Pull one clip (media transfer)
8. Check for firmware updates (OTA endpoint; grab the firmware image for static analysis)

## Fallbacks
- **Cert pinning**: the App Store IPA is FairPlay-encrypted, so Frida needs a decrypted IPA (a jailbroken device) or a gadget-injected re-sign. Decide only if session 1 shows pinning failures.
- **Encrypted LAN payloads**: get the keys from the cloud pairing response or BLE provisioning. Otherwise use the firmware from the OTA capture.

## Deliverable
`protocol.md` (message catalog: transport, framing, opcodes, fields, units, rates) plus a Python client that connects, reads telemetry, and drives Beni.
