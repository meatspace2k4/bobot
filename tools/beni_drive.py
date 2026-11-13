#!/usr/bin/env python3
"""Minimal Beni stick client over UCP (UDP 9007). Unencrypted stick frames only.

Safety:
  - Output is capped to +/- --cap around 1000 (default 100, i.e. ~10%).
  - Every pulse ends with neutral frames; Ctrl-C or any exception sends neutral too.
  - No stand/sit/mode commands are sent (those are E2E-encrypted).

usage:
  beni_drive.py --selftest
  beni_drive.py --dry-run --throttle 0.1 --secs 1
  beni_drive.py --throttle 0.1 --secs 1            # forward 10% for 1 s
  beni_drive.py --yaw -0.1 --secs 0.5              # left 10% for 0.5 s
"""
import argparse, socket, struct, time, sys

ROBOT = ("192.168.7.1", 9007)
# Captured neutral body (20:20:02-20:20:54 run, ch5=545). Bytes 0..8 = 11-bit channels; 9..10 = video ack.
TEMPLATE = bytes.fromhex("e8431ffabe0f021805")
CAPTURED = {  # body[0:9] hex -> (ch0, ch1) seen in capture
    "e8431ffabe0f021805": (1000, 1000),
    "cff71efabe0f821005": (1999, 990),
}


def chans(b9):
    v = int.from_bytes(b9, "little")
    return [(v >> (11 * i)) & 0x7FF for i in range(6)], v >> 66


def pack(ch0, ch1, template=TEMPLATE):
    v = int.from_bytes(template, "little")
    v &= ~((1 << 22) - 1)
    v |= (ch0 & 0x7FF) | ((ch1 & 0x7FF) << 11)
    return v.to_bytes(9, "little")


def frame(body9, ack, t0):
    ts = int((time.monotonic() - t0) * 1e6) & 0xFFFFFFFF
    body = body9 + struct.pack("<H", ack) + b"\0" * 8
    hdr = struct.pack("<BBHHHI", 0x51, 0x10, ack, 0, 12 + len(body), ts)
    return hdr + body


def selftest():
    ok = True
    for hx, (c0, c1) in CAPTURED.items():
        b = bytes.fromhex(hx)
        got = chans(b)[0][:2]
        rt = pack(c0, c1, b) == b
        print(hx, "decode", got, "expect", (c0, c1), "roundtrip", rt)
        ok &= got == [c0, c1] and rt
    f = frame(pack(1000, 1000), 0x448D, time.monotonic())
    print("neutral frame", f.hex(), len(f))
    ok &= len(f) == 31 and f[12:21] == TEMPLATE
    print("SELFTEST", "PASS" if ok else "FAIL")
    return ok


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--throttle", type=float, default=0.0, help="-1..1")
    a.add_argument("--yaw", type=float, default=0.0, help="-1 left .. 1 right")
    a.add_argument("--secs", type=float, default=1.0)
    a.add_argument("--cap", type=int, default=100, help="max |ch-1000|")
    a.add_argument("--hz", type=float, default=50)
    a.add_argument("--dry-run", action="store_true")
    a.add_argument("--selftest", action="store_true")
    args = a.parse_args()
    if args.selftest:
        sys.exit(0 if selftest() else 1)
    if args.secs > 3:
        sys.exit("refusing pulses longer than 3 s")

    def ch(x):
        return 1000 + max(-args.cap, min(args.cap, int(round(x * 999))))

    c0, c1 = ch(args.throttle), ch(args.yaw)
    print(f"pulse ch0={c0} ch1={c1} for {args.secs}s (cap ±{args.cap})")
    t0 = time.monotonic()
    if args.dry_run:
        print("active ", frame(pack(c0, c1), 0, t0).hex())
        print("neutral", frame(pack(1000, 1000), 0, t0).hex())
        return

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0.3)
    s.connect(ROBOT)
    for _ in range(10):
        s.send(b"ucp:sync")
        try:
            if s.recv(2048).startswith(b"ucp:ack"):
                print("ucp:ack received")
                break
        except socket.timeout:
            pass
    else:
        sys.exit("no ucp:ack from robot")
    s.setblocking(False)
    ack = 0
    period = 1 / args.hz
    rx = 0

    def pump(body9, dur):
        nonlocal ack, rx
        end = time.monotonic() + dur
        while time.monotonic() < end:
            s.send(frame(body9, ack, t0))
            try:
                while True:
                    p = s.recv(4096)
                    rx += 1
                    if len(p) >= 6 and p[0] == 0x51 and p[1] in (5, 7):
                        ack = struct.unpack_from("<H", p, 4)[0]
            except (BlockingIOError, OSError):
                pass
            time.sleep(period)

    try:
        pump(pack(1000, 1000), 1.0)          # neutral lead-in
        pump(pack(c0, c1), args.secs)        # pulse
    finally:
        pump(pack(1000, 1000), 1.0)          # neutral tail (dead-man)
        print(f"done; robot packets received: {rx}, last video ack {ack:#06x}")


if __name__ == "__main__":
    main()
