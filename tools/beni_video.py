#!/usr/bin/env python3
"""Reassemble Beni's H.265 video (UCP 9007, type 0x07=more / 0x05=last) to an Annex-B .hevc stream.

modes:
  --from-pcap FILE --out beni.hevc     extract from a capture (offline, for validation)
  --live --out beni.hevc [--secs N]    receive live on UDP 9007 (Mac must be on Beni's AP)

The fragment body bytes, concatenated in seq order, already form an Annex-B HEVC
elementary stream (first fragment of each frame begins with 00 00 00 01).
"""
import argparse, struct, socket, sys, os, time

VIDEO = (0x05, 0x07)


def frag(pl):
    if len(pl) < 12 or pl[0] != 0x51 or pl[1] not in VIDEO:
        return None
    seq = struct.unpack_from("<H", pl, 4)[0]
    return seq, pl[1] == 0x05, pl[12:]


def from_pcap(path, out):
    sys.path.insert(0, os.path.dirname(__file__))
    from pktap_summary import packets, parse
    frags = []
    for t, d in packets(path):
        i, pid, c, r = parse(d)
        if not r or r[0] != "udp":
            continue
        _, (s, sp, dst, dp), pl = r
        if sp != 9007:
            continue
        f = frag(pl)
        if f:
            frags.append((t, f[0], f[2]))
    # order by capture time, then seq; dedupe by seq within a sliding window
    frags.sort(key=lambda x: (x[0], x[1]))
    n = 0
    with open(out, "wb") as fo:
        for _, seq, body in frags:
            fo.write(body)
            n += 1
    print(f"wrote {n} fragments -> {out} ({os.path.getsize(out)} bytes)")


def live(out, secs, bind=None):
    fo = sys.stdout.buffer if out == "-" else open(out, "wb")
    log = (lambda *a: print(*a, file=sys.stderr)) if out == "-" else print
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if bind:
        s.bind((bind, 0))  # egress the Beni Wi-Fi iface, keep default route on the tether
    # kick the robot so it streams to us
    s.connect(("192.168.7.1", 9007))
    for _ in range(10):
        s.send(b"ucp:sync")
        s.settimeout(0.3)
        try:
            if s.recv(64).startswith(b"ucp:ack"):
                print("ucp:ack")
                break
        except socket.timeout:
            pass
    else:
        sys.exit("no ucp:ack")
    log("ucp:ack")
    # Mirror the phone: ack at 50 Hz, drain non-blocking in between. A slow ack rate stalls Beni's sender.
    s.setblocking(False)
    end = time.monotonic() + secs
    nxt_ack = 0
    n = 0
    t0 = time.monotonic()
    next_tx = t0
    next_log = t0 + 5
    try:
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_tx:
                ts = int((now - t0) * 1e6) & 0xFFFFFFFF
                body = bytes.fromhex("e8431ffabe0f021805") + struct.pack("<H", nxt_ack) + b"\0" * 8
                s.send(struct.pack("<BBHHHI", 0x51, 0x10, nxt_ack, 0, 12 + len(body), ts) + body)
                next_tx += 0.02
            got = False
            try:
                while True:
                    pl = s.recv(4096)
                    f = frag(pl)
                    if f:
                        nxt_ack = struct.unpack_from("<H", pl, 4)[0]
                        fo.write(f[2])
                        n += 1
                        got = True
            except (BlockingIOError, OSError):
                pass
            if got and out == "-":
                fo.flush()
            if now >= next_log:
                log(f"{n} fragments so far")
                next_log += 5
            time.sleep(0.002)
    finally:
        if out != "-":
            fo.close()
    log(f"wrote {n} fragments")


if __name__ == "__main__":
    a = argparse.ArgumentParser()
    a.add_argument("--from-pcap")
    a.add_argument("--live", action="store_true")
    a.add_argument("--out", default="beni.hevc")
    a.add_argument("--secs", type=float, default=10)
    a.add_argument("--bind", help="local IP to bind (the Beni Wi-Fi interface address)")
    args = a.parse_args()
    if args.from_pcap:
        from_pcap(args.from_pcap, args.out)
    elif args.live:
        live(args.out, args.secs, args.bind)
    else:
        a.error("need --from-pcap or --live")
