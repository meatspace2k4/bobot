#!/usr/bin/env python3
"""Fake Beni for bench-testing beni_bridge. Answers ucp:sync, streams H.265 from a file in UCP
video fragments, and logs the stick channels it receives.

usage: fake_beni.py --hevc beni.hevc [--port 19007] [--max-mb 40]
"""
import argparse, socket, struct, time


def nal_units(data):
    starts, i = [], 0
    while True:
        i = data.find(b"\x00\x00\x00\x01", i)
        if i < 0:
            break
        starts.append(i)
        i += 4
    for a, b in zip(starts, starts[1:] + [len(data)]):
        yield data[a:b]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hevc", required=True)
    ap.add_argument("--port", type=int, default=19007)
    ap.add_argument("--max-mb", type=float, default=40)
    ap.add_argument("--pps", type=float, default=650)
    a = ap.parse_args()
    data = open(a.hevc, "rb").read(int(a.max_mb * 1e6))
    nals = list(nal_units(data))
    print(f"loaded {len(nals)} NAL units", flush=True)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", a.port))
    s.settimeout(0.5)
    peer, seq, k, last_ch, ctl_frames = None, 0, 0, None, 0
    gap = 1.0 / a.pps
    while True:
        try:
            while True:
                p, addr = s.recvfrom(4096)
                if p.startswith(b"ucp:sync"):
                    peer = addr
                    s.sendto(b"ucp:ack", addr)
                    print("sync from", addr, flush=True)
                elif len(p) == 31 and p[0] == 0x51 and p[1] == 0x10:
                    ctl_frames += 1
                    v = int.from_bytes(p[12:21], "little")
                    ch = (v & 0x7FF, (v >> 11) & 0x7FF)
                    if ch != last_ch:
                        print(f"{time.strftime('%H:%M:%S')} sticks ch0={ch[0]} ch1={ch[1]}  (ctl frames {ctl_frames})", flush=True)
                        last_ch = ch
                s.settimeout(0 if peer else 0.5)
        except (socket.timeout, BlockingIOError):
            pass
        if not peer:
            continue
        nal = nals[k % len(nals)]
        k += 1
        frags = [nal[i:i + 1460] for i in range(0, len(nal), 1460)]
        for j, f in enumerate(frags):
            typ = 0x05 if j == len(frags) - 1 else 0x07
            hdr = struct.pack("<BBHHHI", 0x51, typ, 0, seq & 0xFFFF, 12 + len(f), int(time.time() * 1e6) & 0xFFFFFFFF)
            s.sendto(hdr + f, peer)
            seq += 1
            time.sleep(gap)


if __name__ == "__main__":
    main()
