#!/usr/bin/env python3
"""Summarize an rvictl (DLT_PKTAP) pcap by interface, process and 5-tuple.

usage: pktap_summary.py file.pcap [--proc NAME] [--iface en0] [--dump N]
--dump N prints the first N payloads (hex) of each matching UDP/TCP flow.
"""
import argparse, struct, socket, collections

DLT_PKTAP = (149, 258)


def pcapng_opts(buf):
    i, out = 0, []
    while i + 4 <= len(buf):
        code, ln = struct.unpack_from("<HH", buf, i)
        if code == 0:
            break
        out.append((code, buf[i + 4:i + 4 + ln]))
        i += 4 + ((ln + 3) & ~3)
    return out


def packets_ng(f):
    """Apple pcapng: IDB per interface, PIB (0x80000001) per process, EPB opt 0x8001 = PIB index."""
    ifs, pibs = [], []
    while True:
        h = f.read(8)
        if len(h) < 8:
            return
        bt, bl = struct.unpack("<II", h)
        body = f.read(bl - 8)
        if bt == 1:
            lt = struct.unpack_from("<H", body, 0)[0]
            name = next((v.rstrip(b"\0").decode() for c, v in pcapng_opts(body[8:-4]) if c == 2), "?")
            ifs.append((lt, name))
        elif bt == 0x80000001:
            pid = struct.unpack_from("<I", body, 0)[0]
            name = next((v.rstrip(b"\0").decode(errors="replace") for c, v in pcapng_opts(body[4:-4]) if c == 2), "?")
            pibs.append((pid, name))
        elif bt == 6:
            iid, th, tl, cap = struct.unpack_from("<IIII", body, 0)
            data = body[20:20 + cap]
            pib = None
            for c, v in pcapng_opts(body[20 + ((cap + 3) & ~3):-4]):
                if c == 0x8001 and len(v) >= 4:
                    pib = struct.unpack("<I", v[:4])[0]
            lt, ifname = ifs[iid] if iid < len(ifs) else (1, "?")
            pid, comm = pibs[pib] if pib is not None and pib < len(pibs) else (-1, "")
            yield ((th << 32) | tl) / 1e6, (lt, ifname, pid, comm, data)


def packets(path):
    with open(path, "rb") as f:
        if f.read(4) == b"\x0a\x0d\x0d\x0a":
            f.seek(0)
            yield from packets_ng(f)
            return
        f.seek(0)
        gh = f.read(24)
        magic = gh[:4]
        if magic in (b"\xd4\xc3\xb2\xa1", b"\x4d\x3c\xb2\xa1"):
            e = "<"
        elif magic in (b"\xa1\xb2\xc3\xd4", b"\xa1\xb2\x3c\x4d"):
            e = ">"
        else:
            raise SystemExit("not a classic pcap (pcapng unsupported)")
        nsec = magic in (b"\x4d\x3c\xb2\xa1", b"\xa1\xb2\x3c\x4d")
        linktype = struct.unpack(e + "I", gh[20:24])[0]
        if linktype not in DLT_PKTAP:
            raise SystemExit(f"linktype {linktype} is not PKTAP")
        while True:
            rh = f.read(16)
            if len(rh) < 16:
                return
            ts, tf, incl, _ = struct.unpack(e + "IIII", rh)
            data = f.read(incl)
            if len(data) < incl:
                return
            yield ts + tf / (1e9 if nsec else 1e6), data


def parse(data):
    if isinstance(data, tuple):  # pcapng record
        dlt, ifname, pid, comm, p = data
        if dlt == 1:
            if len(p) < 14:
                return ifname, pid, comm, None
            et = struct.unpack_from(">H", p, 12)[0]
            if et not in (0x0800, 0x86DD):
                return ifname, pid, comm, ("eth", f"0x{et:04x}", len(p))
            p = p[14:]
        elif dlt == 0:
            p = p[4:]
        return ifname, pid, comm, ip(p)
    hlen, _, dlt = struct.unpack_from("<III", data, 0)
    ifname = data[12:36].split(b"\0")[0].decode(errors="replace")
    pid = struct.unpack_from("<i", data, 52)[0]
    comm = data[56:73].split(b"\0")[0].decode(errors="replace")
    p = data[hlen:]
    if dlt == 1:  # ethernet
        if len(p) < 14:
            return ifname, pid, comm, None
        et = struct.unpack_from(">H", p, 12)[0]
        p = p[14:]
        if et not in (0x0800, 0x86DD):
            return ifname, pid, comm, ("eth", f"0x{et:04x}", len(p))
    elif dlt == 0:  # BSD loopback/null
        p = p[4:]
    return ifname, pid, comm, ip(p)


def ip(p):
    if not p:
        return None
    v = p[0] >> 4
    if v == 4:
        ihl = (p[0] & 15) * 4
        proto = p[9]
        src, dst = socket.inet_ntop(socket.AF_INET, p[12:16]), socket.inet_ntop(socket.AF_INET, p[16:20])
        l4 = p[ihl:]
    elif v == 6:
        proto = p[6]
        src, dst = socket.inet_ntop(socket.AF_INET6, p[8:24]), socket.inet_ntop(socket.AF_INET6, p[24:40])
        l4 = p[40:]
    else:
        return None
    sp = dp = 0
    payload = b""
    if proto == 17 and len(l4) >= 8:
        sp, dp = struct.unpack_from(">HH", l4)
        payload = l4[8:]
        name = "udp"
    elif proto == 6 and len(l4) >= 20:
        sp, dp = struct.unpack_from(">HH", l4)
        payload = l4[(l4[12] >> 4) * 4:]
        name = "tcp"
    else:
        name = {1: "icmp", 58: "icmp6"}.get(proto, str(proto))
    return name, (src, sp, dst, dp), payload


def main():
    a = argparse.ArgumentParser()
    a.add_argument("pcap")
    a.add_argument("--proc")
    a.add_argument("--iface")
    a.add_argument("--dump", type=int, default=0)
    a.add_argument("--top", type=int, default=40)
    args = a.parse_args()

    stats = collections.defaultdict(lambda: [0, 0, None, None])  # pkts, bytes, t0, t1
    dumps = collections.defaultdict(list)
    procs = collections.Counter()
    for t, d in packets(args.pcap):
        ifname, pid, comm, r = parse(d)
        procs[(ifname, comm)] += 1
        if r is None or r[0] == "eth":
            continue
        if args.proc and args.proc.lower() not in comm.lower():
            continue
        if args.iface and ifname != args.iface:
            continue
        proto, (s, sp, dst, dp), payload = r
        k = (ifname, comm, proto, s, sp, dst, dp)
        st = stats[k]
        st[0] += 1
        st[1] += len(payload)
        st[2] = st[2] or t
        st[3] = t
        if args.dump and payload and len(dumps[k]) < args.dump:
            dumps[k].append((t, payload))

    print("== packets by interface/process ==")
    for (i, c), n in procs.most_common(25):
        print(f"{n:8d}  {i:10s} {c}")
    print(f"\n== top {args.top} flows by payload bytes ==")
    for k, (n, b, t0, t1) in sorted(stats.items(), key=lambda kv: -kv[1][1])[: args.top]:
        i, c, pr, s, sp, dst, dp = k
        dur = (t1 - t0) if t0 else 0
        rate = n / dur if dur > 0.5 else 0
        print(f"{b:11d}B {n:7d}p {rate:7.1f}pps  {i:9s} {c[:16]:16s} {pr} {s}:{sp} -> {dst}:{dp}")
        for t, pl in dumps.get(k, []):
            print(f"      {t - t0:8.3f}s len={len(pl):5d} {pl[:48].hex()}")


if __name__ == "__main__":
    main()
