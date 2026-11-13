#!/usr/bin/env python3
"""Per-capture trace of stick channels (phone->robot 0x10) and encrypted layer-2 data messages.

Outputs <out>.sticks.csv  t,ch0..ch5
        <out>.topics.csv  t,dir,topic,env_len,counter
Data message: 5d <topic> 41 1a <varint> 18 01 22 <varint env_len> <session[8] counter[8,BE] ct tag>

usage: topic_trace.py capture.pcap --out prefix
"""
import argparse, os, sys
sys.path.insert(0, os.path.dirname(__file__))
from pktap_summary import packets, parse


def varint(b, i):
    v = s = 0
    while i < len(b):
        x = b[i]; i += 1
        v |= (x & 0x7F) << s; s += 7
        if not x & 0x80:
            return v, i
    return None, i


def data_msgs(body):
    i = 0
    while (i := body.find(b"\x5d", i)) >= 0:
        j = i
        if j + 4 < len(body) and body[j + 2] == 0x41 and body[j + 3] == 0x1a:
            topic = body[j + 1]
            _, k = varint(body, j + 4)
            if k + 3 < len(body) and body[k:k + 3] == b"\x18\x01\x22":
                n, m = varint(body, k + 3)
                if n and m + n <= len(body) and n >= 32:
                    env = body[m:m + n]
                    yield topic, n, int.from_bytes(env[8:16], "big")
                    i = m + n
                    continue
        i += 1


def main():
    a = argparse.ArgumentParser()
    a.add_argument("pcap")
    a.add_argument("--out", required=True)
    args = a.parse_args()
    st = open(args.out + ".sticks.csv", "w")
    tp = open(args.out + ".topics.csv", "w")
    st.write("t,ch0,ch1,ch2,ch3,ch4,ch5\n")
    tp.write("t,dir,topic,env_len,counter\n")
    ns = nt = 0
    for t, d in packets(args.pcap):
        _, _, _, r = parse(d)
        if not r or r[0] != "udp":
            continue
        _, (s, sp, dst, dp), pl = r
        if 9007 not in (sp, dp) or len(pl) < 12 or pl[0] != 0x51 or pl[1] in (5, 7):
            continue
        dirn = "R>P" if sp == 9007 else "P>R"
        if dirn == "P>R" and pl[1] == 0x10 and len(pl) == 31:
            v = int.from_bytes(pl[12:21], "little")
            st.write(f"{t:.4f}," + ",".join(str((v >> (11 * i)) & 0x7FF) for i in range(6)) + "\n")
            ns += 1
        elif pl[1] in (0x0d, 0x10):
            for topic, n, ctr in data_msgs(pl[12:]):
                tp.write(f"{t:.4f},{dirn},{topic},{n},{ctr}\n")
                nt += 1
    print(f"{ns} stick frames, {nt} data messages -> {args.out}.*.csv")


if __name__ == "__main__":
    main()
