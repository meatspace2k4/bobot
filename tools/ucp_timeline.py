#!/usr/bin/env python3
"""Timeline of UDP 9007 (UCP) messages, excluding video fragments.

usage: ucp_timeline.py file.pcap [--after HH:MM:SS] [--dir P>R|R>P] [--new]
--new  only print message shapes (dir,type,len,body[:6]) not seen before --after
"""
import argparse, sys, os, datetime, collections
sys.path.insert(0, os.path.dirname(__file__))
from pktap_summary import packets, parse

a = argparse.ArgumentParser()
a.add_argument("pcap")
a.add_argument("--after")
a.add_argument("--dir")
a.add_argument("--new", action="store_true")
a.add_argument("--max", type=int, default=400)
args = a.parse_args()

cut = None
seen = set()
shown = collections.Counter()
for t, d in packets(args.pcap):
    i, pid, c, r = parse(d)
    if not r or r[0] != "udp":
        continue
    _, (s, sp, dst, dp), pl = r
    if 9007 not in (sp, dp) or len(pl) < 12 or pl[0] != 0x51 or pl[1] in (5, 7):
        continue
    dirn = "R>P" if sp == 9007 else "P>R"
    lt = datetime.datetime.fromtimestamp(t)
    if cut is None and args.after:
        h, m, s_ = map(int, args.after.split(":"))
        cut = lt.replace(hour=h, minute=m, second=s_, microsecond=0)
    shape = (dirn, pl[1], len(pl), pl[12:16])
    before = cut is not None and lt < cut
    if before:
        seen.add(shape)
        continue
    if args.dir and dirn != args.dir:
        continue
    if args.new and shape in seen:
        continue
    if shown[shape] >= args.max:
        continue
    shown[shape] += 1
    print(f"{lt:%H:%M:%S.%f}"[:-3], dirn, f"t={pl[1]:02x}", f"len={len(pl):4d}", pl[12:].hex())
