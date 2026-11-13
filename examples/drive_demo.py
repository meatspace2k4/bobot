#!/usr/bin/env python3
"""Open-loop demo: forward, turn, jump, look around, record video meanwhile.

usage: python3 examples/drive_demo.py [--host 127.0.0.1 --port 19007]   (defaults to the robot)
"""
import argparse, os, sys, threading, time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from beni import Beni

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="192.168.7.1")
ap.add_argument("--port", type=int, default=9007)
ap.add_argument("--video", default="demo.hevc")
a = ap.parse_args()

with Beni(a.host, a.port) as b:
    out = open(a.video, "wb")
    stop = threading.Event()

    def record():
        for au in b.frames():
            out.write(au)
            if stop.is_set():
                return

    threading.Thread(target=record, daemon=True).start()
    steps = [
        ("forward", lambda: b.drive(throttle=0.4), 2.0),
        ("turn right", lambda: b.drive(throttle=0.0, yaw=0.5), 1.5),
        ("stop", b.stop, 1.0),
        ("jump", lambda: b.action("jump"), 2.0),
        ("look left", lambda: b.head(pan=-0.5), 1.0),
        ("look up", lambda: b.head(pan=0.0, tilt=0.8), 1.0),
        ("center", b.stop, 1.0),
    ]
    for name, do, secs in steps:
        do()
        time.sleep(0.05)  # let the 50 Hz loop send it
        print(f"{name:10s} sent ch0-4 {b.sent}", flush=True)
        time.sleep(secs - 0.05)
    stop.set()
    print(f"{b.frames_rx} video frames, saved to {a.video}")
