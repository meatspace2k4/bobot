#!/usr/bin/env python3
"""Drive the real Beni from the sim (beni_view live scene, F12 arm, arrow keys) and hold it still when idle.

  sim -> robot   while the sim is armed, its sticks (ch0 throttle, ch1 yaw, ch2 head, ch4 jump) go to the bridge;
                 the bridge goes neutral 0.3 s after the last message, so a closed sim or a dropped link stops it
  robot -> sim   state, camera frames, estimated pose (tools/nav/twin.py); the sim Beni is held on that pose
  drift hold     Beni's idle motions (not in the open-source policy) make the real robot creep. With no stick input
                 for 0.5 s the current view becomes the reference: yaw from optical flow, distance from monocular
                 depth of the centre patch; small corrective sticks bring it back. Deadbands keep it from hunting.

usage: run_teleop.py http://127.0.0.1:18781 [--keep http://127.0.0.1:18780] [--no-hold]
"""
import argparse, math, os, sys, threading, time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from beni_nav import BridgeLink
from rgbnav import DH, DW, FlowYaw, Perception
from twin import TwinPublisher

ap = argparse.ArgumentParser()
ap.add_argument("url")
ap.add_argument("--keep", help="bridge URL to hold open unarmed (keeps the root bridge and its Wi-Fi alive)")
ap.add_argument("--hfov", type=float, default=80)
ap.add_argument("--cam-h", type=float, default=0.19, help="camera height (m); Beni is 21.8 cm tall standing")
ap.add_argument("--vmax", type=float, default=1.0, help="m/s at full throttle (dead reckoning)")
ap.add_argument("--tilt", type=float, default=-0.3)
ap.add_argument("--sim-tilt-max", type=float, default=math.radians(45))
ap.add_argument("--no-hold", action="store_true")
ap.add_argument("--hold-yaw-db", type=float, default=3.0, help="deg")
ap.add_argument("--hold-dist-db", type=float, default=0.03, help="m")
ap.add_argument("--hold-max", type=float, default=0.15, help="max corrective stick")
a = ap.parse_args()

link = BridgeLink(a.url)
twin = TwinPublisher()
per = Perception(a.hfov, a.cam_h)
flow = FlowYaw()
lock = threading.Lock()
S = {"yaw": 0.0, "x": 0.0, "y": 0.0, "Z": None, "pitch": 0.0, "roll": 0.0}
stop = threading.Event()


def centre_distance(P):
    if P["rho"] is None:
        return None
    r = P["rho"][int(DH * 0.45):int(DH * 0.75), int(DW * 0.4):int(DW * 0.6)]
    r = r[r > 1 / 4.0]
    return float(1 / np.median(r)) if r.size > 50 else None


def worker():
    while not stop.is_set():
        rgb = link.frame(2)
        if rgb is None:
            continue
        dyaw = flow(rgb, per.f)
        P = per(rgb)
        with lock:
            S["yaw"] += dyaw
            S["Z"] = centre_distance(P)
            S["pitch"], S["roll"] = per.pitch, per.roll
        twin.frame(rgb)


print("waiting for video", flush=True)
if link.frame(30) is None:
    print("no video in 30 s", flush=True)
    link.close()
    sys.exit(1)
threading.Thread(target=worker, daemon=True).start()
if a.keep:
    from beni_pilot import Shared, ws_thread
    keep = Shared(0.0)
    keep.ctl["arm"] = False
    threading.Thread(target=ws_thread, args=(keep, a.keep), daemon=True).start()
print("up: arm in the sim (live scene, F12), arrows drive; idle = drift hold", flush=True)

n = lambda c: max(-1.0, min(1.0, (c - 1000) / 999))
last = time.monotonic()
idle_since, ref, was_armed, last_ch4, mode = last, None, False, 32, ""
try:
    while True:
        now = time.monotonic()
        dt, last = now - last, now
        with lock:
            yaw, Z, pitch, roll = S["yaw"], S["Z"], S["pitch"], S["roll"]
        ctl = twin.sim_ctl()
        thr = yc = head = 0.0
        if ctl:
            ch = ctl["ch"]
            thr, yc, head = n(ch[0]), n(ch[1]), n(ch[2])
            code = (ch[4] >> 6) if ch[4] & 32 else 0
            if code and ch[4] != last_ch4 and code == 1:
                link.action("jump")
            last_ch4 = ch[4]
            if abs(thr) > 0.02 or abs(yc) > 0.02:
                idle_since, ref = now, None
            m = "sim"
        else:
            m = "idle"
        if not ctl or now - idle_since > 0.5:
            # drift hold: reference taken once idle; corrective sticks inside the deadbands are zero
            if ref is None:
                ref = (yaw, Z)
            elif not a.no_hold:
                ey = math.degrees(yaw - ref[0])
                if abs(ey) > a.hold_yaw_db:
                    yc = max(-a.hold_max, min(a.hold_max, -0.01 * ey))
                if Z is not None and ref[1] is not None:
                    ez = Z - ref[1]
                    if abs(ez) > max(a.hold_dist_db, 0.05 * ref[1]):   # depth noise grows with range
                        thr = max(-a.hold_max, min(a.hold_max, 1.5 * ez))
                if yc or thr:
                    m += " hold"
        if m != mode:
            mode = m
            print(f"{time.strftime('%H:%M:%S')} {mode}", flush=True)
        link.drive(thr, yc)
        with link.sh.lock:
            link.sh.ctl["h"] = head
            link.sh.ctl["v"] = a.tilt
        S["x"] += thr * a.vmax * dt * math.cos(yaw)
        S["y"] += thr * a.vmax * dt * math.sin(yaw)
        sim_tilt = max(0, min(62, round(31 + pitch / a.sim_tilt_max * 31)))
        twin.state([1000 + thr * 999, 1000 + yc * 999, 1000 + head * 999, 1984 | sim_tilt, last_ch4, 546],
                   (S["x"], S["y"], yaw), (S["x"], S["y"], a.cam_h, yaw, pitch, roll), per.hfov)
        time.sleep(1 / 30)
except KeyboardInterrupt:
    pass
finally:
    stop.set()
    link.close()
    print("stopped", flush=True)
