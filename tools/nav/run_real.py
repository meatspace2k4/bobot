#!/usr/bin/env python3
"""RGB box tour on the real Beni through beni_bridge. usage: run_real.py http://127.0.0.1:18780 [--hfov 110 --cam-h 0.15 --vmax 1.0]"""
import argparse, math, os, sys, threading, time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from beni_nav import BridgeLink
from rgb_box_tour import RgbBoxTour
from rgbnav import DW, DH
from twin import TwinPublisher

ap = argparse.ArgumentParser()
ap.add_argument("url")
ap.add_argument("--hfov", type=float, default=80)
ap.add_argument("--cam-h", type=float, default=0.19, help="camera height (m); Beni is 21.8 cm tall standing")
ap.add_argument("--vmax", type=float, default=1.0)
ap.add_argument("--spin", type=float, default=0.15)
ap.add_argument("--turn-max", type=float, default=0.3)
ap.add_argument("--tilt", type=float, default=-0.3, help="head tilt command -1 (down) .. 1 (up); keeps the floor in view")
ap.add_argument("--sim-tilt-max", type=float, default=math.radians(45), help="beni_view --tilt-max, to mirror the measured pitch")
ap.add_argument("--max-seconds", type=float, default=150)
ap.add_argument("--standing", action="store_true", help="Beni is standing (policy running). Default: sitting = "
                "disabled: no driving, head checked before a reduced, monitored sweep")
ap.add_argument("--scan-only", action="store_true", help="head-sweep scan and map, then stop (no driving)")
ap.add_argument("--keep", help="URL of a bridge to hold open unarmed (keeps the root bridge and its Wi-Fi alive)")
a = ap.parse_args()

tour = RgbBoxTour(a.hfov, a.cam_h, a.vmax, spin=a.spin, turn_max=a.turn_max)
tour.sitting = not a.standing
tour.sweep_amp = 1.0 if a.standing else 0.6
link = BridgeLink(a.url)
twin = TwinPublisher()
os.makedirs(os.path.join(HERE, "..", "..", "captures", "nav"), exist_ok=True)
vid = os.path.join(HERE, "..", "..", "captures", "nav", time.strftime("real-tour-%Y%m%d-%H%M%S.mp4"))
out = cv2.VideoWriter(vid, cv2.VideoWriter_fourcc(*"mp4v"), 8, (640, 480))
raw = cv2.VideoWriter(vid.replace(".mp4", "-raw.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 8, (960, 720))
stop = threading.Event()
latest = {"rgb": None}


def cam_pose():
    """Camera pose in the start frame: body pose + head pan (survey) at the camera height."""
    pan = tour.turned if tour.phase == "survey" else 0.0
    return (tour.x, tour.y, a.cam_h, tour.yaw + pan if tour.phase != "survey" else pan, tour.per.pitch, tour.per.roll)


def map_points(rgb, P, cam, n=24):
    """Back-project a grid of depth pixels into the start frame (x fwd, y right, z up)."""
    if P is None or P["rho"] is None:
        return None
    f, (cx, cy, cz, yaw) = tour.per.f, cam[:4]
    us, vs = np.meshgrid(np.linspace(4, DW - 5, n), np.linspace(DH * 0.3, DH - 5, n * 3 // 4))
    us, vs = us.ravel().astype(int), vs.ravel().astype(int)
    rho = P["rho"][vs, us]
    ok = rho > 1 / 3.0
    Z = 1 / rho[ok]
    X = (us[ok] - DW / 2) / f * Z
    Y = (vs[ok] - DH / 2) / f * Z
    xs = cx + Z * math.cos(yaw) - X * math.sin(yaw)
    ys = cy + Z * math.sin(yaw) + X * math.cos(yaw)
    zs = cz - Y
    small = cv2.resize(rgb, (DW, DH))
    return np.stack([xs, ys, zs], 1), small[vs[ok], us[ok]]


def worker():
    while not stop.is_set():
        rgb = link.frame(2)
        if rgb is None:
            continue
        raw.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        tour.observe(rgb)
        twin.frame(rgb)
        pts = map_points(rgb, tour.P, cam_pose())
        if pts is not None:
            twin.points(*pts)
        img = cv2.cvtColor(cv2.resize(rgb, (640, 480)), cv2.COLOR_RGB2BGR)
        P = tour.P
        if P is not None and P["obst"] is not None:
            m = cv2.resize(P["obst"].astype(np.uint8), (640, 480), interpolation=cv2.INTER_NEAREST) > 0
            img[m] = (0.5 * img[m] + 0.5 * np.array([40, 40, 230])).astype(np.uint8)
        s = tour.status()
        cv2.putText(img, s, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(img, s, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        out.write(img)


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
t0 = last = time.monotonic()
lastp = ""
try:
    while time.monotonic() - t0 < a.max_seconds:
        now = time.monotonic()
        thr, yc = tour.control(now - last)
        last = now
        link.drive(thr, yc)
        with link.sh.lock:
            link.sh.ctl["h"] = tour.head
            link.sh.ctl["v"] = a.tilt
        # the sim gets the measured camera pitch (not the tilt command) in ch3's tilt bits
        sim_tilt = max(0, min(62, round(31 + tour.per.pitch / a.sim_tilt_max * 31)))
        twin.state([1000 + thr * 999, 1000 + yc * 999, 1000 + tour.head * 999, 1984 | sim_tilt, 32, 546],
                   (tour.x, tour.y, tour.yaw), cam_pose(), tour.per.hfov, "stand" if a.standing else "sit")
        if int(now * 2) != int((now - 1 / 30) * 2):
            twin.marks(tour.order if tour.order else [m for m in tour.marks if m[3] >= 3])
        s = tour.status()
        if s.split()[0] != lastp or (tour.abort and lastp != "abort"):
            lastp = "abort" if tour.abort else s.split()[0]
            print(f"{now - t0:6.1f}s {s}", flush=True)
        if tour.phase == "done" or (a.scan_only and tour.phase == "tour"):
            if a.scan_only:
                twin.marks(tour.order)
                print(f"scan done: {len(tour.order)} objects mapped", flush=True)
                time.sleep(1.0)
            break
        time.sleep(1 / 30)
except KeyboardInterrupt:
    pass
finally:
    stop.set()
    link.close()
    time.sleep(0.3)
    out.release()
    raw.release()
    print(tour.status(), "\nreached:", tour.reached, "\nmarks:", [[round(v, 2) for v in m] for m in tour.marks], f"\n-> {vid}", flush=True)
