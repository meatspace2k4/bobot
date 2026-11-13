#!/usr/bin/env python3
"""RGB waypoint navigation for Beni: monocular depth -> obstacle map -> steering, all from the head camera.

Beni's only cleartext sensor is the camera, so everything comes from RGB:
  depth      Depth Anything V2 Small (relative inverse depth, ~30 ms on MPS)
  metric     the floor fixes the scale: floor disparity is linear in image row, d = k (v - v0), and a floor
             point at row v lies at Z = h f / (v - v0) for camera height h. So 1/Z = d / (k h f) everywhere.
  obstacles  pixels whose disparity stands above the floor line (closer than the floor at that row);
             per bearing, the nearest one is the free range
  heading    optical flow (LK) of the upper image: horizontal shift / f = yaw change
  speed      metric depth of tracked features, frame to frame (falls back to throttle * --vscale)
  steering   the box tour's law from the sim (tools/sim/beni_view.py BoxTour): turn = 1.8 * err, forward
             only within 0.6 rad, both capped at 0.6 stick, with the target bearing swapped for the free
             bearing nearest to it (vector field histogram)
Beni's own braking stays in force underneath.

usage:
  beni_nav.py --ucp 127.0.0.1:19007 --waypoints "2,0 2,2"     sim bench (tools/sim/launch.sh bench)
  beni_nav.py --bridge http://127.0.0.1:18780 --waypoints ...  real Beni via the Pi bridge
  beni_nav.py --video clip.hevc                                offline: perception on a recording
Waypoints are metres, x forward and y right of the start pose. Output: annotated MP4 + CSV in captures/nav/.
"""
import argparse, csv, math, os, queue, subprocess, sys, threading, time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "mac"))
sys.path.insert(0, os.path.join(HERE, "..", ".."))

DW, DH = 308, 238            # depth input (multiples of 14)
BANDS = 48
R_MAX = 3.0                  # m, free range cap
FFMPEG = "/opt/homebrew/bin/ffmpeg"
STICK_MAX = 0.6              # as BoxTour


# ---------------------------------------------------------------- perception
class Depth:
    def __init__(self):
        import torch
        from transformers import AutoModelForDepthEstimation
        self.torch = torch
        self.dev = "mps" if torch.backends.mps.is_available() else "cpu"
        self.m = AutoModelForDepthEstimation.from_pretrained("depth-anything/Depth-Anything-V2-Small-hf").eval().to(self.dev)
        self.mean = np.array([0.485, 0.456, 0.406], np.float32)
        self.std = np.array([0.229, 0.224, 0.225], np.float32)

    def __call__(self, rgb):
        x = cv2.resize(rgb, (DW, DH), interpolation=cv2.INTER_AREA).astype(np.float32) / 255
        x = ((x - self.mean) / self.std).transpose(2, 0, 1)[None]
        with self.torch.no_grad():
            d = self.m(pixel_values=self.torch.from_numpy(np.ascontiguousarray(x)).to(self.dev)).predicted_depth
        return d[0].float().cpu().numpy()


class Floor:
    """Fits d = k (v - v0) on the floor patch ahead; holds the last good fit."""

    def __init__(self):
        self.k = self.v0 = None

    def fit(self, d):
        r0, r1 = int(DH * 0.72), int(DH * 0.97)
        c0, c1 = int(DW * 0.3), int(DW * 0.7)
        rows = np.arange(r0, r1)
        med = np.median(d[r0:r1, c0:c1], axis=1)
        A = np.vstack([rows, np.ones_like(rows)]).T.astype(np.float64)
        (a, b), res, *_ = np.linalg.lstsq(A, med, rcond=None)
        if a <= 0:
            return False
        v0 = -b / a
        resid = np.abs(A @ [a, b] - med).mean() / max(1e-6, med.mean())
        if not (-1.5 * DH < v0 < 0.7 * DH) or resid > 0.08:
            return False
        if self.k is None:
            self.k, self.v0 = a, v0
        else:
            self.k, self.v0 = 0.8 * self.k + 0.2 * a, 0.8 * self.v0 + 0.2 * v0
        return True


class Perception:
    def __init__(self, hfov_deg, cam_h):
        self.depth = Depth()
        self.floor = Floor()
        self.set_hfov(hfov_deg)
        self.h = cam_h

    def set_hfov(self, hfov_deg):
        self.hfov = hfov_deg
        self.f = (DW / 2) / math.tan(math.radians(hfov_deg) / 2)   # focal at depth-map resolution

    def __call__(self, rgb):
        d = self.depth(rgb)
        ok = self.floor.fit(d)
        out = {"disp": d, "floor_ok": ok, "rho": None, "free": np.full(BANDS, R_MAX), "obst": None}
        if self.floor.k is None:
            return out
        k, v0 = self.floor.k, self.floor.v0
        rho = np.clip(d, 0, None) / (k * self.h * self.f)          # 1/Z in 1/m
        rows = np.arange(DH)[:, None].astype(np.float32)
        floor_d = k * (rows - v0)
        obst = (d > np.maximum(floor_d * 1.15, 0) + 0.04 * k * DH) & (rho > 1 / R_MAX)
        obst[: max(0, int(v0) - int(0.6 * DH)), :] = False             # far ceiling region
        obst = cv2.morphologyEx(obst.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)).astype(bool)
        bw = DW / BANDS
        free = np.full(BANDS, R_MAX)
        for b in range(BANDS):
            c0, c1 = int(b * bw), int((b + 1) * bw)
            r = rho[:, c0:c1][obst[:, c0:c1]]
            if r.size > 15:
                free[b] = min(R_MAX, 1.0 / np.percentile(r, 95))
        out.update(rho=rho, free=free, obst=obst)
        return out

    def bearing(self, b):
        return math.atan2((b + 0.5) * DW / BANDS - DW / 2, self.f)


class Odometry:
    """Yaw from LK flow of the upper image; speed from metric depth of tracked features."""

    def __init__(self):
        self.prev = None
        self.yaw = 0.0
        self.px = 0.0
        self.v = 0.0
        self.n = 0

    def update(self, gray, rho_prev, rho, f, dt):
        """gray at DWxDH; rho maps at the same size (or None). Returns (dyaw, v or None)."""
        if self.prev is None:
            self.prev = gray
            return 0.0, None
        p0 = cv2.goodFeaturesToTrack(self.prev, 300, 0.01, 5)
        dyaw, v = 0.0, None
        self.n = 0
        if p0 is not None and len(p0) >= 10:
            p1, st, _ = cv2.calcOpticalFlowPyrLK(self.prev, gray, p0, None, winSize=(21, 21), maxLevel=3)
            ok = st.ravel() == 1
            a, b = p0[ok].reshape(-1, 2), p1[ok].reshape(-1, 2)
            if len(a) >= 10:
                upper = a[:, 1] < DH * 0.55
                dx = (b - a)[upper, 0] if upper.sum() >= 8 else (b - a)[:, 0]
                med = np.median(dx)
                good = np.abs(dx - med) < 2.5
                self.n = int(good.sum())
                if self.n >= 6:
                    sh = float(np.median(dx[good]))
                    dyaw = -sh / f
                    self.px += -sh
                if rho_prev is not None and rho is not None and dt > 0:
                    ia = np.clip(a.astype(int), 0, [DW - 1, DH - 1])
                    ib = np.clip(b.astype(int), 0, [DW - 1, DH - 1])
                    r0, r1 = rho_prev[ia[:, 1], ia[:, 0]], rho[ib[:, 1], ib[:, 0]]
                    sel = (r0 > 0.4) & (r1 > 0.4) & (r0 < 5) & (r1 < 5)    # 0.2 .. 2.5 m
                    if sel.sum() >= 10:
                        dz = 1 / r1[sel] - 1 / r0[sel]
                        v = float(-np.median(dz) / dt)
        self.prev = gray
        self.yaw += dyaw
        return dyaw, v


# ---------------------------------------------------------------- links
class UcpLink:
    """Direct UCP (sim bench or a host on Beni's Wi-Fi) with the beni SDK; frames decoded by ffmpeg."""

    def __init__(self, hostport, mode=None):
        from beni import Beni
        host, port = hostport.rsplit(":", 1)
        self.b = Beni(host, int(port)).connect()
        if mode is not None:
            self.b.mode(mode)
        self.dec = subprocess.Popen([FFMPEG, "-loglevel", "error", "-fflags", "nobuffer", "-flags", "low_delay",
                                     "-probesize", "32k", "-analyzeduration", "0", "-f", "hevc", "-i", "pipe:0", "-vf", "scale=960:720", "-pix_fmt", "rgb24",
                                     "-fps_mode", "passthrough", "-f", "rawvideo", "pipe:1"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        self.q = queue.Queue(maxsize=4)
        threading.Thread(target=self._feed, daemon=True).start()
        threading.Thread(target=self._read, daemon=True).start()

    def _feed(self):
        try:
            for au in self.b.frames():
                self.dec.stdin.write(au)
        except Exception:
            pass

    def _read(self):
        n = 960 * 720 * 3
        while True:
            buf = bytearray(n)
            mv, got = memoryview(buf), 0
            while got < n:
                k = self.dec.stdout.readinto(mv[got:])
                if not k:
                    return
                got += k
            if self.q.full():
                try:
                    self.q.get_nowait()
                except queue.Empty:
                    pass
            self.q.put(np.frombuffer(buf, np.uint8).reshape(720, 960, 3))

    def frame(self, timeout=5.0):
        try:
            f = self.q.get(timeout=timeout)
            while not self.q.empty():
                f = self.q.get_nowait()
            return f
        except queue.Empty:
            return None

    def drive(self, t, y):
        self.b.drive(throttle=t, yaw=y)

    def action(self, name):
        self.b.action(name)

    def close(self):
        self.b.stop()
        time.sleep(0.3)
        self.b.close()
        self.dec.kill()


class BridgeLink:
    """Real Beni through beni_bridge on the Pi (video + 30 Hz control), reusing beni_pilot's threads."""

    def __init__(self, url):
        from beni_pilot import Shared, video_thread, ws_thread
        self.sh = Shared(1.0)
        self.sh.ctl["arm"] = True
        threading.Thread(target=ws_thread, args=(self.sh, url), daemon=True).start()
        threading.Thread(target=video_thread, args=(self.sh, url), daemon=True).start()
        self.last = None

    def frame(self, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.sh.lock:
                f = self.sh.frames[-1] if self.sh.frames else None
                self.sh.frames.clear()
            if f is not None and f is not self.last:
                self.last = f
                return np.frombuffer(f, np.uint8).reshape(720, 960, 3)
            time.sleep(0.003)
        return None

    def drive(self, t, y):
        with self.sh.lock:
            self.sh.ctl["t"], self.sh.ctl["y"] = float(t), float(y)

    def action(self, name):
        with self.sh.lock:
            self.sh.ctl["action"] = name

    def close(self):
        self.drive(0, 0)
        time.sleep(0.5)
        with self.sh.lock:
            self.sh.ctl["arm"] = False
        time.sleep(0.3)
        self.sh.quit.set()


class FileLink:
    def __init__(self, path, skip=2):
        self.p = subprocess.Popen([FFMPEG, "-loglevel", "error", "-i", path, "-vf", "scale=960:720",
                                   "-pix_fmt", "rgb24", "-f", "rawvideo", "-"], stdout=subprocess.PIPE)
        self.skip = skip

    def frame(self, timeout=0):
        n = 960 * 720 * 3
        for _ in range(self.skip):
            b = self.p.stdout.read(n)
            if len(b) < n:
                return None
        return np.frombuffer(b, np.uint8).reshape(720, 960, 3)

    def drive(self, t, y):
        pass

    def action(self, name):
        pass

    def close(self):
        self.p.kill()


# ---------------------------------------------------------------- planning
def choose(free, per, target, need, half_width=0.17):
    """Free bearing nearest the target. A bearing is passable if every band whose ray passes within
    half_width of the path, up to `need` metres, is free that far."""
    best = None
    for b in range(BANDS):
        a = per.bearing(b)
        ok = True
        for c in range(BANDS):
            dc = abs(per.bearing(c) - a)
            if dc > 1.2:
                continue
            reach = need if dc < 1e-3 else min(need, half_width / math.sin(dc)) if dc < math.pi / 2 else 0
            if free[c] < reach * math.cos(dc) - 0.02:
                ok = False
                break
        if ok and (best is None or abs(a - target) < abs(best - target)):
            best = a
    return best


def draw(rgb, P, per, st, path, wps, chosen, target, text):
    img = cv2.cvtColor(cv2.resize(rgb, (640, 480)), cv2.COLOR_RGB2BGR)
    if P["obst"] is not None:
        m = cv2.resize(P["obst"].astype(np.uint8), (640, 480), interpolation=cv2.INTER_NEAREST) > 0
        img[m] = (0.45 * img[m] + 0.55 * np.array([40, 40, 230])).astype(np.uint8)
    d = P["disp"]
    dn = cv2.applyColorMap(cv2.normalize(d, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8), cv2.COLORMAP_INFERNO)
    img[:120, 640 - 160:] = cv2.resize(dn, (160, 120))
    def ray(a, col):
        x = int(320 + math.tan(max(-1.3, min(1.3, a))) * per.f * 640 / DW)
        cv2.line(img, (320, 479), (x, 330), col, 3)
    if target is not None:
        ray(target, (255, 140, 0))
    if chosen is not None:
        ray(chosen, (0, 255, 0))
    # top-down: waypoints, path, free-range polygon
    S, sc = 220, 50.0
    inset = np.full((S, S, 3), 25, np.uint8)
    ox, oy = S // 2, S // 2 + 40
    def Pt(x, y):
        return (int(ox + y * sc), int(oy - x * sc))
    x, y, yaw = st["x"], st["y"], st["yaw"]
    poly = [Pt(x, y)]
    for b in range(BANDS):
        a = yaw + per.bearing(b)
        poly.append(Pt(x + P["free"][b] * math.cos(a), y + P["free"][b] * math.sin(a)))
    cv2.fillPoly(inset, [np.array(poly)], (50, 90, 50))
    for i, (wx, wy) in enumerate(wps):
        cv2.circle(inset, Pt(wx, wy), 5, (0, 200, 255) if i >= st["wi"] else (90, 90, 90), -1)
    for i in range(1, len(path)):
        cv2.line(inset, Pt(*path[i - 1]), Pt(*path[i]), (0, 255, 0), 2)
    cv2.arrowedLine(inset, Pt(x, y), Pt(x + 0.3 * math.cos(yaw), y + 0.3 * math.sin(yaw)), (255, 255, 255), 2)
    img[480 - S:, 640 - S:] = inset
    for i, t in enumerate(text):
        cv2.putText(img, t, (8, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 3)
        cv2.putText(img, t, (8, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1)
    return img


# ---------------------------------------------------------------- main loop
def run(a):
    if a.video:
        link = FileLink(a.video, a.skip)
    elif a.ucp:
        link = UcpLink(a.ucp, a.ch5)
    else:
        link = BridgeLink(a.bridge)
    per = Perception(a.hfov, a.cam_h)
    od = Odometry()
    stamp = time.strftime("%Y%m%d-%H%M%S")
    os.makedirs(a.outdir, exist_ok=True)
    base = os.path.join(a.outdir, f"nav-{a.tag}-{stamp}" if a.tag else f"nav-{stamp}")
    out = cv2.VideoWriter(base + ".mp4", cv2.VideoWriter_fourcc(*"mp4v"), 10, (640, 480))
    logf = open(base + ".csv", "w")
    log = csv.writer(logf)
    log.writerow(["t", "phase", "x", "y", "yaw_deg", "v_est", "wi", "target_deg", "chosen_deg", "free_ahead", "thr", "yaw_cmd", "floor_ok"])
    wps = [tuple(map(float, p.split(","))) for p in a.waypoints.split()] if a.waypoints else []
    st = {"x": 0.0, "y": 0.0, "yaw": 0.0, "wi": 0}
    path = [(0.0, 0.0)]
    print("waiting for video...", flush=True)
    rgb = link.frame(15)
    if rgb is None:
        print("no video", flush=True)
        link.close()
        return
    t0 = tl = time.monotonic()
    phase = "settle"
    hold = 0.0
    thr = yc = 0.0
    rho_prev = None
    loop_key = spin_t0 = None
    v_f = 0.0
    nframe = 0
    try:
        while True:
            if nframe:
                rgb = link.frame()
            nframe += 1
            if rgb is None:
                print("video ended", flush=True)
                break
            now = time.monotonic() if not a.video else t0 + nframe * a.skip / 25.0
            dt, tl = now - tl, now
            P = per(rgb)
            gray = cv2.cvtColor(cv2.resize(rgb, (DW, DH), interpolation=cv2.INTER_AREA), cv2.COLOR_RGB2GRAY)
            dyaw, v = od.update(gray, rho_prev, P["rho"], per.f, dt)
            rho_prev = P["rho"]
            if v is not None and abs(dyaw) / max(dt, 1e-3) < 0.3 and abs(v) < 2:
                v_f = 0.7 * v_f + 0.3 * v
            elif v is None:
                v_f *= 0.8
            target = chosen = None
            extra = []
            ahead = float(np.min(P["free"][BANDS // 2 - 3: BANDS // 2 + 3]))
            if phase == "settle":
                thr = yc = 0.0
                if now - t0 > a.settle:
                    od.yaw = 0.0
                    if a.video or not wps:
                        phase = "watch"
                    elif a.calibrate:
                        loop_key = cv2.ORB_create(800).detectAndCompute(gray, None)
                        spin_t0, od.px = now, 0.0
                        phase = "calibrate"
                    else:
                        phase = "nav"
            elif phase == "calibrate":
                thr, yc = 0.0, a.spin
                orb = cv2.ORB_create(800)
                kp, des = orb.detectAndCompute(gray, None)
                off, n = None, 0
                if des is not None and loop_key[1] is not None:
                    m = [x for x in cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(loop_key[1], des) if x.distance < 40]
                    if len(m) >= 20:
                        dd = np.array([(kp[x.trainIdx].pt[0] - loop_key[0][x.queryIdx].pt[0],
                                        kp[x.trainIdx].pt[1] - loop_key[0][x.queryIdx].pt[1]) for x in m])
                        mx, my = np.median(dd, axis=0)
                        agree = (np.abs(dd[:, 0] - mx) < 6) & (np.abs(dd[:, 1] - my) < 6)
                        n = int(agree.sum())
                        if n >= 15:
                            off = float(np.median(dd[agree, 0]))
                extra.append(f"calibrating: {od.px:.0f} px turned, start view matches {n}")
                if od.px > 1.3 * DW and off is not None and abs(off) < 10:
                    total = od.px + off
                    f = total / (2 * math.pi)
                    per.set_hfov(math.degrees(2 * math.atan((DW / 2) / f)))
                    print(f"calibrated: 360 deg = {total:.0f} px -> hfov {per.hfov:.1f} deg", flush=True)
                    od.yaw, thr, yc, phase = 0.0, 0.0, 0.0, "nav"
                elif now - spin_t0 > 40:
                    print("no loop closure in 40 s; keeping --hfov", flush=True)
                    od.yaw, phase = 0.0, "nav"
            elif phase == "nav":
                if hold > 0:
                    hold -= dt
                    thr = yc = 0.0
                    if hold <= 0:
                        st["wi"] += 1
                elif st["wi"] >= len(wps):
                    thr = yc = 0.0
                    phase = "done"
                    done_t = now
                else:
                    wx, wy = wps[st["wi"]]
                    dx, dy = wx - st["x"], wy - st["y"]
                    dist = math.hypot(dx, dy)
                    if dist <= a.reach:
                        print(f"waypoint {st['wi']} ({wx:g},{wy:g}) reached; pose ({st['x']:.2f},{st['y']:.2f}) yaw {math.degrees(od.yaw):.0f}", flush=True)
                        hold, thr, yc = 1.0, 0.0, 0.0
                    else:
                        target = (math.atan2(dy, dx) - od.yaw + math.pi) % (2 * math.pi) - math.pi
                        half = math.atan2(DW / 2, per.f) * 0.85
                        if abs(target) > half:
                            chosen = None
                            err = target
                            fwd = 0.0
                        else:
                            chosen = choose(P["free"], per, target, min(dist, a.lookahead))
                            if chosen is None:
                                left, right = P["free"][: BANDS // 2].mean(), P["free"][BANDS // 2:].mean()
                                err = 0.6 if right >= left else -0.6
                                fwd = 0.0
                            else:
                                err = chosen
                                fwd = 0.0 if abs(err) > 0.6 else max(0., min(STICK_MAX, 0.6 * dist + 0.1)) * math.cos(err)
                                fwd *= max(0.3, min(1.0, (ahead - 0.25) / 0.6))
                        thr = min(fwd, a.speed)
                        yc = max(-1., min(1., 1.8 * err)) * STICK_MAX * a.turn_gain
                        extra.append(f"waypoint {st['wi']} ({wx:g},{wy:g})  dist {dist:.2f} m")
            elif phase == "done":
                thr = yc = 0.0
                if now - done_t > 1.5:
                    break
            link.drive(thr, yc)
            # dead reckoning: heading from flow; speed from depth when available, else commanded
            st["yaw"] = od.yaw
            if phase == "nav":
                vv = v_f if a.depth_speed else thr * a.vscale
                st["x"] += vv * dt * math.cos(od.yaw)
                st["y"] += vv * dt * math.sin(od.yaw)
                if math.hypot(st["x"] - path[-1][0], st["y"] - path[-1][1]) > 0.02:
                    path.append((st["x"], st["y"]))
            text = [f"{phase} {now - t0:5.1f}s  pos ({st['x']:+.2f},{st['y']:+.2f})  yaw {math.degrees(od.yaw):+6.1f}  v {v_f:+.2f}",
                    f"thr {thr:+.2f} yaw {yc:+.2f}  ahead {ahead:.2f} m  floor {'ok' if P['floor_ok'] else '--'}  hfov {per.hfov:.0f}"] + extra
            out.write(draw(rgb, P, per, st, path, wps, chosen, target, text))
            log.writerow([f"{now - t0:.3f}", phase, f"{st['x']:.3f}", f"{st['y']:.3f}", f"{math.degrees(od.yaw):.1f}",
                          f"{v_f:.3f}", st["wi"], "" if target is None else f"{math.degrees(target):.1f}",
                          "" if chosen is None else f"{math.degrees(chosen):.1f}", f"{ahead:.2f}",
                          f"{thr:.2f}", f"{yc:.2f}", int(P["floor_ok"])])
            if a.max_seconds and now - t0 > a.max_seconds:
                print("time limit", flush=True)
                break
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        out.release()
        logf.close()
        print(f"-> {base}.mp4", flush=True)


def main():
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--ucp", help="host:port of a UCP Beni (sim bench: 127.0.0.1:19007)")
    src.add_argument("--bridge", help="beni_bridge URL")
    src.add_argument("--video", help="offline: perception on a recording")
    ap.add_argument("--skip", type=int, default=3, help="offline: use every Nth frame")
    ap.add_argument("--ch5", type=int, help="raw ch5 for --ucp")
    ap.add_argument("--outdir", default=os.path.join(HERE, "..", "..", "captures", "nav"))
    ap.add_argument("--tag", default="")
    ap.add_argument("--waypoints", default="", help='"x,y x,y ..." metres; x forward, y right of the start pose')
    ap.add_argument("--hfov", type=float, default=110.0, help="camera horizontal FOV (deg); --calibrate measures it")
    ap.add_argument("--cam-h", type=float, default=0.15, help="camera height above the floor (m); sets the depth scale")
    ap.add_argument("--calibrate", action="store_true", help="360 deg spin first to measure the FOV")
    ap.add_argument("--depth-speed", action="store_true", help="dead-reckon with the depth speed estimate")
    ap.add_argument("--vscale", type=float, default=1.5, help="m/s at full throttle (when not --depth-speed)")
    ap.add_argument("--speed", type=float, default=0.35, help="throttle cap")
    ap.add_argument("--turn-gain", type=float, default=1.0)
    ap.add_argument("--spin", type=float, default=0.3, help="calibration spin yaw stick")
    ap.add_argument("--lookahead", type=float, default=0.8, help="m of free path needed along the chosen bearing")
    ap.add_argument("--reach", type=float, default=0.2)
    ap.add_argument("--settle", type=float, default=2.0)
    ap.add_argument("--max-seconds", type=float, default=120)
    run(ap.parse_args())


if __name__ == "__main__":
    main()
