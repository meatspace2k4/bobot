"""RGB-only navigation core for Beni (shared by the sim scene and the real-robot runner).

  depth      Depth Anything V2 Small, ONNX (third_party/models/depth_anything_v2_small_308x238.onnx)
  metric     floor disparity is linear in image row, d = k (v - v0); a floor point at row v lies at
             Z = h f / (v - v0) for camera height h, so 1/Z = d / (k h f) for every pixel
  obstacles  pixels standing above the floor line; per bearing the nearest one is the free range
  heading    LK flow, x-shift fitted as a + b (x - cx): a is rotation, b absorbs forward expansion
  steering   the sim box tour's law (turn 1.8 * err, forward only within 0.6 rad, 0.6 stick cap) toward the
             free bearing nearest the waypoint bearing
"""
import math, os

import cv2
import numpy as np

DW, DH = 308, 238
BANDS = 48
R_MAX = 3.0
STICK_MAX = 0.6
MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "third_party", "models",
                     "depth_anything_v2_small_308x238.onnx")


class Depth:
    def __init__(self, path=MODEL):
        import onnxruntime as ort
        ort.set_default_logger_severity(3)
        self.s = ort.InferenceSession(path, providers=["CoreMLExecutionProvider", "CPUExecutionProvider"])
        self.mean = np.array([0.485, 0.456, 0.406], np.float32)
        self.std = np.array([0.229, 0.224, 0.225], np.float32)

    def __call__(self, rgb):
        x = cv2.resize(rgb, (DW, DH), interpolation=cv2.INTER_AREA).astype(np.float32) / 255
        x = ((x - self.mean) / self.std).transpose(2, 0, 1)[None]
        return self.s.run(None, {"image": np.ascontiguousarray(x)})[0][0]


class Floor:
    def __init__(self):
        self.k = self.v0 = None
        self.roll = 0.0       # camera roll (rad, + = right side down), from the floor's horizon slope

    def fit(self, d):
        r0, r1 = int(DH * 0.72), int(DH * 0.97)
        c0, c1 = int(DW * 0.3), int(DW * 0.7)
        rows = np.arange(r0, r1)
        med = np.median(d[r0:r1, c0:c1], axis=1)
        A = np.vstack([rows, np.ones_like(rows)]).T.astype(np.float64)
        (a, b), *_ = np.linalg.lstsq(A, med, rcond=None)
        if a <= 0:
            return False
        v0 = -b / a
        resid = np.abs(A @ [a, b] - med).mean() / max(1e-6, med.mean())
        if not (-0.3 * DH < v0 < 1.1 * DH) or resid > 0.08:   # about +-40 deg of pitch
            return False
        # horizon slope: fit d = a v + b u + c on the patch; the horizon d = 0 has dv/du = -b / a
        vv, uu = np.mgrid[r0:r1:3, c0:c1:3]
        A3 = np.stack([vv.ravel(), uu.ravel(), np.ones(vv.size)], 1).astype(np.float64)
        (a3, b3, _), *_ = np.linalg.lstsq(A3, d[r0:r1:3, c0:c1:3].ravel().astype(np.float64), rcond=None)
        roll = math.atan2(b3, a3) if a3 > 0 else 0.0
        if self.k is None:
            self.k, self.v0, self.roll = a, v0, roll
        else:
            self.k, self.v0 = 0.8 * self.k + 0.2 * a, 0.8 * self.v0 + 0.2 * v0
            self.roll = 0.8 * self.roll + 0.2 * roll
        return True


class Perception:
    def __init__(self, hfov_deg, cam_h, depth=None):
        self.depth = depth or Depth()
        self.floor = Floor()
        self.h = cam_h
        self.pitch = 0.0      # camera pitch (rad, + up), measured from the floor's horizon row
        self.roll = 0.0       # camera roll (rad, + right side down), from the horizon slope
        self.set_hfov(hfov_deg)

    def set_hfov(self, hfov_deg):
        self.hfov = hfov_deg
        self.f = (DW / 2) / math.tan(math.radians(hfov_deg) / 2)

    def bearing(self, b):
        return math.atan2((b + 0.5) * DW / BANDS - DW / 2, self.f)

    def __call__(self, rgb):
        d = self.depth(rgb)
        ok = self.floor.fit(d)
        out = {"disp": d, "floor_ok": ok, "rho": None, "free": np.full(BANDS, R_MAX), "obst": None}
        if self.floor.k is None:
            return out
        k, v0 = self.floor.k, self.floor.v0
        self.pitch = math.atan2(v0 - DH / 2, self.f)     # horizon above centre -> looking down
        self.roll = self.floor.roll
        rho = np.clip(d, 0, None) / (k * self.h * self.f)
        rows = np.arange(DH)[:, None].astype(np.float32)
        floor_d = k * (rows - v0)
        obst = (d > np.maximum(floor_d * 1.15, 0) + 0.04 * k * DH) & (rho > 1 / R_MAX)
        obst[: max(0, int(v0) - 4), :] = False      # above the horizon = above the camera: walls, ceiling
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


class FlowYaw:
    """Yaw change between consecutive frames from LK flow (rad, + = turned right)."""

    def __init__(self):
        self.prev = None
        self.n = 0

    def __call__(self, rgb, f):
        g = cv2.cvtColor(cv2.resize(rgb, (DW, DH), interpolation=cv2.INTER_AREA), cv2.COLOR_RGB2GRAY)
        if self.prev is None:
            self.prev = g
            return 0.0
        p0 = cv2.goodFeaturesToTrack(self.prev, 300, 0.01, 5)
        self.prev, prev = g, self.prev
        self.n = 0
        if p0 is None or len(p0) < 10:
            return 0.0
        p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, g, p0, None, winSize=(21, 21), maxLevel=3)
        ok = st.ravel() == 1
        a, b = p0[ok].reshape(-1, 2), p1[ok].reshape(-1, 2)
        if len(a) < 10:
            return 0.0
        x, dx = a[:, 0] - DW / 2, (b - a)[:, 0]
        A = np.vstack([np.ones_like(x), x]).T
        keep = np.ones(len(x), bool)
        for _ in range(3):  # robust fit dx = s + e x
            (s, e), *_ = np.linalg.lstsq(A[keep], dx[keep], rcond=None)
            r = np.abs(A @ [s, e] - dx)
            keep = r < max(1.0, 2.5 * np.median(r[keep]))
        self.n = int(keep.sum())
        return float(-math.atan2(s, f)) if self.n >= 8 else 0.0


def choose(free, per, target, need, half_width=0.17):
    """Free bearing nearest the target: every band whose ray passes within half_width of the path,
    out to `need` metres, must be free that far."""
    best = None
    for b in range(BANDS):
        a = per.bearing(b)
        ok = True
        for c in range(BANDS):
            dc = abs(per.bearing(c) - a)
            if dc > 1.2:
                continue
            reach = need if dc < 1e-3 else (min(need, half_width / math.sin(dc)) if dc < math.pi / 2 else 0)
            if free[c] < reach * math.cos(dc) - 0.02:
                ok = False
                break
        if ok and (best is None or abs(a - target) < abs(best - target)):
            best = a
    return best


class Navigator:
    """Waypoint follower on the estimated pose (x forward, y right of the start; yaw + right)."""

    def __init__(self, waypoints, reach=0.25, lookahead=0.8, speed=STICK_MAX):
        self.wps, self.reach, self.lookahead, self.speed = list(waypoints), reach, lookahead, speed
        self.wi, self.hold = 0, 0.0
        self.target = self.chosen = None
        self.dist = 0.0
        self.events = []

    def done(self):
        return self.wi >= len(self.wps)

    def step(self, pose, P, per, dt):
        """-> (throttle, yaw_stick) in -1..1."""
        self.target = self.chosen = None
        if self.hold > 0:
            self.hold -= dt
            if self.hold <= 0:
                self.wi += 1
            return 0.0, 0.0
        if self.done():
            return 0.0, 0.0
        x, y, yaw = pose
        wx, wy = self.wps[self.wi]
        dx, dy = wx - x, wy - y
        self.dist = math.hypot(dx, dy)
        if self.dist <= self.reach:
            self.events.append((self.wi, (x, y)))
            self.hold = 1.0
            return 0.0, 0.0
        self.target = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
        ahead = float(np.min(P["free"][BANDS // 2 - 3: BANDS // 2 + 3])) if P else R_MAX
        fwd = 0.0
        if P is None or abs(self.target) > math.atan2(DW / 2, per.f) * 0.85:
            err = self.target                      # outside the camera's view: turn toward it first
        else:
            self.chosen = choose(P["free"], per, self.target, min(self.dist, self.lookahead))
            if self.chosen is None:
                left, right = P["free"][: BANDS // 2].mean(), P["free"][BANDS // 2:].mean()
                err = 0.6 if right >= left else -0.6
            else:
                err = self.chosen
                if abs(err) <= 0.6:
                    fwd = max(0., min(self.speed, 0.6 * self.dist + 0.1)) * math.cos(err)
                    fwd *= max(0.3, min(1.0, (ahead - 0.25) / 0.6))
        turn = max(-1., min(1., 1.8 * err)) * STICK_MAX
        return fwd, turn
