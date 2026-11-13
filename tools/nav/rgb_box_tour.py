"""Box tour from RGB only: the sim's BoxTour (clockwise order, turn 1.8 * err, forward within 0.6 rad,
0.6 stick cap, stop at half-size + 0.25 m, 1 s hold) with the box positions and the robot pose
taken from the head camera instead of ground truth.

  survey   one turn in place; every frame, obstacle blobs from monocular depth (rgbnav.Perception) become
           detections (bearing, range, width) placed with the flow-yaw pose; repeated detections merge into
           landmarks
  tour     landmarks in clockwise order from the current heading; while the target is in view it is
           re-observed directly (bearing and range), else its mapped position is used; other obstacles are
           avoided with rgbnav.choose
The core is link-agnostic: feed frames to observe() (any thread) and call control(dt) at the control rate.
"""
import math, threading

import cv2
import numpy as np

from beni_head import HeadModel
from rgbnav import BANDS, DW, FLOW_F, R_MAX, STICK_MAX, FlowYaw, Perception, choose

ORB = cv2.ORB_create(800)
BF = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)


def loop_offset(key, g):
    """Median x shift of the current view against the start view (px), or None."""
    kp, des = ORB.detectAndCompute(g, None)
    if des is None or key[1] is None:
        return None
    m = [x for x in BF.match(key[1], des) if x.distance < 40]
    if len(m) < 20:
        return None
    d = np.array([(kp[x.trainIdx].pt[0] - key[0][x.queryIdx].pt[0], kp[x.trainIdx].pt[1] - key[0][x.queryIdx].pt[1]) for x in m])
    mx, my = np.median(d, axis=0)
    ok = (np.abs(d[:, 0] - mx) < 6) & (np.abs(d[:, 1] - my) < 6)
    return float(np.median(d[ok, 0])) if ok.sum() >= 15 else None


def detections(P, per, min_px=80, max_range=2.0):
    """Obstacle blobs -> [(bearing rad + right, range m, half-size m)]."""
    if P["obst"] is None:
        return []
    lr = np.log(np.clip(P["rho"], 1e-3, None)).astype(np.float32)
    gx, gy = cv2.Sobel(lr, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(lr, cv2.CV_32F, 0, 1, ksize=3)
    edges = np.hypot(gx, gy) > 0.35                     # depth discontinuities split objects
    o = (P["obst"] & ~cv2.dilate(edges.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(o.astype(np.uint8), 8)
    out = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < min_px or w < 4:
            continue
        r = P["rho"][lab == i]
        rng = 1.0 / np.percentile(r, 90)
        if rng > max_range or rng < 0.3 or h < 8:
            continue
        a0 = math.atan2(x - DW / 2, per.f)
        a1 = math.atan2(x + w - DW / 2, per.f)
        touches_edge = x <= 1 or x + w >= DW - 1
        out.append(((a0 + a1) / 2, float(rng), float(rng * math.tan((a1 - a0) / 2)), touches_edge))
    return out


class RgbBoxTour:
    def __init__(self, hfov, cam_h, vmax, spin=0.35, merge=0.45, turn_max=STICK_MAX):
        self.per = Perception(hfov, cam_h)
        self.vmax, self.spin, self.merge, self.turn_max = vmax, spin, merge, turn_max
        self.sweep_s = 24.0
        self.sitting = False        # Beni disabled (sitting): never drive, scan only
        self.sweep_amp = 0.3        # head rate stick during the 360 sweep
        self.lock = threading.Lock()
        self.reset()

    def reset(self):
        with self.lock:
            self.flow = FlowYaw()
            self.x = self.y = self.yaw = 0.0
            self.P = None
            self.seen = []            # current frame detections
            self.raw = []             # survey detections: (raw turned, bearing, range, half)
            self.key = None           # ORB of the survey start view
            self.closed = False
            self.marks = []           # [x, y, half, count]
            self.phase, self.turned = "settle", 0.0
            self.head, self.t_survey = 0.0, 0.0
            self.t_phase, self.burst_t0, self.burst_y0, self.bursts = 0.0, 0.0, 0.0, []
            self.stall_t, self.abort = 0.0, ""
            self._k, self._cmd = -1, 0.0
            self.t = 0.0
            self.order, self.i, self.hold = [], 0, 0.0
            self.dist = 0.0
            self.reached = []
            self.best_d, self.best_t = None, 0.0

    # ---- perception side (call from a worker thread)
    def observe(self, rgb):
        dyaw = self.flow(rgb, FLOW_F)
        P = self.per(rgb)
        g = cv2.cvtColor(cv2.resize(rgb, (DW, 238)), cv2.COLOR_RGB2GRAY)
        with self.lock:
            self.yaw += dyaw
            self.P = P
            self.seen = detections(P, self.per)
            if self.phase == "wiggle":
                self.turned += dyaw
                return
            if self.phase == "survey":
                # body still, head sweeping: flow yaw is the camera's pan relative to the body
                self.turned += dyaw
                for bb, r, hs, edge in self.seen:
                    if not edge:
                        self.raw.append((self.turned, bb, r, hs))
                return
            if self.phase == "tour":
                for b, r, hs, edge in self.seen:
                    if not edge:
                        self._merge(self.yaw + b, r, hs, False)

    def _merge(self, a, r, hs, new_ok):
        px, py = self.x + (r + hs) * math.cos(a), self.y + (r + hs) * math.sin(a)
        for m in self.marks:
            if math.hypot(m[0] - px, m[1] - py) < self.merge:
                k = m[3]
                m[0], m[1] = (m[0] * k + px) / (k + 1), (m[1] * k + py) / (k + 1)
                m[2] = max(m[2], hs)
                m[3] = min(k + 1, 20)
                return
        if new_ok:
            self.marks.append([px, py, hs, 1])

    # ---- control side (call at the control rate) -> (throttle, yaw stick), + right
    def control(self, dt):
        with self.lock:
            self.t += dt
            thr = yc = 0.0
            if self.phase == "settle":
                if self.t > 1.5 and self.P is not None:
                    self.phase, self.t_phase, self.head = "center", self.t, 0.0
            elif self.phase == "center":
                # head to centre and let it settle before anything moves it
                self.head = 0.0
                if self.t - self.t_phase > 1.5:
                    self.phase, self.turned, self.t_survey, self.leg = "survey", 0.0, self.t, 0
                    self.headm = HeadModel()     # head centred by the posture change
            elif self.phase == "survey":
                # head sweep from centre, body held still: left to -180 deg, right to +180 deg, back to 0 (a full
                # 360, then return). Each leg ends on the head model's angle (the same model the sim runs)
                legs = ((-1, -180.0), (1, 180.0), (-1, 0.0))
                pan = -math.degrees(self.headm.q)     # model + = left; here + = right
                u = 0.0
                if self.leg < len(legs):
                    sgn, goal = legs[self.leg]
                    if (pan - goal) * sgn >= 0:
                        self.leg += 1
                if self.leg < len(legs):
                    self.head = legs[self.leg][0] * self.sweep_amp
                else:
                    self.head, u = 0.0, 1.0
                self.headm.step(-self.head, dt)
                if u >= 1.0:
                    self.head = 0.0
                    self.marks = []
                    for t, bb, r, hs in self.raw:
                        self._merge(t + bb, r, hs, True)
                    self.yaw = 0.0          # head centred again; the body did not turn
                    good = sorted([m for m in self.marks if m[3] >= 8 and math.hypot(m[0], m[1]) < 2.5], key=lambda m: -m[3])[:6]
                    key = lambda m: (math.atan2(m[1] - self.y, m[0] - self.x) - self.yaw) % (2 * math.pi)
                    self.order = sorted(good, key=key)
                    self.phase, self.i = ("done" if self.sitting else "tour"), 0
            elif self.phase == "tour":
                if self.i >= len(self.order):
                    self.phase = "done"
                elif self.hold > 0:
                    self.hold -= dt
                    if self.hold <= 0:
                        self.i += 1
                else:
                    thr, yc = self._drive()
            v = thr * self.vmax
            self.x += v * dt * math.cos(self.yaw)
            self.y += v * dt * math.sin(self.yaw)
            return thr, yc

    def _drive(self):
        m = self.order[self.i]
        # re-observe the target if a detection lines up with its mapped position
        best = None
        for b, r, hs, edge in self.seen:
            a = self.yaw + b
            px, py = self.x + (r + hs) * math.cos(a), self.y + (r + hs) * math.sin(a)
            dd = math.hypot(px - m[0], py - m[1])
            if dd < 0.6 and (best is None or dd < best[0]):
                best = (dd, b, r, hs)
        face = None
        if best is not None:
            _, err, r, hs = best
            face = r
            self.dist = r + hs          # to the box centre
            m[2] = 0.7 * m[2] + 0.3 * hs
            a = self.yaw + err          # pull the map toward the observation (corrects drift)
            m[0] += 0.3 * (self.x + self.dist * math.cos(a) - m[0])
            m[1] += 0.3 * (self.y + self.dist * math.sin(a) - m[1])
        else:
            dx, dy = m[0] - self.x, m[1] - self.y
            self.dist = math.hypot(dx, dy)
            err = (math.atan2(dy, dx) - self.yaw + math.pi) % (2 * math.pi) - math.pi
        # arrival: near face within 0.5 m (Beni's own braking holds ~0.4 m), or no progress for 2 s
        prog = face if face is not None else self.dist
        if self.best_d is None or prog < self.best_d - 0.05:
            self.best_d, self.best_t = prog, self.t
        stalled = self.t - self.best_t > 2.0
        stop = m[2] + 0.5
        if (face is not None and face <= 0.5) or self.dist <= stop or stalled:
            self.reached.append((self.i, round(self.x, 2), round(self.y, 2), "stall" if stalled else "near"))
            print(f"box {self.i + 1}/{len(self.order)} reached ({'stall' if stalled else f'face {prog:.2f} m'})", flush=True)
            self.hold, self.best_d = 1.0, None
            return 0.0, 0.0
        P = self.P
        half = math.atan2(DW / 2, self.per.f) * 0.85
        if P is not None and abs(err) < half and self.dist - stop > 0.3:
            # avoid everything except the target itself: only need the path up to the stop distance
            ch = choose(P["free"], self.per, err, min(self.dist - stop, 0.8))
            if ch is not None:
                err = ch
        turn = max(-1., min(1., 1.8 * err)) * self.turn_max
        fwd = 0. if abs(err) > 0.6 else max(0., min(STICK_MAX, 0.6 * (self.dist - stop) + 0.1)) * math.cos(err)
        return fwd, turn

    def status(self):
        with self.lock:
            s = f"{self.phase}  pose ({self.x:+.2f},{self.y:+.2f}) yaw {math.degrees(self.yaw):+.0f}"
            if self.abort:
                s += "  " + self.abort
            if self.phase == "survey":
                s += f"  head {self.head:+.2f}  camera pan {math.degrees(self.turned):+.0f}  detections {len(self.raw)}"
            elif self.phase in ("tour", "done"):
                s += f"  box {min(self.i + 1, len(self.order))}/{len(self.order)}  dist {self.dist:.2f}"
            return s
