#!/usr/bin/env python3
"""Beni sim: the bench sim driven by replay scenes from captures, scripted sources and the live twin.

  (none)  normal view: HUD, head-camera inset, history chart, scene tabs, follow camera
  --dev   debugger view: MuJoCo's stock viewer with its panels (for debugging only)

Each capture's stick timeline (topic_trace.py *.sticks.csv) is a collection; each burst of stick
activity is a scene. "Box tour" drives to each box in clockwise order through the same stick channels.
Keys: F6/F7 scene, F8/F9 collection, F10 camera: body follow / head follow / through the head camera,
in the live collection F6/F7 switch watch / drive (drive: arrow keys drive the real Beni through the sim); everything else is MuJoCo's viewer (Tab panels, Space pause, ...).

usage: mjpython beni_view.py --open-beni PATH --traces DIR [--dev]
"""
import argparse, collections, csv, glob, math, os, sys, time

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from beni_sim import Sim, DT, DECIMATION
from twin_view import TwinSource

STEP = DT * DECIMATION
NEUTRAL = [1000, 1000, 1000, 2015, 32, 546]
ACTION_NAMES = {1: "JUMP", 2: "FLIP FORWARD", 4: "FLIP BACK", 6: "FLIP LEFT", 8: "FLIP RIGHT"}
STRIP = 300


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def heading(d):
    w, x, y, z = d.qpos[3:7]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class CaptureSource:
    def __init__(self, name, rows, where=""):
        self.name, self.rows, self.where = name, rows, where
        self.duration = rows[-1][0] - rows[0][0]

    def start(self, sim):
        self.t, self.j = 0., 0

    def sticks(self, sim):
        t0 = self.rows[0][0]
        while self.j + 1 < len(self.rows) and self.rows[self.j + 1][0] - t0 <= self.t:
            self.j += 1
        self.t += STEP
        return self.rows[self.j][1]

    def done(self):
        return self.t >= self.duration

    def status(self):
        return f"{self.where}  {min(self.t, self.duration):5.1f} / {self.duration:.0f} s"


class BoxTour:
    name = "Box tour"
    STICK_MAX = 0.6  # fraction of full stick

    def __init__(self, boxes):
        self.boxes = boxes  # [(x, y, half-size)]

    def start(self, sim):
        p, h = sim.d.qpos[0:2], heading(sim.d)
        # Clockwise sweep from the current heading, measured around the robot.
        key = lambda b: (h - math.atan2(b[1] - p[1], b[0] - p[0])) % (2 * math.pi)
        self.order = sorted(self.boxes, key=key)
        self.i, self.hold, self.t = 0, 0., 0.
        self.dist = 0.

    def target(self):
        return self.order[self.i] if self.i < len(self.order) else None

    def sticks(self, sim):
        self.t += STEP
        tgt = self.target()
        if tgt is None:
            return NEUTRAL
        d = sim.d
        dx, dy = tgt[0] - d.qpos[0], tgt[1] - d.qpos[1]
        self.dist = math.hypot(dx, dy)
        stop = tgt[2] + 0.25
        if self.hold > 0:
            self.hold -= STEP
            if self.hold <= 0:
                self.i += 1
            return NEUTRAL
        if self.dist <= stop:
            self.hold = 1.0
            return NEUTRAL
        err = wrap(math.atan2(dy, dx) - heading(d))
        turn = max(-1., min(1., 1.8 * err)) * self.STICK_MAX
        fwd = 0. if abs(err) > 0.6 else max(0., min(self.STICK_MAX, 0.6 * (self.dist - stop) + 0.1)) * math.cos(err)
        ch = list(NEUTRAL)
        ch[0] = int(round(1000 + fwd * 999))
        ch[1] = int(round(1000 - turn * 999))  # ch1 = 1 is full left
        return ch

    def done(self):
        return self.target() is None

    def status(self):
        if self.done():
            return f"all {len(self.order)} boxes visited  {self.t:.0f} s"
        return f"box {self.i + 1}/{len(self.order)}  {self.dist:.2f} m"


class KeyboardSource:
    """Arrow keys -> left stick: Up/Down throttle (ch0), Left/Right turn (ch1). Keys are read as held
    keys (macOS key state) and only while this window is in front. Values ramp: a tap nudges, a hold
    reaches full stick. Stand-in for the controller; analog input comes later."""
    name = "keyboard"
    RISE, FALL = 2.5, 6.0  # stick units per second
    UP, DOWN, LEFT, RIGHT, SPACE = 126, 125, 123, 124, 49  # macOS virtual key codes

    def __init__(self):
        import Quartz, AppKit
        self._Q, self._ws, self._pid = Quartz, AppKit.NSWorkspace.sharedWorkspace(), os.getpid()
        self.front, self._front_t = False, 0.

    def start(self, sim):
        self.thr = self.yaw = 0.
        self.t = 0.
        self.space_was, self.pulse_until = False, -1.

    def _held(self, code):
        return bool(self._Q.CGEventSourceKeyState(self._Q.kCGEventSourceStateCombinedSessionState, code))

    def _ramp(self, v, target):
        rate = self.RISE if target and (v == 0 or (v > 0) == (target > 0)) else self.FALL
        step = rate * STEP
        return min(target, v + step) if target > v else max(target, v - step)

    def sticks(self, sim):
        self.t += STEP
        if self.t - self._front_t > 0.25:
            self._front_t = self.t
            app = self._ws.frontmostApplication()
            self.front = app is not None and app.processIdentifier() == self._pid
        tt = ty = 0.
        if self.front:
            tt = float(self._held(self.UP)) - float(self._held(self.DOWN))
            ty = float(self._held(self.RIGHT)) - float(self._held(self.LEFT))
        self.thr, self.yaw = self._ramp(self.thr, tt), self._ramp(self.yaw, ty)
        space = self.front and self._held(self.SPACE)
        if space and not self.space_was:  # Space: jump, as the app's ~190 ms ch4 pulse
            self.pulse_until = self.t + 0.19
        self.space_was = space
        ch = list(NEUTRAL)
        if self.t < self.pulse_until:
            ch[4] = 32 | (1 << 6)
        ch[0] = int(round(1000 + self.thr * 999))
        ch[1] = int(round(1000 + self.yaw * 999))  # ch1 1999 = full right
        return ch

    def done(self):
        return False

    def status(self):
        return f"arrows  thr {self.thr:+.2f}  yaw {self.yaw:+.2f}" + ("" if self.front else "  (click the window)")


class PostureSource:
    """Sim-only posture scene: Beni sits (motors fold the legs) or stands (policy), sticks neutral."""

    def __init__(self, sit):
        self.sit = sit
        self.name = "sit" if sit else "stand"

    def start(self, sim):
        self.t = 0.
        self.fresh = not sim.sit and not getattr(sim, "_sat", False)

    def sticks(self, sim):
        self.t += STEP
        # the sit command reaches a standing Beni: after a reset the policy stands it for a second first
        sim.sit = self.sit and (self.t > 1.0 or not self.fresh)
        return list(NEUTRAL)

    def done(self):
        return False

    def status(self):
        return f"{self.name}  {self.t:4.1f} s"


def active(c):
    return (abs(c[0] - 1000) > 30 or abs(c[1] - 1000) > 30 or abs(c[2] - 1000) > 30
            or (c[3] & 0x3F) != 31 or bool((c[4] >> 6) if c[4] & 32 else 0))


def scene_label(rows):
    kinds = []
    if any(abs(c[0] - 1000) > 30 or abs(c[1] - 1000) > 30 for _, c in rows):
        kinds.append("drive")
    if any(abs(c[2] - 1000) > 30 or (c[3] & 0x3F) != 31 for _, c in rows):
        kinds.append("head")
    acts = []
    for _, c in rows:
        code = (c[4] >> 6) if c[4] & 32 else 0
        nm = ACTION_NAMES.get(code, f"code {code}").lower() if code else None
        if nm and nm not in acts:
            acts.append(nm)
    return "+".join(kinds + acts)


def capture_collections(trace_dir, gap=2.0, lead=1.0, tail=2.0):
    """One collection per capture; one scene per burst of stick activity (bursts closer than
    `gap` s merge), padded with `lead` s before and `tail` s after so each move settles."""
    cols = []
    for path in sorted(glob.glob(os.path.join(trace_dir, "*.sticks.csv"))):
        cap = os.path.basename(path).split(".")[0].replace("beni-", "").replace("rvi-20261112-", "rvi ")
        rows = [(float(r["t"]), [int(r[f"ch{i}"]) for i in range(6)]) for r in csv.DictReader(open(path))]
        spans = []
        for t, c in rows:
            if active(c):
                if spans and t - spans[-1][1] < gap:
                    spans[-1][1] = t
                else:
                    spans.append([t, t])
        scenes = []
        for k, (t0, t1) in enumerate(spans):
            sl = [r for r in rows if t0 - lead <= r[0] <= t1 + tail]
            if len(sl) < 2:
                continue
            off = sl[0][0] - rows[0][0]
            name = f"{k + 1} {scene_label(sl)} {sl[-1][0] - sl[0][0]:.0f}s"
            scenes.append(CaptureSource(name, sl, f"{cap} @ {int(off // 60)}:{int(off % 60):02d}"))
        if scenes:
            cols.append((cap, scenes))
    return cols


def find_boxes(m):
    boxes, geoms = [], []
    for g in range(m.ngeom):
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        if nm.startswith("block"):
            boxes.append((m.geom_pos[g][0], m.geom_pos[g][1], m.geom_size[g][0]))
            geoms.append((g, m.geom_rgba[g].copy(), int(m.geom_contype[g]), int(m.geom_conaffinity[g])))
    return boxes, geoms


def show_boxes(m, geoms, on):
    for g, rgba, ct, ca in geoms:
        m.geom_rgba[g] = rgba if on else (0, 0, 0, 0)
        m.geom_contype[g], m.geom_conaffinity[g] = (ct, ca) if on else (0, 0)


# GLFW key codes; F1-F5 and most letters belong to MuJoCo's own viewer.
KEY_F6, KEY_F7, KEY_F8, KEY_F9, KEY_F10, KEY_F12, KEY_X = 295, 296, 297, 298, 299, 301, 88


class Overlay:
    """The normal view's UI layer: HUD, head-camera inset, history chart, scene tabs.
    Uses only the stock viewer's overlay API."""
    INSET_W, INSET_H = 480, 360

    def __init__(self, sim):
        self.sim = sim
        # Offscreen context before the viewer exists: creating it afterwards hangs under mjpython.
        self.inset = mujoco.Renderer(sim.m, self.INSET_H, self.INSET_W)
        self.speed = collections.deque(maxlen=STRIP)
        self.yaw = collections.deque(maxlen=STRIP)
        self.next_ui = 0.
        self.last_action, self.last_action_t = None, -1e9
        f = self.fig = mujoco.MjvFigure()
        f.title = "history  (seconds ago, newest at right)"
        f.flg_legend = 1
        f.gridsize = [6, 5]
        f.range[0] = [-10, 0]
        f.range[1] = [-3, 3]
        f.xformat = "%.0f"
        f.linename[0] = "speed m/s"
        f.linename[1] = "yaw rate rad/s"
        f.linergb[0] = [.35, .85, .45]
        f.linergb[1] = [.95, .65, .25]
        f.linewidth = 2
        f.figurergba = [.05, .06, .08, .85]
        f.panergba = [.10, .12, .15, .85]
        f.legendrgba = [.05, .06, .08, .6]

    def reset(self):
        self.speed.clear(); self.yaw.clear()
        self.last_action = None

    def action(self, code):
        self.last_action, self.last_action_t = ACTION_NAMES.get(code, f"CODE {code}"), time.monotonic()

    def chase(self, v, base):
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        v.cam.trackbodyid = base
        v.cam.distance, v.cam.elevation = 1.1, -18
        v.cam.azimuth = math.degrees(heading(self.sim.d))

    def draw(self, v, cols, col, cur, src, ch, vx, wz, tilt_pitch):
        now = time.monotonic()
        if now < self.next_ui:
            return
        self.next_ui = now + 1 / 30
        sim, d = self.sim, self.sim.d
        hdg = heading(d)
        spd = d.qvel[0] * math.cos(hdg) + d.qvel[1] * math.sin(hdg)
        self.speed.append(spd)
        self.yaw.append(d.qvel[5])
        if tilt_pitch is not None:  # the live twin sets the head camera itself (measured pitch and roll)
            sim.set_tilt(tilt_pitch)
        self.inset.update_scene(d, camera=sim.cam)
        img = self.inset.render().copy()
        act = self.last_action if self.last_action and now - self.last_action_t < 2 else ""
        left = "\n".join([f"{cols[col][0]}  /  {src.name}", "", "throttle  ch0", "yaw       ch1", "head pan  ch2",
                          "head tilt ch3", "action    ch4", "mode      ch5", "", "cmd vx", "cmd wz", "", "speed",
                          "yaw rate", "heading", "height", "head yaw"])
        right = "\n".join(["", "", f"{ch[0]:4d}", f"{ch[1]:4d}", f"{ch[2]:4d}", f"{ch[3] & 0x3F:4d}",
                           f"{ch[4]:4d}  {act}", f"{ch[5]:4d}", "", f"{vx:+.2f} m/s", f"{wz:+.2f} rad/s", "",
                           f"{spd:+.2f} m/s", f"{d.qvel[5]:+.2f} rad/s", f"{math.degrees(hdg):+5.0f} deg",
                           f"{d.qpos[2]:.3f} m", f"{math.degrees(d.qpos[sim.jq['head_yaw_joint']]):+5.0f} deg"])
        tabs_c = "   ".join(f"[{n}]" if i == col else n for i, (n, _) in enumerate(cols))
        scenes = cols[col][1]
        tabs_s = "   ".join(f"[{x.name}]" if i == cur else x.name for i, x in enumerate(scenes))
        v.set_texts([
            (mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT, left, right),
            (mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPRIGHT,
             "F8/F9 collection\nF6/F7 scene", "Tab / Shift+Tab panels"),
            (mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
             f"collections   {tabs_c}\nscenes        {tabs_s}", ""),
            # scene status on its own line, so a long status never widens the HUD table
            (mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOP, src.status(), ""),
        ])
        vp = v.viewport
        if vp is not None and vp.width > self.INSET_W + 24:
            v.set_images([(mujoco.MjrRect(vp.left + vp.width - self.INSET_W - 12, vp.bottom + 60,
                                          self.INSET_W, self.INSET_H), img)])
            f = self.fig
            for li, series in enumerate((self.speed, self.yaw)):
                k = len(series)
                f.linepnt[li] = k
                f.linedata[li][0:2 * k:2] = (np.arange(k) - k + 1) / 30
                f.linedata[li][1:2 * k:2] = np.asarray(series)
            v.set_figures([(mujoco.MjrRect(vp.left + 12, vp.bottom + 60, vp.width // 3, vp.height // 4), f)])


class FollowCamera:
    """Stock-view camera: MuJoCo's tracking camera on Beni, turned with Beni's heading.
    Mouse orbit/zoom are kept as an offset on top of the heading, so they keep following.
    Align (MuJoCo resets to a free camera at the model center) brings back the default follow;
    choosing Free from the camera list keeps a free camera."""

    def __init__(self, v, m, base, d, distance=1.1, elevation=-18., head_q=None, pov_cam=None):
        self.v, self.m, self.base = v, m, base
        self.distance, self.elevation = distance, elevation
        self.head_q = head_q          # qpos index of the head yaw joint
        self.pov_cam = pov_cam        # camera id to look through (head_cam)
        self.head = False             # follow the head (body heading + head yaw) instead of the body
        self.h = self.ref(d)
        self.reset()

    def ref(self, d):
        h = math.degrees(heading(d))
        if self.head and self.head_q is not None:
            h += math.degrees(d.qpos[self.head_q])
        return h

    def toggle_head(self, d):
        """Cycle body follow -> head follow -> through the head camera -> body follow. Switching body/head
        keeps the view angle (the offset absorbs the reference change)."""
        c = self.v.cam
        if c.type == mujoco.mjtCamera.mjCAMERA_FIXED:          # pov -> body follow
            self.head = False
            self.h = self.ref(d)
            self.reset()
            return "body"
        if self.head and self.pov_cam is not None:            # head follow -> pov
            c.type, c.fixedcamid = mujoco.mjtCamera.mjCAMERA_FIXED, self.pov_cam
            return "head camera"
        old = self.h
        self.head = True                                       # body -> head follow
        new = self.ref(d)
        self.h = old + ((new - old + 180) % 360 - 180)
        self.offset -= self.h - old
        return "head"

    def reset(self):
        c = self.v.cam
        c.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        c.trackbodyid = self.base
        c.distance, c.elevation = self.distance, self.elevation
        self.offset = 0.
        c.azimuth = self.set_az = self.h

    def update(self, d):
        c = self.v.cam
        h = self.ref(d)
        self.h += (h - self.h + 180) % 360 - 180  # unwrapped: a spin never flips direction
        if c.type == mujoco.mjtCamera.mjCAMERA_FREE and np.allclose(c.lookat, self.m.stat.center):
            self.reset()  # Align
            return
        if c.type != mujoco.mjtCamera.mjCAMERA_TRACKING:
            return
        self.offset += c.azimuth - self.set_az  # whatever the mouse did since our last frame
        c.azimuth = self.set_az = self.h + self.offset


def twin_scenes(m):
    """Live collection: watch the real Beni, or drive it with the arrow keys (one link, one keyboard)."""
    from twin import TwinServer
    srv, kb = TwinServer(), KeyboardSource()
    return [TwinSource(m, STEP, kb, server=srv), TwinSource(m, STEP, kb, server=srv, drive=True)]


def run(a, ui):
    """ui=True: the normal view (Overlay, panels folded; Tab / Shift+Tab bring them back).
    ui=False: the debugger view, MuJoCo's stock viewer with its panels.
    Run under mjpython on macOS. F6 / F7 previous / next scene, F8 / F9 previous / next collection."""
    from mujoco import viewer as mjviewer
    sim = Sim(a)
    m, d = sim.m, sim.d
    boxes, box_geoms = find_boxes(m)
    cols = [("manual", [KeyboardSource()])] + capture_collections(a.traces) + [("scripted", [BoxTour(boxes)]),
                                                                                ("live", twin_scenes(m)),
                                                                                ("posture", [PostureSource(True), PostureSource(False)])]
    st = dict(col=next((i for i, c in enumerate(cols) if c[0] == a.start), 0), cur=0, pending=True, toggle_head=False, toggle_arm=False)
    overlay = Overlay(sim) if ui else None

    def key(k):
        n = len(cols[st["col"]][1])
        if k == KEY_F6:
            st["cur"] = (st["cur"] - 1) % n
        elif k == KEY_F7:
            st["cur"] = (st["cur"] + 1) % n
        elif k == KEY_X:                 # X: sit / stand (MuJoCo's own X, texture toggle, is cancelled here)
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_TEXTURE] = 1
            names = [c[0] for c in cols]
            if cols[st["col"]][0] != "posture":
                st["col"], st["cur"] = names.index("posture"), 0
            else:
                st["cur"] = 1 - st["cur"]
            st["pending"] = True
            return
        elif k == KEY_F10:
            st["toggle_head"] = True
            return
        elif k == KEY_F12:
            st["toggle_arm"] = True
            return
        elif k in (KEY_F8, KEY_F9):
            st["col"] = (st["col"] + (1 if k == KEY_F9 else -1)) % len(cols)
            st["cur"] = 0
        else:
            return
        st["pending"] = True

    v = mjviewer.launch_passive(m, d, key_callback=key, show_left_ui=not ui, show_right_ui=not ui)
    base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    # Both views: follow camera; the mouse orbit is kept as an offset on top of Beni's heading.
    follow = FollowCamera(v, m, base, d, distance=1.2 if ui else 1.1, elevation=-20. if ui else -18.,
                          head_q=sim.jq.get("head_yaw_joint"), pov_cam=sim.cam)
    if a.camera:
        v.cam.type, v.cam.fixedcamid = mujoco.mjtCamera.mjCAMERA_FIXED, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, a.camera)
    print("F6/F7 scene, F8/F9 collection, F10 camera body/head/head-cam, live scene: F7 drive / F6 watch", flush=True)
    n = lambda x: max(-1., min(1., (x - 1000) / 999))
    last_ch4, src, ch, vx, wz = 32, None, list(NEUTRAL), 0., 0.
    acc, last, label_next = 0., time.monotonic(), 0.
    while v.is_running():
        now = time.monotonic()
        if st["toggle_head"]:
            st["toggle_head"] = False
            with v.lock():
                on = follow.toggle_head(d)
            print(f"camera: {on}", flush=True)
        if st["toggle_arm"]:
            st["toggle_arm"] = False
            if isinstance(src, TwinSource):
                with v.lock():
                    on = src.set_armed(not src.armed)
                print("ARMED: arrow keys drive the real Beni" if on else "disarmed", flush=True)
        acc = min(acc + now - last, 0.1)  # real time; drop backlog beyond 100 ms
        last = now
        if st["pending"]:
            st["pending"] = False
            with v.lock():
                new = cols[st["col"]][1][st["cur"]]
                # sit <-> stand is a posture change of the same robot: no reset, the motors move it
                if not (isinstance(src, PostureSource) and isinstance(new, PostureSource)):
                    sim.reset()
                    sim.sit = False
                src = new
                show_boxes(m, box_geoms, isinstance(src, BoxTour))
                TwinSource.hide(m, d)
                v.user_scn.ngeom = 0
                src.start(sim)
            last_ch4, acc = 32, 0.
            if overlay:
                overlay.reset()   # the camera keeps the user's view
            print(f"{cols[st['col']][0]} / {src.name}", flush=True)
        with v.lock():
            while acc >= STEP:
                acc -= STEP
                if src.done() and isinstance(src, CaptureSource) and st["cur"] + 1 < len(cols[st["col"]][1]):
                    st["cur"] += 1  # scenes in a collection play through; the robot keeps its state
                    src = cols[st["col"]][1][st["cur"]]
                    src.start(sim)
                    print(f"{cols[st['col']][0]} / {src.name}", flush=True)
                ch = src.sticks(sim) if not src.done() else NEUTRAL
                code = (ch[4] >> 6) if ch[4] & 32 else 0
                if ch[4] != last_ch4 and code:
                    print(f"  ch4 {ACTION_NAMES.get(code, code)}", flush=True)
                    if overlay:
                        overlay.action(code)
                    if code == 1:
                        sim.jump()
                last_ch4 = ch[4]
                vx, wz = n(ch[0]) * a.vx_max, -n(ch[1]) * a.wz_max
                sim.step(vx, wz, -n(ch[2]) * a.pan_max)
        if isinstance(src, TwinSource):
            src.draw(v, m, d)
        if overlay:
            with v.lock():
                follow.update(d)
            overlay.draw(v, cols, st["col"], st["cur"], src, ch, vx, wz,
                         None if isinstance(src, TwinSource) else (((ch[3] & 0x3F) - 31) / 31) * a.tilt_max)
        else:
            # Stock view: follow camera; corner label names collection, scene and time.
            with v.lock():
                follow.update(d)
            if now >= label_next:
                label_next = now + 0.25
                v.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT,
                             f"{cols[st['col']][0]}  /  {src.name}", src.status()))
        v.sync()  # one render per loop; the physics above catches up to wall time
        time.sleep(0.001)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--open-beni", required=True)
    ap.add_argument("--policy")
    ap.add_argument("--traces", required=True, help="directory of topic_trace.py *.sticks.csv")
    ap.add_argument("--vx-max", type=float, default=1.5, help="m/s at full stick")
    ap.add_argument("--wz-max", type=float, default=1.8, help="rad/s at full stick")
    ap.add_argument("--pan-max", type=float, default=math.radians(168), help="head yaw rad at ch2 = 1 (left); measured by head-sweep flow")
    ap.add_argument("--tilt-max", type=float, default=math.radians(45))
    ap.add_argument("--dev", action="store_true", help="debugger view: MuJoCo's stock viewer and panels (default: normal view)")
    ap.add_argument("--start", default="manual", help="collection to open on (manual, live, scripted, ...)")
    ap.add_argument("--camera", help="open looking through this model camera (e.g. head_cam)")
    a = ap.parse_args()
    run(a, ui=not a.dev)


if __name__ == "__main__":
    main()
