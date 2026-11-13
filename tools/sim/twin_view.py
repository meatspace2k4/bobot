"""Live twin scene for beni_view: mirrors the real Beni (tools/nav/run_real.py publishes via tools/nav/twin.py).

  sticks     the real robot's channels drive the sim Beni (policy animates the same commands)
  image      the real camera frame on a plane in front of the sim Beni's head camera, with its frustum
  pose       the sim Beni is pulled toward the RGB pose estimate (drift reminder, tau 0.5 s)
  path       the estimated trajectory
  marks      mapped objects as translucent boxes
  points     map points arrive in TwinServer.points (plumbed for mapping, not drawn yet)
Everything here is world geometry (mocap bodies + viewer user scene); the debugger panels are untouched.
"""
import math, os, sys, time

import mujoco
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "nav"))
from twin import TwinServer

NEUTRAL = [1000, 1000, 1000, 2015, 32, 546]
PLANE_D = 0.6          # m in front of the camera
STALE = 0.5            # s without state -> neutral sticks
REMIND_TAU = 0.5       # s, pull of the sim Beni toward the RGB pose estimate
MAX_POINTS = 3000


class TwinSource:
    name = "Real Beni"

    def __init__(self, m, step, keyboard=None, server=None, drive=False):
        self.srv = server or TwinServer()
        self.kb = keyboard            # arrow keys (beni_view.KeyboardSource) for armed takeover
        self.drive = drive            # the drive scene arms on entry; the watch scene stays disarmed
        if drive:
            self.name = "Real Beni drive"
        self.armed = False
        self.next_ctl = 0.0
        self.step = step
        self.cam_body = m.body("twin_cam").mocapid[0]
        self.plane = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "twin_plane")
        self.marks = []
        i = 0
        while mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"twin_mark{i}") >= 0:
            self.marks.append((m.body(f"twin_mark{i}").mocapid[0], mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"twin_mark{i}")))
            i += 1
        self.tex = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TEXTURE, "twin_img")
        self.cam = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "head_cam")
        self.tex_w, self.tex_h = int(m.tex_width[self.tex]), int(m.tex_height[self.tex])
        self.t = 0.0

    # ---- source interface
    def start(self, sim):
        self.t = 0.0
        self.srv.reset()
        self.set_armed(self.drive)
        if self.kb:
            self.kb.start(sim)

    def set_armed(self, on):
        """Armed: the arrow keys drive the real Beni through this sim (and the sim with them)."""
        self.armed = bool(on) and self.kb is not None
        self.srv.send({"type": "ctl", "armed": self.armed, "ch": list(NEUTRAL)})
        return self.armed

    def sticks(self, sim):
        self.t += self.step
        with self.srv.lock:
            st, age = self.srv.state, time.monotonic() - self.srv.state_t
        if self.armed:
            ch = self.kb.sticks(sim)
            if self.t >= self.next_ctl:          # ~30 Hz to the robot; it goes neutral 0.3 s after the last one
                self.next_ctl = self.t + 1 / 30
                self.srv.send({"type": "ctl", "armed": True, "ch": ch})
            if st is not None and age < STALE:
                self.remind(sim.d, st["pose"])
            return ch
        if st is None or age >= STALE:
            return list(NEUTRAL)
        self.remind(sim.d, st["pose"])
        return list(st["ch"])

    def remind(self, d, pose):
        """Pull the sim Beni toward the RGB-estimated real pose (sim drift correction), tau = REMIND_TAU."""
        a = 1 - math.exp(-self.step / REMIND_TAU)
        tx, ty, tyaw = pose[0], -pose[1], -pose[2]          # runner -> sim axes
        d.qpos[0] += a * (tx - d.qpos[0])
        d.qpos[1] += a * (ty - d.qpos[1])
        w, x, y, z = d.qpos[3:7]
        h = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        dth = a * ((tyaw - h + math.pi) % (2 * math.pi) - math.pi)
        qz = np.array([math.cos(dth / 2), 0, 0, math.sin(dth / 2)])
        q = np.zeros(4)
        mujoco.mju_mulQuat(q, qz, d.qpos[3:7].copy())
        d.qpos[3:7] = q

    def done(self):
        return False

    def status(self):
        with self.srv.lock:
            st, age, n = self.srv.state, time.monotonic() - self.srv.state_t, self.srv.clients
        arm = ("ARMED: arrows drive the real Beni  (F6/F7 to watch)  " + (self.kb.status() if self.kb else "")) if self.armed else "watch only  (F6/F7: drive scene)"
        if not n:
            return "waiting for the robot (tools/nav/run_teleop.py)  " + arm
        if st is None or age > STALE:
            return "robot linked, no state"
        x, y, yaw = st["pose"]
        return f"live  est ({x:+.2f},{y:+.2f}) yaw {math.degrees(yaw):+.0f}  {arm}"

    # ---- world geometry
    @staticmethod
    def hide(m, d):
        for i in range(m.nbody):
            nm = m.body(i).name
            if nm.startswith("twin_"):
                d.mocap_pos[m.body(i).mocapid[0]] = (0, 0, -5)

    def draw(self, v, m, d):
        """Call from the viewer loop (main thread)."""
        with self.srv.lock:
            st = self.srv.state
            frame = self.srv.frame if self.srv.frame_new else None
            self.srv.frame_new = False
            marks = list(self.srv.marks)
            path = list(self.srv.path)
        if frame is not None:
            img = np.ascontiguousarray(np.flipud(frame) if False else frame)
            if img.shape[:2] != (self.tex_h, self.tex_w):
                import cv2
                img = cv2.resize(img, (self.tex_w, self.tex_h))
            adr, nc = m.tex_adr[self.tex], m.tex_nchannel[self.tex]
            if nc == 4:
                img = np.concatenate([img, np.full(img.shape[:2] + (1,), 255, np.uint8)], axis=2)
            m.tex_data[adr: adr + img.size] = img.reshape(-1)
            v.update_texture(self.tex)
        with v.lock():
            scn = v.user_scn
            scn.ngeom = 0
            if st is None:
                return
            # the sim Beni is held on the RGB pose estimate (remind), so the estimate frame is the sim frame
            def to_sim(x, y):
                return x, -y

            # image plane on the sim Beni's head camera (follows body and head pan)
            P = d.cam_xpos[self.cam].copy()
            R = d.cam_xmat[self.cam].reshape(3, 3)        # columns: right, up, back
            r, u, f = R[:, 0], R[:, 1], -R[:, 2]
            q = np.zeros(4)
            mujoco.mju_mat2Quat(q, R.flatten())
            w = 2 * PLANE_D * math.tan(math.radians(st["hfov"]) / 2)
            h = w * self.tex_h / self.tex_w
            C = P + PLANE_D * f
            d.mocap_pos[self.cam_body] = C
            d.mocap_quat[self.cam_body] = q
            m.geom_size[self.plane] = (w / 2, h / 2, 0.002)
            for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                self._line(scn, P, C + sx * r * w / 2 + sy * u * h / 2, (1, 1, .3, 1), 2)
            pts = [to_sim(*p) for p in path]
            for i in range(1, len(pts)):
                self._line(scn, (*pts[i - 1], 0.01), (*pts[i], 0.01), (.2, 1, .3, 1), 4)
            for (bid, gid), mk in zip(self.marks, marks):
                hs = max(0.05, mk[2])
                d.mocap_pos[bid] = (*to_sim(mk[0], mk[1]), hs)
                m.geom_size[gid] = (hs, hs, hs)
            for bid, _ in self.marks[len(marks):]:
                d.mocap_pos[bid] = (0, 0, -5)
            # map points arrive in self.srv.points (plumbed for mapping); not drawn yet

    @staticmethod
    def _line(scn, a, b, rgba, width):
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_LINE, np.zeros(3), np.zeros(3), np.eye(3).flatten(),
                            np.array(rgba, np.float32))
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_LINE, width, np.array(a, float), np.array(b, float))
        scn.ngeom += 1
