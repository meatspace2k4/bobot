#!/usr/bin/env python3
"""MuJoCo bench Beni: the open-beni trained policy behind a UCP endpoint.

Speaks the same UCP as the robot (ucp:sync/ack, 31 B stick frames in, H.265 video
fragments out), so beni_bridge.py --robot 127.0.0.1:19007 drives it unchanged.

Uses an open-beni checkout (MIT, github.com/Mondo-Robotics/open-beni) at runtime for
the model and policy. Nothing from it is vendored here.

Channel -> sim mapping (scales are bench parameters, NOT measured robot units):
  ch0 -> forward velocity command   (ch0-1000)/999 * --vx-max
  ch1 -> yaw-rate command           -(ch1-1000)/999 * --wz-max   (ch1=1 is left)
  ch2 -> head yaw joint target     -(ch2-1000)/999 * --pan-max (ch2 low = left) (overrides the policy's head action)
  ch3 low 6 bits -> camera pitch    (tilt-31)/31 * --tilt-max  (render only; no tilt joint in the model)
  ch4 code 1 -> jump; flip codes are logged only (the policy has no flips)

usage: beni_sim.py --open-beni ~/src/open-beni [--port 19007] [--no-video] [--view]
needs: pip install mujoco onnxruntime numpy; ffmpeg on PATH for video
"""
import argparse, math, os, socket, struct, subprocess, sys, threading, time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import onnxruntime as ort

ACTION_JOINTS = ["L1", "R1", "L2", "R2", "LW", "RW", "head_yaw_joint"]
OBS_JOINTS = ["L1", "R1", "head_yaw_joint", "L2", "R2", "LW", "RW"]
DEFAULT_Q = {"L1": 0.3, "R1": 0.3, "L2": 0.5, "R2": 0.5, "LW": 0.0, "RW": 0.0,
             "head_yaw_joint": 0.0}
# Sitting (Beni's disabled state): legs swung all the way back and folded, wheels up, no policy.
# Joint ranges from open-beni assets/beni_base/mjcf/beni.xml: L1/R1 -2.5307..6.8940, L2/R2 -0.7854..1.1345.
SIT_Q = {"L1": -2.5307 + 0.05, "R1": -2.5307 + 0.05, "L2": 1.1345 - 0.03, "R2": 1.1345 - 0.03, "LW": 0.0, "RW": 0.0}
SCALE = {"L1": .25, "R1": .25, "L2": .25, "R2": .25, "LW": 20.0, "RW": 20.0, "head_yaw_joint": .5}
# open-beni python/beni_rl/beni_robot.py actuator config
MOTOR = {
    "L1": dict(kp=.5, kd=.02, lim=1.375, arm=.0001625, fl=.02),
    "R1": dict(kp=.5, kd=.02, lim=1.375, arm=.0001625, fl=.02),
    "L2": dict(kp=.5, kd=.02, lim=3.6, sim_lim=4.6, spring=1.0, arm=.0005375, fl=.02),
    "R2": dict(kp=.5, kd=.02, lim=3.6, sim_lim=4.6, spring=1.0, arm=.0005375, fl=.02),
    "LW": dict(kp=0., kd=.004, lim=.6, arm=0., fl=.005),
    "RW": dict(kp=0., kd=.004, lim=.6, arm=0., fl=.005),
    "head_yaw_joint": dict(kp=3., kd=.05, lim=.1, arm=0., fl=0.),
}
DT, DECIMATION, DELAY_SUBSTEPS, KNEE_DAMPING = .002, 5, 2, 1.2
JUMP_FRAMES, JUMP_HEIGHT = 41, .18
OFF_W, OFF_H = 1920, 1440
KEEP_HEAD_ROLL = False
ACTIONS = {1: "jump", 2: "flip_forward", 4: "flip_back", 6: "flip_left", 8: "flip_right"}


TWIN_W, TWIN_H, TWIN_MARKS = 320, 240, 16


def build_xml(root_dir: Path) -> str:
    src = root_dir / "assets/beni_base"
    root = ET.parse(src / "mjcf/beni.xml").getroot()
    root.find("compiler").attrib.update(meshdir=str(src / "meshes"), texturedir=str(src / "textures"), autolimits="true")
    ET.SubElement(root, "option", timestep=str(DT), gravity="0 0 -9.81", integrator="implicitfast",
                  iterations="10", ls_iterations="20", ccd_iterations="50", cone="pyramidal")
    old = root.find("actuator")
    if old is not None:
        root.remove(old)
    act = ET.SubElement(root, "actuator")
    for j in root.iter("joint"):
        n = j.get("name")
        if n not in MOTOR:
            continue
        c = MOTOR[n]
        j.attrib.update(ref="0", actuatorfrclimited="false", armature=str(c["arm"]), frictionloss=str(c["fl"]), damping="0")
        j.attrib.pop("actuatorfrcrange", None)
        lim = c.get("sim_lim", c["lim"])
        ET.SubElement(act, "motor", name=n, joint=n, gear="1", ctrllimited="true", ctrlrange=f"{-lim} {lim}")
    for n in ("LW", "RW"):
        for g in root.find(f'.//body[@name="{n}_Link"]').findall("geom"):
            if g.get("contype", "1") != "0" or g.get("conaffinity", "1") != "0":
                g.set("name", n + "_collision")
                g.set("priority", "1")
    vis = root.find("visual")
    if vis is None:
        vis = ET.SubElement(root, "visual")
    ET.SubElement(vis, "global", offwidth=str(OFF_W), offheight=str(OFF_H))
    head = root.find('.//body[@name="head"]')
    # The real head has no roll freedom; the model's ±0.05 rad roll joint rocks it loose.
    if not KEEP_HEAD_ROLL:
        head.remove(head.find('joint[@name="head_roll_joint"]'))
    # fovy 64.4 = 80 deg horizontal at 4:3, the real camera (measured 2026-11-13)
    ET.SubElement(head, "camera", name="head_cam", pos="0.06 0 0.05", xyaxes="0 -1 0 0 0 1", fovy="64.4")
    # Chase camera rig: a mocap body that Sim moves to Beni's position and heading every step
    # (yaw only, ground level), with a camera behind and above looking forward. Shows up in
    # MuJoCo's camera list as "chase", next to "head_cam".
    rig = ET.SubElement(root.find("worldbody"), "body", name="chase_rig", mocap="true")
    ET.SubElement(rig, "camera", name="chase", pos="-1.0 0 0.45", xyaxes="0 -1 0 0.259 0 0.966", fovy="60")
    asset = root.find("asset")
    ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker", rgb1=".35 .37 .4", rgb2=".25 .27 .3", width="512", height="512")
    ET.SubElement(asset, "material", name="grid", texture="grid", texrepeat="40 40", reflectance=".1")
    wb = root.find("worldbody")
    wb.insert(0, ET.Element("geom", name="floor", type="plane", size="20 20 .1", material="grid"))
    wb.insert(0, ET.Element("light", pos="0 0 4", dir="0 0 -1", directional="true"))
    for i, (x, y, s) in enumerate([(3, 1.5, .1), (4, -2, .15), (-2, 2, .1), (-3, -1.5, .12), (1, 4, .2)]):
        wb.insert(0, ET.Element("geom", name=f"block{i}", type="box", pos=f"{x} {y} {s}", size=f"{s} {s} {s}", rgba=".8 .4 .2 1"))
    # Twin layer (tools/nav/twin.py -> beni_view live scene): the real camera image on a plane at the
    # estimated camera pose, and mapped objects as boxes. Mocap, no collision, parked below the floor.
    ET.SubElement(asset, "texture", name="twin_img", type="2d", builtin="flat", rgb1=".2 .2 .2",
                  width=str(TWIN_W), height=str(TWIN_H))
    ET.SubElement(asset, "material", name="twin_img", texture="twin_img", emission="1", rgba="1 1 1 1")
    cam = ET.SubElement(wb, "body", name="twin_cam", mocap="true", pos="0 0 -5")
    ET.SubElement(cam, "geom", name="twin_plane", type="box", size=".2 .15 .002", material="twin_img", group="3",
                  contype="0", conaffinity="0")
    for i in range(TWIN_MARKS):
        mk = ET.SubElement(wb, "body", name=f"twin_mark{i}", mocap="true", pos="0 0 -5")
        ET.SubElement(mk, "geom", name=f"twin_mark{i}", type="box", size=".1 .1 .1", rgba=".2 .8 1 .45",
                      contype="0", conaffinity="0")
    # The model's head collision box overlaps the upper legs whenever L1 < ~0 (sitting); the real head never
    # touches the legs, so those pairs are excluded (the head turns freely while sitting).
    contact = root.find("contact") if root.find("contact") is not None else ET.SubElement(root, "contact")
    for leg in ("L1_Link", "L2_Link", "LW_Link", "R1_Link", "R2_Link", "RW_Link"):
        ET.SubElement(contact, "exclude", body1="head", body2=leg)
    from viewer_ui import dress_scene
    dress_scene(root)
    return ET.tostring(root, encoding="unicode")


def spring(theta):
    a = theta - 1.0
    return .22 * math.sin(a) + .15 * math.sin(2 * a)


def torque(name, q, qd, action, damp=1.0):
    c = MOTOR[name]
    sc = SCALE.get(name, 0.)
    vel_target = action * sc if name in ("LW", "RW") else 0.
    raw = c["kp"] * (DEFAULT_Q[name] + action * sc - q) + c["kd"] * damp * (vel_target - qd)
    t = max(-c["lim"], min(c["lim"], raw))
    if c.get("spring"):
        t += spring(q) * c["spring"]
    tl = c.get("sim_lim", c["lim"])
    return max(-tl, min(tl, t))


class Channels:
    def __init__(self):
        self.lock = threading.Lock()
        self.ch = [1000, 1000, 1000, 2015, 32, 560]
        self.peer = None
        self.last_rx = 0.


class Sim:
    def __init__(self, a):
        self.a = a
        self.m = mujoco.MjModel.from_xml_string(build_xml(Path(a.open_beni).expanduser()))
        self.d = mujoco.MjData(self.m)
        self.jq, self.jv, self.ja = {}, {}, {}
        for n in DEFAULT_Q:
            jid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)
            self.jq[n], self.jv[n] = self.m.jnt_qposadr[jid], self.m.jnt_dofadr[jid]
            self.ja[n] = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
        self.sens = {n: self.m.sensor_adr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SENSOR, n)]
                     for n in ("imu_gyro", "imu_quat", "head_imu_gyro")}
        self.cam = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_CAMERA, "head_cam")
        self.chase_cam = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_CAMERA, "chase")
        self.rig = self.m.body_mocapid[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "chase_rig")]
        pol = Path(a.policy).expanduser() if a.policy else Path(a.open_beni).expanduser() / "assets/models/policy.onnx"
        self.head_override = True
        self.sit = False          # True: Beni's sitting / disabled state (no policy, legs folded)
        self._sat = False
        self.pol = ort.InferenceSession(str(pol), providers=["CPUExecutionProvider"])
        self.reset()

    def reset(self):
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        d.qpos[0:3] = (0, 0, .14)
        d.qpos[3:7] = (1, 0, 0, 0)
        for n, v in DEFAULT_Q.items():
            d.qpos[self.jq[n]] = v
        self.h = np.zeros((1, 1, 512), np.float32)
        self.last = np.zeros(7, np.float32)
        self.delay = [np.zeros(7, np.float32) for _ in range(DELAY_SUBSTEPS)]
        self.vx = self.wz = 0.
        self.jump_frame, self.jump_hold, self.jump_age = -1, None, 0
        self.update_rig()
        mujoco.mj_forward(m, d)

    def obs(self, gimbal_yaw):
        d, s = self.d, self.d.sensordata
        o = np.zeros(40, np.float32)
        o[0:3] = s[self.sens["imu_gyro"]:self.sens["imu_gyro"] + 3]
        w, x, y, z = s[self.sens["imu_quat"]:self.sens["imu_quat"] + 4]
        o[3:6] = (2 * (w * y - x * z), -2 * (y * z + w * x), -(1 - 2 * (x * x + y * y)))
        o[6:9] = (self.vx, 0., self.wz)
        for i, n in enumerate(OBS_JOINTS):
            if i < 5:
                o[9 + i] = d.qpos[self.jq[n]] - DEFAULT_Q[n]
            o[14 + i] = d.qvel[self.jv[n]]
        o[21:28] = self.last
        o[28:31] = (0., 0., .11)
        o[31:33] = (gimbal_yaw, 1.)
        o[33:36] = s[self.sens["head_imu_gyro"]:self.sens["head_imu_gyro"] + 3]
        if self.jump_frame >= 0:
            o[36:40] = (1., self.jump_frame / 40, JUMP_HEIGHT, 0.)
        return o

    def jump(self):
        self.jump_frame, self.jump_age = 0, 0
        self.jump_hold = self.vx

    def step(self, vx_cmd, wz_cmd, gimbal_yaw):
        d = self.d
        if self.jump_hold is not None:
            vx_cmd, wz_cmd = self.jump_hold, 0.
        self.vx += max(-.03, min(.03, vx_cmd - self.vx))
        self.wz += max(-.06, min(.06, wz_cmd - self.wz))
        if self.sit and not self._sat:
            self.enter_sit()
        elif not self.sit and self._sat:
            self.leave_sit()
        if self.sit:
            # disabled: no policy; joints held at the folded pose (action = offset from the default / scale)
            act = np.array([(SIT_Q[n] - DEFAULT_Q[n]) / SCALE[n] if n in SIT_Q else 0. for n in ACTION_JOINTS], np.float32)
        else:
            act, self.h = self.pol.run(None, {"obs": self.obs(gimbal_yaw)[None], "h_in": self.h})
            act = act[0].astype(np.float32)
        # The public policy has no reward on its gimbal input; head yaw follows ch2 directly.
        if self.head_override:
            act[6] = gimbal_yaw / SCALE["head_yaw_joint"]
        for _ in range(DECIMATION):
            self.delay.append(act)
            a = self.delay.pop(0)
            for i, n in enumerate(ACTION_JOINTS):
                if self.sit and n in SIT_Q:
                    # sitting is parked: legs held folded at the limits
                    d.qpos[self.jq[n]] = SIT_Q[n]
                    d.qvel[self.jv[n]] = 0.
                    d.ctrl[self.ja[n]] = 0.
                    continue
                damp = KNEE_DAMPING if n in ("L2", "R2") else 1.
                d.ctrl[self.ja[n]] = torque(n, d.qpos[self.jq[n]], d.qvel[self.jv[n]], a[i], damp)
            if "head_roll_joint" in self.ja:
                hr = "head_roll_joint"
                d.ctrl[self.ja[hr]] = torque(hr, d.qpos[self.jq[hr]], d.qvel[self.jv[hr]], 0.)
            mujoco.mj_step(self.m, d)
            if self.sit:
                self._level(d)
        self.last = act
        self.update_rig()
        mujoco.mj_forward(self.m, d)
        if self.jump_frame >= 0:
            self.jump_frame += 1
            if self.jump_frame >= JUMP_FRAMES:
                self.jump_frame = -1
        if self.jump_hold is not None:
            self.jump_age += 1
            if self.jump_age >= 120:
                self.jump_hold = None
        if not np.all(np.isfinite(d.qpos)) or d.qpos[2] < -1:
            print("sim diverged; reset", flush=True)
            self.reset()

    @staticmethod
    def _level(d):
        """Sitting Beni rests perfectly level: keep heading, zero pitch/roll and their rates (height is free)."""
        w, x, y, z = d.qpos[3:7]
        h = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        d.qpos[3:7] = (math.cos(h / 2), 0., 0., math.sin(h / 2))
        d.qvel[3:5] = 0.
        d.qvel[0:2] = 0.

    def enter_sit(self):
        """Straight into the parked sitting pose: body level on its base, legs folded at the limits, wheels up;
        position and heading kept. (Folding it dynamically tips the body over.)"""
        d = self.d
        self._level(d)
        d.qpos[2] = 0.05
        d.qvel[:] = 0.
        for n, v in SIT_Q.items():
            d.qpos[self.jq[n]] = v
        mujoco.mj_forward(self.m, d)
        self._sat = True

    def leave_sit(self):
        """Back to standing at the same position and heading (the policy takes over from the default pose)."""
        d = self.d
        x, y = d.qpos[0], d.qpos[1]
        w, qx, qy, qz = d.qpos[3:7]
        h = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        self.reset()
        d.qpos[0], d.qpos[1] = x, y
        d.qpos[3:7] = (math.cos(h / 2), 0, 0, math.sin(h / 2))
        mujoco.mj_forward(self.m, d)
        self._sat = False

    def update_rig(self):
        d = self.d
        w, x, y, z = d.qpos[3:7]
        h = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        d.mocap_pos[self.rig] = (d.qpos[0], d.qpos[1], 0.)
        d.mocap_quat[self.rig] = (math.cos(h / 2), 0., 0., math.sin(h / 2))

    def set_tilt(self, pitch):
        f = np.array([math.cos(pitch), 0., math.sin(pitch)])
        xa, ya = np.array([0., -1., 0.]), np.array([-math.sin(pitch), 0., math.cos(pitch)])
        R = np.stack([xa, ya, -f], axis=1).flatten()
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, R)
        self.m.cam_quat[self.cam] = q


def ucp_rx(sock, chs, sim_reset):
    while True:
        try:
            p, addr = sock.recvfrom(4096)
        except OSError:
            return
        if p.startswith(b"ucp:sync"):
            with chs.lock:
                chs.peer = addr
            sock.sendto(b"ucp:ack", addr)
            print(f"{time.strftime('%H:%M:%S')} sync from {addr}", flush=True)
        elif len(p) == 31 and p[0] == 0x51 and p[1] == 0x10:
            v = int.from_bytes(p[12:21], "little")
            ch = [(v >> (11 * i)) & 0x7FF for i in range(6)]
            with chs.lock:
                chs.ch, chs.last_rx = ch, time.monotonic()


class Video:
    """Rendered head camera -> ffmpeg HEVC Annex-B -> UCP 0x07/0x05 fragments."""

    def __init__(self, a, sock, chs):
        self.sock, self.chs, self.seq = sock, chs, 0
        enc = ["-c:v", "hevc_videotoolbox", "-realtime", "1", "-b:v", "4M"] if a.encoder == "vt" else \
              ["-c:v", "libx265", "-preset", "ultrafast", "-tune", "zerolatency", "-x265-params", "log-level=error"]
        self.p = subprocess.Popen(
            ["ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", f"{a.width}x{a.height}", "-r", str(a.fps), "-i", "-", *enc, "-pix_fmt", "yuv420p", "-g", str(a.fps),
             "-bsf:v", "hevc_mp4toannexb", "-f", "hevc", "-"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        threading.Thread(target=self.pump, daemon=True).start()

    def frame(self, rgb):
        try:
            self.p.stdin.write(rgb.tobytes())
        except BrokenPipeError:
            pass

    def send_nal(self, nal):
        with self.chs.lock:
            peer = self.chs.peer
        if not peer:
            return
        frags = [nal[i:i + 1460] for i in range(0, len(nal), 1460)]
        for j, f in enumerate(frags):
            typ = 0x05 if j == len(frags) - 1 else 0x07
            hdr = struct.pack("<BBHHHI", 0x51, typ, 0, self.seq & 0xFFFF, 12 + len(f), int(time.time() * 1e6) & 0xFFFFFFFF)
            try:
                self.sock.sendto(hdr + f, peer)
            except OSError:
                pass
            self.seq += 1

    def pump(self):
        buf = b""
        while True:
            chunk = self.p.stdout.read1(65536)
            if not chunk:
                return
            buf += chunk
            starts, i = [], 0
            while (i := buf.find(b"\x00\x00\x00\x01", i)) >= 0:
                starts.append(i)
                i += 4
            for s0, s1 in zip(starts, starts[1:]):
                self.send_nal(buf[s0:s1])
            if len(starts) > 1:
                buf = buf[starts[-1]:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--open-beni", required=True, help="path to an open-beni checkout")
    ap.add_argument("--policy", help="override policy .onnx")
    ap.add_argument("--port", type=int, default=19007)
    ap.add_argument("--vx-max", type=float, default=1.5, help="m/s at full stick")
    ap.add_argument("--wz-max", type=float, default=1.8, help="rad/s at full stick")
    ap.add_argument("--pan-max", type=float, default=math.radians(168), help="head yaw rad at ch2=1 (left; ch2=1999 right), measured by head-sweep flow")
    ap.add_argument("--tilt-max", type=float, default=math.radians(45), help="camera pitch rad at tilt 62")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--encoder", choices=["vt", "x265"], default="vt")
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--view", action="store_true", help="open the MuJoCo passive viewer (run with mjpython on macOS)")
    a = ap.parse_args()

    sim = Sim(a)
    chs = Channels()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", a.port))
    threading.Thread(target=ucp_rx, args=(sock, chs, sim.reset), daemon=True).start()
    video = None if a.no_video else Video(a, sock, chs)
    renderer = None if a.no_video else mujoco.Renderer(sim.m, a.height, a.width)
    viewer = None
    if a.view:
        from viewer_ui import BeniViewer
        viewer = BeniViewer(sim, "BENCH BENI  live")
    print(f"bench Beni on udp 127.0.0.1:{a.port}", flush=True)

    period, render_every = DT * DECIMATION, max(1, round(1 / (a.fps * DT * DECIMATION)))
    last_ch4, k, t_next, t_log, zmax = 32, 0, time.monotonic(), 0., 0.
    while True:
        with chs.lock:
            ch = list(chs.ch)
            stale = time.monotonic() - chs.last_rx > .5
        if stale:
            ch = [1000, 1000, 1000, 2015, 32, ch[5]]
        n = lambda v: max(-1., min(1., (v - 1000) / 999))
        vx, wz, yaw = n(ch[0]) * a.vx_max, -n(ch[1]) * a.wz_max, -n(ch[2]) * a.pan_max
        code = (ch[4] >> 6) if ch[4] & 32 else 0
        if ch[4] != last_ch4 and code:
            name = ACTIONS.get(code, f"code{code}")
            print(f"{time.strftime('%H:%M:%S')} action {name}" + ("" if code == 1 else " (not in policy; logged only)"), flush=True)
            if code == 1:
                sim.jump()
            if viewer:
                viewer.action(code)
        last_ch4 = ch[4]
        if viewer and viewer.reset_requested:
            viewer.reset_requested = False
            sim.reset()
        if viewer and viewer.paused:
            viewer.update(ch, vx, wz, "stale" if stale else "pilot")
            time.sleep(period)
            t_next = time.monotonic()
            continue
        sim.step(vx, wz, yaw)
        zmax = max(zmax, float(sim.d.qpos[2]))
        if renderer and k % render_every == 0:
            sim.set_tilt((((ch[3] & 0x3F) - 31) / 31) * a.tilt_max)
            renderer.update_scene(sim.d, camera=sim.cam)
            video.frame(renderer.render())
        if viewer:
            if not viewer.running():
                break
            viewer.update(ch, vx, wz, "no pilot" if stale else "pilot", (((ch[3] & 0x3F) - 31) / 31) * a.tilt_max)
        now = time.monotonic()
        if now - t_log > 1:
            t_log = now
            spd = float(np.hypot(*sim.d.qvel[0:2]))
            w, x, y, z = sim.d.qpos[3:7]
            hdg = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
            print(f"ch0={ch[0]} ch1={ch[1]} ch2={ch[2]} cmd vx={vx:+.2f} wz={wz:+.2f} | speed {spd:.2f} m/s "
                  f"z {sim.d.qpos[2]:.3f} (max {zmax:.3f}) hdg {hdg:+.0f} head {sim.d.qpos[sim.jq['head_yaw_joint']]:+.2f} rad", flush=True)
            zmax = 0.
        k += 1
        t_next += period
        dt = t_next - time.monotonic()
        if dt > 0:
            time.sleep(dt)
        else:
            t_next = time.monotonic()


if __name__ == "__main__":
    main()
