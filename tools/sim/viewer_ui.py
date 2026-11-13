"""Viewer UI shared by beni_sim.py and replay.py (run under mjpython on macOS).

Window: chase camera on Beni, HUD (sticks, commands, measured state, last action),
head-camera inset bottom right, speed / yaw-rate strip chart bottom left.
Keys: C cycles camera (chase / head / free), H toggles HUD, R resets the sim, Space pauses.
Tab / Shift+Tab still open MuJoCo's own panels.
"""
import collections, math, time

import mujoco
import numpy as np
from mujoco import viewer as mjviewer

KEY_SPACE, KEY_C, KEY_H, KEY_R = 32, 67, 72, 82
INSET_W, INSET_H = 480, 360
STRIP = 300  # samples kept (≈10 s at the 30 Hz UI rate)
ACTION_NAMES = {1: "JUMP", 2: "FLIP FORWARD", 4: "FLIP BACK", 6: "FLIP LEFT", 8: "FLIP RIGHT"}


def dress_scene(root):
    """Sky, shadows and haze for the XML tree built by beni_sim.build_xml."""
    import xml.etree.ElementTree as ET
    asset = root.find("asset")
    ET.SubElement(asset, "texture", name="sky", type="skybox", builtin="gradient",
                  rgb1=".42 .55 .70", rgb2=".08 .10 .14", width="512", height="512")
    vis = root.find("visual")
    ET.SubElement(vis, "headlight", ambient=".35 .35 .35", diffuse=".45 .45 .45", specular="0 0 0")
    ET.SubElement(vis, "quality", shadowsize="4096")
    ET.SubElement(vis, "map", haze=".25", fogstart="6", fogend="18")
    ET.SubElement(vis, "rgba", haze=".18 .22 .28 1")
    for light in root.find("worldbody").findall("light"):
        light.set("castshadow", "true")
        light.set("dir", "-.3 -.2 -1")
        light.set("pos", "2 1 5")


class BeniViewer:
    def __init__(self, sim, title, inset=True):
        self.sim, self.title = sim, title
        self.paused = False
        self.hud = True
        self.cam_mode = 0  # 0 chase, 1 head, 2 free
        self.reset_requested = False
        self.last_action, self.last_action_t = None, -1e9
        self.speed = collections.deque(maxlen=STRIP)
        self.yaw = collections.deque(maxlen=STRIP)
        self.next_ui = 0.
        # Offscreen context first: creating it after the viewer is up hangs under mjpython.
        self.inset = mujoco.Renderer(sim.m, INSET_H, INSET_W) if inset else None
        self.v = mjviewer.launch_passive(sim.m, sim.d, key_callback=self._key,
                                         show_left_ui=False, show_right_ui=False)
        self.base = mujoco.mj_name2id(sim.m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self._chase()
        self.fig = mujoco.MjvFigure()
        f = self.fig
        f.title = "last 10 s"
        f.flg_extend = 0
        f.flg_legend = 1
        f.gridsize = [3, 5]
        f.range[0] = [-STRIP, 0]
        f.range[1] = [-3, 3]
        f.linename[0] = "speed m/s"
        f.linename[1] = "yaw rate rad/s"
        f.linergb[0] = [.35, .85, .45]
        f.linergb[1] = [.95, .65, .25]
        f.linewidth = 2
        f.figurergba = [.05, .06, .08, .85]
        f.panergba = [.10, .12, .15, .85]
        f.legendrgba = [.05, .06, .08, .6]

    def _chase(self):
        c = self.v.cam
        c.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        c.trackbodyid = self.base
        c.distance, c.elevation, c.azimuth = 1.1, -18, 0

    def _key(self, key):
        if key == KEY_SPACE:
            self.paused = not self.paused
        elif key == KEY_H:
            self.hud = not self.hud
        elif key == KEY_R:
            self.reset_requested = True
        elif key == KEY_C:
            self.cam_mode = (self.cam_mode + 1) % 3
            c = self.v.cam
            if self.cam_mode == 0:
                self._chase()
            elif self.cam_mode == 1:
                c.type = mujoco.mjtCamera.mjCAMERA_FIXED
                c.fixedcamid = self.sim.cam
            else:
                c.type = mujoco.mjtCamera.mjCAMERA_FREE

    def running(self):
        return self.v.is_running()

    def action(self, code):
        self.last_action, self.last_action_t = ACTION_NAMES.get(code, f"CODE {code}"), time.monotonic()

    def update(self, ch, vx_cmd, wz_cmd, status, tilt_pitch=0.):
        """Sync physics every call; redraw overlays at ~30 Hz."""
        now = time.monotonic()
        if now < self.next_ui:
            self.v.sync()
            return
        self.next_ui = now + 1 / 30
        d, sim = self.sim.d, self.sim
        w, x, y, z = d.qpos[3:7]
        hdg = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        spd = d.qvel[0] * math.cos(hdg) + d.qvel[1] * math.sin(hdg)
        if self.cam_mode == 0:
            # Follow from behind: ease azimuth toward heading.
            c = self.v.cam
            err = (math.degrees(hdg) - c.azimuth + 180) % 360 - 180
            c.azimuth += 0.15 * err
        self.speed.append(spd)
        self.yaw.append(d.qvel[5])
        img = None
        if self.inset and self.hud:
            sim.set_tilt(tilt_pitch)
            self.inset.update_scene(d, camera=sim.cam)
            img = self.inset.render().copy()
        if True:  # set_texts/set_images/set_figures take the viewer lock themselves
            texts = []
            if self.hud:
                act = ""
                if self.last_action and now - self.last_action_t < 2.0:
                    act = self.last_action
                tilt = ch[3] & 0x3F
                left = "\n".join([
                    self.title, "",
                    "throttle  ch0", "yaw       ch1", "head pan  ch2", "head tilt ch3", "action    ch4", "mode      ch5", "",
                    "cmd vx", "cmd wz", "", "speed", "yaw rate", "heading", "height", "head yaw",
                ])
                right = "\n".join([
                    status + ("   PAUSED" if self.paused else ""), "",
                    f"{ch[0]:4d}", f"{ch[1]:4d}", f"{ch[2]:4d}", f"{tilt:4d}", f"{ch[4]:4d}  {act}", f"{ch[5]:4d}", "",
                    f"{vx_cmd:+.2f} m/s", f"{wz_cmd:+.2f} rad/s", "",
                    f"{spd:+.2f} m/s", f"{d.qvel[5]:+.2f} rad/s", f"{math.degrees(hdg):+5.0f} deg",
                    f"{d.qpos[2]:.3f} m", f"{math.degrees(d.qpos[sim.jq['head_yaw_joint']]):+5.0f} deg",
                ])
                texts.append((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT, left, right))
                texts.append((mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPRIGHT,
                              "C camera  H hud  R reset  Space pause", ["chase", "head", "free"][self.cam_mode]))
            self.v.set_texts(texts)
            vp = self.v.viewport
            if vp is not None and vp.width > 0:
                imgs, figs = [], []
                if img is not None:
                    imgs.append((mujoco.MjrRect(vp.width - INSET_W - 12, 12, INSET_W, INSET_H), img))
                if self.hud:
                    f = self.fig
                    for li, series in enumerate((self.speed, self.yaw)):
                        n = len(series)
                        f.linepnt[li] = n
                        f.linedata[li][0:2 * n:2] = np.arange(n) - n
                        f.linedata[li][1:2 * n:2] = np.asarray(series)
                    fw, fh = vp.width // 3, vp.height // 4
                    figs.append((mujoco.MjrRect(12, 12, fw, fh), f))
                self.v.set_images(imgs) if imgs else self.v.clear_images()
                self.v.set_figures(figs) if figs else self.v.clear_figures()
        self.v.sync()
