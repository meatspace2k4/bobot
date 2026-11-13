#!/usr/bin/env python3
"""Replay a captured stick timeline (topic_trace.py *.sticks.csv) through the bench sim, headless,
and record the sim's raw sensors at the 100 Hz policy rate.

Output npz: t (capture clock), ch (N,6), sensors (N, nsensordata), names, qpos, qvel,
            speed (m/s, body forward), yaw_rate, pitch, roll, height, head_yaw

usage: replay.py --open-beni PATH --sticks X.sticks.csv --out X.sim.npz [same scale flags as beni_sim]
"""
import argparse, csv, math, os, sys, time
import numpy as np
import mujoco

sys.path.insert(0, os.path.dirname(__file__))
from beni_sim import Sim, DT, DECIMATION


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--open-beni", required=True)
    ap.add_argument("--policy")
    ap.add_argument("--sticks", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--vx-max", type=float, default=1.5, help="m/s at full stick")
    ap.add_argument("--wz-max", type=float, default=1.8, help="rad/s at full stick")
    ap.add_argument("--pan-max", type=float, default=math.radians(168))
    ap.add_argument("--view", action="store_true", help="watch it in real time (run with mjpython on macOS)")
    a = ap.parse_args()
    rows = [(float(r["t"]), [int(r[f"ch{i}"]) for i in range(6)]) for r in csv.DictReader(open(a.sticks))]
    sim = Sim(a)
    m, d = sim.m, sim.d
    names = []
    for i in range(m.nsensor):
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SENSOR, i)
        names += [f"{nm}[{k}]" for k in range(m.sensor_dim[i])]
    step = DT * DECIMATION
    t, j, last_ch4 = rows[0][0], 0, 32
    T, CH, S, QP, QV = [], [], [], [], []
    n = lambda v: max(-1., min(1., (v - 1000) / 999))
    viewer = None
    if a.view:
        from viewer_ui import BeniViewer
        viewer = BeniViewer(sim, f"REPLAY  {os.path.basename(a.sticks).split('.')[0]}")
        wall0 = time.monotonic()
    while t < rows[-1][0]:
        while j + 1 < len(rows) and rows[j + 1][0] <= t:
            j += 1
        ch = rows[j][1]
        code = (ch[4] >> 6) if ch[4] & 32 else 0
        if ch[4] != last_ch4 and code:
            print(f"{t - rows[0][0]:7.2f}s action code {code}" + ("" if code == 1 else " (no flip in policy)"), flush=True)
            if code == 1:
                sim.jump()
            if viewer:
                viewer.action(code)
        last_ch4 = ch[4]
        sim.step(n(ch[0]) * a.vx_max, -n(ch[1]) * a.wz_max, sim.head.step(-n(ch[2]), step))
        T.append(t); CH.append(ch); S.append(d.sensordata.copy()); QP.append(d.qpos.copy()); QV.append(d.qvel.copy())
        t += step
        if viewer:
            if not viewer.running():
                break
            if viewer.reset_requested:
                viewer.reset_requested = False
                sim.reset()
            while viewer.paused and viewer.running():
                viewer.update(ch, n(ch[0]) * a.vx_max, -n(ch[1]) * a.wz_max, f"{t - rows[0][0]:6.1f} / {rows[-1][0] - rows[0][0]:.0f} s")
                time.sleep(1 / 30)
                wall0 += 1 / 30
            viewer.update(ch, n(ch[0]) * a.vx_max, -n(ch[1]) * a.wz_max, f"{t - rows[0][0]:6.1f} / {rows[-1][0] - rows[0][0]:.0f} s")
            ahead = (t - rows[0][0]) - (time.monotonic() - wall0)
            if ahead > 0:
                time.sleep(ahead)
    qp, qv = np.array(QP), np.array(QV)
    w, x, y, z = qp[:, 3], qp[:, 4], qp[:, 5], qp[:, 6]
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    hdg = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    fwd = qv[:, 0] * np.cos(hdg) + qv[:, 1] * np.sin(hdg)
    np.savez_compressed(a.out, t=np.array(T), ch=np.array(CH), sensors=np.array(S), names=np.array(names),
                        qpos=qp, qvel=qv, speed=fwd, yaw_rate=qv[:, 5], pitch=pitch, roll=roll,
                        height=qp[:, 2], head_yaw=qp[:, sim.jq["head_yaw_joint"]])
    print(f"{len(T)} steps ({len(T) * step:.0f} s sim) -> {a.out}")


if __name__ == "__main__":
    main()
