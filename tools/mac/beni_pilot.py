#!/usr/bin/env python3
"""Fly Beni from the Mac: live video plus a DualSense, through beni_bridge.

usage: beni_pilot.py https://<bridge>.trycloudflare.com

Controls (DualSense). Stick values are passed through raw and linear, full range; the robot shapes motion itself.
  left stick Y     flip direction only (no throttle)
  left stick X     steer
  R2 / L2          throttle / reverse
  right stick X    head pan (ch2), expo curve --head-expo (default 2.5)
  Cross            stick centered: jump.
                   direction held (|LX|+|LY| >= 0.5) + double tap within 0.3 s: direction jump only
                   (body frame, stick up = forward, snapped to Beni's 4 flips). One tap with direction held = jump.
  D-pad            direct flips: up = forward, down = back, left = left, right = right
  Touchpad click   E-STOP (latched on the bridge)
  Options          clear e-stop
Keyboard: Space = e-stop, C = clear, Esc = quit.
Sticks are live while the controller is connected. Link loss (>300 ms) -> neutral on the bridge.

Test without hardware: --headless --test-input "t=0.5,y=0,arm=1" --seconds 8
"""
import argparse, asyncio, collections, json, os, subprocess, threading, time

os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")

W, H = 960, 720
DEADZONE = 0.04  # only to kill stick drift at rest; values outside it pass through unscaled
# SDL2 joystick indices for DualSense
B_CROSS, B_OPTIONS, B_UP, B_DOWN, B_LEFT, B_RIGHT, B_TOUCH = 0, 6, 11, 12, 13, 14, 15
A_LX, A_LY, A_L2, A_R2 = 0, 1, 4, 5
A_RX = 2
DPAD_FLIPS = {B_UP: "flip_forward", B_DOWN: "flip_back", B_LEFT: "flip_left", B_RIGHT: "flip_right"}
# Rocket League dodge rules (RLUtilities src/mechanics/dodge.cc, src/simulation/car.cc)
DODGE_TIMEOUT = 1.5      # s after first jump in which a dodge is allowed
DODGE_THRESHOLD = 0.5    # |pitch| + |yaw| + |roll| >= 0.5 -> dodge, else double jump
DODGE_AXIS_ZERO = 0.1    # dodge_dir components below 0.1 are zeroed
DOUBLE_TAP = 0.3         # s: 2nd Cross within this, with direction held, sends only the direction jump


def rl_dodge(lx, ly):
    """RL dodge from left stick: dodge_dir = normalize(-pitch, yaw) in the body frame.
    Gamepad: stick up gives pitch -1, so forward = -ly; yaw = lx. Returns Beni action or None (double jump)."""
    if abs(lx) + abs(ly) < DODGE_THRESHOLD:
        return None
    fwd, right = -ly, lx
    n = (fwd * fwd + right * right) ** 0.5
    fwd, right = fwd / n, right / n
    fwd = 0.0 if abs(fwd) < DODGE_AXIS_ZERO else fwd
    right = 0.0 if abs(right) < DODGE_AXIS_ZERO else right
    if abs(fwd) >= abs(right):  # Beni has 4 discrete flips: snap to the dominant axis
        return "flip_forward" if fwd > 0 else "flip_back"
    return "flip_right" if right > 0 else "flip_left"


def trig(v):
    """DualSense trigger axis rests at -1, full press +1 -> 0..1."""
    x = (v + 1.0) / 2.0
    return 0.0 if x < DEADZONE else min(1.0, x)


def head_curve(v, expo):
    """Head pan response: |v|^expo keeps full range at full deflection, finer near center."""
    return (abs(v) ** expo) * (1 if v >= 0 else -1)


def raw(v):
    """Stick value as-is (-1..1), zero only inside the drift deadzone."""
    return 0.0 if abs(v) < DEADZONE else max(-1.0, min(1.0, v))


class Shared:
    def __init__(self, cap):
        self.lock = threading.Lock()
        self.ctl = {"t": 0.0, "y": 0.0, "arm": False, "cap": cap, "estop": False, "clear": False,
                    "boost": False, "action": None}
        self.status = {}
        self.ws_up = False
        self.frame = None
        self.frames = collections.deque(maxlen=60)    # jitter buffer of decoded frames
        self.arrivals = collections.deque(maxlen=120)  # arrival times, to estimate the source rate
        self.vfps = 0.0
        self.quit = threading.Event()


def video_thread(sh, url):
    cmd = ["/opt/homebrew/bin/ffmpeg", "-hide_banner", "-loglevel", "error",
           "-fflags", "nobuffer", "-flags", "low_delay", "-probesize", "128k", "-analyzeduration", "0",
           "-f", "hevc", "-i", url + "/video",
           "-vf", f"scale={W}:{H}", "-pix_fmt", "rgb24", "-fps_mode", "passthrough", "-f", "rawvideo", "-"]
    n = W * H * 3
    while not sh.quit.is_set():
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        frames, t0 = 0, time.monotonic()
        try:
            while not sh.quit.is_set():
                buf = bytearray(n)
                mv, got = memoryview(buf), 0
                while got < n:
                    k = p.stdout.readinto(mv[got:])
                    if not k:
                        raise EOFError
                    got += k
                with sh.lock:
                    sh.frames.append(bytes(buf))
                    sh.arrivals.append(time.monotonic())
                frames += 1
                dt = time.monotonic() - t0
                if dt >= 1.0:
                    sh.vfps, frames, t0 = frames / dt, 0, time.monotonic()
        except EOFError:
            pass
        finally:
            p.kill()
        sh.vfps = 0.0
        sh.quit.wait(1.0)


def ws_thread(sh, url):
    import websockets
    wurl = url.replace("https://", "wss://").replace("http://", "ws://") + "/ctl"

    async def run():
        while not sh.quit.is_set():
            try:
                async with websockets.connect(wurl, ping_interval=5, open_timeout=8, max_size=None) as ws:
                    sh.ws_up = True

                    async def rx():
                        async for m in ws:
                            with sh.lock:
                                sh.status = json.loads(m)

                    rt = asyncio.ensure_future(rx())
                    while not sh.quit.is_set() and not rt.done():
                        with sh.lock:
                            msg = dict(sh.ctl)
                            sh.ctl["clear"] = False  # one-shots
                            sh.ctl["action"] = None
                        await ws.send(json.dumps(msg))
                        await asyncio.sleep(1 / 30)
                    rt.cancel()
            except Exception:
                pass
            sh.ws_up = False
            await asyncio.sleep(1.0)

    asyncio.run(run())


def dz(v):
    return 0.0 if abs(v) < DEADZONE else (v - DEADZONE * (1 if v > 0 else -1)) / (1 - DEADZONE)


def main():
    a = argparse.ArgumentParser()
    a.add_argument("url")
    a.add_argument("--cap", type=float, default=1.0)
    a.add_argument("--head-expo", type=float, default=2.5, help="right-stick head pan curve (1 = linear)")
    a.add_argument("--buffer-ms", type=float, default=100, help="video jitter buffer (latency vs smoothness)")
    a.add_argument("--headless", action="store_true")
    a.add_argument("--test-input", help='e.g. "t=0.5,y=0,arm=1" (overrides the controller)')
    a.add_argument("--seconds", type=float, default=0)
    args = a.parse_args()
    url = args.url.rstrip("/")
    if args.headless:
        os.environ["SDL_VIDEODRIVER"] = "dummy"
    import pygame

    sh = Shared(args.cap)
    threading.Thread(target=video_thread, args=(sh, url), daemon=True).start()
    threading.Thread(target=ws_thread, args=(sh, url), daemon=True).start()

    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Beni pilot")
    font = pygame.font.SysFont("Menlo", 16)
    clock = pygame.time.Clock()
    js = pygame.joystick.Joystick(0) if pygame.joystick.get_count() else None
    if js:
        js.init()
    test = None
    if args.test_input:
        test = {k: float(v) for k, v in (kv.split("=") for kv in args.test_input.split(","))}
    end = time.monotonic() + args.seconds if args.seconds else None
    last_btn = None
    playing, next_show, buffered = False, 0.0, 0
    last_action = None
    pend_t, pend_dir = None, None

    while not sh.quit.is_set():
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                sh.quit.set()
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    sh.quit.set()
                elif e.key == pygame.K_SPACE:
                    with sh.lock:
                        sh.ctl["estop"] = True
                elif e.key == pygame.K_c:
                    with sh.lock:
                        sh.ctl["estop"], sh.ctl["clear"] = False, True
            elif e.type == pygame.JOYDEVICEADDED and js is None:
                js = pygame.joystick.Joystick(e.device_index)
                js.init()
            elif e.type == pygame.JOYDEVICEREMOVED:
                js = None
            elif e.type == pygame.JOYBUTTONDOWN:
                last_btn = e.button
                with sh.lock:
                    if e.button == B_TOUCH:
                        sh.ctl["estop"] = True
                    elif e.button == B_OPTIONS:
                        sh.ctl["estop"], sh.ctl["clear"] = False, True
                    elif e.button in DPAD_FLIPS:
                        sh.ctl["action"] = last_action = DPAD_FLIPS[e.button]
                    elif e.button == B_CROSS and js:
                        now_j = time.monotonic()
                        d = rl_dodge(js.get_axis(A_LX), js.get_axis(A_LY))
                        if pend_t is not None and now_j - pend_t <= DOUBLE_TAP:
                            sh.ctl["action"] = last_action = d or pend_dir  # double tap: direction jump only
                            pend_t = None
                        elif d:
                            pend_t, pend_dir = now_j, d  # direction held: wait for a possible 2nd tap
                        else:
                            sh.ctl["action"] = last_action = "jump"  # stick centered: jump now

        if pend_t is not None and time.monotonic() - pend_t > DOUBLE_TAP:
            with sh.lock:
                sh.ctl["action"] = last_action = "jump"  # no 2nd tap: it was a single jump
            pend_t = None

        with sh.lock:
            if test is not None:
                sh.ctl.update(t=test.get("t", 0.0), y=test.get("y", 0.0), arm=bool(test.get("arm", 0)))
            elif js:
                t = trig(js.get_axis(A_R2)) - trig(js.get_axis(A_L2))  # throttle only from triggers (RL); LY = flip direction only
                sh.ctl.update(t=t, y=raw(js.get_axis(A_LX)), h=head_curve(raw(js.get_axis(A_RX)), args.head_expo), boost=False, cap=1.0, arm=True)
            else:
                sh.ctl.update(t=0.0, y=0.0, arm=False)  # no controller: never armed
            ctl = dict(sh.ctl)
            st = dict(sh.status)
            # Jitter buffer: play frames out at the measured source rate, holding ~buffer_ms of slack.
            # Bursts from the tunnel are absorbed; if the backlog grows past 3x target, catch up by dropping.
            if len(sh.arrivals) >= 2:
                src_fps = (len(sh.arrivals) - 1) / max(1e-3, sh.arrivals[-1] - sh.arrivals[0])
            else:
                src_fps = 30.0
            target = max(1, int(src_fps * args.buffer_ms / 1000))
            now = time.monotonic()
            if len(sh.frames) > 3 * target:
                while len(sh.frames) > target:
                    sh.frames.popleft()
            if sh.frames and (playing or len(sh.frames) >= target) and now >= next_show:
                sh.frame = sh.frames.popleft()
                playing = True
                next_show = max(next_show + 1.0 / src_fps, now - 0.05)
            elif not sh.frames:
                playing = False  # underrun: rebuffer to target before resuming
            frame = sh.frame
            buffered = len(sh.frames)

        if frame:
            screen.blit(pygame.image.frombuffer(frame, (W, H), "RGB"), (0, 0))
        else:
            screen.fill((20, 20, 20))
        armed = ctl["arm"] and not st.get("estop")
        lines = [
            f"link {'UP' if sh.ws_up else 'DOWN'}  ucp {'up' if st.get('ucp') else 'down'}  "
            f"video {sh.vfps:4.1f} fps (beni {st.get('fps', 0)})  buf {buffered}  ctl age {st.get('ctl_age_ms')} ms",
            f"{'E-STOP (Options to clear)' if st.get('estop') else ('LIVE' if armed else 'no controller')}"
            f"{'  BOOST' if ctl.get('boost') else ''}"
            f"   cap {ctl['cap']:.0%} (max {st.get('max_cap', 0):.0%})   ch {st.get('ch')}"
            f"   pad {'ok' if js else 'NONE'}  btn {last_btn}",
            f"last action {last_action}  bridge: {st.get('last_action')} ({st.get('action_note', '')})",
        ]
        color = (255, 60, 60) if st.get("estop") else ((60, 220, 90) if armed else (230, 230, 230))
        for i, s in enumerate(lines):
            surf = font.render(s, True, color if i == 1 else (230, 230, 230), (0, 0, 0))
            screen.blit(surf, (8, 8 + i * 20))
        pygame.display.flip()
        clock.tick(60)
        if end and time.monotonic() > end:
            print("status:", json.dumps(st), "vfps:", round(sh.vfps, 1), "ws_up:", sh.ws_up, flush=True)
            sh.quit.set()

    with sh.lock:  # send a final neutral/disarm
        sh.ctl.update(t=0.0, y=0.0, arm=False)
    time.sleep(0.2)
    pygame.quit()


if __name__ == "__main__":
    main()
