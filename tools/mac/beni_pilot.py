#!/usr/bin/env python3
"""Fly Beni from the Mac: live video plus a DualSense, through beni_bridge.

usage: beni_pilot.py https://<bridge>.trycloudflare.com [--cap 0.3]

Controls (DualSense):
  hold R1          arm (deadman; release = neutral)
  left stick Y     throttle
  right stick X    yaw
  Circle           E-STOP (latched on the bridge)
  Options          clear e-stop
  D-pad up/down    speed cap +/- 10 %
Keyboard: Space = e-stop, C = clear, Esc = quit.

Test without hardware: --headless --test-input "t=0.5,y=0,arm=1" --seconds 8
"""
import argparse, asyncio, json, os, subprocess, threading, time

os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")

W, H = 960, 720
DEADZONE = 0.08
# SDL2 joystick indices for DualSense
B_CROSS, B_CIRCLE, B_OPTIONS, B_R1, B_UP, B_DOWN = 0, 1, 6, 10, 11, 12
A_LY, A_RX = 1, 2


class Shared:
    def __init__(self, cap):
        self.lock = threading.Lock()
        self.ctl = {"t": 0.0, "y": 0.0, "arm": False, "cap": cap, "estop": False, "clear": False}
        self.status = {}
        self.ws_up = False
        self.frame = None
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
                buf = bytearray()
                while len(buf) < n:
                    chunk = p.stdout.read(n - len(buf))
                    if not chunk:
                        raise EOFError
                    buf += chunk
                with sh.lock:
                    sh.frame = bytes(buf)
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
                            sh.ctl["clear"] = False  # one-shot
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
    a.add_argument("--cap", type=float, default=0.3)
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
                    if e.button == B_CIRCLE:
                        sh.ctl["estop"] = True
                    elif e.button == B_OPTIONS:
                        sh.ctl["estop"], sh.ctl["clear"] = False, True
                    elif e.button == B_UP:
                        sh.ctl["cap"] = round(min(1.0, sh.ctl["cap"] + 0.1), 2)
                    elif e.button == B_DOWN:
                        sh.ctl["cap"] = round(max(0.1, sh.ctl["cap"] - 0.1), 2)

        with sh.lock:
            if test is not None:
                sh.ctl.update(t=test.get("t", 0.0), y=test.get("y", 0.0), arm=bool(test.get("arm", 0)))
            elif js:
                sh.ctl.update(t=-dz(js.get_axis(A_LY)), y=dz(js.get_axis(A_RX)), arm=bool(js.get_button(B_R1)))
            else:
                sh.ctl.update(t=0.0, y=0.0, arm=False)  # no controller: never armed
            ctl = dict(sh.ctl)
            st = dict(sh.status)
            frame = sh.frame

        if frame:
            screen.blit(pygame.image.frombuffer(frame, (W, H), "RGB"), (0, 0))
        else:
            screen.fill((20, 20, 20))
        armed = ctl["arm"] and not st.get("estop")
        lines = [
            f"link {'UP' if sh.ws_up else 'DOWN'}  ucp {'up' if st.get('ucp') else 'down'}  "
            f"video {sh.vfps:4.1f} fps (beni {st.get('fps', 0)})  ctl age {st.get('ctl_age_ms')} ms",
            f"{'E-STOP (Options to clear)' if st.get('estop') else ('ARMED' if armed else 'disarmed (hold R1)')}"
            f"   cap {ctl['cap']:.0%} (max {st.get('max_cap', 0):.0%})   ch {st.get('ch')}"
            f"   pad {'ok' if js else 'NONE'}  btn {last_btn}",
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
