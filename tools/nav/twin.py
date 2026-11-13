"""Real-to-sim twin link: the robot side publishes, the sim (beni_view live scene) mirrors.

Transport: TCP 127.0.0.1:19100, one JSON object per line. Frame of reference is the runner's start pose:
x forward, y right, yaw + right (the sim flips y and yaw).
  {"type": "state", "t": s, "ch": [ch0..ch5], "pose": [x, y, yaw], "cam": [x, y, z, yaw, pitch], "hfov": deg}
  {"type": "frame", "jpg": base64 JPEG of the camera image}
  {"type": "marks", "marks": [[x, y, half], ...]}          mapped objects
  {"type": "points", "xyz": base64 float32 Nx3, "rgb": base64 uint8 Nx3}   (map points; for mapping)
Sim -> robot on the same connection:
  {"type": "ctl", "armed": bool, "ch": [ch0..ch5]}         sticks from the sim (debugger keys) while armed
The pose source is pluggable: anything that yields a camera pose per frame (flow+depth odometry now; a learned
multi-view pose model later) fills "cam" and "pose".
"""
import base64, json, queue, socket, threading, time

import cv2
import numpy as np

PORT = 19100


class TwinPublisher:
    """Non-blocking: drops messages while the sim is not listening; reconnects every 2 s."""

    def __init__(self, host="127.0.0.1", port=PORT):
        self.addr = (host, port)
        self.q = queue.Queue(maxsize=64)
        self.connected = False
        self.ctl, self.ctl_t = None, 0.0     # latest sim control message and its arrival time
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                s = socket.create_connection(self.addr, timeout=2)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(None)
                self.connected = True
                threading.Thread(target=self._rx, args=(s,), daemon=True).start()
                while True:
                    s.sendall(self.q.get())
            except OSError:
                self.connected = False
                self.ctl = None
                time.sleep(2)

    def _rx(self, s):
        try:
            for line in s.makefile("rb"):
                try:
                    m = json.loads(line)
                except ValueError:
                    continue
                if m.get("type") == "ctl":
                    self.ctl, self.ctl_t = m, time.monotonic()
        except OSError:
            pass

    def sim_ctl(self, max_age=0.3):
        """The sim's control message if armed and fresh, else None."""
        m = self.ctl
        if m and m.get("armed") and time.monotonic() - self.ctl_t < max_age:
            return m
        return None

    def send(self, msg):
        if not self.connected:
            return
        b = (json.dumps(msg) + "\n").encode()
        try:
            self.q.put_nowait(b)
        except queue.Full:
            pass

    def state(self, ch, pose, cam, hfov, posture="stand"):
        """posture: "sit" (disabled, legs folded, no policy) or "stand"."""
        self.send({"type": "state", "t": time.time(), "ch": [int(c) for c in ch],
                   "pose": [float(v) for v in pose], "cam": [float(v) for v in cam], "hfov": float(hfov),
                   "posture": posture})

    def frame(self, rgb, w=320, h=240):
        small = cv2.resize(rgb, (w, h), interpolation=cv2.INTER_AREA)
        ok, jpg = cv2.imencode(".jpg", cv2.cvtColor(small, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            self.send({"type": "frame", "jpg": base64.b64encode(jpg.tobytes()).decode()})

    def marks(self, marks):
        self.send({"type": "marks", "marks": [[float(m[0]), float(m[1]), float(m[2])] for m in marks]})

    def points(self, xyz, rgb):
        self.send({"type": "points", "xyz": base64.b64encode(np.asarray(xyz, np.float32).tobytes()).decode(),
                   "rgb": base64.b64encode(np.asarray(rgb, np.uint8).tobytes()).decode()})


class TwinServer:
    """Sim side: latest state / frame / marks, plus an accumulating point map."""

    def __init__(self, port=PORT):
        self.lock = threading.Lock()
        self.state = None
        self.state_t = 0.0
        self.frame = None          # RGB uint8 HxWx3
        self.frame_new = False
        self.frame_t = -1e9        # when the last camera frame arrived
        self.marks = []
        self.path = []
        self.points = []           # list of (xyz Nx3, rgb Nx3)
        self.clients = 0
        self.socks = []
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", port))
        srv.listen(2)
        threading.Thread(target=self._accept, args=(srv,), daemon=True).start()

    def _accept(self, srv):
        while True:
            c, _ = srv.accept()
            threading.Thread(target=self._client, args=(c,), daemon=True).start()

    def _client(self, c):
        c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        with self.lock:
            self.clients += 1
            self.socks.append(c)
        f = c.makefile("rb")
        try:
            for line in f:
                try:
                    m = json.loads(line)
                except ValueError:
                    continue
                self._handle(m)
        finally:
            with self.lock:
                self.clients -= 1
                if c in self.socks:
                    self.socks.remove(c)

    def _handle(self, m):
        t = m.get("type")
        with self.lock:
            if t == "state":
                self.state, self.state_t = m, time.monotonic()
                p = m["pose"]
                if not self.path or abs(self.path[-1][0] - p[0]) + abs(self.path[-1][1] - p[1]) > 0.02:
                    self.path.append((p[0], p[1]))
                    self.path = self.path[-400:]
            elif t == "frame":
                buf = np.frombuffer(base64.b64decode(m["jpg"]), np.uint8)
                img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                if img is not None:
                    self.frame, self.frame_new = cv2.cvtColor(img, cv2.COLOR_BGR2RGB), True
                    self.frame_t = time.monotonic()
            elif t == "marks":
                self.marks = m["marks"]
            elif t == "points":
                xyz = np.frombuffer(base64.b64decode(m["xyz"]), np.float32).reshape(-1, 3)
                rgb = np.frombuffer(base64.b64decode(m["rgb"]), np.uint8).reshape(-1, 3)
                self.points.append((xyz, rgb))

    def send(self, msg):
        b = (json.dumps(msg) + "\n").encode()
        with self.lock:
            socks = list(self.socks)
        for c in socks:
            try:
                c.sendall(b)
            except OSError:
                pass

    def reset(self):
        with self.lock:
            self.path, self.marks, self.points = [], [], []
