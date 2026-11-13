#!/usr/bin/env python3
"""Beni bridge: one UCP session to Beni, serving video and controls over HTTP/WebSocket. Stdlib only.

  GET /video   raw H.265 Annex-B, starting at the next VPS/IRAP
  GET /ctl     WebSocket. Client sends JSON {"t":-1..1, "y":-1..1, "arm":bool, "cap":0..1, "estop":bool, "clear":bool};
               the server pushes status JSON at 2 Hz.
  GET /status  JSON
  GET /stop    exit (the wrapper then restores home Wi-Fi)

Safety: output is neutral unless armed, not e-stopped, and the last control message is < --deadman s old.
The speed cap is clamped to --max-cap. The e-stop latches until the client sends "clear".
The 50 Hz ack frame doubles as the stick frame: same packet, current channels plus video ack.
"""
import argparse, base64, hashlib, json, queue, socket, struct, threading, time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

TEMPLATE = bytes.fromhex("e8431ffabe0f021805")  # captured neutral body, ch5=545
VIDEO = (0x05, 0x07)
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def pack(ch0, ch1):
    v = int.from_bytes(TEMPLATE, "little") & ~((1 << 22) - 1)
    v |= (ch0 & 0x7FF) | ((ch1 & 0x7FF) << 11)
    return v.to_bytes(9, "little")


class Sub:
    def __init__(self):
        self.q = queue.Queue(maxsize=240)
        self.started = False


class State:
    def __init__(self, max_cap, deadman):
        self.lock = threading.Lock()
        self.thr = self.yaw = 0.0
        self.arm = self.estop = False
        self.cap = min(0.3, max_cap)
        self.max_cap, self.deadman = max_cap, deadman
        self.ctl_t = 0.0
        self.ch = (1000, 1000)
        self.ucp_up = False
        self.rx_frags = self.rx_frames = 0
        self.fps = 0.0
        self.subs = []
        self.ws_clients = 0
        self.stop = threading.Event()
        self.t0 = time.time()

    def channels(self):
        with self.lock:
            live = self.arm and not self.estop and (time.monotonic() - self.ctl_t) < self.deadman
            cap = min(self.cap, self.max_cap)
            t, y = (self.thr, self.yaw) if live else (0.0, 0.0)
        c0 = 1000 + int(round(max(-1.0, min(1.0, t)) * cap * 999))
        c1 = 1000 + int(round(max(-1.0, min(1.0, y)) * cap * 999))
        return c0, c1

    def apply(self, m):
        with self.lock:
            self.thr = float(m.get("t", 0.0))
            self.yaw = float(m.get("y", 0.0))
            self.arm = bool(m.get("arm", False))
            if "cap" in m:
                self.cap = max(0.0, min(float(m["cap"]), self.max_cap))
            if m.get("estop"):
                if not self.estop:
                    log("E-STOP latched")
                self.estop = True
            if m.get("clear") and self.estop:
                log("e-stop cleared")
                self.estop = False
            self.ctl_t = time.monotonic()

    def status(self):
        with self.lock:
            return {
                "up_s": round(time.time() - self.t0), "ucp": self.ucp_up,
                "fps": round(self.fps, 1), "rx_frags": self.rx_frags,
                "ch": self.ch, "arm": self.arm, "estop": self.estop,
                "cap": self.cap, "max_cap": self.max_cap,
                "ctl_age_ms": int((time.monotonic() - self.ctl_t) * 1000) if self.ctl_t else None,
                "video_clients": len(self.subs), "ws_clients": self.ws_clients,
            }


def is_key(frame):
    i, n = 0, 0
    head = frame[:8192]
    while n < 8:
        i = head.find(b"\x00\x00\x01", i)
        if i < 0 or i + 3 >= len(head):
            return False
        t = (head[i + 3] >> 1) & 0x3F
        if t in (32, 33, 34) or 16 <= t <= 21:
            return True
        i += 3
        n += 1
    return False


def publish(st, frame):
    st.rx_frames += 1
    key = None
    with st.lock:
        subs = list(st.subs)
    for s in subs:
        if not s.started:
            if key is None:
                key = is_key(frame)
            if not key:
                continue
            s.started = True
        try:
            s.q.put_nowait(frame)
        except queue.Full:  # slow client: drop its backlog, resume at the next keyframe
            while not s.q.empty():
                try:
                    s.q.get_nowait()
                except queue.Empty:
                    break
            s.started = False


def ucp_session(st, robot, bind):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        if bind:
            s.bind((bind, 0))
        s.connect(robot)
        s.settimeout(0.3)
        for _ in range(20):
            if st.stop.is_set():
                return
            s.send(b"ucp:sync")
            try:
                if s.recv(64).startswith(b"ucp:ack"):
                    break
            except socket.timeout:
                pass
        else:
            log("no ucp:ack")
            return
        log("ucp:ack")
        st.ucp_up = True
        s.setblocking(False)
        ack, frame = 0, []
        t0 = time.monotonic()
        nxt = t0
        last_rx = t0
        while not st.stop.is_set():
            now = time.monotonic()
            if now >= nxt:
                c0, c1 = st.channels()
                st.ch = (c0, c1)
                body = pack(c0, c1) + struct.pack("<H", ack) + b"\0" * 8
                ts = int((now - t0) * 1e6) & 0xFFFFFFFF
                s.send(struct.pack("<BBHHHI", 0x51, 0x10, ack, 0, 12 + len(body), ts) + body)
                nxt += 0.02
                if nxt < now:
                    nxt = now + 0.02
            try:
                while True:
                    p = s.recv(4096)
                    if len(p) >= 12 and p[0] == 0x51 and p[1] in VIDEO:
                        ack = struct.unpack_from("<H", p, 4)[0]
                        frame.append(p[12:])
                        st.rx_frags += 1
                        last_rx = now
                        if p[1] == 0x05 or len(frame) > 600:
                            publish(st, b"".join(frame))
                            frame = []
            except (BlockingIOError, InterruptedError):
                pass
            if now - last_rx > 5:
                log("video stalled 5 s; resyncing")
                return
            time.sleep(0.002)
    finally:
        try:  # leave Beni at neutral
            body = pack(1000, 1000) + b"\0" * 10
            for _ in range(10):
                s.send(struct.pack("<BBHHHI", 0x51, 0x10, 0, 0, 12 + len(body), 0) + body)
                time.sleep(0.02)
        except OSError:
            pass
        st.ucp_up = False
        s.close()


def ucp_loop(st, robot, bind):
    while not st.stop.is_set():
        try:
            ucp_session(st, robot, bind)
        except OSError as e:
            log("ucp error:", e)
        if not st.stop.is_set():
            time.sleep(1)


def fps_loop(st):
    last = st.rx_frames
    while not st.stop.wait(1.0):
        st.fps = st.rx_frames - last
        last = st.rx_frames


def recv_exact(conn, n, first=False):
    buf = b""
    while len(buf) < n:
        try:
            chunk = conn.recv(n - len(buf))
        except socket.timeout:
            if first and not buf:
                raise
            continue
        if not chunk:
            raise ConnectionError("closed")
        buf += chunk
    return buf


def ws_recv(conn):
    h = recv_exact(conn, 2, first=True)
    op, masked, ln = h[0] & 0x0F, h[1] & 0x80, h[1] & 0x7F
    if ln == 126:
        ln = struct.unpack(">H", recv_exact(conn, 2))[0]
    elif ln == 127:
        ln = struct.unpack(">Q", recv_exact(conn, 8))[0]
    mask = recv_exact(conn, 4) if masked else b"\0\0\0\0"
    data = bytearray(recv_exact(conn, ln))
    for i in range(ln):
        data[i] ^= mask[i & 3]
    return op, bytes(data)


def ws_send(conn, data, op=1):
    n = len(data)
    if n < 126:
        hdr = bytes([0x80 | op, n])
    elif n < 65536:
        hdr = bytes([0x80 | op, 126]) + struct.pack(">H", n)
    else:
        hdr = bytes([0x80 | op, 127]) + struct.pack(">Q", n)
    conn.sendall(hdr + data)


ST = None


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def do_GET(self):
        p = self.path.split("?")[0]
        if p == "/video":
            return self.video()
        if p == "/ctl":
            return self.ws()
        if p in ("/", "/status"):
            return self.json(ST.status())
        if p == "/stop":
            log("stop requested over HTTP")
            ST.stop.set()
            return self.json({"stopping": True})
        self.send_error(404)

    def json(self, obj):
        b = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(b)

    def video(self):
        sub = Sub()
        with ST.lock:
            ST.subs.append(sub)
        log("video client +", self.client_address[0])
        try:
            self.send_response(200)
            self.send_header("Content-Type", "video/hevc")
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Accel-Buffering", "no")
            self.send_header("Transfer-Encoding", "chunked")
            self.end_headers()
            while not ST.stop.is_set():
                try:
                    f = sub.q.get(timeout=1.0)
                except queue.Empty:
                    continue
                self.wfile.write(b"%x\r\n" % len(f) + f + b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass
        finally:
            with ST.lock:
                ST.subs.remove(sub)
            log("video client -")
            self.close_connection = True

    def ws(self):
        key = self.headers.get("Sec-WebSocket-Key")
        if not key or "websocket" not in self.headers.get("Upgrade", "").lower():
            return self.send_error(400, "websocket upgrade required")
        acc = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", acc)
        self.end_headers()
        conn = self.connection
        conn.settimeout(0.25)
        with ST.lock:
            ST.ws_clients += 1
        log("ctl client +")
        last_status = 0.0
        try:
            while not ST.stop.is_set():
                if time.monotonic() - last_status > 0.5:
                    ws_send(conn, json.dumps(ST.status()).encode())
                    last_status = time.monotonic()
                try:
                    op, data = ws_recv(conn)
                except socket.timeout:
                    continue
                if op == 8:
                    break
                if op == 9:
                    ws_send(conn, data, 10)
                elif op == 1:
                    ST.apply(json.loads(data))
        except (ConnectionError, OSError, ValueError):
            pass
        finally:
            with ST.lock:
                ST.ws_clients -= 1
                ST.arm = False  # dropped link disarms
            log("ctl client - (disarmed)")
            self.close_connection = True


def main():
    global ST
    a = argparse.ArgumentParser()
    a.add_argument("--robot", default="192.168.7.1:9007")
    a.add_argument("--bind", help="local IP on Beni's AP (wlan0)")
    a.add_argument("--port", type=int, default=8780)
    a.add_argument("--max-cap", type=float, default=0.5, help="hard ceiling on |stick| fraction")
    a.add_argument("--deadman", type=float, default=0.3, help="seconds without ctl before neutral")
    args = a.parse_args()
    host, port = args.robot.rsplit(":", 1)
    ST = State(args.max_cap, args.deadman)
    threading.Thread(target=ucp_loop, args=(ST, (host, int(port)), args.bind), daemon=True).start()
    threading.Thread(target=fps_loop, args=(ST,), daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    log(f"bridge on :{args.port}, robot {args.robot}, bind {args.bind}, max_cap {args.max_cap}, deadman {args.deadman}s")
    try:
        while not ST.stop.wait(0.5):
            pass
    except KeyboardInterrupt:
        ST.stop.set()
    time.sleep(0.5)
    srv.shutdown()
    log("bridge exit")


if __name__ == "__main__":
    main()
