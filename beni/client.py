"""Direct UCP client for Beni: sticks, head, actions out; H.265 video in.

    from beni import Beni
    with Beni() as b:                 # joins 192.168.7.1:9007 (be on Beni's Wi-Fi)
        b.drive(throttle=0.3)         # held and re-sent at 50 Hz until changed
        time.sleep(2)
        b.action("jump")
        for frame in b.frames(seconds=5):
            ...                       # Annex-B H.265 access units

Beni serves one UCP peer at a time: while this client is connected the phone app cannot be,
and the app must be off Beni's Wi-Fi for this client to get ucp:ack.
"""
import queue, socket, threading, time

from . import protocol as P


class NotConnected(RuntimeError):
    pass


def is_keyframe(au):
    """True if the access unit carries VPS/SPS/PPS or an IRAP slice."""
    i = 0
    head = au[:8192]
    for _ in range(8):
        i = head.find(b"\x00\x00\x01", i)
        if i < 0 or i + 3 >= len(head):
            return False
        t = (head[i + 3] >> 1) & 0x3F
        if t in (32, 33, 34) or 16 <= t <= 21:
            return True
        i += 3
    return False


class Beni:
    def __init__(self, host=P.ROBOT[0], port=P.ROBOT[1], bind=None, rate_hz=50, video_queue=240):
        self.addr, self.bind, self.period = (host, port), bind, 1.0 / rate_hz
        self._lock = threading.Lock()
        self._throttle = self._yaw = self._pan = self._tilt = 0.0
        self._mode = None
        self._pulse_code, self._pulse_until = 0, 0.0
        self._video = queue.Queue(maxsize=video_queue)
        self._stop = threading.Event()
        self._thread = None
        self._sock = None
        self.connected = False
        self.frames_rx = 0
        self.last_video = 0.0
        self.sent = [P.CENTER, P.CENTER, P.CENTER, P.TILT_LEVEL, P.CH4_REST]

    # --- lifecycle -------------------------------------------------------------------------
    def connect(self, timeout=6.0):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if self.bind:
            s.bind((self.bind, 0))
        s.connect(self.addr)
        s.settimeout(0.3)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            s.send(b"ucp:sync")
            try:
                if s.recv(64).startswith(b"ucp:ack"):
                    break
            except (socket.timeout, ConnectionRefusedError):
                # Refused: another peer (usually the phone app) holds Beni's single session.
                time.sleep(0.2)
        else:
            s.close()
            raise NotConnected(f"no ucp:ack from {self.addr[0]}:{self.addr[1]} "
                               "(not on Beni's Wi-Fi, or another app holds the session)")
        s.setblocking(False)
        self._sock, self.connected = s, True
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="beni-ucp", daemon=True)
        self._thread.start()
        return self

    def close(self):
        """Neutral sticks for 200 ms, then release the session."""
        self.stop()
        if self._thread:
            time.sleep(0.2)
            self._stop.set()
            self._thread.join(timeout=1)
        if self._sock:
            self._sock.close()
        self.connected = False

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.close()

    # --- commands (all values -1..1; held until changed) ------------------------------------
    def drive(self, throttle=None, yaw=None):
        """throttle +1 = full forward; yaw +1 = full right (ch1 1999), -1 = full left."""
        with self._lock:
            if throttle is not None:
                self._throttle = throttle
            if yaw is not None:
                self._yaw = yaw

    def head(self, pan=None, tilt=None):
        """pan -1..1 over ch2's range (about ±180° at the ends); tilt -1 looks down, +1 up."""
        with self._lock:
            if pan is not None:
                self._pan = pan
            if tilt is not None:
                self._tilt = tilt

    def action(self, name):
        """One of protocol.ACTIONS: jump, flip_forward, flip_back, flip_left, flip_right.
        A flip sent within ~1.5 s of a jump fires in the air (direction jump)."""
        code = P.ACTIONS[name]
        with self._lock:
            self._pulse_code, self._pulse_until = code, time.monotonic() + P.PULSE_S

    def mode(self, value):
        """Raw ch5. Captured values: 545/546/560 idle, 801/816 pilot. None keeps the captured default."""
        with self._lock:
            self._mode = value

    def stop(self):
        """All sticks and head to neutral."""
        with self._lock:
            self._throttle = self._yaw = self._pan = self._tilt = 0.0

    # --- video -----------------------------------------------------------------------------
    def frames(self, seconds=None):
        """Yield H.265 Annex-B access units, starting at the next keyframe so the output decodes.
        Ends after `seconds` if given."""
        end = None if seconds is None else time.monotonic() + seconds
        started = False
        while end is None or time.monotonic() < end:
            if not self.connected and self._video.empty():
                raise NotConnected("session ended")
            try:
                au = self._video.get(timeout=0.2)
            except queue.Empty:
                continue
            started = started or is_keyframe(au)
            if started:
                yield au

    # --- io loop ---------------------------------------------------------------------------
    def _block(self):
        with self._lock:
            code = self._pulse_code if time.monotonic() < self._pulse_until else 0
            ch = [P.axis(self._throttle), P.axis(self._yaw), P.axis(self._pan), P.tilt_bits(self._tilt)]
            mode = self._mode
        self.sent = ch + [P.CH4_REST | (code << 6)]
        return P.pack_sticks(ch[0], ch[1], ch[2], ch[3], code, mode)

    def _run(self):
        s, ack, seq, part = self._sock, 0, 0, []
        t0 = nxt = time.monotonic()
        self.last_video = t0
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                if now >= nxt:
                    # The stick frame is also the video ack; Beni stalls video without it at ~50 Hz.
                    s.send(P.stick_frame(self._block(), ack, seq, int((now - t0) * 1e6)))
                    seq += 1
                    nxt = max(nxt + self.period, now)
                try:
                    while True:
                        f = P.parse(s.recv(4096))
                        if not f or f[0] not in (P.T_VIDEO_MORE, P.T_VIDEO_LAST):
                            continue
                        ack = f[1]
                        part.append(f[2])
                        self.last_video = now
                        if f[0] == P.T_VIDEO_LAST or len(part) > 600:
                            self.frames_rx += 1
                            au = b"".join(part)
                            part = []
                            try:
                                self._video.put_nowait(au)
                            except queue.Full:  # nobody reading: keep the newest
                                try:
                                    self._video.get_nowait()
                                except queue.Empty:
                                    pass
                                self._video.put_nowait(au)
                except (BlockingIOError, InterruptedError):
                    pass
                except ConnectionRefusedError:
                    break
                if now - self.last_video > 5:
                    break  # video stalled: session lost
                time.sleep(0.002)
        finally:
            self.connected = False
