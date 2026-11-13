"""The real Beni head, shared by the sim (tools/sim) and the real runners (tools/nav)."""
import math


class HeadModel:
    """ch2 is a head RATE (the head stays where it is at ch2 = centre), turning up to the policy's head range.
    The sim and the real runners both step this with the commands sent, so both know where the head is.
    RATE measured on the real sweep (2026-11-13): stick 0.3 turned the head 150 + 270 + ~100 deg in 7.7 s."""
    RATE = math.radians(225.0)   # rad/s at full stick; + = left (ch2 low)
    LIM = 5.3                    # rad, head yaw range in the policy's training states

    def __init__(self, q=0.0):
        self.q = q          # None = unknown: nothing measures the real head; only a posture command centres it

    def step(self, u, dt):
        if self.q is None:
            return None
        self.q = max(-self.LIM, min(self.LIM, self.q + u * self.RATE * dt))
        return self.q
