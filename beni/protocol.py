"""Beni UCP wire format (cleartext parts only). See protocol.md for how each field was mapped.

UDP 9007 on the robot. Handshake: send b"ucp:sync", robot answers b"ucp:ack".
Header (LE): magic 0x51 | type u8 | ack u16 | seq u16 | len u16 | timestamp u32 (us)
  type 0x10: stick frame (we send 31 B at 50 Hz)   type 0x07/0x05: video fragment (more / last)
Stick body: 9 bytes holding six 11-bit channels, then the last video seq (u16) as the video ack.
"""
import struct

ROBOT = ("192.168.7.1", 9007)
MAGIC = 0x51
T_STICK, T_VIDEO_MORE, T_VIDEO_LAST = 0x10, 0x07, 0x05
HEADER = struct.Struct("<BBHHHI")

# Captured neutral body (ch5 mode and the 6 spare bits above it come from here).
TEMPLATE = bytes.fromhex("e8431ffabe0f021805")
CENTER, LO, HI = 1000, 1, 1999

# ch4 rests at 32; an action is a ~190 ms pulse of 32 | code << 6.
CH4_REST = 32
ACTIONS = {"jump": 1, "flip_forward": 2, "flip_back": 4, "flip_left": 6, "flip_right": 8}
PULSE_S = 0.19

# ch3: head tilt in the low 6 bits; 0 looks down, 31 level, 62 looks up. Upper 5 bits constant.
TILT_LEVEL, TILT_MAX = 31, 62


def axis(v):
    """-1..1 -> 1..1999 (center 1000)."""
    v = max(-1.0, min(1.0, float(v)))
    return CENTER + int(round(v * (HI - CENTER)))


def tilt_bits(v):
    """-1 (down) .. +1 (up) -> 0..62."""
    v = max(-1.0, min(1.0, float(v)))
    return max(0, min(TILT_MAX, TILT_LEVEL + int(round(v * TILT_LEVEL))))


def pack_sticks(ch0=CENTER, ch1=CENTER, ch2=CENTER, tilt=TILT_LEVEL, code=0, mode=None):
    """Build the 9-byte channel block. Channels are raw 11-bit values; mode=None keeps the captured ch5."""
    v = int.from_bytes(TEMPLATE, "little")
    v &= ~((1 << 22) - 1) & ~(0x7FF << 22) & ~(0x3F << 33) & ~(0x7FF << 44)
    v |= (ch0 & 0x7FF) | ((ch1 & 0x7FF) << 11) | ((ch2 & 0x7FF) << 22) | ((tilt & 0x3F) << 33)
    v |= ((CH4_REST | (code << 6)) & 0x7FF) << 44
    if mode is not None:
        v = (v & ~(0x7FF << 55)) | ((mode & 0x7FF) << 55)
    return v.to_bytes(9, "little")


def unpack_sticks(block):
    """9-byte channel block -> [ch0..ch5]."""
    v = int.from_bytes(block[:9], "little")
    return [(v >> (11 * i)) & 0x7FF for i in range(6)]


def stick_frame(block, video_ack=0, seq=0, ts_us=0):
    body = block + struct.pack("<H", video_ack & 0xFFFF) + b"\0" * 8
    return HEADER.pack(MAGIC, T_STICK, video_ack & 0xFFFF, seq & 0xFFFF, HEADER.size + len(body), ts_us & 0xFFFFFFFF) + body


def parse(datagram):
    """-> (type, seq, payload) or None for anything that is not a UCP frame."""
    if len(datagram) < HEADER.size or datagram[0] != MAGIC:
        return None
    _, typ, _, seq, _, _ = HEADER.unpack_from(datagram)
    return typ, seq, datagram[HEADER.size:]
