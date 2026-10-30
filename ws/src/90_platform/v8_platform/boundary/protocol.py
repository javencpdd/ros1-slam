"""v8 robot boundary -- draft v0 command frame.

Pure stdlib. Sim/host loopback only: the real-robot link (vendor QNX 43893
via message_transformer/ros2qnx) stays untouched until an L2 authorization.
Frame: magic(4) ver(1) mode(1) seq(4) t_ns(8) vx/vy/wz(12) flags(1) crc16(2).
"""
import math
import struct
from dataclasses import dataclass

MAGIC = b'V8BD'
VERSION = 1
HEADER = struct.Struct('<4sBBIQfffB')
FRAME = struct.Struct('<4sBBIQfffBH')
CRC = struct.Struct('<H')

MODE_ACTIVE, MODE_HOLD = 1, 2
FLAG_PLANNER_OK = 1


class ProtocolError(ValueError):
    pass


def crc16_ccitt(data, crc=0xFFFF):
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class CommandFrame:
    seq: int
    t_ns: int
    vx: float
    vy: float
    wz: float
    mode: int = MODE_ACTIVE
    flags: int = 0

    def encode(self):
        if not all(math.isfinite(v) for v in (self.vx, self.vy, self.wz)):
            raise ProtocolError('non-finite command')
        head = HEADER.pack(MAGIC, VERSION, self.mode, self.seq & 0xFFFFFFFF,
                           self.t_ns & 0xFFFFFFFFFFFFFFFF, self.vx, self.vy, self.wz,
                           self.flags & 0xFF)
        return head + CRC.pack(crc16_ccitt(head))


def decode(data):
    if len(data) != FRAME.size:
        raise ProtocolError('frame size %d != %d' % (len(data), FRAME.size))
    magic, version, mode, seq, t_ns, vx, vy, wz, flags, crc = FRAME.unpack(data)
    if magic != MAGIC:
        raise ProtocolError('bad magic')
    if version != VERSION:
        raise ProtocolError('unsupported version %d' % version)
    if crc != crc16_ccitt(data[:-2]):
        raise ProtocolError('crc mismatch')
    if not all(math.isfinite(v) for v in (vx, vy, wz)):
        raise ProtocolError('non-finite command')
    return CommandFrame(seq, t_ns, vx, vy, wz, mode, flags)
