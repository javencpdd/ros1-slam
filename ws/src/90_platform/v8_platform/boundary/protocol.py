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

TELEM_MAGIC = b'V8TM'
TELEM_OK, TELEM_FAULT = 1, 2
FAULT_NONFINITE = 1
FAULT_POLICY = 2
JOINTS = 12
TELEM_HEAD = struct.Struct('<4sBBIQ3f3f4f3f3fH12f12f')
TELEM_FRAME = struct.Struct('<4sBBIQ3f3f4f3f3fH12f12fH')


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


@dataclass(frozen=True)
class TelemetryFrame:
    """Robot -> adapter state frame: pose/velocity/quat/IMU/joints and fault bits."""
    seq: int
    t_ns: int
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    qw: float
    qx: float
    qy: float
    qz: float
    gx: float
    gy: float
    gz: float
    ax: float
    ay: float
    az: float
    state: int = TELEM_OK
    fault: int = 0
    joint_pos: tuple = (0.,) * JOINTS
    joint_vel: tuple = (0.,) * JOINTS

    def encode(self):
        numbers = (self.x, self.y, self.z, self.vx, self.vy, self.vz,
                   self.qw, self.qx, self.qy, self.qz, self.gx, self.gy, self.gz,
                   self.ax, self.ay, self.az) + tuple(self.joint_pos) + tuple(self.joint_vel)
        if len(self.joint_pos) != JOINTS or len(self.joint_vel) != JOINTS:
            raise ProtocolError('telemetry needs %d joint positions/velocities' % JOINTS)
        if self.state not in (TELEM_OK, TELEM_FAULT):
            raise ProtocolError('bad telemetry state')
        if not all(math.isfinite(v) for v in numbers):
            raise ProtocolError('non-finite telemetry')
        head = TELEM_HEAD.pack(TELEM_MAGIC, VERSION, self.state, self.seq & 0xFFFFFFFF,
                               self.t_ns & 0xFFFFFFFFFFFFFFFF,
                               self.x, self.y, self.z, self.vx, self.vy, self.vz,
                               self.qw, self.qx, self.qy, self.qz,
                               self.gx, self.gy, self.gz, self.ax, self.ay, self.az,
                               self.fault & 0xFFFF,
                               *self.joint_pos, *self.joint_vel)
        return head + CRC.pack(crc16_ccitt(head))


def decode_telemetry(data):
    if len(data) != TELEM_FRAME.size:
        raise ProtocolError('telemetry size %d != %d' % (len(data), TELEM_FRAME.size))
    (magic, version, state, seq, t_ns, x, y, z, vx, vy, vz,
     qw, qx, qy, qz, gx, gy, gz, ax, ay, az, fault,
     *rest) = TELEM_FRAME.unpack(data)
    joint_pos = tuple(rest[:JOINTS])
    joint_vel = tuple(rest[JOINTS:2 * JOINTS])
    crc = rest[-1]
    if magic != TELEM_MAGIC:
        raise ProtocolError('bad telemetry magic')
    if version != VERSION:
        raise ProtocolError('unsupported version %d' % version)
    if state not in (TELEM_OK, TELEM_FAULT):
        raise ProtocolError('bad telemetry state')
    if crc != crc16_ccitt(data[:-2]):
        raise ProtocolError('telemetry crc mismatch')
    numbers = (x, y, z, vx, vy, vz, qw, qx, qy, qz, gx, gy, gz, ax, ay, az)
    if not all(math.isfinite(v) for v in numbers + joint_pos + joint_vel):
        raise ProtocolError('non-finite telemetry')
    return TelemetryFrame(seq, t_ns, x, y, z, vx, vy, vz, qw, qx, qy, qz,
                          gx, gy, gz, ax, ay, az, state, fault,
                          joint_pos, joint_vel)
