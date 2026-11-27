"""T1/T2/T3 rigid-transform contract shared by localization and tests.

T1 maps the Faster-LIO world (`camera_init`) into `map`.  The LIO world is the
first body frame, not a gravity-levelled frame, so T1 carries the full initial
rotation (the rslidar pitch in this URDF), not just yaw.

T2 maps LIO `body` (the rslidar/IMU frame) to `base_link`.  It is the inverse of
the URDF base->rslidar extrinsic, including rotation.  Pure translation is only
valid when the extrinsic rotation is identity.

T3 maps the base reference to the planning contact point as a world-z offset.
"""
import math


def normalize_quat(q):
    x, y, z, w = (float(v) for v in q)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return normalize_quat((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def quat_mul(left, right):
    ax, ay, az, aw = normalize_quat(left)
    bx, by, bz, bw = normalize_quat(right)
    return normalize_quat((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def quat_rotate(q, vector):
    x, y, z, w = normalize_quat(q)
    vx, vy, vz = (float(v) for v in vector)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz)) * vx + 2.0 * (xy - wz) * vy + 2.0 * (xz + wy) * vz,
        2.0 * (xy + wz) * vx + (1.0 - 2.0 * (xx + zz)) * vy + 2.0 * (yz - wx) * vz,
        2.0 * (xz - wy) * vx + 2.0 * (yz + wx) * vy + (1.0 - 2.0 * (xx + yy)) * vz,
    )


def rigid(xyz, rpy):
    q = quat_from_rpy(*rpy)
    return (float(xyz[0]), float(xyz[1]), float(xyz[2])) + q


def compose(left, right):
    lx, ly, lz = left[:3]
    lq = left[3:]
    rx, ry, rz = right[:3]
    rq = right[3:]
    dx, dy, dz = quat_rotate(lq, (rx, ry, rz))
    return (lx + dx, ly + dy, lz + dz) + quat_mul(lq, rq)


def inverse(pose):
    x, y, z = pose[:3]
    q = normalize_quat(pose[3:])
    qi = (-q[0], -q[1], -q[2], q[3])
    dx, dy, dz = quat_rotate(qi, (x, y, z))
    return (-dx, -dy, -dz) + qi


def invert_extrinsic(xyz, rpy):
    """Return the inverse rigid transform as (xyz, rpy), preserving rpy input semantics."""
    inv = inverse(rigid(xyz, rpy))
    qx, qy, qz, qw = inv[3:]
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return inv[:3], (roll, pitch, yaw)


def lio_pose_to_base(lio_pose, origin_xyz, origin_rpy, body_to_base_xyz, body_to_base_rpy):
    t1 = rigid(origin_xyz, origin_rpy)
    t2 = rigid(body_to_base_xyz, body_to_base_rpy)
    return compose(compose(t1, lio_pose), t2)


def contact_pose(base_pose, contact_dz):
    """T3 is a world-z offset from the base reference, not a body-frame translation."""
    return (base_pose[0], base_pose[1], base_pose[2] + float(contact_dz)) + tuple(base_pose[3:])


def pose_from_lio(pose, origin_xyz, origin_rpy, body_to_base_xyz, body_to_base_rpy):
    return lio_pose_to_base(pose, origin_xyz, origin_rpy, body_to_base_xyz, body_to_base_rpy)


def rpy_from_quat(q):
    x, y, z, w = normalize_quat(q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw