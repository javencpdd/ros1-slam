"""Velocity clamp shared by the safety mux (no ROS import, unit-testable)."""

import math


def clamp(vx, vy, vz, wx, wy, wz, max_linear=None, max_angular=None):
    """Scale the 3-vectors so |linear| <= max_linear and |angular| <= max_angular.

    None means no bound on that vector. Direction and the linear/angular ratio
    are preserved; a zero vector stays zero.
    """
    out = [vx, vy, vz, wx, wy, wz]
    if max_linear is not None:
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        if speed > max_linear and speed > 0.0:
            scale = max_linear / speed
            out[0], out[1], out[2] = vx * scale, vy * scale, vz * scale
    if max_angular is not None:
        spin = math.sqrt(wx * wx + wy * wy + wz * wz)
        if spin > max_angular and spin > 0.0:
            scale = max_angular / spin
            out[3], out[4], out[5] = wx * scale, wy * scale, wz * scale
    return out
