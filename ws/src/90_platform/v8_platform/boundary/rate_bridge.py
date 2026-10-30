"""Planner-rate command latch -> fixed-rate boundary command.

The planner side may publish at 10--50 Hz; the boundary streams at 50--500 Hz.
This bridge holds the latest intent, enforces the envelope, slews the output
and zeroes it when the planner goes silent. Pure stdlib, no ROS.
"""
import math
from dataclasses import dataclass


@dataclass
class Limits:
    max_speed: float = 0.8
    max_wz: float = 1.0
    max_accel: float = 1.5
    max_yaw_accel: float = 3.0
    command_timeout_s: float = 0.25


def _slew(current, target, dt, accel):
    if dt <= 0:
        return current
    return min(current + accel * dt, max(current - accel * dt, target))


class CommandBridge:
    def __init__(self, limits=None):
        self.limits = limits or Limits()
        self._cmd = (0., 0., 0.)
        self._at = None
        self._out = (0., 0., 0.)
        self._last = None
        self.mode = 'init'
        self.stats = {'updates': 0, 'clamped': 0, 'holds': 0}

    def update(self, vx, vy, wz, t):
        values = (float(vx), float(vy), float(wz))
        if not all(math.isfinite(v) for v in values):
            raise ValueError('non-finite command')
        lim = self.limits
        speed = math.hypot(values[0], values[1])
        if speed > lim.max_speed:
            scale = lim.max_speed / speed
            values = (values[0] * scale, values[1] * scale, values[2])
            self.stats['clamped'] += 1
        if abs(values[2]) > lim.max_wz:
            values = (values[0], values[1], math.copysign(lim.max_wz, values[2]))
            self.stats['clamped'] += 1
        self._cmd, self._at = values, t
        self.stats['updates'] += 1

    def age(self, t):
        return None if self._at is None else max(0., t - self._at)

    def command(self, t):
        lim = self.limits
        if self._at is None:
            self.mode = 'init'
            target = (0., 0., 0.)
        elif t - self._at > lim.command_timeout_s:
            self.mode = 'hold'
            self.stats['holds'] += 1
            target = (0., 0., 0.)
        else:
            self.mode = 'active'
            target = self._cmd
        dt = 0. if self._last is None else max(0., t - self._last)
        self._last = t
        self._out = (_slew(self._out[0], target[0], dt, lim.max_accel),
                     _slew(self._out[1], target[1], dt, lim.max_accel),
                     _slew(self._out[2], target[2], dt, lim.max_yaw_accel))
        return self._out
