"""Stop-before-switch handshake for /v8/chassis/gait.

Pure state, no ROS. The controller feeds two observations:

- ``desired``: the gait the navigation stack currently asks for (gait_guard);
- ``active``: the gait the chassis bridge reports as active (ros2qnx status).

A gait change must happen while the robot is stopped, because the chassis
bridge refuses switches above its speed gate and the gait node publishes on
change only (the bridge is not guaranteed to retry). The controller therefore
holds zero velocity from the moment the desired gait changes until the chasing
bridge reports the new gait as active for ``settle_s`` with a fresh status.
Timeout and stale status keep the hold (fail-closed); they never release it.
"""

import math

IDLE = 'gait_switch_idle'
WAITING = 'gait_switch_waiting'
STATUS_STALE = 'gait_switch_status_stale'
TIMEOUT = 'gait_switch_timeout'
CLOCK = 'gait_switch_clock'
DESIRED_LOST = 'gait_switch_desired_lost'
DONE = 'gait_switch_done'


def _finite(name, value, positive):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0.
            or (positive and value == 0.)):
        raise ValueError('%s must be %s finite number, got %r'
                         % (name, 'a positive' if positive else 'a non-negative', value))
    return float(value)


class GaitSwitchHold(object):
    """Latch a zero-velocity hold around each gait change.

    ``baseline`` is the desired gait that needs no hold. The first observed
    desired gait becomes the baseline: startup is not a switch. A later change
    opens a hold; reverting to the baseline before confirmation cancels it.
    """

    def __init__(self, settle_s=0.5, timeout_s=5.0, status_stale_s=1.5):
        self.settle_s = _finite('settle_s', settle_s, positive=False)
        self.timeout_s = _finite('timeout_s', timeout_s, positive=True)
        self.status_stale_s = _finite('status_stale_s', status_stale_s, positive=True)
        self.baseline = None
        self.target = None
        self.hold_since = None
        self.matched_since = None
        self.timed_out = False

    def _reset(self):
        self.target = None
        self.hold_since = None
        self.matched_since = None
        self.timed_out = False

    def _idle(self, active, fresh, reason=IDLE):
        return dict(hold=False, target=None, active=active, reason=reason,
                    held_s=0.0, timed_out=False, status_fresh=fresh)

    def update(self, desired, active, status_at, wall_now):
        if (isinstance(wall_now, bool) or not isinstance(wall_now, (int, float))
                or not math.isfinite(wall_now)):
            if self.target is None:
                return self._idle(active, False)
            return dict(hold=True, target=self.target, active=active, reason=CLOCK,
                        held_s=0.0, timed_out=self.timed_out, status_fresh=False)
        fresh = (isinstance(status_at, (int, float))
                 and not isinstance(status_at, bool)
                 and math.isfinite(status_at)
                 and 0.0 <= wall_now - status_at <= self.status_stale_s)
        if not isinstance(desired, str) or not desired:
            if self.target is None:
                return self._idle(active, fresh)
            # The gait source disappeared while a switch was pending.
            return dict(hold=True, target=self.target, active=active,
                        reason=DESIRED_LOST,
                        held_s=max(0.0, wall_now - self.hold_since)
                        if self.hold_since is not None else 0.0,
                        timed_out=self.timed_out, status_fresh=fresh)
        if self.baseline is None:
            self.baseline = desired
            return self._idle(active, fresh)
        if desired == self.baseline:
            self._reset()
            return self._idle(active, fresh)
        if self.target != desired:
            self.target = desired
            self.hold_since = wall_now
            self.matched_since = None
            self.timed_out = False
        matched = fresh and active == desired
        if matched:
            if self.matched_since is None:
                self.matched_since = wall_now
            if wall_now - self.matched_since >= self.settle_s:
                self.baseline = desired
                self._reset()
                return dict(hold=False, target=None, active=active, reason=DONE,
                            held_s=0.0, timed_out=False, status_fresh=True)
        else:
            self.matched_since = None
        held_s = (0.0 if self.hold_since is None
                  else max(0.0, wall_now - self.hold_since))
        if held_s > self.timeout_s:
            self.timed_out = True
        reason = (TIMEOUT if self.timed_out
                  else STATUS_STALE if not fresh else WAITING)
        return dict(hold=True, target=desired, active=active, reason=reason,
                    held_s=held_s, timed_out=self.timed_out, status_fresh=fresh)
