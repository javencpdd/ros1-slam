"""Flat/step gait decision with hysteresis (pure Python, no ROS)."""


class GaitDecision(object):
    """Flat <-> step gait with hysteresis; refuses steps above max_step_m."""

    def __init__(self, enter_m=1.0, exit_m=1.5, min_step_m=0.05, max_step_m=0.15,
                 flat_gait='flat_mid', step_gait='high_step'):
        self.enter_m = enter_m
        self.exit_m = exit_m
        self.min_step_m = min_step_m
        self.max_step_m = max_step_m
        self.flat_gait = flat_gait
        self.step_gait = step_gait
        self.climbing = False
        self.current = flat_gait
        self.refusal = ''

    def update(self, result):
        ahead = bool(result.get('ahead'))
        step = float(result.get('step_m') or 0.0)
        distance = float(result.get('distance_m') or 0.0)
        self.refusal = ''
        if ahead and step > self.max_step_m:
            self.refusal = 'step_too_high'
            if self.climbing:
                self.climbing = False
                self.current = self.flat_gait
            return self.current
        if ahead and distance <= self.enter_m and step >= self.min_step_m:
            if not self.climbing:
                self.climbing = True
                self.current = self.step_gait
            return self.current
        if self.climbing and (not ahead or distance > self.exit_m):
            self.climbing = False
            self.current = self.flat_gait
        return self.current
