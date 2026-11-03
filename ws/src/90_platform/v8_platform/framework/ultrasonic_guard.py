"""Front ultrasonic speed-cap decision with hysteresis (pure Python, no ROS).

The robot reports two ultrasonic ranges (index 0 = front, 1 = rear). When the
front range drops below `cap_distance` the navigation speed is capped; it is
restored only after the range exceeds `clear_distance`.
"""
CAP_SPEED = 0.1
CLEAR_SPEED = 0.5


class SpeedCapDecision(object):
    def __init__(self, cap_distance_m=0.8, clear_distance_m=1.2,
                 cap_speed_mps=CAP_SPEED, clear_speed_mps=CLEAR_SPEED,
                 stop_distance_m=0.3, stop_speed_mps=0.0,
                 fail_cap=False, max_valid_m=8.0):
        self.cap_distance_m = cap_distance_m
        self.clear_distance_m = clear_distance_m
        self.cap_speed_mps = cap_speed_mps
        self.clear_speed_mps = clear_speed_mps
        self.stop_distance_m = stop_distance_m
        self.stop_speed_mps = stop_speed_mps
        self.fail_cap = fail_cap
        self.max_valid_m = max_valid_m
        self.capped = False
        self.reason = 'init'

    def update(self, distance_m, fresh=True):
        if not fresh:
            self.reason = 'stale'
            return self.cap_speed_mps if self.fail_cap else self.clear_speed_mps
        if (distance_m is None or not isinstance(distance_m, (int, float))
                or distance_m <= 0.0 or distance_m > self.max_valid_m):
            self.reason = 'invalid_range'
            return self.cap_speed_mps if self.fail_cap else self.clear_speed_mps
        if distance_m <= self.stop_distance_m:
            self.capped = True
            self.reason = 'front_stop'
            return self.stop_speed_mps
        if self.capped:
            if distance_m >= self.clear_distance_m:
                self.capped = False
        elif distance_m <= self.cap_distance_m:
            self.capped = True
        self.reason = 'front_near' if self.capped else 'clear'
        return self.cap_speed_mps if self.capped else self.clear_speed_mps
