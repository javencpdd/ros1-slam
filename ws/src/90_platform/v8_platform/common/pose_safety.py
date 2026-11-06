"""Pose geometry and measurement/monotonic freshness, independent of ROS.

The planar command contract re-expresses velocity at the same base origin in
its yaw plane. It does not implement a 6D spatial twist or a height offset.
"""
import math


class PoseError(ValueError):
    def __init__(self, code):
        super().__init__(code)
        self.code = code


def finite_number(value):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(float(value))
    except (OverflowError, TypeError, ValueError):
        return False


def finite_vector(values, size, code='E_POSE_POSITION'):
    if (not isinstance(values, (list, tuple)) or len(values) != size
            or not all(isinstance(value, (int, float)) and not isinstance(value, bool)
                       for value in values)):
        raise PoseError(code)
    try:
        converted = tuple(float(value) for value in values)
    except (OverflowError, TypeError, ValueError) as error:
        raise PoseError(code) from error
    if not all(math.isfinite(value) for value in converted):
        raise PoseError(code)
    return converted


def normalized_quaternion(xyzw):
    x, y, z, w = finite_vector(xyzw, 4, 'E_POSE_ORIENTATION')
    norm = math.hypot(x, y, z, w)
    if not math.isfinite(norm) or norm == 0.:
        raise PoseError('E_POSE_ORIENTATION')
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return (x, y, z, w)


def validated_yaw(xyzw):
    x, y, z, w = normalized_quaternion(xyzw)
    return math.atan2(2. * (w * z + x * y), 1. - 2. * (y * y + z * z))


def map_to_body_planar(velocity, xyzw):
    vx, vy, vz = finite_vector(velocity, 3, 'E_LOCAL_VELOCITY')
    yaw = validated_yaw(xyzw)
    # Arithmetic zero tolerance, not a physical step/slope capability bound.
    if abs(vz) > 1e-6:
        raise PoseError('E_LOCAL_VERTICAL_UNSUPPORTED')
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return finite_vector((cosine * vx + sine * vy, -sine * vx + cosine * vy, 0.),
                         3, 'E_LOCAL_VELOCITY')


class PoseFreshness:
    """A valid measurement advances source time; duplicate delivery is no lease.

    Both measurement age in ROS time and source progress age in monotonic time
    must be strictly less than max_age_s. Faults persist until a fresh advancing
    observation, including rejected samples in that progress sequence. A ROS-clock rollback
    invalidates one cycle and opens a new epoch; the last cached source samples
    stay quarantined until a distinct valid measurement proves new progress.
    """
    def __init__(self, max_age_s):
        if not finite_number(max_age_s) or max_age_s <= 0.:
            raise ValueError('max_age_s must be positive and finite')
        self.max_age_s = float(max_age_s)
        self.source_stamp = None
        self.progress_at = None
        self.observed_stamp = None
        self.epoch_cached_stamps = frozenset()
        self.last_ros = None
        self.last_wall = None
        self.reason = 'E_POSE_MISSING'
        self.epoch = 0

    def invalidate(self, reason):
        self.reason = reason

    def _clock(self, ros_now, wall_now):
        if (not finite_number(ros_now) or ros_now <= 0.
                or not finite_number(wall_now)):
            self.invalidate('E_POSE_CLOCK')
            return False
        if self.last_wall is not None and wall_now < self.last_wall:
            self.invalidate('E_POSE_WALL_BACKWARD')
            return False
        self.last_wall = wall_now
        if self.last_ros is not None and ros_now < self.last_ros:
            self.last_ros = ros_now
            self.epoch_cached_stamps = self.epoch_cached_stamps.union(
                stamp for stamp in (self.source_stamp, self.observed_stamp) if stamp is not None)
            self.source_stamp = self.progress_at = self.observed_stamp = None
            self.epoch += 1
            self.invalidate('E_POSE_CLOCK_BACKWARD')
            return False
        self.last_ros = ros_now
        return True

    def observe(self, source_stamp, ros_now, wall_now, rejection=None):
        """Observe every sample, including rejected ones; duplicates cannot heal.

        ROS epochs reset the observation sequence. Within one epoch, a fault
        requires newly observed source progress, not merely wall/ROS catch-up.
        """
        clock_ok = self._clock(ros_now, wall_now)
        previous_observed = self.observed_stamp
        valid_stamp = finite_number(source_stamp) and source_stamp > 0.
        if valid_stamp and source_stamp not in self.epoch_cached_stamps:
            self.observed_stamp = float(source_stamp)
        if not clock_ok:
            return self._status(ros_now, wall_now), False
        if rejection is not None:
            self.invalidate(rejection)
            return self._status(ros_now, wall_now), False
        if not valid_stamp:
            self.invalidate('E_POSE_STAMP')
        elif source_stamp in self.epoch_cached_stamps:
            self.invalidate('E_POSE_EPOCH_REPLAY')
        elif source_stamp > ros_now:
            self.invalidate('E_POSE_FUTURE')
        elif ros_now - source_stamp >= self.max_age_s:
            self.invalidate('E_POSE_STALE')
        elif ((self.source_stamp is not None and source_stamp < self.source_stamp)
              or (previous_observed is not None and source_stamp < previous_observed)):
            self.invalidate('E_POSE_STAMP_BACKWARD')
        elif (self.reason is not None and previous_observed is not None
              and source_stamp == previous_observed):
            pass  # The same rejected sample cannot become a new measurement.
        elif self.source_stamp is None or source_stamp > self.source_stamp:
            self.source_stamp = float(source_stamp)
            self.progress_at = float(wall_now)
            self.reason = None
            self.epoch_cached_stamps = frozenset()
            return self._status(ros_now, wall_now), True
        return self._status(ros_now, wall_now), False

    def check(self, ros_now, wall_now):
        self._clock(ros_now, wall_now)
        return self._status(ros_now, wall_now)

    def _status(self, ros_now, wall_now):
        source_age = (ros_now - self.source_stamp
                      if finite_number(ros_now) and self.source_stamp is not None else None)
        progress_age = (wall_now - self.progress_at
                        if finite_number(wall_now) and self.progress_at is not None else None)
        if self.reason is None:
            if source_age is None or progress_age is None:
                self.invalidate('E_POSE_MISSING')
            elif source_age < 0.:
                self.invalidate('E_POSE_FUTURE')
            elif source_age >= self.max_age_s:
                self.invalidate('E_POSE_STALE')
            elif progress_age >= self.max_age_s:
                self.invalidate('E_POSE_NO_PROGRESS')
        return {'ok': self.reason is None, 'reason': self.reason or 'ok',
                'source_stamp': self.source_stamp, 'observed_source_stamp': self.observed_stamp,
                'source_age_s': source_age,
                'progress_age_s': progress_age, 'epoch': self.epoch}
