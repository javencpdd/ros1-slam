"""Pose-source handover logic (no ROS): seamless when sources agree, safe reject otherwise."""
import math


class PoseMuxError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


def _xyz(value):
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or not all(isinstance(item, (int, float)) and not isinstance(item, bool)
                       and math.isfinite(float(item)) for item in value)):
        raise PoseMuxError('E_POSE_MUX_INPUT', 'pose must be finite xyz')
    return tuple(float(item) for item in value)


class PoseMux:
    def __init__(self, max_gap_m=0.1, max_age_s=0.5):
        if max_gap_m <= 0 or max_age_s <= 0:
            raise PoseMuxError('E_POSE_MUX_INPUT', 'thresholds must be positive')
        self.max_gap_m = float(max_gap_m)
        self.max_age_s = float(max_age_s)
        self.sources = {}
        self.active = None
        self.last_output = None

    def update(self, source, pose, now):
        if source not in ('a', 'b'):
            raise PoseMuxError('E_POSE_MUX_INPUT', 'source must be a or b')
        self.sources[source] = {'pose': _xyz(pose), 'at': float(now)}
        if self.active is None:
            self.active = source

    def _fresh(self, source, now):
        item = self.sources.get(source)
        return item is not None and now - item['at'] <= self.max_age_s

    def select(self, target, now):
        if target not in ('a', 'b'):
            return {'ok': False, 'active': self.active, 'reason': 'E_POSE_MUX_SOURCE'}
        if not self._fresh(target, now):
            return {'ok': False, 'active': self.active, 'reason': 'E_POSE_MUX_STALE'}
        pose = self.sources[target]['pose']
        if self.last_output is not None:
            gap = math.sqrt(sum((pose[i] - self.last_output[i]) ** 2 for i in range(3)))
            if gap > self.max_gap_m:
                return {'ok': False, 'active': self.active, 'reason': 'E_POSE_MUX_JUMP',
                        'gap_m': round(gap, 6)}
        self.active = target
        return {'ok': True, 'active': self.active, 'reason': None}

    def output(self, now):
        if self.active is None:
            return {'ok': False, 'reason': 'E_POSE_MUX_NOT_READY'}
        if not self._fresh(self.active, now):
            return {'ok': False, 'reason': 'E_POSE_MUX_STALE'}
        pose = self.sources[self.active]['pose']
        if self.last_output is not None:
            gap = math.sqrt(sum((pose[i] - self.last_output[i]) ** 2 for i in range(3)))
            if gap > self.max_gap_m:
                return {'ok': False, 'reason': 'E_POSE_MUX_JUMP',
                        'gap_m': round(gap, 6)}
        self.last_output = pose
        return {'ok': True, 'pose': pose, 'source': self.active, 'reason': None}
