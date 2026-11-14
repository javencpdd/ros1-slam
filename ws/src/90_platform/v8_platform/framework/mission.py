"""Mission task helpers: ordered goals and the BT mission wire format."""
from __future__ import annotations

import math


def goals(task):
    value = task.get('goals')
    if value is None:
        return [task['goal']]
    if not isinstance(value, list) or not value:
        raise ValueError('goals must be a non-empty list')
    return value


def _num(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError(name)
    if not math.isfinite(number):
        raise ValueError(name)
    return number


def goal_wire(goal, index, total):
    """Pose + per-goal policy in one flat string (BT /v8/bt/goal_mission)."""
    if not isinstance(index, int) or index < 1 or not isinstance(total, int) or total < 1:
        raise ValueError('index/total')
    x, y, z = (_num(v, 'xyz') for v in goal['xyz'])
    yaw = _num(goal.get('yaw', 0.0), 'yaw')
    parts = ['x=%.6f' % x, 'y=%.6f' % y, 'z=%.6f' % z, 'yaw=%.6f' % yaw,
             'index=%d' % index, 'total=%d' % total]
    if goal.get('dwell_s') is not None:
        parts.append('dwell=%.3f' % _num(goal['dwell_s'], 'dwell'))
    timeout = goal.get('arrival_timeout_s', goal.get('timeout_s'))
    if timeout is not None:
        parts.append('arrival=%.3f' % _num(timeout, 'arrival'))
    if goal.get('path_timeout_s') is not None:
        parts.append('path=%.3f' % _num(goal['path_timeout_s'], 'path'))
    if goal.get('retry_max') is not None:
        retry = goal['retry_max']
        if not isinstance(retry, int) or isinstance(retry, bool) or retry < 0:
            raise ValueError('retry')
        parts.append('retry=%d' % retry)
    label = goal.get('label')
    if label is not None:
        if (not isinstance(label, str) or not label or len(label) > 80
                or any(ch in label for ch in ';=\n')):
            raise ValueError('label')
        parts.append('label=' + label)
    return ';'.join(parts)
