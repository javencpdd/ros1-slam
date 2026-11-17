#!/usr/bin/env python3
"""local_planner 的信号与判决纯函数（自 ros_node.py 拆出，2026-11-18 计划 M9）。

本模块只做「给定读数算出判决」：无状态、不碰 IO、不 import ROS。
LocalControl 与 façade 都从这里取信号；它是依赖图的叶子。
"""
import json
import math
import sys
from pathlib import Path

_sys_root = Path(__file__).resolve().parents[1]
if str(_sys_root) not in sys.path:
    sys.path.insert(0, str(_sys_root))
from common.pose_safety import finite_number  # noqa: E402



NON_LATCHING = {'E_LOCAL_WORLD_UPDATED', 'E_LOCAL_EXPIRED', 'E_LOCAL_NO_TRAJECTORY',
                'E_LOCAL_NOT_READY', 'E_LOCAL_INPUT', 'E_LOCAL_CLOCK'}
def freshness(now, pose_received, obstacles_received, localization_received,
              localization_ok, max_age_s):
    if not localization_ok or now - localization_received >= max_age_s:
        return 'E_LOCAL_LOCALIZATION'
    if now - pose_received >= max_age_s:
        return 'E_LOCAL_POSE_STALE'
    if now - obstacles_received >= max_age_s:
        return 'E_LOCAL_OBSTACLES_STALE'
    return None

def feedback_at(pose, observed_at, now, commands):
    """Predict to controller time from timestamped feedback and issued commands.

    This changes the controller estimate only. Received/raw poses are never
    moved or snapped. Piecewise integration avoids braking against an old pose
    while the previous nonzero command is still moving the experimental carrier.
    """
    if observed_at <= 0:
        return tuple(pose)
    if observed_at > now:
        raise ValueError('pose timestamp is in the future')
    position, cursor, velocity = tuple(pose), observed_at, (0., 0., 0.)
    for stamp, issued in commands:
        if stamp <= observed_at:
            velocity = tuple(issued)
        elif stamp <= now:
            position = tuple(position[i] + (stamp - cursor) * velocity[i] for i in range(3))
            cursor, velocity = stamp, tuple(issued)
        else:
            break
    return tuple(position[i] + (now - cursor) * velocity[i] for i in range(3))

def feedback_uncertainty(observed_at, now, commands, latency_bound_s):
    """Distance bound from issued-vs-applied timing and one pending command."""
    velocity, variation = (0., 0., 0.), 0.
    for stamp, issued in commands:
        if stamp > now:
            break
        if stamp >= observed_at:
            variation += math.sqrt(sum((issued[i] - velocity[i]) ** 2 for i in range(3)))
        velocity = tuple(issued)
    return latency_bound_s * (variation + math.sqrt(sum(v * v for v in velocity)))

def stop_reason(fresh, latched, latched_reason, result, result_at, wall_now,
                wall_watchdog_s, sim_now, generation, result_generation, last_error):
    """Ordered fail-closed policy; plan validity is sim time, wall is a fallback.

    World updates precede plan expiry so a sensed change is not relabelled as
    an expired plan. A terminal reason latches until a new goal arrives.
    """
    if fresh is not None:
        return fresh
    if latched:
        return latched_reason or 'E_LOCAL_STOP_LATCHED'
    if result is None:
        return last_error if last_error and last_error != 'ok' else 'E_LOCAL_NO_TRAJECTORY'
    if generation != result_generation:
        return 'E_LOCAL_WORLD_UPDATED'
    if not isinstance(result, dict) or not finite_number(result.get('valid_until')):
        return 'E_LOCAL_INPUT'
    valid_until = result.get('valid_until')
    if sim_now and sim_now > 0 and valid_until is not None and sim_now >= valid_until:
        return 'E_LOCAL_EXPIRED'
    if wall_now - result_at >= wall_watchdog_s:
        return 'E_LOCAL_WALL_EXPIRED'
    return None

def map_from_message(text):
    """Accept a raw support_map or a delta_map_node snapshot envelope."""
    value = json.loads(text)
    if isinstance(value, dict) and 'map' in value:
        value = value['map']
    if not isinstance(value, dict):
        raise ValueError('map message must be an object')
    return value

def _local_pose_reason(reason):
    if reason.startswith('E_LOCAL_'):
        return reason
    if reason in ('E_POSE_STALE', 'E_POSE_NO_PROGRESS'):
        return 'E_LOCAL_POSE_STALE'
    if reason == 'E_POSE_ORIENTATION':
        return 'E_LOCAL_ORIENTATION'
    if reason == 'E_POSE_FRAME':
        return 'E_LOCAL_FRAME'
    if reason == 'E_POSE_POSITION':
        return 'E_LOCAL_POSE_INVALID'
    if reason == 'E_POSE_MISSING':
        return 'E_LOCAL_NOT_READY'
    return 'E_LOCAL_CLOCK'
