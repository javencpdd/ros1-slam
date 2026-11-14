"""Scene-specific observation checks, separate from the planner and old gates."""
import bisect
import math

import scene


class ValidationError(ValueError):
    pass


def require(value, message):
    if not value:
        raise ValidationError(message)


def finite(value):
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return True
    if isinstance(value, (int, float)):
        return math.isfinite(value)
    if isinstance(value, list):
        return all(finite(item) for item in value)
    if isinstance(value, dict):
        return all(isinstance(k, str) and finite(v) for k, v in value.items())
    return False


def xyz(value):
    require(isinstance(value, (list, tuple)) and len(value) >= 3, 'missing xyz')
    require(all(type(v) in (int, float) and math.isfinite(v) for v in value[:3]), 'invalid xyz')
    return tuple(value[:3])


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a[:3], b[:3])))


def intersects(a, b, lo, hi):
    """Strict-volume intersection of the complete segment; tangent contact allowed."""
    near, far = 0., 1.
    for i in range(3):
        lower, upper = lo[i] + 1e-7, hi[i] - 1e-7
        delta = b[i] - a[i]
        if abs(delta) < 1e-12:
            if not lower <= a[i] <= upper:
                return False
        else:
            p, q = sorted(((lower - a[i]) / delta, (upper - a[i]) / delta))
            near, far = max(near, p), min(far, q)
            if near > far:
                return False
    return near <= far


def segments(points):
    if len(points) == 1:
        return [(points[0], points[0])]
    return list(zip(points, points[1:]))


def collision_free(points, geometry):
    """Check actual carrier visual sweep AND declared planner body vs obstacles.

    This checks geometry only. The carrier has no collision body or feet.
    Support transfers remain the current kinematic step approximation.
    """
    half = [.225, .15, .1]
    supports = {'ground_floor', 'upper_floor'} | {'step%02d' % i for i in range(1, 11)}
    for a, b in segments(points):
        for box in geometry['boxes']:
            lo = [box['min'][i] - half[i] for i in range(3)]
            hi = [box['max'][i] + half[i] for i in range(3)]
            visual_a, visual_b = list(a[:3]), list(b[:3])
            visual_a[2] += .30; visual_b[2] += .30
            require(not intersects(visual_a, visual_b, lo, hi), 'carrier visual intersects ' + box['name'])
            if box['name'] not in supports:
                pad = scene.RADIUS + scene.CLEARANCE
                lo = [box['min'][0] - pad, box['min'][1] - pad, box['min'][2] - scene.HEIGHT]
                hi = list(box['max'])
                hi[0] += pad; hi[1] += pad
                require(not intersects(a, b, lo, hi), 'planning envelope intersects ' + box['name'])


def level_visits(points):
    return {i for i in range(11)
            if any(abs(p[2] - .16 * i) <= .045 for p in points)}


def validate_trace(value, case, app_result, interactive=False):
    require(case in scene.CASES, 'unknown scene case')
    require(isinstance(value, dict) and value.get('schema') == 'navigation_app_trace/v1', 'trace schema')
    require(value.get('capture_ok') is True and value.get('overflow') is False and value.get('errors') == [],
            'capture failed/overflow/error')
    require(finite(value), 'trace has nonfinite/unsupported data')
    meta = value.get('metadata', {})
    require(meta.get('backend') == 'gazebo' and meta.get('execution_mode') == 'simulation', 'wrong execution domain')
    require(meta.get('model_name') == 'app_carrier', 'wrong carrier')
    map_value = scene.scene_map(case == 'blocked')
    expected_sha = __import__('hashlib').sha256(scene.json_bytes(map_value)).hexdigest()
    require(meta.get('map_sha256') == expected_sha, 'trace map does not match declared scene')
    require(not app_result.get('error') and not app_result.get('stop_error') and not app_result.get('trace_error'),
            'runtime/cleanup/capture error')
    require(app_result.get('checks', {}).get('assembly_ready') is True, 'assembly was not ready')
    events = value.get('events')
    require(isinstance(events, list) and events, 'empty events')
    grouped = {}
    for e in events:
        require(isinstance(e, dict) and set(e) == {'kind', 'payload', 'ros_time', 'wall_monotonic'}, 'event structure')
        require(type(e['wall_monotonic']) in (int, float) and type(e['ros_time']) in (int, float), 'event clocks')
        require(isinstance(e['payload'], dict), 'event payload')
        grouped.setdefault(e['kind'], []).append(e)
    for kind in ('pose', 'command', 'gazebo_model', 'global_status'):
        require(grouped.get(kind), 'missing ' + kind)
    poses = sorted(grouped['pose'], key=lambda e: e['wall_monotonic'])
    commands = sorted(grouped['command'], key=lambda e: e['wall_monotonic'])
    models = [e for e in grouped['gazebo_model'] if e['payload'].get('present') is True]
    models.sort(key=lambda e: e['wall_monotonic'])
    require(len(poses) >= 20 and len(models) >= 20 and len(commands) >= 5, 'insufficient process samples')
    for e in poses:
        require(e['payload'].get('frame') == 'map', 'pose frame mismatch')
        require(type(e['payload'].get('stamp')) in (int, float) and
                math.isfinite(e['payload']['stamp']) and e['payload']['stamp'] >= 0,
                'pose source stamp must be finite real seconds')
        xyz(e['payload'].get('xyz'))
    require(poses[-1]['payload']['stamp'] > poses[0]['payload']['stamp'], 'pose source clock did not advance')
    model_gap = max((b['wall_monotonic'] - a['wall_monotonic'] for a, b in zip(models, models[1:])), default=0.)
    require(model_gap <= .25 and models[0]['wall_monotonic'] <= poses[0]['wall_monotonic'] + .15
            and models[-1]['wall_monotonic'] >= poses[-1]['wall_monotonic'] - .15,
            'Gazebo model observations do not cover the complete pose interval')
    for e in models:
        require(e['payload'].get('model') == 'app_carrier', 'model identity mismatch')
        xyz(e['payload'].get('xyz'))
    for previous, current in zip(poses, poses[1:]):
        dt = current['wall_monotonic'] - previous['wall_monotonic']
        require(current['payload'].get('stamp', -1) >= previous['payload'].get('stamp', 0), 'source time reversed')
        require(distance(xyz(current['payload']['xyz']), xyz(previous['payload']['xyz'])) <= .8 * dt + .08,
                'pose jumped beyond velocity integration/snap allowance')
    start, goal = (scene.task(case)[k]['xyz'] for k in ('start', 'goal'))
    points = [xyz(e['payload']['xyz']) for e in poses]
    require(distance(points[0], start) <= .30, 'capture missed task start')
    # Compare observations from two topics against a common wall-time neighbourhood.
    model_times = [e['wall_monotonic'] for e in models]
    paired, mismatch = 0, 0
    for e in poses:
        at = e['wall_monotonic']; index = bisect.bisect_left(model_times, at)
        candidates = models[max(0, index - 1):min(len(models), index + 1)]
        if not candidates:
            continue
        model = min(candidates, key=lambda x: abs(x['wall_monotonic'] - at))
        dt = abs(model['wall_monotonic'] - at)
        if dt > .25:
            continue
        observed = list(xyz(model['payload']['xyz'])); observed[2] -= .30
        mismatch += distance(observed, xyz(e['payload']['xyz'])) > .8 * dt + .10
        paired += 1
    require(paired >= 20 and paired >= .95 * len(poses) and mismatch == 0,
            'Gazebo/ROS pose mismatch or insufficient whole-run pairing coverage')
    # The carrier integrates the latest received linear command against a
    # monotonic wall clock (with its existing .5 s stale cutoff), not ROS time.
    # Source pose stamps and event ros_time may belong to different clocks.
    for e in commands:
        xyz(e['payload'].get('linear')); xyz(e['payload'].get('angular'))
    nonzero = [e for e in commands if any(abs(v) > 1e-6 for name in ('linear', 'angular')
                                         for v in xyz(e['payload'][name]))]
    pose_gap = max((b['wall_monotonic'] - a['wall_monotonic'] for a, b in zip(poses, poses[1:])), default=0.)
    command_gap = max((b['wall_monotonic'] - a['wall_monotonic'] for a, b in zip(commands, commands[1:])), default=0.)
    require(pose_gap <= .25 and command_gap <= .25, 'observation gap exceeds .25 s')
    require(commands[0]['wall_monotonic'] <= poses[0]['wall_monotonic'] + .15 and
            commands[-1]['wall_monotonic'] >= poses[-1]['wall_monotonic'] - .15,
            'commands do not cover the pose observation interval')
    command_times = [e['wall_monotonic'] for e in commands]
    prefix = [[0., 0., 0.]]
    for i, e in enumerate(commands[:-1]):
        dt = min(.5, max(0., command_times[i + 1] - command_times[i]))
        prefix.append([prefix[-1][axis] + xyz(e['payload']['linear'])[axis] * dt for axis in range(3)])
    def integral(at):
        i = bisect.bisect_right(command_times, at) - 1
        if i < 0:
            return [0., 0., 0.]
        dt = min(.5, max(0., at - command_times[i]))
        return [prefix[i][axis] + xyz(commands[i]['payload']['linear'])[axis] * dt for axis in range(3)]
    origin_integral = integral(poses[0]['wall_monotonic'])
    max_xy_error, max_z_error = 0., 0.
    for e, observed in zip(poses, points):
        total = integral(e['wall_monotonic'])
        predicted = [points[0][i] + total[i] - origin_integral[i] for i in range(3)]
        max_xy_error = max(max_xy_error, math.hypot(predicted[0] - observed[0], predicted[1] - observed[1]))
        max_z_error = max(max_z_error, abs(predicted[2] - observed[2]))
    require(max_xy_error <= .15, 'linear command integral does not explain observed XY motion')
    # Up to ten existing carrier snaps, each bounded by .03 m; this allowance
    # is explicit and is not a claim of physical foot support at step transfer.
    require(max_z_error <= .03 * 10 + .05, 'linear command integral does not explain observed Z motion')
    if interactive:
        return {'ok': True, 'scope': 'interactive capture integrity only; no task arrival acceptance',
                'pose_samples': len(poses), 'gazebo_samples': len(models), 'paired': paired,
                'command_integral_max_xy_error_m': max_xy_error, 'command_integral_max_z_error_m': max_z_error}
    geometry = scene.geometry(case == 'blocked')
    if case == 'blocked':
        require(app_result.get('ok') is False, 'blocked app incorrectly reported arrival success')
        statuses = [e['payload'] for e in grouped['global_status']]
        require(any(s.get('reason') == 'E_PLAN_NO_ROUTE' for s in statuses), 'not the expected planner no-route refusal')
        require(not any(s.get('ok') is True for s in statuses), 'blocked case produced a success status')
        require(not any(e['payload'].get('points') for e in grouped.get('global_path', [])), 'blocked case produced a path')
        require(not nonzero, 'blocked case emitted nonzero commands')
        require(poses[-1]['wall_monotonic'] - poses[0]['wall_monotonic'] >= 5.,
                'blocked refusal observed for less than five seconds')
        require(max(distance(p, start) for p in points) <= .10, 'blocked case moved before returning')
        collision_free(points, geometry)
        return {'ok': True, 'scope': 'known-map kinematic refusal', 'reason': 'E_PLAN_NO_ROUTE',
                'pose_samples': len(poses), 'command_samples': len(commands), 'paired': paired}
    require(app_result.get('ok') is True, 'app did not report successful arrival')
    require(nonzero, 'no nonzero command during positive case')
    require(all(abs(v) <= 1e-6 for name in ('linear', 'angular')
                for v in xyz(commands[-1]['payload'].get(name))), 'last command was not zero')
    require(not any(abs(v) > 1e-6 for e in commands for v in xyz(e['payload']['angular'])),
            'fixed-heading carrier received unsupported angular commands')
    require(distance(points[-1], goal) <= scene.task(case)['goal']['tolerance']['position_m'] + 1e-6
            and abs(points[-1][2] - goal[2]) <= .03, 'actual pose did not reach task goal')
    require(grouped.get('local_trajectory') and grouped.get('local_status'), 'missing local planning process')
    paths = [e['payload'] for e in grouped.get('global_path', []) if e['payload'].get('points')]
    require(paths, 'missing complete global path')
    for path in paths:
        require(path.get('frame') == 'map', 'global path frame')
        route = [xyz(p) for p in path['points']]
        require(distance(route[0], start) <= .4 and distance(route[-1], goal) <= .4, 'global path endpoints')
        collision_free(route, geometry)
        if case in ('upstairs', 'downstairs'):
            require(level_visits(route) == set(range(11)), 'global path skipped a stair level')
    for event in grouped['local_trajectory']:
        planned = event['payload'].get('trajectory')
        require(isinstance(planned, list) and len(planned) >= 2, 'local trajectory points missing')
        collision_free([xyz(p) for p in planned], geometry)
    collision_free(points, geometry)
    observed_model_points = []
    for e in models:
        p = list(xyz(e['payload']['xyz'])); p[2] -= .30; observed_model_points.append(p)
    collision_free(observed_model_points, geometry)
    if case in ('upstairs', 'downstairs'):
        require(level_visits(points) == set(range(11)), 'execution did not visit every stair height')
    if case == 'upper_detour':
        require(max(abs(p[1] + 3) for p in points) >= .70, 'execution did not route around the declared box')
    return {'ok': True, 'scope': 'declared-map Gazebo kinematic planning; no physical gait',
            'pose_samples': len(poses), 'gazebo_samples': len(models), 'command_samples': len(commands),
            'paired': paired, 'global_paths': len(paths), 'local_plans': len(grouped['local_trajectory']),
            'command_integral_max_xy_error_m': max_xy_error, 'command_integral_max_z_error_m': max_z_error,
            'max_pose_gap_s': pose_gap, 'max_command_gap_s': command_gap, 'max_model_gap_s': model_gap,
            'visited_height_levels': sorted(level_visits(points)), 'four_foot_physics': 'NOT_TESTED',
            'global_corridor_following': 'NOT_ESTABLISHED: current adapter forwards final goal only'}
