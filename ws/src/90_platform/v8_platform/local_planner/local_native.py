"""Rolling space-time search on explicitly supported layers (R655 native fallback).

This module creates new local motion from current state and predicted obstacle
volumes; it does not time-stamp a global route. Coordinates are support/contact
coordinates, not base_link height. Limits without a robot profile describe an
experimental kinematic carrier and do not establish Lite3 capability.
"""
import heapq
import itertools
import math
import sys
from dataclasses import asdict, dataclass, replace
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
for _directory in ('contracts', 'validator'):
    sys.path.insert(0, str(ROOT / _directory))
import validate as contract
import trajectory_checker as checker
import enforcement

SUPPORT_TOL_S = 1e-6


class LocalError(Exception):
    def __init__(self, code, detail):
        self.code, self.detail = code, detail
        super().__init__(code + ': ' + detail)


@dataclass(frozen=True)
class Config:
    resolution: float = 0.2
    tick_s: float = 0.5
    horizon_s: float = 6.0
    validity_s: float = 0.75
    max_speed_mps: float = 0.6
    max_step_m: float = 0.2
    max_slope_deg: float = 35.0
    radius_m: float = 0.15
    height_m: float = 0.35
    clearance_m: float = 0.2
    goal_tolerance_m: float = 0.08
    max_expansions: int = 20000
    command_latency_bound_s: float = 0.01

    def validate(self):
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise LocalError('E_LOCAL_INPUT', name + ' must be finite numeric')
            if value <= 0:
                raise LocalError('E_LOCAL_INPUT', name + ' must be positive')
        if self.horizon_s < 2 * self.tick_s:
            raise LocalError('E_LOCAL_INPUT', 'horizon must leave time for a terminal hold')
        if not isinstance(self.max_expansions, int):
            raise LocalError('E_LOCAL_INPUT', 'max_expansions must be an integer')


def _xyz(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise LocalError('E_LOCAL_INPUT', name + ' must be xyz')
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in value):
        raise LocalError('E_LOCAL_INPUT', name + ' must have finite numeric coordinates')
    return tuple(float(x) for x in value)


def _distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a[:3], b[:3])))


def _obstacles(values):
    if not isinstance(values, (list, tuple)):
        raise LocalError('E_LOCAL_INPUT', 'obstacles must be a list of box predictions')
    out = []
    for value in values:
        if not isinstance(value, dict):
            raise LocalError('E_LOCAL_INPUT', 'each obstacle must be an object')
        try:
            lo, hi = _xyz(value['min'], 'obstacle.min'), _xyz(value['max'], 'obstacle.max')
            velocity = _xyz(value.get('velocity', (0, 0, 0)), 'obstacle.velocity')
            epoch = float(value.get('stamp', 0.0))
            start = float(value.get('active_from', -1e100))
            end = float(value.get('active_until', 1e100))
        except (KeyError, TypeError, ValueError) as error:
            raise LocalError('E_LOCAL_INPUT', 'invalid obstacle: ' + str(error))
        if any(lo[i] >= hi[i] for i in range(3)) or not all(map(math.isfinite, (epoch, start, end))) or start > end:
            raise LocalError('E_LOCAL_INPUT', 'invalid obstacle bounds/time window')
        out.append((lo, hi, velocity, epoch, start, end))
    return out


def _swept_hit(a, b, obstacle, config):
    """Exact segment/linearly moving inflated AABB intersection, including time.

    Horizontal expansion contains radius and requested clearance. Vertical
    expansion represents the body above support z; no endpoint-only sampling.
    """
    lo, hi, velocity, epoch, active_from, active_until = obstacle
    duration = b[3] - a[3]
    if duration <= 0:
        raise LocalError('E_LOCAL_INPUT', 'segment duration must be positive')
    enter = max(0.0, (active_from - a[3]) / duration)
    leave = min(1.0, (active_until - a[3]) / duration)
    if enter > leave:
        return False
    lower = (lo[0] - config.radius_m - config.clearance_m,
             lo[1] - config.radius_m - config.clearance_m,
             lo[2] - config.height_m)
    upper = (hi[0] + config.radius_m + config.clearance_m,
             hi[1] + config.radius_m + config.clearance_m, hi[2])
    for axis in range(3):
        origin = a[axis] - velocity[axis] * (a[3] - epoch)
        delta = b[axis] - a[axis] - velocity[axis] * duration
        if abs(delta) < 1e-12:
            if origin < lower[axis] or origin > upper[axis]:
                return False
        else:
            u, v = (lower[axis] - origin) / delta, (upper[axis] - origin) / delta
            enter, leave = max(enter, min(u, v)), min(leave, max(u, v))
            if enter > leave:
                return False
    return True


def _profile_limits(robot_profile, config):
    """Do not turn declared/prohibited Lite3 numbers into usable capability."""
    if robot_profile is None:
        return config.max_step_m, config.max_slope_deg
    enforcement._validated_robot(robot_profile)
    # Envelope/clearance must also be usable; checking only step/slope could
    # otherwise promote an incomplete profile while using a smaller test body.
    clearance = enforcement._usable(robot_profile['capability']['min_clearance_m'], 'min_clearance_m')
    enforcement._usable(robot_profile['brake'], 'brake.max_decel_mps2')
    footprint = robot_profile['footprint']
    radius = (footprint['radius_m'] if footprint['type'] == 'circle' else
              max(math.hypot(*p) for p in footprint['points']))
    if config.radius_m + 1e-9 < radius or config.height_m + 1e-9 < footprint['height_m']:
        raise LocalError('E_LOCAL_ENVELOPE', 'experimental envelope does not contain robot profile')
    if config.clearance_m + 1e-9 < clearance:
        raise LocalError('E_LOCAL_ENVELOPE', 'experimental clearance is below robot profile')
    return (enforcement._usable(robot_profile['capability']['max_step_height_m'], 'max_step_height_m'),
            enforcement._usable(robot_profile['capability']['max_slope_deg'], 'max_slope_deg'))


def body_collision(map_value, a, b, obstacles, config, contacts=(None, None)):
    """Recheck a whole executed segment at its actual activation timestamps."""
    source, target = contacts
    allowed = set()
    for surface in map_value['surfaces']:
        for edge in surface['edges']:
            allowed.add((surface['surface_id'], edge['to']))
            if edge['bidirectional']:
                allowed.add((edge['to'], surface['surface_id']))
    transfer = source != target and (source, target) in allowed
    for surface in map_value['surfaces']:
        if transfer and surface['surface_id'] in (source, target):
            continue
        if source == target == surface['surface_id'] and all(
                abs(p[2] - surface['z_reference']) <= SUPPORT_TOL_S for p in (a, b)):
            continue
        top = surface['z_reference']
        if top <= min(a[2], b[2]) + SUPPORT_TOL_S:
            continue
        fp = surface['footprint']
        volume = ((fp['min_x'], fp['min_y'], top - surface.get('thickness_m', 0.)),
                  (fp['max_x'], fp['max_y'], top), (0., 0., 0.), 0., -1e100, 1e100)
        if _swept_hit(a, b, volume, replace(config, height_m=max(config.height_m, config.clearance_m))):
            return 'static:' + surface['surface_id']
    for index, obstacle in enumerate(_obstacles(obstacles)):
        if _swept_hit(a, b, obstacle, config):
            return 'dynamic:%d' % index
    return None


def track_segment(map_value, result, pose, now, obstacles, config=None, control_period_s=.05,
                  feedback_pose=None, uncertainty_m=0., previous_velocity=None):
    """Execute one accepted local segment using actual position feedback.

    The node must finish/reject this segment before searching from a new pose.
    A late command never inherits collision safety from an earlier plan clock:
    the complete remaining movement is checked again at *now*.
    """
    config = config or Config()
    pose = _xyz(pose, 'feedback pose')
    zero = {'velocity': [0., 0., 0.], 'stop': True, 'complete': False}
    if not result or not result.get('ok'):
        return dict(zero, reason='E_LOCAL_NO_TRAJECTORY')
    if result['map_id'] != map_value['map_id'] or result['map_version'] != map_value['map_version']:
        return dict(zero, reason='E_LOCAL_MAP_STALE')
    if not math.isfinite(now) or now < result['created_at'] or now >= result['valid_until']:
        return dict(zero, reason='E_LOCAL_EXPIRED')
    points = result['trajectory']
    a, target = points[0], points[1]
    duration = target[3] - a[3]
    distance = _distance(pose, target)
    planned_distance = _distance(a, target)
    contacts = tuple(result['surfaces'][:2])
    if planned_distance <= 1e-12:
        if now >= target[3]:
            return dict(zero, reason='segment_complete', complete=True)
        collision = body_collision(map_value, (*pose, now), (*pose, target[3]), obstacles, config, contacts)
        return dict(zero, reason='E_LOCAL_COLLISION' if collision else 'hold')
    observed = pose if feedback_pose is None else tuple(feedback_pose)
    previous_speed = math.sqrt(sum(v * v for v in previous_velocity)) if previous_velocity else 0.
    tail = config.command_latency_bound_s * previous_speed
    if _distance(observed, target) + tail <= 1e-6:
        return dict(zero, reason='segment_complete', complete=True)
    # The geometric segment is committed; its actual-time collision check below
    # permits feedback tracking up to the existing experimental speed cap. This
    # leaves terminal settling time inside the original trajectory validity.
    speed = config.max_speed_mps
    remaining_time = max(control_period_s, distance / speed)
    collision = body_collision(map_value, (*pose, now), (*target[:3], now + remaining_time),
                               obstacles, replace(config, radius_m=config.radius_m + uncertainty_m), contacts)
    if collision:
        return dict(zero, reason='E_LOCAL_COLLISION', collision=collision)
    # Reserve the old command's in-flight tail and this command's own travel
    # over one control period plus the transport bound. The final creep band
    # caps the terminal speed so the tail itself stays inside the 1e-6 band.
    if distance <= 1e-15:
        return dict(zero, reason='hold')
    if distance <= 5e-6:
        scale = min(speed / distance, 5e-5 / distance)
    else:
        horizon = control_period_s + config.command_latency_bound_s
        scale = min(speed / distance, max(0., distance - tail - 5e-8) / (horizon * distance))
    return {'velocity': [(target[i] - pose[i]) * scale for i in range(3)],
            'stop': False, 'complete': False, 'reason': 'track',
            'segment_target': list(target[:3]), 'execution_checked_at': now}


def plan(map_value, start, goal, obstacles=(), now=0.0, config=None,
         robot_profile=None, cancel=None):
    """Return a newly searched finite 3D trajectory or an explicit refusal.

    A trajectory has absolute timestamps, map id/version, valid_until, layer
    identities and a stationary terminal segment. A partial horizon must make
    geometric progress; a trapped robot is refused, not returned as success.
    """
    config = config or Config()
    config.validate()
    start, goal = _xyz(start, 'start'), _xyz(goal, 'goal')
    if isinstance(now, bool) or not isinstance(now, (int, float)) or not math.isfinite(now):
        raise LocalError('E_LOCAL_INPUT', 'now must be finite numeric')
    try:
        contract.validate('map', map_value)
        max_step, max_slope = _profile_limits(robot_profile, config)
    except (contract.ContractError, enforcement.EnforcementError) as error:
        raise LocalError(getattr(error, 'code', 'E_LOCAL_MAP'), str(error))
    moving = _obstacles(obstacles)
    surfaces = {s['surface_id']: s for s in map_value['surfaces']
                if s['traversability'] == 'supported'}
    required_ceiling = max(config.height_m, config.clearance_m)

    def inside(surface, point, margin=0.0):
        box = surface['footprint']
        return (box['min_x'] + margin - 1e-9 <= point[0] <= box['max_x'] - margin + 1e-9
                and box['min_y'] + margin - 1e-9 <= point[1] <= box['max_y'] - margin + 1e-9)

    def safe_ceiling(point, current_sid=None):
        padding = config.radius_m + config.clearance_m
        return not any(not (s['surface_id'] == current_sid and abs(point[2] - s['z_reference']) <= SUPPORT_TOL_S)
                       and inside(s, point, margin=-padding)
                       and point[2] + SUPPORT_TOL_S < s['z_reference']
                       and s['z_reference'] - s.get('thickness_m', 0.) - point[2] < required_ceiling
                       for s in map_value['surfaces'])

    def resolve(point, label):
        choices = [s for s in surfaces.values() if inside(s, point, margin=config.radius_m)
                   and abs(point[2] - s['z_reference']) <= SUPPORT_TOL_S]
        if len(choices) != 1:
            raise LocalError('E_LOCAL_SUPPORT', label + ' requires exactly one matching z support')
        if not safe_ceiling(point, choices[0]['surface_id']):
            raise LocalError('E_LOCAL_CLEARANCE', label + ' is below a low ceiling')
        return choices[0]['surface_id']

    first_sid, goal_sid = resolve(start, 'start'), resolve(goal, 'goal')
    allowed = {}
    for sid, surface in surfaces.items():
        for edge in surface['edges']:
            if edge['to'] in surfaces:
                allowed[sid, edge['to']] = edge
                if edge['bidirectional']:
                    allowed[edge['to'], sid] = edge
    # Explicit topological impossibility is distinguished from a blocked horizon.
    reachable = {first_sid}
    while True:
        expanded = reachable | {b for a, b in allowed if a in reachable}
        if expanded == reachable:
            break
        reachable = expanded
    if goal_sid not in reachable:
        raise LocalError('E_LOCAL_NO_ROUTE', 'goal support is unreachable by directed edges')

    first = (first_sid, 0, 0)
    last = (goal_sid, 'goal', 'goal')
    reach_cells = int(math.ceil(config.max_speed_mps * config.tick_s / config.resolution))
    offsets = [(dx, dy) for dx in range(-reach_cells, reach_cells + 1)
               for dy in range(-reach_cells, reach_cells + 1)]

    def static_collision(a, b, source, target):
        # Explicit support-edge transfers are authorized contact transitions,
        # not a permission to cross any unrelated slab. Same-surface motion
        # must clear every overlying slab along the complete swept body.
        contact_transfer = source != target and (source, target) in allowed
        for surface in map_value['surfaces']:
            if contact_transfer and surface['surface_id'] in (source, target):
                continue
            top = surface['z_reference']
            if top <= min(a[2], b[2]) + SUPPORT_TOL_S:
                continue
            fp = surface['footprint']
            volume = ((fp['min_x'], fp['min_y'], top - surface.get('thickness_m', .0)),
                      (fp['max_x'], fp['max_y'], top), (0., 0., 0.), 0., -1e100, 1e100)
            if _swept_hit(a, b, volume, replace(config, height_m=required_ceiling)):
                return True
        return False

    def xyz(node):
        if node == first:
            return start
        if node == last:
            return goal
        return (start[0] + node[1] * config.resolution,
                start[1] + node[2] * config.resolution, surfaces[node[0]]['z_reference'])

    def neighbors(node):
        if node == last:
            return [last]
        sid, ix, iy = node
        candidates = [(sid, ix + dx, iy + dy) for dx, dy in offsets]
        for source, target in sorted(allowed):
            if source == sid:
                candidates.extend((target, ix + dx, iy + dy) for dx, dy in offsets)
        if goal_sid == sid or (sid, goal_sid) in allowed:
            candidates.append(last)
        point = xyz(node)
        accepted = []
        for other in candidates:
            dest = xyz(other)
            dist = _distance(point, dest)
            if dist > config.max_speed_mps * config.tick_s + 1e-9:
                continue
            if not inside(surfaces[other[0]], dest, margin=config.radius_m) or not safe_ceiling(dest, other[0]):
                continue
            if other[0] != sid:
                edge = allowed[sid, other[0]]
                dz, dxy = abs(dest[2] - point[2]), math.hypot(dest[0] - point[0], dest[1] - point[1])
                if edge['kind'] == 'step' and dz > min(max_step, edge['max_step_height_m']) + 1e-9:
                    continue
                if edge['kind'] == 'slope' and (dxy <= 1e-9 or math.degrees(math.atan2(dz, dxy))
                                                > min(max_slope, edge['max_slope_deg']) + 1e-9):
                    continue
            accepted.append(other)
        return accepted

    ticks = int(math.floor(config.horizon_s / config.tick_s)) - 1
    serial = itertools.count()
    initial = (first, 0)
    frontier = [(_distance(start, goal) / config.max_speed_mps, next(serial), initial)]
    parents, costs = {}, {initial: 0.0}
    expansions, terminal, best = 0, None, None
    best_score = float('inf')
    start_distance = _distance(start, goal)
    while frontier:
        if cancel is not None and cancel():
            raise LocalError('E_LOCAL_CANCELLED', 'cancelled before candidate acceptance')
        _priority, _sequence, state = heapq.heappop(frontier)
        node, tick = state
        expansions += 1
        if expansions > config.max_expansions:
            raise LocalError('E_LOCAL_BUDGET', 'space-time expansion budget exhausted')
        point, stamp = xyz(node), now + tick * config.tick_s
        hold = (*point, stamp + config.tick_s)
        can_hold = not any(_swept_hit((*point, stamp), hold, o, config) for o in moving)
        remaining = _distance(point, goal)
        if can_hold and remaining <= config.goal_tolerance_m:
            terminal = state
            break
        if tick == ticks:
            # Progress must be real in xyz, not just a sequence of waits.
            score = remaining + 0.01 * costs[state]
            if can_hold and remaining < start_distance - 0.5 * config.resolution and score < best_score:
                best, best_score = state, score
            continue
        for other in neighbors(node):
            nxt = (other, tick + 1)
            dest = xyz(other)
            a, b = (*point, stamp), (*dest, stamp + config.tick_s)
            if static_collision(a, b, node[0], other[0]) or any(_swept_hit(a, b, o, config) for o in moving):
                continue
            ng = costs[state] + config.tick_s + 0.05 * _distance(point, dest)
            if ng >= costs.get(nxt, float('inf')) - 1e-12:
                continue
            costs[nxt], parents[nxt] = ng, state
            priority = ng + _distance(dest, goal) / config.max_speed_mps
            heapq.heappush(frontier, (priority, next(serial), nxt))
    terminal = terminal or best
    if terminal is None:
        raise LocalError('E_LOCAL_NO_ROUTE', 'no collision-free, supported progress within horizon')
    chain = []
    cursor = terminal
    while cursor is not None:
        chain.append(cursor)
        cursor = parents.get(cursor)
    chain.reverse()
    points = [(*xyz(node), now + tick * config.tick_s) for node, tick in chain]
    # A terminal hold makes stale command cutoff explicit and permits decel
    # checking. Keep it just inside the horizon: a full tick can exceed the
    # checker's time budget through floating-point roundoff at some start times.
    hold_s = max(1e-6, min(config.tick_s, config.horizon_s - (points[-1][3] - now)) - 1e-6)
    points.append((*points[-1][:3], points[-1][3] + hold_s))
    layers = [state[0][0] for state in chain] + [chain[-1][0][0]]
    try:
        verdict = checker.check(map_value, points, z_tol=SUPPORT_TOL_S, min_clearance=required_ceiling,
                                max_step=config.max_speed_mps * config.tick_s + 1e-9,
                                time_budget=config.horizon_s, expected_map_version=map_value['map_version'])
    except checker.CheckerError as error:
        raise LocalError('E_LOCAL_CHECKER', str(error))
    if not verdict['ok']:
        raise LocalError('E_LOCAL_CHECKER', repr(verdict['violations']))
    if robot_profile is not None:
        try:
            violations = enforcement.check_kinematics(points, robot_profile)
        except enforcement.EnforcementError as error:
            raise LocalError(error.code, str(error))
        if violations:
            raise LocalError('E_LOCAL_KINEMATICS', repr(violations))
    return {'ok': True, 'reason': 'arrived' if _distance(points[-1], goal) <= config.goal_tolerance_m else 'horizon',
            'trajectory': points, 'surfaces': layers, 'map_id': map_value['map_id'],
            'map_version': map_value['map_version'], 'created_at': float(now),
            'valid_until': float(now) + min(config.validity_s, points[-1][3] - now),
            'expansions': expansions, 'checker': verdict, 'config': asdict(config),
            'capability_domain': 'validated_robot_profile' if robot_profile else 'experimental_kinematic_carrier'}


def command(result, now, map_id, map_version, localization_ok=True):
    """Fail-closed 3D velocity; validity/map/localization are checked every tick."""
    zero = {'velocity': [0.0, 0.0, 0.0], 'stop': True}
    if not localization_ok:
        return dict(zero, reason='E_LOCAL_LOCALIZATION')
    if not result or not result.get('ok'):
        return dict(zero, reason='E_LOCAL_NO_TRAJECTORY')
    if result['map_id'] != map_id or result['map_version'] != map_version:
        return dict(zero, reason='E_LOCAL_MAP_STALE')
    if not math.isfinite(now) or now < result['created_at'] or now >= result['valid_until']:
        return dict(zero, reason='E_LOCAL_EXPIRED')
    points = result['trajectory']
    for a, b in zip(points, points[1:]):
        if a[3] <= now < b[3]:
            velocity = [(b[i] - a[i]) / (b[3] - a[3]) for i in range(3)]
            return {'velocity': velocity, 'stop': all(abs(v) < 1e-12 for v in velocity),
                    'reason': 'hold' if all(abs(v) < 1e-12 for v in velocity) else 'track'}
    return dict(zero, reason='E_LOCAL_EXPIRED')
