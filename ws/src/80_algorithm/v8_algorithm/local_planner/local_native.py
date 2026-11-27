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
    sys.path.insert(0, str(ROOT.parent.parent / '90_platform/v8_platform' / _directory))
sys.path.insert(0, str(ROOT / 'representation'))
import validate as contract
import trajectory_checker as checker
import enforcement
import support_map as rep
import contact_egress as contact_geometry

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
    support_tol_m: float = 1e-6
    # ★估计位姿的支撑面关联容差（2026-11-27 裁决批准）★
    # 与 support_tol_m 分离：support_tol_m 仍是**几何/接触**判据（目标点、轨迹点、碰撞、净空
    # 全部继续用它），本字段只用于"把估计位姿关联到某个支撑面"这一步。
    # 默认 1e-6 保持历史兼容；正式取值必须先有误差依据与多层可分辨性证据，另行裁决。
    # 绝不允许用关联后的 z 覆盖原始定位输出——关联结果只用于选面与记录。
    support_assoc_tol_m: float = 1e-6
    # 速度相关安全距离（只在 base 之上做加法，永不减少既有 clearance）。
    # reaction_s       感知周期 + 决策延迟；obstacles 实测 10 Hz，取 0.10 覆盖一个周期
    # brake_decel_mps2 保守制动减速度；Lite3 实际能力高于此值，取小值属安全侧
    # dynamic_margin_cap_m 额外量的上限，避免高速时把通道算成不可通行
    # 定位误差不单独计入：clearance_m=0.10 已大于 hdl 实测 XY 误差（mean 0.0265 / max 0.0452 m）
    reaction_s: float = 0.10
    brake_decel_mps2: float = 1.0
    dynamic_margin_cap_m: float = 0.30

    def migration_notes(self):
        """历史配置迁移提示（只提示，不自动改值）。

        2026-11-27：起点关联改用 support_assoc_tol_m。若某份历史配置显式写了较大的
        support_tol_m（例如 0.25/0.05）而没写 assoc，起点关联会**比过去更严**——
        这不是放宽，而是不能静默"继承大容差"。这里返回提示供节点打日志，
        让人显式决定要不要给 assoc 单独取值占位，而不是代码替他们继承。
        """
        notes = []
        if self.support_tol_m != 1e-6 and self.support_assoc_tol_m == 1e-6:
            notes.append('support_tol_m=%.6g 为显式非默认值，但 support_assoc_tol_m 仍是 1e-6：'
                         '起点支撑关联会比旧版本更严（不会自动继承大容差）。'
                         '如该场景依赖旧行为，请显式给出 support_assoc_tol_m 并通过验收裁决。'
                         % self.support_tol_m)
        return notes

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


def _support_at(map_value, point, config, surfaces=None, cell_sets=None, tol=None):
    """Resolve a contact with the same cells, radius and z rule as planning.

    tol 显式给出时用该容差（估计位姿关联用 support_assoc_tol_m）；
    不给则沿用 support_tol_m（几何/接触判据）。
    """
    z_tol = config.support_tol_m if tol is None else tol
    # ★唯一实现★：plan 与 track 都经这里 → 都经 associate() 的 valid/ambiguous 判定，
    # 不再各自调 rep.select_support（那会把"歧义"吞掉，诊断与控制两套语义）。
    info = associate(map_value, point, config, tol=z_tol,
                     surfaces=surfaces, cell_sets=cell_sets)
    if not info['valid']:
        return None
    if surfaces is None:
        surfaces = {s['surface_id']: s for s in map_value['surfaces']
                    if s['traversability'] == 'supported'}
    return surfaces.get(info['surface_id'])


def associate(map_value, point, config, tol=None, map_version=None,
              surfaces=None, cell_sets=None):
    """把（估计）位姿关联到支撑面，并给出**可判读的关联结果**。

    返回 dict：valid / surface_id / residual_z / candidates / ambiguous / map_version。
    无匹配、多层歧义（同 XY 有多个候选且 z 不可区分）都返回 valid=False —— 调用方据此拒绝运动，
    而不是把最近的面对面硬套上去。绝不修改 point 本身（关联后的 z 不得覆盖原始定位输出）。
    """
    z_tol = config.support_assoc_tol_m if tol is None else tol
    if surfaces is None:
        surfaces = {s['surface_id']: s for s in map_value.get('surfaces', ())
                    if s.get('traversability') == 'supported'}
    if cell_sets is None:
        cell_sets = {sid: rep.surface_cells(s) for sid, s in surfaces.items()}
    cands = []
    for sid, s in surfaces.items():
        if not rep.contains_xy(s, map_value['resolution'], point[0], point[1],
                               config.radius_m, cells=cell_sets[sid]):
            continue
        residual = point[2] - s['z_reference']
        if abs(residual) <= z_tol:
            cands.append({'surface_id': sid, 'residual_z': residual,
                          'z_reference': s['z_reference']})
    result = {'valid': False, 'surface_id': None, 'residual_z': None,
              'candidates': cands, 'ambiguous': False,
              'map_version': map_version if map_version is not None else map_value.get('map_version')}
    if not cands:
        return result
    chosen = rep.select_support([surfaces[c['surface_id']] for c in cands], point[2])
    if chosen is None:
        return result
    # 多层歧义：候选里存在与所选面高度**不同**的另一面，且同样在容差内
    others = [c for c in cands if c['surface_id'] != chosen['surface_id']
              and abs(c['z_reference'] - chosen['z_reference']) > 1e-9]
    ambiguous = bool(others)
    # ★歧义即拒绝★（2026-11-27 真人指出）：上一版在这里仍写 valid=True，
    # 与"多层歧义必须拒绝运动"的契约自相矛盾，而且诊断说歧义、控制照走，
    # 等于两套安全语义。surface_id 仍回填，仅供日志判读，valid 必须为 False。
    result.update(valid=not ambiguous, ambiguous=ambiguous,
                  surface_id=chosen['surface_id'],
                  residual_z=point[2] - chosen['z_reference'])
    return result


def _support_contains_segment(surface, resolution, a, b, config, cells=None):
    """Check the complete stop tail against its declared target support.

    Cell maps may have holes even when both endpoints are supported. Split at
    every exact cell boundary (including contains_xy's existing offset), then
    check each boundary and open interval. Cell-less shrunk boxes are convex.
    """
    if (not math.isfinite(resolution) or resolution <= 0.
            or any(not math.isfinite(v) for p in (a, b) for v in p[:3])
            or any(abs(p[2] - surface['z_reference']) > config.support_tol_m for p in (a, b))):
        return False
    times = {0., 1.}
    if cells is not None:
        normalized = [tuple(p[axis] / resolution + 1e-9 for axis in (0, 1)) for p in (a, b)]
        if any(not math.isfinite(v) for p in normalized for v in p):
            return False
        indices = [tuple(math.floor(v) for v in p) for p in normalized]
        if any(index not in cells for index in indices):
            return False
        # Every crossed column/row needs at least one support cell. This
        # necessary coverage bound also caps enumeration by the actual map,
        # even for finite but enormous velocities or sparse distant endpoints.
        if any(abs(indices[1][axis] - indices[0][axis]) + 1 > len(cells) for axis in (0, 1)):
            return False
        for axis in (0, 1):
            start, end = normalized[0][axis], normalized[1][axis]
            if start == end:
                continue
            for boundary in range(min(indices[0][axis], indices[1][axis]) + 1,
                                  max(indices[0][axis], indices[1][axis]) + 1):
                fraction = (boundary - start) / (end - start)
                if 0. < fraction < 1.:
                    times.add(fraction)
    ordered = sorted(times)
    probes = ordered + [(lo + hi) / 2 for lo, hi in zip(ordered, ordered[1:])]
    return all(rep.contains_xy(surface, resolution,
                               a[0] * (1. - t) + b[0] * t,
                               a[1] * (1. - t) + b[1] * t,
                               config.radius_m, cells=cells) for t in probes)


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


class _ObstacleIndex:
    """Uniform XY bucket index for static map boxes.

    Static boxes are the thousands-of-boxes case (refined real map); dynamic
    predictions stay in a short list and are always tested. Only boxes that can
    reach the query segment's inflated XY envelope are returned.
    """

    def __init__(self, obstacles, cell_m=1.0):
        self.boxes = list(obstacles)
        self.cell = float(cell_m)
        self.buckets = {}
        for index, (lo, hi, _velocity, _epoch, _start, _end) in enumerate(self.boxes):
            i0 = int(math.floor(lo[0] / self.cell))
            i1 = int(math.floor(hi[0] / self.cell))
            j0 = int(math.floor(lo[1] / self.cell))
            j1 = int(math.floor(hi[1] / self.cell))
            for i in range(i0, i1 + 1):
                for j in range(j0, j1 + 1):
                    self.buckets.setdefault((i, j), []).append(index)

    def query(self, a, b, margin=0.0):
        pad = margin + 1e-9
        x0 = int(math.floor((min(a[0], b[0]) - pad) / self.cell))
        x1 = int(math.floor((max(a[0], b[0]) + pad) / self.cell))
        y0 = int(math.floor((min(a[1], b[1]) - pad) / self.cell))
        y1 = int(math.floor((max(a[1], b[1]) + pad) / self.cell))
        seen = set()
        found = []
        for i in range(x0, x1 + 1):
            for j in range(y0, y1 + 1):
                for index in self.buckets.get((i, j), ()):
                    if index in seen:
                        continue
                    seen.add(index)
                    found.append(self.boxes[index])
        return found


def _near_boundary(surface, point, band):
    box = surface['footprint']
    distance = min(abs(point[0] - box['min_x']), abs(point[0] - box['max_x']),
                   abs(point[1] - box['min_y']), abs(point[1] - box['max_y']))
    return distance <= band + 1e-9


def _steppable_surfaces(surfaces, start_id, max_step, max_slope, radius):
    """Direct step/slope targets that the current robot limits can traverse."""
    if start_id not in surfaces:
        return set()
    result = set()
    for edge in surfaces[start_id].get('edges', []):
        kind, target = edge.get('kind'), edge.get('to')
        if target not in surfaces:
            continue
        box = surfaces[target]['footprint']
        if min(box['max_x'] - box['min_x'], box['max_y'] - box['min_y']) <= 2 * radius + 1e-9:
            continue
        if kind == 'step' and edge.get('max_step_height_m', 0.) <= max_step + 1e-9:
            result.add(target)
        elif kind == 'slope' and edge.get('max_slope_deg', 0.) <= max_slope + 1e-9:
            result.add(target)
    return result


def dynamic_margin_extra(config, speed_mps):
    """速度相关的额外安全距离（米）。**只增不减**，且 fail-closed。

    物理含义：以 speed 行驶时，从"感知到障碍"到"停下来"会多走
        v * reaction_s + v^2 / (2 * brake_decel_mps2)
    这段距离必须在碰撞判定里提前预留，否则低速阈值在高速度下不够用。

    速度无法确定（None / 非有限 / 负数）时按 config.max_speed_mps 取值 ——
    宁可多留，不能因为拿不到速度就把余量降回固定值。
    """
    limit = max(float(getattr(config, 'max_speed_mps', 0.6)), 1e-6)
    speed = limit if speed_mps is None else float(speed_mps)
    if not math.isfinite(speed) or speed < 0.0:
        speed = limit
    reaction = float(getattr(config, 'reaction_s', 0.0))
    decel = float(getattr(config, 'brake_decel_mps2', 0.0))
    extra = speed * reaction
    if decel > 1e-6:
        extra += speed * speed / (2.0 * decel)
    cap = float(getattr(config, 'dynamic_margin_cap_m', extra))
    return min(max(extra, 0.0), cap)


def _segment_speed(a, b):
    """计划段的平均速度；段时长无效时返回 None（交给 fail-closed 分支）。"""
    duration = b[3] - a[3]
    if not math.isfinite(duration) or duration <= 1e-9:
        return None
    return _distance(a, b) / duration


def _swept_hit(a, b, obstacle, config, margin_extra=0.0):
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
    lower = (lo[0] - config.radius_m - config.clearance_m - margin_extra,
             lo[1] - config.radius_m - config.clearance_m - margin_extra,
             lo[2] - config.height_m)
    upper = (hi[0] + config.radius_m + config.clearance_m + margin_extra,
             hi[1] + config.radius_m + config.clearance_m + margin_extra, hi[2])
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


def _surface_blocks(map_value, surface, a, b, config, height_m, cells=None):
    """Does the swept body [z, z+height] hit this surface slab?

    Real cell sets replace the enclosing box; the XY line is sampled at half a
    map cell so a thin tread cannot slip between two samples.
    """
    top = surface['z_reference']
    thickness = surface.get('thickness_m', 0.)
    if cells is None:
        cells = rep.surface_cells(surface)
    if cells is None:
        fp = surface['footprint']
        volume = ((fp['min_x'], fp['min_y'], top - thickness),
                  (fp['max_x'], fp['max_y'], top), (0., 0., 0.), 0., -1e100, 1e100)
        return _swept_hit(a, b, volume, replace(config, height_m=height_m))
    if max(a[2], b[2]) + height_m <= top - thickness + 1e-9:
        return False
    if min(a[2], b[2]) > top - 1e-9:
        return False
    resolution = map_value['resolution']
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    steps = max(1, int(math.ceil(length / (resolution * 0.5))))
    padding = config.radius_m + config.clearance_m
    for k in range(steps + 1):
        x = a[0] + (b[0] - a[0]) * k / steps
        y = a[1] + (b[1] - a[1]) * k / steps
        if rep.contains_xy(surface, resolution, x, y, margin=-padding, cells=cells):
            return True
    return False


def body_collision(map_value, a, b, obstacles, config, contacts=(None, None), contact_egress=None):
    """Recheck a whole executed segment at its actual activation timestamps."""
    source, target = contacts
    allowed = set()
    for surface in map_value['surfaces']:
        for edge in surface['edges']:
            allowed.add((surface['surface_id'], edge['to']))
            if edge['bidirectional']:
                allowed.add((edge['to'], surface['surface_id']))
    transfer = source != target and (source, target) in allowed
    by_id = {surface['surface_id']: surface for surface in map_value['surfaces']}
    steppable = _steppable_surfaces(by_id, source, config.max_step_m,
                                    config.max_slope_deg, config.radius_m)
    cell_sets = {surface['surface_id']: rep.surface_cells(surface)
                 for surface in map_value['surfaces']}
    for surface in map_value['surfaces']:
        if (type(contact_egress) is contact_geometry.ContactWindow
                and contact_egress.permits(map_value, surface, a, b, config)):
            continue
        if transfer and surface['surface_id'] in (source, target):
            continue
        if (surface['surface_id'] in steppable and surface['surface_id'] != source
                and _near_boundary(surface, b, config.radius_m + config.clearance_m)):
            continue
        if source == target == surface['surface_id'] and all(
                abs(p[2] - surface['z_reference']) <= SUPPORT_TOL_S for p in (a, b)):
            continue
        top = surface['z_reference']
        if top <= min(a[2], b[2]) + SUPPORT_TOL_S:
            continue
        if _surface_blocks(map_value, surface, a, b, config,
                           max(config.height_m, config.clearance_m),
                           cells=cell_sets.get(surface['surface_id'])):
            return 'static:' + surface['surface_id']
    # 速度相关的额外安全距离：按本段计划速度预留反应+制动距离（只增不减）。
    # 段时长无效时 _segment_speed 返回 None，dynamic_margin_extra 会退到 max_speed（fail-closed）。
    speed = _segment_speed(a, b)
    margin_extra = dynamic_margin_extra(config, speed)
    for index, obstacle in enumerate(_obstacles(obstacles)):
        if _swept_hit(a, b, obstacle, config, margin_extra):
            return 'dynamic:%d' % index
    return None


def track_segment(map_value, result, pose, now, obstacles, config=None, control_period_s=.05,
                  feedback_pose=None, uncertainty_m=0., previous_velocity=None, contact_egress=None):
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
    index = 0
    if contact_egress is not None:
        if (type(contact_egress) is not contact_geometry.ContactEgress
                or not contact_egress.valid_for(map_value, result, config)):
            return dict(zero, reason='E_LOCAL_INPUT')
        index = contact_egress.index
    a, target = points[index], points[index + 1]
    duration = target[3] - a[3]
    distance = _distance(pose, target)
    planned_distance = _distance(a, target)
    contacts = tuple(result['surfaces'][index:index + 2])
    observed = pose if feedback_pose is None else _xyz(feedback_pose, 'actual feedback pose')
    previous = _xyz(previous_velocity, 'previous velocity') if previous_velocity is not None else (0., 0., 0.)
    tail = config.command_latency_bound_s * math.sqrt(sum(v * v for v in previous))
    if isinstance(uncertainty_m, bool) or not isinstance(uncertainty_m, (int, float)):
        return dict(zero, reason='E_LOCAL_INPUT')
    try:
        uncertainty_m = float(uncertainty_m)
    except OverflowError:
        return dict(zero, reason='hold')
    if not math.isfinite(uncertainty_m) or uncertainty_m < 0.:
        return dict(zero, reason='E_LOCAL_INPUT')
    travel_bound = distance + tail
    motion_bound = travel_bound + config.radius_m + config.height_m + config.clearance_m
    if not math.isfinite(travel_bound) or uncertainty_m > motion_bound:
        return dict(zero, reason='hold')
    contact_window = None
    if contact_egress is not None:
        contact_window = contact_egress.window(observed, pose, previous, uncertainty_m,
                                               _support_contains_segment)
        if contact_window is None:
            return dict(zero, reason='hold')
    # observed / predicted 都是**估计量**，用关联容差；后续碰撞/净空/制动仍走几何判据
    support = _support_at(map_value, observed, config, tol=config.support_assoc_tol_m)
    target_supported = support is not None and support['surface_id'] == contacts[1]
    predicted_support = _support_at(map_value, pose, config, tol=config.support_assoc_tol_m)
    predicted_target_supported = predicted_support is not None and predicted_support['surface_id'] == contacts[1]
    tail_end = tuple(pose[i] + previous[i] * config.command_latency_bound_s for i in range(3))
    target_cells = rep.surface_cells(support) if target_supported else None
    tail_supported = (target_supported and predicted_target_supported
                      and _support_contains_segment(support, map_value['resolution'], observed, pose,
                                                    config, cells=target_cells)
                      and _support_contains_segment(support, map_value['resolution'], pose, tail_end,
                                                    config, cells=target_cells))
    if planned_distance <= 1e-12:
        if now >= target[3]:
            if not target_supported:
                return dict(zero, reason='E_LOCAL_SUPPORT')
            return dict(zero, reason='segment_complete' if tail_supported else 'hold',
                        complete=bool(tail_supported))
        collision = body_collision(map_value, (*pose, now), (*pose, target[3]), obstacles, config, contacts, contact_window)
        return dict(zero, reason='E_LOCAL_COLLISION' if collision else 'hold')
    # A low-speed legged carrier can overshoot a short committed step.  Chasing
    # that fixed waypoint backwards causes forward/backward oscillation; once
    # the observed pose is past the waypoint along the segment, advance. A
    # declared transfer instead keeps tracking until its target support holds.
    dx, dy = target[0] - a[0], target[1] - a[1]
    horizontal = math.hypot(dx, dy)
    overshot = (horizontal > 1e-9 and
                (observed[0] - target[0]) * dx + (observed[1] - target[1]) * dy > 1e-3 * horizontal)
    if overshot and not target_supported and contacts[0] == contacts[1]:
        return dict(zero, reason='E_LOCAL_SUPPORT')
    # A zero command still has the previous command's transport tail. Keep the
    # committed segment until that entire tail stays on its target support.
    if target_supported and not tail_supported:
        return dict(zero, reason='hold')
    observed_distance = _distance(observed, target)
    if target_supported and (overshot or observed_distance + tail <= 1e-6):
        return dict(zero, reason='segment_complete', complete=True)
    # The geometric segment is committed; its actual-time collision check below
    # permits feedback tracking up to the existing experimental speed cap. This
    # leaves terminal settling time inside the original trajectory validity.
    speed = config.max_speed_mps
    remaining_time = max(control_period_s, distance / speed)
    collision_config = replace(config, radius_m=config.radius_m + uncertainty_m)
    collision = body_collision(map_value, (*pose, now), (*target[:3], now + remaining_time),
                               obstacles, collision_config, contacts, contact_window)
    if collision:
        return dict(zero, reason='E_LOCAL_COLLISION', collision=collision)
    # Prediction can already be across a cell boundary while raw feedback is
    # still outside the target support. Only inside the EXISTING creep band,
    # use raw feedback for the correction direction; retain the prediction's
    # collision check above and also check the actual correction sweep.
    if not target_supported and observed_distance <= 5e-6 and distance <= 5e-6:
        collision = body_collision(map_value, (*observed, now), (*target[:3], now + remaining_time),
                                   obstacles, collision_config, contacts, contact_window)
        if collision:
            return dict(zero, reason='E_LOCAL_COLLISION', collision=collision)
        pose, distance = observed, observed_distance
    # Reserve the old command's in-flight tail and this command's own travel
    # over one control period plus the transport bound. The final creep band
    # caps the terminal speed so the tail itself stays inside the 1e-6 band.
    if distance <= 1e-15:
        return dict(zero, reason='hold' if target_supported else 'E_LOCAL_SUPPORT')
    if distance <= 5e-6:
        scale = min(speed / distance, 5e-5 / distance)
        if tail_supported:
            horizon = control_period_s + config.command_latency_bound_s
            end = tuple(pose[i] + (target[i] - pose[i]) * scale * horizon for i in range(3))
            if not _support_contains_segment(support, map_value['resolution'], pose, end,
                                             config, cells=target_cells):
                # Reuse the existing travel/tail reserve only when the normal
                # creep would leave an already acquired support. Applying it
                # outside that support would strand correction short of a cell.
                scale = min(scale, max(0., distance - tail - 5e-8) / (horizon * distance))
                end = tuple(pose[i] + (target[i] - pose[i]) * scale * horizon for i in range(3))
                if not _support_contains_segment(support, map_value['resolution'], pose, end,
                                                 config, cells=target_cells):
                    return dict(zero, reason='hold')
    else:
        horizon = control_period_s + config.command_latency_bound_s
        scale = min(speed / distance, max(0., distance - tail - 5e-8) / (horizon * distance))
    if target_supported and uncertainty_m > 0.:
        direction = tuple((target[i] - pose[i]) / distance for i in range(3))
        uncertain_end = tuple(pose[i] + previous[i] * config.command_latency_bound_s +
                              (target[i] - pose[i]) * scale * horizon +
                              direction[i] * uncertainty_m for i in range(3))
        if not _support_contains_segment(support, map_value['resolution'], pose, uncertain_end,
                                         config, cells=target_cells):
            return dict(zero, reason='hold')
    velocity = [(target[i] - pose[i]) * scale for i in range(3)]
    if contact_window is not None and contact_window.allow_old:
        end = tuple(pose[i] + velocity[i] * (control_period_s + config.command_latency_bound_s)
                    for i in range(3))
        if not contact_egress.outward(pose, end, uncertainty_m):
            return dict(zero, reason='hold')
    return {'velocity': velocity,
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
        contract.validate('map', map_value, allow_obstacles=True)
        max_step, max_slope = _profile_limits(robot_profile, config)
    except (contract.ContractError, enforcement.EnforcementError) as error:
        raise LocalError(getattr(error, 'code', 'E_LOCAL_MAP'), str(error))
    if not isinstance(obstacles, (list, tuple)):
        raise LocalError('E_LOCAL_INPUT', 'obstacles must be a list of box predictions')
    static_boxes = list(map_value.get('obstacles') or [])
    static_moving = _obstacles(static_boxes)
    dynamic_moving = _obstacles(list(obstacles))
    moving = static_moving + dynamic_moving
    static_index = _ObstacleIndex(static_moving) if static_moving else None

    def hit(a, b):
        if static_index is not None and any(
                _swept_hit(a, b, obstacle, config)
                for obstacle in static_index.query(
                    a, b, config.radius_m + config.clearance_m)):
            return True
        return any(_swept_hit(a, b, obstacle, config) for obstacle in dynamic_moving)
    surfaces = {s['surface_id']: s for s in map_value['surfaces']
                if s['traversability'] == 'supported'}
    required_ceiling = max(config.height_m, config.clearance_m)

    # A goal inside an inflated static obstacle can only be approached by
    # partial progress and then refused at close range; refuse it first.
    # Moving obstacles are left to the time-aware search, since a goal covered
    # now may clear before arrival.
    for lo, hi, velocity, epoch, active_from, active_until in moving:
        if any(abs(v) > 0 for v in velocity) or not active_from <= now <= active_until:
            continue
        shift = [velocity[i] * (now - epoch) for i in range(3)]
        lower = [lo[0] + shift[0] - config.radius_m - config.clearance_m,
                 lo[1] + shift[1] - config.radius_m - config.clearance_m,
                 lo[2] + shift[2] - config.height_m]
        upper = [hi[0] + shift[0] + config.radius_m + config.clearance_m,
                 hi[1] + shift[1] + config.radius_m + config.clearance_m,
                 hi[2] + shift[2]]
        if all(lower[i] - 1e-9 <= goal[i] <= upper[i] + 1e-9 for i in range(3)):
            raise LocalError('E_LOCAL_NO_ROUTE', 'goal lies inside an inflated obstacle volume')

    map_resolution = map_value['resolution']
    cell_sets = {sid: rep.surface_cells(surface) for sid, surface in surfaces.items()}

    def inside(surface, point, margin=0.0):
        return rep.contains_xy(surface, map_resolution, point[0], point[1],
                               margin, cells=cell_sets.get(surface['surface_id']))

    def safe_ceiling(point, current_sid=None):
        padding = config.radius_m + config.clearance_m
        steppable = (_steppable_surfaces(surfaces, current_sid, max_step, max_slope, config.radius_m)
                     if current_sid is not None else set())
        tol = config.support_tol_m
        return not any(not (s['surface_id'] == current_sid and abs(point[2] - s['z_reference']) <= tol)
                       and not (s['surface_id'] in steppable and _near_boundary(s, point, padding))
                       and inside(s, point, margin=-padding)
                       and point[2] + tol < s['z_reference']
                       and s['z_reference'] - s.get('thickness_m', 0.) - point[2] < required_ceiling
                       for s in map_value['surfaces'])

    def assoc_info(point, tol):
        return associate(map_value, point, config, tol=tol,
                         surfaces=surfaces, cell_sets=cell_sets,
                         map_version=map_value.get('map_version'))

    def resolve(point, label):
        # ★起点是**估计位姿**，用 support_assoc_tol_m 关联；目标是标称路径点，仍用几何容差
        #   support_tol_m★（2026-11-27 裁决：两类容差分离，几何/碰撞/净空判据一律不动）。
        tol = config.support_assoc_tol_m if label == 'start' else None
        chosen = _support_at(map_value, point, config, surfaces, cell_sets, tol=tol)
        if chosen is None:
            info = associate(map_value, point, config,
                             tol=config.support_assoc_tol_m if label == 'start' else None,
                             map_version=map_value.get('map_version'))
            raise LocalError('E_LOCAL_SUPPORT',
                             '%s requires a matching z support | z=%s z_tol=%.6g 候选=%d 歧义=%s 地图=%s'
                             % (label, round(point[2], 4),
                                config.support_assoc_tol_m if label == 'start' else config.support_tol_m,
                                len(info['candidates']), info['ambiguous'], info['map_version']))
        if not safe_ceiling(point, chosen['surface_id']):
            raise LocalError('E_LOCAL_CLEARANCE', label + ' is below a low ceiling')
        return chosen['surface_id'], assoc_info(point, tol)

    first_sid, start_assoc = resolve(start, 'start')
    goal_sid, goal_assoc = resolve(goal, 'goal')
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
    # Anchor the lattice to the map frame: a pose-anchored lattice can strand a
    # replan between two feasible transfer points of a narrow step.
    grid_x = map_value['origin'][0] + round(
        (start[0] - map_value['origin'][0]) / config.resolution) * config.resolution
    grid_y = map_value['origin'][1] + round(
        (start[1] - map_value['origin'][1]) / config.resolution) * config.resolution
    reach_cells = int(math.ceil(config.max_speed_mps * config.tick_s / config.resolution))
    offsets = [(dx, dy) for dx in range(-reach_cells, reach_cells + 1)
               for dy in range(-reach_cells, reach_cells + 1)]

    def static_collision(a, b, source, target):
        # Explicit support-edge transfers are authorized contact transitions,
        # not a permission to cross any unrelated slab. Same-surface motion
        # must clear every overlying slab along the complete swept body, except
        # the next tread of a declared step/slope edge (the stair approach).
        contact_transfer = source != target and (source, target) in allowed
        steppable = _steppable_surfaces(surfaces, source, max_step, max_slope,
                                        config.radius_m)
        for surface in map_value['surfaces']:
            if contact_transfer and surface['surface_id'] in (source, target):
                continue
            if (surface['surface_id'] in steppable and surface['surface_id'] != source
                    and _near_boundary(surface, b, required_ceiling)):
                continue
            top = surface['z_reference']
            if top <= min(a[2], b[2]) + SUPPORT_TOL_S:
                continue
            if _surface_blocks(map_value, surface, a, b, config, required_ceiling,
                               cells=cell_sets.get(surface['surface_id'])):
                return True
        return False

    def xyz(node):
        if node == first:
            return start
        if node == last:
            return goal
        return (grid_x + node[1] * config.resolution,
                grid_y + node[2] * config.resolution, surfaces[node[0]]['z_reference'])

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
        can_hold = not hit((*point, stamp), hold)
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
            if static_collision(a, b, node[0], other[0]) or hit(a, b):
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
    # If the local horizon cannot reach the goal and a static obstacle blocks
    # the direct goal line, stop now instead of driving up to the obstacle and
    # refusing at centimetre range.  A detour that actually reaches the goal is
    # unaffected; moving obstacles remain the time-aware search's job.
    if _distance(points[-1], goal) > config.goal_tolerance_m:
        last = points[-1]
        for lo, hi, velocity, epoch, active_from, active_until in moving:
            if any(abs(v) > 0 for v in velocity) or not active_from <= now <= active_until:
                continue
            if not _swept_hit((*start, now), (*goal, now + 1.),
                              (lo, hi, velocity, epoch, active_from, active_until), config):
                continue
            # Project the blocker onto the goal-line axis and its lateral
            # perpendicular; only a plan still inside the blocker's lateral
            # span and short of its front is a head-on refusal.
            dx, dy = goal[0] - start[0], goal[1] - start[1]
            if abs(dy) >= abs(dx):
                lateral, forward = 0, 1
            else:
                lateral, forward = 1, 0
            pad_x = config.radius_m + config.clearance_m
            inside_lateral = (lo[lateral] - pad_x - 1e-9 <= last[lateral]
                              <= hi[lateral] + pad_x + 1e-9)
            if goal[forward] >= start[forward]:
                before_front = last[forward] < hi[forward] + pad_x
            else:
                before_front = last[forward] > lo[forward] - pad_x
            if inside_lateral and before_front:
                raise LocalError('E_LOCAL_NO_ROUTE',
                                 'static obstacle blocks the direct goal line')
    # A terminal hold makes stale command cutoff explicit and permits decel
    # checking. Keep it just inside the horizon: a full tick can exceed the
    # checker's time budget through floating-point roundoff at some start times.
    hold_s = max(1e-6, min(config.tick_s, config.horizon_s - (points[-1][3] - now)) - 1e-6)
    points.append((*points[-1][:3], points[-1][3] + hold_s))
    layers = [state[0][0] for state in chain] + [chain[-1][0][0]]
    try:
        verdict = checker.check(map_value, points, z_tol=config.support_tol_m, min_clearance=required_ceiling,
                                max_step=config.max_speed_mps * config.tick_s + 1e-9,
                                time_budget=config.horizon_s, expected_map_version=map_value['map_version'],
                                max_step_height=config.max_step_m,
                                robot_radius=config.radius_m, robot_height=config.height_m)
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
    # ★契约分层（2026-11-27）★：把"估计位姿 / 关联结果 / 规划接触点 / 不确定度 / 轨迹"
    # 分开表达。接触点只用于规划层与日志，**不覆盖** estimated_pose；
    # 轨迹、碰撞、净空、制动仍按原几何判据计算，未因关联而放宽。
    contact_pose = (start[0], start[1], start_assoc.get('z_reference', start[2])
                    if start_assoc.get('valid') else start[2])
    first_pt = points[0][:3] if points else None
    return {'ok': True, 'reason': 'arrived' if _distance(points[-1], goal) <= config.goal_tolerance_m else 'horizon',
            'estimated_pose': start,
            'support_association': {'start': start_assoc, 'goal': goal_assoc},
            'contact_pose': contact_pose,
            'pose_uncertainty': {'support_assoc_tol_m': config.support_assoc_tol_m,
                                 'support_tol_m': config.support_tol_m},
            'trajectory_first_matches_contact': bool(
                first_pt is not None
                and abs(first_pt[0] - contact_pose[0]) <= 1e-9
                and abs(first_pt[1] - contact_pose[1]) <= 1e-9
                and abs(first_pt[2] - contact_pose[2]) <= 1e-6),
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
