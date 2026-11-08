"""Private in-process continuation geometry, never a planner/JSON proof.

The ROS controller owns evidence of actual execution and all epoch bindings.
This object only permits the immediate same-landed leg of that same result.
It is not an isolation boundary against arbitrary Python code in this process.
"""
import math
import support_map as rep

# Cell edges are produced as index*resolution; an exact lattice pose can sit
# 1e-16 behind the computed plane. This only absorbs that representation gap;
# clear_points keeps its strict, epsilon-free clearance comparison.
BOUNDARY_EPS_M = 1e-9


def simulation_contact_egress(value, sim_clock, profile=None, body_enabled=False):
    if type(value) is not bool:
        raise ValueError('sim_contact_egress must be bool')
    if value and (sim_clock is not True or profile is not None or body_enabled):
        raise ValueError('sim_contact_egress requires simulation, no profile and no plain Twist')
    return value


def _finite(point):
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
               for v in point[:3])


def _bounds(cells, resolution):
    return (min(i * resolution for i, j in cells), max((i + 1) * resolution for i, j in cells),
            min(j * resolution for i, j in cells), max((j + 1) * resolution for i, j in cells))


class ContactEgress:
    """Runtime-owned geometry; the constructor does not attest execution."""
    @classmethod
    def create(cls, map_value, result, config):
        try:
            return cls(map_value, result, config)
        except (ValueError, TypeError, KeyError, IndexError, OverflowError, StopIteration):
            return None

    def __init__(self, map_value, result, config):
        self.owner, self.map_object, self.config = result, map_value, config
        self.active, self.permission_closed = True, False
        self.index = 1
        self.identity = (map_value['map_id'], map_value['map_version'])
        self.signature = self._signature(result)
        source, landed, next_sid = result['surfaces'][:3]
        self.source, self.landed = source, landed
        surfaces = {s['surface_id']: s for s in map_value['surfaces']}
        self.old, self.target = surfaces[source], surfaces[landed]
        edge = next((e for s in surfaces.values() for e in s['edges']
                     if (s['surface_id'] == source and e['to'] == landed)
                     or (s['surface_id'] == landed and e['to'] == source and e['bidirectional'])), None)
        dz = self.old['z_reference'] - self.target['z_reference']
        a, b = result['trajectory'][1:3]
        if (source == landed or next_sid != landed or edge is None or edge['kind'] != 'step'
                or not 0. < dz <= min(config.max_step_m, edge['max_step_height_m'])
                or self.old['traversability'] != 'supported' or self.target['traversability'] != 'supported'
                or not all(_finite(p) for p in result['trajectory'][:3])
                or sum((a[i] - b[i]) ** 2 for i in range(3)) <= 1e-24):
            raise ValueError('not a descending step followed by a nonzero landed leg')
        self.old_cells, self.target_cells = rep.surface_cells(self.old), rep.surface_cells(self.target)
        if not self.old_cells or not self.target_cells:
            raise ValueError('exact cells are required for contact egress')
        self.resolution = map_value['resolution']
        self.old_bounds = _bounds(self.old_cells, self.resolution)
        target_bounds = _bounds(self.target_cells, self.resolution)
        candidates = []
        for axis in (0, 1):
            lo, hi = self.old_bounds[2 * axis:2 * axis + 2]
            tlo, thi = target_bounds[2 * axis:2 * axis + 2]
            if hi <= tlo:
                candidates.append((axis, 1., hi))
            if thi <= lo:
                candidates.append((axis, -1., lo))
        if not candidates:
            # A lower support can enclose the higher tread (ground under a
            # step). Its face is still one of the old tread's own boundaries;
            # the landed leg selects the one the robot is leaving.
            for axis in (0, 1):
                lo, hi = self.old_bounds[2 * axis:2 * axis + 2]
                candidates.extend(((axis, 1., hi), (axis, -1., lo)))
        self.axis, self.side, self.plane = next(
            p for p in candidates
            if p[1] * b[p[0]] > p[1] * p[2] + config.radius_m + config.clearance_m)
        if (not self.outward(a, b, 0.) or not all(self.on_target(p) for p in (a, b))
                or not all(self.outside_old(p) for p in (a, b))):
            raise ValueError('next leg has no proved outward exit')

    @staticmethod
    def _signature(result):
        return (tuple(tuple(p) for p in result['trajectory'][:3]), tuple(result['surfaces'][:3]),
                result['map_id'], result['map_version'], result['created_at'], result['valid_until'])

    def valid_for(self, map_value, result, config):
        try:
            return (self.active and map_value is self.map_object and result is self.owner
                    and self.identity == (map_value['map_id'], map_value['map_version'])
                    and config == self.config and self.signature == self._signature(result))
        except (KeyError, TypeError, ValueError):
            return False

    def retire(self):
        self.active = False
        self.permission_closed = True

    def close_permission(self):
        self.permission_closed = True

    def on_target(self, point):
        return (_finite(point) and abs(point[2] - self.target['z_reference']) <= self.config.support_tol_m
                and rep.contains_xy(self.target, self.resolution, point[0], point[1],
                                    self.config.radius_m, cells=self.target_cells))

    def outside_old(self, point):
        return (self.on_target(point)
                and not rep.contains_xy(self.old, self.resolution, point[0], point[1],
                                        cells=self.old_cells))

    def at_or_beyond(self, point):
        return _finite(point) and self.side * (point[self.axis] - self.plane) >= -BOUNDARY_EPS_M

    def outside_segment(self, a, b):
        """No sampled pose of the swept leg enters the old tread's own cells."""
        if not _finite(a) or not _finite(b):
            return False
        times = {0., 1.}
        normalized = [tuple(p[axis] / self.resolution + 1e-9 for axis in (0, 1))
                      for p in (a, b)]
        indices = [tuple(math.floor(v) for v in p) for p in normalized]
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
        samples = ordered + [(x + y) / 2 for x, y in zip(ordered, ordered[1:])]
        for fraction in samples:
            x = a[0] + (b[0] - a[0]) * fraction
            y = a[1] + (b[1] - a[1]) * fraction
            if rep.contains_xy(self.old, self.resolution, x, y, cells=self.old_cells):
                return False
        return True

    def outward(self, a, b, uncertainty):
        if not _finite(a) or not _finite(b) or not math.isfinite(uncertainty) or uncertainty < 0.:
            return False
        if not self.on_target(a) or not self.on_target(b):
            return False
        if any(self.side * (p[self.axis] - self.plane) < -BOUNDARY_EPS_M for p in (a, b)):
            return False
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if not math.isfinite(length):
            return False
        if length <= BOUNDARY_EPS_M:
            return True
        if self.side * (dx, dy)[self.axis] <= 0.:
            return False
        # The inflated collision radius carries uncertainty; this predicate only
        # proves that a nonzero checked leg is monotonically leaving the old face.
        return True

    def clear_points(self, points, uncertainty):
        if not math.isfinite(uncertainty) or uncertainty < 0. or not all(_finite(p) for p in points):
            return False
        padding = self.config.radius_m + self.config.clearance_m + uncertainty
        # Direct strict comparison: equality is NOT clearance; no new epsilon.
        return min(self.side * p[self.axis] for p in points) > self.side * self.plane + padding

    def full_clear(self, raw, predicted, previous, uncertainty):
        tail = tuple(predicted[i] + previous[i] * self.config.command_latency_bound_s for i in range(3))
        return self.clear_points((raw, predicted, tail), uncertainty)

    def window(self, raw, predicted, previous, uncertainty, supported_segment):
        tail = tuple(predicted[i] + previous[i] * self.config.command_latency_bound_s for i in range(3))
        target = self.owner['trajectory'][2][:3]
        segments = ((raw, predicted), (predicted, tail), (predicted, target))
        if not all(supported_segment(self.target, self.resolution, a, b, self.config,
                                     cells=self.target_cells) for a, b in segments):
            return None
        if not all(self.at_or_beyond(p) for p in (raw, predicted, tail, target)):
            return None
        if not all(self.outside_segment(a, b) for a, b in segments):
            return None
        clear = self.full_clear(raw, predicted, previous, uncertainty)
        if self.permission_closed or clear:
            if not clear or not self.clear_points((predicted, target), uncertainty):
                return None
            return ContactWindow(self, uncertainty, False)
        # Only the committed leg must leave the face monotonically; raw/pred/tail
        # are terminal corrections that stay outside the old cells and face.
        if not self.outward(predicted, target, uncertainty):
            return None
        return ContactWindow(self, uncertainty, True)


class ContactWindow:
    """One checked tracking call; cannot grant contact after context closure."""
    def __init__(self, context, uncertainty, allow_old):
        self.context, self.uncertainty, self.allow_old = context, uncertainty, allow_old

    def permits(self, map_value, surface, a, b, config):
        c = self.context
        return (self.allow_old and c.active and not c.permission_closed
                and map_value is c.map_object and surface is c.old
                and config.radius_m == c.config.radius_m + self.uncertainty
                and config.height_m == c.config.height_m and config.clearance_m == c.config.clearance_m
                and c.outward(a, b, self.uncertainty))
