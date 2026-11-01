"""Resolve (x, y[, z], surface_hint) to one support surface.

The planner ultimately needs a 3D point on one surface. Single-layer maps can
infer z from the only surface at x/y; multi-layer maps must provide z or a
surface hint. Ambiguity is explicit, never silently guessed.
"""
import copy


class SurfaceResolutionError(Exception):
    def __init__(self, code, detail, candidates=None):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail
        self.candidates = list(candidates or [])


def supported_surfaces(map_value):
    return [s for s in map_value.get('surfaces', [])
            if s.get('traversability') == 'supported']


def candidates_at(map_value, x, y):
    result = []
    for surface in supported_surfaces(map_value):
        box = surface['footprint']
        if box['min_x'] <= x <= box['max_x'] and box['min_y'] <= y <= box['max_y']:
            result.append(surface)
    return sorted(result, key=lambda s: s['z_reference'])


def _candidate_view(surfaces):
    return [{'surface_id': s['surface_id'], 'z_reference': s['z_reference']}
            for s in surfaces]


def resolve_surface(map_value, x, y, z=None, hint=None):
    """Return (surface, z_reference). `hint` wins over z for layer selection."""
    candidates = candidates_at(map_value, x, y)
    if not candidates:
        raise SurfaceResolutionError('E_SURFACE_NOT_FOUND',
                                     'no supported surface contains (%.3f, %.3f)' % (x, y))
    if hint is not None:
        matches = [s for s in candidates if s['surface_id'] == hint]
        if not matches:
            raise SurfaceResolutionError(
                'E_SURFACE_HINT', 'hint %s does not contain (%.3f, %.3f)' % (hint, x, y),
                _candidate_view(candidates))
        surface = matches[0]
        if z is not None and abs(float(z) - surface['z_reference']) > .5:
            raise SurfaceResolutionError(
                'E_SURFACE_HINT_Z', 'hint %s z %.3f differs from %.3f'
                % (hint, float(z), surface['z_reference']), _candidate_view(candidates))
        return surface, surface['z_reference']
    if z is None:
        if len(candidates) == 1:
            return candidates[0], candidates[0]['z_reference']
        raise SurfaceResolutionError(
            'E_SURFACE_AMBIGUOUS',
            'multiple surfaces at (%.3f, %.3f); supply z or surface_hint' % (x, y),
            _candidate_view(candidates))
    surface = min(candidates, key=lambda item: abs(item['z_reference'] - float(z)))
    return surface, surface['z_reference']


def resolve_task(task, map_value):
    """Fill task start/goal z and surface hints; return (resolved_task, meta).

    The input task may carry null z for start/goal. The output is a copy with
    numeric z values and an added `resolved` metadata object. The original task
    contract's strict 3D goal requirement is applied by the caller after this
    convenience step.
    """
    resolved = copy.deepcopy(task)
    start = resolved['start']['xyz']
    sx, sy, sz = start
    start_surface, start_z = resolve_surface(map_value, float(sx), float(sy),
                                            None if sz is None else float(sz))
    resolved['start']['xyz'] = [float(sx), float(sy), start_z]
    goal = resolved['goal']
    gx, gy, gz = goal['xyz']
    hint = goal.get('surface_hint')
    mode = goal.get('surface_disambiguation', 'not_applicable')
    if mode == 'explicit_hint' and not hint:
        raise SurfaceResolutionError('E_TASK_SURFACE_AMBIGUOUS',
                                     'explicit_hint requires surface_hint')
    goal_surface, goal_z = resolve_surface(
        map_value, float(gx), float(gy), None if gz is None else float(gz), hint)
    resolved['goal']['xyz'] = [float(gx), float(gy), goal_z]
    meta = {
        'start_surface': start_surface['surface_id'],
        'start_surface_z': start_z,
        'goal_surface': goal_surface['surface_id'],
        'goal_surface_z': goal_z,
        'goal_candidates': _candidate_view(candidates_at(map_value, float(gx), float(gy))),
    }
    resolved['resolved'] = meta
    return resolved, meta


def pick_goal(map_value, point, hint=None):
    """Operator click -> (x,y,z,surface) or an explicit ambiguity error."""
    x, y, z = point
    surface, surface_z = resolve_surface(map_value, float(x), float(y),
                                         None if z is None else float(z), hint)
    return {'x': float(x), 'y': float(y), 'z': surface_z,
            'surface_id': surface['surface_id']}
