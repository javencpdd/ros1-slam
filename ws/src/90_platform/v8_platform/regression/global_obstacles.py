"""Derived SC02/05/06 static-box cases and an independent continuous route check.

No ROS or dynamics claims. The evaluator uses six box faces, not the planner's
slab-intersection function, and never edits the candidate route.
"""
import argparse
import copy
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
for folder in ('contracts', 'scenarios', 'validator', 'representation'):
    sys.path.insert(0, str(ROOT / folder))
import validate as contract
import trajectory_checker as checker


def segment_hits_box(a, b, lower, upper):
    """Independently intersect the six faces and endpoints of a closed box."""
    def inside(point):
        return all(lower[i] <= point[i] <= upper[i] for i in range(3))
    if inside(a) or inside(b):
        return True
    for axis in range(3):
        delta = b[axis] - a[axis]
        if delta == 0:
            continue
        for face in (lower[axis], upper[axis]):
            t = (face - a[axis]) / delta
            if 0 <= t <= 1:
                point = [a[i] + t * (b[i] - a[i]) for i in range(3)]
                if all(lower[i] - 1e-12 <= point[i] <= upper[i] + 1e-12
                       for i in range(3) if i != axis):
                    return True
    return False


def evaluate_route(mapping, route, radius, height, start=None, goal=None):
    """Support/edge check plus independent swept body/box intersection."""
    contract.validate('map', mapping, allow_obstacles=True)
    failures = []
    points = route.get('path', [])
    if not points or any(len(p) != 4 or any(not isinstance(v, (int, float))
                      or isinstance(v, bool) or not math.isfinite(v) for v in p[:3])
                         for p in points):
        return {'ok': False, 'failures': ['empty_or_invalid_path']}
    # Project only the support component for the v1 checker. Obstacles are
    # checked separately below; this projection is never given to a planner.
    support = copy.deepcopy(mapping)
    support['schema_version'] = 1
    support.pop('obstacles', None)
    support_verdict = checker.check(support, [p[:3] for p in points], z_tol=1e-6)
    failures.extend(support_verdict['violations'])
    surfaces = {s['surface_id']: s for s in mapping['surfaces']}
    edges = set()
    for surface in mapping['surfaces']:
        for edge in surface['edges']:
            edges.add((surface['surface_id'], edge['to']))
            if edge['bidirectional']:
                edges.add((edge['to'], surface['surface_id']))
    for index, point in enumerate(points):
        surface = surfaces.get(point[3])
        if (surface is None or surface['traversability'] != 'supported'
                or abs(point[2] - surface['z_reference']) > 1e-9
                or not surface['footprint']['min_x'] <= point[0] <= surface['footprint']['max_x']
                or not surface['footprint']['min_y'] <= point[1] <= surface['footprint']['max_y']):
            failures.append(['surface_identity', index])
        if index and points[index - 1][3] != point[3] and (points[index - 1][3], point[3]) not in edges:
            failures.append(['surface_transition', index])
    for label, expected, actual in (('start', start, points[0]), ('goal', goal, points[-1])):
        if expected is not None and math.dist(expected, actual[:3]) > 1e-9:
            failures.append([label + '_connector_missing'])
    for obstacle in mapping.get('obstacles', []):
        lower = [obstacle['min'][0] - radius, obstacle['min'][1] - radius,
                 obstacle['min'][2] - height]
        upper = [obstacle['max'][0] + radius, obstacle['max'][1] + radius,
                 obstacle['max'][2] - 1e-9]
        for index, (a, b) in enumerate(zip(points, points[1:] or points)):
            if segment_hits_box(a, b, lower, upper):
                failures.append(['obstacle', obstacle['obstacle_id'], index])
    return {'ok': not failures, 'failures': failures, 'checked_points': len(points),
            'checked_segments': max(0, len(points) - 1)}


def box(name, lower, upper):
    return {'obstacle_id': name, 'min': list(lower), 'max': list(upper)}


def derived_case(scenario_id, variant):
    import scenarios
    case = scenarios.scenario(scenario_id)
    mapping = copy.deepcopy(case['map'])
    mapping['schema_version'] = 2
    mapping['map_id'] += '_' + scenario_id.lower() + '_' + variant
    mapping['obstacles'] = []
    if variant == 'detour':
        volumes = {
            'SC02': box('mezz_box', (2.30, 1.35, .6), (2.70, 2.65, 1.2)),
            'SC05': box('bridge_box', (-.2, -.25, .6), (.2, .25, 1.0)),
            'SC06': box('ground_box', (.4, -.6, 0), (1.1, .6, .4)),
        }
        mapping['obstacles'] = [volumes[scenario_id]]
    elif variant == 'blocked':
        if scenario_id == 'SC05':
            mapping['obstacles'] = [box('bridge_barrier', (-.1, -1.2, .6), (.1, 1.2, 1.3))]
        else:
            # Ground overlaps the mezzanine in SC02/06: a deck-only box can
            # be bypassed underneath, so this derived wall spans both layers.
            mapping['obstacles'] = [box('both_layers_barrier', (.2, -5.5, -.05), (.4, 5.5, 1.3))]
    elif variant != 'clear':
        raise ValueError('unknown variant ' + variant)
    return mapping, case['start'], case['goal']


def run():
    sys.path.insert(0, str(ROOT / 'planner'))
    import support_planner
    rows = []
    radius, height = .12, .35  # declared experiment geometry, not Lite3 capability
    for sid in ('SC02', 'SC05', 'SC06'):
        baseline = None
        for variant in ('clear', 'detour', 'blocked'):
            mapping, start, goal = derived_case(sid, variant)
            row = {'scenario': sid, 'variant': variant, 'map': mapping,
                   'start': start, 'goal': goal, 'expected': 'E_PLAN_NO_ROUTE'
                   if variant == 'blocked' else 'route'}
            try:
                route = support_planner.plan(mapping, start, goal, robot_radius_m=radius,
                                             robot_height_m=height)
            except support_planner.PlannerError as error:
                row.update(ok=variant == 'blocked' and error.code == 'E_PLAN_NO_ROUTE',
                           reason=error.code, detail=error.detail, route=None)
            else:
                evaluation = evaluate_route(mapping, route, radius, height, start, goal)
                again = support_planner.plan(mapping, start, goal, robot_radius_m=radius,
                                             robot_height_m=height)
                changed = variant != 'detour' or route['path'] != baseline
                row.update(ok=variant != 'blocked' and evaluation['ok'] and changed and route == again,
                           reason='route', route=route, evaluation=evaluation,
                           deterministic=route == again, changed_from_clear=changed)
                if variant == 'clear':
                    baseline = route['path']
                if variant == 'detour':
                    control = evaluate_route(mapping, {'path': baseline}, radius, height, start, goal)
                    row['unaware_route_evaluation'] = control
                    row['ok'] = row['ok'] and not control['ok']
            rows.append(row)
    return {'schema': 'v8-global-obstacles/1', 'scope': {'hardware': False, 'dynamics': False,
            'derived_scenarios': True, 'robot_geometry': 'declared_experiment_only'},
            'robot_radius_m': radius, 'robot_height_m': height, 'cases': rows,
            'passed': sum(row['ok'] for row in rows), 'total': len(rows),
            'ok': all(row['ok'] for row in rows)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    result = run()
    Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({key: result[key] for key in ('ok', 'passed', 'total')}))
    return 0 if result['ok'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
