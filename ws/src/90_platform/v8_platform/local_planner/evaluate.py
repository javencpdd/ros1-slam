"""Independent dense-sampling evaluator for local 3D output.

No planner import, candidate generation, slab predicate, repair or data feedback.
It consumes truth obstacle volumes and recorded output, and flags a deliberately
unsafe straight line as well as expired/mismatched output. Sampling is bounded
by explicit spatial and temporal spacing and is evidence, not an analytic proof.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'validator'))
import trajectory_checker


def evaluate(map_value, result, truth_obstacles, now=None, spacing_m=0.01, spacing_s=0.01):
    if spacing_m <= 0 or spacing_s <= 0:
        raise ValueError('sampling spacings must be positive')
    points = result.get('trajectory', [])
    failures = []
    if result.get('map_id') != map_value['map_id'] or result.get('map_version') != map_value['map_version']:
        failures.append('map_version')
    if now is not None and not result.get('created_at', math.inf) <= now < result.get('valid_until', -math.inf):
        failures.append('expired')
    config = result['config']
    try:
        checked = trajectory_checker.check(
            map_value, points, z_tol=1e-6,
            min_clearance=max(config['height_m'], config['clearance_m']),
            max_step=config['max_speed_mps'] * config['tick_s'] + 1e-9,
            time_budget=config['horizon_s'], expected_map_version=map_value['map_version'])
        failures.extend(v[0] for v in checked['violations'])
    except trajectory_checker.CheckerError as error:
        failures.append(error.code)
    samples, collisions = 0, []
    static_collisions = []
    surfaces = map_value['surfaces']
    edge_pairs = set()
    for surface in surfaces:
        for edge in surface['edges']:
            edge_pairs.add((surface['surface_id'], edge['to']))
            if edge['bidirectional']:
                edge_pairs.add((edge['to'], surface['surface_id']))

    def contact(point):
        hits = [s for s in surfaces if abs(s['z_reference'] - point[2]) <= 1e-6
                and s['footprint']['min_x'] <= point[0] <= s['footprint']['max_x']
                and s['footprint']['min_y'] <= point[1] <= s['footprint']['max_y']]
        return hits[0]['surface_id'] if len(hits) == 1 else None

    execution = result.get('execution_trace', points)
    if len(execution) < 2:
        failures.append('execution_empty')
    # Contact-event trajectory and dense body execution are different channels.
    # Verify every unsupported execution sample against an actual declared
    # transfer and its geometric segment; never project or drop that sample.
    intervals = []
    for left, right in zip(points, points[1:]):
        source, target = contact(left), contact(right)
        if source and target and source != target and (source, target) in edge_pairs:
            intervals.append((left, right, source, target))

    def transfer_at(point):
        for left, right, source, target in intervals:
            if not left[3] - 1e-9 <= point[3] <= right[3] + 1e-9:
                continue
            delta = [right[i] - left[i] for i in range(3)]
            norm = sum(v * v for v in delta)
            if norm <= 1e-15:
                continue
            fraction = sum((point[i] - left[i]) * delta[i] for i in range(3)) / norm
            residual = math.sqrt(sum((point[i] - left[i] - fraction * delta[i]) ** 2 for i in range(3)))
            if -1e-6 <= fraction <= 1 + 1e-6 and residual <= 2e-6:
                return source, target
        return None

    if 'execution_trace' in result:
        if (not execution or not points or any(abs(execution[0][i] - points[0][i]) > 1e-6 for i in range(4))
                or any(abs(execution[-1][i] - points[-1][i]) > 1e-6 for i in range(4))):
            failures.append('execution_contact_mismatch')
        for sample in execution:
            if contact(sample) is None and transfer_at(sample) is None:
                failures.append('execution_unsupported')

    for index, (a, b) in enumerate(zip(execution, execution[1:])):
        if b[3] <= a[3]:
            failures.append('execution_time')
            continue
        source, target = contact(a), contact(b)
        transfer = source is not None and target is not None and source != target and (source, target) in edge_pairs
        length = math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(3)))
        count = max(1, int(math.ceil(length / spacing_m)), int(math.ceil(abs(b[3] - a[3]) / spacing_s)))
        for sub in range(count + 1):
            fraction = float(sub) / count
            point = [a[i] + fraction * (b[i] - a[i]) for i in range(4)]
            samples += 1
            # Independent truth-map sweep sampling includes the body envelope;
            # checking only the planner's lattice endpoints missed thin roofs.
            pad = config['radius_m'] + config['clearance_m']
            declared_transfer = transfer_at(point) if 'execution_trace' in result else None
            current_contact = contact(point)
            for surface in surfaces:
                if ((transfer and surface['surface_id'] in (source, target)) or
                        (declared_transfer and surface['surface_id'] in declared_transfer)):
                    continue
                if surface['surface_id'] == current_contact:
                    continue
                top = surface['z_reference']
                if top <= point[2] + 1e-9:
                    continue
                fp = surface['footprint']
                if (fp['min_x'] - pad <= point[0] <= fp['max_x'] + pad
                        and fp['min_y'] - pad <= point[1] <= fp['max_y'] + pad
                        and top - surface.get('thickness_m', 0.) < point[2] + max(config['height_m'], config['clearance_m'])):
                    static_collisions.append({'segment': index, 'time': point[3], 'surface': surface['surface_id']})
                    break
            for obstacle in truth_obstacles:
                if not obstacle.get('active_from', -math.inf) <= point[3] <= obstacle.get('active_until', math.inf):
                    continue
                offset = [v * (point[3] - obstacle.get('stamp', 0.0))
                          for v in obstacle.get('velocity', [0.0, 0.0, 0.0])]
                lo = [obstacle['min'][i] + offset[i] for i in range(3)]
                hi = [obstacle['max'][i] + offset[i] for i in range(3)]
                pad = config['radius_m'] + config['clearance_m']
                if (lo[0] - pad <= point[0] <= hi[0] + pad
                        and lo[1] - pad <= point[1] <= hi[1] + pad
                        and lo[2] <= point[2] + config['height_m'] and point[2] <= hi[2]):
                    collisions.append({'segment': index, 'time': point[3], 'obstacle': obstacle.get('id', '')})
                    break
    if collisions:
        failures.append('dynamic_collision')
    if static_collisions:
        failures.append('static_collision')
    return {'ok': not failures, 'failures': sorted(set(failures)),
            'samples': samples, 'collisions': collisions, 'static_collisions': static_collisions,
            'sampling': {'max_spacing_m': spacing_m, 'max_spacing_s': spacing_s}}
