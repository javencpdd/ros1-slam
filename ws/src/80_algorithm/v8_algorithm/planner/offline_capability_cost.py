"""CAP-04 offline capability cost evaluation over the existing Support A*.

The online A* cost stays untouched: horizontal distance plus the existing 0.1
per explicit layer transfer. This module only filters hard capability violations
before calling `support_planner.plan`, then reports distance/time/risk breakdowns
for the baseline route and edge-exclusion alternatives.
"""
import copy
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str((Path(__file__).resolve().parent / '../../..' / '90_platform/v8_platform/validator').resolve()))
sys.path.insert(0, str((Path(__file__).resolve().parent / '../../..' / '90_platform/v8_platform/contracts').resolve()))
import enforcement  # noqa: E402
import support_planner  # noqa: E402


def _edge_allowed(edge, capabilities):
    if edge.get('kind') == 'slope':
        limit = capabilities.get('max_slope_deg')
        return limit is not None and float(edge.get('max_slope_deg', 0.0)) <= limit + 1e-12
    if edge.get('kind') == 'step':
        limit = capabilities.get('max_step_height_m')
        return limit is not None and float(edge.get('max_step_height_m', 0.0)) <= limit + 1e-12
    return True


def filter_map(value, capabilities):
    """Remove declared transitions that exceed the authorized capability envelope."""
    filtered = copy.deepcopy(value)
    removed = []
    for surface in filtered['surfaces']:
        kept = []
        for edge in surface.get('edges', []):
            if _edge_allowed(edge, capabilities):
                kept.append(edge)
            else:
                removed.append({'from': surface['surface_id'], 'to': edge['to'],
                                'kind': edge.get('kind')})
        surface['edges'] = kept
    return filtered, removed


def _route_cost(route, speed_mps, success_rate):
    horizontal = 0.0
    for left, right in zip(route['path'], route['path'][1:]):
        horizontal += ((left[0] - right[0]) ** 2 + (left[1] - right[1]) ** 2) ** 0.5
    horizontal += 0.1 * len(route['transitions'])
    time_s = horizontal / speed_mps if speed_mps else None
    known = success_rate is not None
    risk = horizontal * (1.0 - success_rate) if known else None
    return {
        'distance_m': round(horizontal, 6),
        'time_s': round(time_s, 6) if time_s is not None else None,
        'risk_expected_m': round(risk, 6) if risk is not None else None,
        'risk_known': known,
        'transitions': list(route['transitions']),
    }


def _key(route):
    return tuple((item['from'], item['to']) for item in route['transitions']) + (tuple(route['path'][-1:]),)


def _plan_or_none(map_value, start, goal, **kwargs):
    try:
        return support_planner.plan(map_value, start, goal, **kwargs)
    except support_planner.PlannerError as error:
        return error.code


def evaluate(map_value, start, goal, profile, safety_limits=None, plan_kwargs=None):
    query = enforcement.query_capabilities(profile, safety_limits)
    plan_kwargs = dict(plan_kwargs or {})
    filtered, removed = filter_map(map_value, query['capabilities'])
    base = _plan_or_none(filtered, start, goal, **plan_kwargs)
    result = {
        'capability_query': query,
        'removed_transitions': removed,
        'risk_known': query['success_rate'] is not None,
        'candidate_routes': [],
        'verdict': 'NO_ROUTE',
    }
    if not isinstance(base, dict):
        result['failure'] = base
        return result

    routes = [('baseline', base)]
    # Alternatives: remove one explicit transition from the hard-filtered map.
    for surface in filtered['surfaces']:
        for edge in list(surface.get('edges', [])):
            alternate_map = copy.deepcopy(filtered)
            for candidate_surface in alternate_map['surfaces']:
                if candidate_surface['surface_id'] == surface['surface_id']:
                    candidate_surface['edges'] = [e for e in candidate_surface['edges']
                                                  if e['to'] != edge['to']]
            alternate = _plan_or_none(alternate_map, start, goal, **plan_kwargs)
            if isinstance(alternate, dict):
                routes.append(('without:%s->%s' % (surface['surface_id'], edge['to']), alternate))

    seen = set()
    for label, route in routes:
        key = _key(route)
        if key in seen:
            continue
        seen.add(key)
        cost = _route_cost(route, query['capabilities'].get('max_speed_mps'), query['success_rate'])
        result['candidate_routes'].append({'label': label, 'cost': cost,
                                           'path': route['path'], 'segments': route['segments']})
    result['candidate_routes'].sort(key=lambda item: item['cost']['distance_m'])
    result['verdict'] = 'FEASIBLE' if result['candidate_routes'] else 'NO_ROUTE'
    result['selected_baseline'] = result['candidate_routes'][0]['label'] if result['candidate_routes'] else None
    return result