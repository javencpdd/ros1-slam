"""Versioned GlobalRoute -> LocalTrajectory handoff contract (CAP-05 design only).

No planner calls this module yet. It makes the missing online handoff explicit:
the complete route, per-point surface identity when available, transition kinds,
capability/map versions and a decomposable cost. Unknown risk stays null.
"""
SCHEMA_VERSION = 1
TRANSITION_KINDS = {'flat', 'slope', 'step', 'terrain'}


class RouteError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


def _require(value, keys):
    if not isinstance(value, dict):
        raise RouteError('E_ROUTE_TYPE', 'route must be an object')
    missing = [key for key in keys if key not in value]
    if missing:
        raise RouteError('E_ROUTE_MISSING', 'missing %s' % missing)


def validate_route(value):
    _require(value, ('schema_version', 'backend', 'map', 'capability', 'path',
                     'transitions', 'cost_decomposition'))
    if value['schema_version'] != SCHEMA_VERSION:
        raise RouteError('E_ROUTE_VERSION', 'schema_version must be %d' % SCHEMA_VERSION)
    if value['backend'] not in ('support', 'far'):
        raise RouteError('E_ROUTE_BACKEND', 'unsupported backend %r' % value['backend'])
    _require(value['map'], ('map_id', 'map_version'))
    _require(value['capability'], ('robot_id', 'capability_version'))
    if not isinstance(value['path'], list) or len(value['path']) < 2:
        raise RouteError('E_ROUTE_PATH', 'complete route needs at least two points')
    for index, point in enumerate(value['path']):
        _require(point, ('xyz',))
        if len(point['xyz']) != 3:
            raise RouteError('E_ROUTE_PATH', 'point %d xyz must have three numbers' % index)
        if value['backend'] == 'support' and not point.get('surface_id'):
            raise RouteError('E_ROUTE_SURFACE', 'support route point %d lacks surface_id' % index)
    transitions = value['transitions']
    if not isinstance(transitions, list):
        raise RouteError('E_ROUTE_TRANSITION', 'transitions must be a list')
    last = -1
    for item in transitions:
        _require(item, ('from_index', 'to_index', 'kind'))
        if item['kind'] not in TRANSITION_KINDS:
            raise RouteError('E_ROUTE_TRANSITION', 'unknown kind %r' % item['kind'])
        if not (0 <= item['from_index'] < item['to_index'] < len(value['path'])):
            raise RouteError('E_ROUTE_TRANSITION', 'transition indexes out of order')
        if item['from_index'] < last:
            raise RouteError('E_ROUTE_TRANSITION', 'transitions overlap/backtrack')
        last = item['to_index']
        if value['backend'] == 'support':
            source = value['path'][item['from_index']]['surface_id']
            target = value['path'][item['to_index']]['surface_id']
            if item.get('from_surface') != source or item.get('to_surface') != target:
                raise RouteError('E_ROUTE_TRANSITION', 'transition surface mismatch at %d' % item['from_index'])
            if item['kind'] == 'flat' and source != target:
                raise RouteError('E_ROUTE_TRANSITION', 'flat transfer cannot change surface')
            if item['kind'] in ('slope', 'step') and source == target:
                raise RouteError('E_ROUTE_TRANSITION', 'slope/step must change surface')
    cost = value['cost_decomposition']
    _require(cost, ('distance_m', 'time_s', 'risk_expected_m', 'risk_known'))
    if cost['risk_known'] and cost['risk_expected_m'] is None:
        raise RouteError('E_ROUTE_COST', 'known risk needs a value')
    if not cost['risk_known'] and cost['risk_expected_m'] is not None:
        raise RouteError('E_ROUTE_COST', 'unknown risk must stay null')
    return True


def from_support_plan(plan_result, robot_id, capability_version, transition_kinds=None):
    transitions = []
    changes = [index for index in range(1, len(plan_result['path']))
               if plan_result['path'][index][3] != plan_result['path'][index - 1][3]]
    for index, change in enumerate(changes):
        source = plan_result['path'][change - 1][3]
        target = plan_result['path'][change][3]
        kind = (transition_kinds or {}).get((source, target), 'terrain')
        if kind not in TRANSITION_KINDS:
            raise RouteError('E_ROUTE_TRANSITION', 'unknown supplied kind %r' % kind)
        transitions.append({'from_index': change - 1, 'to_index': change,
                            'from_surface': source, 'to_surface': target, 'kind': kind})
    route = {
        'schema_version': SCHEMA_VERSION,
        'backend': 'support',
        'map': {'map_id': plan_result['map_id'], 'map_version': plan_result['map_version']},
        'capability': {'robot_id': robot_id, 'capability_version': capability_version},
        'path': [{'xyz': list(point[:3]), 'surface_id': point[3]} for point in plan_result['path']],
        'transitions': transitions,
        'cost_decomposition': {'distance_m': plan_result.get('length'),
                               'time_s': None, 'risk_expected_m': None, 'risk_known': False},
    }
    validate_route(route)
    return route
