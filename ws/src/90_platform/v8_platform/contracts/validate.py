"""v8 契约校验器：mini JSON-Schema 子集 + 跨字段语义检查。

只支持本目录 schema 实际用到的关键字；子集外写法一律 E_SCHEMA_UNSUPPORTED，
避免把「校验器不认识」误当成「实例合规」。不依赖第三方库。
"""
import copy
import json
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCHEMAS = {'manifest': 'manifest.v1.json', 'robot': 'robot.v1.json',
           'map': 'map.v1.json', 'task': 'task.v1.json'}
KNOWN = {'title', 'description', 'type', 'required', 'properties', 'additionalProperties', 'enum', 'const',
         'pattern', 'minLength', 'minimum', 'maximum', 'items', 'minItems', 'maxItems', 'uniqueItems',
         'allOf', 'definitions', '$ref'}
EDGE_TOL = 0.05


class ContractError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


def _type_ok(value, name):
    if name == 'object':
        return isinstance(value, dict)
    if name == 'array':
        return isinstance(value, list)
    if name == 'string':
        return isinstance(value, str)
    if name == 'integer':
        return isinstance(value, int) and not isinstance(value, bool)
    if name == 'number':
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if name == 'boolean':
        return isinstance(value, bool)
    if name == 'null':
        return value is None
    raise ContractError('E_SCHEMA_UNSUPPORTED', 'type %s' % name)


def _same(left, right):
    if isinstance(left, bool) or isinstance(right, bool):
        return isinstance(left, bool) and isinstance(right, bool) and left == right
    if isinstance(right, int):
        return isinstance(left, int) and not isinstance(left, bool) and left == right
    return left == right


def _resolve(schema, root):
    seen = 0
    while '$ref' in schema:
        ref = schema['$ref']
        if not ref.startswith('#/definitions/'):
            raise ContractError('E_SCHEMA_UNSUPPORTED', '$ref %s' % ref)
        name = ref.split('/')[-1]
        if 'definitions' not in root or name not in root['definitions']:
            raise ContractError('E_SCHEMA_UNSUPPORTED', 'missing $ref target %s' % ref)
        schema = dict(root['definitions'][name], **{k: v for k, v in schema.items() if k != '$ref'})
        seen += 1
        if seen > 8:
            raise ContractError('E_SCHEMA_UNSUPPORTED', 'recursive $ref')
    return schema


def _walk(schema, value, root, path):
    schema = _resolve(schema, root)
    for keyword in schema:
        if keyword not in KNOWN:
            raise ContractError('E_SCHEMA_UNSUPPORTED', '%s keyword %s' % (path or '<root>', keyword))
    ap = schema.get('additionalProperties', True)
    if ap is not True and ap is not False:
        raise ContractError('E_SCHEMA_UNSUPPORTED', '%s nested additionalProperties' % (path or '<root>'))
    if 'type' in schema:
        names = schema['type'] if isinstance(schema['type'], list) else [schema['type']]
        if not any(_type_ok(value, name) for name in names):
            raise ContractError('E_SCHEMA_TYPE', '%s expects %s, got %s' % (path or '<root>', names, type(value).__name__))
    if 'const' in schema and not _same(value, schema['const']):
        raise ContractError('E_VERSION', '%s must equal %r' % (path or '<root>', schema['const']))
    if 'enum' in schema and not any(_same(value, item) for item in schema['enum']):
        raise ContractError('E_SCHEMA_ENUM', '%s value %r not in %s' % (path or '<root>', value, schema['enum']))
    if isinstance(value, str):
        if 'minLength' in schema and len(value.strip()) < schema['minLength']:
            raise ContractError('E_SCHEMA_BLANK', '%s is blank or shorter than %s' % (path, schema['minLength']))
        if 'pattern' in schema and not re.fullmatch(schema['pattern'], value):
            raise ContractError('E_SCHEMA_PATTERN', '%s %r does not match %s' % (path, value, schema['pattern']))
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if not math.isfinite(value):
            raise ContractError('E_SCHEMA_NONFINITE', '%s is not finite' % path)
        if 'minimum' in schema and value < schema['minimum']:
            raise ContractError('E_SCHEMA_MINIMUM', '%s %s < %s' % (path, value, schema['minimum']))
        if 'maximum' in schema and value > schema['maximum']:
            raise ContractError('E_SCHEMA_MAXIMUM', '%s %s > %s' % (path, value, schema['maximum']))
    if isinstance(value, dict):
        for name in schema.get('required', []):
            if name not in value:
                raise ContractError('E_SCHEMA_REQUIRED', '%s missing %s' % (path or '<root>', name))
        properties = schema.get('properties', {})
        if schema.get('additionalProperties') is False:
            extra = sorted(set(value) - set(properties))
            if extra:
                raise ContractError('E_SCHEMA_UNKNOWN_FIELD', '%s unknown %s' % (path or '<root>', extra))
        for name, child in value.items():
            if name in properties:
                _walk(properties[name], child, root, '%s.%s' % (path, name))
    if isinstance(value, list):
        if 'minItems' in schema and len(value) < schema['minItems']:
            raise ContractError('E_SCHEMA_MINITEMS', '%s has %s < %s' % (path, len(value), schema['minItems']))
        if 'maxItems' in schema and len(value) > schema['maxItems']:
            raise ContractError('E_SCHEMA_MAXITEMS', '%s has %s > %s' % (path, len(value), schema['maxItems']))
        if schema.get('uniqueItems'):
            seen = [json.dumps(item, sort_keys=True) for item in value]
            if len(seen) != len(set(seen)):
                raise ContractError('E_SCHEMA_UNIQUE', '%s has duplicate items' % path)
        if 'items' not in schema:
            raise ContractError('E_SCHEMA_UNSUPPORTED', '%s array without items' % path)
        for index, item in enumerate(value):
            _walk(schema['items'], item, root, '%s[%s]' % (path, index))
    for child in schema.get('allOf', []):
        _walk(child, value, root, path)


def _find_key(value, key, path=''):
    if isinstance(value, dict):
        if key in value:
            return path or '<root>'
        for name, child in value.items():
            found = _find_key(child, key, '%s.%s' % (path, name))
            if found:
                return found
    if isinstance(value, list):
        for index, item in enumerate(value):
            found = _find_key(item, key, '%s[%s]' % (path, index))
            if found:
                return found
    return ''


def _pre_map(value):
    where = _find_key(value, 'floor_id')
    if where:
        raise ContractError('E_MAP_FLOOR_ID', 'floor_id at %s cannot replace connectivity' % where)
    if _find_key(value, 'treat_unknown_as_free'):
        raise ContractError('E_MAP_UNKNOWN_AS_FREE', 'unknown cells must not be treated as free')
    kind = value.get('kind')
    if kind == 'support_map' and 'surfaces' not in value:
        raise ContractError('E_MAP_SURFACES_MISSING', 'support_map requires surfaces')
    if kind == 'occupancy_grid' and 'layers' not in value:
        raise ContractError('E_MAP_LAYERS_MISSING', 'occupancy_grid requires a layers list')


def _reachable(surfaces):
    graph = {surface['surface_id']: set() for surface in surfaces}
    for surface in surfaces:
        for edge in surface['edges']:
            graph[surface['surface_id']].add(edge['to'])
            if edge['bidirectional']:
                graph[edge['to']].add(surface['surface_id'])
    start = surfaces[0]['surface_id']
    seen, stack = {start}, [start]
    while stack:
        for nxt in graph[stack.pop()]:
            if nxt not in seen:
                seen.add(nxt)
                stack.append(nxt)
    return seen


def _manifest_v2(value):
    """Named-port invariants; v1 manifests are untouched."""
    if value.get('schema_version') != 2:
        return
    seen = set()
    for direction in ('inputs', 'outputs'):
        for port in value['ports'][direction]:
            key = (direction, port['name'])
            if key in seen:
                raise ContractError('E_MANIFEST_PORT_DUP', '%s %s' % key)
            seen.add(key)
            if direction == 'inputs' and 'command_topic' in port:
                raise ContractError('E_MANIFEST_PORT_COMMAND', 'command_topic on input %s' % port['name'])
    commands = [port['command_topic'] for port in value['ports']['outputs']
                if 'command_topic' in port]
    if len(commands) != len(set(commands)):
        raise ContractError('E_MANIFEST_COMMAND_DUP', 'duplicate command topic in one component')
    for port in value['ports']['inputs'] + value['ports']['outputs']:
        schema = port.get('schema', '')
        if schema.startswith('support_map/') and value['dimensions'] != 3:
            raise ContractError('E_MANIFEST_DIM', 'support_map port requires dimensions 3')
        if schema.startswith('occupancy_grid/') and value['dimensions'] != 2:
            raise ContractError('E_MANIFEST_DIM', 'occupancy_grid port requires dimensions 2')


def normalize_manifest(value):
    """v2 -> v1-compatible view: derive io from ports + dimensions.

    v1 manifests are returned unchanged. The derived map_schema only uses the
    frozen v1 enum (support_map/* collapses to support_map/v1); exact port
    schema keeps the version for graph checks.
    """
    if not isinstance(value, dict) or value.get('schema_version') != 2:
        return value
    ports = value['ports']
    schemas = [port.get('schema', '') for port in ports['inputs'] + ports['outputs']]
    occupancy = any(schema.startswith('occupancy_grid/') for schema in schemas)
    support = any(schema.startswith('support_map/') for schema in schemas)
    if occupancy and support:
        raise ContractError('E_MANIFEST_MAP_MIXED',
                            'occupancy_grid and support_map ports cannot mix in one component')
    map_schema = ('occupancy_grid/v1' if occupancy else
                  'support_map/v1' if support else 'not_applicable')
    normalized = copy.deepcopy(value)
    normalized['io'] = {
        'map_schema': map_schema,
        'dimensions': value['dimensions'],
        'inputs': ['%s（%s）' % (port['type'], port['name']) for port in ports['inputs']],
        'outputs': ['%s（%s）' % (port['type'], port['name']) for port in ports['outputs']],
    }
    return normalized


def semantic(kind, value):
    if kind == 'manifest':
        _manifest_v2(value)
        value = normalize_manifest(value)
        if value['evidence']['level'] != 'declared' and not value['evidence']['reports']:
            raise ContractError('E_MANIFEST_EVIDENCE', 'evidence above declared requires reports')
        if value['io']['map_schema'] == 'support_map/v1' and value['io']['dimensions'] != 3:
            raise ContractError('E_MANIFEST_DIM', 'support_map/v1 requires dimensions 3')
        if value['io']['map_schema'] == 'occupancy_grid/v1' and value['io']['dimensions'] != 2:
            raise ContractError('E_MANIFEST_DIM', 'occupancy_grid/v1 requires dimensions 2')
        edges = [(edge['parent'], edge['child']) for edge in value['ownership']['tf_edges']]
        if len(edges) != len(set(edges)):
            raise ContractError('E_OWNERSHIP_TF_DUP', 'duplicate tf edge %s' % (edges,))
        for parent, child in edges:
            if parent == child:
                raise ContractError('E_OWNERSHIP_TF_DUP', 'self edge %s' % parent)
        if 'controller' in value['roles']:
            if value['ownership']['stop'] == 'none':
                raise ContractError('E_MANIFEST_STOP', 'controller role requires a stop method')
            if not value['ownership']['command_topics']:
                raise ContractError('E_MANIFEST_CONTROL', 'controller role requires command_topics')
    elif kind == 'robot':
        items = [value['capability'][name] for name in ('max_slope_deg', 'max_step_height_m', 'min_clearance_m')]
        items.append(value['brake'])
        for item in items:
            if item['level'] == 'declared' and item['composition_use'] != 'prohibited':
                raise ContractError('E_ROBOT_UNSOURCED', 'declared %r must be prohibited' % item['source'])
            if item['level'] == 'tested' and not item.get('report'):
                raise ContractError('E_ROBOT_EVIDENCE', 'tested value requires a report path')
        footprint = value['footprint']
        if footprint['type'] == 'circle':
            if 'radius_m' not in footprint:
                raise ContractError('E_ROBOT_FOOTPRINT', 'circle footprint requires radius_m')
            if 'points' in footprint:
                raise ContractError('E_ROBOT_FOOTPRINT', 'circle footprint must not carry points')
        if footprint['type'] == 'polygon':
            if 'points' not in footprint:
                raise ContractError('E_ROBOT_FOOTPRINT', 'polygon footprint requires points')
            if 'radius_m' in footprint:
                raise ContractError('E_ROBOT_FOOTPRINT', 'polygon footprint must not carry radius_m')
    elif kind == 'map':
        obstacles = value.get('obstacles', [])
        ids = [item['obstacle_id'] for item in obstacles]
        if len(ids) != len(set(ids)):
            raise ContractError('E_MAP_OBSTACLE_DUP', 'duplicate obstacle_id')
        for item in obstacles:
            if any(lo >= hi for lo, hi in zip(item['min'], item['max'])):
                raise ContractError('E_MAP_OBSTACLE_BOUNDS',
                                    '%s requires min < max on all axes' % item['obstacle_id'])
        if sorted(value['cell_states']) != ['free', 'occupied', 'unknown']:
            raise ContractError('E_MAP_CELL_STATES', 'cell_states must be exactly free/occupied/unknown')
        if value['kind'] == 'occupancy_grid':
            if value['multiple_surfaces_per_xy']:
                raise ContractError('E_MAP_2D_SURFACES', 'occupancy_grid must not claim multiple surfaces per XY')
            if 'surfaces' in value:
                raise ContractError('E_MAP_2D_SURFACES', 'occupancy_grid must not carry surfaces')
            if len(value.get('layers', [])) != 1:
                raise ContractError('E_MAP_LAYERS', 'occupancy_grid requires exactly one occupancy layer')
        else:
            if 'layers' in value:
                raise ContractError('E_MAP_LAYERS', 'support_map stores surfaces, not occupancy layers')
            surfaces = value['surfaces']
            if value['multiple_surfaces_per_xy'] and len(surfaces) < 2:
                raise ContractError('E_MAP_ML_COUNT', 'multi-surface map requires at least two surfaces')
            if not value['multiple_surfaces_per_xy'] and len(surfaces) != 1:
                raise ContractError('E_MAP_ML_COLLAPSE', 'single-layer map must declare exactly one surface')
            ids = [surface['surface_id'] for surface in surfaces]
            if len(ids) != len(set(ids)):
                raise ContractError('E_MAP_SURFACE_DUP', 'duplicate surface_id %s' % (ids,))
            by_id = {surface['surface_id']: surface for surface in surfaces}
            for surface in surfaces:
                box = surface['footprint']
                if box['min_x'] >= box['max_x'] or box['min_y'] >= box['max_y']:
                    raise ContractError('E_MAP_FOOTPRINT_DEGENERATE', 'surface %s footprint has no area' % surface['surface_id'])
                if surface['traversability'] != 'supported' and surface['edges']:
                    raise ContractError('E_MAP_EDGE_TRAVERSABILITY', 'unsupported %s cannot own edges' % surface['surface_id'])
            for surface in surfaces:
                for edge in surface['edges']:
                    if edge['to'] == surface['surface_id'] or edge['to'] not in by_id:
                        raise ContractError('E_MAP_EDGE_TARGET', '%s -> %s' % (surface['surface_id'], edge['to']))
                    target = by_id[edge['to']]
                    if target['traversability'] != 'supported':
                        raise ContractError('E_MAP_EDGE_TRAVERSABILITY', 'edge into unsupported %s' % target['surface_id'])
                    drop = abs(target['z_reference'] - surface['z_reference'])
                    if edge['kind'] == 'flat':
                        if 'max_slope_deg' in edge or 'max_step_height_m' in edge:
                            raise ContractError('E_MAP_EDGE_PARAM', 'flat edge %s->%s must not carry slope/step parameters' % (surface['surface_id'], edge['to']))
                        if drop > EDGE_TOL:
                            raise ContractError('E_MAP_EDGE_Z', 'flat edge %s->%s drops %.2f m' % (surface['surface_id'], edge['to'], drop))
                    if edge['kind'] == 'slope':
                        if 'max_slope_deg' not in edge:
                            raise ContractError('E_MAP_EDGE_PARAM', 'slope edge %s->%s lacks max_slope_deg' % (surface['surface_id'], edge['to']))
                        if 'max_step_height_m' in edge:
                            raise ContractError('E_MAP_EDGE_PARAM', 'slope edge %s->%s must not carry max_step_height_m' % (surface['surface_id'], edge['to']))
                    if edge['kind'] == 'step':
                        if 'max_step_height_m' not in edge:
                            raise ContractError('E_MAP_EDGE_PARAM', 'step edge %s->%s lacks max_step_height_m' % (surface['surface_id'], edge['to']))
                        if 'max_slope_deg' in edge:
                            raise ContractError('E_MAP_EDGE_PARAM', 'step edge %s->%s must not carry max_slope_deg' % (surface['surface_id'], edge['to']))
                        if drop > edge['max_step_height_m']:
                            raise ContractError('E_MAP_EDGE_Z', 'step edge %s->%s drops %.2f m' % (surface['surface_id'], edge['to'], drop))
            reached = _reachable(surfaces)
            if len(reached) != len(ids):
                raise ContractError('E_MAP_DISCONNECTED', 'unreachable %s' % sorted(set(ids) - reached))
            overlap = not value['multiple_surfaces_per_xy']
            for index, left in enumerate(surfaces):
                for right in surfaces[index + 1:]:
                    if (left['footprint']['min_x'] < right['footprint']['max_x']
                            and right['footprint']['min_x'] < left['footprint']['max_x']
                            and left['footprint']['min_y'] < right['footprint']['max_y']
                            and right['footprint']['min_y'] < left['footprint']['max_y']):
                        overlap = True
            if not overlap:
                raise ContractError('E_MAP_ML_NO_OVERLAP', 'no two surfaces overlap in XY')
            for index, left in enumerate(surfaces):
                for right in surfaces[index + 1:]:
                    if abs(left['z_reference'] - right['z_reference']) <= EDGE_TOL and (
                            left['footprint']['min_x'] < right['footprint']['max_x']
                            and right['footprint']['min_x'] < left['footprint']['max_x']
                            and left['footprint']['min_y'] < right['footprint']['max_y']
                            and right['footprint']['min_y'] < left['footprint']['max_y']):
                        raise ContractError('E_MAP_ML_SAME_Z', 'overlapping surfaces at the same z')
    elif kind == 'task':
        goal = value['goal']
        for label, point in (('start', value['start']), ('goal', goal)):
            if point['xyz'][0] is None or point['xyz'][1] is None:
                raise ContractError('E_TASK_XY_NULL', '%s x/y must be numbers' % label)
        if value['start']['xyz'][2] is None:
            raise ContractError('E_TASK_MISSING_Z', 'start pose requires z')
        if value['goal_reference'] == 'support_surface':
            if goal['xyz'][2] is None:
                raise ContractError('E_TASK_MISSING_Z', 'support_surface goal requires z')
            if goal['surface_disambiguation'] == 'not_applicable':
                raise ContractError('E_TASK_DISAMBIGUATION', 'support_surface goal needs a disambiguation policy')
            if goal['surface_disambiguation'] == 'explicit_hint' and goal['surface_hint'] is None:
                raise ContractError('E_TASK_SURFACE_AMBIGUOUS', 'explicit_hint requires surface_hint')
            if goal['surface_disambiguation'] == 'declared_search' and goal['surface_hint'] is not None:
                raise ContractError('E_TASK_SURFACE_AMBIGUOUS', 'declared_search forbids a fixed hint')
        else:
            if goal['surface_disambiguation'] != 'not_applicable':
                raise ContractError('E_TASK_DISAMBIGUATION', 'plane_xy goal must use not_applicable')
            if goal['surface_hint'] is not None:
                raise ContractError('E_TASK_SURFACE_AMBIGUOUS', 'plane_xy goal must not carry surface_hint')
            if goal['xyz'][2] is not None:
                raise ContractError('E_TASK_PLANE_Z', 'plane_xy goal must leave z null')
        timeout = value['timeout']
        if timeout['budget_s'] >= timeout['harness_deadline_s']:
            raise ContractError('E_TASK_BUDGET', 'budget %s >= deadline %s' % (timeout['budget_s'], timeout['harness_deadline_s']))


def load(kind):
    return json.loads((HERE / 'schema' / SCHEMAS[kind]).read_text())


def validate(kind, value, *, allow_obstacles=False):
    """Obstacle-aware consumers explicitly opt into map/v2; v1 consumers fail closed."""
    if kind == 'map' and isinstance(value, dict):
        _pre_map(value)
    if kind == 'map' and isinstance(value, dict) and value.get('schema_version') == 2:
        if not allow_obstacles:
            raise ContractError('E_VERSION', 'map/v2 requires an obstacle-aware consumer')
        root = json.loads((HERE / 'schema/map.v2.json').read_text())
    elif kind == 'manifest' and isinstance(value, dict) and value.get('schema_version') == 2:
        root = json.loads((HERE / 'schema/manifest.v2.json').read_text())
    else:
        root = load(kind)
    _walk(root, value, root, '')
    semantic(kind, value)
    return True


def check_pair(task, mapping):
    if not isinstance(mapping, dict):
        mapping = {}
    if mapping.get('kind') != 'support_map' and task['goal_reference'] == 'support_surface':
        raise ContractError('E_PAIR_MAP_KIND', 'support_surface goal needs a support_map')
    if task['frame'] != mapping.get('frame'):
        raise ContractError('E_PAIR_FRAME', 'task frame %s vs map frame %s' % (task['frame'], mapping.get('frame')))
    if task['map']['map_id'] != mapping.get('map_id'):
        raise ContractError('E_PAIR_MAP_ID', 'task and map disagree')
    if task['map']['map_version'] != mapping.get('map_version'):
        raise ContractError('E_PAIR_MAP_VERSION', 'task targets another map version')
    if task['goal_reference'] == 'support_surface' and task['goal']['surface_hint'] is not None:
        ids = [surface['surface_id'] for surface in mapping.get('surfaces', [])]
        if task['goal']['surface_hint'] not in ids:
            raise ContractError('E_PAIR_SURFACE', 'unknown surface_hint %s' % task['goal']['surface_hint'])
    return True


def main(argv):
    if len(argv) != 3 or argv[1] not in SCHEMAS:
        print('usage: validate.py {manifest|robot|map|task} FILE', file=sys.stderr)
        return 9
    kind, path = argv[1], Path(argv[2])
    if not path.is_file():
        print(json.dumps({'ok': False, 'reason': 'E_INPUT_MISSING', 'detail': str(path)}))
        return 9
    try:
        validate(kind, json.loads(path.read_text()), allow_obstacles=True)
    except ContractError as error:
        print(json.dumps({'ok': False, 'reason': error.code, 'detail': error.detail}, ensure_ascii=False))
        return 5
    except json.JSONDecodeError as error:
        print(json.dumps({'ok': False, 'reason': 'E_INPUT_JSON', 'detail': str(error)}))
        return 5
    print(json.dumps({'ok': True, 'kind': kind, 'file': path.name}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
