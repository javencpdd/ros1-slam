"""Framework config: component catalog + pipeline graph YAML loading (ROS1, v0).

The YAML is the single assembly entry: instances, named ports, explicit edges,
parameters and external inputs. This module only parses structure; semantic
checks live in graph.py and rendering in render.py.
"""
import hashlib
import json
import re
import sys
from pathlib import Path, PurePosixPath

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'contracts'))
import validate as contract  # noqa: E402

CATALOG_SCHEMA = 'component_catalog/v1'
GRAPH_SCHEMA = 'pipeline_graph/v1'
INSTANCE_ID = re.compile(r'^[a-z0-9_]+(\.[a-z0-9_]+)*$')


class FrameworkError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise FrameworkError('E_CFG_DUP_KEY', 'duplicate key %r at %s' % (key, key_node.start_mark))
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping)


def load_yaml(path):
    path = Path(path)
    if not path.is_file():
        raise FrameworkError('E_CFG_MISSING', str(path))
    try:
        value = yaml.load(path.read_text(encoding='utf-8'), Loader=_UniqueKeyLoader)
    except yaml.YAMLError as error:
        raise FrameworkError('E_CFG_YAML', '%s: %s' % (path.name, error))
    if not isinstance(value, dict):
        raise FrameworkError('E_CFG_TYPE', '%s: top level must be a mapping' % path.name)
    return value, hashlib.sha256(path.read_bytes()).hexdigest()


def _require(mapping, key, kind, where):
    if key not in mapping:
        raise FrameworkError('E_CFG_FIELD', '%s missing %s' % (where, key))
    value = mapping[key]
    if not isinstance(value, kind):
        raise FrameworkError('E_CFG_TYPE', '%s.%s must be %s' % (where, key, kind.__name__))
    return value


def _port(raw, where, direction):
    if not isinstance(raw, dict):
        raise FrameworkError('E_CFG_TYPE', '%s must be a mapping' % where)
    name = _require(raw, 'name', str, where)
    msg_type = _require(raw, 'type', str, where)
    if not name or not msg_type:
        raise FrameworkError('E_CFG_FIELD', '%s name/type must be non-empty' % where)
    schema = raw.get('schema')
    if schema is not None and (not isinstance(schema, str) or not schema):
        raise FrameworkError('E_CFG_TYPE', '%s.schema must be non-empty string' % where)
    topic = raw.get('topic')
    if topic is not None and (not isinstance(topic, str) or not topic):
        raise FrameworkError('E_CFG_TYPE', '%s.topic must be non-empty string' % where)
    command_topic = raw.get('command_topic')
    if command_topic is not None and (not isinstance(command_topic, str) or not command_topic):
        raise FrameworkError('E_CFG_TYPE', '%s.command_topic must be non-empty string' % where)
    if command_topic is not None and direction != 'output':
        raise FrameworkError('E_CFG_FIELD', '%s.command_topic only valid on outputs' % where)
    port = {'name': name, 'type': msg_type, 'direction': direction}
    for key, value in (('schema', schema), ('topic', topic), ('command_topic', command_topic)):
        if value is not None:
            port[key] = value
    if direction == 'input':
        required = raw.get('required', True)
        if not isinstance(required, bool):
            raise FrameworkError('E_CFG_TYPE', '%s.required must be bool' % where)
        port['required'] = required
    return port


def _launch_remap(raw, cid):
    remap = raw.get('remap', [])
    if not isinstance(remap, list):
        raise FrameworkError('E_CFG_LAUNCH', '%s remap must be a list' % cid)
    for item in remap:
        if (not isinstance(item, dict) or not isinstance(item.get('from'), str)
                or not isinstance(item.get('to'), str)
                or not item['from'] or not item['to']):
            raise FrameworkError('E_CFG_LAUNCH', '%s remap needs from/to' % cid)
    return [{'from': item['from'], 'to': item['to']} for item in remap]


def _launch_spec(raw, cid):
    """Normalize a launch entry: node(type), include(launch_file) or plugin.

    plugin means one nodelet loaded through `nodelet standalone`: package is the
    host (`nodelet`), type is the pluginlib name `pkg/Class`.
    """
    if not isinstance(raw, dict):
        raise FrameworkError('E_CFG_LAUNCH', '%s launch must be a mapping' % cid)
    kind = raw.get('kind', 'node')
    if kind not in ('node', 'include', 'plugin'):
        raise FrameworkError('E_CFG_LAUNCH_KIND', '%s kind=%r' % (cid, kind))
    package = raw.get('package')
    if not isinstance(package, str) or not package:
        raise FrameworkError('E_CFG_LAUNCH', '%s package missing' % cid)
    spec = {'kind': kind, 'package': package,
            'test_double': bool(raw.get('test_double', False))}
    marker = raw.get('ready_marker')
    if marker is not None:
        if not isinstance(marker, str) or not marker:
            raise FrameworkError('E_CFG_LAUNCH', '%s ready_marker must be a string' % cid)
        spec['ready_marker'] = marker
    if kind == 'node':
        node_type = raw.get('type')
        if not isinstance(node_type, str) or not node_type:
            raise FrameworkError('E_CFG_LAUNCH', '%s node type missing' % cid)
        if 'launch_file' in raw:
            raise FrameworkError('E_CFG_LAUNCH', '%s node must not set launch_file' % cid)
        spec['type'] = node_type
        param_files = raw.get('param_files', [])
        if not isinstance(param_files, list):
            raise FrameworkError('E_CFG_LAUNCH', '%s param_files must be a list' % cid)
        clean = []
        for item in param_files:
            if not isinstance(item, str) or not item:
                raise FrameworkError('E_CFG_LAUNCH', '%s param file must be a path' % cid)
            path = PurePosixPath(item)
            if path.is_absolute() or '..' in path.parts:
                raise FrameworkError('E_CFG_LAUNCH_PATH',
                                     '%s param file escapes package: %s' % (cid, item))
            clean.append(item)
        spec['param_files'] = clean
        spec['remap'] = _launch_remap(raw, cid)
        if 'args' in raw:
            raise FrameworkError('E_CFG_LAUNCH', '%s node must not set args' % cid)
    elif kind == 'plugin':
        if package != 'nodelet':
            raise FrameworkError('E_CFG_LAUNCH',
                                 '%s plugin host package must be nodelet' % cid)
        plugin_type = raw.get('type')
        parts = plugin_type.split('/') if isinstance(plugin_type, str) else []
        if len(parts) != 2 or not all(parts):
            raise FrameworkError('E_CFG_LAUNCH',
                                 '%s plugin type must be pkg/Class' % cid)
        if 'launch_file' in raw or 'param_files' in raw or 'args' in raw:
            raise FrameworkError('E_CFG_LAUNCH',
                                 '%s plugin only takes type/remap' % cid)
        spec['plugin'] = plugin_type
        spec['remap'] = _launch_remap(raw, cid)
    else:
        launch_file = raw.get('launch_file')
        if not isinstance(launch_file, str) or not launch_file.endswith('.launch'):
            raise FrameworkError('E_CFG_LAUNCH',
                                 '%s include needs a .launch file' % cid)
        if 'type' in raw or 'param_files' in raw or 'remap' in raw:
            raise FrameworkError('E_CFG_LAUNCH',
                                 '%s include only takes launch_file/args' % cid)
        spec['launch_file'] = launch_file
        args = raw.get('args', {})
        if not isinstance(args, dict):
            raise FrameworkError('E_CFG_LAUNCH', '%s args must be a mapping' % cid)
        spec['args'] = args
    return spec


def load_components(path):
    """Load v2 manifests (named ports + launch) as the single component source.

    v1 manifests remain valid for the legacy registry/composer; they cannot enter
    a graph because they have no named ports.
    """
    path = Path(path)
    files = sorted(path.glob('manifest_*.json')) if path.is_dir() else [path]
    if not files or (path.is_dir() and not any(f.is_file() for f in files)):
        raise FrameworkError('E_CFG_MISSING', str(path))
    components, digest = {}, hashlib.sha256()
    for file in files:
        if not file.is_file():
            raise FrameworkError('E_CFG_MISSING', str(file))
        raw_bytes = file.read_bytes()
        try:
            value = json.loads(raw_bytes)
        except json.JSONDecodeError as error:
            raise FrameworkError('E_CFG_YAML', '%s: %s' % (file.name, error))
        try:
            contract.validate('manifest', value)
        except contract.ContractError as error:
            raise FrameworkError('E_CFG_MANIFEST', '%s: %s: %s'
                                 % (file.name, error.code, error.detail))
        if value.get('schema_version') != 2 or 'ports' not in value:
            raise FrameworkError('E_CFG_MANIFEST_V1',
                                 '%s has no named ports; use manifest v2' % file.name)
        cid = value['id']
        if cid in components:
            raise FrameworkError('E_CFG_DUP_COMPONENT', cid)
        launch = _launch_spec(value.get('launch'), cid)
        inputs = [dict(port, direction='input') for port in value['ports']['inputs']]
        outputs = [dict(port, direction='output') for port in value['ports']['outputs']]
        components[cid] = {'id': cid, 'launch': launch, 'roles': list(value['roles']),
                           'inputs': inputs, 'outputs': outputs,
                           'dimensions': value['dimensions'],
                           'kinematics': value['capability']['kinematics'],
                           'param_schema': value.get('param_schema'),
                           'stop': value['ownership']['stop'],
                           'tf_edges': list(value['ownership'].get('tf_edges', [])),
                           'resources': list(value.get('requires', {}).get('resources', []))}
        digest.update(file.name.encode() + b'\0' + raw_bytes)
    return {'components': components, 'sha256': digest.hexdigest(), 'path': str(path)}


def load_graph(path):
    value, sha = load_yaml(path)
    if value.get('schema') != GRAPH_SCHEMA:
        raise FrameworkError('E_CFG_SCHEMA', 'graph schema must be %s' % GRAPH_SCHEMA)
    name = _require(value, 'name', str, 'graph')
    if not name:
        raise FrameworkError('E_CFG_FIELD', 'graph.name must be non-empty')
    instances_raw = _require(value, 'instances', list, 'graph')
    if not instances_raw:
        raise FrameworkError('E_CFG_EMPTY', 'graph instances is empty')
    instances = {}
    for index, raw in enumerate(instances_raw):
        where = 'instances[%d]' % index
        if not isinstance(raw, dict):
            raise FrameworkError('E_CFG_TYPE', '%s must be a mapping' % where)
        iid = _require(raw, 'id', str, where)
        if not INSTANCE_ID.match(iid):
            raise FrameworkError('E_CFG_INSTANCE_ID', '%s: %s' % (where, iid))
        if iid in instances:
            raise FrameworkError('E_CFG_DUP_INSTANCE', iid)
        component = _require(raw, 'component', str, where)
        params = raw.get('params', {})
        if not isinstance(params, dict):
            raise FrameworkError('E_CFG_TYPE', '%s.params must be a mapping' % where)
        namespace = raw.get('namespace')
        if namespace is not None and (not isinstance(namespace, str) or not namespace.startswith('/')):
            raise FrameworkError('E_CFG_TYPE', '%s.namespace must be absolute' % where)
        config_path = raw.get('config')
        if config_path is not None and (not isinstance(config_path, str) or not config_path):
            raise FrameworkError('E_CFG_CONFIG', '%s.config must be a non-empty path' % where)
        instances[iid] = {'id': iid, 'component': component, 'params': params,
                          'namespace': namespace}
        if config_path is not None:
            instances[iid]['config'] = config_path
    edges = []
    for index, raw in enumerate(value.get('edges', [])):
        where = 'edges[%d]' % index
        if not isinstance(raw, dict):
            raise FrameworkError('E_CFG_TYPE', '%s must be a mapping' % where)
        source = _require(raw, 'from', str, where)
        target = _require(raw, 'to', str, where)
        feedback = raw.get('feedback', False)
        if not isinstance(feedback, bool):
            raise FrameworkError('E_CFG_TYPE', '%s.feedback must be bool' % where)
        policy = raw.get('policy', {})
        if not isinstance(policy, dict):
            raise FrameworkError('E_CFG_TYPE', '%s.policy must be a mapping' % where)
        edges.append({'from': source, 'to': target, 'feedback': feedback,
                      'policy': policy, 'index': index})
    external = []
    for index, raw in enumerate(value.get('external', [])):
        where = 'external[%d]' % index
        if not isinstance(raw, dict):
            raise FrameworkError('E_CFG_TYPE', '%s must be a mapping' % where)
        target = _require(raw, 'to', str, where)
        msg_type = _require(raw, 'type', str, where)
        source = raw.get('source', 'external')
        if not isinstance(source, str) or not source:
            raise FrameworkError('E_CFG_TYPE', '%s.source must be non-empty string' % where)
        external.append({'to': target, 'type': msg_type, 'source': source, 'index': index})
    return {'name': name, 'instances': instances, 'edges': edges, 'external': external,
            'sha256': sha, 'path': str(path), 'raw': value}
