"""Framework config: component catalog + pipeline graph YAML loading (ROS1, v0).

The YAML is the single assembly entry: instances, named ports, explicit edges,
parameters and external inputs. This module only parses structure; semantic
checks live in graph.py and rendering in render.py.
"""
import hashlib
import re
from pathlib import Path

import yaml

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


def load_catalog(path):
    value, sha = load_yaml(path)
    if value.get('schema') != CATALOG_SCHEMA:
        raise FrameworkError('E_CFG_SCHEMA', 'catalog schema must be %s' % CATALOG_SCHEMA)
    raw_components = _require(value, 'components', list, 'catalog')
    if not raw_components:
        raise FrameworkError('E_CFG_EMPTY', 'catalog components is empty')
    components = {}
    for index, raw in enumerate(raw_components):
        where = 'components[%d]' % index
        if not isinstance(raw, dict):
            raise FrameworkError('E_CFG_TYPE', '%s must be a mapping' % where)
        cid = _require(raw, 'id', str, where)
        if cid in components:
            raise FrameworkError('E_CFG_DUP_COMPONENT', cid)
        launch = _require(raw, 'launch', dict, where)
        package = _require(launch, 'package', str, where + '.launch')
        node_type = _require(launch, 'type', str, where + '.launch')
        roles = raw.get('roles', [])
        if not isinstance(roles, list) or not all(isinstance(r, str) and r for r in roles):
            raise FrameworkError('E_CFG_TYPE', '%s.roles must be a string list' % where)
        inputs, outputs = [], []
        seen = set()
        for direction, items in (('input', raw.get('inputs', [])),
                                 ('output', raw.get('outputs', []))):
            if not isinstance(items, list):
                raise FrameworkError('E_CFG_TYPE', '%s.%ss must be a list' % (where, direction))
            for pindex, item in enumerate(items):
                port = _port(item, '%s.%ss[%d]' % (where, direction, pindex), direction)
                key = (direction, port['name'])
                if key in seen:
                    raise FrameworkError('E_CFG_DUP_PORT', '%s %s.%s' % (where, direction, port['name']))
                seen.add(key)
                (inputs if direction == 'input' else outputs).append(port)
        components[cid] = {'id': cid, 'launch': {'package': package, 'type': node_type},
                           'roles': roles, 'inputs': inputs, 'outputs': outputs}
    return {'components': components, 'sha256': sha, 'path': str(path)}


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
        instances[iid] = {'id': iid, 'component': component, 'params': params,
                          'namespace': namespace}
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
