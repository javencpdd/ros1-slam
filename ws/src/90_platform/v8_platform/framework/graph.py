"""Pipeline graph checks: named ports, explicit edges, startup DAG, command ownership.

The data graph may contain feedback edges only when explicitly marked; the
startup DAG is built from non-feedback edges so a cache/map update loop does
not deadlock process startup.
"""
import hashlib
import json
import re

from config import FrameworkError


def _endpoint(text, where):
    if not isinstance(text, str) or '.' not in text:
        raise FrameworkError('E_GRAPH_ENDPOINT', '%s must be instance.port' % where)
    instance, port = text.rsplit('.', 1)
    if not instance or not port:
        raise FrameworkError('E_GRAPH_ENDPOINT', '%s must be instance.port' % where)
    return instance, port


def _port(instances, catalog, endpoint, direction, where):
    instance, name = _endpoint(endpoint, where)
    if instance not in instances:
        raise FrameworkError('E_GRAPH_INSTANCE', '%s references unknown instance %s' % (where, instance))
    rec = instances[instance]
    comp = catalog['components'][rec['component']]
    matches = [p for p in comp['inputs' if direction == 'input' else 'outputs']
               if p['name'] == name]
    if not matches:
        raise FrameworkError('E_GRAPH_PORT', '%s references %s.%s as %s, not declared'
                             % (where, instance, name, direction))
    return instance, matches[0]


def resolve(graph, catalog):
    """Resolve component types and endpoint ports; structural only."""
    instances, names = {}, {}
    for iid, rec in graph['instances'].items():
        node_name = re.sub(r'[^A-Za-z0-9_]', '_', iid)
        if node_name in names:
            raise FrameworkError('E_GRAPH_NAME_COLLISION',
                                 '%s and %s both map to ROS node name %s'
                                 % (names[node_name], iid, node_name))
        names[node_name] = iid
        comp = catalog['components'].get(rec['component'])
        if comp is None:
            raise FrameworkError('E_GRAPH_COMPONENT',
                                 'instance %s references unknown component %s' % (iid, rec['component']))
        instances[iid] = dict(rec)
    edges = []
    for edge in graph['edges']:
        src_i, src_p = _port(instances, catalog, edge['from'], 'output', 'edge.from')
        dst_i, dst_p = _port(instances, catalog, edge['to'], 'input', 'edge.to')
        if src_p['type'] != dst_p['type']:
            raise FrameworkError('E_GRAPH_TYPE',
                                 '%s -> %s: %s != %s' % (edge['from'], edge['to'],
                                                         src_p['type'], dst_p['type']))
        if src_p.get('schema') != dst_p.get('schema'):
            raise FrameworkError('E_GRAPH_SCHEMA',
                                 '%s -> %s: schema %r != %r' % (
                                     edge['from'], edge['to'],
                                     src_p.get('schema'), dst_p.get('schema')))
        if edge['feedback'] and not edge['policy']:
            raise FrameworkError('E_GRAPH_FEEDBACK_POLICY',
                                 '%s -> %s feedback edge needs a policy' % (edge['from'], edge['to']))
        edges.append({'from': edge['from'], 'to': edge['to'],
                      'from_instance': src_i, 'to_instance': dst_i,
                      'from_port': src_p, 'to_port': dst_p,
                      'feedback': edge['feedback'], 'policy': edge['policy']})
    external = []
    for item in graph['external']:
        inst, port = _port(instances, catalog, item['to'], 'input', 'external.to')
        if item['type'] != port['type']:
            raise FrameworkError('E_GRAPH_TYPE',
                                 'external %s: %s != %s' % (item['to'], item['type'], port['type']))
        external.append({'to': item['to'], 'instance': inst, 'port': port,
                         'type': item['type'], 'source': item['source']})
    return instances, edges, external


def _topological(instances, edges):
    deps = {iid: set() for iid in instances}
    for edge in edges:
        if not edge['feedback']:
            deps[edge['to_instance']].add(edge['from_instance'])
    order, remaining = [], dict(deps)
    while remaining:
        ready = sorted(iid for iid, dep in remaining.items() if not dep - set(order))
        if not ready:
            raise FrameworkError('E_GRAPH_STARTUP_CYCLE',
                                 'non-feedback startup cycle: ' + ','.join(sorted(remaining)))
        for iid in ready:
            order.append(iid)
            del remaining[iid]
    return order


def check(graph, catalog):
    instances, edges, external = resolve(graph, catalog)
    bindings = {iid: {} for iid in instances}
    for edge in edges:
        port = edge['to_port']
        if port['name'] in bindings[edge['to_instance']]:
            raise FrameworkError('E_GRAPH_INPUT_DUP',
                                 '%s.%s bound more than once' % (edge['to_instance'], port['name']))
        bindings[edge['to_instance']][port['name']] = {
            'source': 'edge', 'from': edge['from'], 'type': edge['from_port']['type'],
            'schema': edge['from_port'].get('schema')}
    for item in external:
        port = item['port']
        if port['name'] in bindings[item['instance']]:
            raise FrameworkError('E_GRAPH_INPUT_DUP',
                                 '%s.%s bound more than once' % (item['instance'], port['name']))
        bindings[item['instance']][port['name']] = {
            'source': 'external:' + item['source'], 'type': item['type'],
            'schema': port.get('schema')}

    for iid, rec in instances.items():
        comp = catalog['components'][rec['component']]
        for port in comp['inputs']:
            if port.get('required', True) and port['name'] not in bindings[iid]:
                raise FrameworkError('E_GRAPH_INPUT',
                                     '%s.%s is required but unbound' % (iid, port['name']))

    writers = {}
    for iid, rec in instances.items():
        comp = catalog['components'][rec['component']]
        for port in comp['outputs']:
            topic = port.get('command_topic')
            if topic is None:
                continue
            if topic in writers:
                raise FrameworkError('E_GRAPH_COMMAND',
                                     '%s written by %s and %s' % (topic, writers[topic], iid))
            writers[topic] = iid

    order = _topological(instances, edges)
    feedback = [{'from': e['from'], 'to': e['to'], 'policy': e['policy']}
                for e in edges if e['feedback']]
    normalized = {
        'catalog': catalog['sha256'],
        'graph': {
            'name': graph['name'],
            'instances': [{'id': iid, 'component': rec['component'],
                           'params': rec['params'], 'namespace': rec['namespace']}
                          for iid, rec in sorted(instances.items())],
            'edges': [{'from': e['from'], 'to': e['to'], 'feedback': e['feedback'],
                       'policy': e['policy']} for e in edges],
            'external': [{'to': e['to'], 'type': e['type'], 'source': e['source']}
                         for e in external],
        },
    }
    digest = hashlib.sha256(json.dumps(normalized, sort_keys=True, ensure_ascii=False,
                                       separators=(',', ':')).encode()).hexdigest()
    return {'ok': True, 'name': graph['name'], 'digest': digest,
            'instances': instances, 'edges': edges, 'external': external,
            'bindings': bindings, 'startup_order': order,
            'feedback': feedback, 'command_writers': dict(sorted(writers.items()))}
