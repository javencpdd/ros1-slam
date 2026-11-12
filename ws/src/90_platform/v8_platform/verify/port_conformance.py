#!/usr/bin/env python3
"""Port conformance: declared named-port topics vs an observed interface table.

graph.check only proves that two manifests agree with each other.  Two
declarations can agree and both be wrong.  This checker compares every declared
topic/type against an external observation, so declarations cannot cross-certify.

Truth table format (v8_port_truth/v1)::

    schema: v8_port_truth/v1
    topics:
      /global_path: {type: nav_msgs/Path, publishers: [global]}
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'framework'))

TRUTH_SCHEMA = 'v8_port_truth/v1'


class PortError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


def load_truth(path):
    value = json.loads(Path(path).read_text(encoding='utf-8'))
    if value.get('schema') != TRUTH_SCHEMA:
        raise PortError('E_PORT_TRUTH_VERSION', repr(value.get('schema')))
    topics = value.get('topics')
    if not isinstance(topics, dict):
        raise PortError('E_PORT_TRUTH_SHAPE', 'topics must be a mapping')
    for topic, entry in topics.items():
        if not isinstance(topic, str) or not topic.startswith('/'):
            raise PortError('E_PORT_TRUTH_SHAPE', repr(topic))
        if not isinstance(entry, dict) or not isinstance(entry.get('type'), str):
            raise PortError('E_PORT_TRUTH_SHAPE', topic)
    return value


def topic_for(instance, port):
    """Effective topic: command_topic wins, else the <name>_topic param.

    A component may reuse a port name on both sides (mux cmd in/out); the
    output's command_topic is authoritative and must not inherit the input's
    param.
    """
    if port.get('command_topic'):
        return port['command_topic']
    params = instance.get('params') or {}
    return params.get(port['name'] + '_topic', port.get('topic'))


def check(report, catalog, truth):
    """Compare every declared port topic/type against the observed table."""
    violations, checked = [], 0
    for iid, instance in report['instances'].items():
        component = catalog['components'][instance['component']]
        for direction in ('inputs', 'outputs'):
            for port in component[direction]:
                topic = topic_for(instance, port)
                if not topic:
                    violations.append({'code': 'E_PORT_NO_TOPIC',
                                       'at': '%s.%s' % (iid, port['name'])})
                    continue
                checked += 1
                observed = truth['topics'].get(topic)
                if observed is None:
                    violations.append({'code': 'E_PORT_UNOBSERVED',
                                       'at': '%s.%s' % (iid, port['name']),
                                       'topic': topic})
                elif observed['type'] != port['type']:
                    violations.append({'code': 'E_PORT_TYPE',
                                       'at': '%s.%s' % (iid, port['name']),
                                       'topic': topic, 'declared': port['type'],
                                       'observed': observed['type']})
    return {'ok': not violations, 'checked': checked, 'violations': violations}
