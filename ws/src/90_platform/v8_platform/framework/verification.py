"""Independent simulation verification config (F6).

The verification backend is selected alongside the algorithm graph, never inside
it. Truth channels are checked against algorithm topics; the runner is injected
so tests use fake backends without ROS/MuJoCo/Gazebo.
"""
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import config  # noqa: E402
import launch as launch_mod  # noqa: E402

SCHEMA = 'verification/v1'


def load_config(path):
    value, sha = config.load_yaml(path)
    if value.get('schema') != SCHEMA:
        raise config.FrameworkError('E_VERIFY_SCHEMA', 'expected %s' % SCHEMA)
    for key in ('name', 'backend', 'truth_channel'):
        if not isinstance(value.get(key), str) or not value[key]:
            raise config.FrameworkError('E_VERIFY_FIELD', key)
    if not isinstance(value.get('scenario'), dict):
        raise config.FrameworkError('E_VERIFY_FIELD', 'scenario')
    if not isinstance(value.get('report'), dict):
        raise config.FrameworkError('E_VERIFY_FIELD', 'report')
    return {'name': value['name'], 'backend': value['backend'],
            'scenario': value['scenario'], 'truth_channel': value['truth_channel'],
            'report': value['report'], 'sha256': sha}


def load_backends(directory):
    return config.load_components(directory)


def check(spec, plan, catalog, backends, packages):
    backend = backends['components'].get(spec['backend'])
    if backend is None:
        raise config.FrameworkError('E_VERIFY_BACKEND', spec['backend'])
    if 'simulator' not in backend['roles']:
        raise config.FrameworkError('E_VERIFY_ROLE', spec['backend'])
    algorithm_components = {instance['component'] for instance in plan['instances']}
    if spec['backend'] in algorithm_components:
        raise config.FrameworkError('E_VERIFY_BACKEND_IN_PLAN', spec['backend'])
    launch_mod.check_components({'components': {spec['backend']: backend}}, packages)
    algorithm_topics = set()
    for component in catalog['components'].values():
        for port in component['inputs'] + component['outputs']:
            if port.get('topic'):
                algorithm_topics.add(port['topic'])
    if spec['truth_channel'] in algorithm_topics:
        raise config.FrameworkError('E_VERIFY_TRUTH_LEAK', spec['truth_channel'])
    digest = hashlib.sha256(
        json.dumps({'verification': spec['sha256'], 'backend': spec['backend'],
                    'algorithm': plan['digest']}, sort_keys=True).encode()).hexdigest()
    return {'ok': True, 'name': spec['name'], 'backend': spec['backend'],
            'scenario': spec['scenario'], 'truth_channel': spec['truth_channel'],
            'algorithm_digest': plan['digest'], 'verification_digest': digest}


def run(spec, checked, runner):
    """Run an injected backend; a backend failure never mutates the algorithm plan."""
    try:
        result = runner(spec)
        status = 'ok'
        error = None
    except Exception as exc:  # backend failure is data, not a framework crash
        result = None
        status = 'error'
        error = repr(exc)
    return {'status': status, 'name': checked['name'], 'backend': checked['backend'],
            'scenario': checked['scenario'], 'truth_channel': checked['truth_channel'],
            'algorithm_digest': checked['algorithm_digest'],
            'verification_digest': checked['verification_digest'],
            'result': result, 'error': error}
