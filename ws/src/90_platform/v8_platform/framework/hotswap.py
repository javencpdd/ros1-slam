"""V8-X3 v0.1 hot-swap contract: prepare a compatible replacement, then apply
with a stop-before-start handoff. This is 'stop then change composition', not a
live in-process state transfer; the opaque state record is carried for the next
component to consume explicitly.
"""
import copy
import hashlib
import json

import safety
from config import FrameworkError


def _digest(plan):
    payload = {key: value for key, value in plan.items() if key != 'digest'}
    text = json.dumps(payload, sort_keys=True, ensure_ascii=False, separators=(',', ':'))
    return hashlib.sha256(text.encode()).hexdigest()


def _ports(component):
    rows = []
    for direction, key in (('input', 'inputs'), ('output', 'outputs')):
        for port in component.get(key, []):
            rows.append((direction, port['name'], port.get('type'), port.get('schema'),
                         port.get('command_topic')))
    return tuple(sorted(rows, key=lambda row: (row[0], row[1])))


def validate_record(record):
    required = ('schema_version', 'task_id', 'instance_id', 'old_component',
                'new_component', 'state', 'epoch', 'preconditions', 'postconditions')
    if not isinstance(record, dict) or any(key not in record for key in required):
        raise FrameworkError('E_SWAP_RECORD', 'missing hot_swap fields')
    if record['schema_version'] != 1 or not isinstance(record['epoch'], int) or record['epoch'] < 0:
        raise FrameworkError('E_SWAP_RECORD', 'bad schema_version/epoch')
    if not isinstance(record['state'], dict):
        raise FrameworkError('E_SWAP_RECORD', 'state must be an object')
    try:
        json.dumps(record['state'])
    except (TypeError, ValueError):
        raise FrameworkError('E_SWAP_RECORD', 'state is not JSON serializable')
    for group in ('preconditions', 'postconditions'):
        if not isinstance(record[group], dict):
            raise FrameworkError('E_SWAP_RECORD', '%s must be an object' % group)
    return record


def prepare(plan, catalog, instance_id, new_component_id, state=None,
            task_id=None, epoch=0):
    """Check role/port/ownership compatibility; return (record, new_plan)."""
    instance = next((item for item in plan['instances'] if item['id'] == instance_id), None)
    if instance is None:
        raise FrameworkError('E_SWAP_INSTANCE', instance_id)
    old_id = instance['component']
    old = catalog['components'].get(old_id)
    new = catalog['components'].get(new_component_id)
    if old is None or new is None:
        raise FrameworkError('E_SWAP_COMPONENT', new_component_id)
    if sorted(old['roles']) != sorted(new['roles']):
        raise FrameworkError('E_SWAP_ROLES', '%s vs %s' % (old['roles'], new['roles']))
    if _ports(old) != _ports(new):
        raise FrameworkError('E_SWAP_PORTS', '%s -> %s' % (old_id, new_component_id))
    new_plan = copy.deepcopy(plan)
    for item in new_plan['instances']:
        if item['id'] == instance_id:
            item['component'] = new_component_id
            item['launch'] = new['launch']
            break
    new_plan['digest'] = _digest(new_plan)
    checked = safety.check_plan(new_plan, catalog)
    record = validate_record({
        'schema_version': 1,
        'task_id': task_id or plan['name'],
        'instance_id': instance_id,
        'old_component': old_id,
        'new_component': new_component_id,
        'state': dict(state or {}),
        'epoch': epoch,
        'preconditions': {'assembly_stopped': False,
                          'no_command_overlap': True,
                          'tf_edges_preserved': True},
        'postconditions': {'new_ready': False, 'single_command_writer': bool(checked['ok'])}})
    return record, new_plan


def apply(assembly, catalog, record, new_plan, ready_timeout=10.0):
    """Stop the old assembly, then start the new plan; no dual writers by order."""
    from runtime import Assembly

    validate_record(record)
    old_runners = list(assembly.runners.values())
    assembly.stop()
    if any(runner.is_alive() for runner in old_runners):
        raise FrameworkError('E_SWAP_STOP', 'old assembly still alive')
    record['preconditions']['assembly_stopped'] = True
    new_assembly = Assembly(new_plan, catalog, command_factory=assembly.command_factory,
                            ready_timeout=ready_timeout)
    try:
        new_assembly.start()
    except Exception as error:
        record['postconditions'] = {'new_ready': False, 'single_command_writer': False}
        raise FrameworkError('E_SWAP_START', '%s: %s' % (type(error).__name__, error))
    record['postconditions'] = {'new_ready': new_assembly.ready(),
                                'single_command_writer': True}
    record['epoch'] += 1
    return record, new_assembly
