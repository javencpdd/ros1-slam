"""V8-X3 v0.1 hot-swap contract: prepare a compatible replacement, then apply
with a stop-before-start handoff. This is 'stop then change composition', not a
live in-process state transfer; the opaque state record is carried for the next
component to consume explicitly.
"""
import copy
import hashlib
import json
import tempfile
import time
from pathlib import Path

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
            task_id=None, epoch=0, params=None):
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
            if params is not None:
                item['params'] = dict(params)
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


def _overlap_safe(component):
    return (not component.get('tf_edges')
            and not any(port.get('command_topic')
                        for port in component.get('outputs', [])))


def apply_overlap(assembly, catalog, record, new_plan, ready_timeout=10.0,
                  ready_check=None):
    """Start the replacement first, wait until ready, then stop the old one.

    Restricted to instances without command topics and TF edges: during the
    overlap both may publish ordinary data, which is acceptable only for
    read-only/latched providers. `ready_check()` is polled when given; otherwise
    the runner's ready marker is used.
    """
    import orchestrator as orch
    from process_runner import ProcessRunner

    validate_record(record)
    instance_id = record['instance_id']
    old_runner = assembly.runners.get(instance_id)
    old_spec = next((spec for spec in assembly.task.specs if spec.id == instance_id), None)
    if old_runner is None or old_spec is None:
        raise FrameworkError('E_SWAP_INSTANCE', instance_id)
    old_component = catalog['components'][record['old_component']]
    new_component = catalog['components'][record['new_component']]
    if not _overlap_safe(old_component) or not _overlap_safe(new_component):
        raise FrameworkError('E_SWAP_OVERLAP', 'command or TF ownership')
    new_instance = next(item for item in new_plan['instances']
                        if item['id'] == instance_id)
    new_instance['runtime_name'] = '%s_swap%d' % (instance_id, record['epoch'] + 1)
    new_spec = orch.ComponentSpec(instance_id, new_component['roles'],
                                  stop_method=new_component['stop'],
                                  resources=new_component.get('resources', []))
    new_argv, new_marker = assembly.command_factory(new_instance, new_component)
    state_file = None
    env = None
    if record['state']:
        handle = tempfile.NamedTemporaryFile('w', prefix='v8_swap_', suffix='.json',
                                             delete=False, encoding='utf-8')
        json.dump(record['state'], handle, ensure_ascii=False, sort_keys=True)
        handle.close()
        state_file = handle.name
        env = {'V8_HOT_SWAP_STATE': state_file,
               'V8_HOT_SWAP_INSTANCE': instance_id,
               'V8_HOT_SWAP_EPOCH': str(record['epoch'] + 1)}
    new_runner = ProcessRunner(new_spec, new_argv, ready_marker=new_marker, env=env)
    try:
        new_runner.start()
        if ready_check is not None:
            deadline = time.monotonic() + ready_timeout
            while True:
                try:
                    if ready_check():
                        break
                except Exception:
                    pass
                if time.monotonic() >= deadline:
                    raise TimeoutError('replacement not ready')
                time.sleep(.1)
        else:
            from orchestrator import wait_ready
            wait_ready([new_runner], timeout=ready_timeout)
    except Exception as error:
        new_runner.stop(new_component['stop'])
        new_runner.reap()
        if state_file:
            Path(state_file).unlink(missing_ok=True)
        record['postconditions'] = {'new_ready': False, 'single_command_writer': True,
                                    'rolled_back': True, 'overlap': False}
        raise FrameworkError('E_SWAP_START', '%s; old kept running' % error)
    record['preconditions']['new_ready_before_stop'] = True
    record['preconditions']['old_alive_before_stop'] = old_runner.is_alive()
    old_runner.stop(old_spec.stop_method)
    old_runner.reap()
    if old_runner.is_alive():
        new_runner.stop(new_component['stop'])
        new_runner.reap()
        if state_file:
            Path(state_file).unlink(missing_ok=True)
        raise FrameworkError('E_SWAP_STOP', 'old instance still alive')
    assembly.runners[instance_id] = new_runner
    for index, spec in enumerate(assembly.task.specs):
        if spec.id == instance_id:
            assembly.task.specs[index] = new_spec
            break
    assembly.plan = new_plan
    new_runner.state_file = state_file
    record['postconditions'] = {'new_ready': True, 'single_command_writer': True,
                                'rolled_back': False, 'overlap': True,
                                'old_stopped': True}
    record['epoch'] += 1
    return record, new_runner


def apply_instance(assembly, catalog, record, new_plan, ready_timeout=10.0,
                   pre_stop=None, post_start=None):
    """Replace one instance while the rest of the assembly keeps running.

    The old process is stopped and reaped before the new one starts, so there is
    no command/TF overlap. On start failure the old component is restarted; the
    task object stays RUNNING and carries the opaque state record.
    """
    import orchestrator as orch
    from process_runner import ProcessRunner

    validate_record(record)
    instance_id = record['instance_id']
    old_runner = assembly.runners.get(instance_id)
    if old_runner is None:
        raise FrameworkError('E_SWAP_INSTANCE', instance_id)
    old_spec = next((spec for spec in assembly.task.specs if spec.id == instance_id), None)
    if old_spec is None:
        raise FrameworkError('E_SWAP_INSTANCE', instance_id)
    old_instance = next(item for item in assembly.plan['instances']
                        if item['id'] == instance_id)
    old_component = catalog['components'][record['old_component']]
    old_argv, old_marker = assembly.command_factory(old_instance, old_component)
    if pre_stop is not None:
        pre_stop()
    old_runner.stop(old_spec.stop_method)
    old_runner.reap()
    if old_runner.is_alive():
        raise FrameworkError('E_SWAP_STOP', 'old instance still alive')
    record['preconditions']['target_stopped'] = True

    new_component = catalog['components'][record['new_component']]
    new_instance = next(item for item in new_plan['instances']
                        if item['id'] == instance_id)
    new_spec = orch.ComponentSpec(instance_id, new_component['roles'],
                                  stop_method=new_component['stop'],
                                  resources=new_component.get('resources', []))
    new_argv, new_marker = assembly.command_factory(new_instance, new_component)
    state_file = None
    env = None
    if record['state']:
        handle = tempfile.NamedTemporaryFile('w', prefix='v8_swap_', suffix='.json',
                                             delete=False, encoding='utf-8')
        json.dump(record['state'], handle, ensure_ascii=False, sort_keys=True)
        handle.close()
        state_file = handle.name
        env = {'V8_HOT_SWAP_STATE': state_file,
               'V8_HOT_SWAP_INSTANCE': instance_id,
               'V8_HOT_SWAP_EPOCH': str(record['epoch'] + 1)}
    new_runner = ProcessRunner(new_spec, new_argv, ready_marker=new_marker, env=env)
    try:
        new_runner.start()
        from orchestrator import wait_ready
        wait_ready([new_runner], timeout=ready_timeout)
    except Exception as error:
        new_runner.stop(new_component['stop'])
        new_runner.reap()
        rollback = ProcessRunner(old_spec, old_argv, ready_marker=old_marker)
        try:
            rollback.start()
            from orchestrator import wait_ready
            wait_ready([rollback], timeout=ready_timeout)
        except Exception as rollback_error:
            record['postconditions'] = {'new_ready': False,
                                        'single_command_writer': False,
                                        'rolled_back': False}
            raise FrameworkError('E_SWAP_ROLLBACK',
                                 '%s; rollback: %s' % (error, rollback_error))
        if state_file:
            Path(state_file).unlink(missing_ok=True)
        assembly.runners[instance_id] = rollback
        record['postconditions'] = {'new_ready': False,
                                    'single_command_writer': True,
                                    'rolled_back': True}
        raise FrameworkError('E_SWAP_START', '%s; rolled back' % error)
    assembly.runners[instance_id] = new_runner
    for index, spec in enumerate(assembly.task.specs):
        if spec.id == instance_id:
            assembly.task.specs[index] = new_spec
            break
    assembly.plan = new_plan
    new_runner.state_file = state_file
    record['postconditions'] = {'new_ready': True, 'single_command_writer': True,
                                'rolled_back': False}
    record['epoch'] += 1
    if post_start is not None:
        post_start()
    return record, new_runner
