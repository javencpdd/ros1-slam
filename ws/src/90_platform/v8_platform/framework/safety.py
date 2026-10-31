"""F7 safety wiring: one /cmd_vel owner, advice into the arbiter, validator path."""
from config import FrameworkError


def _component(catalog, instance):
    return catalog['components'][instance['component']]


def check_plan(plan, catalog):
    instances = {item['id']: item for item in plan['instances']}
    owners = []
    for iid, instance in instances.items():
        for port in _component(catalog, instance)['outputs']:
            if port.get('command_topic') == '/cmd_vel':
                owners.append(iid)
    if len(owners) > 1:
        raise FrameworkError('E_SAFETY_ARBITER',
                             'expected at most one /cmd_vel writer, got %s' % owners)
    owner = owners[0] if owners else None
    if owner is not None and 'safety_arbiter' not in _component(
            catalog, instances[owner])['roles']:
        raise FrameworkError('E_SAFETY_ARBITER', '%s is not safety_arbiter' % owner)
    advice = []
    for iid, instance in instances.items():
        for port in _component(catalog, instance)['outputs']:
            topic = port.get('command_topic')
            if not topic or topic == '/cmd_vel':
                continue
            endpoint = '%s.%s' % (iid, port['name'])
            if owner is None or not any(
                    edge['from'] == endpoint and edge['to'].startswith(owner + '.')
                    for edge in plan['edges']):
                raise FrameworkError('E_SAFETY_ADVICE',
                                     '%s not consumed by %s' % (endpoint, owner))
            advice.append({'endpoint': endpoint, 'topic': topic})
    validators = [iid for iid, instance in instances.items()
                  if 'trajectory_validator' in _component(catalog, instance)['roles']]
    validator_state = 'absent'
    if len(validators) > 1:
        raise FrameworkError('E_SAFETY_VALIDATOR', 'multiple validators %s' % validators)
    if validators:
        vid = validators[0]
        comp = _component(catalog, instances[vid])
        names = {port['name'] for port in comp['inputs']}
        if 'path' not in names or not any(port['name'] == 'path' for port in comp['outputs']):
            raise FrameworkError('E_SAFETY_VALIDATOR', '%s needs path in/out' % vid)
        has_in = any(edge['to'].startswith(vid + '.') for edge in plan['edges'])
        has_out = any(edge['from'].startswith(vid + '.') for edge in plan['edges'])
        if not (has_in and has_out):
            raise FrameworkError('E_SAFETY_VALIDATOR', '%s is not on the path' % vid)
        validator_state = 'present'
    return {'ok': True, 'command_owner': owner,
            'advice': sorted(advice, key=lambda item: item['endpoint']),
            'validator': validator_state}


def enforce(assembly, verdict):
    """Validator verdict gate: anything but explicit approval stops the assembly."""
    if not isinstance(verdict, dict) or 'approved' not in verdict:
        raise FrameworkError('E_SAFETY_VERDICT', 'verdict requires approved')
    if verdict['approved'] is True:
        return {'ok': True, 'stopped': False}
    assembly.stop()
    return {'ok': False, 'stopped': True,
            'reason': verdict.get('reason', 'validator_reject')}
