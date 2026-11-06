"""Execution-domain compatibility, checked before process/ROS side effects.

Mode classifies a deployment. It never grants permission to operate hardware.
Legacy graphs remain useful offline; execution requires positively classified
components. No ROS, package discovery, networking or process creation here.
"""
import hashlib
import ipaddress
import json

from config import FrameworkError

MODES = ('simulation', 'real_robot')
CONTEXT_KEYS = ('execution_mode', 'robot_id')


def context(value, required=False):
    present = [key in value for key in CONTEXT_KEYS]
    if not any(present):
        if required:
            raise FrameworkError('E_EXEC_MISSING', 'execution_mode and robot_id required')
        return None
    if not all(present):
        raise FrameworkError('E_EXEC_MISSING', 'execution_mode and robot_id must occur together')
    mode, robot = (value[key] for key in CONTEXT_KEYS)
    if not isinstance(mode, str) or mode not in MODES:
        raise FrameworkError('E_EXEC_MODE', repr(mode))
    if not isinstance(robot, str) or not robot.strip() or robot != robot.strip() or robot == '*':
        raise FrameworkError('E_EXEC_ROBOT', repr(robot))
    if mode == 'real_robot' and robot != 'lite3':
        raise FrameworkError('E_EXEC_ROBOT', 'real backend not implemented for ' + robot)
    return {'execution_mode': mode, 'robot_id': robot}


def merge_context(app, graph, real=False):
    app_context, graph_context = context(app), context(graph)
    if real and app_context is None:
        raise FrameworkError('E_EXEC_MISSING', 'real app requires explicit context')
    if app_context and graph_context and app_context != graph_context:
        raise FrameworkError('E_EXEC_CONTEXT', 'app and graph disagree')
    resolved = app_context or graph_context
    if resolved and (resolved['execution_mode'] == 'real_robot') != bool(real):
        raise FrameworkError('E_EXEC_MODE', '--real and configured mode disagree')
    return resolved


def declaration(component):
    cid = component.get('id', '<unknown>')
    value = component.get('execution')
    if not isinstance(value, dict) or set(value) - {'modes', 'robot_id', 'selectors'}:
        raise FrameworkError('E_EXEC_COMPONENT', cid + ': execution declaration required')
    launch = component.get('launch', {})
    if (not isinstance(launch, dict) or launch.get('kind') not in ('node', 'include', 'plugin', 'nodelet_group')
            or not isinstance(launch.get('package'), str) or not launch['package']):
        raise FrameworkError('E_EXEC_BACKEND', cid + ': unknown launch backend')
    modes = value.get('modes')
    if (not isinstance(modes, list) or not modes
            or any(not isinstance(mode, str) or mode not in MODES for mode in modes)
            or len(set(modes)) != len(modes) or 'robot_id' not in value):
        raise FrameworkError('E_EXEC_COMPONENT', cid + ': invalid modes/robot binding')
    robot = value['robot_id']
    if robot is not None and (not isinstance(robot, str) or not robot.strip()
                              or robot != robot.strip() or robot == '*'):
        raise FrameworkError('E_EXEC_ROBOT', cid + ': invalid robot binding')
    if 'real_robot' in modes:
        if component.get('launch', {}).get('test_double'):
            raise FrameworkError('E_EXEC_COMPONENT', cid + ': test double cannot run on hardware')
        if 'robot_adapter' in component.get('roles', []) and robot is None:
            raise FrameworkError('E_EXEC_ROBOT', cid + ': hardware adapter needs robot binding')
    selectors = value.get('selectors', [])
    if not isinstance(selectors, list):
        raise FrameworkError('E_EXEC_BACKEND', cid + ': selectors must be a list')
    names = set()
    for selector in selectors:
        if (not isinstance(selector, dict) or set(selector) != {'name', 'default', 'values'}
                or not isinstance(selector['name'], str) or not selector['name']
                or selector['name'] in names or not isinstance(selector['values'], list)
                or not selector['values']
                or any(not isinstance(v, str) or not v for v in selector['values'])
                or len(set(selector['values'])) != len(selector['values'])
                or not isinstance(selector['default'], str)
                or selector['default'] not in selector['values']):
            raise FrameworkError('E_EXEC_BACKEND', cid + ': invalid selector')
        names.add(selector['name'])
    return value


def effective_params(instance, component, load_yaml=None):
    """Materialize backend selectors with the same precedence as param_exec.

    Package param_files cannot be resolved without package discovery; selectors
    combined with those files fail closed. No current selector component has
    such files. Config paths in executable plans are absolute and hash bound.
    """
    value = declaration(component)
    params = dict(component['launch'].get('args', {}))
    if value.get('selectors') or component['id'] == 'v8.boundary.adapter':
        if component['launch'].get('param_files'):
            raise FrameworkError('E_EXEC_CONFIG', 'selector param_files need explicit materialization')
        if instance.get('config'):
            if load_yaml is None:
                raise FrameworkError('E_EXEC_CONFIG', 'config loader required for backend selectors')
            configured, sha = load_yaml(instance['config'])
            expected = instance.get('config_sha256')
            if expected is not None and sha != expected:
                raise FrameworkError('E_EXEC_CONFIG', 'backend config changed after planning')
            params.update(configured)
    params.update(instance.get('params', {}))
    for selector in value.get('selectors', []):
        name = selector['name']
        selected = params.get(name, selector['default'])
        if not isinstance(selected, str) or selected not in selector['values']:
            raise FrameworkError('E_EXEC_BACKEND', '%s.%s=%r' % (component['id'], name, selected))
        params[name] = selected
    return params


def loopback_endpoint(value, allow_zero=False):
    if not isinstance(value, str):
        raise FrameworkError('E_EXEC_ENDPOINT', repr(value))
    host, sep, port = value.rpartition(':')
    if not sep or not port.isdecimal():
        raise FrameworkError('E_EXEC_ENDPOINT', value)
    host = host[1:-1] if host.startswith('[') and host.endswith(']') else host
    if host == 'localhost':
        host = '127.0.0.1'  # canonical literal; never resolve a host name
    try:
        address = ipaddress.ip_address(host)
    except ValueError:
        raise FrameworkError('E_EXEC_ENDPOINT', value)
    # The current UDP backend uses AF_INET, not an IPv6 socket.
    if address.version != 4 or not address.is_loopback or not (0 if allow_zero else 1) <= int(port) <= 65535:
        raise FrameworkError('E_EXEC_ENDPOINT', value)
    return '%s:%d' % (('[' + host + ']') if address.version == 6 else host, int(port))


def check_component(instance, component, execution_context=None, load_yaml=None):
    value = declaration(component)
    resolved = context(execution_context or {})
    if resolved is None:
        if 'simulation' not in value['modes'] or value['robot_id'] is not None:
            raise FrameworkError('E_EXEC_MISSING', component['id'] + ': explicit context required')
        resolved = {'execution_mode': 'simulation', 'robot_id': 'generic_sim'}
    if resolved['execution_mode'] not in value['modes']:
        raise FrameworkError('E_EXEC_MODE', component['id'] + ': incompatible execution domain')
    if value['robot_id'] is not None and value['robot_id'] != resolved['robot_id']:
        raise FrameworkError('E_EXEC_ROBOT', component['id'] + ': robot binding mismatch')
    params = effective_params(instance, component, load_yaml)
    if component['id'] == 'v8.boundary.adapter':
        if resolved['execution_mode'] != 'simulation':
            raise FrameworkError('E_EXEC_MODE', 'UDP boundary currently supports simulation only')
        params['target'] = loopback_endpoint(params.get('target'))
        params['bind'] = loopback_endpoint(params.get('bind', '127.0.0.1:0'), allow_zero=True)
    return params


def component_digest(component):
    return hashlib.sha256(json.dumps(component, sort_keys=True, ensure_ascii=False,
                                     separators=(',', ':')).encode()).hexdigest()


def graph_input_digest(graph):
    """Bind parsed graph inputs, not the loader's cached file sha/raw mapping."""
    payload = {key: graph.get(key) for key in ('name', 'path', 'instances', 'edges', 'external')}
    payload['context'] = context(graph)
    return hashlib.sha256(json.dumps(payload, sort_keys=True, ensure_ascii=False,
                                     separators=(',', ':')).encode()).hexdigest()


def report_digest(report):
    """Tamper-evident checked report envelope, not an authorization signature."""
    payload = {key: value for key, value in report.items() if key != 'report_sha256'}
    try:
        encoded = json.dumps(payload, sort_keys=True, ensure_ascii=False, separators=(',', ':'))
    except (TypeError, ValueError) as error:
        raise FrameworkError('E_EXEC_REPORT', 'invalid checked report: ' + str(error))
    return hashlib.sha256(encoded.encode()).hexdigest()


def check_report(graph, catalog, report):
    """Never bless a changed catalog/graph using an earlier successful report."""
    required = ('source_graph_sha256', 'checked_components', 'catalog_sha256', 'report_sha256')
    if any(key not in report for key in required):
        raise FrameworkError('E_EXEC_REPORT', 'report lacks original bindings; run graph.check again')
    graph_context, report_context = context(graph), context(report)
    if ((graph_context is not None and graph_context != report_context)
            or (report.get('execution_origin') == 'declared' and graph_context != report_context)):
        raise FrameworkError('E_EXEC_CONTEXT', 'checked report and graph deployment identity disagree')
    if report_digest(report) != report['report_sha256']:
        raise FrameworkError('E_EXEC_REPORT', 'checked report changed after validation')
    if graph_input_digest(graph) != report['source_graph_sha256']:
        raise FrameworkError('E_EXEC_REPORT', 'graph changed after validation')
    if catalog.get('sha256') != report['catalog_sha256']:
        raise FrameworkError('E_EXEC_CATALOG', 'catalog changed after graph check')
    if set(report['checked_components']) != set(report['instances']):
        raise FrameworkError('E_EXEC_REPORT', 'checked component instances disagree')
    for iid, instance in report['instances'].items():
        binding = report['checked_components'][iid]
        cid = instance['component']
        component = catalog['components'].get(cid)
        if (binding.get('component') != cid or component is None
                or component_digest(component) != binding.get('sha256')):
            raise FrameworkError('E_EXEC_CATALOG', iid + ': component changed after graph check')
    return True


def plan_binding(plan):
    """Canonical deployment binding, not an operation permit or full-plan signature."""
    payload = {'schema': plan.get('schema'),
               'context': {key: plan.get(key) for key in CONTEXT_KEYS},
               'catalog': plan.get('catalog', {}).get('sha256'),
               'instances': sorted((i.get('id'), i.get('component'), i.get('component_sha256'))
                                   for i in plan.get('instances', []))}
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def check_plan(plan, catalog, load_yaml=None):
    """Recheck selected components against the current catalog; never trust ok."""
    if plan.get('schema') != 'assembly_plan/v2':
        raise FrameworkError('E_EXEC_PLAN', 'execution requires assembly_plan/v2')
    resolved = context(plan, required=True)
    if plan.get('execution_sha256') != plan_binding(plan):
        raise FrameworkError('E_EXEC_BINDING', 'deployment identity changed after planning')
    if plan.get('catalog', {}).get('sha256') != catalog.get('sha256'):
        raise FrameworkError('E_EXEC_CATALOG', 'plan/catalog digest mismatch')
    for instance in plan['instances']:
        component = catalog['components'].get(instance.get('component'))
        if component is None:
            raise FrameworkError('E_EXEC_COMPONENT', repr(instance.get('component')))
        if instance.get('component_sha256') != component_digest(component):
            raise FrameworkError('E_EXEC_CATALOG', instance['id'] + ': component changed after planning')
        if instance.get('launch') != component['launch']:
            raise FrameworkError('E_EXEC_CATALOG', instance['id'] + ': launch changed after planning')
        instance['params'] = check_component(instance, component, resolved, load_yaml)
    return resolved


def check_transition(old_plan, new_plan, catalog, load_yaml=None):
    old = check_plan(old_plan, catalog, load_yaml)
    new = check_plan(new_plan, catalog, load_yaml)
    if old != new:
        raise FrameworkError('E_EXEC_CONTEXT', 'hot-swap cannot change deployment identity')
    return new


def validate_app_request(app, resolved, real, with_gazebo):
    """Closed app backends; this runs before map IO and all execution effects."""
    own = context(app)
    if real and own is None:
        raise FrameworkError('E_EXEC_MISSING', 'real app requires explicit context')
    carrier, localization = app.get('carrier'), app.get('localization')
    if not isinstance(carrier, dict) or not isinstance(localization, dict):
        raise FrameworkError('E_EXEC_BACKEND', 'carrier/localization must be mappings')
    backend = carrier.get('backend')
    source = localization.get('source')
    if backend not in ('gazebo', 'kinematic', 'lite3'):
        raise FrameworkError('E_EXEC_BACKEND', 'unknown carrier ' + repr(backend))
    if source not in ('carrier', 'sim_gt', 'faster_lio', 'ndt'):
        raise FrameworkError('E_EXEC_BACKEND', 'unknown localization ' + repr(source))
    effective = resolved or own
    hardware = bool(effective and effective['execution_mode'] == 'real_robot')
    if hardware != bool(real):
        raise FrameworkError('E_EXEC_MODE', '--real and configured mode disagree')
    if real:
        if backend != 'lite3' or source != 'carrier' or with_gazebo:
            raise FrameworkError('E_EXEC_BACKEND', 'real requires Lite3 carrier, live pose and no Gazebo')
    elif backend not in ('gazebo', 'kinematic'):
        raise FrameworkError('E_EXEC_BACKEND', 'simulation requires an implemented simulator carrier')
    return effective
