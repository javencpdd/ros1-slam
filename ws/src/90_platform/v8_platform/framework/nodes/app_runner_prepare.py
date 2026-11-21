#!/usr/bin/env python3
"""app_runner prepare module — façade split (M9); external imports unchanged."""
import argparse
import copy
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
FRAMEWORK = HERE.parent
sys.path.insert(0, str(FRAMEWORK))
sys.path.insert(0, str((Path(__file__).resolve().parent / '../../../..' / '90_platform/v8_platform/contracts').resolve()))
import app_config  # noqa: E402
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402
import execution  # noqa: E402
import mission  # noqa: E402
import safety  # noqa: E402



def resolve_sim_launch(app, root=None):
    """Launch argv for a simulation carrier, declared by the app.

    The navigation platform never hard-codes a simulator package or path.  The
    app (owned by the simulation/scenario project) names its declaration; the
    platform only validates the world launch against that declaration.
    """
    root = Path(root or app_config.ROOT)
    declared = app.get('carrier', {}).get('simulation_declaration')
    if not declared:
        raise config.FrameworkError('E_EXEC_BACKEND', 'carrier.simulation_declaration is required')
    path = app_config._path(declared, root)
    if not path.is_file():
        raise config.FrameworkError('E_EXEC_BACKEND', 'simulation backend declaration missing: %s' % path)
    declaration, _ = config.load_yaml(path)
    if declaration.get('schema') != 'v8_simulation_backends/v1':
        raise config.FrameworkError('E_EXEC_BACKEND', 'unknown simulation backend schema')
    backend = app.get('carrier', {}).get('backend')
    entry = declaration.get('backends', {}).get(backend)
    if not isinstance(entry, dict):
        raise config.FrameworkError('E_EXEC_BACKEND', 'undeclared simulation backend ' + repr(backend))
    world_launch = app['carrier'].get('world_launch')
    if world_launch:
        candidate = app_config._path(world_launch, root).resolve()
        allowed = [app_config._path(item, root).resolve()
                   for item in entry.get('world_launch_allowlist', [])]
        if candidate not in allowed:
            raise config.FrameworkError('E_EXEC_BACKEND', 'unknown simulation world launch')
        return {'argv': [str(candidate)], 'declared': True}
    default = entry.get('legacy_default')
    if not isinstance(default, dict) or not default.get('package') or not default.get('launch'):
        raise config.FrameworkError('E_EXEC_BACKEND', 'no declared world launch and no legacy default')
    return {'argv': [default['package'], default['launch']], 'declared': False}


def _prepare_app(app_path, real=False, with_gazebo=False, root=None, executable=True):
    """Read/resolve/check only. No environment, directory, process or ROS effects."""
    raw, _ = config.load_yaml(app_path)
    if executable:
        execution.validate_app_request(raw, execution.context(raw), real, with_gazebo)
    root = Path(root or app_config.ROOT)
    source = raw.get('localization', {}).get('source')
    selected_path = raw.get('localization', {}).get('graph') if source != 'carrier' else None
    selected_path = app_config._path(selected_path or raw.get('graph', ''), root)
    graph = config.load_graph(selected_path)
    if executable:
        resolved_context = execution.merge_context(raw, graph, real)
        execution.validate_app_request(raw, resolved_context, real, with_gazebo)
    else:
        app_context, graph_context = execution.context(raw), execution.context(graph)
        if app_context and graph_context and app_context != graph_context:
            raise config.FrameworkError('E_EXEC_CONTEXT', 'app and graph disagree')
        resolved_context = app_context or graph_context
    catalog = config.load_components(FRAMEWORK / 'fixtures' / 'manifests')
    origin = 'declared' if resolved_context else 'unclassified'
    if executable and resolved_context is None:
        # The narrow legacy fallback is simulation only, and only positively
        # classified, robot-independent components may enter it.
        for instance in graph['instances'].values():
            component = catalog['components'].get(instance['component'])
            if component is None:
                raise config.FrameworkError('E_EXEC_COMPONENT', instance['component'])
            checked_instance = dict(instance)
            if checked_instance.get('config'):
                checked_instance['config'] = str(app_config._path(checked_instance['config'], selected_path.parent))
            execution.check_component(checked_instance, component, load_yaml=config.load_yaml)
        resolved_context = {'execution_mode': 'simulation', 'robot_id': 'generic_sim'}
        origin = 'legacy_sim'
    if resolved_context:
        graph.update(resolved_context)
    carrier_id = {'gazebo': 'v8.fw.gazebo_carrier3d',
                  'kinematic': 'v8.fw.carrier3d', 'lite3': 'v8.fw.carrier_lite3'}.get(
                      raw.get('carrier', {}).get('backend'))
    carrier = graph['instances'].get('carrier')
    if carrier_id is None or carrier is None or carrier['component'] != carrier_id:
        raise config.FrameworkError('E_EXEC_BACKEND', 'app backend and actual graph carrier disagree')
    if raw['carrier'].get('backend') == 'lite3' and raw['carrier'].get('pose_source', 'tf') not in ('tf', 'topic'):
        raise config.FrameworkError('E_EXEC_BACKEND', 'unknown Lite3 pose source')
    sim_launch = (resolve_sim_launch(raw, root)
                  if raw.get('carrier', {}).get('backend') == 'gazebo' else None)
    resolved = app_config.load_app(app_path, root=root)
    task = resolved['task_resolved']
    start = tuple(float(v) for v in task['start']['xyz'])
    # Fill the graph BEFORE check/hash, including selector overrides.
    for instance in graph['instances'].values():
        instance['params'] = copy.deepcopy(instance['params'])
        if instance['id'] == 'map':
            instance['params'].update(map_json=str(resolved['map_path']), map_topic='/v8_support_map')
        if instance['id'] == 'carrier':
            instance['params'].update(start_x=start[0], start_y=start[1], start_z=start[2])
            if raw['carrier']['backend'] == 'gazebo':
                instance['params'].update(model_name=raw['carrier'].get('model_name', 'app_carrier'),
                                         center_z=raw['carrier'].get('center_z', .1),
                                         snap_m=raw['carrier'].get('snap_m', .03), spawn=True)
            if raw['carrier']['backend'] == 'lite3':
                for key, default in (('pose_source', 'tf'), ('map_frame', 'map'),
                                     ('base_frame', 'base_link'), ('input_pose_topic', '/odom'),
                                     ('rate_hz', 20.0), ('stale_s', .5)):
                    instance['params'][key] = raw['carrier'].get(key, default)
        if instance['id'] == 'loc':
            instance['params']['source'] = raw['localization']['source']
        if instance['component'] == 'v8.fw.bt_console' and instance['params'].get('tree_file'):
            instance['params']['tree_file'] = str(app_config._path(instance['params']['tree_file'], root))
    report = graph_mod.check(graph, catalog)
    plan = render.render_plan(graph, catalog, report)
    plan['execution_origin'] = origin if origin != 'unclassified' else report['execution_origin']
    if executable:
        execution.check_plan(plan, catalog, config.load_yaml)
    safety.check_plan(plan, catalog)
    return {'resolved': resolved, 'catalog': catalog, 'graph': graph, 'report': report,
            'plan': plan, 'sim_launch': sim_launch}


def prepare_execution(app_path, real=False, with_gazebo=False, root=None):
    return _prepare_app(app_path, real, with_gazebo, root, executable=True)


def dry_run(app_path):
    prepared = _prepare_app(app_path, executable=False)
    resolved, report, plan = (prepared[key] for key in ('resolved', 'report', 'plan'))
    return {
        'scope': 'navigation_app_resolution', 'ok': True,
        'app': str(resolved['app_path']), 'app_name': resolved['app']['name'],
        'map': {'entry': resolved['map_entry'], 'path': str(resolved['map_path']),
                'sha256': resolved['map_sha256']},
        'task_resolved': resolved['task_resolved'],
        'graph': {'path': str(resolved['graph_path']), 'digest': report['digest'],
                  'startup_order': report['startup_order'],
                  'command_writers': report['command_writers']},
        'execution': execution.context(plan), 'execution_origin': plan['execution_origin'],
        'plan': plan,
    }

