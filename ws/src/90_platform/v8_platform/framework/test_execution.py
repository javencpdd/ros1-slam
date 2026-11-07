"""Hermetic execution-domain controls: no subprocess/ROS/network is started."""
import builtins
import copy
import hashlib
import io
import json
import os
import sys
import tempfile
import unittest
from contextlib import contextmanager, ExitStack, redirect_stdout
from pathlib import Path
from unittest.mock import Mock, call, patch

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import app_runner
import config
import execution
import graph as graph_mod
import hotswap
import render
import runtime
import selector
import verification

MANIFESTS = HERE / 'fixtures/manifests'
GRAPHS = HERE / 'fixtures/graphs'
LITE3 = HERE.parent / 'configs/lite3/graph_3d_lite3.yaml'


def plan_for(path):
    catalog = config.load_components(MANIFESTS)
    graph = config.load_graph(path)
    return render.render_plan(graph, catalog, graph_mod.check(graph, catalog)), catalog


@contextmanager
def no_execution_effects():
    original_import = builtins.__import__
    attempted_imports = []

    def guarded_import(name, *args, **kwargs):
        if name.split('.')[0] in ('rospy', 'rosgraph', 'geometry_msgs', 'nav_msgs',
                                  'std_msgs', 'visualization_msgs', 'gazebo_msgs'):
            attempted_imports.append(name)
            raise AssertionError('ROS import attempted: ' + name)
        return original_import(name, *args, **kwargs)

    before = dict(os.environ)
    with ExitStack() as stack:
        popen = stack.enter_context(patch('subprocess.Popen', side_effect=AssertionError('Popen attempted')))
        output = stack.enter_context(patch('subprocess.check_output', side_effect=AssertionError('discovery attempted')))
        runner = stack.enter_context(patch.object(runtime, 'ProcessRunner', side_effect=AssertionError('runner attempted')))
        mkdir = stack.enter_context(patch.object(Path, 'mkdir', side_effect=AssertionError('mkdir attempted')))
        stack.enter_context(patch('builtins.__import__', side_effect=guarded_import))
        yield
        assert not attempted_imports, attempted_imports
        for sentinel in (popen, output, runner, mkdir):
            sentinel.assert_not_called()
        assert dict(os.environ) == before, 'environment changed before validation'


class AppFixture:
    """Owned temporary map/registry/task; never writes local deployment assets."""
    def __init__(self, directory, hardware=False):
        self.root = Path(directory)
        mapping = json.loads((HERE.parent / 'contracts/fixtures/map_ml_two_surfaces.json').read_text())
        map_path = self.root / 'map.json'
        map_path.write_text(json.dumps(mapping))
        entry = {'map_id': mapping['map_id'], 'version': mapping['map_version'],
                 'path': str(map_path), 'sha256': hashlib.sha256(map_path.read_bytes()).hexdigest()}
        registry = self.root / 'registry.json'
        registry.write_text(json.dumps({'schema': 'map_registry/v1', 'maps': [entry]}))
        surface = mapping['surfaces'][0]
        footprint = surface['footprint']
        x = (footprint['min_x'] + footprint['max_x']) / 2
        y = (footprint['min_y'] + footprint['max_y']) / 2
        task_path = self.root / 'task.json'
        task_path.write_text(json.dumps({'map': {'map_id': entry['map_id'], 'map_version': entry['version']},
                                        'start': {'xyz': [x, y, surface['z_reference']], 'yaw': 0},
                                        'goal': {'xyz': [x, y, surface['z_reference']], 'yaw': 0,
                                                 'surface_hint': surface['surface_id']}}))
        self.value = {'schema': 'navigation_app/v1', 'name': 'owned_execution_fixture',
                      'execution_mode': 'real_robot' if hardware else 'simulation',
                      'robot_id': 'lite3' if hardware else 'generic_sim',
                      'graph': str(LITE3 if hardware else GRAPHS / 'graph_3d_gazebo_app.yaml'),
                      'map_registry': str(registry),
                      'map': {'id': entry['map_id'], 'version': entry['version']},
                      'task': str(task_path), 'localization': {'source': 'carrier'},
                      'carrier': {'backend': 'lite3' if hardware else 'gazebo'}}
        self.path = self.root / 'app.yaml'

    def save(self):
        self.path.write_text(yaml.safe_dump(self.value))
        return self.path


class ExecutionTests(unittest.TestCase):
    def test_owned_gazebo_clock_is_selected_before_node_initialization(self):
        ros = Mock()
        app_runner.initialize_ros_node(ros, gazebo_clock=True)
        self.assertEqual(ros.method_calls[0], call.set_param('/use_sim_time', True))
        self.assertEqual(ros.method_calls[1][0], 'init_node')
        ros.reset_mock()
        app_runner.initialize_ros_node(ros)
        ros.set_param.assert_not_called()
        ros.init_node.assert_called_once_with('navigation_app_runner', anonymous=True, disable_signals=True)

    def test_model_trace_sampling_keeps_current_sample_time_and_reports_errors(self):
        from app_trace import Trace
        clocks = {'wall': 1., 'ros': 20.}
        trace = Trace({})
        convert = Mock(side_effect=lambda value: {'sample': value})
        observe = app_runner.trace_observer(trace, 'gazebo_model', convert,
                    lambda: clocks['ros'], lambda: clocks['wall'], period_s=.02)
        observe('first')
        clocks.update(wall=1.01, ros=21.); observe('skipped')
        clocks.update(wall=1.03, ros=22.); observe('current')
        events = trace.snapshot()['events']
        self.assertEqual([e['payload']['sample'] for e in events], ['first', 'current'])
        self.assertEqual([e['wall_monotonic'] for e in events], [1., 1.03])
        self.assertEqual([e['ros_time'] for e in events], [20., 22.])
        self.assertEqual(convert.call_count, 2)
        clocks['wall'] = 1.06; convert.side_effect = ValueError('bad observation')
        observe('broken')
        self.assertFalse(trace.snapshot()['capture_ok'])

    def rejected(self, code, callback):
        with no_execution_effects():
            with self.assertRaises(config.FrameworkError) as caught:
                callback()
            self.assertEqual(caught.exception.code, code)

    def test_repository_root_resolution_does_not_spawn_git_before_preflight(self):
        import paths
        with patch.dict(os.environ):
            os.environ.pop('ROS1_SLAM_ROOT', None)
            with no_execution_effects():
                self.assertTrue((paths.repo_root(HERE / 'app_runner.py') / 'ws/src').is_dir())

    def test_declared_sim_and_real_plans_are_positive_without_processes(self):
        with no_execution_effects():
            for path, mode, robot in ((GRAPHS / 'graph_2d_sim.yaml', 'simulation', 'test_b'),
                                      (LITE3, 'real_robot', 'lite3')):
                plan, catalog = plan_for(path)
                self.assertEqual(plan['schema'], 'assembly_plan/v2')
                self.assertEqual(execution.check_plan(plan, catalog, config.load_yaml),
                                 {'execution_mode': mode, 'robot_id': robot})

    def test_owned_real_and_sim_app_preparation_has_no_execution_effects(self):
        with tempfile.TemporaryDirectory() as directory:
            for hardware in (False, True):
                fixture = AppFixture(directory, hardware)
                path = fixture.save()
                with no_execution_effects():
                    prepared = app_runner.prepare_execution(path, real=hardware, root=directory)
                self.assertEqual(prepared['plan']['robot_id'], 'lite3' if hardware else 'generic_sim')
                map_instance = next(i for i in prepared['plan']['instances'] if i['id'] == 'map')
                self.assertEqual(map_instance['params']['map_json'], str(Path(directory) / 'map.json'))

    def test_app_execute_rejects_before_effects(self):
        cases = [
            ('E_EXEC_MODE', {'execution_mode': 'simulation', 'robot_id': 'generic_sim'}, True, False),
            ('E_EXEC_BACKEND', {'carrier': {'backend': 'gazebo'}}, True, False),
            ('E_EXEC_BACKEND', {'localization': {'source': 'sim_gt'}}, True, False),
            ('E_EXEC_BACKEND', {}, True, True),
            ('E_EXEC_BACKEND', {'carrier': {'backend': 'm20'}}, True, False),
            ('E_EXEC_BACKEND', {'carrier': {'backend': 'lite3', 'pose_source': 'unknown'}}, True, False),
            ('E_EXEC_ROBOT', {'robot_id': 'm20'}, True, False),
            ('E_EXEC_MODE', {}, False, False),
        ]
        with tempfile.TemporaryDirectory() as directory:
            for code, changes, real, gazebo in cases:
                with self.subTest(code=code, changes=changes, real=real):
                    fixture = AppFixture(directory, True)
                    fixture.value.update(changes)
                    path = fixture.save()
                    self.rejected(code, lambda: app_runner.execute(path, '/unused/out', 12345, gazebo, False, 1, real=real))

    def test_real_app_missing_or_partial_context_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            for fields in (('execution_mode', 'robot_id'), ('execution_mode',), ('robot_id',)):
                fixture = AppFixture(directory, True)
                for key in fields:
                    fixture.value.pop(key)
                path = fixture.save()
                self.rejected('E_EXEC_MISSING', lambda: app_runner.prepare_execution(path, real=True))

    def test_app_graph_context_and_carrier_cannot_be_overridden(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = AppFixture(directory, True)
            fixture.value['graph'] = str(GRAPHS / 'graph_3d_gazebo_app.yaml')
            path = fixture.save()
            self.rejected('E_EXEC_CONTEXT', lambda: app_runner.prepare_execution(path, real=True))
            fixture = AppFixture(directory)
            fixture.value['carrier'] = {'backend': 'kinematic'}
            path = fixture.save()
            self.rejected('E_EXEC_BACKEND', lambda: app_runner.prepare_execution(path))

    def test_legacy_sim_app_requires_only_positive_sim_components(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = AppFixture(directory)
            fixture.value.pop('execution_mode'); fixture.value.pop('robot_id')
            raw = yaml.safe_load((GRAPHS / 'graph_3d_gazebo_app.yaml').read_text())
            raw.pop('execution_mode'); raw.pop('robot_id')
            for instance in raw['instances']:
                if instance.get('config'):
                    instance['config'] = str((GRAPHS / instance['config']).resolve())
            path = Path(directory) / 'legacy_graph.yaml'
            path.write_text(yaml.safe_dump(raw))
            fixture.value['graph'] = str(path)
            with no_execution_effects():
                prepared = app_runner.prepare_execution(fixture.save(), root=directory)
            self.assertEqual(prepared['plan']['execution_origin'], 'legacy_sim')
            self.assertEqual(prepared['plan']['execution_mode'], 'simulation')

    def test_offline_mixed_graph_remains_analyzable_but_never_executable(self):
        plan, catalog = plan_for(GRAPHS / 'graph_2d.yaml')
        self.assertNotIn('execution_mode', plan)
        factory = Mock(side_effect=AssertionError('factory attempted'))
        self.rejected('E_EXEC_MISSING', lambda: runtime.Assembly(plan, catalog, command_factory=factory).start())
        factory.assert_not_called()
        graph = config.load_graph(GRAPHS / 'graph_2d.yaml')
        self.rejected('E_EXEC_MISSING', lambda: render.render_launch(graph, catalog, graph_mod.check(graph, catalog)))

    def test_simulation_cannot_resolve_bridge_or_hardware_middleware(self):
        for cid in ('v8.fw.robot_lite3', 'v8.fw.carrier_lite3'):
            with self.subTest(component=cid):
                plan, catalog = plan_for(GRAPHS / 'graph_2d_sim.yaml')
                item = plan['instances'][0]
                item.update(component=cid, launch=copy.deepcopy(catalog['components'][cid]['launch']),
                            component_sha256=execution.component_digest(catalog['components'][cid]))
                plan['execution_sha256'] = execution.plan_binding(plan)
                factory = Mock()
                self.rejected('E_EXEC_MODE', lambda: runtime.Assembly(plan, catalog, command_factory=factory).start())
                factory.assert_not_called()

    def test_real_plan_rejects_simulation_and_wrong_identity(self):
        plan, catalog = plan_for(LITE3)
        for cid in ('v8.fw.gazebo_carrier3d', 'v8.fw.localization_source'):
            bad = copy.deepcopy(plan)
            item = bad['instances'][0]
            item.update(component=cid, launch=copy.deepcopy(catalog['components'][cid]['launch']),
                        component_sha256=execution.component_digest(catalog['components'][cid]))
            bad['execution_sha256'] = execution.plan_binding(bad)
            self.rejected('E_EXEC_MODE', lambda: runtime.Assembly(bad, catalog).start())
        plan['robot_id'] = 'm20'
        self.rejected('E_EXEC_ROBOT', lambda: runtime.Assembly(plan, catalog).start())

    def test_v2_manifest_loads_and_analyzes_offline_without_execution_grant(self):
        value = json.loads((MANIFESTS / 'manifest_map_file.json').read_text())
        value['schema_version'] = 2; value.pop('execution')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'manifest_old.json').write_text(json.dumps(value))
            graph_path = root / 'graph.yaml'
            graph_path.write_text(yaml.safe_dump({'schema': 'pipeline_graph/v1', 'name': 'old',
                'instances': [{'id': 'map', 'component': value['id']}]}))
            with no_execution_effects():
                catalog = config.load_components(root)
                graph = config.load_graph(graph_path)
                plan = render.render_plan(graph, catalog, graph_mod.check(graph, catalog))
            self.assertNotIn('execution_mode', plan)
            self.rejected('E_EXEC_MISSING', lambda: runtime.Assembly(plan, catalog).start())

    def test_cli_offline_plan_is_available_but_runnable_mixed_output_is_refused(self):
        import cli
        with tempfile.TemporaryDirectory() as directory:
            plan_path, launch_path = Path(directory) / 'plan.json', Path(directory) / 'graph.launch'
            argv = ['plan', str(MANIFESTS), str(GRAPHS / 'graph_2d.yaml'), '--out-plan', str(plan_path)]
            with no_execution_effects(), redirect_stdout(io.StringIO()):
                self.assertEqual(cli.main(argv), 0)
            self.assertNotIn('execution_mode', json.loads(plan_path.read_text()))
            plan_path.unlink()
            with no_execution_effects(), redirect_stdout(io.StringIO()):
                self.assertEqual(cli.main(argv + ['--out-launch', str(launch_path)]), 5)
            self.assertFalse(plan_path.exists()); self.assertFalse(launch_path.exists())

    def test_legacy_sim_graph_emits_explicit_bound_plan(self):
        catalog = config.load_components(MANIFESTS)
        graph = config.load_graph(GRAPHS / 'graph_global3d_topic.yaml')
        graph.pop('execution_mode'); graph.pop('robot_id')
        with no_execution_effects():
            report = graph_mod.check(graph, catalog)
            plan = render.render_plan(graph, catalog, report)
            execution.check_plan(plan, catalog, config.load_yaml)
        self.assertEqual(plan['execution_origin'], 'legacy_sim')
        self.assertEqual(plan['execution_mode'], 'simulation')
        self.assertEqual(plan['robot_id'], 'generic_sim')

    def test_unclassified_component_and_old_plan_are_runtime_rejected(self):
        plan, catalog = plan_for(GRAPHS / 'graph_2d_sim.yaml')
        component = catalog['components'][plan['instances'][0]['component']]
        component.pop('execution')
        item = plan['instances'][0]
        item['component_sha256'] = execution.component_digest(component)
        plan['execution_sha256'] = execution.plan_binding(plan)
        self.rejected('E_EXEC_COMPONENT', lambda: runtime.Assembly(plan, catalog).start())
        plan['schema'] = 'assembly_plan/v1'
        self.rejected('E_EXEC_PLAN', lambda: runtime.Assembly(plan, catalog).start())

    def test_catalog_mutation_with_old_sha_cannot_change_launch_or_declaration(self):
        for field in ('launch', 'execution'):
            plan, catalog = plan_for(GRAPHS / 'graph_2d_sim.yaml')
            item = plan['instances'][0]
            component = catalog['components'][item['component']]
            self.assertIsNot(item['launch'], component['launch'])
            if field == 'launch':
                component['launch'].update(package='message_transformer', type='ros2qnx')
            else:
                component['execution']['modes'] = ['real_robot']
            self.rejected('E_EXEC_CATALOG', lambda: runtime.Assembly(plan, catalog, command_factory=Mock()).start())

    def test_render_refuses_catalog_mutation_after_graph_check_before_any_new_seal(self):
        for renderer in (render.render_plan, render.render_launch):
            with self.subTest(renderer=renderer.__name__):
                catalog = config.load_components(MANIFESTS)
                graph = config.load_graph(GRAPHS / 'graph_global3d_topic.yaml')
                report = graph_mod.check(graph, catalog)
                with no_execution_effects():
                    renderer(graph, catalog, report)  # unchanged snapshot is valid
                original_sha, original_digest = catalog['sha256'], report['digest']
                cid = report['instances']['global']['component']
                catalog['components'][cid]['launch'].update(package='message_transformer', type='ros2qnx')
                self.assertEqual(catalog['sha256'], original_sha)
                self.assertEqual(report['digest'], original_digest)
                self.rejected('E_EXEC_CATALOG', lambda: renderer(graph, catalog, report))

    def test_graph_digest_contains_original_selected_component_hashes(self):
        catalog = config.load_components(MANIFESTS)
        graph = config.load_graph(GRAPHS / 'graph_global3d_topic.yaml')
        before = graph_mod.check(graph, catalog)
        catalog['components']['v8.fw.global3d_topic']['launch']['ready_marker'] = 'NEW_READY'
        after = graph_mod.check(graph, catalog)
        self.assertNotEqual(before['digest'], after['digest'])
        self.assertNotEqual(before['checked_components'], after['checked_components'])
        with no_execution_effects():
            plan = render.render_plan(graph, catalog, after)
        self.assertEqual(plan['instances'][0]['component_sha256'],
                         after['checked_components']['global']['sha256'])
        self.rejected('E_EXEC_CATALOG', lambda: render.render_plan(graph, catalog, before))

    def test_render_refuses_changed_graph_context_or_report_context(self):
        for renderer in (render.render_plan, render.render_launch):
            for changed in ('graph', 'report', 'both', 'params', 'name'):
                with self.subTest(renderer=renderer.__name__, changed=changed):
                    catalog = config.load_components(MANIFESTS)
                    graph = config.load_graph(GRAPHS / 'graph_global3d_topic.yaml')
                    report = graph_mod.check(graph, catalog)
                    if changed in ('graph', 'both'):
                        graph['robot_id'] = 'lite3'
                    if changed in ('report', 'both'):
                        report['robot_id'] = 'lite3'
                    if changed == 'params':
                        graph['instances']['global']['params']['budget'] = 12345
                    if changed == 'name':
                        graph['name'] = 'different_after_check'
                    code = 'E_EXEC_CONTEXT' if changed in ('graph', 'report') else 'E_EXEC_REPORT'
                    self.rejected(code, lambda: renderer(graph, catalog, report))

    def test_render_requires_bound_report_even_for_old_unclassified_inputs(self):
        for renderer in (render.render_plan, render.render_launch):
            for key in ('source_graph_sha256', 'checked_components', 'catalog_sha256', 'report_sha256'):
                catalog = config.load_components(MANIFESTS)
                graph = config.load_graph(GRAPHS / 'graph_2d.yaml')
                report = graph_mod.check(graph, catalog)
                report.pop(key)
                self.rejected('E_EXEC_REPORT', lambda: renderer(graph, catalog, report))

    def test_checked_report_snapshot_does_not_alias_graph_catalog_or_new_plan(self):
        catalog = config.load_components(MANIFESTS)
        graph = config.load_graph(GRAPHS / 'graph_3d_gazebo_app.yaml')
        graph['instances']['global']['params']['nested'] = {'source': 'original'}
        report = graph_mod.check(graph, catalog)
        original_report = copy.deepcopy(report)
        with no_execution_effects():
            plan = render.render_plan(graph, catalog, report)
            first = render.render_launch(graph, catalog, report)
            self.assertEqual(first, render.render_launch(graph, catalog, report))
        plan['startup_order'].reverse()
        plan['instances'][0]['inputs']['new'] = {'source': 'modified_plan'}
        graph['instances']['global']['params']['nested']['source'] = 'changed_graph'
        catalog['components']['v8.fw.global3d_topic']['inputs'][0]['topic'] = '/changed_catalog'
        self.assertEqual(report, original_report)
        self.rejected('E_EXEC_REPORT', lambda: render.render_plan(graph, catalog, report))

    def test_mode_and_identity_change_digest(self):
        catalog = config.load_components(MANIFESTS)
        graph = config.load_graph(GRAPHS / 'graph_global3d_topic.yaml')
        original = graph_mod.check(graph, catalog)['digest']
        graph['robot_id'] = 'lite3'
        changed_identity = graph_mod.check(graph, catalog)['digest']
        graph['execution_mode'] = 'real_robot'
        changed_mode = graph_mod.check(graph, catalog)['digest']
        self.assertEqual(len({original, changed_identity, changed_mode}), 3)

    def test_stale_deployment_binding_is_refused_and_checks_are_idempotent(self):
        plan, catalog = plan_for(GRAPHS / 'graph_global3d_topic.yaml')
        original = copy.deepcopy(plan)
        with no_execution_effects():
            execution.check_plan(plan, catalog, config.load_yaml)
            execution.check_plan(plan, catalog, config.load_yaml)
        self.assertEqual(plan, original)
        plan['robot_id'] = 'lite3'  # This component is robot-independent; binding still must catch it.
        factory = Mock()
        self.rejected('E_EXEC_BINDING', lambda: runtime.Assembly(plan, catalog, command_factory=factory).start())
        factory.assert_not_called()
        plan = copy.deepcopy(original); plan.update(execution_mode='real_robot', robot_id='lite3')
        self.rejected('E_EXEC_BINDING', lambda: runtime.Assembly(plan, catalog, command_factory=factory).start())

    def test_independent_command_refuses_real_missing_mode_before_rospack_or_files(self):
        catalog = config.load_components(MANIFESTS)
        self.rejected('E_EXEC_MISSING', lambda: runtime.default_command(
            {'id': 'real', 'params': {}}, catalog['components']['v8.fw.faster_lio']))
        unknown = copy.deepcopy(catalog['components']['v8.fw.map2d'])
        unknown.pop('execution')
        self.rejected('E_EXEC_COMPONENT', lambda: runtime.default_command({'id': 'old', 'params': {}}, unknown))

    def test_selector_defaults_are_materialized_and_unknown_values_rejected(self):
        catalog = config.load_components(MANIFESTS)
        component = catalog['components']['v8.fw.localization_source']
        with no_execution_effects():
            command, _ = runtime.default_command({'id': 'loc', 'params': {}}, component)
        self.assertIn('_source:=sim_gt', command)
        self.rejected('E_EXEC_BACKEND', lambda: runtime.default_command(
            {'id': 'loc', 'params': {'source': 'vendor_hardware'}}, component))

    def test_selector_checks_effective_config_precedence_and_hash(self):
        catalog = config.load_components(MANIFESTS)
        component = catalog['components']['v8.fw.localization_source']
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'params.yaml'
            path.write_text('source: unknown_backend\n')
            item = {'id': 'loc', 'params': {}, 'config': str(path)}
            self.rejected('E_EXEC_BACKEND', lambda: execution.check_component(item, component, load_yaml=config.load_yaml))
            item['params']['source'] = 'faster_lio'
            with no_execution_effects():
                params = execution.check_component(item, component, load_yaml=config.load_yaml)
            self.assertEqual(params['source'], 'faster_lio')
            item['config_sha256'] = 'stale'
            self.rejected('E_EXEC_CONFIG', lambda: execution.check_component(item, component, load_yaml=config.load_yaml))
            item.pop('config_sha256')
            component['launch']['param_files'] = ['unresolved.yaml']
            self.rejected('E_EXEC_CONFIG', lambda: execution.check_component(item, component, load_yaml=config.load_yaml))

    def test_boundary_local_endpoint_controls_and_render_use_the_checked_params(self):
        catalog = config.load_components(MANIFESTS)
        component = catalog['components']['v8.boundary.adapter']
        for target in (None, '10.0.0.1:1234', 'localhost.evil:1234', 'http://127.0.0.1:1234',
                       '127.0.0.1:0', '127.0.0.1:65536', '127.0.0.1:-1', '[::1]:1234'):
            self.rejected('E_EXEC_ENDPOINT', lambda: execution.check_component(
                {'params': {'target': target}}, component))
        self.rejected('E_EXEC_ENDPOINT', lambda: execution.check_component(
            {'params': {'target': '127.0.0.1:1234', 'bind': '0.0.0.0:0'}}, component))
        graph = config.load_graph(GRAPHS / 'graph_boundary_mux.yaml')
        graph.pop('execution_mode'); graph.pop('robot_id')
        graph['instances']['boundary']['params']['target'] = 'localhost:5026'
        graph['instances']['boundary']['params'].pop('bind')
        with no_execution_effects():
            report = graph_mod.check(graph, catalog)
            xml = render.render_launch(graph, catalog, report)
        self.assertIn('name="target" value="127.0.0.1:5026"', xml)
        self.assertIn('name="bind" value="127.0.0.1:0"', xml)

    def test_all_hot_swap_paths_refuse_forged_record_before_stop_or_callbacks(self):
        plan, catalog = plan_for(GRAPHS / 'graph_2d_sim.yaml')
        variant = copy.deepcopy(catalog['components']['v8.fw.map2d'])
        variant['id'] = 'test.map_variant'
        catalog['components'][variant['id']] = variant
        record, new_plan = hotswap.prepare(plan, catalog, 'map', variant['id'])
        assembly = runtime.Assembly(plan, catalog, command_factory=Mock())
        assembly.stop = Mock(side_effect=AssertionError('old stop attempted'))
        callback = Mock(side_effect=AssertionError('callback attempted'))
        with no_execution_effects():
            hotswap._check_apply(assembly, catalog, record, new_plan)
        record['new_component'] = 'v8.fw.robot_lite3'
        for apply in (hotswap.apply, hotswap.apply_overlap, hotswap.apply_instance):
            self.rejected('E_SWAP_RECORD', lambda: apply(assembly, catalog, record, new_plan))
        self.rejected('E_SWAP_RECORD', lambda: hotswap.apply_instance(
            assembly, catalog, record, new_plan, pre_stop=callback))
        assembly.stop.assert_not_called(); assembly.command_factory.assert_not_called(); callback.assert_not_called()

    def test_hot_swap_cannot_change_mode_or_robot_binding(self):
        plan, catalog = plan_for(GRAPHS / 'graph_2d_sim.yaml')
        assembly = runtime.Assembly(plan, catalog, command_factory=Mock())
        assembly.stop = Mock()
        record = {'schema_version': 1, 'task_id': 'x', 'instance_id': 'map',
                  'old_component': 'v8.fw.map2d', 'new_component': 'v8.fw.map2d',
                  'state': {}, 'epoch': 0, 'preconditions': {}, 'postconditions': {}}
        changed = copy.deepcopy(plan); changed['robot_id'] = 'other_robot'
        changed['execution_sha256'] = execution.plan_binding(changed)
        for apply in (hotswap.apply, hotswap.apply_overlap, hotswap.apply_instance):
            self.rejected('E_EXEC_ROBOT', lambda: apply(assembly, catalog, record, changed))
        assembly.stop.assert_not_called(); assembly.command_factory.assert_not_called()

    def test_selector_export_keeps_identity_and_rejects_real_variant(self):
        catalog = config.load_components(MANIFESTS)
        graph = config.load_graph(GRAPHS / 'graph_2d_sim.yaml')
        with no_execution_effects():
            data = selector.variant_matrix(graph, catalog)
        self.assertEqual(data['execution_mode'], 'simulation')
        robot = next(i for i in data['instances'] if i['id'] == 'robot')
        self.assertFalse(next(v for v in robot['variants'] if v['component'] == 'v8.fw.robot_lite3')['ok'])
        self.assertIn('out.robot_id = DATA.robot_id', selector._html(data, str(MANIFESTS), 'x'))

    def test_hardware_plan_cannot_attach_simulation_verification_backend(self):
        plan, catalog = plan_for(LITE3)
        backends = verification.load_backends(HERE / 'fixtures/verification_manifests')
        spec = verification.load_config(HERE / 'fixtures/verification_gazebo.yaml')
        packages = __import__('launch').load_index(HERE / 'fixtures/package_index.json')
        self.rejected('E_EXEC_MODE', lambda: verification.check(spec, plan, catalog, backends, packages))


if __name__ == '__main__':
    unittest.main()
