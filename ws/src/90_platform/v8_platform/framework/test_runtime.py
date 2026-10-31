"""F3: assembly plan through ProcessRunner/Orchestrator with fake commands."""
import copy
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402
import orchestrator as orch  # noqa: E402

MANIFESTS = HERE / 'fixtures' / 'manifests'
G2D = HERE / 'fixtures' / 'graphs' / 'graph_2d.yaml'
G2D_STACK = HERE / 'fixtures' / 'graphs' / 'graph_2d_stack_gp_teb.yaml'
BOUNDARY = HERE / 'fixtures' / 'graphs' / 'graph_boundary_mux.yaml'
GPLUGIN = HERE / 'fixtures' / 'graphs' / 'graph_plugin_nodelet.yaml'
GSEMANTIC = HERE / 'fixtures' / 'graphs' / 'graph_semantic.yaml'
GGROUP = HERE / 'fixtures' / 'graphs' / 'graph_nodelet_group.yaml'
GMAPCACHE = HERE / 'fixtures' / 'graphs' / 'graph_map_cache_ros.yaml'
G_LOCAL_NATIVE = HERE / 'fixtures' / 'graphs' / 'graph_local_native.yaml'
PY = sys.executable
SLEEPER = "import time; print('READY', flush=True); time.sleep(3600)"


def plan_for(path):
    catalog = config.load_components(MANIFESTS)
    graph = config.load_graph(path)
    report = graph_mod.check(graph, catalog)
    return render.render_plan(graph, catalog, report), catalog


def fake(_instance, _component):
    return [PY, '-c', SLEEPER], 'READY'


class RuntimeTests(unittest.TestCase):
    def test_start_ready_stop_order_and_cleanup(self):
        plan, catalog = plan_for(G2D)
        assembly = runtime.Assembly(plan, catalog, command_factory=fake,
                                    ready_timeout=5)
        assembly.start()
        self.assertTrue(assembly.ready())
        assembly.stop()
        self.assertFalse(assembly.started)
        self.assertTrue(all(r.is_stopped() for r in assembly.runners.values()))
        order = [row['id'] for row in assembly.stop_record]
        self.assertEqual(order[-1], 'mux')
        self.assertEqual(order[0], 'local')

    def test_start_failure_cleans_previous(self):
        plan, catalog = plan_for(G2D)

        def bad(instance, _component):
            if instance['id'] == 'local':
                return ['/nonexistent/f3-runner'], None
            return [PY, '-c', SLEEPER], 'READY'

        assembly = runtime.Assembly(plan, catalog, command_factory=bad,
                                    ready_timeout=2)
        with self.assertRaises(orch.OrchestratorError):
            assembly.start()
        self.assertFalse(assembly.started)
        self.assertTrue(assembly.runners)
        self.assertTrue(all(r.is_stopped() for r in assembly.runners.values()))

    def test_health_detects_death_and_stops(self):
        plan, catalog = plan_for(G2D)
        assembly = runtime.Assembly(plan, catalog, command_factory=fake,
                                    ready_timeout=5)
        assembly.start()
        assembly.runners['global'].proc.terminate()
        assembly.runners['global'].reap()
        dead = assembly.health()
        self.assertEqual(dead, ['global'])
        assembly.stop()
        self.assertTrue(all(r.is_stopped() for r in assembly.runners.values()))

    def test_default_commands_for_node_and_include(self):
        plan, catalog = plan_for(G2D)
        by_id = {item['id']: item for item in plan['instances']}
        lio = by_id['lio']
        command, marker = runtime.default_command(
            lio, catalog['components'][lio['component']])
        self.assertEqual(command[0], 'roslaunch')
        self.assertTrue(any(item.endswith('launch/10_slam_mapping.launch')
                            for item in command), command)
        self.assertIsNone(marker)
        map2d = by_id['map']
        command, marker = runtime.default_command(
            map2d, catalog['components'][map2d['component']])
        self.assertEqual(command, ['rosrun', 'map_server', 'map_server'])
        mux = copy.deepcopy(by_id['mux'])
        mux['namespace'] = '/nav'
        command, _ = runtime.default_command(
            mux, catalog['components'][mux['component']])
        self.assertEqual((command[0], Path(command[1]).name),
                         ('python3', 'param_exec.py'))
        payload = json.loads(Path(command[2]).read_text())
        self.assertEqual(payload['namespace'], '/nav')
        self.assertEqual(payload['param_files'],
                         ['lite3_safety:config/cmd_vel_mux.yaml'])
        self.assertEqual(payload['params'], {'default_priority': 'navigation'})
        self.assertEqual(payload['argv'][:3],
                         ['rosrun', 'lite3_safety', 'safe_cmd_vel_mux.py'])
        self.assertIn('__name:=mux', payload['argv'])
        self.assertIn('__ns:=/nav', payload['argv'])
        Path(command[2]).unlink()

    def test_node_config_is_loaded_through_param_exec(self):
        plan, catalog = plan_for(G_LOCAL_NATIVE)
        local = plan['instances'][0]
        command, marker = runtime.default_command(
            local, catalog['components'][local['component']])
        self.assertEqual((command[0], Path(command[1]).name),
                         ('python3', 'param_exec.py'))
        self.assertIsNone(marker)
        payload = json.loads(Path(command[2]).read_text())
        self.assertEqual(payload['config_path'], local['config'])
        self.assertEqual((payload['name'], payload['namespace']), ('local', None))
        self.assertEqual(payload['argv'][:3],
                         ['rosrun', 'v8_platform', 'local_planner/ros_node.py'])
        self.assertIn('__name:=local', payload['argv'])
        Path(command[2]).unlink()
        plugin_plan, plugin_catalog = plan_for(GPLUGIN)
        plus = copy.deepcopy(plugin_plan['instances'][0])
        plus['config'] = local['config']
        command, _ = runtime.default_command(
            plus, plugin_catalog['components'][plus['component']])
        payload = json.loads(Path(command[2]).read_text())
        self.assertEqual(payload['argv'][:3], ['rosrun', 'nodelet', 'nodelet'])
        self.assertIn('standalone', payload['argv'])
        self.assertTrue(payload['config_path'].endswith('configs/local_native.yaml'))
        Path(command[2]).unlink()

    def test_boundary_graph_passes_twist_and_udp_params_to_real_node(self):
        plan, catalog = plan_for(BOUNDARY)
        boundary = next(item for item in plan['instances'] if item['id'] == 'boundary')
        command, marker = runtime.default_command(
            boundary, catalog['components'][boundary['component']])
        self.assertEqual(command[:3], ['rosrun', 'v8_platform', 'boundary/ros_bridge.py'])
        self.assertIn('_command_type:=twist', command)
        self.assertIn('_target:=127.0.0.1:5026', command)
        self.assertIn('_bind:=127.0.0.1:5025', command)
        self.assertIsNone(marker)

    def test_plugin_default_command_and_assembly_without_command_owner(self):
        plan, catalog = plan_for(GPLUGIN)
        plus = plan['instances'][0]
        command, marker = runtime.default_command(
            plus, catalog['components'][plus['component']])
        self.assertEqual(command, ['rosrun', 'nodelet', 'nodelet', 'standalone',
                                   'nodelet_tutorial_math/Plus', '__name:=plus',
                                   '__ns:=/v8_plugin_smoke', '_value:=2.5'])
        self.assertIsNone(marker)
        assembly = runtime.Assembly(plan, catalog, command_factory=fake,
                                    ready_timeout=5)
        assembly.start()
        self.assertTrue(assembly.ready())
        self.assertIsNone(runtime.safety.check_plan(plan, catalog)['command_owner'])
        assembly.stop()
        self.assertTrue(all(r.is_stopped() for r in assembly.runners.values()))

    def test_launch_remap_is_applied_to_node_and_plugin_commands(self):
        node = {'launch': {'kind': 'node', 'package': 'p', 'type': 't',
                           'remap': [{'from': 'in', 'to': '/abs/in'}]}}
        command, _ = runtime.default_command(
            {'id': 'n', 'params': {'k': 1}, 'namespace': None}, node)
        self.assertEqual(command, ['rosrun', 'p', 't', '_k:=1', 'in:=/abs/in'])
        plugin = {'launch': {'kind': 'plugin', 'package': 'nodelet',
                             'plugin': 'pkg/Class',
                             'remap': [{'from': 'out', 'to': '/abs/out'}]}}
        command, _ = runtime.default_command(
            {'id': 'n', 'params': {}, 'namespace': None}, plugin)
        self.assertEqual(command, ['rosrun', 'nodelet', 'nodelet', 'standalone',
                                   'pkg/Class', '__name:=n', 'out:=/abs/out'])

    def test_semantic_provider_assembles_without_command_owner(self):
        plan, catalog = plan_for(GSEMANTIC)
        sem = plan['instances'][0]
        command, marker = runtime.default_command(
            sem, catalog['components'][sem['component']])
        self.assertEqual(command, ['rosrun', 'v8_platform',
                                   'representation/semantic_map_node.py',
                                   '_keep_alive_s:=30',
                                   '_semantic_files:=framework/fixtures/maps/sem_a.json'])
        self.assertIsNone(marker)
        assembly = runtime.Assembly(plan, catalog, command_factory=fake,
                                    ready_timeout=5)
        assembly.start()
        self.assertTrue(assembly.ready())
        assembly.stop()
        self.assertTrue(all(r.is_stopped() for r in assembly.runners.values()))

    def test_nodelet_group_command_renders_manager_and_loaders(self):
        plan, catalog = plan_for(GGROUP)
        group = plan['instances'][0]
        command, marker = runtime.default_command(
            group, catalog['components'][group['component']])
        self.assertEqual(command[0], 'roslaunch')
        text = Path(command[1]).read_text()
        self.assertIn('name="v8_group_mgr" args="manager"', text)
        self.assertIn('load nodelet_tutorial_math/Plus v8_group_mgr --no-bond', text)
        self.assertIn('<param name="value" value="2.5"/>', text)
        self.assertIsNone(marker)
        Path(command[1]).unlink()

    def test_map_cache_ros_command_carries_absolute_topics(self):
        plan, catalog = plan_for(GMAPCACHE)
        cache = next(item for item in plan['instances'] if item['id'] == 'cache_a')
        command, _ = runtime.default_command(
            cache, catalog['components'][cache['component']])
        self.assertEqual(command[:3], ['rosrun', 'v8_platform',
                                       'representation/map_cache_node.py'])
        self.assertIn('__ns:=/v8/cache_a', command)
        self.assertIn('_base_map_topic:=/v8/cache_a/base_map', command)
        self.assertIn('_delta_topic:=/v8/cache_a/delta', command)

    def test_nav2d_graph_passes_planner_args_to_real_include(self):
        plan, catalog = plan_for(G2D_STACK)
        nav = next(item for item in plan['instances'] if item['id'] == 'nav')
        command, marker = runtime.default_command(
            nav, catalog['components'][nav['component']])
        self.assertEqual(command[0], 'roslaunch')
        self.assertTrue(any(item.endswith('flat2d_stack_sc09_f64nli25_teb.launch')
                            for item in command), command)
        self.assertIn('global_planner:=global_planner/GlobalPlanner', command)
        self.assertIn('local_planner:=teb_local_planner/TebLocalPlannerROS', command)
        self.assertIn('cmd_topic:=/cmd_vel_raw', command)
        self.assertIn('map_yaml:=/tmp/v8_graph_map.yaml', command)
        self.assertIsNone(marker)

    def test_include_namespace_is_rejected(self):
        plan, catalog = plan_for(G2D)
        lio = copy.deepcopy(next(item for item in plan['instances'] if item['id'] == 'lio'))
        lio['namespace'] = '/nav'
        with self.assertRaises(config.FrameworkError) as caught:
            runtime.default_command(lio, catalog['components'][lio['component']])
        self.assertEqual(caught.exception.code, 'E_RUNTIME_NAMESPACE')


if __name__ == '__main__':
    unittest.main()
