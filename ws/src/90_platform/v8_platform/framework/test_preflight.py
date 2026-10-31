"""Preflight: per-instance config files and 2D/3D compatibility checks."""
import copy
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402

MANIFESTS = HERE / 'fixtures' / 'manifests'
G2D = HERE / 'fixtures' / 'graphs' / 'graph_2d.yaml'
G2D_CONFIG = HERE / 'fixtures' / 'graphs' / 'graph_2d_config.yaml'
G_LOCAL_NATIVE = HERE / 'fixtures' / 'graphs' / 'graph_local_native.yaml'
G_SEMANTIC_PLAN = HERE / 'fixtures' / 'graphs' / 'graph_semantic_planning.yaml'
G_SEMANTIC_OBSTACLE = HERE / 'fixtures' / 'graphs' / 'graph_semantic_obstacle_planning.yaml'
G_POSE_MUX = HERE / 'fixtures' / 'graphs' / 'graph_pose_mux.yaml'
G_COMMAND_OVERLAP = HERE / 'fixtures' / 'graphs' / 'graph_command_overlap.yaml'


def setup():
    return config.load_components(MANIFESTS), config.load_graph(G2D)


class PreflightTests(unittest.TestCase):
    def test_config_file_is_checked_and_rendered(self):
        catalog, graph = setup()
        with_config = config.load_graph(G2D_CONFIG)
        report = graph_mod.check(with_config, catalog)
        self.assertIn('local', report['configs'])
        self.assertEqual(len(report['configs']['local']['sha256']), 64)
        plan = render.render_plan(with_config, catalog, report)
        local = next(item for item in plan['instances'] if item['id'] == 'local')
        self.assertTrue(local['config'].endswith('configs/local2d.yaml'))
        self.assertEqual(local['config_sha256'], report['configs']['local']['sha256'])
        launch = render.render_launch(with_config, catalog, report)
        self.assertIn('rosparam command="load"', launch)
        self.assertIn('configs/local2d.yaml', launch)
        self.assertNotEqual(report['digest'], graph_mod.check(graph, catalog)['digest'])

    def test_missing_bad_duplicate_and_non_mapping_configs_are_rejected(self):
        catalog, graph = setup()
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            source = G2D.read_text(encoding='utf-8')
            for name, content, code in (
                    ('missing.yaml', None, 'E_CFG_MISSING'),
                    ('bad.yaml', 'a: [\n', 'E_CFG_YAML'),
                    ('dup.yaml', 'a: 1\na: 2\n', 'E_CFG_DUP_KEY'),
                    ('list.yaml', '- 1\n', 'E_CFG_TYPE')):
                path = base / name
                if content is not None:
                    path.write_text(content, encoding='utf-8')
                text = source.replace(
                    '{id: local, component: v8.fw.local2d}',
                    '{id: local, component: v8.fw.local2d, config: %s}' % name)
                graph_path = base / 'graph.yaml'
                graph_path.write_text(text, encoding='utf-8')
                with self.assertRaises(config.FrameworkError) as caught:
                    graph_mod.check(config.load_graph(graph_path), catalog)
                self.assertEqual(caught.exception.code, code)

    def test_config_on_include_is_rejected(self):
        catalog, graph = setup()
        graph = copy.deepcopy(graph)
        graph['instances']['lio']['config'] = 'x.yaml'
        with self.assertRaises(config.FrameworkError) as caught:
            graph_mod.check(graph, catalog)
        self.assertEqual(caught.exception.code, 'E_CFG_CONFIG_KIND')

    def test_2d_local_with_3d_global_is_rejected(self):
        component = {'roles': ['global_planner'], 'launch': {'kind': 'node'},
                     'inputs': [], 'outputs': [{'name': 'path', 'type': 'nav_msgs/Path',
                                                'direction': 'output'}],
                     'dimensions': 3}
        local = {'roles': ['local_planner'], 'launch': {'kind': 'node'},
                 'inputs': [{'name': 'path', 'type': 'nav_msgs/Path',
                             'direction': 'input', 'required': True}],
                 'outputs': [], 'dimensions': 2}
        catalog = {'components': {'g3': component, 'l2': local}, 'sha256': 'x', 'path': 'x'}
        graph = {'name': 'x', 'path': 'x',
                 'instances': {'g': {'id': 'g', 'component': 'g3', 'params': {}, 'namespace': None},
                               'l': {'id': 'l', 'component': 'l2', 'params': {}, 'namespace': None}},
                 'edges': [{'from': 'g.path', 'to': 'l.path', 'feedback': False, 'policy': {}}],
                 'external': []}
        with self.assertRaises(config.FrameworkError) as caught:
            graph_mod.check(graph, catalog)
        self.assertEqual(caught.exception.code, 'E_GRAPH_DIMENSION')

    def test_map_schema_must_match_component_dimensions(self):
        component = {'roles': ['local_planner'], 'launch': {'kind': 'node'},
                     'inputs': [{'name': 'map', 'type': 'v8_msgs/SupportMap',
                                 'schema': 'support_map/v1', 'direction': 'input',
                                 'required': True}],
                     'outputs': [], 'dimensions': 2}
        catalog = {'components': {'bad': component}, 'sha256': 'x', 'path': 'x'}
        graph = {'name': 'x', 'path': 'x',
                 'instances': {'l': {'id': 'l', 'component': 'bad', 'params': {},
                                     'namespace': None}},
                 'edges': [], 'external': [{'to': 'l.map', 'type': 'v8_msgs/SupportMap',
                                            'source': 'operator'}]}
        with self.assertRaises(config.FrameworkError) as caught:
            graph_mod.check(graph, catalog)
        self.assertEqual(caught.exception.code, 'E_GRAPH_MAP_DIMENSION')

    def test_native_planner_graph_binds_config_and_command(self):
        catalog = config.load_components(MANIFESTS)
        graph = config.load_graph(G_LOCAL_NATIVE)
        report = graph_mod.check(graph, catalog)
        self.assertEqual(report['startup_order'], ['local'])
        self.assertEqual(report['command_writers'], {'/v8/local/cmd_vel_3d': 'local'})
        config_row = report['configs']['local']
        self.assertIn('experimental_config', config_row['keys'])
        self.assertIn('wall_watchdog_s', config_row['keys'])
        self.assertEqual(len(config_row['sha256']), 64)
        root = ET.fromstring(render.render_launch(graph, catalog, report))
        node = root.find('node')
        self.assertEqual((node.get('pkg'), node.get('type'), node.get('name')),
                         ('v8_platform', 'local_planner/ros_node.py', 'local'))
        loads = [item.get('file') for item in node.findall('rosparam')]
        self.assertTrue(any(item.endswith('configs/local_native.yaml') for item in loads))
        self.assertEqual(report['bindings']['local']['map']['schema'], 'support_map/v1')

    def test_semantic_planning_graph_wires_the_bridge(self):
        catalog = config.load_components(MANIFESTS)
        graph = config.load_graph(G_SEMANTIC_PLAN)
        report = graph_mod.check(graph, catalog)
        self.assertEqual(report['startup_order'], ['semantic', 'bridge', 'local'])
        self.assertEqual(report['command_writers'], {'/v8/local/cmd_vel_3d': 'local'})
        self.assertEqual(report['bindings']['bridge']['semantic_map']['from'], 'semantic.map')
        self.assertIn('map_topic', report['configs']['local']['keys'])
        root = ET.fromstring(render.render_launch(graph, catalog, report))
        types = sorted(node.get('type') for node in root.findall('node'))
        self.assertEqual(types, ['local_planner/ros_node.py',
                                 'representation/semantic_map_node.py',
                                 'representation/semantic_support_bridge_node.py'])

    def test_semantic_obstacle_graph_binds_both_consumers(self):
        catalog = config.load_components(MANIFESTS)
        graph = config.load_graph(G_SEMANTIC_OBSTACLE)
        report = graph_mod.check(graph, catalog)
        self.assertEqual(report['startup_order'], ['semantic', 'obstacles', 'support', 'local'])
        self.assertEqual(report['bindings']['local']['obstacles']['from'], 'obstacles.obstacles')
        self.assertEqual(report['bindings']['support']['semantic_map']['from'], 'semantic.map')
        root = ET.fromstring(render.render_launch(graph, catalog, report))
        types = sorted(node.get('type') for node in root.findall('node'))
        self.assertIn('representation/semantic_obstacle_bridge_node.py', types)

    def test_pose_mux_graph_is_registrable(self):
        catalog = config.load_components(MANIFESTS)
        graph = config.load_graph(G_POSE_MUX)
        report = graph_mod.check(graph, catalog)
        self.assertEqual(report['startup_order'], ['mux'])
        self.assertEqual(report['command_writers'], {})
        root = ET.fromstring(render.render_launch(graph, catalog, report))
        node = root.find('node')
        self.assertEqual((node.get('pkg'), node.get('type')),
                         ('v8_platform', 'local_planner/pose_mux_node.py'))

    def test_command_overlap_graph_keeps_single_arbiter(self):
        catalog = config.load_components(MANIFESTS)
        graph = config.load_graph(G_COMMAND_OVERLAP)
        report = graph_mod.check(graph, catalog)
        self.assertEqual(report['startup_order'], ['mux'])
        self.assertEqual(report['command_writers'], {'/cmd_vel': 'mux'})
        plan = render.render_plan(graph, catalog, report)
        checked = __import__('safety').check_plan(plan, catalog)
        self.assertEqual(checked['command_owner'], 'mux')


if __name__ == '__main__':
    unittest.main()
