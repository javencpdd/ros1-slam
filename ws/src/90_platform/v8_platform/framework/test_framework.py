"""Framework v0 tests: YAML graph, multi-instance ports, startup/feedback, render."""
import copy
import io
import json
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from contextlib import redirect_stdout
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cli  # noqa: E402
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import launch as launch_mod  # noqa: E402
import render  # noqa: E402

MANIFESTS = HERE / 'fixtures' / 'manifests'
V1_MANIFESTS = HERE.parent / 'registry' / 'data' / 'manifests'
G2D = HERE / 'fixtures' / 'graphs' / 'graph_2d.yaml'
G3D = HERE / 'fixtures' / 'graphs' / 'graph_3d_cache.yaml'
MISSING = HERE / 'fixtures' / 'graphs' / 'invalid_missing_input.yaml'
DUP_KEY = HERE / 'fixtures' / 'graphs' / 'invalid_dup_key.yaml'
INDEX = HERE / 'fixtures' / 'package_index.json'


def load_pair(path):
    return config.load_components(MANIFESTS), config.load_graph(path)


def find_edge(value, source, target):
    for index, edge in enumerate(value['edges']):
        if edge['from'] == source and edge['to'] == target:
            return index
    raise AssertionError('edge not found: %s -> %s' % (source, target))


class FrameworkTests(unittest.TestCase):
    def test_2d_startup_and_command_owner(self):
        catalog, value = load_pair(G2D)
        report = graph_mod.check(value, catalog)
        self.assertTrue(report['ok'])
        order = report['startup_order']
        self.assertLess(order.index('sensors'), order.index('lio'))
        self.assertLess(order.index('lio'), order.index('global'))
        self.assertLess(order.index('map'), order.index('global'))
        self.assertLess(order.index('global'), order.index('local'))
        self.assertLess(order.index('local'), order.index('mux'))
        self.assertLess(order.index('mux'), order.index('robot'))
        self.assertEqual(report['command_writers'],
                         {'/cmd_vel': 'mux', '/cmd_vel_advice/local': 'local'})

    def test_same_component_two_instances_and_feedback(self):
        catalog, value = load_pair(G3D)
        report = graph_mod.check(value, catalog)
        self.assertIn('cache_a', report['instances'])
        self.assertIn('cache_b', report['instances'])
        self.assertEqual(len(report['feedback']), 1)
        self.assertEqual(report['feedback'][0]['policy']['period_s'], 5.0)
        order = report['startup_order']
        self.assertLess(order.index('map'), order.index('cache_a'))
        self.assertLess(order.index('cache_a'), order.index('merge'))
        self.assertEqual(report['command_writers'],
                         {'/cmd_vel': 'mux', '/cmd_vel_advice/3d': 'local'})

    def test_digest_is_stable_and_param_sensitive(self):
        catalog, first = load_pair(G2D)
        _, second = load_pair(G2D)
        one = graph_mod.check(first, catalog)['digest']
        two = graph_mod.check(second, catalog)['digest']
        self.assertEqual(one, two)
        changed = copy.deepcopy(first)
        changed['instances']['mux']['params']['default_priority'] = 'task'
        self.assertNotEqual(one, graph_mod.check(changed, catalog)['digest'])

    def test_missing_required_input_rejected(self):
        catalog, value = load_pair(G2D)
        value['external'] = [item for item in value['external']
                             if item['to'] != 'global.goal']
        with self.assertRaises(config.FrameworkError) as caught:
            graph_mod.check(value, catalog)
        self.assertEqual(caught.exception.code, 'E_GRAPH_INPUT')

    def test_port_type_mismatch_rejected(self):
        catalog, value = load_pair(G3D)
        del value['edges'][find_edge(value, 'global.path', 'local.path')]
        value['edges'].append({'from': 'map.map', 'to': 'local.path',
                               'feedback': False, 'policy': {}, 'index': 99})
        with self.assertRaises(config.FrameworkError) as caught:
            graph_mod.check(value, catalog)
        self.assertEqual(caught.exception.code, 'E_GRAPH_TYPE')

    def test_map_schema_mismatch_rejected(self):
        catalog, value = load_pair(G3D)
        output = catalog['components']['v8.fw.map3d']['outputs'][0]
        output['schema'] = 'support_map/v2'
        with self.assertRaises(config.FrameworkError) as caught:
            graph_mod.check(value, catalog)
        self.assertEqual(caught.exception.code, 'E_GRAPH_SCHEMA')

    def test_non_feedback_cycle_rejected(self):
        catalog, value = load_pair(G3D)
        value['edges'][find_edge(value, 'merge.delta', 'map.delta')]['feedback'] = False
        with self.assertRaises(config.FrameworkError) as caught:
            graph_mod.check(value, catalog)
        self.assertEqual(caught.exception.code, 'E_GRAPH_STARTUP_CYCLE')

    def test_feedback_needs_policy(self):
        catalog, value = load_pair(G3D)
        value['edges'][find_edge(value, 'merge.delta', 'map.delta')]['policy'] = {}
        with self.assertRaises(config.FrameworkError) as caught:
            graph_mod.check(value, catalog)
        self.assertEqual(caught.exception.code, 'E_GRAPH_FEEDBACK_POLICY')

    def test_duplicate_command_writer_rejected(self):
        catalog, value = load_pair(G3D)
        value['instances']['mux_b'] = {'id': 'mux_b', 'component': 'v8.fw.mux',
                                       'params': {}, 'namespace': None}
        value['edges'].append({'from': 'local.cmd', 'to': 'mux_b.cmd',
                               'feedback': False, 'policy': {}, 'index': 99})
        with self.assertRaises(config.FrameworkError) as caught:
            graph_mod.check(value, catalog)
        self.assertEqual(caught.exception.code, 'E_GRAPH_COMMAND')

    def test_duplicate_input_binding_rejected(self):
        catalog, value = load_pair(G2D)
        value['edges'].append({'from': 'global.path', 'to': 'local.path',
                               'feedback': False, 'policy': {}, 'index': 99})
        with self.assertRaises(config.FrameworkError) as caught:
            graph_mod.check(value, catalog)
        self.assertEqual(caught.exception.code, 'E_GRAPH_INPUT_DUP')

    def test_unknown_component_and_port_rejected(self):
        catalog, value = load_pair(G2D)
        value['instances']['local']['component'] = 'missing'
        with self.assertRaises(config.FrameworkError) as caught:
            graph_mod.check(value, catalog)
        self.assertEqual(caught.exception.code, 'E_GRAPH_COMPONENT')
        catalog, value = load_pair(G2D)
        value['edges'].append({'from': 'local.nope', 'to': 'mux.cmd',
                               'feedback': False, 'policy': {}, 'index': 99})
        with self.assertRaises(config.FrameworkError) as caught:
            graph_mod.check(value, catalog)
        self.assertEqual(caught.exception.code, 'E_GRAPH_PORT')

    def test_sanitized_node_name_collision_rejected(self):
        catalog, value = load_pair(G2D)
        value['instances']['map.a'] = {'id': 'map.a', 'component': 'v8.fw.map2d',
                                       'params': {}, 'namespace': None}
        value['instances']['map_a'] = {'id': 'map_a', 'component': 'v8.fw.map2d',
                                       'params': {}, 'namespace': None}
        with self.assertRaises(config.FrameworkError) as caught:
            graph_mod.check(value, catalog)
        self.assertEqual(caught.exception.code, 'E_GRAPH_NAME_COLLISION')

    def test_external_type_mismatch_rejected(self):
        catalog, value = load_pair(G2D)
        for item in value['external']:
            if item['to'] == 'global.goal':
                item['type'] = 'std_msgs/Bool'
        with self.assertRaises(config.FrameworkError) as caught:
            graph_mod.check(value, catalog)
        self.assertEqual(caught.exception.code, 'E_GRAPH_TYPE')

    def test_v1_manifest_rejected_for_graph(self):
        with self.assertRaises(config.FrameworkError) as caught:
            config.load_components(V1_MANIFESTS)
        self.assertEqual(caught.exception.code, 'E_CFG_MANIFEST_V1')

    def test_duplicate_yaml_key_rejected(self):
        with self.assertRaises(config.FrameworkError) as caught:
            config.load_yaml(DUP_KEY)
        self.assertEqual(caught.exception.code, 'E_CFG_DUP_KEY')

    def test_render_plan_and_launch(self):
        catalog, value = load_pair(G3D)
        report = graph_mod.check(value, catalog)
        plan = render.render_plan(value, catalog, report)
        ids = [instance['id'] for instance in plan['instances']]
        self.assertIn('cache_a', ids)
        self.assertIn('cache_b', ids)
        self.assertEqual(plan['feedback'][0]['policy']['period_s'], 5.0)
        self.assertEqual(plan['command_writers'],
                         {'/cmd_vel': 'mux', '/cmd_vel_advice/3d': 'local'})
        launch = render.render_launch(value, catalog, report)
        root = ET.fromstring(launch)
        names = [node.get('name') for node in root.findall('node')]
        self.assertIn('cache_a', names)
        self.assertIn('cache_b', names)
        self.assertEqual(root.find('arg').get('name'), 'graph_digest')
        remaps = [remap.get('from') for node in root.findall('node')
                  for remap in node.findall('remap')]
        self.assertIn('/cloud_registered', remaps)

    def test_launch_specs_and_index_counts(self):
        catalog, _ = load_pair(G3D)
        lio = catalog['components']['v8.fw.faster_lio']['launch']
        self.assertEqual((lio['kind'], lio['launch_file']),
                         ('include', 'launch/10_slam_mapping.launch'))
        mux = catalog['components']['v8.fw.mux']['launch']
        self.assertEqual(mux['param_files'], ['config/cmd_vel_mux.yaml'])
        summary = launch_mod.check_components(
            catalog, launch_mod.load_index(INDEX))
        self.assertEqual((summary['components'], summary['real'], summary['test_double']),
                         (13, 5, 8))

    def test_launch_spec_rejects_bad_entries(self):
        cases = [
            ({'package': 'p'}, 'E_CFG_LAUNCH'),
            ({'package': 'p', 'kind': 'nope'}, 'E_CFG_LAUNCH_KIND'),
            ({'package': 'p', 'kind': 'plugin', 'type': 't'}, 'E_CFG_LAUNCH_KIND'),
            ({'package': 'p', 'kind': 'include', 'launch_file': 'x'}, 'E_CFG_LAUNCH'),
            ({'package': 'p', 'type': 't', 'launch_file': 'x.launch'}, 'E_CFG_LAUNCH'),
            ({'package': 'p', 'type': 't', 'param_files': ['../x']}, 'E_CFG_LAUNCH_PATH'),
        ]
        for raw, code in cases:
            with self.assertRaises(config.FrameworkError) as caught:
                config._launch_spec(raw, 'probe')
            self.assertEqual(caught.exception.code, code, raw)

    def test_package_index_rejects_unknowns(self):
        catalog, _ = load_pair(G3D)
        index = launch_mod.load_index(INDEX)
        launch_mod.check_components(catalog, index)
        broken = copy.deepcopy(index)
        del broken['map_server']
        with self.assertRaises(config.FrameworkError) as caught:
            launch_mod.check_components(catalog, broken)
        self.assertEqual(caught.exception.code, 'E_LAUNCH_PACKAGE')
        broken = copy.deepcopy(index)
        broken['lite3_slam']['launches'] = []
        with self.assertRaises(config.FrameworkError) as caught:
            launch_mod.check_components(catalog, broken)
        self.assertEqual(caught.exception.code, 'E_LAUNCH_FILE')
        broken = copy.deepcopy(index)
        broken['lite3_drivers']['executables'] = []
        with self.assertRaises(config.FrameworkError) as caught:
            launch_mod.check_components(catalog, broken)
        self.assertEqual(caught.exception.code, 'E_LAUNCH_TYPE')
        broken = copy.deepcopy(index)
        broken['lite3_safety']['files'] = []
        with self.assertRaises(config.FrameworkError) as caught:
            launch_mod.check_components(catalog, broken)
        self.assertEqual(caught.exception.code, 'E_LAUNCH_FILE')

    def test_render_include_and_param_files(self):
        catalog, value = load_pair(G3D)
        report = graph_mod.check(value, catalog)
        root = ET.fromstring(render.render_launch(value, catalog, report))
        includes = [item.get('file') for item in root.findall('include')]
        self.assertIn('$(find lite3_slam)/launch/10_slam_mapping.launch', includes)
        mux = next(node for node in root.findall('node') if node.get('name') == 'mux')
        loads = [item.get('file') for item in mux.findall('rosparam')]
        self.assertIn('$(find lite3_safety)/config/cmd_vel_mux.yaml', loads)
        sensors = next(node for node in root.findall('node') if node.get('name') == 'sensors')
        self.assertEqual(sensors.get('pkg'), 'lite3_drivers')

    def test_cli_check_and_plan(self):
        json_out = io.StringIO()
        with redirect_stdout(json_out):
            self.assertEqual(cli.main(['check', str(MANIFESTS), str(G3D),
                                      '--package-index', str(INDEX)]), 0)
        payload = json.loads(json_out.getvalue())
        self.assertTrue(payload['ok'])
        self.assertEqual(payload['launch']['real'], 5)
        with tempfile.TemporaryDirectory() as directory:
            plan_path = Path(directory) / 'plan.json'
            launch_path = Path(directory) / 'graph.launch'
            with redirect_stdout(io.StringIO()):
                rc = cli.main(['plan', str(MANIFESTS), str(G3D),
                               '--package-index', str(INDEX),
                               '--out-plan', str(plan_path),
                               '--out-launch', str(launch_path)])
            self.assertEqual(rc, 0)
            self.assertTrue(plan_path.is_file())
            ET.fromstring(launch_path.read_text())

    def test_cli_error_rc(self):
        with redirect_stdout(io.StringIO()):
            self.assertEqual(cli.main(['check', str(MANIFESTS), str(MISSING)]), 5)


if __name__ == '__main__':
    unittest.main()
