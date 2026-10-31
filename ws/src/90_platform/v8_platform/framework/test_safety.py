"""F7: single /cmd_vel owner, advice into arbiter, validator placement, perf."""
import copy
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import perf  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402
import safety  # noqa: E402

MANIFESTS = HERE / 'fixtures' / 'manifests'
G2D = HERE / 'fixtures' / 'graphs' / 'graph_2d.yaml'
PY = sys.executable
SLEEPER = "import time; print('READY', flush=True); time.sleep(3600)"


def fake(_instance, _component):
    return [PY, '-c', SLEEPER], 'READY'


def setup():
    catalog = config.load_components(MANIFESTS)
    graph = config.load_graph(G2D)
    report = graph_mod.check(graph, catalog)
    return graph, catalog, render.render_plan(graph, catalog, report)


def add_validator(catalog, plan):
    catalog = copy.deepcopy(catalog)
    catalog['components']['v8.fw.validator_test'] = {
        'id': 'v8.fw.validator_test',
        'launch': {'kind': 'node', 'package': 'v8_platform', 'type': 'validator_test',
                   'test_double': True},
        'roles': ['trajectory_validator'],
        'inputs': [{'name': 'path', 'type': 'nav_msgs/Path', 'direction': 'input',
                    'required': True}],
        'outputs': [{'name': 'path', 'type': 'nav_msgs/Path', 'direction': 'output'}],
        'stop': 'process_group_signal', 'resources': []}
    plan = copy.deepcopy(plan)
    plan['instances'].append({'id': 'validator', 'component': 'v8.fw.validator_test'})
    plan['edges'] = [edge for edge in plan['edges']
                     if not (edge['from'] == 'global.path' and edge['to'] == 'local.path')]
    plan['edges'] += [{'from': 'global.path', 'to': 'validator.path', 'feedback': False,
                       'policy': {}},
                      {'from': 'validator.path', 'to': 'local.path', 'feedback': False,
                       'policy': {}}]
    return catalog, plan


class SafetyTests(unittest.TestCase):
    def test_valid_plan_has_single_owner_and_advice(self):
        _, catalog, plan = setup()
        result = safety.check_plan(plan, catalog)
        self.assertEqual(result['command_owner'], 'mux')
        self.assertEqual(result['validator'], 'absent')
        self.assertEqual([item['endpoint'] for item in result['advice']], ['local.cmd'])

    def test_owner_without_arbiter_role_is_rejected(self):
        _, catalog, plan = setup()
        catalog = copy.deepcopy(catalog)
        catalog['components']['v8.fw.mux']['roles'] = []
        with self.assertRaises(config.FrameworkError) as caught:
            safety.check_plan(plan, catalog)
        self.assertEqual(caught.exception.code, 'E_SAFETY_ARBITER')

    def test_advice_must_reach_the_arbiter(self):
        _, catalog, plan = setup()
        plan = copy.deepcopy(plan)
        plan['edges'] = [edge for edge in plan['edges']
                         if not (edge['from'] == 'local.cmd' and edge['to'] == 'mux.cmd')]
        with self.assertRaises(config.FrameworkError) as caught:
            safety.check_plan(plan, catalog)
        self.assertEqual(caught.exception.code, 'E_SAFETY_ADVICE')

    def test_validator_must_sit_on_the_path(self):
        _, catalog, plan = setup()
        catalog, plan = add_validator(catalog, plan)
        result = safety.check_plan(plan, catalog)
        self.assertEqual(result['validator'], 'present')
        broken = copy.deepcopy(plan)
        broken['edges'] = [edge for edge in broken['edges']
                           if not (edge['from'] == 'validator.path'
                                   and edge['to'] == 'local.path')]
        with self.assertRaises(config.FrameworkError) as caught:
            safety.check_plan(broken, catalog)
        self.assertEqual(caught.exception.code, 'E_SAFETY_VALIDATOR')

    def test_plan_without_command_topics_has_no_owner(self):
        plan = {'instances': [{'id': 'readonly', 'component': 'probe'}], 'edges': []}
        catalog = {'components': {'probe': {
            'roles': ['middleware'],
            'inputs': [],
            'outputs': [{'name': 'out', 'type': 'std_msgs/Float64'}]}}}
        self.assertIsNone(safety.check_plan(plan, catalog)['command_owner'])
        catalog['components']['probe']['outputs'][0]['command_topic'] = '/cmd_vel_advice/x'
        with self.assertRaises(config.FrameworkError) as caught:
            safety.check_plan(plan, catalog)
        self.assertEqual(caught.exception.code, 'E_SAFETY_ADVICE')

    def test_startup_benchmark_is_reported(self):
        graph, catalog, _ = setup()
        result = perf.benchmark(graph, catalog, iterations=10)
        self.assertEqual(result['iterations'], 10)
        self.assertTrue(result['startup_only'])
        self.assertGreaterEqual(result['p95_ms'], result['p50_ms'])
        self.assertIn('500 Hz', result['note'])

    def test_assembly_refuses_second_cmd_vel_writer(self):
        _, catalog, plan = setup()
        plan = copy.deepcopy(plan)
        plan['instances'].append({'id': 'mux2', 'component': 'v8.fw.mux'})
        assembly = runtime.Assembly(plan, catalog, command_factory=fake)
        with self.assertRaises(config.FrameworkError) as caught:
            assembly.start()
        self.assertEqual(caught.exception.code, 'E_SAFETY_ARBITER')
        self.assertFalse(assembly.started)
        self.assertFalse(assembly.runners)

    def test_validator_reject_stops_running_assembly(self):
        _, catalog, plan = setup()
        assembly = runtime.Assembly(plan, catalog, command_factory=fake,
                                    ready_timeout=10)
        assembly.start()
        self.assertTrue(assembly.ready())
        verdict = safety.enforce(assembly, {'approved': False, 'reason': 'collision'})
        self.assertTrue(verdict['stopped'])
        self.assertEqual(verdict['reason'], 'collision')
        self.assertTrue(all(r.is_stopped() for r in assembly.runners.values()))
        self.assertEqual(assembly.stop_record[-1]['id'], 'mux')
        approved = safety.enforce(assembly, {'approved': True})
        self.assertEqual(approved, {'ok': True, 'stopped': False})
        with self.assertRaises(config.FrameworkError) as caught:
            safety.enforce(assembly, {})
        self.assertEqual(caught.exception.code, 'E_SAFETY_VERDICT')


if __name__ == '__main__':
    unittest.main()
