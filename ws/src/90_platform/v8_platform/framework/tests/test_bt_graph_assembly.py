#!/usr/bin/env python3
"""BT-orchestrated 3D graph: bt_console is a first-class component.

The tree owns WHEN; the global planner still computes the path.  These checks
pin the contract wiring and prove the failure modes the graph must reject.
"""
import sys as _sys
from pathlib import Path as _Path

_BASE = _Path(__file__).resolve().parent.parent
for _entry in ('', 'nodes', 'tools', 'smokes'):
    _sys.path.insert(0, str(_BASE / _entry) if _entry else str(_BASE))
import copy
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
FRAMEWORK = HERE.parent
sys.path.insert(0, str(FRAMEWORK))
sys.path.insert(0, str(FRAMEWORK.parent / 'contracts'))
sys.path.insert(0, str(FRAMEWORK.parent / 'orchestrator'))
import config  # noqa: E402
import graph  # noqa: E402
import render  # noqa: E402
import safety  # noqa: E402


class BtGraphAssemblyTests(unittest.TestCase):
    def setUp(self):
        self.catalog = config.load_components(FRAMEWORK / 'fixtures' / 'manifests')
        self.value = config.load_graph(FRAMEWORK / 'fixtures' / 'graphs' / 'graph_3d_gazebo_bt.yaml')
        self.report = graph.check(self.value, self.catalog)
        self.plan = render.render_plan(self.value, self.catalog, self.report)

    def test_bt_component_is_registered_and_wired(self):
        self.assertIn('bt', self.report['instances'])
        self.assertEqual(self.report['instances']['bt']['component'], 'v8.fw.bt_console')
        edges = {(edge['from'], edge['to']) for edge in self.report['edges']}
        self.assertIn(('global.path', 'bt.path'), edges)
        self.assertIn(('global.status', 'bt.path_status'), edges)
        self.assertIn(('bt.out_goal', 'local.goal'), edges)
        self.assertIn(('bt.planner_goal', 'global.goal'), edges)
        self.assertNotIn('path_goal', self.report['instances'])

    def test_startup_dag_orders_bt_between_global_and_local(self):
        order = self.report['startup_order']
        self.assertLess(order.index('global'), order.index('bt'))
        self.assertLess(order.index('bt'), order.index('local'))

    def test_request_path_loop_is_feedback_with_policy(self):
        by_pair = {(edge['from'], edge['to']): edge for edge in self.report['edges']}
        loop = by_pair[('bt.planner_goal', 'global.goal')]
        self.assertTrue(loop['feedback'])
        self.assertTrue(loop['policy'])

    def test_single_command_arbiter_unchanged(self):
        self.assertEqual(self.report['command_writers']['/cmd_vel'], 'mux3d')
        verdict = safety.check_plan(self.plan, self.catalog)
        self.assertTrue(verdict['ok'])
        self.assertEqual(verdict['command_owner'], 'mux3d')

    def _check(self, mutate):
        value = copy.deepcopy(self.value)
        mutate(value)
        return graph.check(value, self.catalog)

    def test_unbound_bt_goal_is_rejected(self):
        def mutate(value):
            value['external'] = [e for e in value['external'] if e['to'] != 'bt.goal']
        with self.assertRaises(config.FrameworkError) as caught:
            self._check(mutate)
        self.assertEqual(caught.exception.code, 'E_GRAPH_INPUT')

    def test_wrong_type_on_bt_path_is_rejected(self):
        def mutate(value):
            for edge in value['edges']:
                if edge['to'] == 'bt.path':
                    edge['from'] = 'global.status'
        with self.assertRaises(config.FrameworkError) as caught:
            self._check(mutate)
        self.assertEqual(caught.exception.code, 'E_GRAPH_TYPE')

    def test_double_goal_writer_is_rejected(self):
        def mutate(value):
            value['instances']['path_goal'] = {
                'component': 'v8.fw.path_goal3d', 'namespace': '/v8/3d/path_goal',
                'params': {'path_topic': '/v8/3d/global/path', 'goal_topic': '/v8/local/goal'}}
            value['edges'].append({'from': 'global.path', 'to': 'path_goal.path',
                                   'feedback': False, 'policy': {}})
            value['edges'].append({'from': 'path_goal.goal', 'to': 'local.goal',
                                   'feedback': False, 'policy': {}})
        with self.assertRaises(config.FrameworkError) as caught:
            self._check(mutate)
        self.assertEqual(caught.exception.code, 'E_GRAPH_INPUT_DUP')


if __name__ == '__main__':
    unittest.main()
