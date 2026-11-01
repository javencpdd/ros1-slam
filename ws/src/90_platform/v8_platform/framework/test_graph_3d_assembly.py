#!/usr/bin/env python3
"""Preflight test for the 3D Assembly graph and command arbiter ownership."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
sys.path.insert(0, str(HERE.parent / 'orchestrator'))
import config  # noqa: E402
import graph  # noqa: E402
import render  # noqa: E402
import safety  # noqa: E402


class Graph3DAssemblyTests(unittest.TestCase):
    def setUp(self):
        self.catalog = config.load_components(HERE / 'fixtures' / 'manifests')
        self.value = config.load_graph(HERE / 'fixtures' / 'graphs' / 'graph_3d_real.yaml')
        self.report = graph.check(self.value, self.catalog)
        self.plan = render.render_plan(self.value, self.catalog, self.report)

    def test_single_3d_command_arbiter_and_feedback_edges(self):
        self.assertEqual(self.report['command_writers']['/cmd_vel'], 'mux3d')
        self.assertEqual(self.report['command_writers']['/v8/local/cmd_vel_3d'], 'local')
        feedback = [edge for edge in self.report['edges'] if edge['feedback']]
        self.assertEqual(len(feedback), 2)
        self.assertTrue(all(edge['policy'] for edge in feedback))

    def test_safety_gate_accepts_assembly(self):
        verdict = safety.check_plan(self.plan, self.catalog)
        self.assertTrue(verdict['ok'])
        self.assertEqual(verdict['command_owner'], 'mux3d')
        self.assertEqual(verdict['advice'], [{'endpoint': 'local.cmd',
                                              'topic': '/v8/local/cmd_vel_3d'}])

    def test_path_goal_and_carrier_are_on_the_loop(self):
        edges = {(edge['from'], edge['to']) for edge in self.report['edges']}
        self.assertIn(('global.path', 'path_goal.path'), edges)
        self.assertIn(('path_goal.goal', 'local.goal'), edges)
        self.assertIn(('carrier.pose', 'local.pose'), edges)


if __name__ == '__main__':
    unittest.main()
