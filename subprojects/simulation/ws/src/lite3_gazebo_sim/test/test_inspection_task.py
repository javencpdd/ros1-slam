#!/usr/bin/env python3
"""Inspection task/app fixture: ordered goals and the BT mission contract."""
import json
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parents[1]
SCENARIO = HERE / 'scenarios' / 'two_floor_nav'
P2_ROOT = HERE.parents[2]


def load(rel):
    return json.loads((SCENARIO / rel).read_text(encoding='utf-8'))


class InspectionTaskTests(unittest.TestCase):
    def test_ordered_goals(self):
        task = load('tasks/inspection.json')
        goals = task['goals']
        self.assertEqual(len(goals), 3)
        labels = [goal.get('label') for goal in goals]
        self.assertEqual(len(labels), len(set(labels)))
        for goal in goals:
            self.assertEqual(len(goal['xyz']), 3)
            self.assertEqual(goal['surface_hint'], 'upper_floor')
            self.assertGreater(goal['dwell_s'], 0)
            self.assertGreater(goal['arrival_timeout_s'], 0)
            self.assertGreaterEqual(goal['retry_max'], 1)
        self.assertEqual(task['map'], {'map_id': 'two_floor_nav', 'map_version': 1})

    def test_app_and_graph_contract(self):
        app = load('apps/inspection_bt.yaml')
        self.assertEqual(app['schema'], 'navigation_app/v1')
        self.assertEqual(app['task'], 'sim_ws/src/lite3_gazebo_sim/scenarios/two_floor_nav/tasks/inspection.json')
        self.assertEqual(app['mission_topic'], '/v8/bt/goal_mission')
        for key in ('graph', 'map_registry', 'task'):
            host = P2_ROOT / app[key].replace('sim_ws/src/', 'ws/src/')
            self.assertTrue(host.is_file(), (key, host))
        graph = load('graphs/navigation_bt.yaml')
        bt = next(item for item in graph['instances'] if item['id'] == 'bt')
        self.assertEqual(bt['params']['mission_topic'], '/v8/bt/goal_mission')
        self.assertEqual(bt['params']['cancel_input_topic'], '/v8/bt/cancel')
        edges = {tuple(edge) for edge in (
            (item['from'], item['to']) for item in graph['edges'])}
        self.assertIn(('bt.cancel_cmd', 'local.cancel'), edges)
        self.assertIn(('local.cancel_ack', 'bt.cancel_ack'), edges)
        external = {item['to'] for item in graph['external']}
        self.assertIn('bt.mission_goal', external)
        self.assertIn('bt.cancel', external)


if __name__ == '__main__':
    unittest.main()
