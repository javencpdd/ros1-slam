#!/usr/bin/env python3
"""Mission task schema and BT wire-format controls (no ROS)."""
import sys
from pathlib import Path

import unittest

HERE = Path(__file__).resolve().parent
FRAMEWORK = HERE.parent
sys.path.insert(0, str(FRAMEWORK))

import mission  # noqa: E402
import surface_resolver  # noqa: E402


def flat_map():
    footprint = {'min_x': -20, 'max_x': 20, 'min_y': -20, 'max_y': 20}
    return {'surfaces': [
        {'surface_id': 'lower', 'z_reference': 0.0, 'traversability': 'supported',
         'footprint': footprint,
         'cells': {'runs': [[-5, -5, 10], [0, -5, 10], [5, -5, 10]]}},
        {'surface_id': 'upper', 'z_reference': 1.6, 'traversability': 'supported',
         'footprint': footprint,
         'cells': {'runs': [[-5, -5, 10], [0, -5, 10], [5, -5, 10]]}},
    ]}


class GoalWireTests(unittest.TestCase):
    def test_single_goal_wire(self):
        wire = mission.goal_wire({'xyz': [1.25, -2.5, 1.6], 'yaw': 1.57,
                                  'dwell_s': 1.5, 'arrival_timeout_s': 90,
                                  'retry_max': 2, 'label': 'up'}, 1, 1)
        self.assertEqual(wire, 'x=1.250000;y=-2.500000;z=1.600000;yaw=1.570000;'
                              'index=1;total=1;dwell=1.500;arrival=90.000;retry=2;label=up')

    def test_timeout_fallback_and_rejections(self):
        wire = mission.goal_wire({'xyz': [0, 0, 0], 'timeout_s': 45}, 2, 3)
        self.assertIn('index=2;total=3', wire)
        self.assertIn('arrival=45.000', wire)
        for goal in ({'xyz': [0, 0, None]}, {'xyz': [0, 0, 0], 'label': 'a;b'},
                     {'xyz': [0, 0, 0], 'retry_max': -1},
                     {'xyz': [0, 0, 0], 'label': 'x' * 81}):
            with self.assertRaises(ValueError):
                mission.goal_wire(goal, 1, 1)

    def test_goals_single_and_multi(self):
        self.assertEqual(mission.goals({'goal': {'xyz': [0, 0, 0]}}),
                         [{'xyz': [0, 0, 0]}])
        multi = {'goals': [{'xyz': [0, 0, 0]}, {'xyz': [1, 1, 1]}]}
        self.assertEqual(len(mission.goals(multi)), 2)
        with self.assertRaises(ValueError):
            mission.goals({'goals': []})


class TaskResolutionTests(unittest.TestCase):
    def test_multi_goal_resolves_order_and_first_legacy_goal(self):
        task = {
            'start': {'xyz': [0, 0, 0.0]},
            'goals': [
                {'xyz': [-5.0, 0.0, None], 'surface_hint': 'upper',
                 'surface_disambiguation': 'explicit_hint', 'label': 'a'},
                {'xyz': [-6.0, 1.0, 1.6], 'label': 'b'},
            ],
            'map': {'map_id': 'm', 'map_version': 1},
        }
        resolved, meta = surface_resolver.resolve_task(task, flat_map())
        self.assertEqual(len(resolved['goals']), 2)
        self.assertEqual(resolved['goal']['xyz'], resolved['goals'][0]['xyz'])
        self.assertEqual(resolved['goals'][0]['xyz'][2], 1.6)
        self.assertEqual(resolved['goals'][1]['xyz'][2], 1.6)
        self.assertEqual([item['index'] for item in meta['goals']], [1, 2])

    def test_goal_and_goals_disagree_refused(self):
        task = {'start': {'xyz': [0, 0, 0]},
                'goal': {'xyz': [0, 0, 0]},
                'goals': [{'xyz': [1, 1, 0]}]}
        with self.assertRaises(surface_resolver.SurfaceResolutionError):
            surface_resolver.resolve_task(task, flat_map())

    def test_task_needs_goal(self):
        with self.assertRaises(surface_resolver.SurfaceResolutionError):
            surface_resolver.resolve_task({'start': {'xyz': [0, 0, 0]}}, flat_map())


if __name__ == '__main__':
    unittest.main()
