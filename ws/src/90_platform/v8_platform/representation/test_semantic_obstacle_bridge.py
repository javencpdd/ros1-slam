"""Semantic obstacle convention -> planner obstacle JSON (no ROS)."""
import copy
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import semantic_obstacle_bridge as bridge  # noqa: E402

FIXTURE = HERE.parent / 'framework' / 'fixtures' / 'maps' / 'sem_obstacle.json'


def fixture():
    return json.loads(FIXTURE.read_text())


class SemanticObstacleTests(unittest.TestCase):
    def test_box_obstacle_is_converted(self):
        obstacles = bridge.convert(fixture())
        self.assertEqual(len(obstacles), 1)
        item = obstacles[0]
        self.assertEqual(item['id'], 'crate')
        self.assertEqual(item['min'], [3.1, 1.8, .5])
        self.assertEqual(item['max'], [3.4, 2.2, 1.2])

    def test_velocity_and_window_are_passed(self):
        value = fixture()
        value['features'][-1]['attributes'].update(
            velocity=[0.1, 0.0, 0.0], stamp=3.0, active_from=2.5, active_until=9.0)
        item = bridge.convert(value)[0]
        self.assertEqual(item['velocity'], [.1, 0., 0.])
        self.assertEqual((item['stamp'], item['active_from'], item['active_until']),
                         (3.0, 2.5, 9.0))

    def test_support_only_map_yields_no_obstacles(self):
        value = fixture()
        value['features'] = [item for item in value['features']
                             if (item.get('attributes') or {}).get('role') != 'obstacle']
        self.assertEqual(bridge.convert(value), [])

    def test_bad_geometry_vector_and_time_are_rejected(self):
        value = fixture()
        polygon = copy.deepcopy(value)
        polygon['features'][-1]['geometry'] = {
            'type': 'polygon', 'points': [[3., 1.7, .6], [3.4, 1.7, .6], [3.4, 2.3, .6]]}
        with self.assertRaises(bridge.SemanticObstacleError) as caught:
            bridge.convert(polygon)
        self.assertEqual(caught.exception.code, 'E_SEMANTIC_OBSTACLE_GEOMETRY')
        empty = copy.deepcopy(value)
        empty['features'][-1]['geometry']['max'][2] = 0.5
        with self.assertRaises(bridge.SemanticObstacleError) as caught:
            bridge.convert(empty)
        self.assertEqual(caught.exception.code, 'E_SEMANTIC_OBSTACLE_GEOMETRY')
        bad = copy.deepcopy(value)
        bad['features'][-1]['attributes']['velocity'] = [1., 2.]
        with self.assertRaises(bridge.SemanticObstacleError) as caught:
            bridge.convert(bad)
        self.assertEqual(caught.exception.code, 'E_SEMANTIC_OBSTACLE_VECTOR')
        stamp = copy.deepcopy(value)
        stamp['features'][-1]['attributes']['stamp'] = 'now'
        with self.assertRaises(bridge.SemanticObstacleError) as caught:
            bridge.convert(stamp)
        self.assertEqual(caught.exception.code, 'E_SEMANTIC_OBSTACLE_TIME')


if __name__ == '__main__':
    unittest.main()
