import json
import tempfile
import unittest
from pathlib import Path

import demo_slam_survey


class DemoSlamSurveyTests(unittest.TestCase):
    def test_interpolate_midpoint_and_clamp(self):
        points = [
            {'x': 0., 'y': 0., 'z': 0., 'yaw': 0.},
            {'x': 1., 'y': 0., 'z': 1., 'yaw': 0.},
        ]
        self.assertEqual(demo_slam_survey.interpolate(points, 0.), (0., 0., 0., 0.))
        self.assertAlmostEqual(demo_slam_survey.interpolate(points, 0.5)[0], 0.3535533906)
        self.assertAlmostEqual(demo_slam_survey.interpolate(points, 0.5)[2], 0.3535533906)
        self.assertEqual(demo_slam_survey.interpolate(points, 9.), (1., 0., 1., 0.))

    def test_load_waypoints_requires_two_points(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'path.json'
            path.write_text(json.dumps({'waypoints': [{'x': 0, 'y': 0, 'z': 0}]}))
            with self.assertRaises(ValueError):
                demo_slam_survey.load_waypoints(path)


if __name__ == '__main__':
    unittest.main()
