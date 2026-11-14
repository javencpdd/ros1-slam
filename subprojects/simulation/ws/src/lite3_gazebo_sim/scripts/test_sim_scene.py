"""sim_scene 纯函数测试（宿主无 ROS 可跑）。"""
import math
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim_scene  # noqa: E402


class SceneTests(unittest.TestCase):
    def test_scene_names(self):
        names = sim_scene.scene_names()
        for name in ('stairs_easy', 'wall_h045', 'blank', 'ramp_envelope'):
            self.assertIn(name, names)
        self.assertGreaterEqual(len(names), 9)

    def test_resolve_short_and_full(self):
        short = sim_scene.resolve_scene('stairs_easy')
        full = sim_scene.resolve_scene('stair_lab_stairs_easy')
        self.assertEqual(short, full)
        self.assertTrue(short.is_file())
        with self.assertRaises(ValueError):
            sim_scene.resolve_scene('no_such_scene')

    def test_models_from_world(self):
        models = sim_scene.models_from_world(sim_scene.resolve_scene('stairs_easy'))
        self.assertEqual(len(models), 1)
        model = models[0]
        self.assertEqual(model['name'], 'stairs_easy')
        self.assertEqual(model['pose'], [0.0] * 6)
        self.assertTrue(model['xml'].startswith('<sdf version="1.6">'))
        self.assertIn('<static>1</static>', model['xml'])
        self.assertNotIn('<pose>0.000 0.000 0 0 0 0</pose>', model['xml'])

    def test_blank_has_no_models(self):
        self.assertEqual(sim_scene.models_from_world(sim_scene.resolve_scene('blank')), [])

    def test_rpy_to_quaternion(self):
        self.assertEqual(sim_scene.rpy_to_quaternion(0, 0, 0), (0.0, 0.0, 0.0, 1.0))
        z = sim_scene.rpy_to_quaternion(0, 0, math.pi / 2)
        self.assertAlmostEqual(z[2], math.sqrt(0.5))
        self.assertAlmostEqual(z[3], math.sqrt(0.5))

    def test_pose_values_invalid(self):
        for text in ('0 0 0', 'a b c d e f', '0 0 0 0 0 nan'):
            with self.assertRaises(ValueError):
                sim_scene.pose_values(text)


if __name__ == '__main__':
    unittest.main()
