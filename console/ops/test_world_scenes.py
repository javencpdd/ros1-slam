#!/usr/bin/env python3
"""Contract tests for the console world-scene generator (read-only, temp output)."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
SCRIPT = HERE / 'build_world_scenes.py'
SPEC = importlib.util.spec_from_file_location('build_world_scenes', SCRIPT)
SCENES = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCENES)


class WorldSceneTest(unittest.TestCase):
    def test_pose_composition_yaw(self):
        matrix = SCENES.pose_matrix('0 0 0 0 0 1.5707963267948966')
        point = SCENES.transform_point(matrix, [1.0, 0.0, 0.0])
        self.assertAlmostEqual(point[0], 0.0, places=5)
        self.assertAlmostEqual(point[1], 1.0, places=5)

    def test_generated_scenes(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = subprocess.run([
                sys.executable, str(SCRIPT),
                '--only', 'm1_world', 'stair_lab_stairs_easy', 'nist_stairs_120',
                '--out', tmp,
            ], cwd=str(REPO), capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
            out = Path(tmp)

            m1 = json.loads((out / 'm1_world.json').read_text(encoding='utf-8'))
            self.assertEqual(m1['schema'], 'v8-world-scene/v1')
            self.assertEqual(len(m1['boxes']), 8)
            names = {box['name'].split('/')[0] for box in m1['boxes']}
            self.assertIn('obs_1', names)
            self.assertIn('perimeter_north', names)
            obs1 = next(box for box in m1['boxes'] if box['name'].startswith('obs_1/'))
            self.assertEqual(obs1['size'], [0.8, 0.8, 0.8])
            self.assertEqual(obs1['color'], [0.8, 0.4, 0.2])
            self.assertEqual(len(obs1['matrix']), 16)

            stairs = json.loads((out / 'stair_lab_stairs_easy.json').read_text(encoding='utf-8'))
            self.assertEqual(len(stairs['boxes']), 195)

            nist = json.loads((out / 'nist_stairs_120.json').read_text(encoding='utf-8'))
            self.assertEqual(len(nist['boxes']), 0)
            self.assertEqual(len(nist['meshes']), 1)
            self.assertEqual(len(nist['meshes'][0]['faces']), 60)
            self.assertGreater(len(nist['meshes'][0]['vertices']), 30)
            self.assertGreater(nist['bounds'][5], 1.0)

            index = json.loads((out / 'worlds.json').read_text(encoding='utf-8'))
            self.assertEqual(len(index['worlds']), 3)


if __name__ == '__main__':
    unittest.main()
