#!/usr/bin/env python3
"""Pure checks for the DreamWaQ 270-state adapter (no MuJoCo/ONNX required)."""
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import mj_dreamwaq_walk as walk

POSE = walk.DEFAULT_POSE


class AdapterTest(unittest.TestCase):
    def test_observation_layout(self):
        qpos = np.zeros(19)
        qpos[3:7] = [1., 0., 0., 0.]
        qpos[7:19] = POSE
        qvel = np.zeros(18)
        obs = walk.observation(qpos, qvel, [.3, 0., 0.], np.zeros(12))
        self.assertEqual(obs.shape, (45,))
        self.assertTrue(np.allclose(obs[0:3], [0., 0., 0.]))
        self.assertTrue(np.allclose(obs[3:6], [0., 0., -1.]))
        self.assertTrue(np.allclose(obs[6:9], [.6, 0., 0.]))
        self.assertTrue(np.allclose(obs[9:21], np.zeros(12)))
        self.assertTrue(np.allclose(obs[21:33], np.zeros(12)))
        self.assertTrue(np.allclose(obs[33:45], np.zeros(12)))

    def test_observation_uses_body_frame(self):
        qpos = np.zeros(19)
        qpos[3:7] = [np.sqrt(.5), 0., 0., np.sqrt(.5)]
        qvel = np.zeros(18)
        qvel[3:6] = [1., 0., 0.]
        obs = walk.observation(qpos, qvel, [0., 0., 0.], np.zeros(12))
        self.assertTrue(np.allclose(obs[0:3], [0., -.25, 0.], atol=1e-6))

    def test_state_newest_first(self):
        history = [np.full(45, i, dtype=np.float32) for i in range(1, 6)]
        state = walk.state_vector(np.zeros(45, dtype=np.float32), history)
        self.assertEqual(state.shape, (270,))
        for i in range(5):
            self.assertTrue(np.allclose(state[45 * (i + 1):45 * (i + 2)], 5 - i))

    def test_state_oldest_first(self):
        history = [np.full(45, i, dtype=np.float32) for i in range(1, 6)]
        state = walk.state_vector(np.zeros(45, dtype=np.float32), history, newest_first=False)
        for i in range(5):
            self.assertTrue(np.allclose(state[45 * (i + 1):45 * (i + 2)], i + 1))

    def test_invalid_history_is_rejected(self):
        with self.assertRaises(ValueError):
            walk.state_vector(np.zeros(45), [np.zeros(45)] * 4)

    def test_yaw_of_and_heading_sign(self):
        self.assertAlmostEqual(np.degrees(walk.yaw_of([np.sqrt(.5), 0., 0., np.sqrt(.5)])),
                               90., places=6)
        left = walk.heading_command(.3, .2, 0.)
        self.assertTrue(left[2] < 0)
        right = walk.heading_command(.3, 0., .5)
        self.assertTrue(right[2] < 0)
        self.assertAlmostEqual(left[0], .3)
        self.assertLessEqual(abs(walk.heading_command(.3, 1.0, 1.0, max_wz=.2)[2]), .2)

    def test_seeded_initial_perturbation(self):
        a = walk.sample_initial(7, True)
        self.assertEqual(a, walk.sample_initial(7, True))
        self.assertLessEqual(abs(a['dx']), .05)
        self.assertLessEqual(abs(a['dy']), .10)
        self.assertLessEqual(abs(a['dyaw']), .0873)
        self.assertLessEqual(abs(a['dz']), .005)
        self.assertEqual(walk.sample_initial(7, False),
                         {'dx': 0., 'dy': 0., 'dyaw': 0., 'dz': 0.})

    def test_joint_target_scaling(self):
        target = walk.joint_target(np.ones(12), POSE)
        self.assertTrue(np.allclose(target, POSE + walk.ACTION_SCALE))

    def test_stairs_xml_has_expected_geoms(self):
        with tempfile.TemporaryDirectory() as directory:
            flat = Path(directory) / 'flat.xml'
            flat.write_text('<mujoco model="flat"><worldbody/></mujoco>\n')
            path = walk.make_stairs_xml(flat, .10, steps=3, depth=.5, width=3., front=1.)
            try:
                root = ET.parse(path).getroot()
                geoms = root.find('worldbody').findall('geom')
                self.assertEqual(len(geoms), 3)
                self.assertEqual([g.get('name') for g in geoms],
                                 ['step1', 'step2', 'step3'])
                self.assertEqual(geoms[-1].get('pos').split()[2], '0.15')
            finally:
                shutil.rmtree(Path(path).parent, ignore_errors=True)


if __name__ == '__main__':
    unittest.main()
