"""Boundary S2 MuJoCo endpoint helpers (no MuJoCo import in the test)."""
import math
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
DYNAMICS = HERE.parents[2] / '95_simulation' / 'lite3_gazebo_sim' / 'dynamics'
sys.path.insert(0, str(DYNAMICS))
import boundary_endpoint as endpoint  # noqa: E402
import boundary.gazebo_endpoint as gazebo  # noqa: E402
from boundary.protocol import TELEM_FAULT, TELEM_OK  # noqa: E402


class BoundaryEndpointTests(unittest.TestCase):
    def test_policy_command_maps_lateral_intent_to_heading(self):
        self.assertEqual(endpoint.policy_command(0., 0., 0.), (0., 0.))
        speed, wz = endpoint.policy_command(.3, 0., .1, max_speed=1., max_wz=1.)
        self.assertAlmostEqual(speed, .3)
        self.assertAlmostEqual(wz, .1)
        self.assertEqual(endpoint.policy_command(.3, 0., 0.)[0], .25)
        speed, wz = endpoint.policy_command(0., .4, 0., max_speed=1.)
        self.assertAlmostEqual(speed, .4)
        self.assertEqual(wz, .5)
        with self.assertRaises(ValueError):
            endpoint.policy_command(float('nan'), 0., 0.)

    def test_telemetry_from_state_is_body_frame(self):
        qpos = [1., 2., .4, 1., 0., 0., 0.] + [0.] * 12
        qvel = [1., 0., 0., 0., 0., .1] + [0.] * 12
        qacc = [0., 0., .19] + [0.] * 6
        frame = endpoint.telemetry_from_state(5, 7, qpos, qvel, qacc)
        self.assertEqual((frame.seq, frame.t_ns), (5, 7))
        self.assertAlmostEqual(frame.x, 1.)
        self.assertAlmostEqual(frame.z, .4)
        self.assertAlmostEqual(frame.vx, 1.)
        self.assertAlmostEqual(frame.gz, .1)
        self.assertAlmostEqual(frame.az, 10.)
        self.assertEqual(frame.state, TELEM_OK)
        self.assertEqual(len(frame.joint_pos), 12)

    def test_gazebo_integrate_rotates_body_velocity(self):
        position, yaw = gazebo.integrate((0., 0., .1), 0., 1., 0., 0., 1.)
        self.assertAlmostEqual(position[0], 1.)
        self.assertAlmostEqual(position[1], 0.)
        position, yaw = gazebo.integrate((0., 0., .1), math.pi / 2, 1., 0., 0., 1.)
        self.assertAlmostEqual(position[0], 0., places=6)
        self.assertAlmostEqual(position[1], 1.)
        position, yaw = gazebo.integrate((0., 0., .1), 0., 0., 0., 1., 1.)
        self.assertAlmostEqual(yaw, 1.)

    def test_fault_bits_are_reported(self):
        qpos = [0.] * 19
        qvel = [0.] * 18
        qacc = [0.] * 6
        frame = endpoint.telemetry_from_state(1, 2, qpos, qvel, qacc,
                                              fault=endpoint.FAULT_POLICY)
        self.assertEqual(frame.state, TELEM_FAULT)
        self.assertEqual(frame.fault, endpoint.FAULT_POLICY)
        self.assertTrue(math.isfinite(frame.az))


if __name__ == '__main__':
    unittest.main()
