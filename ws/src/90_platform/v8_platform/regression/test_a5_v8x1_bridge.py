"""A5->V8-X1 bridge helpers (no ROS/MuJoCo)."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2] / '95_simulation' / 'lite3_gazebo_sim' / 'dynamics'))
import a5_v8x1_bridge as bridge


def fixture():
    return {'surfaces': [
        {'surface_id': 'ground', 'z_reference': 0., 'traversability': 'supported',
         'footprint': {'min_x': 0., 'max_x': 4., 'min_y': 0., 'max_y': 1.}},
        {'surface_id': 'step1', 'z_reference': .2, 'traversability': 'supported',
         'footprint': {'min_x': 1.2, 'max_x': 3., 'min_y': 0., 'max_y': 1.}},
        {'surface_id': 'ignored', 'z_reference': 9., 'traversability': 'blocked',
         'footprint': {'min_x': 0., 'max_x': 9., 'min_y': 0., 'max_y': 9.}}]}


class BridgeTests(unittest.TestCase):
    def test_world_map_round_trip_places_map_origin(self):
        x, y = bridge.world_to_map(0.525, 0., 0., -2.7)
        self.assertAlmostEqual(x, 0.)
        self.assertAlmostEqual(y, -2.175)
        world_x, world_y = bridge.map_to_world(x, y, 0., -2.7)
        self.assertAlmostEqual(world_x, 0.525)
        self.assertAlmostEqual(world_y, 0.)

    def test_map_velocity_rotates_into_world(self):
        self.assertEqual(bridge.map_velocity_to_world(0., 1.), (1., 0.))
        self.assertEqual(bridge.map_velocity_to_world(1., 0.), (0., -1.))
        vx, vy, wz = bridge.body_command(0., .5, 0., .4)
        self.assertAlmostEqual(vx, .2)
        self.assertAlmostEqual(vy, 0.)
        self.assertAlmostEqual(wz, 0.)
        vx, vy, wz = bridge.body_command(0., .5, 1.5707963267948966, .4)
        self.assertAlmostEqual(vx, .2)
        self.assertAlmostEqual(vy, 0.)
        self.assertAlmostEqual(wz, -.5)
        self.assertAlmostEqual(bridge.body_command(0., 0., .3, .4)[2], -.3)

    def test_support_z_uses_highest_supported_footprint_with_margin(self):
        value = fixture()
        self.assertEqual(bridge.support_z(value, 2., .5, .15), .2)
        self.assertEqual(bridge.support_z(value, .5, .5, .15), 0.)
        self.assertEqual(bridge.support_z(value, 3.8, .5, .15), 0.)
        self.assertIsNone(bridge.support_z(value, 10., 10., .15))

    def test_waypoint_snap_and_braking_are_bounded(self):
        snap, error = bridge.should_snap(0., 1.28, .890358, (0., 1.34, .890358))
        self.assertTrue(snap)
        self.assertAlmostEqual(error, .06)
        self.assertFalse(bridge.should_snap(0., 1.20, .890358, (0., 1.34, .890358))[0])
        self.assertFalse(bridge.should_snap(0., 1.34, .544578, (0., 1.34, .890358))[0])
        self.assertEqual(bridge.path_scale(None), 1.)
        self.assertEqual(bridge.path_scale(.6), 1.)
        self.assertAlmostEqual(bridge.path_scale(.15), .5)

    def test_nonfinite_state_recovers_then_terminal(self):
        self.assertEqual(bridge.walk.nonfinite_state(0, True), (0, False))
        self.assertEqual(bridge.walk.nonfinite_state(0, False, 5), (1, False))
        self.assertEqual(bridge.walk.nonfinite_state(4, False, 5), (5, True))
        self.assertEqual(bridge.walk.nonfinite_state(3, True, 5), (0, False))

    def test_stall_verdict_retries_then_fails(self):
        self.assertEqual(bridge.stall_verdict(.09, 30., 10., 0, 3), 'progress')
        self.assertEqual(bridge.stall_verdict(.01, 9.9, 10., 0, 3), 'wait')
        self.assertEqual(bridge.stall_verdict(.01, 10., 10., 0, 3), 'retry')
        self.assertEqual(bridge.stall_verdict(.01, 10., 10., 2, 3), 'retry')
        self.assertEqual(bridge.stall_verdict(.01, 10., 10., 3, 3), 'fail')
        # A jump below the progress threshold still counts only by distance.
        self.assertEqual(bridge.stall_verdict(.0799, 0., 10., 0, 3), 'wait')


if __name__ == '__main__':
    unittest.main()
