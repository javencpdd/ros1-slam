#!/usr/bin/env python3
"""Pure tests for the 3D command arbiter and kinematic integrator."""
import math
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cmd3d_mux_node as mux  # noqa: E402
import kinematic3d_node as carrier  # noqa: E402


class CommandArbiterTests(unittest.TestCase):
    def test_fresh_map_command_is_tracked(self):
        values, reason = mux.arbitrate([.2, -.1, .05], 'map', 10., False, 10.1)
        self.assertEqual(reason, 'track')
        self.assertEqual(values, (.2, -.1, .05, 0.))

    def test_stale_hold_and_bad_frame_zero(self):
        self.assertEqual(mux.arbitrate([.2, 0., 0.], 'map', 9., False, 10.)[1],
                         'E_CMD3D_STALE')
        self.assertEqual(mux.arbitrate([.2, 0., 0.], 'map', 10., True, 10.1)[1], 'hold')
        self.assertEqual(mux.arbitrate([.2, 0., 0.], 'base_link', 10., False, 10.1)[1],
                         'E_CMD3D_FRAME')

    def test_nonfinite_and_bool_are_rejected(self):
        self.assertEqual(mux.arbitrate([float('inf'), 0., 0.], 'map', 10., False, 10.1)[1],
                         'E_CMD3D_INVALID')
        self.assertEqual(mux.arbitrate([True, 0., 0.], 'map', 10., False, 10.1)[1],
                         'E_CMD3D_INVALID')
        self.assertEqual(mux.arbitrate(None, 'map', 10., False, 10.1)[1],
                         'E_CMD3D_INVALID')


class KinematicCarrierTests(unittest.TestCase):
    def test_integrate_uses_map_velocity_and_yaw_rate(self):
        x, y, z, yaw = carrier.integrate((3., 2., .6), 0., (.2, -.1, .05, .3), .5)
        self.assertAlmostEqual(x, 3.1)
        self.assertAlmostEqual(y, 1.95)
        self.assertAlmostEqual(z, .625)
        self.assertAlmostEqual(yaw, .15)

    def test_integrate_zero_is_stationary(self):
        self.assertEqual(carrier.integrate((1., 2., 3.), .4, (0., 0., 0., 0.), 1.),
                         (1., 2., 3., .4))

    def test_support_snap_clamps_to_declared_surface(self):
        surfaces = [{'traversability': 'supported', 'z_reference': .198798,
                     'footprint': {'min_x': -1.7, 'max_x': 1.7,
                                   'min_y': -1.7, 'max_y': -.98}}]
        self.assertEqual(carrier.snap_support(0., -1.4, .19923, surfaces), .198798)
        self.assertEqual(carrier.snap_support(0., -1.4, .24, surfaces), .24)
        self.assertEqual(carrier.snap_support(0., -2.2, .19923, surfaces), .19923)


if __name__ == '__main__':
    unittest.main()
