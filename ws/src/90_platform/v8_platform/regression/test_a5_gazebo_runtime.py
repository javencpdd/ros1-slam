#!/usr/bin/env python3
"""Negative controls for the A5 refusal-chain measurement gate."""
import unittest

from a5_gazebo_runtime import (classify_measurement, first_terminal_zero,
                               radial_box_gap, sensed_box)

WANTED = 'E_LOCAL_NO_ROUTE'


def row(reason, wall):
    return {'reason': reason, 'wall': wall, 'sim': wall, 'stop': True}


class MeasurementGateTest(unittest.TestCase):
    def test_sensed_zero_after_input_is_valid(self):
        self.assertEqual(classify_measurement(
            'sensed_no_route', WANTED, row(WANTED, 2.0), row(WANTED, 2.0), {'wall': 1.5}), 'VALID')

    def test_wall_expiry_before_target_is_invalid(self):
        value = classify_measurement(
            'sensed_no_route', WANTED, row('E_LOCAL_WALL_EXPIRED', 1.0), row(WANTED, 1.2),
            {'wall': 0.9})
        self.assertEqual(value, 'INVALID_PREEMPTED_E_LOCAL_WALL_EXPIRED')

    def test_zero_before_obstacle_input_is_invalid(self):
        self.assertEqual(classify_measurement(
            'sensed_no_route', WANTED, row(WANTED, 1.0), row(WANTED, 1.0), {'wall': 1.1}),
            'INVALID_BEFORE_SENSING')

    def test_missing_obstacle_input_is_invalid(self):
        self.assertEqual(classify_measurement(
            'sensed_no_route', WANTED, row(WANTED, 1.0), row(WANTED, 1.0), None),
            'INVALID_BEFORE_SENSING')

    def test_missing_target_reason_is_invalid(self):
        self.assertEqual(classify_measurement(
            'sensed_no_route', WANTED, row('E_LOCAL_WALL_EXPIRED', 1.0), None, {'wall': .5}),
            'INVALID_NO_TARGET_REASON')

    def test_nonzero_before_rest_is_unlatched(self):
        status = [row(WANTED, 1.0), {'reason': 'track', 'wall': 1.1, 'sim': 1.1, 'stop': False},
                  row(WANTED, 1.2)]
        self.assertEqual(classify_measurement(
            'sensed_no_route', WANTED, row(WANTED, 1.0), row(WANTED, 1.2), {'wall': .5},
            status_rows=status, rest_wall=1.5), 'INVALID_UNLATCHED')

    def test_latched_zero_is_valid(self):
        status = [row(WANTED, 1.0), row(WANTED, 1.2)]
        self.assertEqual(classify_measurement(
            'sensed_no_route', WANTED, row(WANTED, 1.0), row(WANTED, 1.2), {'wall': .5},
            status_rows=status, rest_wall=1.5), 'VALID')

    def test_localization_does_not_require_obstacle_input(self):
        wanted = 'E_LOCAL_LOCALIZATION'
        self.assertEqual(classify_measurement(
            'localization', wanted, row(wanted, 1.0), row(wanted, 1.0), None), 'VALID')

    def test_sensor_freshness_does_not_require_obstacle_input(self):
        wanted = 'E_LOCAL_OBSTACLES_STALE'
        self.assertEqual(classify_measurement(
            'sensor_freshness', wanted, row(wanted, 1.0), row(wanted, 1.0), None), 'VALID')

    def test_transient_zero_is_skipped(self):
        rows = [row('E_LOCAL_EXPIRED', 1.0), row('E_LOCAL_NO_TRAJECTORY', 1.1),
                row(WANTED, 1.2)]
        self.assertEqual(first_terminal_zero(rows)['reason'], WANTED)

    def test_world_update_is_a_terminal_zero(self):
        rows = [row('E_LOCAL_WORLD_UPDATED', 1.0)]
        self.assertEqual(first_terminal_zero(rows)['reason'], 'E_LOCAL_WORLD_UPDATED')


class SensedBoxTests(unittest.TestCase):
    POSE = (0.1, 0.05, 0.)

    def face(self, x=0.65, y=-0.2, z=0.1, n=4):
        return [(x + 0.01 * i, y + 0.1 * j, z + 0.05 * k)
                for i in range(n) for j in range(5) for k in range(5)]

    def test_roi_box_is_mapped_from_pitched_sensor_points(self):
        observed, roi = sensed_box(self.face(), self.POSE)
        self.assertIsNotNone(observed)
        self.assertGreaterEqual(len(roi), 12)
        self.assertLessEqual(observed['min'][0], 0.75)
        self.assertGreaterEqual(observed['max'][0], 0.95)
        self.assertLessEqual(observed['min'][1], -0.15)
        self.assertGreaterEqual(observed['max'][1], 0.25)
        self.assertAlmostEqual(observed['max'][2], 0.62, places=2)
        self.assertEqual(observed['min'][2], 0.0)

    def test_side_object_corner_is_excluded(self):
        corner = [(1.1 + 0.01 * i, 0.649, 0.3) for i in range(20)]
        observed, _roi = sensed_box(corner, self.POSE)
        self.assertIsNone(observed)

    def test_outside_roi_is_not_an_obstacle(self):
        points = [(3.0 + 0.01 * i, 0.0, 0.3) for i in range(20)]
        observed, roi = sensed_box(points, self.POSE)
        self.assertIsNone(observed)
        self.assertEqual(roi, [])

    def test_partial_scan_is_not_an_obstacle(self):
        points = [(0.8, 0.0, 0.3), (0.81, 0.0, 0.3), (0.82, 0.0, 0.3)]
        observed, _roi = sensed_box(points, self.POSE)
        self.assertIsNone(observed)


class BoxGapTests(unittest.TestCase):
    def test_gap_uses_box_center_and_half_extents(self):
        center, half, radius = (1.8, .2), (.3, .2), .15
        self.assertAlmostEqual(radial_box_gap((1.8, .2), center, half, radius), -.15)
        self.assertAlmostEqual(radial_box_gap((1.8, .85), center, half, radius), .30)
        self.assertAlmostEqual(radial_box_gap((1.2, .2), center, half, radius), .15)


if __name__ == '__main__':
    unittest.main()
