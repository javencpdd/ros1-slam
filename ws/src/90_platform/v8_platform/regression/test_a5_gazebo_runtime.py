#!/usr/bin/env python3
"""Negative controls for the A5 refusal-chain measurement gate."""
import unittest

from a5_gazebo_runtime import classify_measurement

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


if __name__ == '__main__':
    unittest.main()
