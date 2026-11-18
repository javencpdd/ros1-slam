#!/usr/bin/env python3
"""Negative controls for the local planner node stop/expiry ordering."""
import unittest

from ros_node import NON_LATCHING, map_from_message, stop_reason

RESULT = {'valid_until': 1.9}


def call(**kw):
    base = dict(fresh=None, latched=False, latched_reason=None, result=RESULT,
                result_at=0., wall_now=0.2, wall_watchdog_s=1.8, sim_now=1.0,
                generation=1, result_generation=1, last_error='ok')
    base.update(kw)
    return stop_reason(**base)


class MapMessageTest(unittest.TestCase):
    def test_raw_map_and_snapshot_envelope(self):
        self.assertEqual(map_from_message('{"map_id": "m", "map_version": 2}'),
                         {'map_id': 'm', 'map_version': 2})
        self.assertEqual(
            map_from_message('{"map": {"map_id": "m", "map_version": 3}, "ok": true}'),
            {'map_id': 'm', 'map_version': 3})

    def test_non_object_is_rejected(self):
        for text in ('[]', '"x"', '{"map": []}'):
            with self.assertRaises(ValueError):
                map_from_message(text)


class StopPolicyTest(unittest.TestCase):
    def test_freshness_wins(self):
        self.assertEqual(call(fresh='E_LOCAL_LOCALIZATION'), 'E_LOCAL_LOCALIZATION')

    def test_latch_wins_over_valid_plan(self):
        self.assertEqual(call(latched=True, latched_reason='E_LOCAL_EXPIRED'), 'E_LOCAL_EXPIRED')

    def test_ok_is_not_a_stop_reason(self):
        self.assertEqual(call(result=None, last_error='ok'), 'E_LOCAL_NO_TRAJECTORY')

    def test_terminal_last_error_survives_without_result(self):
        self.assertEqual(call(result=None, last_error='E_LOCAL_NO_ROUTE'), 'E_LOCAL_NO_ROUTE')

    def test_world_update_precedes_expiry(self):
        self.assertEqual(call(generation=2, sim_now=2.0, wall_now=2.0,
                              result_generation=1), 'E_LOCAL_WORLD_UPDATED')

    def test_sim_expiry_precedes_wall_fallback(self):
        self.assertEqual(call(sim_now=1.9, wall_now=2.0), 'E_LOCAL_EXPIRED')

    def test_wall_fallback_when_clock_is_paused(self):
        self.assertEqual(call(sim_now=0., wall_now=2.0), 'E_LOCAL_WALL_EXPIRED')

    def test_wall_fallback_when_sim_has_not_expired(self):
        self.assertEqual(call(sim_now=1.0, wall_now=2.0), 'E_LOCAL_WALL_EXPIRED')

    def test_valid_plan_has_no_reason(self):
        self.assertIsNone(call())

    def test_latched_without_reason_is_labeled(self):
        self.assertEqual(call(latched=True), 'E_LOCAL_STOP_LATCHED')

    def test_plan_expiry_and_world_update_are_replannable(self):
        self.assertIn('E_LOCAL_EXPIRED', NON_LATCHING)
        self.assertIn('E_LOCAL_WORLD_UPDATED', NON_LATCHING)
        self.assertNotIn('E_LOCAL_WALL_EXPIRED', NON_LATCHING)


if __name__ == '__main__':
    unittest.main()
