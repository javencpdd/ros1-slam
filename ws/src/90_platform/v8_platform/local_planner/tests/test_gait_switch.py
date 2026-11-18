"""gait_switch.GaitSwitchHold 单测（纯 Python，无 ROS）。"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import gait_switch as gs


class ValidationTests(unittest.TestCase):
    def test_rejects_bad_numbers(self):
        for kwargs in ({'settle_s': -0.1}, {'settle_s': float('nan')},
                       {'timeout_s': 0.0}, {'timeout_s': float('inf')},
                       {'status_stale_s': -1.0}, {'status_stale_s': True}):
            with self.assertRaises(ValueError):
                gs.GaitSwitchHold(**kwargs)


class GaitSwitchHoldTests(unittest.TestCase):
    def setUp(self):
        self.hold = gs.GaitSwitchHold(settle_s=0.5, timeout_s=5.0,
                                      status_stale_s=1.5)
        self.t = 100.0

    def status(self, age=0.0):
        return self.t - age

    def update(self, desired, active, age=0.0):
        return self.hold.update(desired, active, self.status(age), self.t)

    def test_first_desired_is_baseline_not_switch(self):
        out = self.update('flat_mid', 'flat_low')
        self.assertFalse(out['hold'])
        self.assertEqual(out['reason'], gs.IDLE)
        self.assertEqual(self.hold.baseline, 'flat_mid')

    def test_change_holds_until_active_matches_and_settles(self):
        self.update('flat_mid', 'flat_mid')
        out = self.update('high_step', 'flat_mid')
        self.assertTrue(out['hold'])
        self.assertEqual(out['reason'], gs.WAITING)
        self.assertEqual(out['target'], 'high_step')
        self.assertFalse(out['timed_out'])
        self.t += 0.2
        out = self.update('high_step', 'high_step')
        self.assertTrue(out['hold'])          # settle not reached
        self.assertEqual(out['reason'], gs.WAITING)
        self.t += 0.5
        out = self.update('high_step', 'high_step')
        self.assertFalse(out['hold'])
        self.assertEqual(out['reason'], gs.DONE)
        self.assertEqual(self.hold.baseline, 'high_step')
        out = self.update('high_step', 'high_step')
        self.assertFalse(out['hold'])
        self.assertEqual(out['reason'], gs.IDLE)

    def test_zero_settle_accepts_immediately(self):
        hold = gs.GaitSwitchHold(settle_s=0.0)
        self.t = 10.0
        hold.update('flat_mid', 'flat_mid', 10.0, self.t)
        out = hold.update('high_step', 'high_step', 10.0, self.t)
        self.assertFalse(out['hold'])
        self.assertEqual(out['reason'], gs.DONE)

    def test_stale_status_never_releases(self):
        self.update('flat_mid', 'flat_mid')
        out = self.update('high_step', 'high_step', age=1.6)
        self.assertTrue(out['hold'])
        self.assertEqual(out['reason'], gs.STATUS_STALE)
        self.assertFalse(out['status_fresh'])

    def test_missing_status_is_stale(self):
        self.update('flat_mid', 'flat_mid')
        out = self.hold.update('high_step', None, None, self.t)
        self.assertTrue(out['hold'])
        self.assertEqual(out['reason'], gs.STATUS_STALE)

    def test_future_status_is_stale(self):
        self.update('flat_mid', 'flat_mid')
        out = self.hold.update('high_step', 'high_step', self.t + 1.0, self.t)
        self.assertTrue(out['hold'])
        self.assertEqual(out['reason'], gs.STATUS_STALE)

    def test_timeout_keeps_holding(self):
        self.update('flat_mid', 'flat_mid')
        out = self.update('high_step', 'flat_mid')
        self.t += 5.1
        out = self.update('high_step', 'flat_mid')
        self.assertTrue(out['hold'])
        self.assertTrue(out['timed_out'])
        self.assertEqual(out['reason'], gs.TIMEOUT)
        self.t += 60.0
        out = self.update('high_step', 'flat_mid')
        self.assertTrue(out['hold'])
        self.assertEqual(out['reason'], gs.TIMEOUT)

    def test_revert_to_baseline_cancels_hold(self):
        self.update('flat_mid', 'flat_mid')
        self.update('high_step', 'flat_mid')
        out = self.update('flat_mid', 'flat_mid')
        self.assertFalse(out['hold'])
        self.assertEqual(out['reason'], gs.IDLE)
        self.assertIsNone(self.hold.target)

    def test_retarget_while_holding_resets_timer(self):
        self.update('flat_mid', 'flat_mid')
        self.update('high_step', 'flat_mid')
        self.t += 0.4
        out = self.update('flat_high', 'flat_mid')
        self.assertTrue(out['hold'])
        self.assertEqual(out['target'], 'flat_high')
        self.assertAlmostEqual(out['held_s'], 0.0, places=6)

    def test_desired_lost_still_holds(self):
        self.update('flat_mid', 'flat_mid')
        self.update('high_step', 'flat_mid')
        out = self.update(None, 'flat_mid')
        self.assertTrue(out['hold'])
        self.assertEqual(out['reason'], gs.DESIRED_LOST)

    def test_clock_failure_keeps_existing_hold(self):
        self.update('flat_mid', 'flat_mid')
        self.update('high_step', 'flat_mid')
        out = self.hold.update('high_step', 'flat_mid', self.t, float('nan'))
        self.assertTrue(out['hold'])
        self.assertEqual(out['reason'], gs.CLOCK)


if __name__ == '__main__':
    unittest.main(verbosity=2)
