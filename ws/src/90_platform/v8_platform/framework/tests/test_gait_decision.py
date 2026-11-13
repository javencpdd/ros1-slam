"""步态决策状态机单测（纯 Python，无 ROS）。"""
import sys as _sys
from pathlib import Path as _Path

_BASE = _Path(__file__).resolve().parent.parent
for _entry in ('', 'nodes', 'tools', 'smokes'):
    _sys.path.insert(0, str(_BASE / _entry) if _entry else str(_BASE))
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(_BASE))
from gait_decision import GaitDecision  # noqa: E402


def ahead(step_m, distance_m):
    return {'ahead': True, 'step_m': step_m, 'distance_m': distance_m}


NO_STEP = {'ahead': False, 'reason': 'no_step_within_lookahead'}


class GaitDecisionTests(unittest.TestCase):
    def setUp(self):
        self.d = GaitDecision(enter_m=1.0, exit_m=1.5, min_step_m=0.05,
                              max_step_m=0.20, flat_gait='flat_mid',
                              step_gait='high_step')

    def test_switch_to_step_when_close(self):
        self.assertEqual(self.d.update(ahead(0.15, 0.8)), 'high_step')
        self.assertTrue(self.d.climbing)

    def test_no_switch_beyond_enter(self):
        self.assertEqual(self.d.update(ahead(0.15, 1.2)), 'flat_mid')

    def test_back_to_flat_when_clear(self):
        self.d.update(ahead(0.15, 0.8))
        self.assertEqual(self.d.update(NO_STEP), 'flat_mid')
        self.assertFalse(self.d.climbing)

    def test_hysteresis_keeps_climbing(self):
        self.d.update(ahead(0.15, 0.8))
        self.assertEqual(self.d.update(ahead(0.15, 1.2)), 'high_step')

    def test_too_high_refused(self):
        self.assertEqual(self.d.update(ahead(0.30, 0.5)), 'flat_mid')
        self.assertEqual(self.d.refusal, 'step_too_high')
        self.assertFalse(self.d.climbing)

    def test_small_ledge_ignored(self):
        self.assertEqual(self.d.update(ahead(0.02, 0.5)), 'flat_mid')

    def test_enter_once_no_repeat_change(self):
        self.assertEqual(self.d.update(ahead(0.15, 0.8)), 'high_step')
        self.assertEqual(self.d.update(ahead(0.15, 0.7)), 'high_step')


if __name__ == '__main__':
    unittest.main(verbosity=2)
