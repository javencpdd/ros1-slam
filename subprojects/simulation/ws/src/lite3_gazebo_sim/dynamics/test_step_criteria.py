"""V8-X1 判据件测试：台阶通过/跌落/未越过后沿，坡道下坡不误判，打滑超限改判。"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import step_criteria as sc


class StepCriteriaTests(unittest.TestCase):
    def test_step_pass(self):
        trace = [(0.0, 0.0, 0.32), (1.0, 0.5, 0.32), (2.0, 1.2, 0.42),
                 (3.0, 1.8, 0.42), (4.0, 2.2, 0.32)]
        result = sc.judge_step(trace, front_x=1.0, rear_x=1.6)
        self.assertTrue(result['pass'])
        self.assertFalse(result['fell'])
        self.assertEqual(result['reason'], 'ok')

    def test_step_fall_after_front(self):
        trace = [(0.0, 0.0, 0.32), (1.0, 0.9, 0.32), (2.0, 1.3, 0.20),
                 (2.5, 1.4, 0.10), (3.0, 1.4, 0.09)]
        result = sc.judge_step(trace, front_x=1.0, rear_x=1.6)
        self.assertFalse(result['pass'])
        self.assertTrue(result['fell'])
        self.assertEqual(result['reason'], 'fell_after_step')

    def test_step_not_crossed(self):
        trace = [(0.0, 0.0, 0.32), (1.0, 0.6, 0.32), (2.0, 0.9, 0.32)]
        result = sc.judge_step(trace, front_x=1.0, rear_x=1.6)
        self.assertFalse(result['pass'])
        self.assertFalse(result['fell'])
        self.assertEqual(result['reason'], 'not_crossed')

    def test_ramp_descent_not_fall(self):
        trace = [(0.0, 0.0, 0.32), (2.0, 1.5, 0.50), (4.0, 3.0, 0.40),
                 (5.0, 3.7, 0.12)]
        result = sc.judge_ramp(trace, rear_x=3.0, z_floor=0.12)
        self.assertTrue(result['pass'])
        self.assertFalse(result['fell'])

    def test_ramp_fall(self):
        trace = [(0.0, 0.0, 0.32), (2.0, 1.5, 0.50), (2.5, 1.6, 0.10),
                 (3.0, 3.7, 0.10)]
        result = sc.judge_ramp(trace, rear_x=3.0)
        self.assertFalse(result['pass'])
        self.assertTrue(result['fell'])

    def test_slip_overrides_pass(self):
        base = {'pass': True, 'reason': 'ok', 'crossed': True}
        self.assertFalse(sc.judge_with_slip(base, 0.08)['pass'])
        self.assertEqual(sc.judge_with_slip(base, 0.08)['reason'], 'slip_exceeded')
        self.assertTrue(sc.judge_with_slip(base, 0.02)['pass'])


if __name__ == '__main__':
    unittest.main()
