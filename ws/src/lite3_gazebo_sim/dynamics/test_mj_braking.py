"""Synthetic braking measurements; no physics engine or policy execution."""
import contextlib
import io
import json
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mj_step_baseline as baseline


def sample(time, x, vx, y=0.0, vy=0.0):
    return {'time': float(time), 'xyz': [float(x), float(y), 0.35],
            'base_vxy': [float(vx), float(vy)],
            'command': [0.3 if time < 1 else 0.0, 0.0, 0.0], 'tau_max': 12.0}


def normal_trace():
    return [sample(0, 0, 0.3), sample(1, 0.3, 0.3),
            sample(1.25, 0.34, 0.01), sample(1.5, 0.342, 0.005),
            sample(1.75, 0.343, 0), sample(2, 0.343, 0)]


class BrakingTests(unittest.TestCase):
    def test_no_movement_rejected(self):
        result = baseline.analyze_braking([sample(t, 0, 0) for t in (0, 1, 2)], 1)
        self.assertFalse(result['completed'])
        self.assertEqual(result['status'], 'INVALID')
        self.assertEqual(result['reason'], 'no_pre_trigger_motion')
        self.assertIsNone(result['confirmation_time'])

    def test_rebound_inside_window_not_hidden_by_endpoints(self):
        trace = [sample(0, 0, 0.3), sample(1, 0.3, 0.3),
                 sample(1.25, 0.312, 0), sample(1.5, 0.3, 0)]
        result = baseline.analyze_braking(trace, 1)
        self.assertFalse(result['completed'])
        self.assertEqual(result['reason'], 'stop_not_completed')
        self.assertAlmostEqual(result['peak_forward_m'], 0.012)
        self.assertAlmostEqual(result['cumulative_xy_m'], 0.024)

    def test_interpolated_trigger_preserves_initial_distance(self):
        trace = [sample(0, 0, 0.3), sample(1, 0.4, 0.3),
                 sample(2, 0.6, 0), sample(2.5, 0.6, 0)]
        result = baseline.analyze_braking(trace, 0.5)
        self.assertTrue(result['completed'])
        self.assertAlmostEqual(result['peak_forward_m'], 0.4)
        self.assertAlmostEqual(result['cumulative_xy_m'], 0.4)
        self.assertEqual(result['first_stationary_time'], 2)
        self.assertEqual(result['confirmation_time'], 2.5)

    def test_old_motion_does_not_qualify_a_stopped_trigger(self):
        trace = [sample(0, 0, .3), sample(.5, .1, 0),
                 sample(1, .1, 0), sample(2, .1, 0)]
        result = baseline.analyze_braking(trace, 1)
        self.assertEqual(result['reason'], 'no_pre_trigger_motion')

    def test_motion_after_confirmation_is_not_a_completed_stop(self):
        result = baseline.analyze_braking(normal_trace() + [sample(3, .6, .3)], 1)
        self.assertFalse(result['completed'])
        self.assertEqual(result['reason'], 'motion_resumed')

    def test_fall_invalidates_the_braking_subresult(self):
        trace = normal_trace()
        trace[-1]['xyz'][2] = .1
        result = baseline.analyze_braking(trace, 1)
        self.assertFalse(result['completed'])
        self.assertEqual(result['reason'], 'base_height_below_floor')

    def test_never_stationary_is_incomplete(self):
        trace = [sample(t, t * 0.3, 0.3) for t in (0, 1, 2, 3)]
        result = baseline.analyze_braking(trace, 1)
        self.assertFalse(result['completed'])
        self.assertEqual(result['status'], 'FAIL')
        self.assertIsNone(result['first_stationary_time'])
        self.assertIsNone(result['confirmation_time'])
        self.assertAlmostEqual(result['cumulative_xy_m'], 0.6)

    def test_normal_stop(self):
        result = baseline.analyze_braking(normal_trace(), 1)
        self.assertTrue(result['completed'])
        self.assertEqual(result['status'], 'PASS')
        self.assertEqual(result['first_stationary_time'], 1.25)
        self.assertEqual(result['confirmation_time'], 1.75)
        self.assertAlmostEqual(result['peak_forward_m'], 0.043)
        self.assertAlmostEqual(result['cumulative_xy_m'], 0.043)
        self.assertEqual(result['parameters']['speed_limit_m_s'], 0.02)
        self.assertEqual(result['parameters']['xy_range_limit_m'], 0.01)
        self.assertEqual(result['parameters']['hold_seconds'], 0.5)
        self.assertFalse(result['parameters']['criteria_151_frozen'])
        self.assertFalse(result['full_chain'])
        self.assertFalse(result['hardware'])

    def test_confirmation_between_samples_is_interpolated(self):
        trace = [sample(0, 0, 0.3), sample(1, 0.3, 0.3),
                 sample(1.2, 0.33, 0.01), sample(2.2, 0.338, 0.01)]
        result = baseline.analyze_braking(trace, 1)
        self.assertAlmostEqual(result['confirmation_time'], 1.7)
        self.assertAlmostEqual(result['cumulative_xy_m'], 0.034)

    def test_short_window_and_lateral_motion_rejected(self):
        for trace in (normal_trace()[:4],
                      [sample(0, 0, 0.3), sample(1, 0.3, 0, vy=0.1),
                       sample(2, 0.3, 0, y=0.1, vy=0.1)]):
            self.assertFalse(baseline.analyze_braking(trace, 1)['completed'])

    def test_invalid_samples_and_unobserved_trigger(self):
        for trace, trigger in ((normal_trace(), None), (normal_trace(), 4),
                               ([sample(0, 0, 0.3), sample(0, 0.3, 0)], 0.5),
                               ([sample(0, 0, 0.3), sample(1, float('nan'), 0)], 0.5)):
            self.assertEqual(baseline.analyze_braking(trace, trigger)['status'], 'INVALID')

    def test_stop_at_validation_before_importing_simulator(self):
        for stop_at in (0, -1, 10, 11, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                baseline.rollout('unused.xml', 'unused.onnx', stop_at=stop_at)

    def test_cli_flat_only_and_default_without_simulation(self):
        for flags, expected_calls in (([], 2), (['--flat-only'], 1)):
            output = io.StringIO()
            with patch.object(sys, 'argv', ['mj_step_baseline.py'] + flags), \
                    patch.object(baseline, 'rollout', return_value={'fell': False, 'xmax': 2}) as run, \
                    contextlib.redirect_stdout(output):
                self.assertEqual(baseline.main(), 0)
            self.assertEqual(run.call_count, expected_calls)
            self.assertEqual('step10' in json.loads(output.getvalue()), expected_calls == 2)

    def test_cli_incomplete_braking_cannot_inherit_baseline_pass(self):
        output = io.StringIO()
        with patch.object(sys, 'argv', ['mj_step_baseline.py', '--flat-only', '--stop-at', '1', '--seed', '7']), \
                patch.object(baseline, 'rollout', return_value={
                    'fell': False, 'braking': {'status': 'FAIL'}}) as run, \
                contextlib.redirect_stdout(output):
            self.assertEqual(baseline.main(), 6)
        self.assertEqual(json.loads(output.getvalue())['verdict'], 'FAIL')
        self.assertEqual(run.call_args.kwargs, {'stop_at': 1.0, 'seed': 7})


if __name__ == '__main__':
    unittest.main()
