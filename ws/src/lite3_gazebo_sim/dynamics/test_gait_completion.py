"""A successful phase prefix must never certify a requested full gait."""
import unittest
from crawl_gait import complete_sequence, run


class CompletionTests(unittest.TestCase):
    def test_green_prefix_is_incomplete(self):
        self.assertFalse(complete_sequence([{'ok': True}] * 60, 8, 7, False))
        self.assertFalse(complete_sequence([{'ok': True}] * 60, 16, 7, False))

    def test_invalid_pilot_parameters_fail_before_loading_a_model(self):
        for kwargs in ({'shift_attempts': 0}, {'shift_attempts': True},
                       {'start_yaw': float('nan')}, {'start_yaw': 2.0}):
            with self.assertRaises(ValueError):
                run(xml='missing.xml', **kwargs)

    def test_full_sequence_requires_phase_safety_and_no_abort(self):
        self.assertTrue(complete_sequence([{'ok': True}] * 60, 8, 8, False))
        self.assertFalse(complete_sequence([{'ok': True}], 8, 8, True))
        self.assertFalse(complete_sequence([{'ok': False}], 8, 8, False))
        self.assertFalse(complete_sequence([{'ok': True}], 8, 8, False, 'E_GAIT_SUPPORT_MARGIN'))
        self.assertFalse(complete_sequence([], 0, 0, False))


if __name__ == '__main__':
    unittest.main()
