"""A successful phase prefix must never certify a requested full gait."""
import unittest
from crawl_gait import complete_sequence


class CompletionTests(unittest.TestCase):
    def test_green_prefix_is_incomplete(self):
        self.assertFalse(complete_sequence([{'ok': True}] * 60, 8, 7, False))
        self.assertFalse(complete_sequence([{'ok': True}] * 60, 16, 7, False))

    def test_full_sequence_requires_phase_safety_and_no_abort(self):
        self.assertTrue(complete_sequence([{'ok': True}] * 60, 8, 8, False))
        self.assertFalse(complete_sequence([{'ok': True}], 8, 8, True))
        self.assertFalse(complete_sequence([{'ok': False}], 8, 8, False))
        self.assertFalse(complete_sequence([{'ok': True}], 8, 8, False, 'E_GAIT_SUPPORT_MARGIN'))
        self.assertFalse(complete_sequence([], 0, 0, False))


if __name__ == '__main__':
    unittest.main()
