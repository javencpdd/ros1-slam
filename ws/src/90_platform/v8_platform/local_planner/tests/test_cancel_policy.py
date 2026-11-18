#!/usr/bin/env python3
"""Cancel wire-format controls; no ROS required."""
import unittest

import cancel_policy


class CancelPolicyTests(unittest.TestCase):
    def test_parse_current_and_all(self):
        self.assertEqual(cancel_policy.parse_cancel('7:current:operator'),
                         (7, 'current', 'operator'))
        self.assertEqual(cancel_policy.parse_cancel('8:all'), (8, 'all', ''))
        self.assertEqual(cancel_policy.parse_cancel('9:all:two words'),
                         (9, 'all', 'two words'))

    def test_malformed_refused(self):
        for text in ('', '7', 'x:all', '7:bad', '7:all:bad;reason',
                     '7:all:' + 'a' * 201, None, 7):
            self.assertIsNone(cancel_policy.parse_cancel(text), text)

    def test_ack_text(self):
        self.assertEqual(cancel_policy.ack_text(3, 'cancelled'), '3:cancelled')
        self.assertEqual(cancel_policy.ack_text(0, 'ack_fail'), '0:ack_fail')
        for seq, outcome in ((-1, 'cancelled'), (True, 'cancelled'), (1, 'ok')):
            with self.assertRaises(ValueError):
                cancel_policy.ack_text(seq, outcome)


if __name__ == '__main__':
    unittest.main()
