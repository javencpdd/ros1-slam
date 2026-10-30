"""Gazebo 3D chain support-sequence check (no ROS)."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import a5_gazebo3d_runtime as runner


class GazeboChainTests(unittest.TestCase):
    def test_sequence_and_times_in_order(self):
        trace = [[0., 0., 0., 0.], [2., 0., 0., .1988], [4., 0., 0., .3717]]
        sequence, times = runner.support_sequence(trace, [0., .1988, .3717])
        self.assertEqual(sequence, [0., .1988, .3717])
        self.assertEqual(times, [0., 2., 4.])

    def test_missing_level_returns_short_sequence(self):
        sequence, times = runner.support_sequence([[0., 0., 0., 0.]], [0., .1988])
        self.assertEqual(sequence, [0.])
        self.assertEqual(len(times), 1)

    def test_out_of_order_is_rejected_by_the_time_check(self):
        trace = [[0., 0., 0., .3717], [1., 0., 0., 0.], [2., 0., 0., .1988]]
        sequence, times = runner.support_sequence(trace, [0., .1988, .3717])
        self.assertEqual(sequence, [0., .1988, .3717])
        self.assertFalse(all(a < b for a, b in zip(times, times[1:])))


if __name__ == '__main__':
    unittest.main()
