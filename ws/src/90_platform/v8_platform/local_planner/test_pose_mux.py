"""Pose handover logic: seamless agreement, jump/stale rejection (no ROS)."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pose_mux  # noqa: E402


class PoseMuxTests(unittest.TestCase):
    def test_switch_requires_fresh_agreeing_source(self):
        mux = pose_mux.PoseMux(max_gap_m=.1, max_age_s=.5)
        mux.update('a', (0., 0., 0.), 10.0)
        self.assertEqual(mux.output(10.0)['source'], 'a')
        mux.update('b', (.05, 0., 0.), 10.1)
        self.assertTrue(mux.select('b', 10.1)['ok'])
        self.assertEqual(mux.output(10.2)['source'], 'b')
        mux.update('b', (5., 0., 0.), 10.3)
        rejected = mux.select('b', 10.3)
        self.assertFalse(rejected['ok'])
        self.assertEqual(rejected['reason'], 'E_POSE_MUX_JUMP')
        self.assertEqual(mux.output(10.4)['reason'], 'E_POSE_MUX_JUMP')
        self.assertTrue(mux.select('a', 10.4)['ok'])
        self.assertEqual(mux.output(10.4)['pose'], (0., 0., 0.))

    def test_stale_and_unknown_sources_are_rejected(self):
        mux = pose_mux.PoseMux(max_gap_m=.1, max_age_s=.5)
        mux.update('a', (0., 0., 0.), 20.0)
        self.assertEqual(mux.select('b', 20.0)['reason'], 'E_POSE_MUX_STALE')
        self.assertEqual(mux.select('c', 20.0)['reason'], 'E_POSE_MUX_SOURCE')
        self.assertEqual(mux.output(20.6)['reason'], 'E_POSE_MUX_STALE')

    def test_not_ready_and_invalid_input(self):
        mux = pose_mux.PoseMux()
        self.assertEqual(mux.output(0.)['reason'], 'E_POSE_MUX_NOT_READY')
        with self.assertRaises(pose_mux.PoseMuxError):
            mux.update('a', (0., 0.), 0.)
        with self.assertRaises(pose_mux.PoseMuxError):
            mux.update('z', (0., 0., 0.), 0.)
        with self.assertRaises(pose_mux.PoseMuxError):
            pose_mux.PoseMux(max_gap_m=0.)


if __name__ == '__main__':
    unittest.main()
