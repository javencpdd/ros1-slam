import math, os, sys, unittest
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import follower

class FollowerTests(unittest.TestCase):
    def test_forward(self):
        v, w, arrived = follower.twist_for((0, 0, 0), (1.0, 0.0))
        self.assertGreater(v, 0); self.assertAlmostEqual(w, 0, places=6); self.assertFalse(arrived)

    def test_turn_before_drive(self):
        v, w, _ = follower.twist_for((0, 0, 0), (0.0, 0.5))
        self.assertEqual(v, 0.0); self.assertGreater(w, 0)

    def test_arrived(self):
        v, w, arrived = follower.twist_for((0.02, 0.0, 0), (0.0, 0.0))
        self.assertEqual((v, w, arrived), (0.0, 0.0, True))

    def test_cancel_is_zero(self):
        self.assertEqual(follower.stop_twist(), (0.0, 0.0))

    def test_wrap(self):
        self.assertAlmostEqual(follower.wrap(3 * math.pi), math.pi, places=6)

if __name__ == '__main__':
    unittest.main(verbosity=2)
