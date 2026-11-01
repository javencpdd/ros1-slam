"""Unit checks for the NDT -> A6 candidate bridge; no ROS or simulator needed."""
import math
import unittest

from ndt_recovery_bridge import candidate_payload, score_from_fitness, yaw_of


class NdtBridgeTest(unittest.TestCase):
    def test_score_boundary_matches_acceptance(self):
        self.assertEqual(score_from_fitness(10.0, 10.0), 0.5)
        self.assertAlmostEqual(score_from_fitness(0.0, 10.0), 1.0)
        self.assertAlmostEqual(score_from_fitness(5.0, 10.0), 2.0 / 3.0)
        self.assertIsNone(score_from_fitness(10.0001, 10.0))
        self.assertIsNone(score_from_fitness(-0.1, 10.0))
        self.assertIsNone(score_from_fitness(float('nan'), 10.0))
        self.assertIsNone(score_from_fitness(1.0, 0.0))
        self.assertIsNone(score_from_fitness(True, 10.0))

    def test_candidate_payload_and_rejection(self):
        payload = candidate_payload(1.5, -0.2, 0.0, 0.1, 5.0, 10.0)
        self.assertEqual(payload['pose'], [1.5, -0.2, 0.0])
        self.assertAlmostEqual(payload['score'], 2.0 / 3.0)
        self.assertAlmostEqual(payload['fitness'], 5.0)
        self.assertEqual(payload['source'], 'ndt_monitor')
        self.assertIsNone(candidate_payload(float('inf'), 0, 0, 0, 1.0, 10.0))
        self.assertIsNone(candidate_payload(0, 0, 0, 0, 11.0, 10.0))
        self.assertIsNone(candidate_payload(0, 0, 0, float('nan'), 1.0, 10.0))

    def test_yaw_of_quaternion(self):
        self.assertAlmostEqual(yaw_of(0, 0, 0, 1), 0.0)
        self.assertAlmostEqual(yaw_of(0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)),
                               math.pi / 2)
        self.assertIsNone(yaw_of(0, 0, 0, 0))
        self.assertIsNone(yaw_of(0, 0, float('nan'), 1))


if __name__ == '__main__':
    unittest.main()
