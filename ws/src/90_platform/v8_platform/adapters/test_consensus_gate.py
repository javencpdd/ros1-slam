import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from consensus_policy import decide


class ConsensusGateTests(unittest.TestCase):
    def test_agreed_candidates_use_independent_pose(self):
        result, reason = decide(
            {'pose': (1.50, 0.0), 'yaw': 0.0},
            {'pose': (1.55, 0.02), 'yaw': 0.1, 'score': 0.8}, .5)
        self.assertEqual(reason, 'agreed')
        self.assertEqual(result['pose'], [1.55, 0.02])
        self.assertAlmostEqual(result['agreement_m'], 0.05385164807134505)
        self.assertEqual(result['score'], 0.8)

    def test_disagreement_is_rejected(self):
        result, reason = decide(
            {'pose': (1.50, 0.0)},
            {'pose': (2.10, 0.0)}, .5)
        self.assertIsNone(result)
        self.assertEqual(reason, 'disagree')

    def test_missing_consensus_fails_closed(self):
        self.assertEqual(decide({'pose': (1.5, 0.0)}, None, .5), (None, 'missing'))

    def test_malformed_pose_is_rejected(self):
        result, reason = decide({'pose': (1.5, 0.0)}, {'pose': None}, .5)
        self.assertIsNone(result)
        self.assertEqual(reason, 'malformed')


if __name__ == '__main__':
    unittest.main()
