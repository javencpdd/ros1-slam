import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from candidate_consensus import CandidateConsensus


class CandidateConsensusTests(unittest.TestCase):
    def test_r1075_wrong_basin_is_rejected_by_consensus(self):
        gate = CandidateConsensus(radius=0.6, min_support=2)
        result = gate.select([
            {'source': 0.5, 'pose': (0.5, 0.0)},
            {'source': 0.2, 'pose': (0.0, -0.2)},
            {'source': 0.1, 'pose': (4.0, -0.1)},
        ])
        self.assertIsNotNone(result)
        self.assertEqual(result['support'], 2)
        self.assertEqual(result['source'], 0.2)
        self.assertEqual(result['pose'], [0.0, -0.2])

    def test_disjoint_candidates_are_rejected(self):
        gate = CandidateConsensus(radius=0.3, min_support=2)
        self.assertIsNone(gate.select([
            {'source': 0.5, 'pose': (0.0, 0.0)},
            {'source': 0.2, 'pose': (1.0, 0.0)},
            {'source': 0.1, 'pose': (2.0, 0.0)},
        ]))

    def test_tied_clusters_are_rejected(self):
        gate = CandidateConsensus(radius=0.5, min_support=2)
        self.assertIsNone(gate.select([
            {'source': 0.5, 'pose': (0.0, 0.0)},
            {'source': 0.2, 'pose': (0.1, 0.0)},
            {'source': 0.5, 'pose': (5.0, 0.0)},
            {'source': 0.2, 'pose': (5.1, 0.0)},
        ]))

    def test_chain_does_not_absorb_third_point(self):
        gate = CandidateConsensus(radius=0.6, min_support=2)
        result = gate.select([
            {'source': 0.5, 'pose': (0.0, 0.0)},
            {'source': 0.2, 'pose': (0.55, 0.0)},
            {'source': 0.1, 'pose': (1.1, 0.0)},
        ])
        self.assertIsNotNone(result)
        self.assertEqual(result['support'], 2)
        self.assertEqual(result['source'], 0.2)
        self.assertEqual(result['pose'], [0.55, 0.0])


if __name__ == '__main__':
    unittest.main()
