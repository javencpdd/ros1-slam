#!/usr/bin/env python3
"""Contract tests for the 3D run ledger collector (read-only, temp output)."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

REPO = Path(__file__).resolve().parents[2]
COLLECTOR = REPO / 'scripts/ops/collect_3d_runs.py'


class CollectorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.out = Path(cls.tmp.name) / 'runs.json'
        result = subprocess.run([sys.executable, str(COLLECTOR), '--out', str(cls.out)],
                                cwd=str(REPO), capture_output=True, text=True)
        assert result.returncode == 0, result.stderr + result.stdout
        cls.data = json.loads(cls.out.read_text(encoding='utf-8'))

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_schema_and_current(self):
        self.assertEqual(self.data['schema'], 'v8_3d_runs/v1')
        self.assertEqual(self.data['current_3d']['passed'], 4)
        self.assertEqual(self.data['current_3d']['total'], 4)

    def test_unique_runs_and_evidence(self):
        runs = self.data['runs']
        self.assertGreater(len(runs), 50)
        uids = [run['uid'] for run in runs]
        self.assertEqual(len(uids), len(set(uids)))
        for run in runs:
            self.assertTrue(run['evidence'], run['uid'])
            for item in run['evidence']:
                self.assertTrue((REPO / item['path']).exists(), item['path'])

    def test_aggregates_exclude_expected_refusal(self):
        agg = self.data['aggregates']['by_type']['3d_evt']
        self.assertEqual(agg['runs'], 3)
        self.assertEqual(agg['passed'], 3)
        self.assertEqual(agg['metric_runs'], 2)
        self.assertAlmostEqual(agg['goal_distance_m']['mean'], 0.167, places=3)


if __name__ == '__main__':
    unittest.main()
