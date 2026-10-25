"""V8-13 测试：矩阵全绿（正式集 6 场景×4 组合×2 轨=48 cell）、负例预期失败定位、
估计轨扰动不假失败、锁可追溯、失败注入定位。"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'scenarios'))
sys.path.insert(0, str(HERE.parent / 'calibration'))
import calibration as cal  # noqa: E402
import regression as reg  # noqa: E402
import scenarios as scn  # noqa: E402


class MatrixTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = reg.run_matrix('official')

    def test_matrix_all_green(self):
        s = self.result['summary']
        self.assertEqual(s['cells'], 6 * 4 * 2)
        self.assertEqual(s['passed'], 4 * 4 * 2)          # 正例 SC01/02/03/08 ×4×2
        self.assertEqual(s['expected_failures'], 2 * 4 * 2)  # 负例 SC07/10 ×4×2
        self.assertEqual(s['unexpected_failures'], 0)

    def test_negative_localized_at_plan(self):
        sc07 = [c for c in self.result['cells'] if c['scenario'] == 'SC07']
        self.assertTrue(sc07)
        self.assertTrue(all(c['stage'] == 'plan' and c['reason'] == 'E_PLAN_NO_ROUTE'
                            and c['outcome'] == 'expected_failure' for c in sc07))
        sc10 = [c for c in self.result['cells'] if c['scenario'] == 'SC10']
        self.assertTrue(all(c['reason'] == 'E_PLAN_BUDGET_EXCEEDED' for c in sc10))

    def test_estimate_track_not_falsly_failing(self):
        """估计轨扰动（0.05<容差）不得假失败——正例场景两轨同过。"""
        for scenario_id in ('SC01', 'SC02', 'SC03', 'SC08'):
            truth = [c for c in self.result['cells']
                     if c['scenario'] == scenario_id and c['track'] == 'truth']
            estimate = [c for c in self.result['cells']
                        if c['scenario'] == scenario_id and c['track'] == 'estimate']
            self.assertEqual({c['outcome'] for c in truth}, {'pass'})
            self.assertEqual({c['outcome'] for c in estimate}, {'pass'})

    def test_lock_digest_traceable(self):
        digests = {c['lock_digest'] for c in self.result['cells']}
        self.assertEqual(len(digests), 4)  # 组合四款锁互异且全部在矩阵中
        combos = {c['combo'] for c in self.result['cells']}
        self.assertEqual(combos, set(cal.FROZEN['denominators']['combos']['value']))

    def test_failure_injection_localizes(self):
        """注入：目标面外（map 有效但 goal 无支撑）→ plan 阶段 E_PLAN_GOAL_INVALID。"""
        case = scn.scenario('SC01')
        cell = reg._run_cell(case, 'lite3+dwa', 'deadbeef0000', 'truth', 'official')
        # 正常应 pass；注入坏 goal
        case_bad = dict(case)
        case_bad['goal'] = (50.0, 50.0, 0.0)
        cell_bad = reg._run_cell(case_bad, 'lite3+dwa', 'deadbeef0000', 'truth', 'official')
        self.assertEqual(cell_bad['stage'], 'plan')
        self.assertEqual(cell_bad['reason'], 'E_PLAN_GOAL_INVALID')
        self.assertEqual(cell_bad['outcome'], 'unexpected_failure')
        self.assertEqual(cell['outcome'], 'pass')

    def test_cli(self):
        import json
        import subprocess
        result = subprocess.run([sys.executable, '-B', str(HERE / 'regression.py'), 'run'],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0)
        summary = json.loads(result.stdout.split('\n')[0])
        self.assertEqual(summary['unexpected_failures'], 0)
        result = subprocess.run([sys.executable, '-B', str(HERE / 'regression.py')],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 9)


if __name__ == '__main__':
    unittest.main(verbosity=2)
