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
        """v2 分母（14 场景×4 组合×2 轨=112 cell）：pass=11 场景×8；预期失败=SC07/09/10×8。"""
        s = self.result['summary']
        self.assertEqual(s['cells'], 14 * 4 * 2)
        self.assertEqual(s['passed'], 11 * 4 * 2)
        self.assertEqual(s['expected_failures'], 3 * 4 * 2)
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

    def test_sc09_clearance_expected_at_eval(self):
        """SC09：低顶下穿＝eval 期预期失败（E_TRAJ_CLEARANCE），plan 期正常出径。"""
        sc09 = [c for c in self.result['cells'] if c['scenario'] == 'SC09']
        self.assertEqual(len(sc09), 8)
        self.assertTrue(all(c['stage'] == 'eval' and c['outcome'] == 'expected_failure'
                            and 'E_TRAJ_CLEARANCE' in c['reason'] for c in sc09))

    def test_non_planning_kinds_pass(self):
        """SC11（跟踪撤销）/SC12（多任务隔离）/SC13（版本漂移）/SC14（子矩阵）全过。"""
        for sid in ('SC11', 'SC12', 'SC13', 'SC14'):
            cells = [c for c in self.result['cells'] if c['scenario'] == sid]
            self.assertEqual(len(cells), 8, sid)
            self.assertEqual({c['outcome'] for c in cells}, {'pass'}, sid)
            self.assertEqual({c['kind'] for c in cells}, {cells[0]['kind']}, sid)

    def test_sc05_bridge_solved_by_support_graph(self):
        """SC05 同高天桥：支撑图原生可解（非单调剖面，对照 mode3 不可表达）。"""
        sc05 = [c for c in self.result['cells'] if c['scenario'] == 'SC05']
        self.assertEqual({c['outcome'] for c in sc05}, {'pass'})

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
