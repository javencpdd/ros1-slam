"""V8-07 次切片测试：损失声明四要素／未声明拒绝／组合矩阵 4×全链过筛／负例。

矩阵断言：4 组合全过（compose＋wiring＋losses＋lock）、摘要互异、
双控制器负例按 E_COMP_DUP_ROLE 拒绝。
"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'composer'))
import composer as comp  # noqa: E402
import declarations as decl  # noqa: E402

MDIR = HERE.parent / 'registry' / 'data' / 'manifests'


class DeclarationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.store = comp.load(MDIR)

    def test_losses_grounded(self):
        report = decl.losses(self.store, decl.BASE)
        by_comp = {r['component']: r['losses'] for r in report}
        # scan 的两条损失回指 V8-03 实证
        scan = by_comp['v8.local_planner.scan_planner']
        self.assertEqual(len(scan), 2)
        self.assertTrue(all('probe_mode3' in l['evidence'] or 'R63' in l['evidence'] for l in scan))
        # 全局规划器与控制器的维度损失在位；定位双后端声明无损（hdl 另行调用声明）
        self.assertEqual(len(by_comp['v8.global_planner.apt']), 1)
        self.assertEqual(by_comp['v8.global_localization.lite3'], [])
        hdl = decl.losses(self.store, ['v8.global_localization.hdl'])
        self.assertEqual(hdl[0]['losses'], [])
        # 四要素齐（losses() 已校验，此处抽验证据非空）
        for records in by_comp.values():
            for record in records:
                self.assertTrue(record['evidence'].strip())

    def test_undeclared_rejected(self):
        # 新组件无声明＝拒绝（临时摘除声明验证，finally 恢复）
        saved = decl.ADAPTER_LOSSES.pop('v8.odometry.lite3_slam')
        try:
            with self.assertRaises(Exception) as caught:
                decl.losses(self.store, ['v8.odometry.lite3_slam'])
            self.assertEqual(getattr(caught.exception, 'code', ''), 'E_ADAPTER_UNDECLARED')
        finally:
            decl.ADAPTER_LOSSES['v8.odometry.lite3_slam'] = saved

    def test_matrix_all_pass_and_distinct(self):
        result = decl.matrix(self.store)
        self.assertEqual(len(result['combos']), 4)
        self.assertTrue(all(c['ok'] for c in result['combos']), result['combos'])
        digests = {c['digest'] for c in result['combos']}
        self.assertEqual(len(digests), 4)  # 组合间摘要互异（换后端可区分）
        for combo in result['combos']:
            self.assertEqual(combo['wires'], 13)       # 八件套接线数固定
            self.assertEqual(combo['loss_records'], 5)  # scan 2＋global_planner 1＋costmap 1＋controller 1
        self.assertEqual(result['negative_dual_controller'],
                         {'ok': False, 'reason': 'E_COMP_DUP_ROLE'})

    def test_cli(self):
        import json
        import subprocess
        result = subprocess.run([sys.executable, '-B', str(HERE / 'declarations.py'),
                                 'matrix', str(MDIR)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0)
        verdict = json.loads(result.stdout)
        self.assertEqual(verdict['ok_count'], 4)
        result = subprocess.run([sys.executable, '-B', str(HERE / 'declarations.py')],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 9)


if __name__ == '__main__':
    unittest.main(verbosity=2)
