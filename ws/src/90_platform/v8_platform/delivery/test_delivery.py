"""V8-14 测试：规模曲线（四档有界、单调合理、确定性）、复现清单（覆盖九模块、
再生成一致、含本件）、未接入说明在册。"""
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import delivery as dl  # noqa: E402


class DeliveryTests(unittest.TestCase):
    def test_scale_curve_bounded_and_growing(self):
        curve = dl.scale_curve()
        self.assertEqual([c['cells_per_side'] for c in curve], [10, 20, 40, 80])
        # 展开随规模增长（性能曲线有形状）且全在有界断言内（scale_curve 自带界检查）
        self.assertLess(curve[0]['expansions'], curve[1]['expansions'])
        self.assertLess(curve[1]['expansions'], curve[2]['expansions'])
        self.assertLess(curve[2]['expansions'], curve[3]['expansions'])
        # 路径长随规模增长
        lengths = [c['path_length'] for c in curve]
        self.assertEqual(lengths, sorted(lengths))

    def test_scale_curve_deterministic(self):
        self.assertEqual(dl.scale_curve(sizes=(20,)), dl.scale_curve(sizes=(20,)))

    def test_manifest_covers_modules(self):
        entries = dl.manifest()
        modules = {p.split('/')[0] for p in entries}
        for required in ('contracts', 'registry', 'composer', 'orchestrator', 'adapters',
                         'representation', 'planner', 'validator', 'scenarios',
                         'calibration', 'regression'):
            self.assertIn(required, modules, required)
        # 复现性自证：再生成逐字节一致
        self.assertEqual(entries, dl.manifest())
        # 关键件在清单
        self.assertIn('planner/support_planner.py', entries)
        self.assertIn('calibration/calibration.py', entries)

    def test_unplugged_statement_on_file(self):
        """未接入说明在用户指南在册（apt 后端/SCAN 运行面未实装）。"""
        guide = (HERE.parent / 'README.md').read_text()
        self.assertIn('未接入', guide)
        self.assertIn('apt', guide)

    def test_cli(self):
        import subprocess
        result = subprocess.run([sys.executable, '-B', str(HERE / 'delivery.py'), 'curve'],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0)
        self.assertEqual(len(json.loads(result.stdout)), 4)
        result = subprocess.run([sys.executable, '-B', str(HERE / 'delivery.py'), 'manifest'],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0)
        self.assertGreater(json.loads(result.stdout)['files'], 40)
        result = subprocess.run([sys.executable, '-B', str(HERE / 'delivery.py')],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 9)


if __name__ == '__main__':
    unittest.main(verbosity=2)
