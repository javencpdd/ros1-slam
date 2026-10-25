"""V8-12 测试：无待定关键值铁则、集合分离、封版 digest、负例注入、定标集真实可跑。"""
import copy
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'scenarios'))
sys.path.insert(0, str(HERE.parent / 'contracts'))
sys.path.insert(0, str(HERE.parent / 'planner'))
import calibration as cal  # noqa: E402
import scenarios as scn  # noqa: E402
import support_planner as pln  # noqa: E402


class CalibrationTests(unittest.TestCase):
    def test_frozen_valid_and_digest_stable(self):
        self.assertTrue(cal.validate_frozen())
        self.assertEqual(len(cal.digest()), 64)
        self.assertEqual(cal.digest(), cal.digest())  # 确定性

    def test_pending_value_rejected(self):
        bad = copy.deepcopy(cal.FROZEN)
        bad['tolerances']['tracking_lateral_m'] = {'value': 'TODO', 'evidence': 'x'}
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.validate_frozen(bad)
        self.assertEqual(caught.exception.code, 'E_CAL_PENDING')

    def test_missing_evidence_rejected(self):
        bad = copy.deepcopy(cal.FROZEN)
        bad['budgets']['planner_max_expansions'] = {'value': 100}
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.validate_frozen(bad)
        self.assertEqual(caught.exception.code, 'E_CAL_PENDING')

    def test_unknown_scenario_rejected(self):
        bad = copy.deepcopy(cal.FROZEN)
        bad['denominators']['official_scenarios']['value'] = ['SC99']
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.validate_frozen(bad)
        self.assertEqual(caught.exception.code, 'E_CAL_UNKNOWN_SCENARIO')

    def test_all_scenarios_in_denominator(self):
        """v2 解封后：正式分母＝全 14 场景（构造器齐，R661）。"""
        official = set(cal.FROZEN['denominators']['official_scenarios']['value'])
        self.assertEqual(official, set(scn.SCENARIOS))
        self.assertEqual(cal.FROZEN['calibration_version'], 2)

    def test_separation_guard(self):
        records = [{'run_class': 'calibration', 'calibration_only': True},
                   {'run_class': 'official'},
                   {'run_class': 'official'}]
        summary = cal.separation_guard(records)
        self.assertEqual(summary, {'official_count': 2, 'calibration_count': 1})
        leaked = [{'run_class': 'official', 'calibration_only': True}]
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.separation_guard(leaked)
        self.assertEqual(caught.exception.code, 'E_CAL_LEAK')

    def test_calibration_set_runs(self):
        """定标集两场景真实可跑（正向＋单向正例）——封版不只纸面。"""
        for name in cal.FROZEN['denominators']['calibration_scenarios']['value']:
            case = scn.scenario(name)
            self.assertEqual(case['kind'], 'planning')
            if case['truth']['route_exists']:
                route = pln.plan(case['map'], case['start'], case['goal'])
                self.assertTrue(route['ok'], name)
            else:
                with self.assertRaises(pln.PlannerError):
                    pln.plan(case['map'], case['start'], case['goal'])

    def test_cli(self):
        import json
        import subprocess
        result = subprocess.run([sys.executable, '-B', str(HERE / 'calibration.py'), 'freeze'],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0)
        self.assertTrue(json.loads(result.stdout)['ok'])
        result = subprocess.run([sys.executable, '-B', str(HERE / 'calibration.py')],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 9)


if __name__ == '__main__':
    unittest.main(verbosity=2)
