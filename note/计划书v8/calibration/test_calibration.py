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

    # ------------------------------------------------------------------
    # 整改（2026-10-27，独立复核 P0）：fail-closed / 放宽守卫 / golden digest
    # ------------------------------------------------------------------
    def test_missing_key_rejected(self):
        bad = copy.deepcopy(cal.FROZEN)
        del bad['tolerances']['goal_position_m']
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.validate_frozen(bad)
        self.assertEqual(caught.exception.code, 'E_CAL_PENDING')
        self.assertIn('goal_position_m', caught.exception.detail)

    def test_missing_section_rejected(self):
        bad = copy.deepcopy(cal.FROZEN)
        del bad['tolerances']
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.validate_frozen(bad)
        self.assertEqual(caught.exception.code, 'E_CAL_PENDING')
        self.assertIn('tolerances', caught.exception.detail)

    def test_empty_evidence_rejected(self):
        bad = copy.deepcopy(cal.FROZEN)
        bad['tolerances']['tracking_lateral_m']['evidence'] = '   '
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.validate_frozen(bad)
        self.assertEqual(caught.exception.code, 'E_CAL_PENDING')

    def test_none_type_and_empty_table_rejected(self):
        cases = {
            'none_value': lambda c: c['budgets']['task_budget_s'].__setitem__('value', None),
            'wrong_type': lambda c: c['tolerances']['tracking_lateral_m'].__setitem__('value', '0.15'),
            'empty_table': lambda c: c['denominators']['combos'].__setitem__('value', []),
            'todo_value': lambda c: c['tolerances']['tracking_lateral_m'].__setitem__('value', 'TODO'),
        }
        for label, mutate in cases.items():
            with self.subTest(label=label):
                bad = copy.deepcopy(cal.FROZEN)
                mutate(bad)
                with self.assertRaises(cal.CalibrationError) as caught:
                    cal.validate_frozen(bad)
                self.assertEqual(caught.exception.code, 'E_CAL_PENDING')

    def test_loosened_without_unfreeze_rejected(self):
        previous = copy.deepcopy(cal.FROZEN)
        current = copy.deepcopy(cal.FROZEN)
        current['tolerances']['tracking_lateral_m']['value'] = 0.30
        current['tolerances']['min_clearance_m']['value'] = 0.10
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.check_loosening(previous, current)
        self.assertEqual(caught.exception.code, 'E_CAL_LOOSENED')
        self.assertIn('tracking_lateral_m', caught.exception.detail)
        self.assertIn('min_clearance_m', caught.exception.detail)

    def test_loosened_with_unfreeze_passes(self):
        previous = copy.deepcopy(cal.FROZEN)
        current = copy.deepcopy(cal.FROZEN)
        current['calibration_version'] = 3
        current['tolerances']['tracking_lateral_m']['value'] = 0.30
        current['tolerances']['min_clearance_m']['value'] = 0.10
        current['unfreeze_log'] = {
            'to_version': 3, 'when': '2026-10-27', 'from': 2, 'to': 3,
            'change': '临时放宽失败样本以复跑',
            'reason': '复跑定位需要，显式解封',
        }
        loosened = cal.check_loosening(previous, current)
        self.assertEqual(sorted(loosened),
                         ['tolerances.min_clearance_m.value',
                          'tolerances.tracking_lateral_m.value'])

    def test_loosened_with_empty_reason_rejected(self):
        previous = copy.deepcopy(cal.FROZEN)
        current = copy.deepcopy(cal.FROZEN)
        current['tolerances']['tracking_lateral_m']['value'] = 0.30
        current['unfreeze_log'] = {'from': 0.15, 'to': 0.30, 'reason': '  '}
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.check_loosening(previous, current)
        self.assertEqual(caught.exception.code, 'E_CAL_LOOSENED')

    def test_digest_golden_rejects_value_change(self):
        bad = copy.deepcopy(cal.FROZEN)
        bad['tolerances']['tracking_lateral_m']['value'] = 0.99
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.assert_digest(bad)
        self.assertEqual(caught.exception.code, 'E_CAL_DIGEST')
        self.assertEqual(cal.assert_digest(), cal.DIGEST)
        self.assertEqual(len(cal.DIGEST), 64)

    def test_pending_report_not_green(self):
        report = cal.validate_frozen()
        self.assertFalse(report['ok'])
        self.assertTrue(report['blocked'])
        self.assertEqual(report['status'], 'blocked')
        self.assertFalse(report['freeze_allowed'])
        pending = report['pending']
        self.assertTrue(pending)
        for token in ('sampling.scenario_repetitions', 'sampling.perturbation_seeds',
                      'latency.p95_global_s', 'latency.p95_local_s',
                      'budgets.cancel_time_s', 'budgets.stop_time_s',
                      'budgets.stop_distance_m', 'tolerances.max_localization_error_m',
                      'resources.max_cpu_percent', 'success_rate.min_pass_rate'):
            self.assertTrue(any(token in key for key in pending), token)
        reasons = '; '.join(item['reason'] for item in report['pending_detail'])
        self.assertIn('4.1', reasons)


if __name__ == '__main__':
    unittest.main(verbosity=2)
