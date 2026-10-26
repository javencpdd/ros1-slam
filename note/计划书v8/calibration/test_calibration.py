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
        self.assertEqual(len(pending), 5)
        for token in ('latency.p95_local_s', 'budgets.stop_time_s',
                      'budgets.stop_distance_m', 'tolerances.max_localization_error_m',
                      'success_rate.min_pass_rate'):
            self.assertTrue(any(token in key for key in pending), token)
        # 2026-10-27 判据面已落实的项不得再 pending（资源/采样/两个带域字段）。
        for token in ('resources.max_cpu_percent', 'resources.max_memory_mb',
                      'sampling.scenario_repetitions', 'sampling.perturbation_seeds',
                      'latency.p95_global_s', 'budgets.cancel_time_s'):
            self.assertFalse(any(token in key for key in pending), token)
        reasons = '; '.join(item['reason'] for item in report['pending_detail'])
        self.assertIn('4.1', reasons)

    # ------------------------------------------------------------------
    # 判据面裁定（2026-10-27，小美，R713）：resources 两项＋sampling 两项落实。
    # 判据面裁定（2026-10-27，小美，R715）：p95_global 与 cancel_time 带域冻结；
    # p95_local、stop_time、stop_distance、定位误差、成功率保持 pending。
    # ------------------------------------------------------------------
    def test_resources_and_sampling_frozen_shape(self):
        cfg = cal.FROZEN
        cpu = cfg['resources']['max_cpu_percent']['value']
        mem = cfg['resources']['max_memory_mb']['value']
        self.assertIsInstance(cpu, (int, float))
        self.assertNotIsInstance(cpu, bool)
        self.assertTrue(0 < cpu <= 100)
        self.assertIsInstance(mem, int)
        self.assertNotIsInstance(mem, bool)
        self.assertGreater(mem, 0)
        self.assertLessEqual(mem, 8 * 1024)  # 运行容器 8 GiB 上限内（R625/R660）
        for key in ('max_cpu_percent', 'max_memory_mb'):
            evidence = cfg['resources'][key]['evidence']
            self.assertIn('开发机采样', evidence)
            self.assertIn('非正式定标', evidence)
        self.assertEqual(cfg['sampling']['scenario_repetitions']['value'], 3)
        seeds = cfg['sampling']['perturbation_seeds']['value']
        self.assertEqual(seeds, ['0', '1', '2'])
        self.assertEqual(len(seeds), len(set(seeds)))
        self.assertTrue(all(isinstance(seed, str) for seed in seeds))
        # unfreeze 记录须带守卫期望的 key/from/to/reason。
        entries = {entry.get('key'): entry for entry in cfg['unfreeze_log']
                   if isinstance(entry, dict) and entry.get('key')}
        repetitions = entries['sampling.scenario_repetitions.value']
        self.assertEqual((repetitions['from'], repetitions['to']), (1, 3))
        perturbation = entries['sampling.perturbation_seeds.value']
        self.assertEqual(perturbation['from'], [])
        self.assertEqual(perturbation['to'], ['0', '1', '2'])
        for entry in (repetitions, perturbation):
            self.assertTrue(entry['reason'].strip())
        report = cal.validate_frozen()
        self.assertFalse(any(key.startswith(('resources.', 'sampling.'))
                             for key in report['pending']))

    def test_repetitions_revert_caught(self):
        """把 repetitions 从 3 改回 1：golden digest 必红且 pending 复现。"""
        bad = copy.deepcopy(cal.FROZEN)
        bad['sampling']['scenario_repetitions']['value'] = 1
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.assert_digest(bad)
        self.assertEqual(caught.exception.code, 'E_CAL_DIGEST')
        report = cal.validate_frozen(bad)
        self.assertIn('sampling.scenario_repetitions.value', report['pending'])
        self.assertFalse(report['freeze_allowed'])
        # 回退属放宽：无 3→1 的 unfreeze from/to（现有记录是 1→3），守卫须拒。
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.check_loosening(cal.FROZEN, bad)
        self.assertEqual(caught.exception.code, 'E_CAL_LOOSENED')

    def test_resources_section_required(self):
        bad = copy.deepcopy(cal.FROZEN)
        del bad['resources']
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.validate_frozen(bad)
        self.assertEqual(caught.exception.code, 'E_CAL_PENDING')
        self.assertIn('resources', caught.exception.detail)

    def test_resource_cap_raise_requires_unfreeze(self):
        previous = copy.deepcopy(cal.FROZEN)
        current = copy.deepcopy(cal.FROZEN)
        current['resources']['max_cpu_percent']['value'] = 60.0
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.check_loosening(previous, current)
        self.assertEqual(caught.exception.code, 'E_CAL_LOOSENED')
        self.assertIn('max_cpu_percent', caught.exception.detail)

    # ------------------------------------------------------------------
    # 带域冻结（2026-10-27，小美，R715）：p95_global=0.1394 s
    # （scope=planner_service_only, n=9）、cancel_time=0.0372 s
    # （scope=mux_hold_only, n=3）；域外不得冒用，放宽须显式解封。
    # ------------------------------------------------------------------
    def test_scoped_fields_frozen_with_scope_sample_evidence(self):
        p95 = cal.FROZEN['latency']['p95_global_s']
        cancel = cal.FROZEN['budgets']['cancel_time_s']
        self.assertEqual(p95['value'], 0.1394)
        self.assertEqual(p95['scope'], 'planner_service_only')
        self.assertEqual(p95['sample_n'], 9)
        self.assertEqual(cancel['value'], 0.0372)
        self.assertEqual(cancel['scope'], 'mux_hold_only')
        self.assertEqual(cancel['sample_n'], 3)
        for entry, scope in ((p95, 'planner_service_only'),
                             (cancel, 'mux_hold_only')):
            self.assertTrue(entry['evidence'].strip())
            self.assertIn('R715', entry['evidence'])
            self.assertIn(scope, entry['evidence'])
            self.assertGreaterEqual(entry['sample_n'], 3)
        report = cal.validate_frozen()
        for key in ('latency.p95_global_s', 'budgets.cancel_time_s'):
            self.assertNotIn(key, report['pending'])
        # 带域冻结不改变总体可冻结性：尚有 5 项系统级读数缺失。
        self.assertFalse(report['freeze_allowed'])

    def test_scoped_field_metadata_required(self):
        cases = {
            'missing_scope': lambda c: c['latency']['p95_global_s'].pop('scope'),
            'missing_sample_n': lambda c: c['latency']['p95_global_s'].pop('sample_n'),
            'missing_evidence': lambda c: c['budgets']['cancel_time_s'].pop('evidence'),
            'wrong_scope': lambda c: c['latency']['p95_global_s'].__setitem__(
                'scope', 'full_stack'),
            'low_sample_n': lambda c: c['budgets']['cancel_time_s'].__setitem__(
                'sample_n', 1),
        }
        for label, mutate in cases.items():
            with self.subTest(label=label):
                bad = copy.deepcopy(cal.FROZEN)
                mutate(bad)
                with self.assertRaises(cal.CalibrationError) as caught:
                    cal.validate_frozen(bad)
                self.assertEqual(caught.exception.code, 'E_CAL_PENDING')

    def test_p95_global_raise_blocked_without_unfreeze(self):
        previous = copy.deepcopy(cal.FROZEN)
        current = copy.deepcopy(cal.FROZEN)
        current['latency']['p95_global_s']['value'] = 0.20
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.check_loosening(previous, current)
        self.assertEqual(caught.exception.code, 'E_CAL_LOOSENED')
        self.assertIn('latency.p95_global_s.value', caught.exception.detail)

    def test_p95_global_raise_with_scoped_unfreeze_passes(self):
        previous = copy.deepcopy(cal.FROZEN)
        current = copy.deepcopy(cal.FROZEN)
        current['latency']['p95_global_s']['value'] = 0.20
        current['unfreeze_log'] = list(current['unfreeze_log']) + [{
            'when': '2026-10-27', 'key': 'latency.p95_global_s.value',
            'from': 0.1394, 'to': 0.20,
            'reason': 'scope=planner_service_only 实测漂移，显式解封放宽'}]
        self.assertEqual(cal.check_loosening(previous, current),
                         ['latency.p95_global_s.value'])

    def test_cancel_time_lower_requires_unfreeze(self):
        previous = copy.deepcopy(cal.FROZEN)
        current = copy.deepcopy(cal.FROZEN)
        current['budgets']['cancel_time_s']['value'] = 0.02
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.check_loosening(previous, current)
        self.assertEqual(caught.exception.code, 'E_CAL_LOOSENED')
        self.assertIn('budgets.cancel_time_s.value', caught.exception.detail)


if __name__ == '__main__':
    unittest.main(verbosity=2)
