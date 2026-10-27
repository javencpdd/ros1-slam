"""V8-12 测试：无待定关键值铁则、集合分离、封版 digest、负例注入、定标集真实可跑。"""
import copy
import json
import os
import sys
import tempfile
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
        report = cal.validate_frozen()
        self.assertIsInstance(report, dict)
        self.assertIn(report['status'], ('blocked', 'pass'))
        self.assertEqual(report['digest'], cal.DIGEST)
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
        self.assertEqual(len(pending), 3)
        for token in ('budgets.stop_time_s', 'budgets.stop_distance_m',
                      'success_rate.min_pass_rate'):
            self.assertTrue(any(token in key for key in pending), token)
        # 2026-10-27 判据面已落实的项不得再 pending（资源/采样/两个带域字段/制动分量）。
        for token in ('resources.max_cpu_percent', 'resources.max_memory_mb',
                      'sampling.scenario_repetitions', 'sampling.perturbation_seeds',
                      'latency.p95_global_s', 'latency.p95_local_s', 'budgets.cancel_time_s',
                      'budgets.stop_brake_peak_distance_m', 'budgets.stop_brake_peak_time_s',
                      'budgets.stop_chain_osmesa_time_s', 'budgets.stop_chain_osmesa_distance_m'):
            self.assertFalse(any(token in key for key in pending), token)
        reasons = '; '.join(item['reason'] for item in report['pending_detail'])
        self.assertIn('4.1', reasons)

    # ------------------------------------------------------------------
    # 判据面裁定（2026-10-27，小美，R713）：resources 两项＋sampling 两项落实。
    # 判据面裁定（2026-10-27，小美，R715）：p95_global 与 cancel_time 带域冻结；
    # stop_time、stop_distance、成功率保持 pending（p95_local 已在 R722 带域冻结；
    # 定位误差已在 R787 按 B 方案带域冻结）。
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # 判据面裁定（2026-10-27，小美，R720）：制动分量另立两键；
    # 全链 stop_time/stop_distance 继续 pending。
    # ------------------------------------------------------------------
    def test_stop_brake_components_frozen_scoped(self):
        b = cal.FROZEN['budgets']
        d = b['stop_brake_peak_distance_m']
        t = b['stop_brake_peak_time_s']
        self.assertEqual(d['scope'], 'lite3_mj_dyn_hold_peak_0p3_flat_wall_render')
        self.assertEqual(t['scope'], d['scope'])
        self.assertEqual(d['sample_n'], 9)
        self.assertEqual(t['sample_n'], 9)
        self.assertAlmostEqual(d['value'], 0.124)
        self.assertAlmostEqual(t['value'], 1.17)
        self.assertIn('budgets.stop_brake_peak_distance_m', cal.SCOPED_REQUIRED)
        self.assertIn('budgets.stop_brake_peak_time_s', cal.SCOPED_REQUIRED)
        report = cal.validate_frozen()
        self.assertEqual(report['status'], 'blocked')
        self.assertFalse(report['freeze_allowed'])

    def test_stop_brake_component_scope_and_sample_guard(self):
        bad = copy.deepcopy(cal.FROZEN)
        bad['budgets']['stop_brake_peak_distance_m']['scope'] = 'full_stack'
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.validate_frozen(bad)
        self.assertEqual(caught.exception.code, 'E_CAL_PENDING')
        bad = copy.deepcopy(cal.FROZEN)
        bad['budgets']['stop_brake_peak_time_s']['sample_n'] = 2
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.validate_frozen(bad)
        self.assertEqual(caught.exception.code, 'E_CAL_PENDING')

    def test_stop_full_chain_keys_stay_pending_r720(self):
        report = cal.validate_frozen()
        keys = [item['key'] for item in report['pending_detail']]
        self.assertIn('budgets.stop_time_s.value', keys)
        self.assertIn('budgets.stop_distance_m.value', keys)
        self.assertNotIn('budgets.stop_brake_peak_time_s.value', keys)
        self.assertNotIn('budgets.stop_brake_peak_distance_m.value', keys)
        self.assertNotIn('budgets.stop_chain_osmesa_time_s.value', keys)
        self.assertNotIn('budgets.stop_chain_osmesa_distance_m.value', keys)

    def test_p95_local_frozen_scoped_r722(self):
        item = cal.FROZEN['latency']['p95_local_s']
        self.assertEqual(item['scope'], 'scan_sim_consumed_cloud_to_bspline_27run')
        self.assertEqual(item['sample_n'], 470)
        self.assertAlmostEqual(item['value'], 0.8966)
        self.assertIn('latency.p95_local_s', cal.SCOPED_REQUIRED)
        self.assertEqual(cal.SCOPED_REQUIRED['latency.p95_local_s']['scope'],
                         'scan_sim_consumed_cloud_to_bspline_27run')
        report = cal.validate_frozen()
        self.assertEqual(report['status'], 'blocked')
        self.assertEqual(len(report['pending']), 3)
        keys = [x['key'] for x in report['pending_detail']]
        self.assertNotIn('latency.p95_local_s.value', keys)

    def test_p95_local_scope_and_sample_guard(self):
        bad = copy.deepcopy(cal.FROZEN)
        bad['latency']['p95_local_s']['scope'] = 'full_stack'
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.validate_frozen(bad)
        self.assertEqual(caught.exception.code, 'E_CAL_PENDING')
        bad = copy.deepcopy(cal.FROZEN)
        bad['latency']['p95_local_s']['sample_n'] = 2
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.validate_frozen(bad)
        self.assertEqual(caught.exception.code, 'E_CAL_PENDING')

    def test_stop_chain_osmesa_proxies_scoped_r726(self):
        b = cal.FROZEN['budgets']
        t = b['stop_chain_osmesa_time_s']
        d = b['stop_chain_osmesa_distance_m']
        self.assertAlmostEqual(t['value'], 2.471)
        self.assertAlmostEqual(d['value'], 0.3306)
        self.assertEqual(t['scope'], 'lite3_mj_depth_osmesa_perception_to_rest_flat_0p3_3x3')
        self.assertEqual(d['scope'], 'lite3_mj_depth_osmesa_perception_to_peak_flat_0p3_3x3')
        self.assertEqual(t['sample_n'], 9)
        self.assertEqual(d['sample_n'], 9)
        self.assertIn('budgets.stop_chain_osmesa_time_s', cal.SCOPED_REQUIRED)
        self.assertIn('budgets.stop_chain_osmesa_distance_m', cal.SCOPED_REQUIRED)
        report = cal.validate_frozen()
        self.assertEqual(len(report['pending']), 3)
        keys = [x['key'] for x in report['pending_detail']]
        self.assertIn('budgets.stop_time_s.value', keys)
        self.assertIn('budgets.stop_distance_m.value', keys)

    def test_stop_chain_osmesa_scope_guard(self):
        bad = copy.deepcopy(cal.FROZEN)
        bad['budgets']['stop_chain_osmesa_time_s']['scope'] = 'full_chain'
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.validate_frozen(bad)
        self.assertEqual(caught.exception.code, 'E_CAL_PENDING')

    def test_stop_chain_osmesa_distance_guard(self):
        bad = copy.deepcopy(cal.FROZEN)
        bad['budgets']['stop_chain_osmesa_distance_m']['scope'] = 'full_chain'
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.validate_frozen(bad)
        self.assertEqual(caught.exception.code, 'E_CAL_PENDING')
        bad = copy.deepcopy(cal.FROZEN)
        bad['budgets']['stop_chain_osmesa_distance_m']['sample_n'] = 2
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.validate_frozen(bad)
        self.assertEqual(caught.exception.code, 'E_CAL_PENDING')

    def test_budget_raise_not_flagged_by_loosening(self):
        prev = copy.deepcopy(cal.FROZEN)
        prev['budgets']['stop_brake_peak_distance_m']['value'] = 0.076
        self.assertEqual(cal.check_loosening(prev, cal.FROZEN), [])  # 已知盲区：budgets 仅识别下调；靠独立复核

    def test_stop_component_raise_logged(self):
        log = cal.FROZEN['unfreeze_log']
        found = [e for e in log if e.get('key') == 'budgets.stop_brake_peak_distance_m.value'
                 and e.get('from') == 0.076 and e.get('to') == 0.124]
        self.assertEqual(len(found), 1)
        found = [e for e in log if e.get('key') == 'budgets.stop_brake_peak_time_s.value'
                 and e.get('from') == 0.387 and e.get('to') == 1.17]
        self.assertEqual(len(found), 1)

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

    # ------------------------------------------------------------------
    # 判据面裁定（2026-10-27，小美，R716）：全链停车读数 0.1265 m／108.4 ms
    # 为 n=1，且载体是 kinematic planar 底盘、无制动模型；stop 两项**不解冻**，
    # 保持 pending；作废首轮读数（0.3568 m／1648.5 ms）不得入配置。
    # ------------------------------------------------------------------
    def test_stop_fields_stay_pending_r716(self):
        report = cal.validate_frozen()
        pending = report['pending']
        for key in ('budgets.stop_time_s.value', 'budgets.stop_distance_m.value'):
            self.assertIn(key, pending)
            self.assertTrue(any(item['key'] == key and item['reason'].strip()
                                for item in report['pending_detail']), key)
        self.assertEqual(report['status'], 'blocked')
        self.assertFalse(report['freeze_allowed'])
        # 保持 pending ⇒ FROZEN 中不得出现这两键，也不许先写半套 scope/sample_n。
        for name in ('stop_time_s', 'stop_distance_m'):
            self.assertNotIn(name, cal.FROZEN['budgets'])
        # 守卫不得为收下 n=1 而放宽：两项不在带域表，全表样本下限仍为 3。
        for key in ('budgets.stop_time_s', 'budgets.stop_distance_m'):
            self.assertNotIn(key, cal.SCOPED_REQUIRED)
        self.assertTrue(all(rule['min_sample_n'] >= 3
                            for rule in cal.SCOPED_REQUIRED.values()))
        # 本次读数与作废首轮读数都不得写进冻结配置。
        text = repr(cal.FROZEN)
        for token in ('0.3568', '1648.5', '0.1265', '108.4'):
            self.assertNotIn(token, text)
        self.assertEqual(cal.assert_digest(), cal.DIGEST)
    # ------------------------------------------------------------------
    # 判据面裁定（2026-10-27，小美，R748）：成功率是比例门槛；裸 value 不得
    # 离开 pending，必须有 scope/sample_n/method/alpha/denominator/aggregation/
    # evidence，且 36-run 下限与 CP95 方法不得缺省；下调按放宽处理。
    # ------------------------------------------------------------------
    def _success_rate_node(self, **overrides):
        node = {
            'value': 0.90,
            'scope': 'v8_official_positive_sc01_sc02_sc09_4group_3rep_n36_truth_mux_per_combo_ref_lite3_dwa',
            'sample_n': 36,
            'method': 'clopper_pearson_lower_95',
            'alpha': 0.05,
            'denominator': 'N_valid_positive',
            'aggregation': 'per_config_per_track',
            'evidence': '36/36 valid positive runs; CP one-sided 0.9202, two-sided 0.9026',
        }
        node.update(overrides)
        return {'min_pass_rate': node}

    def test_success_rate_bare_value_stays_pending_r748(self):
        bad = copy.deepcopy(cal.FROZEN)
        bad['success_rate'] = {'min_pass_rate': {'value': 0.90}}
        report = cal.validate_frozen(bad)
        self.assertEqual(report['status'], 'blocked')
        self.assertIn('success_rate.min_pass_rate', report['pending'])
        self.assertTrue(any('缺元数据' in item['reason']
                            for item in report['pending_detail']
                            if item['key'] == 'success_rate.min_pass_rate'))

    def test_success_rate_metadata_complete_only_other_three_pending_r748(self):
        good = copy.deepcopy(cal.FROZEN)
        good['success_rate'] = self._success_rate_node()
        report = cal.validate_frozen(good)
        self.assertNotIn('success_rate.min_pass_rate', report['pending'])
        self.assertEqual(report['status'], 'blocked')
        self.assertEqual(len(report['pending']), 2)
        self.assertEqual(cal.assert_digest(), cal.DIGEST)

    def test_success_rate_sample_and_method_guard_r748(self):
        bad = copy.deepcopy(cal.FROZEN)
        bad['success_rate'] = self._success_rate_node(sample_n=18,
                                                      method='max_times_1.15')
        report = cal.validate_frozen(bad)
        self.assertIn('success_rate.min_pass_rate.sample_n', report['pending'])
        self.assertIn('success_rate.min_pass_rate.method', report['pending'])

    def test_success_rate_lower_requires_unfreeze_r748(self):
        previous = copy.deepcopy(cal.FROZEN)
        current = copy.deepcopy(cal.FROZEN)
        previous['success_rate'] = self._success_rate_node(value=0.90)
        current['success_rate'] = self._success_rate_node(value=0.80)
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.check_loosening(previous, current)
        self.assertEqual(caught.exception.code, 'E_CAL_LOOSENED')
        self.assertIn('success_rate.min_pass_rate.value', caught.exception.detail)

    # ------------------------------------------------------------------
    # R750 独立复核（R751）查出的 P1 缺口：值域、元数据类型、scope 精确匹配
    # 与 alpha 边界；R752 补齐并锁测试。
    # ------------------------------------------------------------------
    def test_success_rate_value_domain_r752(self):
        for bad_value in (0, -0.5, 1.5, float('nan'), float('inf')):
            cfg = copy.deepcopy(cal.FROZEN)
            cfg['success_rate'] = self._success_rate_node(value=bad_value)
            report = cal.validate_frozen(cfg)
            self.assertIn('success_rate.min_pass_rate.value', report['pending'], bad_value)

    def test_success_rate_scope_exact_and_type_r752(self):
        cfg = copy.deepcopy(cal.FROZEN)
        cfg['success_rate'] = self._success_rate_node(scope='x')
        report = cal.validate_frozen(cfg)
        self.assertIn('success_rate.min_pass_rate.scope', report['pending'])
        cfg = copy.deepcopy(cal.FROZEN)
        cfg['success_rate'] = self._success_rate_node(scope=123)
        report = cal.validate_frozen(cfg)
        self.assertIn('success_rate.min_pass_rate', report['pending'])

    def test_success_rate_missing_evidence_and_alpha_r752(self):
        cfg = copy.deepcopy(cal.FROZEN)
        node = self._success_rate_node()
        del node['min_pass_rate']['evidence']
        cfg['success_rate'] = node
        self.assertIn('success_rate.min_pass_rate', cal.validate_frozen(cfg)['pending'])
        cfg = copy.deepcopy(cal.FROZEN)
        cfg['success_rate'] = self._success_rate_node(alpha=0)
        self.assertIn('success_rate.min_pass_rate.alpha',
                      cal.validate_frozen(cfg)['pending'])

    def test_success_rate_raise_not_loosening_r752(self):
        previous = copy.deepcopy(cal.FROZEN)
        current = copy.deepcopy(cal.FROZEN)
        previous['success_rate'] = self._success_rate_node(value=0.90)
        current['success_rate'] = self._success_rate_node(value=0.95)
        self.assertEqual(cal.check_loosening(previous, current), [])

    # ------------------------------------------------------------------
    # R751 P2-1：scope/样本数/分母/聚合口径变更也算放宽（节点级比较）。
    # ------------------------------------------------------------------
    def test_success_rate_scope_change_requires_unfreeze_r755(self):
        previous = copy.deepcopy(cal.FROZEN)
        current = copy.deepcopy(cal.FROZEN)
        previous['success_rate'] = self._success_rate_node(scope='x')
        current['success_rate'] = self._success_rate_node(scope='y')
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.check_loosening(previous, current)
        self.assertEqual(caught.exception.code, 'E_CAL_LOOSENED')
        self.assertIn('success_rate.min_pass_rate.scope', caught.exception.detail)

    def test_success_rate_sample_n_down_requires_unfreeze_r755(self):
        previous = copy.deepcopy(cal.FROZEN)
        current = copy.deepcopy(cal.FROZEN)
        previous['success_rate'] = self._success_rate_node(sample_n=100)
        current['success_rate'] = self._success_rate_node(sample_n=36)
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.check_loosening(previous, current)
        self.assertEqual(caught.exception.code, 'E_CAL_LOOSENED')
        self.assertIn('success_rate.min_pass_rate.sample_n', caught.exception.detail)

    def test_success_rate_same_metadata_no_change_r755(self):
        previous = copy.deepcopy(cal.FROZEN)
        current = copy.deepcopy(cal.FROZEN)
        previous['success_rate'] = self._success_rate_node()
        current['success_rate'] = self._success_rate_node()
        self.assertEqual(cal.check_loosening(previous, current), [])

    # ------------------------------------------------------------------
    # R758（R751 P2-2）：freeze 流程对上一版冻结基线跑 loosening。
    # ------------------------------------------------------------------
    def test_freeze_guard_first_freeze_without_baseline_r758(self):
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'none.json')
            self.assertEqual(cal.freeze_guard(copy.deepcopy(cal.FROZEN), path), [])

    def test_freeze_guard_detects_loosening_r758(self):
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'baseline.json')
            previous = copy.deepcopy(cal.FROZEN)
            previous['success_rate'] = self._success_rate_node(value=0.90)
            current = copy.deepcopy(cal.FROZEN)
            current['success_rate'] = self._success_rate_node(value=0.80)
            with open(path, 'w') as handle:
                json.dump({'frozen': previous, 'digest': 'x'}, handle)
            with self.assertRaises(cal.CalibrationError) as caught:
                cal.freeze_guard(current, path)
            self.assertEqual(caught.exception.code, 'E_CAL_LOOSENED')

    def test_freeze_guard_same_config_passes_r758(self):
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'baseline.json')
            previous = copy.deepcopy(cal.FROZEN)
            previous['success_rate'] = self._success_rate_node()
            with open(path, 'w') as handle:
                json.dump({'frozen': previous, 'digest': 'x'}, handle)
            self.assertEqual(cal.freeze_guard(copy.deepcopy(previous), path), [])

    def test_freeze_guard_bad_baseline_r758(self):
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'baseline.json')
            with open(path, 'w') as handle:
                json.dump({'nope': 1}, handle)
            with self.assertRaises(cal.CalibrationError) as caught:
                cal.freeze_guard(copy.deepcopy(cal.FROZEN), path)
            self.assertEqual(caught.exception.code, 'E_CAL_BASELINE')

    # ------------------------------------------------------------------
    # R759（R751 P3-1）：另三个 pending 键填值也必须带 scope/evidence 且为正数。
    # ------------------------------------------------------------------
    def test_stop_time_bare_value_stays_pending_r759(self):
        cfg = copy.deepcopy(cal.FROZEN)
        cfg['budgets']['stop_time_s'] = {'value': 1.5}
        report = cal.validate_frozen(cfg)
        self.assertIn('budgets.stop_time_s', report['pending'])

    def test_stop_time_with_metadata_leaves_pending_r759(self):
        cfg = copy.deepcopy(cal.FROZEN)
        cfg['budgets']['stop_time_s'] = {'value': 1.5, 'scope': 'x', 'evidence': 'y'}
        report = cal.validate_frozen(cfg)
        self.assertNotIn('budgets.stop_time_s', report['pending'])
        self.assertEqual(len(report['pending']), 2)

    def test_localization_bare_value_rejected_r787(self):
        cfg = copy.deepcopy(cal.FROZEN)
        cfg['tolerances']['max_localization_error_m'] = {'value': 0.2}
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.validate_frozen(cfg)
        self.assertEqual(caught.exception.code, 'E_CAL_PENDING')
        self.assertIn('max_localization_error_m', caught.exception.detail)

    def test_stop_distance_negative_rejected_r759(self):
        cfg = copy.deepcopy(cal.FROZEN)
        cfg['budgets']['stop_distance_m'] = {'value': -1, 'scope': 'x', 'evidence': 'y'}
        report = cal.validate_frozen(cfg)
        self.assertIn('budgets.stop_distance_m.value', report['pending'])

    # ------------------------------------------------------------------
    # R787：B 方案（真人 2026-10-27）——NDT 降级为只做重定位/失效检测，
    # 定位精度主键只压 LIO 单后端；值按 R742 乙案取值式冻结为 0.22。
    # ------------------------------------------------------------------
    def test_localization_frozen_scoped_r787(self):
        item = cal.FROZEN['tolerances']['max_localization_error_m']
        self.assertAlmostEqual(item['value'], 0.22)
        self.assertEqual(item['scope'],
                         'm1_corridor_12p57m_xy_base_link_se2init_9runs_lio_only_b_ndt_monitor')
        self.assertEqual(item['sample_n'], 9)
        self.assertIn('tolerances.max_localization_error_m', cal.SCOPED_REQUIRED)
        self.assertEqual(
            cal.SCOPED_REQUIRED['tolerances.max_localization_error_m']['min_sample_n'], 9)
        report = cal.validate_frozen()
        self.assertNotIn('tolerances.max_localization_error_m.value', report['pending'])
        self.assertEqual(len(report['pending']), 3)

    def test_localization_scope_and_sample_guard_r787(self):
        for field, val in (('scope', 'full_stack'), ('sample_n', 3)):
            cfg = copy.deepcopy(cal.FROZEN)
            cfg['tolerances']['max_localization_error_m'][field] = val
            with self.assertRaises(cal.CalibrationError) as caught:
                cal.validate_frozen(cfg)
            self.assertEqual(caught.exception.code, 'E_CAL_PENDING')

    def test_localization_raise_requires_unfreeze_r787(self):
        previous = copy.deepcopy(cal.FROZEN)
        previous['tolerances']['max_localization_error_m']['value'] = 0.194
        with self.assertRaises(cal.CalibrationError) as caught:
            cal.check_loosening(previous, copy.deepcopy(cal.FROZEN))
        self.assertEqual(caught.exception.code, 'E_CAL_LOOSENED')
        self.assertIn('tolerances.max_localization_error_m.value',
                      caught.exception.detail)
        current = copy.deepcopy(cal.FROZEN)
        current['unfreeze_log'] = list(current['unfreeze_log']) + [{
            'when': '2026-10-27', 'key': 'tolerances.max_localization_error_m.value',
            'from': 0.194, 'to': 0.22, 'reason': 'B 方案单后端主键，R787 显式解封'}]
        self.assertEqual(cal.check_loosening(previous, current),
                         ['tolerances.max_localization_error_m.value'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
