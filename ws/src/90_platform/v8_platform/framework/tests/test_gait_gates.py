"""步态门值解析单测（纯 Python，无 ROS）。

覆盖：声明基线零回归、空集拒判、模板态拒用、契约反算生效、撤值反控。
门值数值一律不在此硬编码为「应改成多少」：没有真机实测就没有依据改数值，
本件只验证「可反算、可拒用、可回退」这套机制本身。
"""
import sys as _sys
from pathlib import Path as _Path

_BASE = _Path(__file__).resolve().parent.parent
for _entry in ('', 'nodes', 'tools', 'smokes'):
    _sys.path.insert(0, str(_BASE / _entry) if _entry else str(_BASE))
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(_BASE))
from gait_gates import resolve_gates, validate_declared  # noqa: E402


DECLARED = {'lookahead_m': 1.5, 'enter_m': 1.0, 'exit_m': 1.5,
            'min_step_m': 0.05, 'max_step_m': 0.15}


def filled_contract(v=2.0, step_max=0.12):
    return {
        'status': 'FILLED', 'effective': True,
        'gaits': {'flat_mid': {
            'latency_jitter': {'cmd_lag_p95_s': 0.20, 'switch_settle_s': 0.80},
            'failure': {'stop_distance_m': 0.40},
            'envelope': {'v_max_mps': v, 'step_max_m': step_max},
        }},
    }


class GaitGatesTests(unittest.TestCase):
    def test_declared_is_byte_identical_baseline(self):
        r = resolve_gates(DECLARED)
        self.assertEqual(r['source'], 'declared')
        self.assertEqual(r['applied'], [])
        self.assertEqual(r['gates'], {k: float(v) for k, v in DECLARED.items()})
        self.assertEqual(DECLARED['lookahead_m'], 1.5)  # 未改动调用方

    def test_empty_set_refused(self):
        with self.assertRaises(ValueError):
            resolve_gates({'lookahead_m': 1.5})
        with self.assertRaises(ValueError):
            resolve_gates(dict(DECLARED, max_step_m=None))
        with self.assertRaises(ValueError):
            validate_declared('not a dict')

    def test_template_not_effective_falls_back(self):
        r = resolve_gates(DECLARED, source='contract',
                          contract={'status': 'TEMPLATE_ONLY', 'effective': False})
        self.assertEqual(r['source'], 'declared')
        self.assertEqual(r['gates']['lookahead_m'], 1.5)
        self.assertTrue(any('template_not_effective' in x for x in r['refusals']))

    def test_incomplete_contract_falls_back(self):
        partial = filled_contract()
        partial['gaits']['flat_mid']['failure']['stop_distance_m'] = None
        r = resolve_gates(DECLARED, source='contract', contract=partial)
        self.assertEqual(r['source'], 'declared')
        self.assertTrue(any('stop_distance_m' in x for x in r['refusals']))
        self.assertEqual(r['gates'], {k: float(v) for k, v in DECLARED.items()})

    def test_contract_recomputes_gates(self):
        r = resolve_gates(DECLARED, source='contract',
                          contract=filled_contract(), margin_m=0.20)
        # 2.0*(0.20+0.80)+0.40+0.20 = 2.60
        self.assertEqual(r['source'], 'contract')
        self.assertEqual(r['gates']['lookahead_m'], 2.6)
        self.assertEqual(r['gates']['max_step_m'], 0.12)
        self.assertIn('lookahead_m', r['applied'])
        self.assertIn('max_step_m', r['applied'])
        # 没有实测依据的门值保持声明基线
        self.assertEqual(r['gates']['enter_m'], 1.0)
        self.assertEqual(r['gates']['exit_m'], 1.5)
        self.assertEqual(r['gates']['min_step_m'], 0.05)

    def test_only_increasing_lookahead(self):
        # 反算 0.5*(0.20+0.80)+0.40 = 0.90 m 小于声明 1.5 m，
        # 不得缩短前视降低安全裕度
        r = resolve_gates(DECLARED, source='contract',
                          contract=filled_contract(v=0.5), margin_m=0.0)
        self.assertEqual(r['gates']['lookahead_m'], 1.5)
        self.assertTrue(any('lookahead_below_declared' in x for x in r['refusals']))

    def test_reverse_control(self):
        contract = filled_contract()
        hot = resolve_gates(DECLARED, source='contract',
                            contract=contract, margin_m=0.20)
        self.assertEqual(hot['gates']['lookahead_m'], 2.6)
        contract['effective'] = False
        cold = resolve_gates(DECLARED, source='contract',
                             contract=contract, margin_m=0.20)
        self.assertEqual(cold['source'], 'declared')
        self.assertEqual(cold['gates'], {k: float(v) for k, v in DECLARED.items()})


if __name__ == '__main__':
    unittest.main(verbosity=2)
