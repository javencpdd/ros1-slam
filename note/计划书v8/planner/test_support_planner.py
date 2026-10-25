"""V8-09 首切片测试：有界 A* 支撑图搜索——正例（有支撑且连续安全）、负例三族、
预算/取消/诊断；地图由 representation 生产者构造（三件链集成）。

语义发现（R652 登记）：map.v1 单连通＋「不可通行面不得持边/不得为边目标」⇒
unsupported 面在任何合法 support_map 中结构性不可存在（traversability 枚举的
'unsupported' 值在 map 层为死值）——本文件以断言锁定该发现。
"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
sys.path.insert(0, str(HERE.parent / 'representation'))
import support_map as rep  # noqa: E402
import support_planner as pln  # noqa: E402
import validate as contract  # noqa: E402

RES = 0.5
SOURCE = {'uri': 'design://floors', 'sha256': 'f' * 64}


def floors(oneway=False):
    """双面图：ground 全域＋mezz 夹层。oneway＝ground→mezz 单向（mezz 无回边）。"""
    return rep.build_support_map(
        'plan_map', RES, [-5.0, -5.0, 0.0],
        [{'surface_id': 'ground', 'z_reference': 0.0, 'data_uri': 'g',
          'footprint': {'min_x': -5, 'min_y': -5, 'max_x': 5, 'max_y': 5},
          'edges': [{'to': 'mezz', 'kind': 'slope', 'max_slope_deg': 12.0,
                     'bidirectional': not oneway}]},
         {'surface_id': 'mezz', 'z_reference': 0.6, 'data_uri': 'm',
          'footprint': {'min_x': 2, 'min_y': 1, 'max_x': 4, 'max_y': 3},
          'edges': [] if oneway else [{'to': 'ground', 'kind': 'slope',
                                       'max_slope_deg': 12.0, 'bidirectional': True}]}],
        None, SOURCE)


def reason(fn):
    try:
        fn()
        return 'OK', None
    except pln.PlannerError as error:
        return error.code, error.detail


class PlannerTests(unittest.TestCase):
    def test_positive_route_supported_and_continuous(self):
        """A4 正例：地面→坡→夹层；路径逐点落其分段面足迹内（有支撑且连续安全）。"""
        value = floors()
        result = pln.plan(value, (-4.0, -4.0, 0.0), (3.0, 2.0, 0.6))
        self.assertTrue(result['ok'])
        self.assertEqual(result['path'][0][3], 'ground')
        self.assertEqual(result['path'][-1][3], 'mezz')
        self.assertEqual([s['surface'] for s in result['segments']], ['ground', 'mezz'])
        self.assertEqual(result['transitions'], [{'from': 'ground', 'to': 'mezz'}])
        surfaces = {s['surface_id']: s for s in value['surfaces']}
        for x, y, z, sid in result['path']:
            fp = surfaces[sid]['footprint']
            self.assertTrue(fp['min_x'] - 1e-9 <= x <= fp['max_x'] + 1e-9, (x, y, sid))
            self.assertTrue(fp['min_y'] - 1e-9 <= y <= fp['max_y'] + 1e-9, (x, y, sid))
            self.assertAlmostEqual(z, surfaces[sid]['z_reference'])
        self.assertGreater(result['expansions'], 0)
        self.assertGreater(result['length'], 0)

    def test_same_surface_short_route(self):
        result = pln.plan(floors(), (-4.0, -4.0, 0.0), (4.0, 4.0, 0.0))
        self.assertEqual(result['transitions'], [])
        self.assertEqual(len(result['segments']), 1)

    def test_goal_off_any_surface(self):
        code, _ = reason(lambda: pln.plan(floors(), (-4, -4, 0.0), (50.0, 50.0, 0.0)))
        self.assertEqual(code, 'E_PLAN_GOAL_INVALID')

    def test_oneway_edge_unreachable(self):
        """单向边：map 合法（首面 ground 可达全图），mezz 出发回不去 ground＝无路。"""
        code, detail = reason(lambda: pln.plan(floors(oneway=True),
                                               (3.0, 2.0, 0.6), (-4.0, -4.0, 0.0)))
        self.assertEqual(code, 'E_PLAN_NO_ROUTE')

    def test_budget_exceeded(self):
        code, _ = reason(lambda: pln.plan(floors(), (-4.0, -4.0, 0.0), (3.0, 2.0, 0.6),
                                          budget=3))
        self.assertEqual(code, 'E_PLAN_BUDGET_EXCEEDED')

    def test_cancelled(self):
        code, _ = reason(lambda: pln.plan(floors(), (-4.0, -4.0, 0.0), (3.0, 2.0, 0.6),
                                          cancel=lambda: True))
        self.assertEqual(code, 'E_PLAN_CANCELLED')

    def test_invalid_map_rejected(self):
        bad = floors()
        bad['surfaces'][0]['edges'] = []  # 断连通（双向边两边都要清）
        bad['surfaces'][1]['edges'] = []
        code, _ = reason(lambda: pln.plan(bad, (-4, -4, 0.0), (3, 2, 0.6)))
        self.assertEqual(code, 'E_PLAN_MAP_INVALID')

    def test_unsupported_surface_inexpressible_in_map(self):
        """语义发现断言：合法 support_map 结构性容不下 unsupported 面。"""
        try:
            value = rep.build_support_map(
                'bad', RES, [-5.0, -5.0, 0.0],
                [{'surface_id': 'g', 'z_reference': 0.0, 'data_uri': 'g',
                  'footprint': {'min_x': -5, 'min_y': -5, 'max_x': 5, 'max_y': 5},
                  'edges': []},
                 {'surface_id': 'u', 'z_reference': 0.6, 'data_uri': 'u',
                  'footprint': {'min_x': 2, 'min_y': 1, 'max_x': 4, 'max_y': 3},
                  'traversability': 'unsupported', 'edges': []}],
                None, SOURCE)
            contract.validate('map', value)
            self.fail('unsupported 面意外成图')
        except (rep.RepresentationError, contract.ContractError) as error:
            self.assertIn('E_MAP', str(error))  # DISCONNECTED：无路可达 unsupported 面


if __name__ == '__main__':
    unittest.main(verbosity=2)
