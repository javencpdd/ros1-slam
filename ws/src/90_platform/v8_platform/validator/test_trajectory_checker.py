"""V8-10 首切片测试：独立轨迹检查器四核验＋V8-09 规划器输出全链通过＋
V8-03「悬空轨迹」场景被拒（优化器兜底掩盖的对策实证）。"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
sys.path.insert(0, str(HERE.parent / 'representation'))
sys.path.insert(0, str(HERE.parent / 'planner'))
import support_map as rep  # noqa: E402
import support_planner as pln  # noqa: E402
import trajectory_checker as chk  # noqa: E402

RES = 0.5
SOURCE = {'uri': 'design://floors', 'sha256': 'a' * 64}


def floors(roof=False, oneway=False):
    surfaces = [
        {'surface_id': 'ground', 'z_reference': 0.0, 'data_uri': 'g',
         'footprint': {'min_x': -5, 'min_y': -5, 'max_x': 5, 'max_y': 5},
         'edges': [{'to': 'mezz', 'kind': 'slope', 'max_slope_deg': 12.0,
                    'bidirectional': not oneway}]},
        {'surface_id': 'mezz', 'z_reference': 0.6, 'data_uri': 'm',
         'footprint': {'min_x': 2, 'min_y': 1, 'max_x': 4, 'max_y': 3},
         'edges': [] if oneway else [{'to': 'ground', 'kind': 'slope',
                                      'max_slope_deg': 12.0, 'bidirectional': True}]},
    ]
    if roof:
        surfaces[0]['edges'].append({'to': 'roof', 'kind': 'step',
                                     'max_step_height_m': 1.0, 'bidirectional': True})
        surfaces.append({'surface_id': 'roof', 'z_reference': 1.0, 'data_uri': 'r',
                         'footprint': {'min_x': -1, 'min_y': -1, 'max_x': 1, 'max_y': 1},
                         'edges': [{'to': 'ground', 'kind': 'step',
                                    'max_step_height_m': 1.0, 'bidirectional': True}]})
    return rep.build_support_map('chk_map', RES, [-5.0, -5.0, 0.0], surfaces, None, SOURCE)


class CheckerTests(unittest.TestCase):
    def test_planner_output_passes(self):
        """全链：V8-09 规划器路径→检查器通过（支撑/净空/连续安全齐）。"""
        value = floors()
        route = pln.plan(value, (-4.0, -4.0, 0.0), (3.0, 2.0, 0.6))
        trajectory = [(x, y, z) for x, y, z, _sid in route['path']]
        result = chk.check(value, trajectory)
        self.assertTrue(result['ok'], result['violations'])
        self.assertEqual(result['checked_points'], len(trajectory))

    def test_hovering_rejected(self):
        """V8-03 对策实证：悬空轨迹（R633 坡脚悬空同型）被逐点支撑检查拒绝。"""
        value = floors()
        hover = [(-4.0, -4.0, 0.0), (-3.5, -4.0, 0.0), (-3.0, -4.0, 0.55),
                 (-2.5, -4.0, 0.55), (-2.0, -4.0, 0.0), (-1.5, -4.0, 0.0)]
        result = chk.check(value, hover)
        self.assertFalse(result['ok'])
        codes = {v[0] for v in result['violations']}
        self.assertIn('E_TRAJ_UNSUPPORTED', codes)
        indexes = [v[1] for v in result['violations'] if v[0] == 'E_TRAJ_UNSUPPORTED']
        self.assertEqual(indexes, [2, 3])  # 恰为悬空段

    def test_transition_without_edge_rejected(self):
        """单向边：mezz→ground 无回边，跨面步被拒（方向敏感）。"""
        value = floors(oneway=True)
        jump = [(3.0, 2.0, 0.6), (3.0, 2.05, 0.6), (2.0, 1.5, 0.0)]
        result = chk.check(value, jump)
        self.assertFalse(result['ok'])
        self.assertEqual(result['violations'][0][0], 'E_TRAJ_TRANSITION')

    def test_discontinuity_rejected(self):
        value = floors()
        teleport = [(-4.0, -4.0, 0.0), (4.0, 4.0, 0.0)]
        result = chk.check(value, teleport)
        self.assertFalse(result['ok'])
        self.assertEqual(result['violations'][0][0], 'E_TRAJ_DISCONTINUITY')

    def test_clearance_rejected(self):
        """roof 距地面 1.0：地面段净空不足阈值（机器人包络）被拒。"""
        value = floors(roof=True)
        low = [(-0.5, -0.5, 0.0), (0.0, 0.0, 0.0), (0.5, 0.5, 0.0)]
        result = chk.check(value, low, min_clearance=1.1)
        self.assertFalse(result['ok'])
        self.assertTrue(any(v[0] == 'E_TRAJ_CLEARANCE' for v in result['violations']))

    def test_time_validity(self):
        value = floors()
        path = [(-4.0, -4.0, 0.0, 0.0), (-3.5, -4.0, 0.0, 1.0), (-3.0, -4.0, 0.0, 2.0)]
        self.assertTrue(chk.check(value, path, time_budget=5.0)['ok'])
        over = [(-4.0, -4.0, 0.0, 0.0), (-3.5, -4.0, 0.0, 3.0), (-3.0, -4.0, 0.0, 6.0)]
        result = chk.check(value, over, time_budget=5.0)
        self.assertFalse(result['ok'])
        self.assertEqual(result['violations'][0][0], 'E_TRAJ_TIME')
        backwards = [(-4.0, -4.0, 0.0, 2.0), (-3.5, -4.0, 0.0, 1.0)]
        self.assertFalse(chk.check(value, backwards)['ok'])

    def test_guards(self):
        with self.assertRaises(chk.CheckerError) as caught:
            chk.check(floors(), [])
        self.assertEqual(caught.exception.code, 'E_TRAJ_EMPTY')
        bad = floors()
        bad['surfaces'][0]['edges'] = []
        bad['surfaces'][1]['edges'] = []
        with self.assertRaises(chk.CheckerError) as caught:
            chk.check(bad, [(-4.0, -4.0, 0.0)])
        self.assertEqual(caught.exception.code, 'E_TRAJ_MAP_INVALID')
        with self.assertRaises(chk.CheckerError) as caught:
            chk.check(floors(), [(-4.0, -4.0, 0.0)], expected_map_version=99)
        self.assertEqual(caught.exception.code, 'E_TRAJ_MAP_STALE')


if __name__ == '__main__':
    unittest.main(verbosity=2)
