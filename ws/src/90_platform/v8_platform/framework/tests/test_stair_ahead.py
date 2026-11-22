"""前方台阶探测单测（纯 Python，无 ROS）。"""
import sys as _sys
from pathlib import Path as _Path

_BASE = _Path(__file__).resolve().parent.parent
for _entry in ('', 'nodes', 'tools', 'smokes'):
    _sys.path.insert(0, str(_BASE / _entry) if _entry else str(_BASE))
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
FRAMEWORK = HERE.parent
sys.path.insert(0, str(FRAMEWORK))
import stair_ahead  # noqa: E402


def two_level_map():
    step = {'kind': 'step', 'max_step_height_m': 0.15, 'bidirectional': True}
    return {'schema_version': 1, 'kind': 'support_map', 'frame': 'map',
            'resolution': 0.1, 'map_id': 'test', 'map_version': 1,
            'surfaces': [
                {'surface_id': 'm0', 'z_reference': 0.0, 'traversability': 'supported',
                 'footprint': {'min_x': 0.0, 'min_y': 0.0, 'max_x': 2.0, 'max_y': 4.0},
                 'edges': [dict(step, to='m1')]},
                {'surface_id': 'm1', 'z_reference': 0.15, 'traversability': 'supported',
                 'footprint': {'min_x': 2.0, 'min_y': 0.0, 'max_x': 4.0, 'max_y': 4.0},
                 'edges': [dict(step, to='m0')]},
            ]}


class StairAheadTests(unittest.TestCase):
    def test_detects_step_ahead(self):
        result = stair_ahead.stair_ahead(two_level_map(), (1.0, 2.0, 0.0),
                                         [(1.0, 2.0, 0.0), (3.0, 2.0, 0.15)],
                                         lookahead_m=2.0)
        self.assertTrue(result['ahead'])
        self.assertAlmostEqual(result['step_m'], 0.15, places=3)
        self.assertEqual(result['direction'], 'up')
        self.assertEqual(result['from_surface'], 'm0')
        self.assertEqual(result['to_surface'], 'm1')
        self.assertAlmostEqual(result['distance_m'], 1.0, delta=0.1)

    def test_lookahead_blocks_detection(self):
        result = stair_ahead.stair_ahead(two_level_map(), (1.0, 2.0, 0.0),
                                         [(1.0, 2.0, 0.0), (3.0, 2.0, 0.15)],
                                         lookahead_m=0.5)
        self.assertFalse(result['ahead'])

    def test_min_step_filters_small_ledges(self):
        result = stair_ahead.stair_ahead(two_level_map(), (1.0, 2.0, 0.0),
                                         [(1.0, 2.0, 0.0), (3.0, 2.0, 0.15)],
                                         min_step_m=0.2)
        self.assertFalse(result['ahead'])

    def test_pose_off_support(self):
        result = stair_ahead.stair_ahead(two_level_map(), (9.0, 9.0, 0.0),
                                         [(9.0, 9.0, 0.0), (3.0, 2.0, 0.0)])
        self.assertFalse(result['ahead'])
        self.assertEqual(result['reason'], 'pose_off_support')


def flat_map():
    """一块平地，没有任何 step 边。"""
    return {'schema_version': 1, 'kind': 'support_map', 'frame': 'map',
            'resolution': 0.1, 'map_id': 'far_flat', 'map_version': 1,
            'surfaces': [
                {'surface_id': 'm0', 'z_reference': 0.0, 'traversability': 'supported',
                 'footprint': {'min_x': -1.0, 'min_y': -2.0, 'max_x': 6.0, 'max_y': 2.0},
                 'edges': []},
            ]}


def step_map():
    """车正前 1.2 m 起有一级 0.2 m 台阶（沿 +x 走会遇到）。"""
    step = {'kind': 'step', 'max_step_height_m': 0.3, 'bidirectional': True}
    return {'schema_version': 1, 'kind': 'support_map', 'frame': 'map',
            'resolution': 0.1, 'map_id': 'far_step', 'map_version': 1,
            'surfaces': [
                {'surface_id': 'm0', 'z_reference': 0.0, 'traversability': 'supported',
                 'footprint': {'min_x': -1.0, 'min_y': -2.0, 'max_x': 6.0, 'max_y': 2.0},
                 'edges': [dict(step, to='m1')]},
                {'surface_id': 'm1', 'z_reference': 0.2, 'traversability': 'supported',
                 'footprint': {'min_x': 1.2, 'min_y': -2.0, 'max_x': 6.0, 'max_y': 2.0},
                 'edges': [dict(step, to='m0')]},
            ]}


def right_turn_map():
    """台阶在右转之后（车辆先沿 +x 走到路口再右转，台阶在 +y 方向 1.0 m 起）。"""
    step = {'kind': 'step', 'max_step_height_m': 0.3, 'bidirectional': True}
    return {'schema_version': 1, 'kind': 'support_map', 'frame': 'map',
            'resolution': 0.1, 'map_id': 'far_turn', 'map_version': 1,
            'surfaces': [
                {'surface_id': 'm0', 'z_reference': 0.0, 'traversability': 'supported',
                 'footprint': {'min_x': -1.0, 'min_y': -2.0, 'max_x': 6.0, 'max_y': 6.0},
                 'edges': [dict(step, to='m1')]},
                {'surface_id': 'm1', 'z_reference': 0.2, 'traversability': 'supported',
                 'footprint': {'min_x': -1.0, 'min_y': 1.0, 'max_x': 1.5, 'max_y': 6.0},
                 'edges': [dict(step, to='m0')]},
            ]}


# 适配器 gait_path 的三点形状：起点＝当前位姿、第二点＝FAR 航点、第三点＝沿同方向延长到
# 总长 lookahead_m + 0.3 = 1.8 m。这里逐字写出而不再 import 适配器，是为了让两个包各自独立：
# 三点的生成规则由 v8_algorithm 侧的 test_far_node 守，本文件守的是「这三种形状喂给判据会怎样」。
FAR_NEAR_PATH = [(0.0, 0.0, 0.0), (0.3, 0.0, 0.0), (1.8, 0.0, 0.0)]


class FarPathTests(unittest.TestCase):
    """FAR 航点接入后，`gait_guard` 的前瞻在什么条件下可信（计划 §4.3 的 T1—T6）。"""

    def test_T1_near_waypoint_scan_reaches_lookahead(self):
        """T1：航点 0.3 m、平地。延长段必须把扫描距离撑到 lookahead 附近。

        失败判据：`scanned_m` ≈ 0.3 m ⇒ 延长段没生效，或在航点处被提前截断。
        """
        result = stair_ahead.stair_ahead(flat_map(), (0.0, 0.0, 0.0), FAR_NEAR_PATH,
                                         lookahead_m=1.5)
        self.assertFalse(result['ahead'])
        self.assertGreaterEqual(result['scanned_m'], 0.95 * 1.5)

    def test_T2_near_waypoint_still_sees_step_at_1m2(self):
        """T2：航点 0.3 m、台阶在 1.2 m。延长段生效时这一级台阶必须被看见。"""
        result = stair_ahead.stair_ahead(step_map(), (0.0, 0.0, 0.0), FAR_NEAR_PATH,
                                         lookahead_m=1.5)
        self.assertTrue(result['ahead'])
        self.assertAlmostEqual(result['step_m'], 0.2, delta=0.01)
        self.assertTrue(1.0 <= result['distance_m'] <= 1.4, result)

    def test_T3_far_waypoint_two_points_still_works(self):
        """T3：航点 2.5 m（不需延长，只有两点）。补齐逻辑不许把正常场景搞坏。"""
        result = stair_ahead.stair_ahead(step_map(), (0.0, 0.0, 0.0),
                                         [(0.0, 0.0, 0.0), (2.5, 0.0, 0.0)],
                                         lookahead_m=1.5)
        self.assertTrue(result['ahead'])
        self.assertTrue(1.0 <= result['distance_m'] <= 1.4, result)

    def test_T4_flat_ground_reports_no_fake_step(self):
        """T4（反向用例）：平地、航点 2 m。不许凭空造出 `step_m`。"""
        result = stair_ahead.stair_ahead(flat_map(), (0.0, 0.0, 0.0),
                                         [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)],
                                         lookahead_m=1.5)
        self.assertFalse(result['ahead'])
        self.assertNotIn('step_m', result)

    def test_T5_without_extension_must_flip_red(self):
        """T5（反控）：把第三点摘掉退回两点，T1／T2 必须同时翻红。

        这一条不过就说明 T1／T2 是空控——场景根本没打到判据，前面的绿不算数。
        """
        two_points = FAR_NEAR_PATH[:2]
        t1 = stair_ahead.stair_ahead(flat_map(), (0.0, 0.0, 0.0), two_points,
                                    lookahead_m=1.5)
        self.assertLessEqual(t1['scanned_m'], 0.35)
        t2 = stair_ahead.stair_ahead(step_map(), (0.0, 0.0, 0.0), two_points,
                                    lookahead_m=1.5)
        self.assertFalse(t2['ahead'])

    def test_T6_known_failure_right_turn_after_near_waypoint(self):
        """T6（已知失败模式，只登记不判通过）：航点 0.5 m 后右转，台阶在右转之后 1.0 m。

        延长段只会沿「位姿→航点」的方向外推，路线拐弯时方向就是错的 ⇒ 预期 `ahead=False`。
        ★ 结论必须写进验证记录：FAR 航点间距小于 lookahead_m 且路线含 ≥60 度转弯时，
        步态前瞻不可信，不得据此宣称自动上下楼。★
        若这里反而出现 `ahead=True`，要查是不是取样点碰巧踏进了台阶面，而不是修这条用例。
        """
        path = [(0.0, 0.0, 0.0), (0.5, 0.0, 0.0), (1.8, 0.0, 0.0)]
        result = stair_ahead.stair_ahead(right_turn_map(), (0.0, 0.0, 0.0), path,
                                         lookahead_m=1.5)
        self.assertFalse(result['ahead'], result)


if __name__ == '__main__':
    unittest.main(verbosity=2)
