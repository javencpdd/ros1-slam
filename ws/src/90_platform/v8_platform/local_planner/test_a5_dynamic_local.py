"""A5 动态障碍本地规划判据（纯 Python，无 ROS）。

两条：
 1) 横向移动的障碍不得被规划成穿过去的轨迹（允许 ok 且绕开，或明确拒绝）；
 2) 横贯走廊的静态障碍必须拒绝，不能给出穿过去的轨迹。
"""
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
sys.path.insert(0, str(HERE.parent / 'validator'))
import local_native  # noqa: E402


def flat():
    value = json.loads((HERE.parent / 'contracts/fixtures/map_single_floor.json').read_text())
    value['surfaces'][0]['footprint'].update(min_x=-2.0, max_x=3.0, min_y=-2.0, max_y=2.0)
    return value


def config():
    return local_native.Config(
        resolution=0.2, tick_s=0.5, horizon_s=4.0, validity_s=0.75,
        max_speed_mps=0.5, max_step_m=0.15, max_slope_deg=40.0,
        radius_m=0.2, height_m=0.4, clearance_m=0.2,
        goal_tolerance_m=0.08, support_tol_m=1e-6)


def moving_box():
    return {'min': [1.6, -0.3, 0.0], 'max': [2.0, 0.3, 0.6],
            'velocity': [-0.5, 0.0, 0.0], 'stamp': 0.0,
            'active_from': -1e9, 'active_until': 1e9}


def static_wall():
    return {'min': [1.2, -2.0, 0.0], 'max': [1.6, 2.0, 0.6],
            'velocity': [0.0, 0.0, 0.0], 'stamp': 0.0,
            'active_from': -1e9, 'active_until': 1e9}


class A5DynamicTests(unittest.TestCase):
    def test_moving_obstacle_never_intersected(self):
        cfg = config()
        try:
            result = local_native.plan(flat(), (0.0, 0.0, 0.0), (2.8, 0.0, 0.0),
                                       obstacles=[moving_box()], now=0.0, config=cfg)
        except local_native.LocalError as error:
            self.assertIn(error.code, ('E_LOCAL_NO_ROUTE', 'E_LOCAL_CLEARANCE',
                                       'E_LOCAL_SUPPORT'))
            return
        for point in result['trajectory']:
            x, y, _z = point[0], point[1], point[2]
            t = point[3] if len(point) > 3 else 0.0
            obstacle_x = 1.8 - 0.5 * t
            overlap_x = abs(x - obstacle_x) < (0.2 + 0.2)
            overlap_y = abs(y - 0.0) < (0.3 + 0.2)
            self.assertFalse(overlap_x and overlap_y,
                             '轨迹在 t=%.2f 处穿过移动障碍（x=%.2f 障碍=%.2f）'
                             % (t, x, obstacle_x))

    def test_static_wall_refused(self):
        with self.assertRaises(local_native.LocalError) as caught:
            local_native.plan(flat(), (0.0, 0.0, 0.0), (2.8, 0.0, 0.0),
                              obstacles=[static_wall()], now=0.0, config=config())
        self.assertIn(caught.exception.code,
                      ('E_LOCAL_NO_ROUTE', 'E_LOCAL_CLEARANCE'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
