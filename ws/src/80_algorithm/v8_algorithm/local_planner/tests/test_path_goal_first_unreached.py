"""起点跳过与去重下发 —— FAR 滚动两点路径下"车一步不走"的回归测试。

背景（2026-11-25 实测）：`on_path` 原先把 `index` 硬置 0 再发 `poses[0]`。support 的完整路径
下 poses[0] 就是当前位姿、由 tick 推进到下一个点，所以看不出来；但 FAR 给的是**两点滚动路径**
（当前位姿 + 航点）且每秒重发，于是每秒都把 local 重置成"去我站的地方"⇒ local 输出恒零。
探针证据：32 条路径 → 64 个局部目标，在 (0,-3.06)（＝当时位姿）与 (1,0)（＝航点）之间交替。

三连：主测（跳过已到达点）＋ 两条反控（全在半径内时不得退回 0；目标未变时不得重发）。
"""
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

for _name in ('rospy', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs',
              'nav_msgs.msg', 'std_msgs', 'std_msgs.msg'):
    if _name not in sys.modules:
        sys.modules[_name] = MagicMock()

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
import path_goal_node as pg  # noqa: E402

import unittest  # noqa: E402


def tearDownModule():
    for _name in ('rospy', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs',
                  'nav_msgs.msg', 'std_msgs', 'std_msgs.msg'):
        if isinstance(sys.modules.get(_name), MagicMock):
            del sys.modules[_name]


def _pose(x, y, z=0.0):
    # position 必须是真 float：publish_current 会对它做 round() 再 json.dumps
    return types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id='map', stamp=MagicMock()),
        pose=types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y, z=z)))


def _obj(poses, robot, arrive_m=0.30):
    """绕过 __init__（它会读 ROS 参数），只装配被测方法需要的三个属性。"""
    o = object.__new__(pg.PathGoal)
    o.poses = list(poses)
    o.pose = robot
    o.arrive_m = arrive_m
    return o


def _robot(x, y):
    r = MagicMock()
    r.x, r.y = x, y
    return r


class FirstUnreachedTest(unittest.TestCase):

    def test_far_two_point_path_starts_at_waypoint(self):
        """主测：FAR 两点路径（poses[0]＝当前位姿）必须从航点开始，不是从脚下开始。"""
        o = _obj([_pose(0.0, -3.06), _pose(1.0, 0.0)], _robot(0.0, -3.06))
        self.assertEqual(1, o._first_unreached(),
                         '仍然把"当前位姿"当目标下发 —— FAR 滚动路径会被每秒重置')

    def test_skips_all_reached_prefix(self):
        o = _obj([_pose(0.0, -3.06), _pose(0.05, -3.05), _pose(1.0, 0.0)], _robot(0.0, -3.06))
        self.assertEqual(2, o._first_unreached())

    def test_control_all_inside_radius_returns_last_not_zero(self):
        """反控：全都在到达半径内时退回最后一个点，绝不能退回 0（那等于原地不动）。"""
        o = _obj([_pose(0.0, -3.06), _pose(0.1, -3.0)], _robot(0.0, -3.06))
        self.assertEqual(1, o._first_unreached())

    def test_no_pose_falls_back_to_zero(self):
        o = _obj([_pose(1.0, 0.0)], None)
        self.assertEqual(0, o._first_unreached())

    def test_control_duplicate_goal_not_republished(self):
        """反控：同一个目标连发不得重复 publish（每秒重发会让 local 反复重开跟踪）。"""
        o = _obj([_pose(1.0, 0.0)], _robot(0.0, 0.0))
        o.index = 0
        o.accepted_generation = 1
        o.goal_pub, o.status_pub = MagicMock(), MagicMock()
        o.published_at = None
        o.last_goal_xyz = None
        o.rejected_stale = 0
        pg.rospy.Time.now.return_value = MagicMock()
        o.publish_current()
        o.publish_current()
        self.assertEqual(1, o.goal_pub.publish.call_count, '同一目标被重复下发')


if __name__ == '__main__':
    unittest.main(verbosity=2)
