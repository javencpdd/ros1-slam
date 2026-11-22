"""FAR 适配器的离线判据（不 import ROS、不需要 roscore、不需要 overlay 已编译）。

跑法（宿主即可，容器外）：
    cd ws/src/80_algorithm/v8_algorithm/planner
    /usr/bin/python3 -B -m unittest discover -s tests -p 'test_far_node.py' -v

七条必测对应计划 §三；末尾两条是跨包契约互检，防取消帧格式在两侧各自漂移。
"""
import json
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str((HERE.parent / 'local_planner').resolve()))
import far_node  # noqa: E402
import cancel_policy  # noqa: E402


def stamped(x, y, z):
    """字段形状与 geometry_msgs/PoseStamped 的 `.pose.position` 一致。

    离线测试不 import ROS 消息包，但要证明「喂真消息也成立」——
    far_node 只按属性名读，不看类型，所以这个替身等价。
    """
    return SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=z)))


def length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


class PathShapeTests(unittest.TestCase):
    def test_01_two_point_path_is_exactly_two_frames(self):
        pose, waypoint = (1., 2., .5), (4., 6., 1.2)
        points = far_node.two_point_path(pose, waypoint)
        self.assertEqual(len(points), 2)
        self.assertEqual(points[0], pose)
        self.assertEqual(points[-1][2], 1.2)

    def test_02_pose_stamped_z_is_not_flattened(self):
        """z 被压平不是美观问题：局部规划器会拿到差几米的落点，多层场景直接撞层间平台。"""
        message = stamped(0., 0., 1.4)
        self.assertEqual(far_node.two_point_path(message, (2., 0., 1.4))[-1][2], 1.4)
        # 补点那条分支同样不许把 z 抹掉
        self.assertEqual(far_node.gait_path(message, (0.2, 0., 1.4))[-1][2], 1.4)

    def test_03_height_classification_uses_strict_less_than(self):
        self.assertEqual(far_node.classify(0.0), 'free')
        self.assertEqual(far_node.classify(0.3), 'obs')
        self.assertEqual(far_node.classify(far_node.FREE_Z_M), 'obs')   # 边界归障碍
        # h 由地面高度派生：同一格取最低点当地面
        levelled = far_node.terrain_cloud([(0., 0., 1.0), (0.05, 0.05, 3.0)], h=1.0)
        self.assertEqual([round(item[3], 6) for item in levelled], [0.0, 2.0])
        self.assertEqual([far_node.classify(item[3]) for item in levelled], ['free', 'obs'])


class StateTests(unittest.TestCase):
    def test_04_missing_input_publishes_status_only(self):
        state = far_node.AdapterState()
        points, text = state.on_waypoint(None, (1., 0., 0.))
        self.assertEqual(points, [])
        self.assertEqual(json.loads(text)['reason'], 'E_FAR_NO_POSE')
        points, text = state.on_waypoint((0., 0., 0.), None)
        self.assertEqual(points, [])
        self.assertEqual(json.loads(text)['reason'], 'E_FAR_NO_WAYPOINT')
        self.assertEqual(json.loads(text)['points'], 0)

    def test_05_cancel_keeps_halted_until_new_goal(self):
        state = far_node.AdapterState()
        seq = state.on_cancel()
        self.assertTrue(state.halted)
        points, text = state.on_waypoint((0., 0., 0.), (2., 0., 0.), now=1.)
        self.assertEqual(points, [])
        self.assertEqual(json.loads(text)['reason'], 'E_FAR_HALTED')
        # ack 到位也不解除——ack 只说明「取消收到了」，不说明「可以继续走」
        self.assertEqual(state.on_ack('%d:cancelled' % seq), (seq, 'cancelled'))
        self.assertTrue(state.halted)
        self.assertEqual(state.on_waypoint((0., 0., 0.), (2., 0., 0.), now=2.)[0], [])
        # 只有新 goal 解除
        self.assertTrue(state.on_goal())
        self.assertFalse(state.halted)
        self.assertEqual(len(state.on_waypoint((0., 0., 0.), (2., 0., 0.), now=3.)[0]), 2)

    def test_06_near_waypoint_extends_to_lookahead(self):
        pose, lookahead = (0., 0., 0.), 1.5
        points = far_node.gait_path(pose, (0.3, 0., 0.), lookahead)
        self.assertEqual(len(points), 3)
        self.assertEqual(points[0], pose)
        self.assertGreaterEqual(length(points), lookahead + 0.3 - 1e-9)
        self.assertAlmostEqual(points[-1][0], lookahead + 0.3, places=9)

    def test_07_far_waypoint_stays_two_points(self):
        points = far_node.gait_path((0., 0., 0.), (2.5, 0., 0.), 1.5)
        self.assertEqual(len(points), 2)


class TransformAndFaultTests(unittest.TestCase):
    def test_10_transform_rotates_then_translates(self):
        """转帧自己算（镜像里没有 tf2_sensor_msgs）。纯平移近似在 1 m 外就 >0.2 m 误差。"""
        # 恒等
        self.assertEqual(far_node.apply_transform([(1., 2., 3.)], (0., 0., 0.),
                                                  (0., 0., 0., 1.)), [(1., 2., 3.)])
        # 绕 z 转 90 度再加平移：(1,0,0) -> (0,1,0) -> (5,7,7)
        out = far_node.apply_transform([(1., 0., 0.)], (5., 6., 7.),
                                       (0., 0., math.sqrt(0.5), math.sqrt(0.5)))
        for got, want in zip(out[0], (5., 7., 7.)):
            self.assertAlmostEqual(got, want, places=9)
        # 四元数未归一化也要成立
        self.assertAlmostEqual(far_node.apply_transform([(0., 1., 0.)], (0., 0., 0.),
                                                        (0., 0., 2., 2.))[0][0], -1., places=9)

    def test_11_fault_cancel_tracks_ack_and_times_out(self):
        state = far_node.AdapterState()
        seq, frame = state.request_cancel(now=100., reason='far_waypoint_stale')
        self.assertTrue(state.halted)
        self.assertEqual(cancel_policy.parse_cancel(frame)[:2], (seq, 'all'))
        # 没到超时点：不报
        self.assertIsNone(state.ack_timeout(101., 2.0))
        # 超时且无 ack：报一次 cancel_unacked
        text = state.ack_timeout(103., 2.0)
        self.assertEqual(json.loads(text)['reason'], 'E_FAR_CANCEL_UNACKED')
        self.assertIsNone(state.ack_timeout(110., 2.0))      # 只报一次，不刷屏
        # ack 及时到位则不报
        state2 = far_node.AdapterState()
        seq2, _ = state2.request_cancel(now=0.)
        self.assertEqual(state2.note_ack(cancel_policy.ack_text(seq2, 'cancelled')),
                         (seq2, 'cancelled'))
        self.assertIsNone(state2.ack_timeout(99., 2.0))

    def test_12_new_goal_clears_pending_cancel(self):
        """新 goal ＝ 操作员重新下达任务，上一轮的 ack 欠账不该在正常行驶里冒出来。"""
        state = far_node.AdapterState()
        state.request_cancel(now=0.)
        state.on_goal()
        self.assertIsNone(state.ack_timeout(999., 0.1))
        self.assertFalse(state.halted)


class ContractTests(unittest.TestCase):
    """跨包互检：far_node 自己组字符串（顶层零跨包 import），格式必须与 local_planner 同一契约。"""

    def test_08_cancel_frame_parsed_by_local_policy(self):
        text = far_node.cancel_text(900000001, 'current', 'far_cancel')
        self.assertEqual(cancel_policy.parse_cancel(text), (900000001, 'current', 'far_cancel'))
        with self.assertRaises(ValueError):
            far_node.cancel_text(1, 'bogus')
        with self.assertRaises(ValueError):
            far_node.cancel_text(1, 'current', 'bad;reason')

    def test_09_ack_frame_roundtrip(self):
        for outcome in ('cancelled', 'ack_fail', 'unacked'):
            text = cancel_policy.ack_text(900000002, outcome)
            self.assertEqual(far_node.parse_ack(text), (900000002, outcome))
        self.assertIsNone(far_node.parse_ack('900000002:'))
        self.assertIsNone(far_node.parse_ack('nonsense'))
        state = far_node.AdapterState()
        state.on_cancel(900000003)
        self.assertIsNone(state.on_ack(cancel_policy.ack_text(900000099, 'cancelled')))


if __name__ == '__main__':
    unittest.main()
