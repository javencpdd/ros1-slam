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



def ready_state():
    """带有效 mission + 有效请求的状态：路径形状/状态类用例只关心语义，
    但必须满足身份闸门，否则它们测到的是"被拦"而不是"形状"。"""
    st = far_node.AdapterState()
    st.on_task_identity({'mission_id': 1, 'execution_generation': 0}, now=0.0)
    st.next_request_seq()
    st.on_goal_request(0.0)
    st.on_far_path(0.1, None)
    return st


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
        path_points, gait_points, text, _payload = state.on_waypoint(None, (1., 0., 0.), seq=state.request_seq)
        self.assertEqual(path_points, [])
        self.assertEqual(gait_points, [])
        self.assertEqual(json.loads(text)['reason'], 'E_FAR_NO_POSE')
        path_points, gait_points, text, _payload = state.on_waypoint((0., 0., 0.), None, seq=state.request_seq)
        self.assertEqual(path_points, [])
        self.assertEqual(gait_points, [])
        self.assertEqual(json.loads(text)['reason'], 'E_FAR_NO_WAYPOINT')
        self.assertEqual(json.loads(text)['points'], 0)

    def test_05_cancel_keeps_halted_until_new_goal(self):
        state = ready_state()
        seq = state.on_cancel()
        self.assertTrue(state.halted)
        path_points, gait_points, text, _payload = state.on_waypoint((0., 0., 0.), (2., 0., 0.), now=1., seq=state.request_seq)
        self.assertEqual(path_points, [])
        self.assertEqual(gait_points, [])
        self.assertEqual(json.loads(text)['reason'], 'E_FAR_HALTED')
        # ack 到位也不解除——ack 只说明「取消收到了」，不说明「可以继续走」
        self.assertEqual(state.on_ack('%d:cancelled' % seq), (seq, 'cancelled'))
        self.assertTrue(state.halted)
        self.assertEqual(state.on_waypoint((0., 0., 0.), (2., 0., 0.), now=2., seq=state.request_seq)[0], [])
        # 只有新 goal 解除
        self.assertTrue(state.on_goal())
        self.assertFalse(state.halted)
        self.assertEqual(len(state.on_waypoint((0., 0., 0.), (2., 0., 0.), now=3., seq=state.request_seq)[0]), 2)

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
        # 切换点在 lookahead + margin（1.5 + 0.3 = 1.8），不是 lookahead 本身：
        # 1.6 m 的航点仍要补，因为扫描到 1.6 m 就断了、看不到 1.5 m 以外
        self.assertEqual(len(far_node.gait_path((0., 0., 0.), (1.6, 0., 0.), 1.5)), 3)
        self.assertEqual(len(far_node.gait_path((0., 0., 0.), (1.8, 0., 0.), 1.5)), 2)
        # 补点的总长固定为 lookahead + margin，与航点远近无关
        for near in (0.3, 1.6):
            points = far_node.gait_path((0., 0., 0.), (near, 0., 0.), 1.5)
            self.assertAlmostEqual(length(points), 1.8, places=9)

    def test_13_main_path_never_carries_lookahead_point(self):
        """`/global_path` 的末点必须就是航点本身；延长点只能走 `/v8/gait_guard/path`。

        `path_goal_node.py` 取 `poses[-1]` 当局部目标。延长点混进去，局部目标就被推到
        航点之外——车会越过 FAR 判断为安全的落点继续走。这条守的是「目标语义不被打折」。
        """
        state = ready_state()
        # 近航点（0.3 m < 前视 1.5 m）：主路径仍两点，步态路径补第三点
        path_points, gait_points, text, _payload = state.on_waypoint((0., 0., 0.), (0.3, 0., 0.), now=1., seq=state.request_seq)
        self.assertEqual(len(path_points), 2)
        self.assertEqual(path_points[-1], (0.3, 0., 0.))
        self.assertEqual(len(gait_points), 3)
        self.assertGreater(gait_points[-1][0], 0.3)
        # 两个数分开报，别混用一个字段
        payload = json.loads(text)
        self.assertEqual((payload['points'], payload['gait_points']), (2, 3))
        # 远航点（2.5 m）：两条话题内容逐字相同
        path_points, gait_points, _text, _payload = state.on_waypoint((0., 0., 0.), (2.5, 0., 0.), now=2., seq=state.request_seq)
        self.assertEqual(path_points, gait_points)


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


class ArrivalTests(unittest.TestCase):
    """到达判据（计划 §4.5 甲案）：适配器自己判，不转发上游 `/far_reach_goal_status`。

    判据面是 `hypot(r_xy, z_tol)`，不是「xy 判一次、z 再判一次」——下面 T15 把这个等价性
    钉死，免得后人照着两段式再写一个门限出来。
    """

    def test_T14_radius_is_hypot_of_two_limits(self):
        self.assertAlmostEqual(far_node.arrival_radius(0.25, 0.30),
                               math.hypot(0.25, 0.30), places=12)
        self.assertAlmostEqual(far_node.arrival_radius(), math.hypot(0.25, 0.30), places=12)
        self.assertLess(far_node.arrival_radius(), 0.5)   # 比上游 converge_dist 更严
        # 同点：到达
        self.assertTrue(far_node.is_arrived((1., 2., 0.5), (1., 2., 0.5)))
        # 跨层同 xy、z 差 3.0：不得到达（主测 1）
        self.assertFalse(far_node.is_arrived((1., 2., 0.5), (1., 2., 3.5)))
        # ★边界改按**逐轴**判（2026-11-26 裁决④）★：arrival_radius 只留作读数上报，
        # 不再是判据本体——用它当判据会把水平 0.38 m 判成到达。
        self.assertTrue(far_node.is_arrived((0., 0., 0.), (0.25, 0., 0.)))
        self.assertFalse(far_node.is_arrived((0., 0., 0.), (0.25 + 1e-6, 0., 0.)))
        self.assertTrue(far_node.is_arrived((0., 0., 0.), (0., 0., 0.30)))
        self.assertFalse(far_node.is_arrived((0., 0., 0.), (0., 0., 0.30 + 1e-6)))

    def test_T15_arrival_is_per_axis(self):
        """★到达判据逐轴判（2026-11-26 裁决④）★

        旧写法 `|p-g|_3d ≤ hypot(0.25,0.30)=0.39` 会把 **水平 0.38 m** 的点判成"到了"，
        等于用竖向容差补贴水平误差。现在要求：水平 ≤0.25 且 |Δz| ≤0.30，两条都满足才算到。
        """
        r_xy, z_tol = 0.25, 0.30
        # 逐轴边界内
        for a in (0.0, 0.1, 0.25):
            for b in (-0.30, -0.1, 0.0, 0.1, 0.30):
                self.assertTrue(far_node.is_arrived((0., 0., 0.), (a, 0., b), r_xy, z_tol),
                                'a=%s b=%s 在逐轴容差内' % (a, b))
        # 水平越界：3D 距离 0.38 < 0.3905，但水平 0.38 > 0.25 ⇒ 不算到达（旧判据会误判）
        point = (0.38, 0., 0.)
        self.assertLess(math.hypot(point[0], point[1]), far_node.arrival_radius(r_xy, z_tol))
        self.assertFalse(far_node.is_arrived((0., 0., 0.), point, r_xy, z_tol),
                         '水平超差不得靠竖向余量补贴')
        # 竖向越界单独测
        self.assertFalse(far_node.is_arrived((0., 0., 0.), (0., 0., 0.31), r_xy, z_tol))
        self.assertTrue(far_node.is_arrived((0., 0., 0.), (0., 0., 0.30), r_xy, z_tol))
        # 水平两轴合成：0.2/0.2 的 dxy=0.283 > 0.25 ⇒ 不到
        self.assertFalse(far_node.is_arrived((0., 0., 0.), (0.2, 0.2, 0.), r_xy, z_tol))

    def test_T16_new_goal_resets_arrival(self):
        state = far_node.AdapterState()
        state.on_goal((0., 0., 0.))
        self.assertEqual(state.poll_arrival((0., 0., 0.)), (True, True))
        # 同一个位姿再问一次：不再算「变化」，避免 5 Hz 刷同一帧
        self.assertEqual(state.poll_arrival((0., 0., 0.)), (True, False))
        # 新目标把 arrived 归零：旧任务的 true 不许留在 latch 上
        state.on_goal((5., 0., 0.))
        self.assertFalse(state.arrived)
        self.assertEqual(state.poll_arrival((0., 0., 0.)), (False, False))
        self.assertEqual(state.poll_arrival((5., 0., 0.)), (True, True))

    def test_T17_no_target_no_pose_or_halted_is_silent(self):
        """没有目标 / 没有位姿 / 已被取消时不许发信号。

        取消后若还按残留位姿发 true，等于取消没取消——BT 的 `arrival_signal` 会把它当成到达。
        """
        state = far_node.AdapterState()
        self.assertEqual(state.poll_arrival((0., 0., 0.)), (None, False))
        state.on_goal((1., 1., 1.))
        self.assertEqual(state.poll_arrival(None), (None, False))
        self.assertEqual(state.poll_arrival((1., 1., 1.)), (True, True))
        state.on_cancel()
        self.assertFalse(state.arrived)
        self.assertEqual(state.poll_arrival((1., 1., 1.)), (None, False))
        with self.assertRaises(ValueError):
            far_node.is_arrived(None, (0., 0., 0.))


class BtHandshakeTests(unittest.TestCase):
    """暖机占位帧必须让 BT 继续等，而不是被判拒绝。

    判据在 C++ 侧：`bt_console.cpp:161-165` 的 `transient_plan_reason` 是**硬编码白名单**，
    不在里面的 reason 会让 `WaitForPath` 直接 FAILURE（`:324-332`，日志
    `[bt] global path refused: <reason>`），任务当场结束。所以这里两层都钉：
    自己发的名字在白名单内，且白名单本身没被上游改掉。
    """

    def test_T18_warmup_reason_is_bt_transient(self):
        payload = json.loads(far_node.status_json(False, 0, far_node.WARMUP_PLAN_REASON,
                                                  detail='E_FAR_WARMUP'))
        self.assertIn(payload['reason'], far_node.BT_TRANSIENT_REASONS)
        self.assertTrue(payload['reason'])
        # 适配器自己的诊断名不能丢，否则日志里查不到「为什么在暖机」
        self.assertEqual(payload['detail'], 'E_FAR_WARMUP')

    def test_T19_transient_list_matches_bt_console(self):
        source = HERE.parents[4] / 'ws/src/90_platform/lite3_bt/src/bt_console.cpp'
        if not source.is_file():
            self.skipTest('bt_console.cpp not found: %s' % source)
        body = source.read_text(encoding='utf-8')
        start = body.find('bool transient_plan_reason')
        self.assertGreaterEqual(start, 0, 'transient_plan_reason not found')
        chunk = body[start:start + 400]
        for reason in far_node.BT_TRANSIENT_REASONS:
            self.assertIn('"%s"' % reason, chunk, '%s 不在 BT 白名单里' % reason)


    def test_T20_reach_reads_do_not_write_path_status(self):
        """上游「还没到」只能当附加字段，不能把路径状态改成失败。

        `/global_path/status` 是 BT `WaitForPath` 的判据：`ok=false` 加不在白名单的
        reason 会让它当场判路径失败（`bt_console.cpp:324-332`）。而「还没到」是常态，
        与「这条路能不能用」无关——两件事混在一个字段上，BT 就会在每次上游报未到达时
        拒一次路径。
        """
        state = ready_state()
        _path, _gait, text, _payload = state.on_waypoint((0., 0., 0.), (1., 0., 0.), now=1., seq=state.request_seq)
        payload = json.loads(text)
        self.assertIsNone(payload['far_reach'])
        self.assertEqual(payload['reason'], 'ok')
        state.far_reach = False
        _path, _gait, text, _payload = state.on_waypoint((0., 0., 0.), (1., 0., 0.), now=1.5, seq=state.request_seq)
        payload = json.loads(text)
        self.assertIs(payload['far_reach'], False)
        self.assertEqual(payload['reason'], 'ok')
        self.assertTrue(payload['ok'])


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
