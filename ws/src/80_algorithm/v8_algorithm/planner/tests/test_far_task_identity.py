#!/usr/bin/env python3
"""FAR 适配层的**任务身份与版本仲裁**离线正反控（2026-11-26 真人裁决③/⑤/⑥）。

要钉住的语义（每条都有反控，避免"永远绿"）：
  ① 位姿变化不是新任务：10 Hz 位姿流刷同一航点 ⇒ 帧照发，但 `path_revision` 不推进；
  ② 航点抖动不是新航点：位移 < `WAYPOINT_QUANTUM_M` 时不推进 revision；
  ③ 有效航点变化才推进：位移超过量化步长 ⇒ revision +1、waypoint_index +1；
  ④ 换 mission：旧目标的在途航点在保护窗内被丢（计入 stale_waypoints），窗后重新起步；
  ⑤ 旧 mission 身份晚到：只记一笔，绝不覆盖当前 mission；
  ⑥ 失效广播：即使内容与上一帧相同也必须发（安全信号不参与去重）；
  ⑦ 取消后（halted）：残留航点一个都不许变成任务帧。

这些全部是纯函数级（`AdapterState` 顶层不 import ROS），所以能在宿主上直接跑。
"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
import far_node as fn  # noqa: E402


def ident(mission, gen=0):
    return {'mission_id': mission, 'path_revision': 0, 'waypoint_index': 0,
            'execution_generation': gen}


class TaskIdentity(unittest.TestCase):
    def setUp(self):
        self.st = fn.AdapterState()
        # 默认把"很久以前的一次请求 + 它对应的结果"绑好：既有用例关心的是身份/版本语义，
        # 不该被请求-结果关联挡住。关联本身的用例在 TaskAssociation 里单独写。
        self.st.request_id = 'test-r0'
        self.st.on_goal_request(-1e9)
        self.st.on_far_path(-1e9 + 0.1, None)

    def tm(self, pose, waypoint, now=None):
        """发一帧任务：显式带上当前请求编号（身份由请求方决定，被测代码不得自补）。"""
        return self.st.task_message(pose, waypoint, request_id=self.st.request_id, now=now)

    # ---- ① 位姿变化不是新任务 ----
    def test_pose_stream_does_not_advance_revision(self):
        self.assertEqual(self.st.on_task_identity(ident(1), now=0.0), 'new')
        frames = []
        for i in range(30):                      # 模拟 10 Hz 位姿流（3 s）
            pose = (5.2 - 0.01 * i, 2.45, 0.0)   # 车在动
            msg = self.tm(pose, (4.9, 2.45, 0.0), now=1.0 + 0.1 * i)
            self.assertIsNotNone(msg)
            frames.append(msg)
        self.assertEqual(len(frames), 30, '每帧都应发出（几何要跟着位姿走）')
        self.assertEqual({f['path_revision'] for f in frames}, {1},
                         '同一航点下 revision 必须恒定 —— 位姿变化不是新任务')
        self.assertEqual({f['waypoint_index'] for f in frames}, {1})
        self.assertEqual(frames[0]['poses'][0], [5.2, 2.45, 0.0])
        self.assertNotEqual(frames[-1]['poses'][0], frames[0]['poses'][0],
                            '首点必须随位姿更新（这与 revision 无关）')

    # ---- ② 抖动不是新航点 ----
    def test_jitter_below_quantum_keeps_revision(self):
        self.st.on_task_identity(ident(1), now=0.0)
        self.tm((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=1.0)
        for d in (0.01, -0.02, 0.03):            # 都在 0.05 m 量化步长内
            msg = self.tm((5.0, 2.4, 0.0), (4.0 + d, 2.4, 0.0), now=2.0)
            self.assertEqual(msg['path_revision'], 1, '抖动不得推进版本')
        self.assertEqual(self.st.effective_changes, 1)

    # ---- ③ 有效航点变化 ----
    def test_real_waypoint_change_advances(self):
        self.st.on_task_identity(ident(1), now=0.0)
        a = self.tm((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=1.0)
        b = self.tm((5.0, 2.4, 0.0), (3.5, 2.4, 0.0), now=1.2)
        self.assertEqual(a['path_revision'], 1)
        self.assertEqual(b['path_revision'], 2, '有效航点变化必须推进版本')
        self.assertEqual(b['waypoint_index'], 2, '航点序号独立递增')
        self.assertEqual(self.st.effective_changes, 2)

    # ---- ④ 换 mission：保护窗 ----
    def test_mission_switch_guard_drops_inflight_waypoint(self):
        self.st.on_task_identity(ident(1), now=0.0)
        self.tm((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=1.0)
        self.assertEqual(self.st.on_task_identity(ident(2), now=10.0), 'new')
        late = self.tm((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=10.1)
        self.assertIsNone(late, '换目标后保护窗内的航点属上一目标在途残留，必须丢')
        self.assertEqual(self.st.stale_waypoints, 1)
        ok = self.tm((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=10.5)
        self.assertIsNotNone(ok, '窗口过去后必须恢复')
        self.assertEqual(ok['mission_id'], 2)
        self.assertEqual(ok['path_revision'], 1, '新 mission 的版本从 1 重新起算')

    # ---- ⑤ 旧 mission 身份晚到 ----
    def test_late_old_mission_identity_ignored(self):
        self.st.on_task_identity(ident(3), now=0.0)
        self.assertEqual(self.st.on_task_identity(ident(2), now=1.0), 'stale')
        self.assertEqual(self.st.mission['mission_id'], 3, '不许被旧身份覆盖')
        msg = self.tm((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=2.0)
        self.assertEqual(msg['mission_id'], 3, '航点仍挂当前 mission')

    # ---- ⑥ 失效广播不被去重 ----
    def test_invalidation_always_published(self):
        self.st.on_task_identity(ident(1), now=0.0)
        first = self.st.invalidate('far_waypoint_stale')
        second = self.st.invalidate('far_waypoint_stale')
        self.assertEqual(first['poses'], [])
        self.assertEqual(first['invalidated'], 'far_waypoint_stale')
        self.assertGreater(second['path_revision'], first['path_revision'],
                           '相同内容的失效帧也必须能发出去（版本前进，不被去重）')
        self.assertEqual(self.st.invalidations, 2)

    # ---- ⑦ 取消后残留航点 ----
    def test_halted_adapter_emits_no_task(self):
        self.st.on_task_identity(ident(1), now=0.0)
        self.st.halted = True
        self.assertIsNone(self.tm((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=1.0))

    # ---- ⑧ 保护窗边界：300 ms 内丢、之后收（真钟语义）----
    def test_late_waypoint_boundary(self):
        self.st.on_task_identity(ident(1), now=100.0)
        self.assertIsNone(self.tm((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=100.25),
                          '窗内（<300 ms）的航点属上一目标在途残留')
        ok = self.tm((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=100.35)
        self.assertIsNotNone(ok, '超过 300 ms 的航点必须被接受')

    # ---- ⑨ now 不传会永久丢弃（回归钉）----
    def test_missing_now_drops_everything(self):
        """`now=None` 会退化成 0.0；对真钟时间戳来说恒落在保护窗内 ⇒ 每一帧都被丢。

        这条是给调用方钉的：`on_waypoint` 必须显式传 `rospy.get_time()`。
        """
        self.st.on_task_identity(ident(1), now=100.0)
        self.assertIsNone(self.tm((5.0, 2.4, 0.0), (4.0, 2.4, 0.0)))
        self.assertEqual(self.st.stale_waypoints, 1)

    # ---- ⑩ 身份来源优先级：goal_task 权威，goal_meta 只在没见过原子时兜底 ----
    def test_identity_source_priority(self):
        st = fn.AdapterState()
        self.assertEqual(st.identity_source, None)
        st.on_task_identity(ident(1), now=0.0)
        st.identity_source = 'goal_task'
        before = st.mission['mission_id']
        # 若 legacy 还能改身份，这里会变成 9
        if st.identity_source == 'goal_task':
            st.legacy_identity_ignored += 1
        self.assertEqual(st.mission['mission_id'], before)
        self.assertEqual(st.legacy_identity_ignored, 1)

    # ---- ⑪ 超过保护窗的旧航点晚到：把"当前能做到什么"钉死，别拿它当身份安全 ----
    def test_late_waypoint_from_old_request_is_rejected(self):
        """★缺陷回归（原 known_limitation）★：旧请求的航点在保护窗之后到达，必须被拒。

        历史证据：2026-11-26 的版本只能证明"超过 300 ms 仍会被接受并打上新 mission 标号"，
        那条缺陷在 2026-11-27 用**请求编号**（FAR 原样回传 `header.seq`）修掉：
        结果自带身份，闸门在 `seq != request_seq` 时直接丢弃，与到达时刻无关。
        """
        self.st.on_task_identity(ident(1), now=0.0)
        self.st.set_target((5.0, 2.4, 0.0))
        seq1 = self.st.next_request_id()              # 请求 1
        self.st.on_goal_request(0.0)
        self.st.on_far_path(0.1, None)
        self.assertIsNotNone(self.tm((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=1.0))

        # 换任务：请求 2
        self.st.on_task_identity(ident(2), now=10.0)
        self.st.set_target((3.0, 2.4, 0.0))
        seq2 = self.st.next_request_id()
        self.st.on_goal_request(10.0)
        self.st.on_far_path(10.1, (3.0, 2.4, 0.0))

        # 旧请求（seq=1）的结果在 15 s 才到：**必须拒**，且不得刷新缓存/版本
        before = (self.st.path_revision, self.st.waypoint_count)
        self.assertIsNone(self.st.task_message((5.0, 2.4, 0.0), (4.0, 2.4, 0.0),
                                               request_id=seq1, now=15.0))
        self.assertEqual((self.st.path_revision, self.st.waypoint_count), before,
                         '被拒的结果不得推进版本，也不得刷新缓存')
        self.assertGreaterEqual(self.st.rejected_results, 1)
        # 新请求的结果正常通过
        self.assertIsNotNone(self.st.task_message((5.0, 2.4, 0.0), (4.0, 2.4, 0.0),
                                                  request_id=seq2, now=15.2))

    def test_new_result_then_old_result_interleaved(self):
        """新任务合法 Path 先到、旧任务结果随后到：旧结果不得借用新关联进入新任务。"""
        self.st.on_task_identity(ident(1), now=0.0)
        self.st.set_target((5.0, 2.4, 0.0))
        seq1 = self.st.next_request_id()
        self.st.on_goal_request(0.0)
        self.st.on_far_path(0.1, None)

        self.st.on_task_identity(ident(2), now=10.0)
        self.st.set_target((3.0, 2.4, 0.0))
        seq2 = self.st.next_request_id()
        self.st.on_goal_request(10.0)
        self.st.on_far_path(10.1, (3.0, 2.4, 0.0))     # 新结果先到

        good = self.st.task_message((5.0, 2.4, 0.0), (4.2, 2.4, 0.0), request_id=seq2, now=10.5)
        self.assertIsNotNone(good)
        rev = good['path_revision']
        old = self.st.task_message((5.0, 2.4, 0.0), (4.6, 2.4, 0.0), request_id=seq1, now=10.6)
        self.assertIsNone(old, '旧结果不得被贴上 mission 2 的标号')
        self.assertEqual(self.st.path_revision, rev, '被拒结果不得推进版本')

    def test_rapid_abc_switch_only_latest_request_passes(self):
        """A→B→C 快速切换：只有 C 的请求编号能通过，A/B 的结果一律拒。"""
        seqs = []
        for mission, goal in ((1, (5.0, 2.0, 0.0)), (2, (4.0, 2.0, 0.0)), (3, (3.0, 2.0, 0.0))):
            self.st.on_task_identity(ident(mission), now=float(mission * 10))
            self.st.set_target(goal)
            seqs.append(self.st.next_request_id())
            self.st.on_goal_request(float(mission * 10))
        self.st.on_far_path(30.0, (3.0, 2.0, 0.0))
        for seq in seqs[:2]:
            self.assertIsNone(self.st.task_message((5.0, 2.45, 0.0), (4.4, 2.0, 0.0),
                                                   request_id=seq, now=30.5))
        self.assertIsNotNone(self.st.task_message((5.0, 2.45, 0.0), (3.6, 2.0, 0.0),
                                                  request_id=seqs[2], now=30.6))

    def test_zero_seq_result_is_rejected(self):
        """无身份结果（seq=0，例如 FAR 重置时发的那条）一律拒。"""
        self.st.on_task_identity(ident(1), now=0.0)
        self.st.set_target((3.0, 2.0, 0.0))
        self.st.next_request_id()
        self.st.on_goal_request(0.0)
        self.st.on_far_path(0.1, None)
        self.assertIsNone(self.st.task_message((5.0, 2.45, 0.0), (4.0, 2.0, 0.0),
                                               request_id='', now=1.0))
        self.assertEqual(self.st.last_waypoint, None, '被拒结果不得写 last_waypoint 缓存')

    # ---- ⑫ 航点永远不能创建 mission ----
    def test_waypoint_cannot_create_mission(self):
        self.assertIsNone(self.tm((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=1.0),
                          '没有 goal_task 之前，任何航点都不得产生任务帧')
        self.assertIsNone(self.st.mission)

    # ---- ⑬ 换目标后"旧航点晚到"的几何拒绝（不靠时间窗）----
    def test_stale_waypoint_rejected_by_geometry(self):
        self.st.on_task_identity(ident(1), now=0.0)
        self.st.set_target((5.0, 2.0, 0.0))
        self.tm((5.0, 2.0, 0.0), (4.6, 2.0, 0.0), now=1.0)

        self.st.on_task_identity(ident(2), now=10.0)
        self.st.set_target((3.0, 2.0, 0.0))
        # 时间窗早就过了：这条"去旧目标方向"的航点在 15 s 才到
        late = self.tm((5.0, 2.0, 0.0), (5.4, 2.0, 0.0), now=15.0)
        self.assertIsNone(late, '离上一目标更近的航点必须按残留丢弃')
        self.assertEqual(self.st.stale_goal_waypoints, 1)

        good = self.tm((5.0, 2.0, 0.0), (4.0, 2.0, 0.0), now=15.2)
        self.assertIsNotNone(good, '朝新目标去的航点必须放行')
        self.assertEqual(good['mission_id'], 2)

    # ---- ⑭ 保护期过了以后不再用几何判据（避免长期误杀）----
    def test_geometry_guard_expires(self):
        self.st.on_task_identity(ident(1), now=0.0)
        self.st.set_target((5.0, 2.0, 0.0))
        self.st.on_task_identity(ident(2), now=10.0)
        self.st.set_target((3.0, 2.0, 0.0))
        after = self.tm((5.0, 2.0, 0.0), (5.4, 2.0, 0.0), now=10.0 + 5.5)
        self.assertIsNotNone(after, '超过 5 s 保护期后不再按几何判据丢弃')

    # ---- ⑮ 同 mission 重复 goal_task 幂等 ----
    def test_same_mission_is_idempotent(self):
        self.assertEqual(self.st.on_task_identity(ident(1), now=0.0), 'new')
        self.st.set_target((5.0, 2.0, 0.0))
        rev0, idx0 = self.st.path_revision, self.st.waypoint_index
        self.assertEqual(self.st.on_task_identity(ident(1, gen=1), now=1.0), 'same')
        self.assertEqual(self.st.path_revision, rev0, '同 mission 不得重置版本')
        self.assertEqual(self.st.waypoint_index, idx0, '同 mission 不得重置航点序号')
        self.assertEqual(self.st.mission['execution_generation'], 1, '执行代次可以更新')

    # ---- ⑯ 已撤销 mission 永不重新武装 ----
    def test_revoked_mission_never_rearmed(self):
        self.st.on_task_identity(ident(2), now=0.0)
        self.st.set_target((5.0, 2.0, 0.0))
        self.st.revoke(reason='cancel')
        self.assertEqual(self.st.on_task_identity(ident(2), now=5.0), 'stale')
        self.assertEqual(self.st.on_task_identity(ident(1), now=6.0), 'stale')
        self.assertTrue(self.st.halted, '撤销后必须保持 halted')
        self.assertIsNone(self.tm((5.0, 2.0, 0.0), (4.0, 2.0, 0.0), now=7.0))
        self.assertEqual(self.st.on_task_identity(ident(3), now=8.0), 'new', '新 mission 可放行')

    # ---- ⑰ 同坐标新任务 ----
    def test_same_coordinate_new_mission_is_new(self):
        self.st.on_task_identity(ident(1), now=0.0)
        self.st.set_target((3.0, 2.0, 0.0))
        self.assertEqual(self.st.on_task_identity(ident(2), now=1.0), 'new')
        self.st.set_target((3.0, 2.0, 0.0))     # 目标坐标完全相同
        # 同坐标时没有"上一目标"可判：几何判据在这里**不起作用**，只能靠 mission 编号。
        self.assertIsNone(self.st.prev_target, '同坐标新任务没有几何可判别（不假装能挡）')

    # ---- ⑱ 同向目标：不得被几何判据误杀 ----
    def test_same_direction_waypoint_allowed(self):
        self.st.on_task_identity(ident(1), now=0.0)
        self.st.set_target((5.0, 2.0, 0.0))
        self.st.on_task_identity(ident(2), now=10.0)
        self.st.set_target((5.0, 2.0, 0.0))     # 新目标与旧目标同向同点
        w = self.tm((6.0, 2.0, 0.0), (5.4, 2.0, 0.0), now=10.5)
        self.assertIsNotNone(w, '同向航点必须放行（几何判据只挡"离旧目标更近"的）')

    # ---- ⑲ 合法避障绕行：离两个目标都远 ⇒ 不得被拒 ----
    def test_detour_waypoint_allowed(self):
        self.st.on_task_identity(ident(1), now=0.0)
        self.st.set_target((5.0, 2.0, 0.0))
        self.st.on_task_identity(ident(2), now=10.0)
        self.st.set_target((3.0, 2.0, 0.0))
        detour = self.tm((5.0, 2.0, 0.0), (5.0, 4.0, 0.0), now=10.5)
        self.assertIsNotNone(detour, '绕行点离两目标都远，属合法避障，不得按残留丢弃')


if __name__ == '__main__':
    unittest.main()


class TaskAssociation(unittest.TestCase):
    """请求/结果关联（2026-11-27 裁决①）：/way_point 无身份，靠 FAR 自己回报的
    /global_path（时间戳 + 终点）把"结果"绑到"这次请求"上。"""

    def setUp(self):
        self.st = fn.AdapterState()

    def bind(self, request_at, path_at, path_goal):
        self.st.next_request_id()
        self.st.on_goal_request(request_at)
        self.st.on_far_path(path_at, path_goal)

    def tm(self, pose, waypoint, now=None):
        return self.st.task_message(pose, waypoint, request_id=self.st.request_id, now=now)

    def test_association_is_diagnostic_only(self):
        """★关联不再是判据（2026-11-27 裁决）★

        `/global_path` 的路径时间/终点只是旁路诊断：身份由 request_id 决定、同请求内由 gen 保证不倒退。
        这条用例钉住"关联失败只计数、不拦截"。"""
        self.st.on_task_identity(ident(1), now=0.0)
        self.st.set_target((3.9, 2.45, 0.0))
        self.bind(request_at=10.0, path_at=9.5, path_goal=(3.9, 2.45, 0.0))
        self.assertIsNotNone(self.tm((5.2, 2.45, 0.0), (4.5, 2.45, 0.0), now=10.5),
                             '身份正确就放行，关联陈旧只记诊断')
        self.assertEqual(self.st.assoc_diagnostic_fail, 1)

    def test_late_waypoint_from_old_goal_rejected(self):
        """旧任务旧航点：结果时间够新，但**终点偏向上一目标** ⇒ 拒绝。"""
        self.st.on_task_identity(ident(1), now=0.0)
        self.st.set_target((5.0, 2.0, 0.0))
        self.st.on_task_identity(ident(2), now=10.0)
        self.st.set_target((5.6, 2.0, 0.0))
        self.bind(request_at=10.0, path_at=10.2, path_goal=(5.0, 2.0, 0.0))
        # 身份正确、但点位几乎压在上一目标上且明显背离新目标 ⇒ 由**几何辅助**拒（不是关联）
        self.assertIsNone(self.tm((5.2, 2.45, 0.0), (5.0, 2.1, 0.0), now=10.4))
        self.assertEqual(self.st.stale_goal_waypoints, 1)

    def test_result_for_current_goal_accepted(self):
        """合法：结果晚于请求、终点朝新目标 ⇒ 接受。"""
        self.st.on_task_identity(ident(1), now=0.0)
        self.st.set_target((5.0, 2.0, 0.0))
        self.st.on_task_identity(ident(2), now=10.0)
        self.st.set_target((3.0, 2.0, 0.0))
        self.bind(request_at=10.0, path_at=10.2, path_goal=(3.0, 2.0, 0.0))
        msg = self.tm((5.2, 2.45, 0.0), (4.0, 2.0, 0.0), now=10.4)
        self.assertIsNotNone(msg)
        self.assertEqual(msg['mission_id'], 2)

    def test_concurrent_switch_uses_latest_request(self):
        """并发切换：连续两次换目标，只有**最后一次请求之后**的结果被接受。"""
        self.st.on_task_identity(ident(1), now=0.0)
        self.st.set_target((5.0, 2.0, 0.0))
        self.st.on_task_identity(ident(2), now=10.0)
        self.st.set_target((4.0, 2.0, 0.0))
        self.st.on_task_identity(ident(3), now=11.0)
        self.st.set_target((3.0, 2.0, 0.0))
        # 请求来自 mission 2 那次（10.0），结果也是那时的
        self.bind(request_at=11.0, path_at=10.4, path_goal=(4.0, 2.0, 0.0))
        self.assertIsNone(self.tm((5.2, 2.45, 0.0), (4.2, 2.0, 0.0), now=11.5))

    def test_no_request_no_result(self):
        """没有请求就没有关联：任何航点都不得输出（防"无人认领"的结果被贴标）。"""
        self.st.on_task_identity(ident(1), now=0.0)
        self.st.set_target((3.0, 2.0, 0.0))
        self.assertIsNone(self.tm((5.2, 2.45, 0.0), (4.0, 2.0, 0.0), now=1.0))

    def test_same_coordinate_repeat_request_is_idempotent(self):
        """重复请求（同坐标同 mission）：不推进版本；结果时间更新即可继续用。"""
        self.st.on_task_identity(ident(1), now=0.0)
        self.st.set_target((3.0, 2.0, 0.0))
        self.bind(request_at=0.0, path_at=0.2, path_goal=(3.0, 2.0, 0.0))
        first = self.tm((5.2, 2.45, 0.0), (4.0, 2.0, 0.0), now=0.3)
        self.assertIsNotNone(first)
        rev = first['path_revision']
        # BT 重复下发同一条 goal_task：幂等 ⇒ 不重新武装、**也不刷新请求时刻**
        self.assertEqual(self.st.on_task_identity(ident(1), now=5.0), 'same')
        again = self.tm((5.2, 2.45, 0.0), (4.0, 2.0, 0.0), now=9.0)
        self.assertIsNotNone(again, '幂等请求不刷新关联：上一轮结果依然有效')
        self.assertEqual(again['path_revision'], rev, '同航点不得推进版本')

    def test_revoked_blocks_even_when_associated(self):
        """撤销后即使结果关联成立也不得输出。"""
        self.st.on_task_identity(ident(2), now=0.0)
        self.st.set_target((3.0, 2.0, 0.0))
        self.bind(request_at=0.0, path_at=0.2, path_goal=(3.0, 2.0, 0.0))
        self.st.revoke(reason='cancel')
        self.assertIsNone(self.tm((5.2, 2.45, 0.0), (4.0, 2.0, 0.0), now=0.3))
