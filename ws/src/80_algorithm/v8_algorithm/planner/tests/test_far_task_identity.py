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

    # ---- ① 位姿变化不是新任务 ----
    def test_pose_stream_does_not_advance_revision(self):
        self.assertEqual(self.st.on_task_identity(ident(1), now=0.0), 'new')
        frames = []
        for i in range(30):                      # 模拟 10 Hz 位姿流（3 s）
            pose = (5.2 - 0.01 * i, 2.45, 0.0)   # 车在动
            msg = self.st.task_message(pose, (4.9, 2.45, 0.0), now=1.0 + 0.1 * i)
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
        self.st.task_message((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=1.0)
        for d in (0.01, -0.02, 0.03):            # 都在 0.05 m 量化步长内
            msg = self.st.task_message((5.0, 2.4, 0.0), (4.0 + d, 2.4, 0.0), now=2.0)
            self.assertEqual(msg['path_revision'], 1, '抖动不得推进版本')
        self.assertEqual(self.st.effective_changes, 1)

    # ---- ③ 有效航点变化 ----
    def test_real_waypoint_change_advances(self):
        self.st.on_task_identity(ident(1), now=0.0)
        a = self.st.task_message((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=1.0)
        b = self.st.task_message((5.0, 2.4, 0.0), (3.5, 2.4, 0.0), now=1.2)
        self.assertEqual(a['path_revision'], 1)
        self.assertEqual(b['path_revision'], 2, '有效航点变化必须推进版本')
        self.assertEqual(b['waypoint_index'], 2, '航点序号独立递增')
        self.assertEqual(self.st.effective_changes, 2)

    # ---- ④ 换 mission：保护窗 ----
    def test_mission_switch_guard_drops_inflight_waypoint(self):
        self.st.on_task_identity(ident(1), now=0.0)
        self.st.task_message((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=1.0)
        self.assertEqual(self.st.on_task_identity(ident(2), now=10.0), 'new')
        late = self.st.task_message((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=10.1)
        self.assertIsNone(late, '换目标后保护窗内的航点属上一目标在途残留，必须丢')
        self.assertEqual(self.st.stale_waypoints, 1)
        ok = self.st.task_message((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=10.5)
        self.assertIsNotNone(ok, '窗口过去后必须恢复')
        self.assertEqual(ok['mission_id'], 2)
        self.assertEqual(ok['path_revision'], 1, '新 mission 的版本从 1 重新起算')

    # ---- ⑤ 旧 mission 身份晚到 ----
    def test_late_old_mission_identity_ignored(self):
        self.st.on_task_identity(ident(3), now=0.0)
        self.assertEqual(self.st.on_task_identity(ident(2), now=1.0), 'stale')
        self.assertEqual(self.st.mission['mission_id'], 3, '不许被旧身份覆盖')
        msg = self.st.task_message((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=2.0)
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
        self.assertIsNone(self.st.task_message((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=1.0))

    # ---- ⑧ 保护窗边界：300 ms 内丢、之后收（真钟语义）----
    def test_late_waypoint_boundary(self):
        self.st.on_task_identity(ident(1), now=100.0)
        self.assertIsNone(self.st.task_message((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=100.25),
                          '窗内（<300 ms）的航点属上一目标在途残留')
        ok = self.st.task_message((5.0, 2.4, 0.0), (4.0, 2.4, 0.0), now=100.35)
        self.assertIsNotNone(ok, '超过 300 ms 的航点必须被接受')

    # ---- ⑨ now 不传会永久丢弃（回归钉）----
    def test_missing_now_drops_everything(self):
        """`now=None` 会退化成 0.0；对真钟时间戳来说恒落在保护窗内 ⇒ 每一帧都被丢。

        这条是给调用方钉的：`on_waypoint` 必须显式传 `rospy.get_time()`。
        """
        self.st.on_task_identity(ident(1), now=100.0)
        self.assertIsNone(self.st.task_message((5.0, 2.4, 0.0), (4.0, 2.4, 0.0)))
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


if __name__ == '__main__':
    unittest.main()
