#!/usr/bin/env python3
"""Publish 3D global Path waypoints to the local planner, one at a time.

为什么不只发终点：local_native 是**步进式**的 —— 收到一个 goal 走 max_step_m（默认 0.30 m）
就置 segment_complete（reason='segment_complete'），之后要等下一个目标才继续。
原实现只把 Path 的 `poses[-1]` 发出去，于是车走完一步 0.30 m 就停住，
BT 的 wait_arrival 一直空等到超时 —— 实测 travel 恰好 0.308 m / 0.307 m（两路一致）。

本节点按路径顺序逐点下发：当前 waypoint 到达（或超时）后推进到下一个。
不重复重发同一点，因为 local 的 accept_goal 每次都把 generation +1，会让在飞结果作废。
"""
import json
import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String


def accepts_path_stamp(cancelled_at, path_stamp):
    """取消之后是否接受这条路径。两个参数都是秒（float 或 None）。

    返回 (accepted, reason)。判定只看时间序，不依赖 ROS 类型，便于离线单测。
    fail-closed：调用方拿不到 stamp 时按拒绝处理 —— 无法区分新旧就不能重放。
    """
    if cancelled_at is None:
        return True, 'no_cancel'
    if not path_stamp:
        return False, 'no_stamp_fail_closed'
    if path_stamp <= cancelled_at:
        return False, 'stale_after_cancel'
    return True, 'newer_than_cancel'


def make_goal(pose, generation=None):
    """把代次写进 header.seq —— 这是**任务身份**，不是本地计数器。

    只靠本地 generation 计数无法防住"旧任务带着更新 timestamp 重新下发"：
    下游看不出两者区别。把代次编码进消息后，接收端可以按身份拒绝更老的代次。
    """
    goal = PoseStamped()
    goal.header = pose.header
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = goal.header.frame_id or 'map'
    if generation is not None:
        goal.header.seq = int(generation)
    goal.pose = pose.pose
    return goal


class PathGoal(object):
    def __init__(self):
        self.path_topic = rospy.get_param('~path_topic', '/v8/3d/global/path')
        self.goal_topic = rospy.get_param('~goal_topic', '/v8/local/goal')
        self.status_topic = rospy.get_param('~status_topic', '/v8/3d/path_goal/status')
        self.pose_topic = rospy.get_param('~pose_topic', '/v8/local/pose')
        # 取消必须贯穿到这一层：本节点是 local 目标的直接下发者，BT 收到 cancel 后
        # 若这里不停手，latched 的全局路径会让它继续逐点下发，local 被重新驱动。
        self.cancel_topic = rospy.get_param('~cancel_topic', '/v8/local/cancel')
        self.arrive_m = float(rospy.get_param('~arrive_m', 0.30))
        self.advance_timeout_s = float(rospy.get_param('~advance_timeout_s', 8.0))
        # ★单段最小目标距离★：local 是"给一个目标走一段"，目标离当前位姿太近时它不产生速度
        # （实测：路径点间距 0.1 m，发出去之后 `mux reason=track` 而 cmd 恒零，车一动不动）。
        # 这里要求推进后的目标至少离当前位姿这么远；接近终点时才允许更近（见 tick 的兜底）。
        self.min_goal_m = float(rospy.get_param('~min_goal_m', 0.30))
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.poses = []
        self.index = 0
        self.published_at = None
        self.pose = None
        # 取消时刻：此后只接受 stamp 更新的 path，杜绝旧路径重放
        self.cancelled_at = None
        # 任务代次：每接受一条路径 +1（供审计"是否重放"）；被拒的旧路径单独计数
        self.accepted_generation = 0
        self.rejected_stale = 0
        self.last_reject_reason = None
        # 上一次真正发出去的局部目标（用于抑制重复下发；见 publish_current 的注释）
        self.last_goal_key = None   # (index, xyz)：见 publish_current 的去重边界

        rospy.Subscriber(self.path_topic, Path, self.on_path, queue_size=10)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.on_pose, queue_size=10)
        rospy.Subscriber(self.cancel_topic, String, self.on_cancel, queue_size=10)
        # ★定位失效＝任务作废（真人裁决 2026-11-26）★
        # 恢复后不得自行重新武装：这里清空活动路径，并把**当前代次**记为"作废"，
        # 之后即使全局规划器带更新的时间戳重发同一代次路径，也必须拒绝（任务身份看代次，不看时间戳）。
        # ★作废标记必须与"拒绝判据"用同一套编号★：on_path 里比较的是 path.header.seq（BT 任务代次），
        # 所以这里也记 path 的代次，不能记本地计数器 accepted_generation（那是 path_goal 自己的自增）。
        self.dead_generation = None
        self.last_path_seq = 0
        # ★任务身份（meta 契约）★：path_meta -> 配对 -> goal_meta。
        # mission_id 是**撤销判据**：故障撤销后必须记住 revoked_mission，任何 <= 它的路径都不许复活。
        self.pending_meta = None
        self.task_meta = None
        self.revoked_mission = -1
        self.goal_meta_pub = rospy.Publisher(
            rospy.get_param('~goal_meta_topic', '/v8/local/goal_meta'), String, queue_size=10)
        # ★原子任务通道★：给 local 的身份+位姿同一条消息
        self.goal_task_pub = rospy.Publisher(
            rospy.get_param('~goal_task_topic', '/v8/local/goal_task'), String, queue_size=10)
        rospy.Subscriber(rospy.get_param('~path_meta_topic', '/v8/3d/global/path_meta'),
                         String, self.on_path_meta, queue_size=10)
        # ★原子路径（首选）★：身份与点列同一条消息
        rospy.Subscriber(rospy.get_param('~path_task_topic', '/v8/3d/global/path_task'),
                         String, self.on_path_task, queue_size=10)
        try:
            from std_msgs.msg import Bool
            rospy.Subscriber('/v8/local/localization_ok', Bool, self.on_localization, queue_size=5)
        except Exception:
            pass
        rospy.Timer(rospy.Duration(0.2), self.tick)

    def on_pose(self, message):
        self.pose = message.pose.position

    def on_cancel(self, message):
        """立即清空活动路径；此后只有 stamp 更新的 path 才能重新武装。

        ★原子模式下取消＝任务撤销★（真人 2026-11-26 第 4 条）：撤销判据只能用
        mission_id，不能靠时间戳 —— 旧任务重放时 timestamp 一定是新的，仅凭 stamp
        区分不开"取消后的合法新任务"与"旧任务复活"。所以这里把当前 mission 记进
        `revoked_mission`，之后 `mission_id <= 它` 的 path_task 一律拒绝。
        """
        dropped = len(self.poses)
        self.cancelled_at = rospy.Time.now()
        if getattr(self, 'task_meta', None):
            self.revoked_mission = max(int(getattr(self, 'revoked_mission', -1)),
                                       int(self.task_meta.get('mission_id', -1)))
        self.poses = []
        self.index = 0
        self.last_goal_key = None      # 取消后同坐标的新任务不得被去重吃掉
        self.published_at = None
        self.status_pub.publish(String(data=json.dumps({
            'ok': True, 'cancelled': True, 'dropped_poses': dropped,
            'reason': message.data[:80]})))

    def on_path_task(self, message):
        """★原子控制通道（唯一权威）★：`{identity..., poses:[[x,y,z],...]}`。

        语义（真人 2026-11-26 第 3 条）：
          · 完全重复（identity 与内容都相同）⇒ 什么都不做，不重置、不重发；
          · 内容变化 ⇒ 必须带**合法的新 revision**（回退/相同皆拒）；
          · 新路径的进度**按几何重算**（`_first_unreached`），不盲目沿用旧 index——
            沿用旧 index 会在路径变短/换条时把目标指到已到达点或越界。
        """
        try:
            value = json.loads(message.data)
            raw_poses = value.pop('poses')
            mission = int(value.get('mission_id', 0))
            revision = int(value.get('path_revision', 0))
        except Exception:
            self.status_pub.publish(String(data=json.dumps({'ok': False, 'reason': 'path_task_malformed'})))
            return
        if mission <= int(getattr(self, 'revoked_mission', -1)):
            # ★拒绝回执也要带撤销线★：验收探针要靠它选一个"在撤销线之上"的 mission 去播种，
            # 否则只能猜编号 —— 猜小了会一直撞 task_revoked（看着像"系统锁死"）。
            self.status_pub.publish(String(data=json.dumps({
                'ok': False, 'reason': 'task_revoked', 'mission_id': mission,
                'revoked_mission': int(getattr(self, 'revoked_mission', -1)),
                'mission': (getattr(self, 'task_meta', None) or {}).get('mission_id')})))
            return
        # 版本回退拒绝（同一 mission 内 revision 只能前进）
        cur = getattr(self, 'task_meta', None)
        if cur is not None and int(cur.get('mission_id', -1)) >= 0:
            cur_mission = int(cur.get('mission_id', -1))
            if mission < cur_mission:
                # ★晚到的旧 mission★：没有撤销线也要拒 —— 否则"取消过的任务在下个任务之后
                # 才到"会重新驱动 local，而撤销线只覆盖 cancel/loc-loss 两条路径。
                self.status_pub.publish(String(data=json.dumps({
                    'ok': False, 'reason': 'stale_mission', 'mission_id': mission,
                    'mission': cur_mission,
                    'revoked_mission': int(getattr(self, 'revoked_mission', -1)),
                    'current': cur_mission})))
                return
            if mission == cur_mission and revision < int(cur.get('path_revision', 0)):
                self.status_pub.publish(String(data=json.dumps({
                    'ok': False, 'reason': 'stale_revision', 'revision': revision,
                    'mission_id': mission, 'mission': cur_mission,
                    'revoked_mission': int(getattr(self, 'revoked_mission', -1)),
                    'current': int(cur.get('path_revision', 0))})))
                return
        # 完全重复：identity + 内容都相同 ⇒ 不动
        signature = (mission, revision, json.dumps(raw_poses))
        if getattr(self, 'task_signature', None) == signature and self.poses:
            return
        self.task_signature = signature
        # ★完整载荷★：make_goal 会读 pose.header，缺 header 会 AttributeError（实测会静默不发布）
        self.poses = []
        for p in raw_poses:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = rospy.Time.now()
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(p[0]), float(p[1]), float(p[2])
            ps.pose.orientation.w = 1.0
            self.poses.append(ps)
        self.task_meta = {k: int(value.get(k, 0)) for k in
                          ('mission_id', 'path_revision', 'waypoint_index', 'execution_generation')}
        self.accepted_generation += 1
        self.last_path_seq = mission
        # ★单一控制通道★：原子路径生效后，legacy 的 Path/path_meta 不再改控制状态
        self.atomic_mode = True
        if not self.poses:
            self.status_pub.publish(String(data=json.dumps({'ok': False, 'reason': 'empty_path'})))
            return
        self.index = self._first_unreached()      # 按几何重算，不沿用旧 index
        self.last_goal_key = None
        self.publish_current()

    def on_path_meta(self, message):
        """legacy 配对通道：原子模式下不再使用（跨 topic 无保序）。"""
        if getattr(self, 'atomic_mode', False):
            return
        try:
            self.pending_meta = json.loads(message.data)
        except Exception:
            self.pending_meta = None

    def on_localization(self, message):
        if message.data:
            return
        if self.poses:
            self.status_pub.publish(String(data=json.dumps({
                'ok': False, 'reason': 'localization_lost',
                'dropped_poses': len(self.poses), 'generation': self.accepted_generation})))
        self.dead_generation = self.last_path_seq
        # ★按 mission_id 记撤销线★：之后任何 mission_id <= 它的路径都不许复活
        if self.task_meta:
            self.revoked_mission = max(self.revoked_mission,
                                       int(self.task_meta.get('mission_id', -1)))
        self.poses = []
        self.index = 0
        self.last_goal_key = None
        self.published_at = None

    def on_path(self, message):
        if getattr(self, 'atomic_mode', False):
            # ★原子模式：legacy Path 只当可视化输入，不改控制状态★（三路写同一状态＝结构冲突）
            self.status_pub.publish(String(data=json.dumps({'ok': True, 'legacy_ignored': True,
                                                            'poses': len(message.poses)})))
            return
        # ★任务身份优先★：meta 与 path 配对后，撤销判据看 mission_id（不看 seq / timestamp）
        meta = getattr(self, 'pending_meta', None)
        self.pending_meta = None
        if meta is not None:
            if int(meta.get('mission_id', -1)) <= int(getattr(self, 'revoked_mission', -1)):
                self.status_pub.publish(String(data=json.dumps({
                    'ok': False, 'reason': 'task_revoked', 'meta': meta})))
                return
            self.task_meta = meta
        # 同一代次的旧任务：无论时间戳多新，都不许复活
        if (self.dead_generation is not None
                and int(getattr(message.header, 'seq', 0) or 0) == int(self.dead_generation)):
            self.status_pub.publish(String(data=json.dumps({
                'ok': False, 'reason': 'task_dead', 'generation': self.dead_generation})))
            return
        if not message.poses:
            self.status_pub.publish(String(data=json.dumps({'ok': False, 'reason': 'empty_path'})))
            return
        stamp_obj = message.header.stamp
        stamp = stamp_obj.to_sec() if stamp_obj else 0.0
        cancelled = self.cancelled_at.to_sec() if self.cancelled_at is not None else None
        accepted, why = accepts_path_stamp(cancelled, stamp)
        if not accepted:
            self.rejected_stale += 1
            self.last_reject_reason = why
            self.status_pub.publish(String(data=json.dumps({
                'ok': False, 'reason': why, 'path_stamp': round(stamp, 3),
                'cancelled_at': None if cancelled is None else round(cancelled, 3),
                'rejected_stale': self.rejected_stale})))
            return
        if self.cancelled_at is not None:
            self.cancelled_at = None
        self.accepted_generation += 1
        try:
            self.last_path_seq = int(getattr(message.header, 'seq', 0) or 0)
        except Exception:
            self.last_path_seq = 0
        self.poses = list(message.poses)
        # ★2026-11-25 修★：原先无条件 `self.index = 0` 再发 poses[0]。
        # 对 support 的完整路径，poses[0] 恰好是当前位姿，发出去等于"原地不动"，
        # 由 tick 推进到下一个点；但 FAR 给的是**两点滚动路径**（当前位姿 + 航点），
        # 每秒重发一次 ⇒ 每秒都把 local 重置成"去我站的地方"，于是 local 输出恒零、
        # 车一步不走（实测 32 条路径 → 64 个局部目标，在 (0,-3.06) 与 (1,0) 之间交替）。
        # 正解：从"第一个还没到达的点"开始，绝不把已到达的点当目标下发。
        self.index = self._first_unreached()
        self.publish_current()

    def _first_unreached(self):
        """返回第一个"还没到达"的点下标；全都已在半径内时退回最后一个点。

        判据用与 tick 相同的 arrive_m，保证"跳过"与"推进"是同一把尺子——
        两处若用不同阈值，会出现跳过点与推进点打架、目标在两处来回跳。
        """
        if not self.poses or self.pose is None:
            return 0
        for i, pose in enumerate(self.poses):
            p = pose.pose.position
            if math.hypot(p.x - self.pose.x, p.y - self.pose.y) > self.arrive_m:
                return i
        return len(self.poses) - 1

    def remaining(self):
        if not self.poses or self.pose is None:
            return None
        p = self.poses[self.index].pose.position
        return math.hypot(p.x - self.pose.x, p.y - self.pose.y)

    def publish_current(self):
        # ★任务代次，不是"第几条路径"★：同一任务的所有滚动重发必须同 seq，
        # 否则 local 每帧都当新任务重置代次 ⇒ 永远不执行（实测 seq 16→23 连发、cmd 恒 0）。
        _seq = getattr(self, 'last_path_seq', 0)
        goal = make_goal(self.poses[self.index],
                         _seq if _seq else self.accepted_generation)
        xyz = (round(goal.pose.position.x, 6), round(goal.pose.position.y, 6),
               round(goal.pose.position.z, 6))
        # ★去重边界（2026-11-25 三修）★：键必须含 index，不能只看坐标——
        #   ①同一 index 且坐标未变（滚动路径每秒重发）⇒ 不发，避免 local 反复重开跟踪；
        #   ②index 前进但坐标恰好相同（路径里连续同点）⇒ 必须发，否则"同航点无法连续步进"；
        #   ③取消时把键清空（见 on_cancel）⇒ 取消后同坐标的新任务照样能下发。
        key = (self.index, xyz)
        if key == self.last_goal_key:
            return
        self.last_goal_key = key
        # ★先发 goal_meta、再发 goal★：把 mission/revision/waypoint 传下去（不依赖 header.seq）
        if getattr(self, 'task_meta', None):
            meta = dict(self.task_meta)
            meta['waypoint_index'] = int(self.index)
            meta['path_revision'] = int(self.task_meta.get('path_revision', 0))
            self.goal_meta_pub.publish(String(data=json.dumps(meta, ensure_ascii=False)))
            # ★原子任务★：身份 + 位姿一条消息（跨话题无保序，配对不可靠）
            task = dict(meta)
            task['x'] = goal.pose.position.x
            task['y'] = goal.pose.position.y
            task['z'] = goal.pose.position.z
            if hasattr(self, 'goal_task_pub') and self.goal_task_pub is not None:
                self.goal_task_pub.publish(String(data=json.dumps(task, ensure_ascii=False)))
        self.goal_pub.publish(goal)
        # ★墙钟★：本节点没有 /clock 时 `rospy.Time.now()` 恒为 0，超时判据永远不成立，
        # 于是"卡住就往前推一格"的兜底彻底失效（实测 index 冻结 139 s 不动）。
        self.published_at = time.monotonic()
        rest = self.remaining()
        self.status_pub.publish(String(data=json.dumps({
            'ok': True, 'generation': self.accepted_generation,
            'rejected_stale': self.rejected_stale,
            # ★暴露任务号与撤销线★：验收探针要"在撤销线之上"播种一条新任务来证明
            # 取消只封旧 mission（不能靠猜编号，那会造出假绿/假红）。
            'mission': (getattr(self, 'task_meta', None) or {}).get('mission_id'),
            'revoked_mission': getattr(self, 'revoked_mission', -1),
            'poses': len(self.poses), 'index': self.index + 1,
            'remaining_m': None if rest is None else round(rest, 3),
            'goal': [round(goal.pose.position.x, 3), round(goal.pose.position.y, 3),
                     round(goal.pose.position.z, 3)],
            'frame': goal.header.frame_id})))

    def tick(self, _event):
        """推进航点：当前目标"到了/太近/卡住超时"就往前找一个**够远**的点，只发一次。

        三条实测约束（2026-11-26，靠 `nav_timeline.tsv` 才看清）：
          ① local 每段只走 `max_step_m`（0.30 m）就停，必须给新目标才继续；
          ② 目标离当前位姿太近（路径点间距仅 0.1 m）时它**不产生速度**： timeline 里
             `mux reason=track` 而 `cmd` 恒零，车不动，看起来像"卡死"；
          ③ 兜底超时不能依赖 `rospy.Time.now()`：该节点没有 `/clock` 时它恒为 0，
             超时永不成立 ⇒ index 冻结（实测 139 s 一直是 index=5/14）。
        所以：按墙钟判超时，并且推进时**跳过**那些离位姿 < `min_goal_m` 的点（末段兜底发最后一点）。
        """
        if self.cancelled_at is not None or not self.poses or self.pose is None:
            return
        now = time.monotonic()
        stale = (self.published_at is not None
                 and now - self.published_at > self.advance_timeout_s)
        cur = self.poses[self.index].pose.position
        rest = math.hypot(cur.x - self.pose.x, cur.y - self.pose.y)
        if rest > self.min_goal_m and not stale:
            return
        for i in range(self.index + 1, len(self.poses)):
            p = self.poses[i].pose.position
            if math.hypot(p.x - self.pose.x, p.y - self.pose.y) >= self.min_goal_m:
                self.index = i
                self.publish_current()
                return
        # 兜底：后面没有"够远"的点了 ⇒ 直接以最后一点为目标（末段可能很近，但必须发出，
        # 否则永远到不了终点）。同一坐标重复发会被 publish_current 的去重键吃掉，不会刷屏。
        self.index = len(self.poses) - 1
        self.publish_current()


def main():
    rospy.init_node('path_goal3d', anonymous=False, disable_signals=True)
    PathGoal()
    rospy.spin()


if __name__ == '__main__':
    main()
