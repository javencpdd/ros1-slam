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
        """立即清空活动路径；此后只有 stamp 更新的 path 才能重新武装。"""
        dropped = len(self.poses)
        self.cancelled_at = rospy.Time.now()
        self.poses = []
        self.index = 0
        self.last_goal_key = None      # 取消后同坐标的新任务不得被去重吃掉
        self.published_at = None
        self.status_pub.publish(String(data=json.dumps({
            'ok': True, 'cancelled': True, 'dropped_poses': dropped,
            'reason': message.data[:80]})))

    def on_path_task(self, message):
        """原子路径：{identity..., poses:[[x,y,z],...]}。身份与点列一起到，不存在错配。"""
        import types as _t
        try:
            value = json.loads(message.data)
            poses = value.pop('poses')
        except Exception:
            self.status_pub.publish(String(data=json.dumps({'ok': False, 'reason': 'path_task_malformed'})))
            return
        if int(value.get('mission_id', -1)) <= int(getattr(self, 'revoked_mission', -1)):
            self.status_pub.publish(String(data=json.dumps({'ok': False, 'reason': 'task_revoked',
                                                            'mission_id': value.get('mission_id')})))
            return
        # ★同一任务/同一版本的滚动重发不得重置航点进度★：否则永远停在第一个航点（实测只走 0.3 m）。
        new_key = (int(value.get('mission_id', 0)), int(value.get('path_revision', 0)))
        old_key = getattr(self, 'task_key', None)
        reset_index = (old_key != new_key)
        self.task_key = new_key
        self.task_meta = value
        self.poses = [_t.SimpleNamespace(
            pose=_t.SimpleNamespace(position=_t.SimpleNamespace(x=p[0], y=p[1], z=p[2])))
            for p in poses]
        if reset_index:
            self.index = 0
            self.last_goal_key = None
        else:
            self.index = min(int(getattr(self, 'index', 0)), max(len(self.poses) - 1, 0))
        self.pending_meta = None
        self.accepted_generation += 1
        self.publish_current()

    def on_path_meta(self, message):
        """路径元信息先到一步，暂存待配对（单发布者 + TCP 保序 ⇒ 到达顺序可靠）。"""
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
        self.published_at = rospy.Time.now()
        rest = self.remaining()
        self.status_pub.publish(String(data=json.dumps({
            'ok': True, 'generation': self.accepted_generation,
            'rejected_stale': self.rejected_stale,
            'poses': len(self.poses), 'index': self.index + 1,
            'remaining_m': None if rest is None else round(rest, 3),
            'goal': [round(goal.pose.position.x, 3), round(goal.pose.position.y, 3),
                     round(goal.pose.position.z, 3)],
            'frame': goal.header.frame_id})))

    def tick(self, _event):
        if self.cancelled_at is not None:
            return
        if not self.poses or self.index >= len(self.poses) - 1:
            return
        rest = self.remaining()
        stale = (self.published_at is not None and
                 (rospy.Time.now() - self.published_at).to_sec() > self.advance_timeout_s)
        if (rest is not None and rest <= self.arrive_m) or stale:
            self.index += 1
            self.publish_current()


def main():
    rospy.init_node('path_goal3d', anonymous=False, disable_signals=True)
    PathGoal()
    rospy.spin()


if __name__ == '__main__':
    main()
