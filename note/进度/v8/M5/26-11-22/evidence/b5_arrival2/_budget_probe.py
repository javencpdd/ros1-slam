"""到达预算探针：量 `WaitArrival` 在 `onStart` 冻结的那条路径有多长。

要验的是计划 §4.5 的【连带发现】：`bt_console.cpp:449-476` 在 `onStart` 里取**当时那条
`/global_path`** 的长度并冻结成 `budget_`（公式 `arrival_budget.hpp:16-21`：
`max(tree_timeout, length / speed_floor)`）。FAR 单航点模式下适配器发的是「位姿→航点」
两点路径，若这一跳只有一两米，而操作员给的目标在五米外，预算就会被按「首段」而不是
「整条路线」定死 ⇒ `elapsed > budget_` 判 FAILURE（`bt_console.cpp:584-593`）。

本探针不臆断，只把三个量摆在一起：
  1. `dist`：目标到当前位姿的直线距离（操作员以为要走多远）；
  2. `path_len`：BT 实际拿到的那条 `/global_path` 折线长度（预算真正乘的那个量）；
  3. 启动日志里的 `[bt] arrival budget %.1f s (path %.2f m, ...)`（冻结值，由外层脚本 grep）。

★ 只在 `budget_ > timeout_` 时才打印那行（`bt_console.cpp:471-475`）★ 所以「没有这行」
本身就等于「预算＝树里的 60 s」，也是读数，不是缺失。

不驱动判定：本探针只发目标、只读话题，movement 由 BT + 局部规划器自己走；到点后
适配器会按 §4.5 判据发 `arrived=true`，BT 进 Dwell，探针照常进入下一段。
"""
import sys
import time

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

# 从 `/rosout` 实时截 BT 的关键行。
# ★ 为什么不 grep `launch.log` ★ 实测 `bt_console` 的 stdout 进文件有明显延迟：
#   本轮 `external arrival signal mode` 那一行到收工时确实在文件里，但在脚本读取的那一刻
#   还没落盘（`BUD_BT_MODE=0` 而文件里 grep 得到 1 条）。判据挂在文件写入时机上，
#   就会把「BT 正常」读成「BT 没起」。`/rosout` 是同一批打印的另一条出口，实时、不缓冲。
BT_LOG_KEYS = ('arrival budget', 'arrival timeout', 'global path refused', 'arrived')

# 与 `_recorder.py`／`_cloud_probe.py` 同一判据：地形云里必须有障碍格，FAR 才会启动主循环
# （`is_cloud_init_` 只在 `surround_obs_cloud_` 非空时置真，`far_planner.cpp:669`）。
OBSTACLE_INTENSITY = 0.2


def polyline_length(msg):
    poses = msg.poses
    total = 0.0
    for prev, cur in zip(poses, poses[1:]):
        a, b = prev.pose.position, cur.pose.position
        total += ((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2) ** 0.5
    return total


class Probe(object):
    def __init__(self, goal_topic):
        rospy.init_node('b5_budget_probe', disable_signals=True)
        self.goal_topic = goal_topic
        self.pose = None
        self.path_lock = {'at': -1e9, 'length': None, 'points': 0}
        self.status = {'text': ''}
        self.terrain = {'frames': 0, 'obs': 0, 'settled': False}
        # ★ 话题按图走 ★ BT 在链上时目标入口是 `/v8/bt/goal`（`graph_3d_lite3_far.yaml:231`
        #   的 operator 输入），BT 再用 planner_goal 发 `/v8/global/goal` 给规划器；
        #   BT 不在链上时（B2/B3 的 enable_bt:=false）才直接发 `/v8/global/goal`。
        #   发错入口不会报错，只会「没人接」——probe 的就绪门会因此超时。
        self.goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=1, latch=True)
        rospy.Subscriber('/v8/local/pose', PoseStamped, self._on_pose, queue_size=2)
        rospy.Subscriber('/global_path', Path, self._on_path, queue_size=5)
        rospy.Subscriber('/global_path/status', String, self._on_status, queue_size=5)
        rospy.Subscriber('/rosout', Log, self._on_log, queue_size=50)
        rospy.Subscriber('/terrain_cloud', PointCloud2, self._on_terrain, queue_size=2)

    def _on_terrain(self, msg):
        self.terrain['frames'] += 1
        if self.terrain['settled']:
            return
        try:
            values = [p[3] for p in pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'),
                                                   skip_nans=True)]
        except Exception:                       # 字段缺失之类：本条不计，等下一帧
            return
        obs = sum(1 for v in values if v >= OBSTACLE_INTENSITY)
        if obs >= 1:
            self.terrain['obs'] = obs
            self.terrain['settled'] = True

    def _on_log(self, msg):
        if msg.name != '/bt_console':
            return
        if any(key in msg.msg for key in BT_LOG_KEYS):
            print('BT_LOG %s' % msg.msg, flush=True)

    def _on_pose(self, msg):
        position = msg.pose.position
        self.pose = (position.x, position.y, position.z)

    def _on_path(self, msg):
        # 时间戳相同的一批路径只算一次：BT 与探针都按 10 Hz 级别收，重复帧不该重复计数
        stamp = msg.header.stamp.to_nsec()
        if stamp == self.path_lock.get('stamp'):
            return
        self.path_lock = {'stamp': stamp, 'at': time.monotonic(),
                          'length': polyline_length(msg), 'points': len(msg.poses)}

    def _on_status(self, msg):
        self.status['text'] = msg.data[:160]

    def wait_for(self, predicate, timeout, note):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.05)
        print('TIMEOUT %s after %.0f s' % (note, timeout), flush=True)
        return False

    def send(self, xyz):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def phase(self, label, dx, seconds=25.0):
        gx, gy, gz = self.pose[0] + dx, self.pose[1], self.pose[2]
        before_stamp = self.path_lock.get('stamp')
        self.send((gx, gy, gz))
        got = self.wait_for(lambda: self.path_lock.get('stamp') != before_stamp, 25.0,
                            label + ' new path')
        time.sleep(seconds)
        dist = dx
        print('PHASE %-10s goal_dist=%.2f new_path=%s path_len=%s path_points=%d status=%s'
              % (label, dist, got,
                 '%.3f' % self.path_lock['length'] if self.path_lock.get('length') is not None else '-',
                 self.path_lock.get('points', 0), self.status['text'] or '-'), flush=True)

    def run(self):
        # 就绪门三条：有位姿、目标入口有人接、地形云里真有障碍格（后者是 FAR 主循环的前置）
        ready = self.wait_for(lambda: self.pose is not None
                              and self.goal_pub.get_num_connections() >= 1
                              and self.terrain['obs'] >= 1, 150.0, 'probe ready')
        if not ready:
            print('PROBE_NOT_READY pose=%s goal_conn=%d terrain_frames=%d obs=%d'
                  % (self.pose is not None, self.goal_pub.get_num_connections(),
                     self.terrain['frames'], self.terrain['obs']), flush=True)
            return 5
        print('PROBE_READY goal_topic=%s pose=%.3f %.3f %.3f terrain_frames=%d obs=%d'
              % (self.goal_topic, self.pose[0], self.pose[1], self.pose[2],
                 self.terrain['frames'], self.terrain['obs']), flush=True)
        # 短目标：预算应当就是树里的 60 s（日志里没有 arrival budget 行）
        self.phase('near_1m', 1.0, seconds=18.0)
        # 长目标：若 path_len 仍是一两米，而操作员给的是 5 m，则坐实「按首段冻结」
        self.phase('far_5m', 5.0, seconds=25.0)
        print('PROBE_DONE', flush=True)
        return 0


if __name__ == '__main__':
    raise SystemExit(Probe(sys.argv[1] if len(sys.argv) > 1 else '/v8/bt/goal').run())
