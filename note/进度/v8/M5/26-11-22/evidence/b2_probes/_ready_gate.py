"""FAR 就绪门：等地形云里出现障碍格，再放行探针发目标。

为什么单独一道门：
  `far_planner` 的 `is_cloud_init_` 只在 `surround_obs_cloud_` 非空时置真
  （far_planner.cpp:669），`Loop()` 的入口守卫是 `PreconditionCheck()`。
  障碍链路整场全空时，主循环从未进入，目标被 `!is_graph_init_` 守卫静默丢弃，
  探针随后报出的「没有路径」是**上游还没起步**，不是判据失效。
  B5 前两轮就是把这个读成判据问题（记成 INCONCLUSIVE），返工两轮才定位。

本门的四条一起看（与 b5_arrival2/_recorder.py 同口径）：
  1. `/global_path/status` 收到至少 1 条 ⇒ 与适配器在同一张图上；
  2. `/v8/local/pose` 收到至少 1 条 ⇒ 仿真真的跑起来；
  3. `/global_path` 的发布者连接数 != 0 ⇒ 证据通道接通；
  4. `/terrain_cloud` 里出现 intensity >= 0.2 的格 ⇒ 整条 FAR 链路的前置。

★ 不要等 `/far_reach_goal_status` ★ 上游 `PlanningCallBack` 在无目标时直接不发。
"""
import sys
import time

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

OBSTACLE_INTENSITY = 0.2
DEADLINE_S = 120.0


def main():
    rospy.init_node('b2_ready_gate', disable_signals=True)
    seen = {'status': 0, 'pose': 0}
    terrain = {'frames': 0, 'obs': 0, 'settled': False}

    def on_status(_msg):
        seen['status'] += 1

    def on_pose(_msg):
        seen['pose'] += 1

    def on_terrain(msg):
        terrain['frames'] += 1
        if terrain['settled']:
            return
        try:
            values = [p[3] for p in pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'),
                                                   skip_nans=True)]
        except Exception as error:                 # 字段缺失之类，如实报，不吞
            print('READY_GATE_TERRAIN_ERROR=%s' % str(error)[:60], flush=True)
            return
        obs = sum(1 for v in values if v >= OBSTACLE_INTENSITY)
        if obs >= 1:
            terrain['obs'] = obs
            terrain['settled'] = True

    path_sub = rospy.Subscriber('/global_path', Path, lambda _m: None, queue_size=5)
    rospy.Subscriber('/global_path/status', String, on_status, queue_size=5)
    rospy.Subscriber('/v8/local/pose', PoseStamped, on_pose, queue_size=5)
    rospy.Subscriber('/terrain_cloud', PointCloud2, on_terrain, queue_size=2)

    def connections(sub):
        try:
            return sub.get_num_connections()
        except AttributeError:
            return -1

    deadline = time.monotonic() + DEADLINE_S
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        path_conn = connections(path_sub)
        if seen['status'] >= 1 and seen['pose'] >= 1 and path_conn != 0 and terrain['obs'] >= 1:
            print('READY_GATE=READY status=%d pose=%d path_conn=%d terrain_frames=%d obs=%d'
                  % (seen['status'], seen['pose'], path_conn, terrain['frames'], terrain['obs']),
                  flush=True)
            return 0
        time.sleep(0.05)
    print('READY_GATE=NOT_READY status=%d pose=%d path_conn=%d terrain_frames=%d obs=%d'
          % (seen['status'], seen['pose'], connections(path_sub), terrain['frames'], terrain['obs']),
          flush=True)
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
