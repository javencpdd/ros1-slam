"""验收记录器：**先订阅、后判读**。

为什么不是「发完 goal 再起一个观察节点」：v1（`../b5_arrival/`）就是那么做的，
读数是 `ARRIVED_TRUE=0 / PATH_UNIQ_STAMPS=0`——不是判据错，是观察者比事件晚到。
`/v8/bt/arrived` 与 `/global_path` 都不 latch（路径故意不 latch，见 far_node 注释），
一次性的到达帧错过了就永远错过了。

本节点在 roslaunch 起来之后、任何 goal 之前启动，一直跑到收工，把每条事件按
ROS 时间落成 NDJSON。切场景不靠墙钟对齐，而靠 `/_b5_mark` 上的字符串标记——
标记与事件同一个时钟，分析脚本按标记切片。

READY 判据见 `main()` 末尾注释：latched 状态话题收到 ＋ 位姿在跳 ＋ 两条证据通道的
发布者连接数 ≥ 1 ＋ **地形云里真的有障碍格**。**不要**等 `/far_reach_goal_status`——
它只在有目标时才发布（上游 `PlanningCallBack` 在无目标分支里不发），拿它当就绪信号
会永远等不到。
"""
import json
import sys
import time

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String

OBSTACLE_INTENSITY = 0.2      # 与 `_cloud_probe.py` 同判据：intensity ≥ 0.2 才算障碍格


def main():
    out_path = sys.argv[1]
    rospy.init_node('b5_recorder', disable_signals=True)
    handle = open(out_path, 'w')
    ready = {'reach': 0, 'status': 0, 'pose': 0}
    terrain = {'frames': 0, 'obs': 0, 'settled': False}
    last_pose = {'value': None, 'at': -1e9}

    def emit(kind, **fields):
        record = {'kind': kind, 't': round(rospy.get_time(), 3)}
        record.update(fields)
        handle.write(json.dumps(record, ensure_ascii=False) + '\n')
        handle.flush()

    def on_reach(msg):
        ready['reach'] += 1
        emit('reach', data=bool(msg.data))

    def on_arrived(msg):
        emit('arrived', data=bool(msg.data))

    def on_path(msg):
        poses = msg.poses
        tail = poses[-1].pose.position if poses else None
        emit('path', n=len(poses), stamp=msg.header.stamp.to_nsec(),
             tail=[round(tail.x, 3), round(tail.y, 3), round(tail.z, 3)] if tail else None)

    def on_waypoint(msg):
        point = msg.point
        emit('waypoint', p=[round(point.x, 3), round(point.y, 3), round(point.z, 3)])

    def on_status(msg):
        ready['status'] += 1
        emit('status', text=msg.data[:200])

    def on_pose(msg):
        ready['pose'] += 1
        position = msg.pose.position
        value = (round(position.x, 3), round(position.y, 3), round(position.z, 3))
        previous = last_pose['value']
        now = rospy.get_time()
        # 位姿 50 Hz，全记会把文件写成几十万行；只记「变了 1 cm 以上」或「距上次已过 1 s」的帧。
        # 心跳那一路是给「静止窗口里位姿仍然活着」留证据用的——只按变化记的话，
        # 静止场景的 poses 会是 0，分析时无法区分「位姿断了」和「车没动」。
        changed = previous is None or max(abs(value[i] - previous[i]) for i in range(3)) > 0.01
        if changed or (now - last_pose['at']) >= 1.0:
            last_pose['value'] = value
            last_pose['at'] = now
            emit('pose', p=list(value))

    def on_mark(msg):
        emit('mark', tag=msg.data)

    def on_terrain(msg):
        """数地形云里的障碍格。攒够一格就不再解析——两万点的 Python 解析不该跑整场。"""
        terrain['frames'] += 1
        if terrain['settled']:
            return
        try:
            values = [p[3] for p in pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'),
                                                   skip_nans=True)]
        except Exception as error:                    # 字段缺失之类，如实报，不吞
            emit('terrain', error=str(error)[:60])
            return
        obs = sum(1 for v in values if v >= OBSTACLE_INTENSITY)
        if obs >= 1:
            terrain['obs'] = obs
            terrain['settled'] = True

    rospy.Subscriber('/far_reach_goal_status', Bool, on_reach, queue_size=5)
    arrived_sub = rospy.Subscriber('/v8/bt/arrived', Bool, on_arrived, queue_size=5)
    path_sub = rospy.Subscriber('/global_path', Path, on_path, queue_size=5)
    rospy.Subscriber('/way_point', PointStamped, on_waypoint, queue_size=5)
    rospy.Subscriber('/global_path/status', String, on_status, queue_size=5)
    rospy.Subscriber('/v8/local/pose', PoseStamped, on_pose, queue_size=5)
    rospy.Subscriber('/_b5_mark', String, on_mark, queue_size=10)
    rospy.Subscriber('/terrain_cloud', PointCloud2, on_terrain, queue_size=2)

    def connections(sub):
        try:
            return sub.get_num_connections()
        except AttributeError:          # 该 rospy 没这个 API：退化为只看事件门
            return -1

    # READY 判据（四条一起看）：
    #   1. latched 的 `/global_path/status` 已收到 ⇒ 记录器与适配器在同一张图上；
    #   2. `/v8/local/pose` 在跳 ⇒ 仿真真的跑起来了，不是刚起 master；
    #   3. `/v8/bt/arrived` 与 `/global_path` 的**发布者连接数 ≥ 1** ⇒ 两条证据通道都接通；
    #   4. `/terrain_cloud` 里出现 `intensity ≥ 0.2` 的障碍格 ⇒ **整条 FAR 链路的前置**。
    # ★ 第 4 条是重跑才加的，原因是读懂了失败轮之后发现的先后倒置 ★
    #   `is_cloud_init_` 只在 `surround_obs_cloud_` 非空时置真（far_planner.cpp:669），
    #   `Loop()` 的入口守卫是 `PreconditionCheck() = is_cloud_init_ && is_odom_init_`。
    #   失败轮（launch.log）里 `[local] obstacles ok: 0` 连出 27 条、一条非 0 都没有，
    #   于是 FAR 主循环从未进入、`Local/Global V-Graph Updated` 各 0 次、目标被
    #   `!is_graph_init_` 守卫全丢——而记录器当时仅凭前三条就报 READY、驱动器随即发目标，
    #   结果是 C0 记成 INCONCLUSIVE，看起来像「判据没测出来」，其实上游还没起步。
    # ★ 也不要拿 `/far_reach_goal_status` 当 READY 信号 ★
    #   上游 `PlanningCallBack`（far_planner.cpp:237-249）在 `!is_graph_init_` 时直接 return，
    #   没有目标时又走 `goal_ptr == NULL` 分支、**不发布**这条话题。第一次实测就是
    #   `RECORDER_NOT_READY reach=0 status=1` 一直等不到——那说的是「还没有目标」，
    #   不是「记录器没连上」（见 attemptA_recorder_notready.log）。
    deadline = time.monotonic() + 120.0
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        arrived_conn = connections(arrived_sub)
        path_conn = connections(path_sub)
        wired = (arrived_conn != 0) and (path_conn != 0)
        if (ready['status'] >= 1 and ready['pose'] >= 1 and wired
                and terrain['obs'] >= 1):
            print('RECORDER_READY status=%d pose=%d arrived_conn=%d path_conn=%d reach=%d '
                  'terrain_frames=%d obs=%d'
                  % (ready['status'], ready['pose'], arrived_conn, path_conn, ready['reach'],
                     terrain['frames'], terrain['obs']), flush=True)
            rospy.spin()
            return 0
        time.sleep(0.05)
    print('RECORDER_NOT_READY status=%d pose=%d arrived_conn=%d path_conn=%d reach=%d '
          'terrain_frames=%d obs=%d'
          % (ready['status'], ready['pose'], connections(arrived_sub), connections(path_sub),
             ready['reach'], terrain['frames'], terrain['obs']), flush=True)
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
