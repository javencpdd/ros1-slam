"""R662 双轨驱动（容器内）：Gazebo /Odometry 真值 → 按 track 转发 mode3 消费面 →
合成参考路径 → 收 optimal_list 轨迹 → 独立检查器（真值图）判定。

truth 轨：body_pose＝真值直转；noisy 轨：x＋0.15 m 确定性偏置（估计误差注入）。
判定（两轨同门）：轨迹非空（≥5 点）、逐点支撑（真值图 ground 面）、同面连续
（步距上限）、终点到达参考目标容差。估计轨偏差如实记录不冒充。
"""
import json
import math
import sys
import time
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from visualization_msgs.msg import Marker

TRACK = sys.argv[1] if len(sys.argv) > 1 else 'truth'
NOISE_X = 0.15 if TRACK == 'noisy' else 0.0
GOAL = (2.5, -1.0, 0.0)   # m1 场地内、距出生点 ~2.7 m


def main():
    rospy.init_node('dual_driver', anonymous=True, disable_signals=True)
    odom_pub = rospy.Publisher('/quad_0/body_pose', Odometry, queue_size=5)
    pose_pub = rospy.Publisher('/quad_0/lidar_pose', Odometry, queue_size=5)
    path_pub = rospy.Publisher('/initial_path', RosPath, queue_size=1, latch=True)

    truth_seen = []
    odom_sub = None

    def on_odom(msg):
        truth_seen.append((msg.pose.pose.position.x, msg.pose.pose.position.y,
                           msg.pose.pose.position.z))
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = 'body'
        out.pose.pose = msg.pose.pose
        out.pose.pose.position.x += NOISE_X
        # 适配器责任（R662 运行面发现）：Gazebo odom z=base_link 基准高（实测 0.11），
        # mode3 契约＝地面系 z ⇒ 转发时归地面系；不归零则检查器拦头段悬空（已实证）
        out.pose.pose.position.z = 0.0
        odom_pub.publish(out)
        lidar = Odometry()
        lidar.header = msg.header
        lidar.child_frame_id = 'lidar'
        lidar.pose.pose = out.pose.pose
        pose_pub.publish(lidar)

    odom_sub = rospy.Subscriber('/sim/odom_gt', Odometry, on_odom)  # lite3_planar_drive 插件真值话题（实测）

    trajectories = {}
    goals = {}

    def on_marker(store, keyed):
        def cb(marker):
            key = (marker.id, marker.ns)
            if keyed and marker.points:
                store[key] = [(round(p.x, 4), round(p.y, 4), round(p.z, 4))
                              for p in marker.points[:200]]
            elif not keyed:
                goals[marker.id] = (marker.pose.position.x, marker.pose.position.y,
                                    marker.pose.position.z)
        return cb

    def raw(store, keyed):
        def cb(raw_msg):
            m = Marker()
            m.deserialize(raw_msg._buff)
            on_marker(store, keyed)(m)
        return cb

    rospy.Subscriber('/scan_planner_node/optimal_list', rospy.AnyMsg,
                     raw(trajectories, True))
    rospy.Subscriber('/scan_planner_node/goal_point', rospy.AnyMsg,
                     raw(goals, False))

    t0 = time.time()
    path_sent = None
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and time.time() - t0 < 300:
        # 就绪条件：Gazebo odom 流 ≥20 条（物理推进中）才发参考路径
        if path_sent is None and len(truth_seen) >= 20:
            path = RosPath()
            path.header.stamp = rospy.Time.now()
            path.header.frame_id = 'world'
            sx, sy, _ = truth_seen[-1]
            for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position.x = sx + frac * (GOAL[0] - sx)
                ps.pose.position.y = sy + frac * (GOAL[1] - sy)
                ps.pose.position.z = 0.0
                path.poses.append(ps)
            path_pub.publish(path)
            globals()['truth_sent_z'] = truth_seen[-1][2] if truth_seen else None
            path_sent = time.time() - t0
        # 完成：goal_point（≥3 waypoint marker，R633 同型）且首个轨迹 marker 到达后收尾窗
        if path_sent is not None and trajectories and len(goals) >= 3 \
                and time.time() - t0 > path_sent + 8:
            break
        rate.sleep()

    # 取最长轨迹（optimal_list id=0 球列表或 1000 线带，同点集）
    best = max((pts for pts in trajectories.values()), key=len, default=[])
    # 独立检查器：真值图（ground 全域＋远端 pad 双面以满足 map.v1 多面约束）
    for sub in ('contracts', 'representation', 'validator'):
        sys.path.insert(0, '/opt/ros1-slam/v8lib/' + sub)
    import support_map as rep
    import trajectory_checker as chk
    truth_map = rep.build_support_map(
        'm1_truth', 0.5, [-10.0, -10.0, 0.0],
        [{'surface_id': 'ground', 'z_reference': 0.0, 'data_uri': 'g',
          'footprint': {'min_x': -10, 'min_y': -10, 'max_x': 10, 'max_y': 10},
          'edges': [{'to': 'pad', 'kind': 'step', 'max_step_height_m': 0.5,
                     'bidirectional': True}]},
         {'surface_id': 'pad', 'z_reference': 0.4, 'data_uri': 'p',
          'footprint': {'min_x': 8, 'min_y': 8, 'max_x': 9, 'max_y': 9},
          'edges': [{'to': 'ground', 'kind': 'step', 'max_step_height_m': 0.5,
                     'bidirectional': True}]}],
        None, {'uri': 'sim://m1_truth', 'sha256': '9' * 64})
    verdict = {'ok': False, 'violations': [('E_NO_TRAJECTORY', 0, '未收到轨迹')]}
    if len(best) >= 5:
        # mode3 输出＝体心系（R633：地面系输入 z+body_height=0.30）⇒ 逆变换回地面系判支撑
        ground_traj = [(x, y, round(z - 0.30, 6)) for x, y, z in best]
        verdict = chk.check(truth_map, ground_traj, max_step=1.5)
    end = best[-1] if best else None
    reached = bool(end and math.hypot(end[0] - GOAL[0], end[1] - GOAL[1]) <= 0.6
                   and abs(end[2] - GOAL[2]) <= 0.35)
    result = {
        'stage': 'sim_dualtrack', 'track': TRACK, 'noise_x': NOISE_X,
        'odom_truth_samples': len(truth_seen),
        'odom_z_at_send': (truth_sent_z if (truth_sent_z := globals().get('truth_sent_z')) is not None else None),
        'path_sent_after_s': path_sent,
        'waypoint_markers': sorted(goals),
        'trajectory_points': len(best),
        'trajectory_head': best[:3], 'trajectory_tail': best[-3:] if best else [],
        'trajectory_z_range': [min((p[2] for p in best), default=None),
                               max((p[2] for p in best), default=None)],
        'checker_ok': verdict['ok'],
        'checker_violations': verdict['violations'][:4],
        'goal_reached': reached,
        'wall_duration_s': round(time.time() - t0, 1),
    }
    result['accepted'] = bool(result['trajectory_points'] >= 5
                              and result['checker_ok'] and result['goal_reached'])
    Path('/opt/ros1-slam/r662/sim-dualtrack-%s.json' % TRACK).write_text(
        json.dumps(result, indent=2) + '\n')
    print('DUALTRACK', json.dumps(result, ensure_ascii=False))
    return 0 if result['accepted'] else 6


if __name__ == '__main__':
    sys.exit(main())
