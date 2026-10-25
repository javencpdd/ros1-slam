"""R665 A1 驱动（容器内）：真值定位链（map→odom 恒等＋插件 odom→base）下发 move_base
目标，跟踪机器人真实运动（/sim/odom_gt 轨迹）＋cmd_vel 流＋action 结果，
轨迹过独立检查器（真值图支撑/避让几何）。后端由 launch 参数注入（dwa|teb）。
"""
import json
import math
import sys
import time
from pathlib import Path

import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.msg import Odometry

TRACK = sys.argv[1] if len(sys.argv) > 1 else 'dwa'
GOAL = (4.0, 0.3)  # m1 内空旷区（obs_3 (4.6,0.9) 半宽 0.4 之外），距出生 ~4.0 m
OBSTACLES = [(1.5, 1.2, 0.8, 0.8, 0.4), (3.0, -1.1, 0.9, 0.7, -0.3),
             (4.6, 0.9, 0.8, 0.8, 0.9), (2.2, -1.6, 0.8, 0.8, 0.2),
             (5.8, -1.0, 0.8, 0.8, -0.6), (0.8, 1.8, 0.8, 0.8, 0.1)]
WALLS = [(2.5, 4.5, 9.0, 0.2, 0.0), (2.5, -4.5, 9.0, 0.2, 0.0)]


def box_dist(px, py, cx, cy, sx, sy, yaw):
    c, s = math.cos(-yaw), math.sin(-yaw)
    dx, dy = px - cx, py - cy
    lx, ly = dx * c - dy * s, dx * s + dy * c
    return max(abs(lx) - sx / 2, abs(ly) - sy / 2, 0.0)


def main():
    rospy.init_node('a1_driver', anonymous=True, disable_signals=True)
    odom_samples = []
    cmd_vel_count = [0]
    result_msg = [None]

    rospy.Subscriber('/sim/odom_gt', Odometry,
                     lambda m: odom_samples.append((m.pose.pose.position.x,
                                                    m.pose.pose.position.y)))
    rospy.Subscriber('/cmd_vel', rospy.AnyMsg, lambda _m: cmd_vel_count.__setitem__(0, cmd_vel_count[0] + 1))

    def on_result(msg):
        result_msg[0] = msg
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, on_result)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)

    # 就绪条件：map 到位（/map 一条）＋odom 流＋move_base action 在册（话题出现）
    t0 = time.time()
    map_seen = [False]

    def on_map(_m):
        map_seen[0] = True
    rospy.Subscriber('/map', rospy.AnyMsg, on_map)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and time.time() - t0 < 90:
        if map_seen[0] and len(odom_samples) >= 30:
            topics = rospy.get_published_topics()
            if any(t == '/move_base/result' for t, _ in topics):
                break
        rate.sleep()

    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = 'map'
    goal.pose.position.x, goal.pose.position.y = GOAL
    goal.pose.orientation.w = 1.0
    goal_pub.publish(goal)
    sent_at = time.time() - t0

    # 完成条件：action 结果到达（成功=3）或超窗
    deadline = time.time() + 480
    while not rospy.is_shutdown() and time.time() < deadline:
        if result_msg[0] is not None:
            break
        rate.sleep()

    time.sleep(1)
    status = result_msg[0].status.status if result_msg[0] else None
    # 独立核验：机器人真实轨迹（真值 odom）对真值障碍几何
    traj = odom_samples
    min_clear = None
    if traj:
        min_clear = min(box_dist(p[0], p[1], *obs[:5]) for p in traj for obs in OBSTACLES + WALLS)
    end = traj[-1] if traj else None
    dist_to_goal = math.hypot(end[0] - GOAL[0], end[1] - GOAL[1]) if end else None
    result = {
        'stage': 'a1_backend_swap', 'backend': TRACK, 'goal': list(GOAL),
        'map_received': map_seen[0], 'goal_sent_after_s': round(sent_at, 1),
        'movebase_result_status': status,
        'status_text': (result_msg[0].status.text if result_msg[0] else None),
        'cmd_vel_messages': cmd_vel_count[0],
        'odom_samples': len(traj),
        'path_length_m': round(sum(math.hypot(b[0] - a[0], b[1] - a[1])
                                   for a, b in zip(traj, traj[1:])), 2) if traj else 0,
        'final_pos': list(end) if end else None,
        'dist_to_goal_m': round(dist_to_goal, 3) if dist_to_goal is not None else None,
        'obstacle_min_clearance_m': round(min_clear, 3) if min_clear is not None else None,
        'wall_duration_s': round(time.time() - t0, 1),
    }
    # 双门：action 成功（主）或轨迹到达（兜底：终点距目标 ≤0.35 且此后无大幅远离）
    arrived_by_track = bool(dist_to_goal is not None and dist_to_goal <= 0.35)
    result['arrived_by_track'] = arrived_by_track
    result['accepted'] = bool(
        result['map_received']
        and (status == GoalStatus.SUCCEEDED or arrived_by_track)
        and result['cmd_vel_messages'] >= 10 and result['odom_samples'] >= 50
        and min_clear is not None and min_clear >= 0.25)
    Path('/opt/ros1-slam/r665/a1-%s.json' % TRACK).write_text(json.dumps(result, indent=2) + '\n')
    print('A1', json.dumps(result, ensure_ascii=False))
    return 0 if result['accepted'] else 6


if __name__ == '__main__':
    sys.exit(main())
