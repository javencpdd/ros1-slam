"""V8-03 第二切片无头烟测驱动（容器内执行）。

合成 odom 与 /initial_path（R632 B2 的 3-4-5 反例 10 位姿），抓 scan_planner_node
的 goal_point marker（逐 waypoint、id=序号、frame=world），与离线探针手算期望对账。
body_height 用官方 advanced_param.xml 的 0.30。
"""
import json
import math
import sys
import time
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as RosPath

BODY_HEIGHT = 0.30

# R632 B2 输入（地面系 z），与 note/计划书v8/probe_mode3 相同
POSES = ([(0.1 * i, 0.0, 0.0) for i in range(4)]
         + [(0.45, 0.0, 0.2), (0.60, 0.0, 0.4)]
         + [(0.7 + 0.1 * i, 0.0, 0.4) for i in range(4)])
# 离线手算期望：抽稀保留 {P0, 坡顶(0.6,0.4), 终点(1.0,0.4)}，各 +body_height
EXPECTED = [(0.0, 0.0, 0.0 + BODY_HEIGHT),
            (0.6, 0.0, 0.4 + BODY_HEIGHT),
            (1.0, 0.0, 0.4 + BODY_HEIGHT)]


def main():
    rospy.init_node('mode3_smoke_driver', anonymous=True, disable_signals=True)
    markers = {}
    globals_seen = []

    def on_goal(marker):
        if marker.id not in markers:
            markers[marker.id] = (marker.pose.position.x, marker.pose.position.y,
                                  marker.pose.position.z)

    def on_global(marker):
        if marker.id not in globals_seen:
            globals_seen.append(marker.id)
            globals_seen.append(len(marker.points))
            globals_seen.append([(round(p.x, 4), round(p.y, 4), round(p.z, 4)) for p in marker.points[:64]])

    rospy.Subscriber('/scan_planner_node/goal_point', rospy.AnyMsg, _raw_marker(on_goal))
    rospy.Subscriber('/scan_planner_node/global_list', rospy.AnyMsg, _raw_marker(on_global, True))
    odom_pub = rospy.Publisher('/quad_0/body_pose', Odometry, queue_size=5)
    pose_pub = rospy.Publisher('/quad_0/lidar_pose', Odometry, queue_size=5)
    path_pub = rospy.Publisher('/initial_path', RosPath, queue_size=1, latch=True)

    t0 = time.time()
    path_sent_at = None
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and time.time() - t0 < 16.0:
        now = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'body'
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0 + BODY_HEIGHT
        odom_pub.publish(odom)
        pose = Odometry()
        pose.header.stamp = now
        pose.header.frame_id = 'world'
        pose.child_frame_id = 'lidar'
        pose.pose.pose = odom.pose.pose
        pose_pub.publish(pose)
        if path_sent_at is None and time.time() - t0 >= 2.5:
            path = Path()
            path.header.stamp = now
            path.header.frame_id = 'world'
            for x, y, z in POSES:
                pose_st = PoseStamped()
                pose_st.header = path.header
                pose_st.pose.position.x = x
                pose_st.pose.position.y = y
                pose_st.pose.position.z = z
                path.poses.append(pose_st)
            path_pub.publish(path)
            path_sent_at = time.time() - t0
        if path_sent_at is not None and markers and time.time() - t0 > path_sent_at + 6.0:
            break
        rate.sleep()

    got = [markers[k] for k in sorted(markers)]
    match = (len(got) == len(EXPECTED)
             and all(math.dist(g, e) < 1e-3 for g, e in zip(got, EXPECTED)))
    result = {
        'input_poses': POSES, 'body_height': BODY_HEIGHT,
        'expected_waypoints': EXPECTED,
        'captured_waypoints': got,
        'captured_waypoints_by_id': {str(k): v for k, v in sorted(markers.items())},
        'global_list_observation': globals_seen,
        'path_sent_after_s': path_sent_at,
        'wall_duration_s': time.time() - t0,
        'use_sim_time': rospy.get_param('/use_sim_time', False),
        'ros_time_at_end': str(rospy.Time.now()),
        'match': bool(match),
    }
    Path('/opt/ros1-slam/r633/smoke-result.json').write_text(json.dumps(result, indent=2) + '\n')
    print('SMOKE_MATCH', match, 'waypoints', got)
    return 0 if match else 5


def _raw_marker(fn, with_points=False):
    """AnyMsg 订阅避免依赖 devel 消息路径；反序列化用标准 visualization_msgs。"""
    from visualization_msgs.msg import Marker

    def cb(raw):
        m = Marker()
        m.deserialize(raw._buff)
        fn(m)
    return cb


if __name__ == '__main__':
    sys.exit(main())
