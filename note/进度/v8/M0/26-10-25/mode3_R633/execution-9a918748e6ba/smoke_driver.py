"""V8-03 第二切片无头烟测驱动（容器内执行，支持四案例）。

ramp ＝ R633 原案例（B2 3-4-5 反例，结果写 smoke-result.json 保持兼容）；
flat ＝ B1 平地等距正例；wall ＝ C2 同高障碍（墙）运行时；slab ＝ C3 双层列运行时。
wall/slab 向 /pcl_render_node/cloud 注入世界系合成点云（cloud_is_world=true），
观测 scan_planner_node 的 goal_point waypoint、a_star_list 与节点日志中的
"a star error"（bspline 优化器碰撞段修复调用 A* 失败时的 ROS_ERROR）。
body_height 用官方 advanced_param.xml 的 0.30；期望值来自 R632 离线探针手算。
"""
import json
import math
import struct
import sys
import time
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as RosPath
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker

BODY_HEIGHT = 0.30
LOG = Path('/opt/ros1-slam/r633/logs/roslaunch.log')

RAMP_POSES = ([(0.1 * i, 0.0, 0.0) for i in range(4)]
              + [(0.45, 0.0, 0.2), (0.60, 0.0, 0.4)]
              + [(0.7 + 0.1 * i, 0.0, 0.4) for i in range(4)])
RAMP_EXPECT = [(0.0, 0.0, 0.30), (0.6, 0.0, 0.70), (1.0, 0.0, 0.70)]

FLAT_POSES = [(round(0.1 * i, 4), 0.0, 0.0) for i in range(21)]
FLAT_EXPECT = [(0.5 * i, 0.0, 0.30) for i in range(5)]

WALL_POSES = [(round(0.1 * i, 4), 0.0, 0.0) for i in range(31)]
WALL_EXPECT = [(0.5 * i, 0.0, 0.30) for i in range(7)]
# 墙：x=1.5，y 全跨度封死平面侧绕，z 0→0.75（体心 0.30 平面全被挡，上方可过但平面表达不了）
WALL_CLOUD = [(1.5, round(-4.0 + 0.1 * i, 3), round(0.05 + 0.05 * j, 3))
              for i in range(81) for j in range(15)]

SLAB_POSES = ([(round(0.1 * i, 4), 0.0, 0.0) for i in range(4)]
              + [(round(0.4 + 0.1 * i, 4), 0.0, round((0.4 + 0.1 * i - 0.3) / 1.5 * 0.6, 6)) for i in range(15)]
              + [(round(1.9 + 0.1 * i, 4), 0.0, 0.6) for i in range(12)])
SLAB_EXPECT = [(0.0, 0.0, 0.30), (0.5, 0.0, 0.38), (1.0, 0.0, 0.58),
               (1.5, 0.0, 0.78), (2.0, 0.0, 0.90), (2.5, 0.0, 0.90), (3.0, 0.0, 0.90)]
# 板：x 1.0→2.0，y ±2，z 0.50/0.60（从地面层升到板顶的轨迹在板 footprint 内穿越板体）
SLAB_CLOUD = [(round(1.0 + 0.1 * i, 3), round(-2.0 + 0.2 * j, 3), z)
              for i in range(11) for j in range(21) for z in (0.50, 0.60)]

CASES = {
    'ramp': {'poses': RAMP_POSES, 'expect': RAMP_EXPECT, 'cloud': None,
             'result': 'smoke-result.json', 'assert_astar_error': False},
    'flat': {'poses': FLAT_POSES, 'expect': FLAT_EXPECT, 'cloud': None,
             'result': 'smoke-result-flat.json', 'assert_astar_error': False},
    'wall': {'poses': WALL_POSES, 'expect': WALL_EXPECT, 'cloud': WALL_CLOUD,
             'result': 'smoke-result-wall.json', 'assert_astar_error': True},
    'slab': {'poses': SLAB_POSES, 'expect': SLAB_EXPECT, 'cloud': SLAB_CLOUD,
             'result': 'smoke-result-slab.json', 'assert_astar_error': False},
}


def make_cloud(points, stamp, seq):
    cloud = PointCloud2()
    cloud.header.stamp = stamp
    cloud.header.frame_id = 'world'
    cloud.header.seq = seq
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, name in enumerate(('x', 'y', 'z'))]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = 12 * len(points)
    cloud.is_dense = True
    cloud.data = struct.pack('<%df' % (3 * len(points)), *[v for p in points for v in p])
    return cloud


def main():
    case_name = sys.argv[1] if len(sys.argv) > 1 else 'ramp'
    case = CASES[case_name]
    goal_markers = {}
    a_star_markers = {}

    def marker_cb(store, keep_points, keyed):
        def cb(raw):
            marker = Marker()
            marker.deserialize(raw._buff)
            key = (marker.id, marker.ns) if keyed else marker.id
            if key in store:
                return
            if keep_points:
                if marker.points:
                    store[key] = [(round(p.x, 4), round(p.y, 4), round(p.z, 4)) for p in marker.points]
            else:
                store[key] = (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
        return cb

    rospy.init_node('mode3_smoke_driver', anonymous=True, disable_signals=True)
    rospy.Subscriber('/scan_planner_node/goal_point', rospy.AnyMsg,
                     marker_cb(goal_markers, keep_points=False, keyed=False))
    rospy.Subscriber('/scan_planner_node/a_star_list', rospy.AnyMsg,
                     marker_cb(a_star_markers, keep_points=True, keyed=True))
    odom_pub = rospy.Publisher('/quad_0/body_pose', Odometry, queue_size=5)
    pose_pub = rospy.Publisher('/quad_0/lidar_pose', Odometry, queue_size=5)
    cloud_pub = rospy.Publisher('/pcl_render_node/cloud', PointCloud2, queue_size=5)
    path_pub = rospy.Publisher('/initial_path', RosPath, queue_size=1, latch=True)

    t0 = time.time()
    path_sent_at = None
    seq = 0
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and time.time() - t0 < 24.0:
        now = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'body'
        odom.pose.pose.position.z = BODY_HEIGHT
        odom_pub.publish(odom)
        pose = Odometry()
        pose.header.stamp = now
        pose.header.frame_id = 'world'
        pose.child_frame_id = 'lidar'
        pose.pose.pose = odom.pose.pose
        pose_pub.publish(pose)
        if case['cloud'] and time.time() - t0 >= 0.3:
            cloud_pub.publish(make_cloud(case['cloud'], now, seq))
            seq += 1
        if path_sent_at is None and time.time() - t0 >= 5.0:
            path = RosPath()
            path.header.stamp = now
            path.header.frame_id = 'world'
            for x, y, z in case['poses']:
                pose_stamped = PoseStamped()
                pose_stamped.header = path.header
                pose_stamped.pose.position.x = x
                pose_stamped.pose.position.y = y
                pose_stamped.pose.position.z = z
                path.poses.append(pose_stamped)
            path_pub.publish(path)
            path_sent_at = time.time() - t0
        if path_sent_at is not None and goal_markers and time.time() - t0 > path_sent_at + 8.0:
            break
        rate.sleep()

    got = [goal_markers[k] for k in sorted(goal_markers)]
    expect = case['expect']
    waypoints_match = (len(got) == len(expect)
                       and all(math.dist(g, e) < 1e-3 for g, e in zip(got, expect)))
    log_text = LOG.read_text(errors='replace') if LOG.exists() else ''
    astar_errors = log_text.count('a star error')
    astar_assert_ok = (astar_errors >= 1) if case['assert_astar_error'] else True
    result = {
        'case': case_name, 'input_poses': case['poses'], 'body_height': BODY_HEIGHT,
        'expected_waypoints': expect, 'captured_waypoints': got,
        'captured_waypoints_by_id': {str(k): v for k, v in sorted(goal_markers.items())},
        'a_star_markers': {'%s/%s' % k: v for k, v in a_star_markers.items()},
        'a_star_error_count_in_log': astar_errors,
        'a_star_path_seen': bool(a_star_markers),
        'assert_astar_error': case['assert_astar_error'],
        'cloud_points_per_frame': len(case['cloud']) if case['cloud'] else 0,
        'path_sent_after_s': path_sent_at, 'wall_duration_s': time.time() - t0,
        'use_sim_time': rospy.get_param('/use_sim_time', False),
        'waypoints_match': bool(waypoints_match),
        'accepted': bool(waypoints_match and astar_assert_ok),
    }
    Path('/opt/ros1-slam/r633/' + case['result']).write_text(json.dumps(result, indent=2) + '\n')
    print('SMOKE_CASE', case_name, 'waypoints_match', waypoints_match,
          'astar_errors', astar_errors, 'a_star_path_seen', bool(a_star_markers))
    return 0 if result['accepted'] else 5


if __name__ == '__main__':
    sys.exit(main())
