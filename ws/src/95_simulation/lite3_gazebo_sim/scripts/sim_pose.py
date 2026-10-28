#!/usr/bin/env python3
"""设定仿真起点/终点位姿（Gazebo 真值栈，map 系）。

用法：
  python3 sim_pose.py --start 1.0 0.5 --yaw 30      # 把 lite3_planar 放到 (1.0, 0.5)，朝向 30°
  python3 sim_pose.py --goal 3.0 0.0 --yaw 0        # 向 move_base 发目标
  python3 sim_pose.py --status                      # 读模型位姿与 /sim/odom_gt

边界：
- 起点＝Gazebo 模型世界位姿；本栈 odom 由插件取 WorldPose（闭环），map→odom 是恒等桥，
  所以模型位姿就是 map 系起点。RViz 的 “2D Pose Estimate” 发 /initialpose，
  本栈没有 AMCL 消费它，不会移动机器人。
- 终点＝/move_base_simple/goal（frame=map），需 flat2d_stack 的 move_base 在跑。
- 只动本仿真模型/发目标；不直接写底盘 cmd_vel，不取消 mux 与安全层。
"""
import argparse
import math
import time


def quaternion_from_yaw_deg(yaw_deg):
    if isinstance(yaw_deg, bool) or not isinstance(yaw_deg, (int, float)):
        raise ValueError('yaw must be numeric degrees')
    if not math.isfinite(yaw_deg):
        raise ValueError('yaw must be finite')
    half = math.radians(yaw_deg) / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def pose_xyz(values, name):
    if len(values) != 2:
        raise ValueError('%s needs X Y' % name)
    out = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('%s coordinates must be finite numbers' % name)
        out.append(float(value))
    return out


def set_start(model, x, y, z, yaw_deg, settle, cancel_goal=True):
    import rospy
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import SetModelState
    from geometry_msgs.msg import Point, Pose, Quaternion, Twist

    qx, qy, qz, qw = quaternion_from_yaw_deg(yaw_deg)
    rospy.wait_for_service('/gazebo/set_model_state', timeout=15)
    state = ModelState()
    state.model_name = model
    state.reference_frame = 'world'
    state.pose = Pose(position=Point(x, y, z),
                      orientation=Quaternion(qx, qy, qz, qw))
    state.twist = Twist()
    response = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)(state)
    if not response.success:
        raise RuntimeError('set_model_state refused: ' + response.status_message)

    if cancel_goal:
        try:
            from actionlib_msgs.msg import GoalID
            publisher = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
            deadline = time.time() + 1.0
            while publisher.get_num_connections() == 0 and time.time() < deadline:
                time.sleep(0.05)
            publisher.publish(GoalID())
        except Exception as error:  # noqa: BLE001 - move_base may not be running
            rospy.logwarn('no move_base cancel channel: %s', error)

    if settle > 0:
        rospy.sleep(settle)
    return model_state(model)


def publish_goal(x, y, yaw_deg, frame):
    import rospy
    from geometry_msgs.msg import PoseStamped

    qx, qy, qz, qw = quaternion_from_yaw_deg(yaw_deg)
    message = PoseStamped()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = frame
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.position.z = 0.0
    message.pose.orientation.x = qx
    message.pose.orientation.y = qy
    message.pose.orientation.z = qz
    message.pose.orientation.w = qw
    publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped,
                                queue_size=1, latch=True)
    seen = 0
    deadline = time.time() + 3.0
    while time.time() < deadline:
        seen = max(seen, publisher.get_num_connections())
        if seen:
            break
        time.sleep(0.05)
    for _ in range(3):
        publisher.publish(message)
        rospy.sleep(0.1)
        seen = max(seen, publisher.get_num_connections())
    if seen == 0:
        rospy.logwarn('no subscriber on /move_base_simple/goal; move_base 未起？')
    return {'frame': frame, 'x': x, 'y': y, 'yaw_deg': yaw_deg,
            'subscribers_seen': seen}


def model_state(model):
    import rospy
    from gazebo_msgs.srv import GetModelState

    rospy.wait_for_service('/gazebo/get_model_state', timeout=15)
    response = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)(model, 'world')
    if not response.success:
        raise RuntimeError('get_model_state refused: ' + response.status_message)
    pose = response.pose
    yaw = math.degrees(math.atan2(2 * (pose.orientation.w * pose.orientation.z
                                       + pose.orientation.x * pose.orientation.y),
                                  1 - 2 * (pose.orientation.y ** 2 + pose.orientation.z ** 2)))
    return {'model': model,
            'x': round(pose.position.x, 4), 'y': round(pose.position.y, 4),
            'z': round(pose.position.z, 4), 'yaw_deg': round(yaw, 2)}


def odom_state(topic='/sim/odom_gt'):
    import rospy
    from nav_msgs.msg import Odometry
    message = rospy.wait_for_message(topic, Odometry, timeout=10)
    pose = message.pose.pose
    yaw = math.degrees(math.atan2(2 * (pose.orientation.w * pose.orientation.z
                                       + pose.orientation.x * pose.orientation.y),
                                  1 - 2 * (pose.orientation.y ** 2 + pose.orientation.z ** 2)))
    return {'frame': message.header.frame_id,
            'x': round(pose.position.x, 4), 'y': round(pose.position.y, 4),
            'yaw_deg': round(yaw, 2)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='lite3_planar')
    parser.add_argument('--start', nargs=2, type=float, metavar=('X', 'Y'))
    parser.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'))
    parser.add_argument('--yaw', type=float, default=0.0, help='degrees')
    parser.add_argument('--z', type=float, default=0.15)
    parser.add_argument('--settle', type=float, default=2.0,
                        help='seconds to wait after teleport (default 2.0)')
    parser.add_argument('--frame', default='map', help='goal frame (default map)')
    parser.add_argument('--odom-topic', default='/sim/odom_gt',
                        help='odometry topic for --status/--start (default /sim/odom_gt)')
    parser.add_argument('--status', action='store_true')
    args = parser.parse_args()
    if not (args.start or args.goal or args.status):
        parser.error('use --start X Y / --goal X Y / --status')

    import rospy
    rospy.init_node('sim_pose', anonymous=True, disable_signals=True)
    result = {}
    if args.start:
        x, y = pose_xyz(args.start, 'start')
        result['start'] = set_start(args.model, x, y, args.z, args.yaw, args.settle)
        try:
            result['odom'] = odom_state(args.odom_topic)
        except Exception as error:  # noqa: BLE001 - odom is informational here
            result['odom_error'] = str(error)
    if args.goal:
        x, y = pose_xyz(args.goal, 'goal')
        result['goal'] = publish_goal(x, y, args.yaw, args.frame)
    if args.status:
        result['state'] = model_state(args.model)
        try:
            result['odom'] = odom_state(args.odom_topic)
        except Exception as error:  # noqa: BLE001 - status must not fail on missing odom
            result['odom_error'] = str(error)
    import json
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
