#!/usr/bin/env python3
"""把网页/外部 pose 消息接到仿真模型位姿（仅 Gazebo 真值栈）。

订阅：
  /sim/set_start   geometry_msgs/PoseStamped                  → 设起点（推荐给 Foxglove Publish 面板）
  /initialpose     geometry_msgs/PoseWithCovarianceStamped    → 兼容 RViz/Foxglove 的 Pose 工具
参数：
  ~model (lite3_planar) ~z (0.15) ~settle (0.5) ~use_initialpose (true)
发布：
  /sim/set_start/status std_msgs/String（JSON：成功/拒绝原因）

注意：只接受 frame 为空 / map / odom 的消息；其他坐标系的数值不会被猜着换算。
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim_pose  # noqa: E402

ALLOWED_FRAMES = ('', 'map', 'odom')


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from std_msgs.msg import String

    rospy.init_node('sim_pose_bridge')
    model = rospy.get_param('~model', 'lite3_planar')
    z = float(rospy.get_param('~z', 0.15))
    settle = float(rospy.get_param('~settle', 0.5))
    use_initialpose = bool(rospy.get_param('~use_initialpose', True))
    status = rospy.Publisher('/sim/set_start/status', String, queue_size=10)

    def apply(pose, frame, source):
        frame = frame or ''
        if frame not in ALLOWED_FRAMES:
            status.publish(json.dumps({'ok': False, 'source': source,
                                       'reason': 'unsupported_frame:' + frame}))
            rospy.logwarn('sim_pose_bridge: refused frame %r', frame)
            return
        try:
            yaw = sim_pose.yaw_deg_from_quaternion(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w)
            result = sim_pose.set_start(model, pose.position.x, pose.position.y,
                                        z, yaw, settle)
        except Exception as error:  # noqa: BLE001 - report to the web panel
            status.publish(json.dumps({'ok': False, 'source': source,
                                       'reason': str(error)}, ensure_ascii=False))
            rospy.logerr('sim_pose_bridge: %s', error)
            return
        status.publish(json.dumps({'ok': True, 'source': source, 'state': result},
                                  ensure_ascii=False))
        rospy.loginfo('sim_pose_bridge: %s -> %s', source, result)

    def on_pose_stamped(message):
        apply(message.pose, message.header.frame_id, 'set_start')

    def on_initialpose(message):
        apply(message.pose.pose, message.header.frame_id, 'initialpose')

    rospy.Subscriber('/sim/set_start', PoseStamped, on_pose_stamped, queue_size=1)
    if use_initialpose:
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, on_initialpose, queue_size=1)
    rospy.loginfo('sim_pose_bridge ready: model=%s z=%.3f settle=%.2f initialpose=%s',
                  model, z, settle, use_initialpose)
    rospy.spin()


if __name__ == '__main__':
    main()
