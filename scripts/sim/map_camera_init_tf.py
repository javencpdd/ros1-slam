#!/usr/bin/env python3
"""发一条 static TF: map -> camera_init（恒等）。

为什么需要它：faster_lio 的点云 frame_id 是 camera_init（它自己的世界原点），
但仿真台里 faster_lio 不发 TF（真值 TF 由 Gazebo 侧 map→odom→base_link 提供）。
两边没有连接 ⇒ rviz 无论以 map 还是 camera_init 为 Fixed Frame，
都只能看到一半：map 下看不到点云、camera_init 下看不到机器人且自己不动。

camera_init 与 map 原点重合（都是建图起点），所以恒等连接是正确语义。
用 tf2_ros.StaticTransformBroadcaster（发 /tf_static latched），
比 classic 的 static_transform_publisher 在 use_sim_time 下更稳。
"""
import rospy
import tf2_ros
import geometry_msgs.msg


def main():
    rospy.init_node('map_camera_init_static', anonymous=False)
    br = tf2_ros.StaticTransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'map'
    t.child_frame_id = 'camera_init'
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.w = 1.0
    br.sendTransform(t)
    rospy.loginfo('[map_camera_init_static] map -> camera_init (identity) published')
    rospy.spin()


if __name__ == '__main__':
    main()
