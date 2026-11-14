#!/usr/bin/env python3
"""m2_tf_fresh.py —— map→odom 恒等变换的 fresh-stamp 动态广播器（M2R3，2026-10-08）。

为何存在：m2_teb.launch 原用 tf2_ros/static_transform_publisher 发 map→odom 恒等桥。
  静态条目进 tf2 的 static 专用缓存，语义为「对所有时刻恒真」，于是
  lookupTransform('map','base_link', Time(0)) 在动态段尚未入缓存时会回退到静态段的
  **零时刻锚点**，组合结果 hdr.stamp = Time(0) ⇒ mbf 保鲜校验得
  Δ_age = t_sim_now − 0 = 整条 sim 时钟（实测 21.491s，tolerance 3s）⇒ teb 立即 ABORT
  （state=4 / dt=0）。且 canTransform 在静态链上恒 True ⇒ 旧门控 G1–G4 结构上无法分辨。
  同族先例：P6 L62（墙钟 1.79e9s old）、P7 N-1（sim_pose_tf.py 改 20Hz fresh 广播）。

本件只改**时间基**：几何值与静态件逐字相同（0 0 0 / 单位四元数），节点数与启动次序不变，
差别仅是以 rate_hz 周期、每次带 rospy.Time.now() 动态发布 ⇒ stamp 新鲜 ⇒ G5 可稳定判定。
"""
import math
import sys

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def main():
    rate_hz = float(rospy.get_param("~rate_hz", "10.0"))
    parent = rospy.get_param("~parent_frame", "map")
    child = rospy.get_param("~child_frame", "odom")
    # 恒等几何：与 m2_teb.launch 原 static_transform_publisher args="0 0 0 0 0 0 map odom" 逐字等价
    x, y, z = (float(rospy.get_param("~x", "0.0")),
               float(rospy.get_param("~y", "0.0")),
               float(rospy.get_param("~z", "0.0")))
    roll, pitch, yaw = (float(rospy.get_param("~roll", "0.0")),
                        float(rospy.get_param("~pitch", "0.0")),
                        float(rospy.get_param("~yaw", "0.0")))

    # 欧拉(roll,pitch,yaw) → 四元数（与 static_transform_publisher 的 Euler 槽位约定一致：
    # args 顺序为 x y z yaw pitch roll，此处全 0 ⇒ 单位四元数，公式保留以便日后非零值复用）
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    ts = TransformStamped()
    ts.header.frame_id = parent
    ts.child_frame_id = child
    ts.transform.translation.x = x
    ts.transform.translation.y = y
    ts.transform.translation.z = z
    ts.transform.rotation.w = qw
    ts.transform.rotation.x = qx
    ts.transform.rotation.y = qy
    ts.transform.rotation.z = qz

    bcast = tf2_ros.TransformBroadcaster()
    r = rospy.Rate(rate_hz)
    n = 0
    while not rospy.is_shutdown():
        ts.header.stamp = rospy.Time.now()      # 关键：每次 fresh stamp，绝不 Time(0)
        bcast.sendTransform(ts)
        n += 1
        if n % 50 == 0:
            rospy.loginfo_throttle(5.0, "[m2_tf_fresh] %s->%s @%.1fHz stamp=%s"
                                   % (parent, child, rate_hz, ts.header.stamp))
        try:
            r.sleep()
        except rospy.ROSInterruptException:
            break


if __name__ == "__main__":
    rospy.init_node("m2_tf_fresh")
    main()
    sys.exit(0)
