#!/usr/bin/env python3
# m2_cmd_relay.py —— M2 自研话题转发器（小张，2026-10-06）
# 用途一：/cmd_vel_raw（mbf 输出，mbf_navigation.launch 已 remap）→ /cmd_vel（lite3_planar_drive 消费）
#         —— M6 safe_cmd_vel_mux 不在仿真闭环内，此 relay 仅 sim 域（ROS master 11313），不触生产 11312。
# 用途二：/livox/imu → /imu/data（hdl_localization 生产 remap 契约，M2-3 仿真域喂 IMU）。
# 用法：m2_cmd_relay.py <from_topic> <to_topic> <Twist|Imu>
# 红线：本节点不写任何真机话题；生产 master 下禁用（m2_*.launch 全部在 11313 隔离域内起）。
import sys, rospy

if __name__ == '__main__':
    rospy.init_node('m2_cmd_relay')
    src, dst, typ = sys.argv[1], sys.argv[2], sys.argv[3]
    mod = {'Twist': 'geometry_msgs.msg', 'Imu': 'sensor_msgs.msg'}[typ]
    msg_cls = getattr(__import__(mod, fromlist=['x']), typ)
    pub = rospy.Publisher(dst, msg_cls, queue_size=10)
    n = [0]
    def cb(m):
        pub.publish(m); n[0] += 1
    rospy.Subscriber(src, msg_cls, cb, queue_size=10)
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        rospy.loginfo_throttle(60, '[m2_cmd_relay] %s -> %s (%s) fwd=%d' % (src, dst, typ, n[0]))
        rate.sleep()
