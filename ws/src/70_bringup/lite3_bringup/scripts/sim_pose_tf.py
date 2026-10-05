#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# sim_pose_tf.py —— mode_nav_sim 仿真测试台真值位姿节点（小张 2026-10-05，「仿真可用化」payload A）
#
# 职责：订阅仿真真值底盘位姿 /quad_0/body_pose（sim_adapter --mode traj @50Hz 发布，frame_id=world、
#       child_frame_id=base_link，开环 on-rails 完美位姿）→
#       ① 20Hz 广播 map→base_link（fresh-stamp，P6 修正批口径：stamp=0/旧戳会被 mbf 保鲜校验拒收）；
#       ② 20Hz 广播 base_link→rslidar 恒等（真机约定同款：mode_nav_stub 头注「base_link→rslidar 恒等由
#          驱动 base2livox 提供」——仿真不起真驱动，由本节点补位，避免 stvl sensor_frame=rslidar 变换链断裂）；
#       ③ 发布 /Odometry 真值里程计（nav_msgs/Odometry，frame_id=map，协方差占位对角 0.01；
#          话题名沿用 faster_lio 输出名，估计层按坑14 排除后该名无冲突，供 M5/SUP/调试消费；M4 只用 TF）。
# 数据源裁决（问3）：用 body_pose 不用 /quad_0/lidar_pose——body_pose 是狗体真值（渲染节点再由它派生
#       lidar_pose，隔一层传感器外参），且二期 SCAN-Planner 接入时轨迹源（规划器驱动回放）仍以 body_pose
#       为约定接口，本节点无需改；world≡map 恒等为本测试台坐标约定（真值图桥同约定，见 sim_map_bridge.py）。
# 替代对象：stub_tf_broadcaster.py（mode_nav_stub 的 10Hz 恒等静态版）——本节点是其动态化替换，
#       mode_nav_sim.launch 不得再引用 stub_tf_broadcaster（避免 map→base_link 双写者）。
# 红线：仅发布 TF 与 /Odometry，不发布/订阅任何速度类话题（速度唯一写者 = M6 safe_cmd_vel_mux，§2.6-a）。
# 参数：~pose_topic（默认 /quad_0/body_pose）、~tf_rate（默认 20.0）、~odom_topic（默认 /Odometry）、
#       ~odom_cov（默认 0.01，协方差占位）、~stale_warn_s（默认 1.0 位姿断流告警阈值）、~rslidar_frame（默认 rslidar）。

import rospy
import tf
from nav_msgs.msg import Odometry


class SimPoseTf(object):
    def __init__(self):
        self.pose_topic = rospy.get_param('~pose_topic', '/quad_0/body_pose')
        self.tf_rate = float(rospy.get_param('~tf_rate', 20.0))
        self.odom_topic = rospy.get_param('~odom_topic', '/Odometry')
        self.odom_cov = float(rospy.get_param('~odom_cov', 0.01))
        self.stale_warn_s = float(rospy.get_param('~stale_warn_s', 1.0))
        self.rslidar_frame = rospy.get_param('~rslidar_frame', 'rslidar')
        self.have_pose = False
        self.px = self.py = self.pz = 0.0
        self.qx = self.qy = self.qz = 0.0
        self.qw = 1.0
        self.last_wall = None
        self.pub_odom = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        rospy.Subscriber(self.pose_topic, Odometry, self.on_pose, queue_size=50)

    def on_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.px, self.py, self.pz = p.x, p.y, p.z
        self.qx, self.qy, self.qz, self.qw = q.x, q.y, q.z, q.w
        self.have_pose = True
        self.last_wall = rospy.get_rostime()

    def spin(self):
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(self.tf_rate)
        while not rospy.is_shutdown():
            if self.have_pose:
                stamp = rospy.Time.now()      # fresh-stamp（P6 修正批：旧戳被 mbf 保鲜校验拒收）
                br.sendTransform((self.px, self.py, self.pz),
                                 (self.qx, self.qy, self.qz, self.qw),
                                 stamp, 'base_link', 'map')
                br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                                 stamp, self.rslidar_frame, 'base_link')
                om = Odometry()
                om.header.stamp, om.header.frame_id, om.child_frame_id = stamp, 'map', 'base_link'
                om.pose.pose.position.x, om.pose.pose.position.y, om.pose.pose.position.z = \
                    self.px, self.py, self.pz
                om.pose.pose.orientation.x, om.pose.pose.orientation.y = self.qx, self.qy
                om.pose.pose.orientation.z, om.pose.pose.orientation.w = self.qz, self.qw
                # 协方差占位：真值位姿名义无偏，对角小值仅供下游滤波/诊断接口占位
                for i in (0, 7, 14, 21, 28, 35):
                    om.pose.covariance[i] = self.odom_cov
                for i in (0, 7, 14, 21, 28, 35):
                    om.twist.covariance[i] = self.odom_cov
                self.pub_odom.publish(om)
                if self.last_wall is not None and \
                        (rospy.get_rostime() - self.last_wall).to_sec() > self.stale_warn_s:
                    rospy.logwarn_throttle(5.0,
                        '[sim_pose_tf] %s 断流>%.1fs：沿用最后真值持续广播（on-rails 源未起？）'
                        % (self.pose_topic, self.stale_warn_s))
            else:
                rospy.logwarn_throttle(5.0, '[sim_pose_tf] 尚未收到 %s 首帧' % self.pose_topic)
            rate.sleep()


def main():
    rospy.init_node('sim_pose_tf')
    SimPoseTf().spin()


if __name__ == '__main__':
    main()
