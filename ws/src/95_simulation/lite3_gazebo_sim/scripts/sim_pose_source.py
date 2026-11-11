#!/usr/bin/env python3
"""Publish /v8/local/pose from Gazebo ground truth, and declare localization OK.

/v8/local/pose is the contract topic every consumer subscribes to (console BT,
local planner), but the sim stack never had a producer for it — earlier runs
bridged it inside a throw-away test script.  This node is the formal bridge:

    /sim/odom_gt  (lite3_planar_drive WorldPose, nav_msgs/Odometry)
        -> /v8/local/pose            (geometry_msgs/PoseStamped, frame = map)
        -> /v8/local/localization_ok (std_msgs/Bool, True while poses are fresh)
        -> /v8/global/start          (PoseStamped, 低频，可关，喂全局规划器)

src_type=pose（坡面链）：源是 map 帧 PoseStamped 的直接生产者（如
gazebo_3d_carrier），本节点只补 localization_ok 心跳与 start，dst_topic 置空
关闭转发，位姿由源直发契约话题。

The second output matters: `local_planner/ros_node.py` refuses to command motion
with `reason=E_LOCAL_LOCALIZATION` until it hears that flag, so a pose source
that stays silent leaves the carrier standing still even with a valid path.

It republishes at a fixed rate so downstream nodes keep a steady stream even if
the drive plugin's odometry rate changes; the stamp is taken from the source
message so consumers can still age-check it.
"""
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class SimPoseSource(object):
    def __init__(self):
        self.src_topic = rospy.get_param('~src_topic', '/sim/odom_gt')
        self.dst_topic = rospy.get_param('~dst_topic', '/v8/local/pose')
        self.loc_topic = rospy.get_param('~localization_topic', '/v8/local/localization_ok')
        # 全局规划器 (planner/topic_node.py) 需要 map+start+goal 三输入才肯出路径，
        # 缺 start 时恒 E_PLAN_NOT_READY（表现为 BT request_path 等不到路径、走 recovery）。
        # start 低频重发：每条都会触发 topic_node 的 replan（换代取消旧规划），
        # 相当于给仿真链一个 Nav2 式的低频重规划节拍。默认关闭保持旧行为。
        self.start_topic = rospy.get_param('~start_topic', '')
        self.start_rate = float(rospy.get_param('~start_rate', 0.5))
        self.publish_ok = bool(rospy.get_param('~publish_localization_ok', True))
        self.ok_period = float(rospy.get_param('~localization_ok_period_s', 1.0))
        self.frame_id = rospy.get_param('~frame_id', 'map')
        # src_type=pose：源已是 map 帧的 PoseStamped（如 gazebo_3d_carrier 直发
        # 契约话题），本节点只当「定位占位桥」——发 localization_ok 心跳与 start，
        # 不再转发位姿本身（dst_topic 置空即关转发，避免与源发布者抢话题）。
        # src_type=odometry（默认）：走 /sim/odom_gt 桥的老行为。
        self.src_type = rospy.get_param('~src_type', 'odometry')
        if self.src_type not in ('odometry', 'pose'):
            raise ValueError('src_type must be odometry or pose')
        # map 坐标系的 z 语义是「支撑点高度」（与支撑图 z_reference 同基准）。
        # Gazebo 位姿取模型中心（方盒半高 0.11），不改算会出现两处假故障：
        # ①支撑判定 |z-0|=0.11 超 support_tol（此前靠 tol 0.25 压住）；
        # ②局部轨迹向路径 z=0 生成 -0.11 垂直分量，被 map_to_body_planar 以
        #   E_LOCAL_VERTICAL_UNSUPPORTED 拒发。默认 0.0 保持旧行为。
        self.z_offset = float(rospy.get_param('~z_offset', 0.0))
        # Gazebo 接触物理有 ~20µm 量级的软贯穿（模型中心 0.10999998 而非 0.11），
        # 换算后支撑点 z 会小幅低于支撑面顶。local_native 的自面豁免用的是硬编码
        # SUPPORT_TOL_S=1e-6（微米），z 低于面顶 1µm 即被判「嵌进地板」而拒发
        # E_LOCAL_NO_ROUTE（26-11-11 实测：z=-1e-5 必失败、z=+1e-4 正常）。
        # 支撑点语义上不可能低于它所站的支撑面，故在位姿源把小幅负贯穿吸附回面。
        # 深于 band 的负值保留原样（那是真异常，应可见）。默认 2 cm。
        self.z_snap_band = float(rospy.get_param('~z_snap_band', 0.02))
        rate = float(rospy.get_param('~rate', 50.0))
        self.latest = None
        self._last_ok = 0.0
        self._last_start = 0.0
        if self.src_type == 'pose' and not self.dst_topic:
            self.pub = None
        else:
            self.pub = rospy.Publisher(self.dst_topic, PoseStamped, queue_size=2)
        self.ok_pub = rospy.Publisher(self.loc_topic, Bool, queue_size=1, latch=True)
        self.start_pub = (rospy.Publisher(self.start_topic, PoseStamped, queue_size=1, latch=True)
                          if self.start_topic else None)
        if self.src_type == 'pose':
            rospy.Subscriber(self.src_topic, PoseStamped, self.on_pose_src, queue_size=5)
        else:
            rospy.Subscriber(self.src_topic, Odometry, self.on_odom, queue_size=5)
        rospy.Timer(rospy.Duration(1.0 / rate), self.tick)
        rospy.loginfo('[sim_pose_source] %s(%s) -> %s @ %.0f Hz; localization_ok -> %s; start -> %s',
                      self.src_topic, self.src_type, self.dst_topic or '(source direct)',
                      rate, self.loc_topic, self.start_topic or '(off)')

    def on_pose_src(self, msg):
        # 源已是契约帧位姿（含正确的支撑点 z 语义）；z 换算交给源侧，
        # 此处原样缓存（z_offset/z_snap 仍可作通用兜底，默认关）。
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose
        if self.z_offset or self.z_snap_band:
            z = pose.pose.position.z + self.z_offset
            if -self.z_snap_band <= z < 0.:
                z = 0.
            pose.pose.position.z = z
        self.latest = pose

    def on_odom(self, msg):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = msg.header.stamp
        pose.pose = msg.pose.pose
        z = pose.pose.position.z + self.z_offset
        if -self.z_snap_band <= z < 0.:
            z = 0.
        pose.pose.position.z = z
        self.latest = pose

    def tick(self, _event):
        if self.latest is None:
            return
        if self.pub is not None:
            self.pub.publish(self.latest)
        if self.start_pub is not None:
            now = time.time()
            if now - self._last_start >= 1.0 / self.start_rate:
                self.start_pub.publish(self.latest)
                self._last_start = now
        if not self.publish_ok:
            return
        now = time.time()
        if now - self._last_ok >= self.ok_period:
            # Republish periodically: the flag means "poses are currently fresh",
            # not "we once had a pose".
            self.ok_pub.publish(Bool(data=True))
            self._last_ok = now


def main():
    rospy.init_node('sim_pose_source')
    SimPoseSource()
    rospy.spin()


if __name__ == '__main__':
    main()
