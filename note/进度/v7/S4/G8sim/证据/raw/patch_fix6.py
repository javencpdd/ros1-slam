#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修复⑥（发散第二根因）：/rslidar_points 保留 render 原始 stamp（=生成该帧的位姿时刻），
   不再以 rospy.now() 覆盖。实证链：render 内部对 body_pose 有 0.2~0.6s 计算延迟且
   sensor_cloud.stamp=odom 戳（内容时刻）→ adapter 改 now() → 点云内容与 IMU/位姿时刻
   系统性错位 ~0.3s（0.5m/s 下 0.15m/帧）→ faster_lio 配准残差系统性偏置 → 发散。
   payload 03 原文 stamp=now() 隐含"点云实时到达"假设，被 render 延迟打破——偏差登记 04。"""
P = "/home/ros1-slam/ws/src/00_drivers/lite3_drivers/scripts/sim_adapter.py"
s = open(P).read()
old = ("        pc.header.stamp = rospy.Time.now()\n"
       "        self.pub_pts.publish(pc)\n")
new = ("        # 保留 render 原戳（内容时刻），fix⑥：now() 覆盖曾致点云-IMU 系统性时域错位\n"
       "        self.pub_pts.publish(pc)\n")
assert old in s, "stamp line not found"
s = s.replace(old, new)
open(P, "w").write(s)
print("fix6 applied, lines:", len(s.splitlines()))
