#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修复补丁③rev2：IMU stamp 改墙钟域（对 tuple 赋值行）"""
P = "/home/ros1-slam/ws/src/00_drivers/lite3_drivers/scripts/sim_adapter.py"
s = open(P).read()

old1 = ("        if self.t_cursor is None or self.t_cursor <= buf[0][0]:\n"
        "            self.t_cursor = buf[-1][0]      # 游标锚定/重锚到最新（落后窗口即追上，防慢进失步）\n"
        "        self.t_cursor = min(self.t_cursor + self.dt_imu, buf[-1][0])\n"
        "        t_now = self.t_cursor\n")
new1 = ("        t_now = buf[-1][0]                  # 插值目标=最新位姿时刻（墙钟域，防爬行/跳变 dt 畸变）\n")
assert old1 in s, "block1 not found"
s = s.replace(old1, new1)

old2 = "        im.header.stamp, im.header.frame_id = rospy.Time(t_now), 'livox_frame'\n"
new2 = "        im.header.stamp, im.header.frame_id = rospy.get_rostime(), 'livox_frame'  # fix③墙钟域\n"
assert old2 in s, "block2 not found"
s = s.replace(old2, new2)
open(P, "w").write(s)
print("fix3rev2 applied, lines:", len(s.splitlines()))
