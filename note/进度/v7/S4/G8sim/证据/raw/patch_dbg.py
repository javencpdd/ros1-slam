#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""临时补丁：给 sim_adapter.imu_timer 加节流调试行（S5 排障用，排障后由 revert 脚本摘除）"""
P = "/home/ros1-slam/ws/src/00_drivers/lite3_drivers/scripts/sim_adapter.py"
s = open(P).read()
anchor = ("        buf = self.pose_buf\n"
          "        if len(buf) < 5 or self.last_pose_wall is None:\n"
          "            return")
dbg = ("        buf = self.pose_buf\n"
       "        self._dbg = getattr(self, '_dbg', 0) + 1\n"
       "        if self._dbg % 1000 == 0:\n"
       "            b = self.pose_buf\n"
       "            rospy.loginfo_throttle(5, '[sim_adapter][imu_dbg] calls=%d buflen=%d wallgap=%.3f cur=%.3f b0=%.3f bend=%.3f',\n"
       "                                    self._dbg, len(b), (rospy.get_rostime() - (self.last_pose_wall or rospy.Time(0))).to_sec(),\n"
       "                                    self.t_cursor or -1, b[0][0] if b else -1, b[-1][0] if b else -1)\n"
       "        if len(buf) < 5 or self.last_pose_wall is None:\n"
       "            return")
assert anchor in s, "anchor not found"
s = s.replace(anchor, dbg)
open(P, "w").write(s)
print("dbg patch applied, lines:", len(s.splitlines()))
