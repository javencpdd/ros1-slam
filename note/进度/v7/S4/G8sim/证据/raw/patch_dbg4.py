#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""补丁④：imu_dbg 扩展为全量 dump（dt/dq 模长/ww_filt/prev_wb/q0/q1）"""
P = "/home/ros1-slam/ws/src/00_drivers/lite3_drivers/scripts/sim_adapter.py"
s = open(P).read()
old = ("        if self._dbg % 1000 == 0:\n"
       "            b = self.pose_buf\n"
       "            rospy.loginfo_throttle(5, '[sim_adapter][imu_dbg] calls=%d buflen=%d wallgap=%.3f cur=%.3f b0=%.3f bend=%.3f',\n"
       "                                    self._dbg, len(b), (rospy.get_rostime() - (self.last_pose_wall or rospy.Time(0))).to_sec(),\n"
       "                                    self.t_cursor or -1, b[0][0] if b else -1, b[-1][0] if b else -1)\n")
new = ("        if self._dbg % 200 == 0:\n"
       "            b = self.pose_buf\n"
       "            dq = quat_mul(buf[-1][2], np.array([-buf[-2][2][0], -buf[-2][2][1], -buf[-2][2][2], buf[-2][2][3]])) if len(b) >= 2 else None\n"
       "            rospy.loginfo_throttle(2, '[sim_adapter][imu_dbg] calls=%d dt=%.4f dq=%s ww_filt=%s prev_wb=%s q0=%s q1=%s',\n"
       "                                    self._dbg, b[-1][0] - b[-2][0] if len(b) >= 2 else -1,\n"
       "                                    np.round(dq, 4).tolist() if dq is not None else 'NA',\n"
       "                                    np.round(self.ww_filt, 3).tolist(), np.round(self.prev_wb, 3).tolist(),\n"
       "                                    np.round(b[-2][2], 3).tolist() if len(b) >= 2 else 'NA',\n"
       "                                    np.round(b[-1][2], 3).tolist() if len(b) >= 1 else 'NA')\n")
assert old in s, "dbg block not found"
s = s.replace(old, new)
open(P, "w").write(s)
print("dbg4 applied, lines:", len(s.splitlines()))
