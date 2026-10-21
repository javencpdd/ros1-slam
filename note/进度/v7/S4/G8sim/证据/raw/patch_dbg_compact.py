#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""补丁⑥：imu_dbg 块压缩 10→3 行（总 306→299 行，守 payload 03 ≤300 行约束）"""
P = "/home/ros1-slam/ws/src/00_drivers/lite3_drivers/scripts/sim_adapter.py"
s = open(P).read()
old = ("        if self._dbg % 200 == 0:\n"
       "            b = self.pose_buf\n"
       "            dq = quat_mul(buf[-1][2], np.array([-buf[-2][2][0], -buf[-2][2][1], -buf[-2][2][2], buf[-2][2][3]])) if len(b) >= 2 else None\n"
       "            rospy.loginfo_throttle(2, '[sim_adapter][imu_dbg] calls=%d dt=%.4f dq=%s ww_filt=%s prev_wb=%s q0=%s q1=%s',\n"
       "                                    self._dbg, b[-1][0] - b[-2][0] if len(b) >= 2 else -1,\n"
       "                                    np.round(dq, 4).tolist() if dq is not None else 'NA',\n"
       "                                    np.round(self.ww_filt, 3).tolist(), np.round(self.prev_wb, 3).tolist(),\n"
       "                                    np.round(b[-2][2], 3).tolist() if len(b) >= 2 else 'NA',\n"
       "                                    np.round(b[-1][2], 3).tolist() if len(b) >= 1 else 'NA')\n")
new = ("        if self._dbg % 200 == 0 and len(self.pose_buf) >= 2:\n"
       "            b = self.pose_buf\n"
       "            rospy.loginfo_throttle(2, '[imu_dbg] calls=%d dt=%.4f ww_filt=%s q1=%s' % (self._dbg, b[-1][0] - b[-2][0], np.round(self.ww_filt, 3).tolist(), np.round(b[-1][2], 3).tolist()))\n")
assert old in s, "dbg block not found"
s = s.replace(old, new)
open(P, "w").write(s)
print("dbg compact applied, lines:", len(s.splitlines()))
