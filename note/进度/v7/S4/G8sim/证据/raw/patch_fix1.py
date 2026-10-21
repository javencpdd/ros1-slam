#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修复补丁①：imu_timer 游标落后缓冲窗时重锚到最新（S5 排障实证：初始锚 buf[0][0] 落后 1.3s+，
   200Hz 实际 ~180Hz 慢进 → 永远追不上 → t_now<=buf[0][0] 每拍早退 → /livox/imu 零输出）"""
P = "/home/ros1-slam/ws/src/00_drivers/lite3_drivers/scripts/sim_adapter.py"
s = open(P).read()

old1 = ("        if self.t_cursor is None:\n"
        "            self.t_cursor = buf[0][0]\n"
        "        self.t_cursor = min(self.t_cursor + self.dt_imu, buf[-1][0])\n"
        "        t_now = self.t_cursor\n"
        "        if t_now <= buf[0][0]:\n"
        "            return\n")
new1 = ("        if self.t_cursor is None or self.t_cursor <= buf[0][0]:\n"
        "            self.t_cursor = buf[-1][0]      # 游标锚定/重锚到最新（落后窗口即追上，防慢进失步）\n"
        "        self.t_cursor = min(self.t_cursor + self.dt_imu, buf[-1][0])\n"
        "        t_now = self.t_cursor\n")
assert old1 in s, "block1 not found"
s = s.replace(old1, new1)
open(P, "w").write(s)
print("fix1 applied, lines:", len(s.splitlines()))
