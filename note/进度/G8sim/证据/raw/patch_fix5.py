#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修复⑤（S5 发散真根因）：Δq 奇异回退从"上一拍输出 prev_wb"改为"上一帧测量值 last_ww_meas"。
   实证：静置时 dq≡identity→None→回退 prev_wb（含偏置+噪声）→ 噪声正反馈随机游走
   （ww_filt 实测 4.98→5.93 rad/s 持续爬升）→ faster_lio 积分假旋转发散至 1e4~1e5 m。
   修复：新增 last_ww_meas（世界系 ω 测量，仅在真实 Δq 时更新）；删除 prev_wb 回喂。"""
P = "/home/ros1-slam/ws/src/00_drivers/lite3_drivers/scripts/sim_adapter.py"
s = open(P).read()

old1 = "        self.prev_wb = np.zeros(3)\n"
new1 = "        self.last_ww_meas = np.zeros(3)     # 世界系 ω 测量值（奇异帧沿用；不回喂输出，防噪声反馈）\n"
assert old1 in s, "init not found"
s = s.replace(old1, new1)

old2 = ("        ww = world_gyro_w(buf[-1][2], buf[-2][2], buf[-1][0] - buf[-2][0])\n"
        "        if ww is None:\n"
        "            ww = self.prev_wb.copy() if np.any(self.prev_wb) else np.zeros(3)\n"
        "        self.gyro_med.append(ww)\n")
new2 = ("        ww = world_gyro_w(buf[-1][2], buf[-2][2], buf[-1][0] - buf[-2][0])\n"
        "        if ww is None:\n"
        "            ww = self.last_ww_meas          # 静置/奇异：沿用上帧测量（payload 03 坑②语义）\n"
        "        else:\n"
        "            self.last_ww_meas = ww\n"
        "        self.gyro_med.append(ww)\n")
assert old2 in s, "gyro block not found"
s = s.replace(old2, new2)

old3 = ("        w_b = R.T @ self.ww_filt + GYRO_BIAS + self.rng.normal(0, GYRO_SIGMA, 3)\n"
        "        self.prev_wb = w_b.copy()\n")
new3 = "        w_b = R.T @ self.ww_filt + GYRO_BIAS + self.rng.normal(0, GYRO_SIGMA, 3)\n"
assert old3 in s, "wb block not found"
s = s.replace(old3, new3)

open(P, "w").write(s)
print("fix5 applied, lines:", len(s.splitlines()))
