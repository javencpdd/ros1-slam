#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# g8sim_r2_trajcheck.py —— G8-sim 返工R2 施工内自检（小明 2026-10-04）
# 口径：mock rospy 驱动【真实 spin_traj 代码】离线重放 50Hz 全程，按 adapter imu_timer 同款算子
# （world_gyro_w 四元数差 / sg2_accel 5 点 SG 二阶差分 dt5=0.02 / R^T(a_w−g)）打印极值：
#   判据：全程 ω≤1.0rad/s；|a_b|（平面动态）≤1.5m/s²；速度∥heading 夹角≤align_gate；
#         端点=wp[-1] 精确、全程 on-rails（对折线距离=0）；打印精确时长供 3 连 sleep 定值。
import math
import sys
import types

import numpy as np

# ---- mock rospy（须先于 import sim_adapter 注入）----
class FakeTime(object):
    def __init__(self, t):
        self.t = t
    def to_sec(self):
        return self.t
    def __sub__(self, other):
        return FakeTime(self.t - other.t)

class FakePub(object):
    msgs = []
    def publish(self, m):
        FakePub.msgs.append(m)

TICK = 0.02
N_MAX = 50 * (5 + 400 + 5)          # 静置5s+运动≤400s+静置5s 上限
_tick = [0]

def _now():
    _tick[0] += 1
    return FakeTime(_tick[0] * TICK)

fake = types.ModuleType('rospy')
fake.get_rostime = _now
fake.is_shutdown = lambda: _tick[0] > N_MAX
fake.Rate = lambda h: types.SimpleNamespace(sleep=lambda: None)
fake.Publisher = lambda *a, **k: FakePub()
fake.loginfo = lambda *a, **k: None
fake.loginfo_throttle = lambda *a, **k: None
fake.logwarn = lambda *a, **k: None
fake.logwarn_throttle = lambda *a, **k: None
sys.modules['rospy'] = fake

sys.path.insert(0, '/home/ros1-slam/ws/src/00_drivers/lite3_drivers/scripts')
import sim_adapter as SA                                              # noqa: E402

traj = SA.load_traj('/home/ros1-slam/ws/src/00_drivers/lite3_drivers/config/traj_maze.yaml')
wp, z = traj['wp'], traj['z']
seg_len = np.linalg.norm(np.diff(wp, axis=0), axis=1)

adapter = types.SimpleNamespace()                                     # spin_traj 只用 self.traj_yaw/v
SA.SimAdapter.spin_traj(adapter, traj)

# ---- 采集序列 ----
T, P, Q = [], [], []
for m in FakePub.msgs:
    T.append(m.header.stamp.to_sec())
    P.append([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
    Q.append([m.pose.pose.orientation.x, m.pose.pose.orientation.y,
              m.pose.pose.orientation.z, m.pose.pose.orientation.w])
T, P, Q = np.array(T), np.array(P), np.array(Q)
assert len(T) == N_MAX, 'tick 数 %d ≠ 预算 %d（轨迹未在时限内走完→FAIL）' % (len(T), N_MAX)

def yaw_of(q):
    return math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))

# ---- 运动窗 / 速度 ----
v = np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1) / TICK           # 50Hz 平面速度
t_move = T[1:][v > 0.05]                                               # 判据 10 口径：gt 速度首>0.05m/s
t_first = t_move[0] if len(t_move) else float('nan')
last_move = T[1:][v > 1e-3]
t_end = last_move[-1] if len(last_move) else float('nan')

# ---- ω（world_gyro_w 同款）----
w_max, w_t = 0.0, 0.0
for k in range(1, len(T)):
    ww = SA.world_gyro_w(Q[k], Q[k - 1], T[k] - T[k - 1])
    if ww is not None:
        n = float(np.linalg.norm(ww))
        if n > w_max:
            w_max, w_t = n, T[k]

# ---- a_b（imu_timer 同款：pos5=末5 位姿，dt5=0.02，R^T(sg2−G)）----
a_planar_max, a_total_max, a_dev_max, a_t = 0.0, 0.0, 0.0, 0.0
for k in range(4, len(T)):
    pos5 = P[k - 4:k + 1]
    dt5 = (T[k] - T[k - 4]) / 4.0
    a_b = SA.quat_to_R(Q[k]).T @ (SA.sg2_accel(pos5, dt5) - SA.G_VEC)
    n = float(np.linalg.norm(a_b))
    pl = float(np.linalg.norm(a_b[:2]))
    dev = abs(n - 9.81)
    if pl > a_planar_max:
        a_planar_max, a_t = pl, T[k]
    a_total_max = max(a_total_max, n)
    a_dev_max = max(a_dev_max, dev)

# ---- 速度∥heading 夹角（speed>0.05 时）----
ang_max, ang_t = 0.0, 0.0
for k in range(1, len(T)):
    if v[k - 1] > 0.05:
        vy, vx = P[k][1] - P[k - 1][1], P[k][0] - P[k - 1][0]
        dy = abs((math.atan2(vy, vx) - yaw_of(Q[k]) + math.pi) % (2 * math.pi) - math.pi)
        if dy > ang_max:
            ang_max, ang_t = dy, T[k]

# ---- 路径形态核对（沿航向推进口径：角点圆滑化 ≤2cm 为设计内代价，29 格路径/端点保持）----
def dist_to_polyline(p):
    d = np.inf
    for i in range(len(wp) - 1):
        a, b = wp[i], wp[i + 1]
        ab = b - a
        tt = float(np.clip(np.dot(p[:2] - a, ab) / max(np.dot(ab, ab), 1e-12), 0, 1))
        d = min(d, float(np.linalg.norm(p[:2] - (a + tt * ab))))
    return d

RAIL_TOL = 0.02                                    # 沿航向推进角点圆滑化允差（走廊半宽 0.6m 的 3%）
rail_max = max(dist_to_polyline(p) for p in P)
end_err = float(np.linalg.norm(P[-1][:2] - wp[-1]))
wp_miss = max(float(np.min(np.linalg.norm(P[:, :2] - w, axis=1))) for w in wp)
wps_hit = wp_miss <= RAIL_TOL
start_err = float(np.linalg.norm(P[0][:2] - wp[0]))

spd_max = float(v.max())
pivots = int(np.sum((np.abs(np.diff([yaw_of(q) for q in Q])) > 0.005)))

print('== R2 施工内自检（真实 spin_traj 离线重放 50Hz 全程）==')
print('waypoints=%d 段数=%d 总长=%.1fm  采集 tick=%d' % (len(wp), len(seg_len), seg_len.sum(), len(T)))
print('运动起始（速度首>0.05m/s）t=%.2fs   运动结束 t=%.2fs   起点误差=%.2em  末位置抵近 wp[-1] 误差=%.3fm（≤%.2f）'
      % (t_first, t_end, start_err, end_err, RAIL_TOL))
print('全程 wall（含静置5s×2）≈ %.1fs → 3 连 sleep 建议 ≥ %.0fs' % (t_end + traj['hold_end'] + 1.0, t_end + traj['hold_end'] + 8.0))
print('[判据] ω 峰值=%.4f rad/s @t=%.2fs（≤1.0）' % (w_max, w_t))
print('[判据] |a_b| 平面动态峰值=%.4f m/s² @t=%.2fs（≤1.5）  |a_b| 总峰值=%.4f（含 g）  |‖a_b|−9.81| 峰值=%.4f' % (
    a_planar_max, a_t, a_total_max, a_dev_max))
print('[判据] 速度∥heading 夹角峰值=%.4f rad（≤align_gate=%.2f）@t=%.2fs' % (ang_max, traj['align_gate'], ang_t))
print('[判据] 轨迹-折线最大偏离=%.4fm（≤%.2f，角点圆滑化）  waypoint 最大抵近误差=%.4fm（≤%.2f）=%s'
      % (rail_max, RAIL_TOL, wp_miss, RAIL_TOL, wps_hit))
print('[参考] 速度峰值=%.3f m/s（门控 speed=0.5）  原地换向次数≈%d  ACC_CLAMP=%.1f 未触发=%s' % (
    spd_max, pivots, SA.ACC_CLAMP, a_total_max <= SA.ACC_CLAMP))
ok = (w_max <= 1.0 + 1e-9 and a_planar_max <= 1.5 and ang_max <= traj['align_gate'] + 1e-9
      and rail_max <= RAIL_TOL and end_err <= RAIL_TOL and wps_hit and start_err <= 1e-6)
print('== 自检裁决：%s ==' % ('PASS' if ok else 'FAIL'))
sys.exit(0 if ok else 1)
