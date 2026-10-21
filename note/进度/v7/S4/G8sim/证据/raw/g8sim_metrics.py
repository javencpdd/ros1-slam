#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# g8sim_metrics.py —— G8-sim 返工R1 单轮判据计算
# 用法: python3 g8sim_metrics.py <前缀>   （前缀=R1_run{i} 完整路径，无扩展名）
# 判据（小张设计稿第四节）：漂移≤0.3m / z∈[-0.3,0.3] / 末5s零漂≤0.05m·0.01rad / 末端均值≤0.3m /
#                          grid≤20000 且末20s增速<10/s / effect≥100 / ≥5Hz / 零ERROR
import re
import sys

import numpy as np
import pandas as pd

prefix = sys.argv[1]
odom = pd.read_csv(prefix + '.odom.csv')
gt = pd.read_csv(prefix + '.gt.csv')

print('== 采集规模 ==')
print('odom rows=%d  gt rows=%d' % (len(odom), len(gt)))

# ---- 频率与跑满（wall 域）----
w = odom['wall'].values
dt = np.diff(w)
dur = w[-1] - w[0]
rate = (len(odom) - 1) / dur
print('== 频率/时长 ==')
print('wall 覆盖 %.1fs  /Odometry 平均率 %.2f Hz  dt 中位 %.3fs  dt 最大 %.3fs' % (
    dur, rate, np.median(dt), dt.max()))

# ---- 帧对齐：GT 平移至 odom 帧（p_rel = p_gt - p_gt[0]）----
gt0 = gt[['x', 'y', 'z']].values[0]
grel = gt[['x', 'y', 'z']].values - gt0
gt_t = gt['stamp'].values
yaw0 = gt['yaw'].values[0]
print('GT 起点=%.3f,%.3f,%.3f  GT 起始 yaw=%.4f rad  GT 终点相对起点偏差=%.4fm' % (
    gt0[0], gt0[1], gt0[2], yaw0,
    np.linalg.norm(grel[-1] - grel[0])))

# ---- 按戳配对 GT ----
oi = np.searchsorted(gt_t, odom['stamp'].values)
oi = np.clip(oi, 1, len(gt) - 1)
pick = np.where(np.abs(gt_t[oi] - odom['stamp'].values) < np.abs(gt_t[oi - 1] - odom['stamp'].values), oi, oi - 1)
gap = np.abs(gt_t[pick] - odom['stamp'].values)
ok = gap <= 0.2
print('戳配对: %d/%d 样本 gap≤0.2s（最大 gap %.3fs）' % (ok.sum(), len(odom), gap.max()))

oxyz = odom[['x', 'y', 'z']].values
err = np.linalg.norm(oxyz - grel[pick], axis=1)
err_o = err[ok]
t_o = w[ok]
i_max = np.argmax(err_o)
print('== 判据1 全程漂移（对 GT 折线，含 z）==')
print('max=%.3fm @t=%.1fs  mean=%.3fm  p99=%.3fm' % (err_o.max(), t_o[i_max] - t_o[0], err_o.mean(), np.percentile(err_o, 99)))

z = odom['z'].values
print('== 判据1b z 范围 ==')
print('z min=%.3f max=%.3f（判据 [-0.3,0.3]）' % (z.min(), z.max()))

# ---- 末 5s 静置段 ----
m5 = w >= w[-1] - 5.0
p5 = oxyz[m5]
yaw5 = odom['yaw'].values[m5]
drift_pos = np.linalg.norm(p5.max(axis=0) - p5.min(axis=0))
span_yaw = yaw5.max() - yaw5.min()
end_mean = err[m5].mean()
end_max = err[m5].max()
print('== 判据2 末 5s 静置段（样本 %d）==' % m5.sum())
print('零漂 |Δpos| ptp=%.4fm（≤0.05）  |Δyaw| ptp=%.4frad（≤0.01）' % (drift_pos, span_yaw))
print('末端 vs GT 起点均值=%.3fm（≤0.3）  末段 max=%.3fm' % (end_mean, end_max))

# ---- faster_lio 日志 grid/effect ----
pat = re.compile(r'I\d{4} (\d\d:\d\d:\d\d\.\d+)\s+\d+ laser_mapping.*Map grid num: (\d+) effect num : (\d+)')
rows = []
try:
    f = open(prefix + '.lio.log', errors='replace')
except OSError:
    f = open(prefix + '.lio.log')
for line in f:
    m = pat.search(line)
    if m:
        hh, mm, ss = m.group(1).split(':')
        rows.append((int(hh) * 3600 + float(mm) * 60 + float(ss), int(m.group(2)), int(m.group(3))))
f.close()
if rows:
    a = np.array(rows, dtype=float)
    a[:, 0] -= a[0, 0]
    print('== 判据3 地图收敛（log 行 %d）==' % len(a))
    print('首行 t=%.1fs grid=%d effect=%d' % (a[0, 0], a[0, 1], a[0, 2]))
    print('终值 grid=%d effect=%d @t=%.1fs' % (a[-1, 1], a[-1, 2], a[-1, 0]))
    m20 = a[:, 0] >= a[-1, 0] - 20.0
    b = a[m20]
    growth = a[-1, 1] - b[0, 1]
    print('末 20s grid 增速=%d grid / %.1fs = %.2f grid/s（<10）' % (growth, a[-1, 0] - b[0, 0], growth / max(a[-1, 0] - b[0, 0], 1e-9)))
    print('末 20s effect: min=%d 终值=%d  全程 effect min(首10s后)=%d' % (
        b[:, 2].min(), a[-1, 2], a[a[:, 0] > 10, 2].min()))
    print('grid 峰值=%d @t=%.1fs' % (a[:, 1].max(), a[np.argmax(a[:, 1]), 0]))
else:
    print('!! lio 日志无 grid/effect 行')

# ---- ERROR ----
import subprocess
for tag in ('lio', 'sim', 'overall', 'roscore'):
    try:
        txt = open(prefix + '.' + tag + '.log', errors='replace').read()
        n1 = txt.count('ERROR')
        n2 = len(re.findall(r'^E\d{4}', txt, re.M))
        print('ERROR 计数 [%s]: ERROR 字样=%d  glog E行=%d' % (tag, n1, n2))
    except OSError:
        print('ERROR 计数 [%s]: 文件缺失' % tag)
