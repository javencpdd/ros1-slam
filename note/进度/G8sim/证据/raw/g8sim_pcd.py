#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# g8sim_pcd.py —— pcd 判据分析（点数 3e4–5e5 / world z<-0.05 占比 <0.1%）
# 用法: python3 g8sim_pcd.py <pcd路径>
import sys

import numpy as np

path = sys.argv[1]
meta = {}
with open(path, 'rb') as f:
    while True:
        line = f.readline()
        s = line.decode('ascii', 'replace').strip()
        if s.startswith('#') or not s:
            continue
        k, _, v = s.partition(' ')
        meta[k] = v
        if k == 'DATA':
            if v != 'binary':
                print('FATAL: DATA=%s 未实现' % v)
                sys.exit(2)
            break
    body = f.read()
n = int(meta['POINTS'])
pts = np.frombuffer(body[:n * 32], dtype=np.float32).reshape(n, 8)
x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
inside = (np.abs(x) <= 12) & (np.abs(y) <= 12)
ratio = float((z < -0.05).sum()) / n * 100.0
print('== pcd 判据: %s ==' % path)
print('POINTS=%d（判据 3e4–5e5）  z min=%.3f max=%.3f' % (n, z.min(), z.max()))
print('z<-0.05 点数=%d 占比=%.4f%%（<0.1%%）' % ((z < -0.05).sum(), ratio))
print('|x|,|y|≤12m 界内占比=%.2f%%（场地合理性参考）' % (inside.mean() * 100))
