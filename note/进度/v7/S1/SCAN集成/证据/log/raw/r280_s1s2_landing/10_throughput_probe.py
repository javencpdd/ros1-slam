#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R280 吞吐探针（离线、零窗）——补 ag §1 预检② 缺原件那一笔

ag 稿报过「向量化稳态≈170 ms／最坏帧 362.3／361.7 ms」，但 `r251_3d_spike_workorder/` 里只有文字与摘要、
无脚本可跑 ⇒ 那组数属 A39 的 C 档（无件可核）。本探针用 S1 真件的函数复跑同一形状，数可复现。
参数照设计件 §8：116,747 点／帧、res 0.1 m、窗口 44×24 m ⇒ 440×240＝105,600 格。
"""
import importlib.util
import math
import os
import time

import numpy as np

def _find_root(start):
    d = os.path.abspath(start)
    for _ in range(12):
        if os.path.isdir(os.path.join(d, "ws", "src")):
            return d
        d = os.path.dirname(d)
    raise SystemExit("找不到含 ws/src 的工作根（前置缺失 ⇒ 不猜路径）")


ROOT = _find_root(os.path.dirname(__file__))
NODE = os.path.join(ROOT, "ws/src/30_perception/lite3_perception/scripts/w3_elevation_node.py")
spec = importlib.util.spec_from_file_location("w3_elev", NODE)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

N = 116747
rng = np.random.default_rng(23)
FRAMES = 5
g = mod.ElevGrid(0.1, -22.0, 22.0, -12.0, 12.0)
print("numpy=%s file=%s" % (np.__version__, np.__file__))
print("grid=%dx%d cells=%d points/frame=%d" % (g.ny, g.nx, g.ny * g.nx, N))
for i in range(FRAMES):
    xyz = np.column_stack([rng.uniform(-22, 22, N), rng.uniform(-12, 12, N), rng.uniform(-1.0, 1.5, N)])
    t0 = time.perf_counter()
    got = g.add_points(xyz)
    t1 = time.perf_counter()
    surf = g.p50()
    t2 = time.perf_counter()
    cm, _ = mod.cost_map(g, math.radians(25.0), 0.15, 0.05)
    t3 = time.perf_counter()
    ratio, covered = g.coverage()
    print("frame %d: ingest=%.1f ms p50=%.1f ms cost=%.1f ms total=%.1f ms accepted=%d covered=%d uncovered=%.4f"
          % (i + 1, (t1 - t0) * 1e3, (t2 - t1) * 1e3, (t3 - t2) * 1e3, (t3 - t0) * 1e3,
             got, covered, ratio))
print("counts: frames=%d clipped_to_edge_bins=%d blocked_cells=%d free_cells=%d unknown_cells=%d"
      % (g.frames, g.clipped, int((cm == mod.BLOCKED).sum()), int((cm == mod.FREE).sum()),
         int((cm == mod.UNKNOWN).sum())))
print("THROUGHPUT_PROBE_DONE threshold_ms=500")
