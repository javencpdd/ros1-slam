# §8 预检 2（吞吐探针）——★只读导入在册件、★不写 ws/、★不开 ROS、★不新增判据
# 口径＝设计件 §8-2：单帧 116,747 点，10Hz 增量融合 ＋ 1Hz 全图重发；阈值＝0.5 s/帧
# 用法：python3 _bench_w8_2.py <件路径> <每帧点数> <帧数>
import importlib.util, math, sys, time
import numpy as np
spec = importlib.util.spec_from_file_location("w3", sys.argv[1])
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
N, FR = int(sys.argv[2]), int(sys.argv[3])
g = m.ElevGrid(0.1, -22.0, 22.0, -12.0, 12.0)
print("格=%d×%d=%d Z_BINS=%d 点数/帧=%d 帧数=%d numpy=%s" % (g.ny, g.nx, g.ny * g.nx, m.Z_BINS, N, FR, np.__version__))
rng = np.random.default_rng(7)
xyz = np.column_stack([rng.uniform(-21.5, 21.5, N), rng.uniform(-11.5, 11.5, N),
                       np.clip(rng.normal(0.0, 0.4, N), -1.9, 1.9)])
t_add = []
for _ in range(FR):
    t0 = time.perf_counter(); g.add_points(xyz); t_add.append(time.perf_counter() - t0)
t0 = time.perf_counter(); out, surf = m.cost_map(g, math.radians(25.0), 0.15, 0.25); t_full = time.perf_counter() - t0
t0 = time.perf_counter(); flat = np.where(out == -1, -1, out).astype(np.int8).ravel().tolist(); t_msg = time.perf_counter() - t0
med = sorted(t_add)[len(t_add) // 2]
per = med + (t_full + t_msg) / 10.0
print("增量融合/帧 min=%.4f med=%.4f max=%.4f" % (min(t_add), med, max(t_add)))
print("全图一次（cost_map 内含 p50/slope/step/rms）=%.4fs；OccupancyGrid 数据转换=%.4fs（元素 %d）" % (t_full, t_msg, len(flat)))
print("★等效单帧 wall（10Hz 增量＋1Hz 全图重发）=%.4fs ⇒ 对 0.5s/帧阈值：%s" % (per, "过" if per < 0.5 else "不过"))
import collections
h = collections.Counter(map(int, out.ravel().tolist()))
print("输出三值分布＝%s（UNKNOWN=-1/FREE=0/BLOCKED=100；零覆盖列应落 -1）" % dict(h))
