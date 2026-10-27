"""图审计 v2：逐位姿 SE(2) 搜索最优对齐（世界->图），再算 NN 与 z 残差。

正控：(0,0) 位姿上场地图应找到接近 0 的平移/旋转且 NN 很小（NDT 在此能锁定）。
"""
import json
import math
import sys

import numpy as np
from scipy.spatial import cKDTree

SCANS = sys.argv[1] if len(sys.argv) > 1 else '/work/map_audit_scans.json'
OUT = sys.argv[2] if len(sys.argv) > 2 else '/work/map_audit_v2.json'
MAPS = {
    'venue_prior_185k': '/work/maps/lite3.pcd',
    'sim_venue': '/work/maps/sim_venue.pcd',
    'repo_lite3': '/work/maps/repo_lite3.pcd',
}
P = math.radians(15.0)
C, S = math.cos(P), math.sin(P)
R_L2B = np.array([[C, 0, S], [0, 1, 0], [-S, 0, C]])
T_L2B = np.array([0.0, 0.0, 0.34])


def read_pcd(path):
    f = open(path, 'rb')
    while True:
        line = f.readline().decode('ascii', 'ignore').strip()
        if line.startswith('FIELDS'):
            fields = line.split()[1:]
        if line.startswith('POINTS'):
            n = int(line.split()[1])
        if line.startswith('DATA'):
            break
    dt = np.dtype([(name, '<f4') for name in fields])
    arr = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
    return np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float64)


maps = {}
for name, path in MAPS.items():
    pts = read_pcd(path)
    maps[name] = (pts, cKDTree(pts[:, :2]))
    print('map', name, pts.shape)

data = json.load(open(SCANS))
report = []
for item in data:
    x, y, yaw = item['pose']
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                   [math.sin(yaw), math.cos(yaw), 0],
                   [0, 0, 1]])
    clouds = []
    for s in item['scans']:
        pts = np.array(s)
        if pts.shape[0] < 200:
            continue
        pb = pts @ R_L2B.T + T_L2B
        pw = pb @ Rz.T + np.array([x, y, 0.11])
        if pw.shape[0] > 800:
            idx = np.linspace(0, pw.shape[0] - 1, 800).astype(int)
            pw = pw[idx]
        clouds.append(pw)
    row = {'pose': item['pose'], 'gt': item['gt'], 'maps': {}}
    for name, (mpts, tree) in maps.items():
        best = None
        for dyaw_deg in np.arange(-6, 6.01, 2.0):
            th = math.radians(dyaw_deg)
            R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
            for dx in np.arange(-0.8, 0.81, 0.2):
                for dy in np.arange(-0.8, 0.81, 0.2):
                    ds = []
                    zs = []
                    for pw in clouds:
                        p2 = pw[:, :2] @ R.T + np.array([dx, dy])
                        d, idx = tree.query(p2, k=1)
                        ds.append(d)
                        zs.append(mpts[idx, 2] - pw[:, 2])
                    d = np.concatenate(ds)
                    # 修剪均值：取最小的 80%，抵抗远端未覆盖点
                    k = int(0.8 * d.size)
                    score = float(np.sort(d)[:k].mean())
                    if best is None or score < best['score']:
                        best = {'score': score, 'dx': round(dx, 2), 'dy': round(dy, 2),
                                'dyaw_deg': round(dyaw_deg, 1), 'd': d,
                                'z': np.concatenate(zs)}
        d = best['d']
        z = best['z']
        row['maps'][name] = {
            'shift': [best['dx'], best['dy'], best['dyaw_deg']],
            'trim_mean': round(best['score'], 4),
            'p50': round(float(np.percentile(d, 50)), 4),
            'p95': round(float(np.percentile(d, 95)), 4),
            'max': round(float(d.max()), 4),
            'frac_gt_0p2': round(float((d > 0.2).mean()), 4),
            'z_res_p50': round(float(np.percentile(z, 50)), 4),
            'z_res_p95': round(float(np.percentile(z, 95)), 4),
        }
        print('pose', item['pose'], name, row['maps'][name])
    report.append(row)

with open(OUT, 'w') as f:
    json.dump(report, f, indent=1)
print('written', OUT)
