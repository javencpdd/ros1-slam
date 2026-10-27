"""D1 图审计：扫描点（经外参+GT 投到世界）到先验图的最近邻距离（xy）。"""
import json
import math
import sys

import numpy as np
from scipy.spatial import cKDTree

SCANS = sys.argv[1] if len(sys.argv) > 1 else '/work/map_audit_scans.json'
OUT = sys.argv[2] if len(sys.argv) > 2 else '/work/map_audit.json'
MAPS = {
    'venue_prior_185k': '/work/maps/lite3.pcd',
    'sim_venue': '/work/maps/sim_venue.pcd',
    'repo_lite3': '/work/maps/repo_lite3.pcd',
}


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


P = math.radians(15.0)
C, S = math.cos(P), math.sin(P)
R_L2B = np.array([[C, 0, S], [0, 1, 0], [-S, 0, C]])
T_L2B = np.array([0.0, 0.0, 0.34])

trees = {}
for name, path in MAPS.items():
    pts = read_pcd(path)
    # 只用高于地面的点做覆盖审计，避免地面点造成假覆盖
    if name == 'sim_venue':
        sel = pts[(pts[:, 2] > -0.30)]
    else:
        sel = pts[(pts[:, 2] > 0.30)]
    trees[name] = (cKDTree(sel[:, :2]), sel.shape[0])
    print('map', name, 'points', pts.shape[0], 'audit_points', sel.shape[0])

data = json.load(open(SCANS))
report = []
for item in data:
    x, y, yaw = item['pose']
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                   [math.sin(yaw), math.cos(yaw), 0],
                   [0, 0, 1]])
    row = {'pose': item['pose'], 'gt': item['gt'], 'maps': {}}
    clouds = [np.array(s) for s in item['scans'] if len(s) > 100]
    if not clouds:
        report.append(row)
        continue
    for name, (tree, npts) in trees.items():
        dists = []
        for pts in clouds:
            pb = pts @ R_L2B.T + T_L2B
            pw = pb @ Rz.T + np.array([x, y, 0.11])
            band = (pw[:, 2] > 0.2) & (pw[:, 2] < 1.5)
            if not band.any():
                continue
            d, _ = tree.query(pw[band, :2], k=1)
            dists.append(d)
        if not dists:
            row['maps'][name] = None
            continue
        d = np.concatenate(dists)
        row['maps'][name] = {
            'n': int(d.size),
            'p50': round(float(np.percentile(d, 50)), 4),
            'p95': round(float(np.percentile(d, 95)), 4),
            'max': round(float(d.max()), 4),
            'frac_gt_0p2': round(float((d > 0.2).mean()), 4),
            'frac_gt_0p5': round(float((d > 0.5).mean()), 4),
        }
    report.append(row)

with open(OUT, 'w') as f:
    json.dump(report, f, indent=1)
print(json.dumps(report, indent=1))
