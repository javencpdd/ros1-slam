#!/usr/bin/env python3
"""R787：B 方案读数——9 有效 run 的 LIO 单后端精度 + NDT 监控器（NDT vs LIO 分歧）。

来源：R785 的 9 份 JSON（1 份/run）＋本目录 csv/ 下同 run 的逐帧 CSV（_lio/_ndt）。
口径：xy 为初始化窗 SE(2) 对齐后的 base_link 水平误差；NDT 与 LIO 各自用 JSON 里的
se2_psi_rad/se2_t 统一到 GT 系后同帧相减（离线评估，非运行时 map→odom 监视量）。
只读，不改任何配置。
"""
import bisect
import csv
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
EV785 = os.path.join(HERE, '..', 'R785')
VALID = ['l9b_s1_r1', 'l9b_s2_r2', 'l9b_s3_r3', 'l9b_s7_r1', 'l9b_s8_r2', 'l9b_s9_r3',
         'l9c_s1_r1', 'l9c_s2_r2', 'l9c_s3_r3']


def load_csv(path):
    return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(path))]


def apply_se2(x, y, psi, tx, ty):
    c, s = math.cos(psi), math.sin(psi)
    return c * x - s * y + tx, s * x + c * y + ty


def q(a, p):
    a = sorted(a)
    return a[min(len(a) - 1, int(round(p * (len(a) - 1))))]


out = []
for tag in VALID:
    d = json.load(open(os.path.join(EV785, tag + '.json')))
    lio, ndt = d['backends']['lio'], d['backends']['ndt']
    l = load_csv(os.path.join(HERE, 'csv', tag + '_lio.csv'))
    n = load_csv(os.path.join(HERE, 'csv', tag + '_ndt.csv'))
    lt = [r['t'] for r in l]
    diffs = []
    for r in n:
        i = bisect.bisect_left(lt, r['t'])
        cand = [j for j in (i - 1, i) if 0 <= j < len(l)]
        j = min(cand, key=lambda j: abs(lt[j] - r['t']))
        if abs(lt[j] - r['t']) > 0.05:
            continue
        nx, ny = apply_se2(r['est_x'], r['est_y'], ndt['se2_psi_rad'],
                           ndt['se2_t'][0], ndt['se2_t'][1])
        lx, ly = apply_se2(l[j]['est_x'], l[j]['est_y'], lio['se2_psi_rad'],
                           lio['se2_t'][0], lio['se2_t'][1])
        diffs.append(math.hypot(nx - lx, ny - ly))
    t0 = n[0]['t']
    head = [r['ali_xy'] for r in n if r['t'] - t0 <= 2.0]
    out.append({
        'run': tag,
        'start_gt_xy': '%0.3f,%0.3f' % (l[0]['gt_x'], l[0]['gt_y']),
        'path_m': round(d['gt_path_len_m'], 3),
        'lio_max': round(lio['ali_xy']['max'], 4),
        'lio_p95': round(lio['ali_xy']['p95'], 4),
        'lio_p50': round(lio['ali_xy']['p50'], 4),
        'ndt_max': round(ndt['ali_xy']['max'], 4),
        'ndt_p95': round(ndt['ali_xy']['p95'], 4),
        'ndt_p50': round(ndt['ali_xy']['p50'], 4),
        'ndt_rej': ndt['n_rejected'],
        'ndt_err_head2s_max': round(max(head), 4),
        'nd_lio_diff_n': len(diffs),
        'nd_lio_diff_max': round(max(diffs), 4),
        'nd_lio_diff_p95': round(q(diffs, 0.95), 4),
    })

print(json.dumps(out, ensure_ascii=False, indent=1))
print('\n== 汇总（n=9）==')
for key in ('lio_max', 'lio_p95', 'ndt_max', 'ndt_p95', 'ndt_err_head2s_max',
            'nd_lio_diff_max', 'nd_lio_diff_p95'):
    v = [r[key] for r in out]
    print('%-20s min=%.4f max=%.4f' % (key, min(v), max(v)))
print('LIO max<=0.194: %d/9' % sum(1 for r in out if r['lio_max'] <= 0.194))
print('LIO p95<=0.12 : %d/9' % sum(1 for r in out if r['lio_p95'] <= 0.12))
print('1.15*LIO_max_max = %.4f -> ceil_cm %.2f'
      % (1.15 * max(r['lio_max'] for r in out),
         math.ceil(1.15 * max(r['lio_max'] for r in out) * 100) / 100))
