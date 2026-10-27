"""R743 口径复核：t_pre 起算的净/峰值位移、峰值净空、帧率与 t_post/上一无墙帧的感知差。"""
import json

RUNS = range(1, 10)
out = []
for i in RUNS:
    ev = json.load(open('/work/scan_stop_s%d.event.json' % i))
    d = json.load(open('/work/scan_stop_s%d.json' % i))
    t_pre, t_post = ev['t_pre'], ev['t_post']
    face = ev['wall']['x'] - ev['wall']['hx']
    gt = d['gt']
    cmds = d['cmds']
    clouds = d['clouds']

    def interp_x(t):
        prev = None
        for g in gt:
            if g[0] >= t:
                if prev is None:
                    return g[1]
                f = (t - prev[0]) / (g[0] - prev[0]) if g[0] > prev[0] else 0
                return prev[1] + f * (g[1] - prev[1])
            prev = g
        return prev[1] if prev else None

    x_pre = interp_x(t_pre)
    seg = [g for g in gt if g[0] >= t_pre]
    # 首次稳定静止
    t_rest = None
    for k, g in enumerate(seg):
        j = k
        while j + 1 < len(seg) and seg[j + 1][0] - g[0] < 0.25:
            j += 1
        if j > k and abs(seg[j][1] - g[1]) < 0.003:
            t_rest = g[0]
            break
    w = [g for g in seg if t_rest is None or g[0] <= t_rest]
    x_peak = max(g[1] for g in w) if w else None
    x_rest = interp_x(t_rest) if t_rest else None
    # 帧率与间隔
    cl = [c['t'] for c in clouds if t_pre - 0.5 <= c['t'] <= t_rest + 0.1]
    gaps = [round(cl[k] - cl[k - 1], 3) for k in range(1, len(cl))]
    # 感知：首个含墙帧 vs t_post、vs 上一无墙帧
    first_wall = next((c['t'] for c in clouds if c.get('n_near')), None)
    prev_none = [c['t'] for c in clouds if c['t'] < (first_wall or 0) and c.get('n_near') is None]
    prev_none_t = max(prev_none) if prev_none else None
    # 恢复运动：静止后首次 |dx|>0.05
    t_resume = None
    if t_rest:
        for g in seg:
            if g[0] > t_rest + 0.2 and abs(g[1] - x_rest) > 0.05:
                t_resume = g[0]
                break
    out.append({
        'run': i,
        'x_pre': round(x_pre, 4),
        'x_peak': round(x_peak, 4),
        'x_rest': round(x_rest, 4),
        'd_net': round(x_rest - x_pre, 4),
        'd_peak': round(x_peak - x_pre, 4),
        'clear_peak': round(face - (x_peak + 0.25), 4),
        'clear_rest': round(face - (x_rest + 0.25), 4),
        'frames': len(cl),
        'window_s': round(cl[-1] - cl[0], 2) if cl else None,
        'max_gap': max(gaps) if gaps else None,
        'n_gap_gt_015': sum(1 for g in gaps if g > 0.15),
        't_perc_pre': round(first_wall - t_pre, 3) if first_wall else None,
        't_perc_post': round(first_wall - t_post, 3) if first_wall else None,
        't_prev_none': round(first_wall - prev_none_t, 3) if (first_wall and prev_none_t) else None,
        't_resume': None if t_resume is None else round(t_resume - t_rest, 2),
        'dx_3s': round(interp_x(t_rest + 3.0) - x_rest, 4) if t_rest else None,
    })

import statistics as st
print(json.dumps({'runs': out}, indent=1))
vals = lambda k: [r[k] for r in out if r[k] is not None]
print('SUMMARY d_net', min(vals('d_net')), max(vals('d_net')))
print('SUMMARY d_peak', min(vals('d_peak')), max(vals('d_peak')))
print('SUMMARY clear_peak', min(vals('clear_peak')), max(vals('clear_peak')))
print('SUMMARY clear_rest', min(vals('clear_rest')), max(vals('clear_rest')))
print('SUMMARY t_perc_post', min(vals('t_perc_post')), max(vals('t_perc_post')))
print('SUMMARY frames', min(vals('frames')), max(vals('frames')), 'gaps>0.15', min(vals('n_gap_gt_015')), max(vals('n_gap_gt_015')))
print('SUMMARY max_gap', min(vals('max_gap')), max(vals('max_gap')))
print('SUMMARY t_prev_none', min(vals('t_prev_none')), max(vals('t_prev_none')))
with open('/work/scan_stop_metrics_v2.json', 'w') as f:
    json.dump({'runs': out}, f, indent=1)
