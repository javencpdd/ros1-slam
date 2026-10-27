"""窄走廊全链停车三次重复的统计算子。"""
import json
import sys

RUNS = [1, 2, 3, 4, 5, 6, 7, 8, 9]


def load(path):
    with open(path) as f:
        return json.load(f)


def first_zero(series, t0):
    for t, v in series:
        if t >= t0 and abs(v) < 0.01:
            return t
    return None


def rest_after(gt, t0):
    """返回（首次稳定静止时刻, x_rest）。稳定＝之后 0.25 s 内 |dx|<0.003。"""
    for i, g in enumerate(gt):
        if g[0] < t0:
            continue
        j = i
        while j + 1 < len(gt) and gt[j + 1][0] - g[0] < 0.25:
            j += 1
        if j > i and abs(gt[j][1] - g[1]) < 0.003 and abs(gt[j][2] - g[2]) < 0.003:
            return g[0], g[1]
    return None, None


out = []
for i in RUNS:
    ev = load('/work/scan_stop_s%d.event.json' % i)
    d = load('/work/scan_stop_s%d.json' % i)
    t0 = ev['t_pre']
    face = ev['wall']['x'] - ev['wall']['hx']
    x_spawn = ev['odom_at_spawn'][1]
    cl = [c for c in d['clouds'] if c.get('n_near')]
    t_sensor = cl[0]['t'] if cl else None
    t_recv = cl[0].get('t_recv') if cl else None
    planner = [(c[0], c[1]) for c in d.get('planner_cmds', [])]
    t_pzero = first_zero(planner, t_sensor if t_sensor else t0)
    t_mzero = first_zero([(c[0], c[1]) for c in d['cmds']], t_sensor if t_sensor else t0)
    gt = [g for g in d['gt'] if g[0] >= t0 - 1.0]
    t_rest, x_rest = rest_after(gt, (t_mzero or t0))
    # 之后是否恢复运动
    t_resume = None
    if t_rest is not None:
        for g in gt:
            if g[0] > t_rest + 0.5 and abs(g[1] - x_rest) > 0.05:
                t_resume = g[0]
                break
    row = {
        'run': i,
        't_pre': t0,
        'wall_face_x': round(face, 3),
        'x_spawn': round(x_spawn, 3),
        'gap_spawn': round(face - (x_spawn + 0.25), 3),
        't_sensor': t_sensor,
        't_perc': None if t_sensor is None else round(t_sensor - t0, 3),
        't_recv': t_recv,
        't_perc_recv': None if t_recv is None else round(t_recv - t0, 3),
        'dmin_first': cl[0]['dmin'] if cl else None,
        'n_first': cl[0]['n_near'] if cl else None,
        't_plan_zero': t_pzero,
        't_plan': None if (t_pzero is None or t_sensor is None) else round(t_pzero - t_sensor, 3),
        't_mux_zero': t_mzero,
        't_ctrl': None if (t_mzero is None or t_pzero is None) else round(t_mzero - t_pzero, 3),
        't_upstream': None if t_mzero is None else round(t_mzero - t0, 3),
        't_rest': t_rest,
        'd_rest': None if t_rest is None else round(t_rest - t0, 3),
        'x_rest': None if x_rest is None else round(x_rest, 3),
        'd_total': None if x_rest is None else round(x_rest - x_spawn, 3),
        'clearance_rest': None if x_rest is None else round(face - (x_rest + 0.25), 3),
        't_resume_after_rest': None if t_resume is None else round(t_resume - t_rest, 2),
    }
    out.append(row)

print(json.dumps({'runs': out}, indent=1))
with open('/work/scan_stop_metrics.json', 'w') as f:
    json.dump({'runs': out}, f, indent=1)
