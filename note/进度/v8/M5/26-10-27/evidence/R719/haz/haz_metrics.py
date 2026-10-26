import json, sys

fr = json.load(open(sys.argv[1]))
det = json.load(open(sys.argv[2]))
F = fr['frames']
THR = det['thr']
k = next(i for i, f in enumerate(F) if f['range_x'] < THR)
prev = F[k - 1] if k > 0 else F[k]
# 线性插值：range_x 穿越 THR 的时刻
r0, r1 = prev['range_x'], F[k]['range_x']
t_cross = prev['t_cross'] + (r0 - THR) / (r0 - r1) * (F[k]['t_cross'] - prev['t_cross'])
hold = next(e['t_hold'] for e in det['events'] if e.get('fired'))
t_pub = F[k]['t_publish']
out = {
    'thr_m': THR,
    'first_frame_under_thr': k,
    'range_at_frame': round(F[k]['range_x'], 4),
    't_cross_to_hold_ms': round((hold - t_cross) * 1000, 1),
    't_frame_publish_to_hold_ms': round((hold - t_pub) * 1000, 1),
    't_capture_to_hold_ms': round((hold - F[k]['t_capture']) * 1000, 1),
    'frame_period_ms': round((F[-1]['t_publish'] - F[0]['t_publish']) / max(1, len(F) - 1) * 1000, 1),
    'render_ms_mean': fr['render_ms_mean'],
    'n_frames': len(F),
}
print(json.dumps(out, ensure_ascii=False))
