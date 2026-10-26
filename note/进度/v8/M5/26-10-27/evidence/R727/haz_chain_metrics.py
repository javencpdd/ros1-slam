import json, sys

b = json.load(open(sys.argv[1]))
d = json.load(open(sys.argv[2]))
THR = d.get('thr', 1.0)
S = b['samples']
if 't_hold' not in d:
    print(json.dumps({'ok': False, 'reason': 'no_hold', 'n_frames': len(b.get('frames', []))}))
    sys.exit(0)
t_hold = d['t_hold']
# 真值穿越：range = wall_front - (x + cam_x)
WALL = b['wall_front']; CAM = 0.252
cross = None
for i in range(1, len(S)):
    r0 = WALL - (S[i-1][1] + CAM); r1 = WALL - (S[i][1] + CAM)
    if r0 > THR >= r1:
        cross = S[i-1][0] + (r0 - THR) / (r0 - r1) * (S[i][0] - S[i-1][0])
        x_cross = S[i-1][1] + (r0 - THR) / (r0 - r1) * (S[i][1] - S[i-1][1])
        break
def at(t, idx):
    lo, hi = 0, len(S) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if S[mid][0] < t: lo = mid + 1
        else: hi = mid
    return S[min(lo, len(S)-1)]
s_hold = at(t_hold, 1); x_hold = s_hold[1]
after = [s for s in S if s[0] >= t_hold]
zero = next((s for s in after if abs(s[4]) < 1e-9), None)
peak = max(after, key=lambda s: s[1])
frames = b.get('frames', [])
first_hit = next((f for f in frames if f['gt_range'] < THR), None)
# 确认静止：base_x 差分速度（0.2 s 窗）|v|<0.02，持续 0.3 s
def v_est(t):
    lo, hi = 0, len(S) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if S[mid][0] < t: lo = mid + 1
        else: hi = mid
    i = min(lo, len(S) - 1)
    j = 0
    while j < len(S) - 1 and S[j][0] < t - 0.2: j += 1
    if S[i][0] == S[j][0]: return None
    return (S[i][1] - S[j][1]) / (S[i][0] - S[j][0])
conf = None; stand = None
for s in after:
    if s[0] - t_hold < 0.2: continue
    v = v_est(s[0])
    if v is not None and abs(v) < 0.02:
        if stand is None: stand = s[0]
        elif s[0] - stand >= 0.3:
            conf = s; break
    else:
        stand = None
out = {
    'ok': True, 'seed': b['seed'], 't_hold': t_hold, 't_cross': cross,
    't_perception_s': round(t_hold - cross, 3) if cross is not None else None,
    't_ctrl_s': round(zero[0] - t_hold, 3) if zero else None,
    't_brake_to_peak_s': round(peak[0] - zero[0], 3) if zero else None,
    't_total_to_peak_s': round(peak[0] - cross, 3) if cross is not None else None,
    'x_cross': round(x_cross, 4) if cross is not None else None,
    'x_hold': round(x_hold, 4), 'x_peak': round(peak[1], 4),
    'stop_distance_from_hold_m': round(peak[1] - x_hold, 4),
    'stop_distance_from_cross_m': round(peak[1] - x_cross, 4) if cross is not None else None,
    't_confirmed_s': None, 'x_confirmed': None,
    'zmin_after': round(min(s[2] for s in after), 3),
    'fell': min(s[2] for s in after) < 0.18,
    'n_frames': len(frames), 'n_samples': len(S),
    'first_frame_under_thr': first_hit,
    'hold_stamp': d.get('hold_stamp'), 'hold_dmin': d.get('hold_dmin'),
    'render_ms_mean': round(sum(f['render_ms'] for f in frames) / len(frames), 1) if frames else None,
    'detector_frames': len(d.get('frames', [])),
}
if conf is not None:
    out['t_confirmed_s'] = round(conf[0] - cross, 3) if cross is not None else round(conf[0] - t_hold, 3)
    out['x_confirmed'] = round(conf[1], 4)
    out['confirmed_from_cross_m'] = round(conf[1] - x_cross, 4) if cross is not None else None
    out['confirmed_from_hold_m'] = round(conf[1] - x_hold, 4)
    out['t_confirmed_from_hold_s'] = round(conf[0] - t_hold, 3)
print(json.dumps(out, ensure_ascii=False))
