import json, sys

def load(path):
    with open(path) as f:
        return json.load(f)

def main(bridge_path, driver_path):
    b = load(bridge_path); t_hold = load(driver_path)['t_hold']
    S = b['samples']
    def at(t):
        lo, hi = 0, len(S) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if S[mid][0] < t: lo = mid + 1
            else: hi = mid
        return S[lo]
    s0 = at(t_hold)
    x0, y0, yaw0 = s0[2], s0[3], s0[4]
    after = [s for s in S if s[0] >= t_hold]
    along = [(s[0], (s[2]-x0)*__import__('math').cos(yaw0) + (s[3]-y0)*__import__('math').sin(yaw0),
              s[5], s[6]) for s in after]
    peak = max(along, key=lambda r: r[1])
    t_peak = peak[0]
    # 0.2 s moving-average speed, threshold 0.02 m/s held 0.3 s (same criterion as free-run test)
    def avg_v(t):
        vs = [s[2] for s in along if t - 0.2 <= s[0] <= t]
        return sum(vs)/len(vs) if vs else None
    stand_start = None; t_confirmed = None
    for t, _, _, _ in along:
        if t - t_hold < 0.2: continue
        va = avg_v(t)
        if va is not None and abs(va) < 0.02:
            if stand_start is None: stand_start = t
            elif t - stand_start >= 0.3:
                t_confirmed = t; break
        else:
            stand_start = None
    zero_events = [e for e in b['cmd_events'] if e[0] >= t_hold and abs(e[1]) < 1e-9]
    first_zero = zero_events[0][0] if zero_events else None
    out = {
        'seed': b['seed'],
        't_hold': t_hold,
        'v_at_hold_mps': round(s0[5], 3),
        'cmd_zero_latency_ms': round((first_zero - t_hold)*1000, 1) if first_zero else None,
        'stop_distance_m': round(peak[1], 4),
        'stop_time_s': round(t_peak - t_hold, 3),
        'stop_time_confirmed_s': round(t_confirmed - t_hold, 3) if t_confirmed else None,
        'final_along_m': round(along[-1][1], 4),
        'zmin': round(min(s[6] for s in after), 3),
        'samples': len(S),
    }
    print(json.dumps(out, ensure_ascii=False))

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
