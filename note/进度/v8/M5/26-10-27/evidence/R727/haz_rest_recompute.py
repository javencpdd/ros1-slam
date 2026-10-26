import json, glob, os, sys

THR = 1.0; WALL = 3.1; CAM = 0.252
rows = []
for bf in sorted(glob.glob('/work/haz_chain_*.json')):
    tag = bf.split('haz_chain_')[1].replace('.json', '')
    if tag.startswith('det_') or tag.startswith('result_'):
        continue
    df = '/work/haz_chain_det_%s.json' % tag
    if not os.path.exists(df):
        continue
    b = json.load(open(bf)); d = json.load(open(df))
    if 't_hold' not in d:
        continue
    S = b['samples']
    xs = [s[1] for s in S]; ts = [s[0] for s in S]
    # crossing of GT range 1.0
    cross = x_cross = None
    for i in range(1, len(S)):
        r0 = WALL - (xs[i-1] + CAM); r1 = WALL - (xs[i] + CAM)
        if r0 > THR >= r1:
            f = (r0 - THR) / (r0 - r1)
            cross = ts[i-1] + f * (ts[i] - ts[i-1])
            x_cross = xs[i-1] + f * (xs[i] - xs[i-1]); break
    t_hold = d['t_hold']
    # base_x difference velocity over 0.2 s aft window
    def at(t):
        lo, hi = 0, len(S)-1
        while lo < hi:
            mid = (lo+hi)//2
            if ts[mid] < t: lo = mid+1
            else: hi = mid
        return min(lo, len(S)-1)
    def v_at(t):
        i = at(t); j = at(t-0.2)
        if i == j: return None
        return (xs[i]-xs[j]) / (ts[i]-ts[j])
    i0 = at(t_hold)
    v_hold = v_at(t_hold)
    rest = None
    for i in range(i0, len(S)):
        t = ts[i]
        if t - t_hold < 0.2: continue
        v = v_at(t)
        if v is None: continue
        if abs(v) < 0.02:
            j = at(t-0.3)
            if all(abs(v_at(ts[k])) is not None and abs(v_at(ts[k])) < 0.02 for k in range(j, i+1, 10)):
                rest = i; break
        # no else reset: contiguous window checked above
    peak = max(range(i0, len(S)), key=lambda k: xs[k])
    row = {'tag': tag, 't_hold': t_hold, 't_cross': cross, 'x_cross': x_cross,
           'v_hold': round(v_hold, 3) if v_hold is not None else None,
           't_peak': ts[peak], 'x_peak': xs[peak],
           't_rest': ts[rest] if rest is not None else None,
           'x_rest': xs[rest] if rest is not None else None,
           'zmin': round(min(s[2] for s in S[i0:]), 3)}
    if cross is not None:
        row['t_perc_from_cross'] = round(t_hold - cross, 3)
        row['t_peak_from_cross'] = round(ts[peak] - cross, 3) if row['t_peak'] else None
        if rest is not None:
            row['t_rest_from_cross'] = round(ts[rest] - cross, 3)
    row['t_rest_from_hold'] = round(ts[rest] - t_hold, 3) if rest is not None else None
    row['d_rest_from_hold'] = round(xs[rest] - xs[i0], 4) if rest is not None else None
    row['t_peak_from_hold'] = round(ts[peak] - t_hold, 3)
    row['d_peak_from_hold'] = round(xs[peak] - xs[i0], 4)
    if rest is not None:
        row['d_rest_from_hold'] = round(xs[rest] - xs[i0], 4)
        row['d_rest_from_cross'] = round(xs[rest] - x_cross, 4)
    row['d_peak_from_hold'] = round(xs[peak] - xs[i0], 4)
    row['d_peak_from_cross'] = round(xs[peak] - x_cross, 4)
    rows.append(row)
rows.sort(key=lambda r: r['tag'])
json.dump(rows, open('/work/haz_rest_summary.json', 'w'), ensure_ascii=False, indent=1)
for r in rows:
    print(r['tag'], 'vhold', r['v_hold'], 't_perc', r.get('t_perc_from_cross'),
          't_peak', r.get('t_peak_from_cross'), 't_rest', r.get('t_rest_from_cross'),
          'd_peak', r.get('d_peak_from_cross'), 'd_rest', r.get('d_rest_from_cross'),
          'zmin', r['zmin'])
