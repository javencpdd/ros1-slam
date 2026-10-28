#!/usr/bin/env python3
"""R801 汇总：读 v11b 结果，算 PASS/CP、墙钟/仿真钟/RTF 与时限变体。"""
import glob
import json
import sys
from math import comb


def cp_lower(k, n, alpha=0.05):
    lo, hi = 0.0, 1.0
    for _ in range(200):
        p = (lo + hi) / 2
        tail = sum(comb(n, i) * p ** i * (1 - p) ** (n - i) for i in range(k, n + 1))
        if tail < alpha:
            lo = p
        else:
            hi = p
    return (lo + hi) / 2


def main(out_glob):
    rows = [json.load(open(f)) for f in sorted(glob.glob(out_glob))]
    n = len(rows)
    ok = sum(1 for d in rows if d['verdict'] == 'PASS')
    print('n=%d pass=%d' % (n, ok))
    print('CP(goal+clearance) = %.6f' % cp_lower(ok, n))
    for key in ('wall_duration_s', 'sim_duration_s', 'rtf'):
        vals = [d.get(key) for d in rows if d.get(key) is not None]
        if vals:
            print('%-16s min=%.3f max=%.3f' % (key, min(vals), max(vals)))
    for clock in ('wall_duration_s', 'sim_duration_s'):
        vals = [d.get(clock) for d in rows]
        if any(v is None for v in vals):
            print('%s: missing values, skip threshold variants' % clock)
            continue
        for limit in (30.0, 60.0):
            k = sum(1 for v in vals if v <= limit)
            print('  %s <= %4.0fs: %d/%d  CP=%.4f' % (clock, limit, k, n, cp_lower(k, n)))
    misses = [d for d in rows if d['verdict'] != 'PASS']
    for d in misses:
        print('  FAIL', d['run_id'], d.get('fail_reason'), d.get('dist_to_goal_m'),
              d.get('obstacle_min_clearance_m'))


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '/home/jack/temp/v11b_out/*.json')
