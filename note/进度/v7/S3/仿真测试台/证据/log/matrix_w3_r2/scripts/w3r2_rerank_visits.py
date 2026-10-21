#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""W3-R2 候选重排序（小明 2026-10-07）：按「封死态洪泛规模」升序重排环带候选。
依据：live global_costmap 稳态 dump（logs/w3r2_live_gcostmap.npy，1000x1000@0.1，stvl 已封窄缝）上
候选→start 的 uniform-cost dijkstra 访问格数=该对在最坏 stvl 相位下的规划规模代理；
KG 锚（跨栈 0.05–0.15s 实测）=28180 visits@geo14.6m → ≤~100k≈≤0.5s 带内，400k≈2s=55 域。
排序演进留证：|d−6.0|（run1）→ 舒适间隙（run3）→ 封死态 visits（本轮）——各版实测依据见 logs/。
只改 w3r2_ring_candidates.json 的 ranked 顺序与附注，不改判据与判据文件。"""
import json

C = json.load(open('/tmp/matrix_w3/w3r2_ring_candidates.json'))
R = json.load(open('/tmp/matrix_w3/w3r2_sealed_flood_rank.json'))
assert [s['sector'] for s in C['sectors']] == [s['sector'] for s in R['sectors']]
for sec, rsec in zip(C['sectors'], R['sectors']):
    by_cand = {}
    for r in rsec['rows']:
        by_cand[(r['cand'][0], r['cand'][1])] = r
    rows = []
    for cand in sec['ranked']:
        r = dict(by_cand[(cand[0], cand[1])])
        rows.append(r)
    rows.sort(key=lambda r: (0 if r['visits'] else 1, r['visits'] if r['visits'] else 1 << 30, r['clr']), reverse=False)
    sec['ranked'] = [[r['cand'][0], r['cand'][1], r['d'], r['clr'], r['visits'], r['geo']] for r in rows]
    sec['rank_key'] = 'sealed_dump_visits_asc'
    print('SECTOR%d 前6: %s' % (sec['sector'], ' '.join(
        '(%.2f,%.2f)d=%.1f v=%s' % (r[0], r[1], r[2], 'NA' if r[4] is None else r[4]) for r in sec['ranked'][:6])))
C['rank_note'] = 'W3-R2 重排序: 封死态(live gcostmap dump stvl)洪泛 visits 升序；visits=None(密封态不可达)垫底'
json.dump(C, open('/tmp/matrix_w3/w3r2_ring_candidates.json', 'w'), ensure_ascii=False, indent=1)
print('reranked -> w3r2_ring_candidates.json')
