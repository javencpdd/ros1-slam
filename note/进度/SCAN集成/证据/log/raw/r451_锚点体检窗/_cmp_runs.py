#!/usr/bin/env python3
# R451 三跑对照（★新建在本留底目录内 ⇒ 不进 scripts/、不入任何面★）
# 用法：_cmp_runs.py A.txt A2.txt C.txt      （A/A′＝噪声地板；C＝验牙跑）
import io, re, sys, collections
def vec(p):
    t = io.open(p, encoding='utf-8', errors='ignore').read()
    out = []
    for line in t.split('\n'):
        m = re.match(r'^CHECK\|([^|]+)\|(PASS|FAIL)\|?(.*)$', line)
        if m: out.append((m.group(1), m.group(2), m.group(3)))
    return out
def key(v): return [(n, s) for n, s, _ in v]
def nums(v):
    d = {}
    for n, s, txt in v:
        d[n] = re.findall(r'-?\d+(?:\.\d+)?', txt)
    return d
names = ['A（第一遍）', 'A2（第二遍＝同参数面）', 'C（验牙跑：中途 rosnode kill /move_base_flex）']
vs = [vec(p) for p in sys.argv[1:4]]
for nm, v in zip(names, vs):
    c = collections.Counter(s for _, s, _ in v)
    print('%-46s CHECK行数=%2d  PASS=%2d FAIL=%2d  %s' % (nm, len(v), c.get('PASS', 0), c.get('FAIL', 0), dict(c)))
print('--- 逐行名序与判定（三列＝A / A2 / C）---')
if len(vs[0]) != len(vs[1]) or len(vs[0]) != len(vs[2]):
    print('★三跑 CHECK 行数不等 ⇒ C1 名序条件不成立、本窗不产出"可替代锚点"结论★')
for i in range(max(len(x) for x in vs)):
    row = []
    for v in vs:
        row.append('%s:%s' % (v[i][0], v[i][1]) if i < len(v) else '缺行')
    a, b2, c = (vs[0][i] if i < len(vs[0]) else None), (vs[1][i] if i < len(vs[1]) else None), (vs[2][i] if i < len(vs[2]) else None)
    sameAA = '同' if (a and b2 and a[0] == b2[0] and a[1] == b2[1]) else '★不同★'
    print('  %2d) %-34s %-34s %-40s [A vs A2：%s]' % (i + 1, row[0], row[1], row[2], sameAA))
dA, dA2 = nums(vs[0]), nums(vs[1])
drift = [(n, dA.get(n), dA2.get(n)) for n, _ in dA.items() if dA.get(n) != dA2.get(n)]
print('--- ★C6 噪声地板★＝A 与 A2 的数字读数差异条数=%d（差异行如下，★地板不为零处、逐条列出★）' % len(drift))
for n, x, y in drift: print('    %-16s A=%s ‖ A2=%s' % (n, x, y))
nf = sum(1 for n, s in key(vs[2]) if s == 'FAIL') - sum(1 for n, s in key(vs[0]) if s == 'FAIL')
print('--- ★C7 验牙★＝C 相对 A 多出的 FAIL=%d ⇒ %s' % (max(nf, 0), '★这把闸能红（有牙）★' if nf > 0 else '★C 态没有让它变红＝这把闸没有牙 ⇒ 段 B 的"同值"读数一律不采信★'))
print('--- ★C11 挂载活性锁★＝下面三行由窗志另计（Using plugin／Received a／新增层 use_maximum 回读）')
