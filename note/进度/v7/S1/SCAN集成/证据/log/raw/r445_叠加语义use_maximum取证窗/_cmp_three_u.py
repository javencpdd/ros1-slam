#!/usr/bin/env python3
# R445 三段叠加语义对比（★新建在本留底目录内 ⇒ 不进 scripts/、不入任何面★）
# 用法：_cmp_three_u.py cost_u1.txt cost_u2.txt cost_u3.txt
import io, re, sys, collections
def grid(p):
    t = io.open(p, encoding='utf-8', errors='ignore').read()
    if 'data:' not in t:
        return None
    v = [int(x) for x in re.findall(r'-?\d+', t.split('data:', 1)[1])][:1600]
    return v if len(v) == 1600 else None
nm = ['u1 单实例（三态＝基准）', 'u2 双实例 use_maximum=★true★', 'u3 双实例 use_maximum=★false★']
gs = []
for p in sys.argv[1:4]:
    g = grid(p)
    if g is None:
        print('★该面未采到 data（N3、不判被试）＝%s ⇒ 本窗不产出结论' % p); sys.exit(1)
    gs.append(g)
# 第一枚那张三态图的 9 个致命格＝rows10-12×cols20-22（发布器自报逐字）
LETHAL = [(r, c) for r in range(10, 13) for c in range(20, 23)]
for n, g in zip(nm, gs):
    print('%-26s distinct=%s 非零=%4d hist=%s' % (n, sorted(set(g)), sum(1 for x in g if x), dict(collections.Counter(g))))
for i, n in enumerate(nm, 1):
    vals = [gs[i - 1][r * 40 + c] for r, c in LETHAL]
    print('%-26s ★那 9 枚致命格现值＝%s（保位枚数=%d／9）' % (n, vals, sum(1 for v in vals if v >= 100)))
d21 = sum(1 for j in range(1600) if gs[1][j] != gs[0][j])
d31 = sum(1 for j in range(1600) if gs[2][j] != gs[0][j])
print('vs u1 逐格差异＝u2 %d／1600 ‖ u3 %d／1600' % (d21, d31))
print('★判读式（★不预设、★按读数选★）：')
print('  (甲) MAX 只升不降 ⇒ 期望 u2 非零仍=9、那 9 格保位 ⇒ "加一枚全自由层不改致命格"')
print('  (乙) 非 MAX 走覆盖 ⇒ 期望 u3 那 9 格被抹成 0、非零=0 ⇒ ★该层能降格＝设计件 A 档"elevation 不得抹掉致命"这条假设的★直接反例★')
print('  (丙) 若 u2 与 u3 ★逐字同★ ⇒ ★use_maximum 在本链路本镜像★不可观测★（两形无差别 ⇒ 只能说"这条参数在此链路不改变发布面"，★不得★外推为"层不能降格"）')
