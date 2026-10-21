#!/usr/bin/env python3
# R442 三面 cost 面对比（★新建在本留底目录内 ⇒ 不进 scripts/、不入任何面★）
# 用法：_cmp_three.py cost_w1.txt cost_w2.txt cost_w3.txt
import io, re, sys, collections
def grid(p):
    t = io.open(p, encoding='utf-8', errors='ignore').read()
    if 'data:' not in t:
        return None
    v = [int(x) for x in re.findall(r'-?\d+', t.split('data:', 1)[1])][:1600]
    return v if len(v) == 1600 else None
names = ['w1 单实例（三态图）', 'w2 双实例·第二枚＝★全未知★', 'w3 双实例·第二枚＝★全致命 100＝阳性对照★']
gs = []
for p in sys.argv[1:4]:
    g = grid(p)
    gs.append(g)
    if g is None:
        print('★该面未采到 data（N3、不判被试）＝%s' % p); sys.exit(1)
for nm, g in zip(names, gs):
    nz = sum(1 for x in g if x)
    print('%-34s distinct=%s 非零=%4d hist=%s' % (nm, sorted(set(g)), nz, dict(collections.Counter(g))))
base = gs[0]
for i in (1, 2):
    d = [(j // 40, j % 40, base[j], gs[i][j]) for j in range(1600) if base[j] != gs[i][j]]
    print('vs w1 ⇒ 逐格差异＝%d／1600；升格 %d 降格 %d；差异样本 %s'
          % (len(d), sum(1 for _, _, x, y in d if y > x), sum(1 for _, _, x, y in d if y < x), d[:6]))
same23 = sum(1 for j in range(1600) if gs[1][j] != gs[2][j])
print('★决定性一行★＝w2 与 w3 ★同为逐格等★？%s（w2vs w3 差异=%d／1600）' % ('True' if same23 == 0 else 'False', same23))
print('★判读★＝若 w3 非零数★远大于★ w1/w2（≈1600）⇒ 第二枚真在合成 ⇒ R441 的"零变化"落在解释(甲)＝全未知被当自由；'
      '若 w3 与 w1/w2 ★逐格等★ ⇒ 第二枚空转 ⇒ R441 结论★降为弱形★（只证"多挂一枚实例不改发布面"）')
