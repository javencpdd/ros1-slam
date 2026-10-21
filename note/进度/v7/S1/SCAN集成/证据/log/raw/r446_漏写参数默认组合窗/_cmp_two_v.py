#!/usr/bin/env python3
# R446 单面判读（★新建在本留底目录内 ⇒ 不进 scripts/、不入任何面★）
# 用法：_cmp_two_v.py cost_v1.txt
# 判据＝第一枚三态图里★发布器自报★的 9 枚致命格（rows10-12×cols20-22）在★合成后★的 cost 面上还是不是 100
import io, re, sys, collections
def grid(p):
    t = io.open(p, encoding='utf-8', errors='ignore').read()
    if 'data:' not in t:
        return None
    v = [int(x) for x in re.findall(r'-?\d+', t.split('data:', 1)[1])][:1600]
    return v if len(v) == 1600 else None
g = grid(sys.argv[1])
if g is None:
    print('★未采到 data 面（N3、不判被试）⇒ 本窗不产出结论'); sys.exit(1)
LETHAL = [(r, c) for r in range(10, 13) for c in range(20, 23)]
vals = [g[r * 40 + c] for r, c in LETHAL]
print('v1（★elevation_layer 漏写两个组合键★）distinct=%s 非零=%4d hist=%s' % (sorted(set(g)), sum(1 for x in g if x), dict(collections.Counter(g))))
print('★那 9 枚致命格现值＝%s（保位枚数=%d／9）' % (vals, sum(1 for v in vals if v >= 100)))
keep = sum(1 for v in vals if v >= 100)
print('★判读★＝保位 9／9 且 非零=9 ⇒ ★漏写时该层走"只升不降"＝与声明层默认 combination_method=1(Maximum) 一致 ⇒ 默认安全★；'
      '保位 0／9 且 非零=0 ⇒ ★漏写即抹致命＝与声明层相反 ⇒ 本镜像一条真实危险、须把 bb 件 K4 升成硬前置★；'
      '其余形状 ⇒ ★两路都没闭合、照实报未决★')
