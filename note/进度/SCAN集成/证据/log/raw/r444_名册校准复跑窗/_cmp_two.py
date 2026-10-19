#!/usr/bin/env python3
# R443 两态 cost 面对比（★新建在本留底目录内 ⇒ 不进 scripts/、不入任何面★）
# 用法：_cmp_two.py cost_c1.txt cost_c2.txt   （c1＝first_map_only:false‖c2＝true，输入与其余参数面全同）
import io, re, sys, collections
def grid(p):
    t = io.open(p, encoding='utf-8', errors='ignore').read()
    if 'data:' not in t:
        return None
    v = [int(x) for x in re.findall(r'-?\d+', t.split('data:', 1)[1])][:1600]
    return v if len(v) == 1600 else None
a, b = grid(sys.argv[1]), grid(sys.argv[2])
if a is None or b is None:
    print('★有一面未采到 data（N3、不判被试）a=%s b=%s' % (a is not None, b is not None)); sys.exit(1)
for nm, g in (('c1 first_map_only=false', a), ('c2 first_map_only=true ', b)):
    print('%-26s distinct=%s 非零=%d hist=%s' % (nm, sorted(set(g)), sum(1 for x in g if x), dict(collections.Counter(g))))
d = [(j // 40, j % 40, a[j], b[j]) for j in range(1600) if a[j] != b[j]]
print('逐格差异＝%d／1600 ⇒ 两态同＝%s（★自校验项＝只差 first_map_only 时应逐格等；不等即本窗有别的变量、名册两读都不可信★）'
      % (len(d), 'True' if not d else 'False'))
for r, c, x, y in d[:8]:
    print('   (r%d,c%d) %d→%d' % (r, c, x, y))
