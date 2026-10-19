import io, re, sys, collections
def grid(p):
    t = io.open(p, encoding='utf-8', errors='ignore').read()
    if 'data:' not in t: return None
    v = [int(x) for x in re.findall(r'-?\d+', t.split('data:', 1)[1])][:1600]
    return v if len(v) == 1600 else None
a, b = grid(sys.argv[1]), grid(sys.argv[2])
if a is None or b is None:
    print('★未采到 data 面（N3、不判被试）a=%s b=%s' % (a is not None, b is not None)); sys.exit(1)
diff = [(i // 40, i % 40, a[i], b[i]) for i in range(1600) if a[i] != b[i]]
print('v1 单实例 distinct=%s 非零=%d hist=%s' % (sorted(set(a)), sum(1 for x in a if x), dict(collections.Counter(a))))
print('v2 双实例（第二实例＝全未知图）distinct=%s 非零=%d hist=%s' % (sorted(set(b)), sum(1 for x in b if x), dict(collections.Counter(b))))
print('逐格差异＝%d／1600；升格数＝%d 降格数＝%d' % (len(diff), sum(1 for _,_,x,y in diff if y > x), sum(1 for _,_,x,y in diff if y < x)))
for r, c, x, y in diff[:12]: print('   (r%d,c%d) %d→%d' % (r, c, x, y))
print('★零行为变化判定＝', 'True' if not diff else 'False')
