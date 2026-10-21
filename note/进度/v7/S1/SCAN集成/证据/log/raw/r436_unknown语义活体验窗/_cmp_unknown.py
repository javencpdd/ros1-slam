import io, re, sys
def grid(p):
    t = io.open(p, encoding='utf-8', errors='ignore').read()
    if 'data:' not in t: return None
    v = [int(x) for x in re.findall(r'-?\d+', t.split('data:', 1)[1])][:1600]
    return v if len(v) == 1600 else None
def rep(tag, p):
    g = grid(p)
    if g is None:
        print('%s ⇒ ★未采到 data 面（N3、不判被试）' % tag); return
    unk = [g[r*40+c] for r in range(10,13) for c in range(20,23)]     # 输入未知格
    free = [g[r*40+c] for r in range(10,13) for c in range(10,13)]    # 输入自由格
    leth = [g[r*40+c] for r in range(10,13) for c in range(20,23)]
    import collections
    print('%s distinct=%s 输入未知区→cost=%s 输入自由区→cost=%s 未知区hist=%s 自由区hist=%s' % (
        tag, sorted(set(g)), sorted(set(unk)), sorted(set(free)), dict(collections.Counter(unk)), dict(collections.Counter(free))))
    print('   未知区与自由区是否同值＝%s（同值⇒未知被并入自由处理）' % ('True' if set(unk)==set(free) and len(set(unk))==1 else 'False'))
for tag, p in (('u1 track_unknown_space:false（＝在册生产形）', sys.argv[1]), ('u2 track_unknown_space:true （对照）', sys.argv[2])):
    rep(tag, p)
