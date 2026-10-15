import sys, re
p = sys.argv[1]
t = open(p, encoding='utf-8', errors='ignore').read()
if 'data:' not in t:
    print("N3|未采到 data 面"); sys.exit(1)
vals = [int(x) for x in re.findall(r'-?\d+', t.split('data:', 1)[1])][:1600]
if len(vals) != 1600:
    print("N3|格数=%d（须 1600）" % len(vals)); sys.exit(1)
g = lambda r, c: vals[r * 40 + c]
print("data_行数=%d 格数=%d 判别区=%s A独占50点=%s B独占70区=%s 未知区=%s 第二帧新置区=%s distinct=%s 254_格数=%d"
      % (t.count('data:'), len(vals), [g(10, c) for c in range(10, 15)], g(20, 20),
         [g(12, c) for c in range(12, 15)], [g(30, c) for c in range(0, 5)],
         [g(25, c) for c in range(25, 30)], sorted(set(vals)), vals.count(254)))
