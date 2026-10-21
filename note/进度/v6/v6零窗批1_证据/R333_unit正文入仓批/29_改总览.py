import sys

ov, blk, tail, markf = sys.argv[1:5]
mark = open(markf, encoding='utf-8').read().rstrip('\n')
new = open(blk, encoding='utf-8').read().rstrip('\n')
lines = open(ov, encoding='utf-8').read().split('\n')
if lines and lines[-1] == '':
    lines.pop()
n = len(lines)
assert lines[1].startswith('> 日期：2026-10-04'), 'line2 is not the rolling header'
cnt = lines[1].count(mark)
assert cnt == 1, 'anchor count=%d (expect 1)' % cnt
lines[1] = lines[1].replace(mark, new + mark, 1)
lines.append(open(tail, encoding='utf-8').read().rstrip('\n'))
open(ov, 'w', encoding='utf-8').write('\n'.join(lines) + '\n')
print('OK 总览 %d -> %d 行（档头行内替换 1 处＋件尾追加 1 行）' % (n, len(lines)))
