import sys
D, ov = sys.argv[1], 'note/进度/00_总览.md'
new = open(D + '/07_档头新块_R335.txt', encoding='utf-8').read().rstrip('\n')
tail = open(D + '/08_件尾条目_R335.txt', encoding='utf-8').read().rstrip('\n')
anchor = '**更新时间：2026-10-16 05:1x（R334 批同步：'
lines = open(ov, encoding='utf-8').read().split('\n')
if lines and lines[-1] == '':
    lines.pop()
before = len(lines)
star_before = lines[1].count('****')
assert lines[1].count(anchor) == 1, 'anchor count=%d' % lines[1].count(anchor)
assert new.endswith('**'), 'new block must end with the bold close'
lines[1] = lines[1].replace(anchor, '**更新时间：' + new + '｜原 2026-10-16 05:1x（R334 批同步：', 1)
assert lines[1].count('****') == star_before, 'seam star-count changed: %d -> %d' % (star_before, lines[1].count('****'))
lines.append(tail)
open(ov, 'w', encoding='utf-8').write('\n'.join(lines) + '\n')
print('总览 %d -> %d 行；档头标签枚数=%d；四连星号=%d（应与滚前同值）'
      % (before, len(lines), lines[1].count('**更新时间：'), lines[1].count('****')))
