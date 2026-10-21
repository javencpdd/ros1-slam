# R336 在册件改写跑法留底（两件各一）：00_总览 档头行内滚＋件尾追加；23 号治-4 行改写
# 用法＝cd /home/ros1-slam && python3 <本件>（块文本须先在 /tmp/ov_head.txt、/tmp/ov_tail.txt、/tmp/q23_fix.md）
p = 'note/进度/00_总览.md'
lines = open(p, encoding='utf-8').read().split('\n')
old = lines[1]
new_block = open('/tmp/ov_head.txt', encoding='utf-8').read().rstrip('\n')
tail_block = open('/tmp/ov_tail.txt', encoding='utf-8').read().rstrip('\n')
LBL = '**更新时间：'
sep, stars = old.count('｜原'), old.count('****')
assert old.count(LBL) == 1 and sep == 26 and stars == 2
head, rest = old.split(LBL, 1)
assert rest.startswith('2026-10-16 06:0x（R335 批同步')
newline = head + LBL + new_block + '｜原 ' + rest
assert newline.count(LBL) == 1 and newline.count('****') == stars and newline.count('｜原') == sep + 1
assert new_block.endswith('（第四环）') and rest in newline
lines[1] = newline
body = '\n'.join(lines)
assert body.endswith('｜小明施工\n')
open(p, 'w', encoding='utf-8').write(body + '\n' + tail_block + '\n')

q = 'note/计划书v6/23_v6任务队列.md'
ql = open(q, encoding='utf-8').read().split('\n')
row = open('/tmp/q23_fix.md', encoding='utf-8').read().rstrip('\n')
assert ql[24].startswith('| 治-4 |') and ql[23].startswith('| 治-3 |')
assert row.count('|') - 1 == ql[7].count('|') - 1 == 7   # 字段数＝| 枚数-1（首版误写成 -2 而中止）
ql[24] = row
open(q, 'w', encoding='utf-8').write('\n'.join(ql))
