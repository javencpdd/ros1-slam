import sys

table, newrows, hdrtxt, dropfile, out = sys.argv[1:6]
lines = open(table, encoding='utf-8').read().split('\n')
if lines and lines[-1] == '':
    lines.pop()
drop = open(dropfile, encoding='utf-8').read()
hdr = open(hdrtxt, encoding='utf-8').read().rstrip('\n')
rows = [r.rstrip('\n') for r in open(newrows, encoding='utf-8') if r.strip()]

assert len(lines) == 25, 'expect 25 lines, got %d' % len(lines)
assert lines[7].startswith('#   T='), 'line8 is not the check-expression'
assert lines[7].count(drop) == 1, 'drop-substring count != 1'
lines[7] = lines[7].replace(drop, '')
assert drop not in lines[7], 'drop-substring still present'

sysd = [i for i, l in enumerate(lines) if l.startswith('tooling/systemd/')]
assert sysd == list(range(18, 25)), 'systemd row indexes changed: %r' % sysd
assert len(rows) == 7, 'new rows count != 7'
for r in rows:
    assert len(r.split('\t')) == 4, 'row lacks 4 tab fields'
lines[18:25] = rows
lines.insert(9, hdr)
open(out, 'w', encoding='utf-8').write('\n'.join(lines) + '\n')
print('OK 总行数=%d 数据行=%d systemd行=%d' % (
    len(lines), len([l for l in lines if not l.startswith('#')]),
    len([l for l in lines if l.startswith('tooling/systemd/')])))
