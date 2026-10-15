import sys
tbl, newhdr, out = sys.argv[1:4]
lines = open(tbl, encoding='utf-8').read().split('\n')
if lines and lines[-1] == '':
    lines.pop()
assert len(lines) == 26, 'unexpected table length %d' % len(lines)
assert lines[9].startswith('# \u2605R333'), 'line 10 is not the R333 header line'
hdr = open(newhdr, encoding='utf-8').read().rstrip('\n')
assert hdr.startswith('# \u2605R333'), 'replacement header malformed'
lines[9] = hdr
open(out, 'w', encoding='utf-8').write('\n'.join(lines) + '\n')
print('OK 换版行已更正 行数=%d' % len(lines))
