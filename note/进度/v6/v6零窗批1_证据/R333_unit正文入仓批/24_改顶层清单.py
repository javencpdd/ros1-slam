import hashlib, os, sys

man, newhdr, out = sys.argv[1:4]
lines = open(man, encoding='utf-8').read().split('\n')
if lines and lines[-1] == '':
    lines.pop()
assert len(lines) == 16, 'unexpected manifest length %d' % len(lines)
hdr = open(newhdr, encoding='utf-8').read().rstrip('\n')
assert hdr.startswith('# '), 'new header line must stay a comment'
entries = [l for l in lines[10:16]]
assert len(entries) == 6 and all('  ' in e for e in entries), 'entry block shape changed'
old_paths = [e.split('  ', 1)[1] for e in entries]
sysd = sorted(os.path.join('systemd', f) for f in os.listdir('tooling/systemd')
              if f.endswith('.service'))
assert len(sysd) == 7, 'systemd unit count != 7: %r' % sysd
base = os.getcwd()


def digest(rel):
    p = os.path.join(base, 'tooling', rel)
    return hashlib.sha256(open(p, 'rb').read()).hexdigest()


fresh = ['%s  %s' % (digest(r), r) for r in old_paths + sysd]
open(out, 'w', encoding='utf-8').write('\n'.join(lines[:10] + [hdr] + fresh) + '\n')
same = [o.split('  ', 1)[0] == n.split('  ', 1)[0] for o, n in zip(entries, fresh[:6])]
print('OK 新清单行数=%d 条目=%d（旧六条中 sha 未变=%d 条、变=%d 条）'
      % (len(lines[:10] + [hdr] + fresh), len(fresh), sum(same), 6 - sum(same)))
