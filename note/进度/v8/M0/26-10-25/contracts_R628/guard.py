import hashlib
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
PRIOR = HERE.parent / 'contracts_R627/registration-baseline.json'
TEXTED = ('note/进度/00_总览.md', 'note/计划书v8/23_v8任务队列.md', 'note/进度/v8/README.md')


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def capture():
    names = list(json.loads(PRIOR.read_text()))
    assert len(names) == 49
    out = {}
    for name in names:
        raw = (ROOT / name).read_bytes()
        record = {'bytes': len(raw), 'sha256': sha(raw)}
        if name in TEXTED:
            record['text'] = raw.decode()
        out[name] = record
    with (HERE / 'registration-baseline.json').open('x') as stream:
        json.dump(out, stream, ensure_ascii=False, indent=1)
        stream.write('\n')
    print('CAPTURED %s files at R628 start' % len(out))


def row(text, prefix):
    found = [line for line in text.splitlines() if line.startswith(prefix)]
    assert len(found) == 1, prefix
    return found[0]


def check():
    base = json.loads((HERE / 'registration-baseline.json').read_text())
    assert len(base) == 49
    exact = 0
    for name, record in base.items():
        raw = (ROOT / name).read_bytes()
        if name in ('note/04_配置改动diff.md', 'note/05_复验作业单.md'):
            assert sha(raw[:record['bytes']]) == record['sha256'], name
            assert b'R628' in raw[record['bytes']:], name
        elif name == 'note/进度/00_总览.md':
            old = record['text'].splitlines(keepends=True)
            new = raw.decode().splitlines(keepends=True)
            assert new[0] == old[0], name
            assert new[2:len(old)] == old[2:], name
            assert 'R628' in new[1] and any('R628' in line for line in new[len(old):]), name
        elif name == 'note/计划书v8/23_v8任务队列.md':
            old_text, new_text = record['text'], raw.decode()
            old_row, new_row = row(old_text, '| V8-02 |'), row(new_text, '| V8-02 |')
            diff = [i for i, (x, y) in enumerate(zip(old_row.split('|'), new_row.split('|'))) if x != y]
            assert diff == [7], diff
            assert len([l for l in new_text.splitlines() if re.match(r'^\| V8-\S+ \|', l)]) == 19
            assert new_text.replace(new_row, old_row, 1).startswith(old_text), name
            assert 'R628 接续登记' in new_text, name
        elif name == 'note/进度/v8/README.md':
            old = record['text'].splitlines(keepends=True)
            new = raw.decode().splitlines(keepends=True)
            new[2] = old[2]
            new = [line for line in new if not line.startswith('| [R628]')]
            assert ''.join(new) == record['text'], name
        else:
            assert len(raw) == record['bytes'] and sha(raw) == record['sha256'], name
            exact += 1
    print('PASS R628 baseline: %s exact + 04/05 prefixes + overview header/tail + queue V8-02 cell + entry row = %s' % (exact, len(base)))


def frozen():
    for manifest, expected in ((HERE.parent / 'packages_R625/sha256.txt', 102),
                               (HERE.parent / 'm0exit_R626/sha256.txt', 4)):
        rows = [line.split('  ', 1) for line in manifest.read_text().splitlines()]
        assert len(rows) == expected, manifest
        for digest, name in rows:
            assert sha((manifest.parent / name).read_bytes()) == digest, name
    print('PASS R625 102 and R626 4 entries unchanged')
    rows = [line.split('  ', 1) for line in (HERE.parent / 'contracts_R627/sha256.txt').read_text().splitlines()]
    stale = [name for _, name in rows if '计划书v8/contracts/' in name]
    for digest, name in rows:
        if name in stale:
            continue
        assert sha((HERE.parent / 'contracts_R627' / name).read_bytes()) == digest, name
    print('PASS R627 manifest: %s non-contract entries unchanged; %s contract entries superseded by R628 revision (old manifest kept, not rewritten)' % (len(rows) - len(stale), len(stale)))


def links():
    report = HERE.parent / 'R628.md'
    from urllib.parse import unquote
    found = re.findall(r'\]\(([^)]+)\)', report.read_text())
    for link in found:
        path, _, anchor = unquote(link).partition('#')
        target = (report.parent / path).resolve()
        if target.name in ('sha256.txt', 'checks.txt') and not target.is_file():
            assert target.parent == HERE, link
            continue
        assert target.is_relative_to(ROOT) and target.is_file(), link
        if anchor:
            assert re.fullmatch(r'L\d+', anchor), link
            assert 1 <= int(anchor[1:]) <= len(target.read_bytes().splitlines()), link
    print('PASS R628 report links: %s' % len(found))


if __name__ == '__main__':
    if sys.argv[1:] == ['capture']:
        capture()
    else:
        check()
        frozen()
        links()
