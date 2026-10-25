import hashlib
import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
BASE = json.loads((HERE / 'registration-baseline.json').read_text())


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def queue_state(text):
    rows = [line for line in text.splitlines() if re.match(r'^\| V8-(?:\d\d|X\d) \|', line)]
    assert len(rows) == 19
    assert all(len(line.strip('|').split('|')) == 9 for line in rows)
    return rows


def check():
    assert len(BASE) == 49
    exact = 0
    for name, record in BASE.items():
        raw = (ROOT / name).read_bytes()
        if name in ('note/04_配置改动diff.md', 'note/05_复验作业单.md'):
            assert sha(raw[:record['bytes']]) == record['sha256'], name
            assert b'## R626' in raw[record['bytes']:] if name.endswith('diff.md') else b'R626' in raw[record['bytes']:]
        elif name == 'note/进度/00_总览.md':
            old = record['text'].splitlines(keepends=True)
            new = raw.decode().splitlines(keepends=True)
            assert new[0] == old[0], name
            assert new[2:len(old)] == old[2:], name
            assert 'R626' in new[1] and any('R626' in line for line in new[len(old):]), name
        elif name == 'note/计划书v8/23_v8任务队列.md':
            old_rows = queue_state(record['text'])
            new_rows = queue_state(raw.decode())
            changes = [(a.split('|')[1].strip(), col) for a, b in zip(old_rows, new_rows)
                       for col, (x, y) in enumerate(zip(a.split('|'), b.split('|'))) if x != y]
            assert changes == [('V8-01', 7)], changes
            old_row = next(line for line in old_rows if line.startswith('| V8-01 |'))
            new_row = next(line for line in new_rows if line.startswith('| V8-01 |'))
            restored = raw.decode().replace(new_row, old_row, 1)
            assert restored.startswith(record['text']), name
            assert 'R626 接续登记'.encode() in raw[record['bytes']:], name
        elif name == 'note/进度/v8/README.md':
            old = record['text'].splitlines(keepends=True)
            new = raw.decode().splitlines(keepends=True)
            new[2] = old[2]
            new = [line for line in new if not line.startswith('| [R626]')]
            assert ''.join(new) == record['text'], name
        else:
            assert len(raw) == record['bytes'] and sha(raw) == record['sha256'], name
            exact += 1
    print(f'PASS R626 baseline: {exact} exact files + 04/05 prefixes + overview header/tail + queue status cell + entry row = {len(BASE)}')


def seal():
    target = HERE.parent / 'packages_R625/sha256.txt'
    rows = [line.split('  ', 1) for line in target.read_text().splitlines()]
    assert len(rows) == 102
    for digest, name in rows:
        assert sha((target.parent / name).read_bytes()) == digest, name
    names = {name for _, name in rows}
    actual = {str(p.relative_to(target.parent)) for p in target.parent.rglob('*') if p.is_file() and p.name != 'sha256.txt'}
    assert names == actual | {'../R625.md'}, 'R625 evidence set changed'
    print('PASS R625 sealed evidence: 102 hashes unchanged, no added or removed file')


def links():
    report = HERE.parent / 'R626.md'
    found = re.findall(r'\]\(([^)]+)\)', report.read_text())
    assert found
    from urllib.parse import unquote
    for link in found:
        path, _, anchor = unquote(link).partition('#')
        target = (report.parent / path).resolve()
        if target.name == 'sha256.txt' and not target.is_file():
            assert target.parent == HERE, link
            continue
        assert target.is_relative_to(ROOT) and target.is_file(), link
        if anchor:
            assert re.fullmatch(r'L\d+', anchor), link
            assert 1 <= int(anchor[1:]) <= len(target.read_bytes().splitlines()), link
    print(f'PASS R626 report local links and line ranges: {len(found)}')


if __name__ == '__main__':
    check()
    seal()
    links()
