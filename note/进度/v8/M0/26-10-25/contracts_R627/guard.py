import hashlib
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
PRIOR = HERE.parent / 'm0exit_R626/registration-baseline.json'
TEXTED = ('note/进度/00_总览.md', 'note/计划书v8/23_v8任务队列.md', 'note/进度/v8/README.md')


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def capture():
    names = list(json.loads(PRIOR.read_text()))
    assert len(names) == 49
    result = {}
    for name in names:
        raw = (ROOT / name).read_bytes()
        record = {'bytes': len(raw), 'sha256': sha(raw)}
        if name in TEXTED:
            record['text'] = raw.decode()
        result[name] = record
    with (HERE / 'registration-baseline.json').open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=1)
        stream.write('\n')
    print(f'CAPTURED {len(result)} files at R627 start')


def queue_row(text, prefix):
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
            assert b'R627' in raw[record['bytes']:], name
        elif name == 'note/进度/00_总览.md':
            old = record['text'].splitlines(keepends=True)
            new = raw.decode().splitlines(keepends=True)
            assert new[0] == old[0], name
            assert new[2:len(old)] == old[2:], name
            assert 'R627' in new[1] and any('R627' in line for line in new[len(old):]), name
        elif name == 'note/计划书v8/23_v8任务队列.md':
            old_text, new_text = record['text'], raw.decode()
            old_row, new_row = queue_row(old_text, '| V8-02 |'), queue_row(new_text, '| V8-02 |')
            changes = [(index, x, y) for index, (x, y) in enumerate(zip(old_row.split('|'), new_row.split('|'))) if x != y]
            assert len(changes) == 1 and changes[0][0] == 7, changes
            assert len([l for l in old_text.splitlines() if re.match(r'^\| V8-\S+ \|', l)]) == 19
            assert len([l for l in new_text.splitlines() if re.match(r'^\| V8-\S+ \|', l)]) == 19
            assert new_text.replace(new_row, old_row, 1).startswith(old_text), name
            assert 'R627 接续登记' in new_text, name
        elif name == 'note/进度/v8/README.md':
            old = record['text'].splitlines(keepends=True)
            new = raw.decode().splitlines(keepends=True)
            new[2] = old[2]
            new = [line for line in new if not line.startswith('| [R627]')]
            assert ''.join(new) == record['text'], name
        else:
            assert len(raw) == record['bytes'] and sha(raw) == record['sha256'], name
            exact += 1
    print(f'PASS R627 baseline: {exact} exact + 04/05 prefixes + overview header/tail + queue V8-02 status + entry row = {len(base)}')


def frozen():
    for manifest, expected in ((HERE.parent / 'packages_R625/sha256.txt', 102),
                               (HERE.parent / 'm0exit_R626/sha256.txt', 4)):
        rows = [line.split('  ', 1) for line in manifest.read_text().splitlines()]
        assert len(rows) == expected, manifest
        for digest, name in rows:
            assert sha((manifest.parent / name).read_bytes()) == digest, name
    print('PASS prior batches frozen: R625 102 entries and R626 4 entries unchanged')


def links():
    report = HERE.parent / 'R627.md'
    from urllib.parse import unquote
    found = re.findall(r'\]\(([^)]+)\)', report.read_text())
    assert found
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
    print(f'PASS R627 report links and line ranges: {len(found)}')


if __name__ == '__main__':
    if sys.argv[1:] == ['capture']:
        capture()
    else:
        check()
        frozen()
        links()
