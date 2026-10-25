import hashlib
import json
import re
from pathlib import Path
from urllib.parse import unquote

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
BASE = json.loads((HERE / 'registration-baseline.json').read_text())


def digest(data):
    return hashlib.sha256(data).hexdigest()


def rows(text):
    result = [line.strip().strip('|').split('|') for line in text.splitlines()
              if re.match(r'^\| V8-(?:\d\d|X\d) \|', line)]
    assert len(result) == 19
    assert all(len(row) == 9 for row in result)
    return [[cell.strip() for cell in row] for row in result]


assert len(BASE) == 40
exact = 0
for name, record in BASE.items():
    data = (ROOT / name).read_bytes()
    if name == 'note/04_配置改动diff.md':
        assert digest(data[:record['bytes']]) == record['sha256'], name
        assert data[record['bytes']:].decode().count('## R624') == 1
        print('PASS 04 historical prefix unchanged')
    elif name == 'note/进度/00_总览.md':
        body = b''.join(data.splitlines(keepends=True)[2:])
        assert digest(body[:record['body_bytes']]) == record['body_sha256'], name
        assert 'R624' in data.splitlines()[1].decode()
        assert body[record['body_bytes']:].decode().count('R624') >= 1
        print('PASS overview historical body unchanged; header and append updated')
    elif name == 'note/计划书v8/23_v8任务队列.md':
        before, after = rows(record['text']), rows(data.decode())
        changes = [(a[0], col + 1) for a, b in zip(before, after)
                   for col in range(9) if a[col] != b[col]]
        assert changes == [('V8-01', 7)], changes
        old_line = next(line for line in record['text'].splitlines()
                        if line.startswith('| V8-01 |'))
        new_line = next(line for line in data.decode().splitlines()
                        if line.startswith('| V8-01 |'))
        restored = data.decode().replace(new_line, old_line, 1)
        assert restored.startswith(record['text'])
        print('PASS queue 19 rows x 9 columns; only V8-01 status changed; history preserved')
    elif name == 'note/进度/v8/README.md':
        text = data.decode()
        assert 'R624／批277' in text.splitlines()[2]
        assert sum(line.startswith('| [R624]') for line in text.splitlines()) == 1
        print('PASS v8 entry shape; historical byte equality not asserted for this edited file')
    else:
        assert len(data) == record['bytes'] and digest(data) == record['sha256'], name
        exact += 1
print(f'PASS protected baseline: {exact} exact files + 2 historical prefixes + queue boundary + entry shape = {len(BASE)} checks')

report = HERE.parent / 'R624.md'
links = re.findall(r'\]\(([^)]+)\)', report.read_text())
assert links
for link in links:
    path, _, anchor = unquote(link).partition('#')
    target = (report.parent / path).resolve()
    assert target.is_relative_to(ROOT) and target.is_file(), link
    if anchor:
        assert re.fullmatch(r'L\d+', anchor), link
        assert 1 <= int(anchor[1:]) <= len(target.read_bytes().splitlines()), link
print(f'PASS report local links and line ranges: {len(links)}')

capture = json.loads((HERE / 'preflight.json').read_text())
assert capture['inputs_unchanged'] is True
assert capture['existing_state_unchanged'] is True
assert capture['remaining'] == []
assert capture['preflight']['rc'] == 5
assert capture['exit_state']['ExitCode'] == 5
print('PASS original capture remains a rejection, not acceptance')
print('RC 0')
