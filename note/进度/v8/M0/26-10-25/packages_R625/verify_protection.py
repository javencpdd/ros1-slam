import hashlib
import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def queue_restore(text, before):
    def rows(value):
        result = [line for line in value.splitlines() if re.match(r'^\| V8-(?:\d\d|X\d) \|', line)]
        assert len(result) == 19
        assert all(len(line.strip('|').split('|')) == 9 for line in result)
        return result
    old, new = rows(before), rows(text)
    changes = [(a.split('|')[1].strip(), col) for a, b in zip(old, new)
               for col, (x, y) in enumerate(zip(a.split('|'), b.split('|'))) if x != y]
    assert changes == [('V8-01', 7)], changes
    return text.replace(new[1], old[1], 1)


def main():
    baseline = json.loads((HERE / 'registration-baseline.json').read_text())
    assert len(baseline) == 49
    for name, record in baseline.items():
        raw = (ROOT / name).read_bytes()
        if name in ('note/04_配置改动diff.md', 'note/05_复验作业单.md'):
            assert sha(raw[:record['bytes']]) == record['sha256'], name
        elif name == 'note/进度/00_总览.md':
            old = record['text'].splitlines(keepends=True)
            new = raw.decode().splitlines(keepends=True)
            assert new[0] == old[0] and new[2:len(old)] == old[2:], name
        elif name == 'note/计划书v8/23_v8任务队列.md':
            assert queue_restore(raw.decode(), record['text']).startswith(record['text']), name
        elif name == 'note/进度/v8/README.md':
            old = record['text'].splitlines(keepends=True)
            new = raw.decode().splitlines(keepends=True)
            new[2] = old[2]
            new = [line for line in new if not line.startswith('| [R625]')]
            assert ''.join(new) == record['text'], name
        else:
            assert len(raw) == record['bytes'] and sha(raw) == record['sha256'], name
    print('PASS registration baseline: 49 files; historical prefixes and queue/entry boundaries preserved')
    implementation = json.loads((HERE / 'implementation-baseline.json').read_text())
    assert len(implementation) == 37
    for name, record in implementation.items():
        raw = (ROOT / name).read_bytes()
        if name.endswith('24_仿真验收与迁移方案.md'):
            text = raw.decode()
            begin = text.index('### 4.0 M0 基础环境出口')
            end = text.index('### 4.1 数值定标与冻结', begin)
            raw = (text[:begin] + text[end:]).encode()
        elif name.endswith('25_v8自动调度作业单.md'):
            text = raw.decode()
            begin = text.index('**R625 新调用式')
            end = text.index('**可复跑判据', begin)
            raw = (text[:begin] + text[end:]).encode()
        elif name.endswith('23_v8任务队列.md'):
            raw = queue_restore(raw.decode(), baseline[name]['text']).encode()[:record['bytes']]
        elif name == 'note/05_复验作业单.md':
            raw = raw[:record['bytes']]
        assert len(raw) == record['bytes'] and sha(raw) == record['sha256'], name
    print('PASS implementation baseline: 37 files; old checkers unchanged, only approved plan additions')


if __name__ == '__main__':
    main()
