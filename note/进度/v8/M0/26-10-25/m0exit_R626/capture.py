import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
PRIOR = HERE.parent / 'packages_R625/registration-baseline.json'
TEXTED = ('note/进度/00_总览.md', 'note/计划书v8/23_v8任务队列.md', 'note/进度/v8/README.md')


def main():
    names = list(json.loads(PRIOR.read_text()))
    assert len(names) == 49
    result = {}
    for name in names:
        raw = (ROOT / name).read_bytes()
        record = {'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest()}
        if name in TEXTED:
            record['text'] = raw.decode()
        result[name] = record
    target = HERE / 'registration-baseline.json'
    with target.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=1)
        stream.write('\n')
    print(f'CAPTURED {len(result)} files at R626 start; no file written outside this evidence directory')


if __name__ == '__main__':
    main()
