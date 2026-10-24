import copy
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import unittest


CLASSES = {'直接复用', '需适配', '待复核', '可退役'}
LAYERS = {'记录', '实现', '运行', '入口'}
BASE = Path(__file__).resolve().parent
REPORT = BASE.parent / 'R618.md'
ROOT = BASE.parents[5]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def table(text, section, heading):
    lines = text.splitlines()
    sections = [i for i, line in enumerate(lines) if line == section]
    headers = [i for i, line in enumerate(lines) if line.startswith(heading)]
    require(len(sections) == 1 and len(headers) == 1, 'section/header uniqueness')
    begin = sections[0] + 1
    end = next((i for i in range(begin, len(lines)) if lines[i].startswith('## ')), len(lines))
    positions = [i for i in range(begin, end) if lines[i].lstrip().startswith('|')]
    require(bool(positions) and positions[0] == headers[0], 'table section')
    require(positions == list(range(positions[0], positions[-1] + 1)), 'disconnected table')
    cells = [[cell.strip() for cell in lines[i].strip().strip('|').split('|')] for i in positions]
    require(len(cells) >= 2, 'missing separator')
    require(len(cells[1]) == len(cells[0]) and all(re.fullmatch(r':?-{3,}:?', cell) for cell in cells[1]), 'invalid separator')
    if heading == '| 旧 ID |':
        task_positions = {i for i, line in enumerate(lines) if re.match(r'^\s*\|\s*(?:[ABCX]-\d|治-\d)', line)}
        require(task_positions <= set(positions[2:]), 'task row outside migration')
    return cells[2:]


def check(snapshot, text):
    source = ROOT / snapshot['source']
    data = source.read_bytes()
    require(hashlib.sha256(data).hexdigest() == snapshot['source_sha256'], 'source hash')
    archived = subprocess.check_output(['git', 'show', f"{snapshot['head']}:{snapshot['source']}"], cwd=ROOT)
    require(archived == data, 'historical source changed')
    blob = subprocess.check_output(['git', 'rev-parse', f"{snapshot['head']}:{snapshot['source']}"], cwd=ROOT, text=True).strip()
    require(blob == snapshot['blob'], 'source blob')
    original = []
    for number, line in enumerate(data.decode().splitlines(), 1):
        if line.startswith('| ') and line.split('|')[1].strip() != 'ID':
            fields = [cell.strip() for cell in line.strip('|').split('|')]
            require(len(fields) == 9, 'source columns')
            original.append({'id': fields[0], 'line': number, 'raw': line, 'fields': fields})
    require(original == snapshot['rows'], 'snapshot rows differ')
    require(len(original) == snapshot['row_count'] > 0, 'snapshot count')
    targets = set(re.findall(r'^\| (V8-(?:\d+|X\d+)) \|', (ROOT / 'note/计划书v8/23_v8任务队列.md').read_text(), re.M))
    evidence = table(text, '## 2. 证据目录与适用边界', '| 证据 ID |')
    require(bool(evidence), 'empty evidence')
    require(all(len(row) == 4 and all(row) for row in evidence), 'evidence columns')
    evidence_ids = [row[0] for row in evidence]
    require(len(evidence_ids) == len(set(evidence_ids)), 'duplicate evidence')
    require(all(re.fullmatch(r'E\d{2}', key) for key in evidence_ids), 'evidence ID')
    require(all(re.search(r'\[[^\]]+\]\([^)]+\)', row[1]) for row in evidence), 'evidence link absent')
    rows = table(text, '## 4. 旧队列逐行迁移表', '| 旧 ID |')
    require(bool(rows), 'empty migration')
    require(all(len(row) == 8 and all(row) for row in rows), 'migration columns')
    require([row[0] for row in rows] == [row['id'] for row in original], 'row coverage/order')
    for row, old in zip(rows, original):
        require(row[1] == str(old['line']), 'source line')
        require(row[2] in CLASSES, 'classification')
        require(row[3] in LAYERS, 'object layer')
        require(set(row[4].split('、')) <= targets, 'unknown target')
        require(set(row[5].split('、')) <= set(evidence_ids), 'unknown evidence')
        if old['id'].startswith('C-'):
            require(row[4] == 'V8-X2' and row[2] == '待复核', 'hardware boundary')
    links = re.findall(r'\[[^\]]+\]\(([^)]+)\)', text)
    require(bool(links), 'empty links')
    for link in links:
        path, _, fragment = link.partition('#')
        target = (REPORT.parent / path).resolve()
        require(target.is_relative_to(ROOT) and target.is_file(), f'missing/local link: {link}')
        if fragment:
            match = re.fullmatch(r'L(\d+)(?:-L?(\d+))?', fragment)
            require(match is not None, f'unsupported anchor: {link}')
            first = int(match[1])
            last = int(match[2] or first)
            require(1 <= first <= last <= len(target.read_text().splitlines()), f'line range: {link}')
    return len(rows), len(evidence), len(links)


class MigrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.snapshot = json.loads((BASE / 'legacy-rows.json').read_text())
        cls.text = REPORT.read_text()

    def rejects(self, text, message, snapshot=None):
        with self.assertRaisesRegex(ValueError, message):
            check(self.snapshot if snapshot is None else snapshot, text)

    def test_current_inventory(self):
        count, _, _ = check(self.snapshot, self.text)
        self.assertEqual(count, self.snapshot['row_count'])

    def test_missing_duplicate_unknown_rows(self):
        line = next(line for line in self.text.splitlines() if line.startswith('| A-2-1 |'))
        for altered in ['', line + '\n' + line, line.replace('A-2-1', 'A-404', 1)]:
            with self.subTest(altered=altered[:30]):
                self.rejects(self.text.replace(line + '\n', altered + '\n'), 'row coverage/order|empty migration|disconnected table')

    def test_duplicate_table(self):
        block = self.text.split('## 4. 旧队列逐行迁移表', 1)[1].split('## 5.', 1)[0]
        self.rejects(self.text + '\n' + block, 'section/header uniqueness')

    def test_separator_replaced_by_data(self):
        line = next(line for line in self.text.splitlines() if line.startswith('| A-2-1 |'))
        self.rejects(self.text.replace('| --- | --- | --- | --- | --- | --- | --- | --- |', line.replace('A-2-1', 'A-404')), 'invalid separator')

    def test_disconnected_and_outside_rows(self):
        line = next(line for line in self.text.splitlines() if line.startswith('| A-2-1 |'))
        self.rejects(self.text.replace(line, line + '\n'), 'disconnected table')
        self.rejects(self.text + '\n' + line, 'task row outside migration')
        self.rejects(self.text + '\n' + line.replace('A-2-1', 'A-404'), 'task row outside migration')

    def test_missing_header(self):
        self.rejects(self.text.replace('| 旧 ID |', '| 任务 |'), 'section/header uniqueness')

    def test_invalid_class_and_target(self):
        self.rejects(self.text.replace('| 待复核 |', '| 已验收 |', 1), 'classification')
        self.rejects(self.text.replace('| V8-01、V8-13 |', '| V8-99 |', 1), 'unknown target')

    def test_unknown_evidence(self):
        self.rejects(self.text.replace('| E00、E03 |', '| E99 |', 1), 'unknown evidence')

    def test_wrong_source_line(self):
        self.rejects(self.text.replace('| A-2-1 | 10 |', '| A-2-1 | 11 |'), 'source line')

    def test_snapshot_status_tampering(self):
        snapshot = copy.deepcopy(self.snapshot)
        snapshot['rows'][0]['fields'][7] = '已验收'
        self.rejects(self.text, 'snapshot rows differ', snapshot)

    def test_bad_columns(self):
        self.rejects(self.text.replace('| A-2-1 | 10 |', '| A-2-1 |'), 'migration columns')

    def test_missing_file_and_bad_anchor(self):
        self.rejects(self.text + '\n[反控](missing-r618-file.txt)\n', 'missing/local link')
        self.rejects(self.text + '\n[反控](R617.md#L999999)\n', 'line range')

    def test_hardware_not_auto_authorized(self):
        line = next(line for line in self.text.splitlines() if line.startswith('| C-1 |'))
        self.rejects(self.text.replace(line, line.replace('待复核', '直接复用')), 'hardware boundary')


if __name__ == '__main__':
    if sys.argv[1:] == ['--selftest']:
        unittest.main(argv=[sys.argv[0]], verbosity=2)
    elif not sys.argv[1:]:
        counts = check(json.loads((BASE / 'legacy-rows.json').read_text()), REPORT.read_text())
        print('MIGRATION PASS rows=%d evidence=%d local_links=%d' % counts)
        print('Scope: inventory structure and source fidelity only; no runtime acceptance.')
    else:
        raise SystemExit('Usage: verify_migration.py [--selftest]')
