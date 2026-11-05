#!/usr/bin/env python3
"""Controls for moving batch audit input away from the release overview.

Every Git repository and writable file belongs to TemporaryDirectory. The
real repository supplies read-only historical objects through Git alternates.
"""
import ast
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/verify/v_ledger_batch_consistency.sh'
BASE = '690aff84e0e02b4af60f8fd45f98d5617be930c4'
OLD = 'note/进度/00_总览.md'
AUDIT = 'note/进度/01_批次.md'
KNOWN = ('batch_known_findings.tsv', 'batch_c5_known_lines.tsv')


def original(path):
    return subprocess.check_output(['git', '-C', str(ROOT), 'show', BASE + ':' + path])


def embedded(text):
    return text.split("<<'PYEOF'\n", 1)[1].rsplit('\nPYEOF', 1)[0]


class LedgerSourceMigrationTests(unittest.TestCase):
    def setUp(self):
        self.owned = tempfile.TemporaryDirectory(prefix='ledger-source-controls-')
        self.addCleanup(self.owned.cleanup)
        self.root = Path(self.owned.name)
        self.env = {k: v for k, v in os.environ.items()
                    if not k.startswith(('GIT_', 'VLB_'))}
        self.env['VLB_ROOT'] = str(self.root)
        self.git('init', '--quiet')
        object_path = subprocess.check_output(
            ['git', '-C', str(ROOT), 'rev-parse', '--git-path', 'objects'], text=True).strip()
        objects = (ROOT / object_path).resolve()
        self.write('.git/objects/info/alternates', str(objects) + '\n')
        self.git('reset', '--soft', BASE)
        self.write(AUDIT, original(OLD))
        self.write(OLD, '# 版本成果总览\n> 最新更新时间：2026-11-05\n## v8\n')
        for name in KNOWN:
            self.write('scripts/verify/' + name,
                       original('scripts/verify/' + name))

    def write(self, name, data):
        p = self.root / name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(data if isinstance(data, bytes) else data.encode())
        return p

    def git(self, *args, cwd=None):
        command = ['git', '-c', 'user.name=ledger-control',
                   '-c', 'user.email=ledger-control@example.invalid',
                   '-c', 'commit.gpgsign=false', '-c', 'core.hooksPath=/dev/null', *args]
        return subprocess.run(command, cwd=cwd or self.root, env=self.env,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              text=True, check=True)

    def scan(self, extra=None, root=None):
        env = dict(self.env)
        if root is not None:
            env['VLB_ROOT'] = str(root)
        env.update(extra or {})
        return subprocess.run(['bash', str(SCRIPT)], cwd=root or self.root,
                              env=env, text=True, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT)

    def test_original_input_retains_verdict_and_initial_header_provenance(self):
        result = self.scan()
        self.assertEqual(0, result.returncode, result.stdout)
        self.assertIn('01_批次.md', result.stdout)
        self.assertIn('初始档头逐字等于迁移基准', result.stdout)
        self.assertIn('声明=R1126／实动档头=R1126', result.stdout)
        self.assertIn('新增 0／存量 58', result.stdout)

    def test_new_release_overview_is_not_a_silent_audit_fallback(self):
        (self.root / AUDIT).unlink()
        self.write(OLD, original(OLD))
        result = self.scan()
        self.assertEqual(9, result.returncode, result.stdout)
        self.assertIn('01_批次.md', result.stdout)

    def test_invalid_audit_shape_rejects_even_when_release_page_exists(self):
        self.write(AUDIT, '# 版本报告，不含批号\n')
        result = self.scan()
        self.assertEqual(9, result.returncode, result.stdout)

    def test_future_git_batch_without_audit_entry_remains_new_failure(self):
        self.git('commit', '--allow-empty', '-m', 'docs(R1127): isolated control')
        result = self.scan()
        self.assertEqual(5, result.returncode, result.stdout)
        self.assertIn('C2 新增=[1127]', result.stdout)

    def test_future_batch_entry_closes_c2_without_freezing_git_history(self):
        self.git('commit', '--allow-empty', '-m', 'docs(R1127): isolated control')
        p = self.root / AUDIT
        p.write_text(p.read_text() + '\n- 2026-11-05：**R1127** 隔离反控真实实验登记\n')
        result = self.scan()
        self.assertEqual(0, result.returncode, result.stdout)
        self.assertIn('C4 滞后 1 批', result.stdout)

    def test_new_bad_c5_row_still_fails(self):
        p = self.root / AUDIT
        p.write_text(p.read_text() + '\n- 2026-11-05 09:00：R1127 非法行首反控\n')
        result = self.scan()
        self.assertEqual(5, result.returncode, result.stdout)
        self.assertIn('新增 1／存量 58', result.stdout)

    def test_changed_uncommitted_header_cannot_claim_old_header_source(self):
        p = self.root / AUDIT
        p.write_text(p.read_text().replace('R1126 批同步', 'R1127 批同步', 1))
        result = self.scan()
        self.assertNotIn('初始档头逐字等于迁移基准', result.stdout)
        self.assertIn('档头已变化，未使用迁移回退', result.stdout)
        self.assertIn('实动档头=R未取到', result.stdout)

    def test_invalid_utf8_header_byte_cannot_claim_migration_identity(self):
        p = self.root / AUDIT
        lines = p.read_bytes().splitlines(keepends=True)
        self.assertIn(b'R1126', lines[1])
        lines[1] = lines[1].replace(b'R1126', b'R1126\xff', 1)
        p.write_bytes(b''.join(lines))
        result = self.scan()
        # Original C1 only reports; this test guards provenance, not a new gate.
        self.assertNotIn('初始档头逐字等于迁移基准', result.stdout)
        self.assertIn('档头已变化，未使用迁移回退', result.stdout)
        self.assertIn('实动档头=R未取到', result.stdout)

    def test_header_line_ending_changes_cannot_claim_migration_identity(self):
        p = self.root / AUDIT
        original_lines = p.read_bytes().splitlines(keepends=True)
        self.assertTrue(original_lines[1].endswith(b'\n'))
        for ending in (b'\r\n', b'\r'):
            with self.subTest(ending=ending):
                lines = list(original_lines)
                lines[1] = lines[1].rstrip(b'\r\n') + ending
                p.write_bytes(b''.join(lines))
                result = self.scan()
                self.assertNotIn('初始档头逐字等于迁移基准', result.stdout)
                self.assertIn('档头已变化，未使用迁移回退', result.stdout)
                self.assertIn('实动档头=R未取到', result.stdout)

    def test_real_new_audit_header_history_takes_precedence(self):
        p = self.root / AUDIT
        p.write_text(p.read_text().replace('R1126 批同步', 'R1127 批同步', 1)
                     + '\n- 2026-11-05：**R1127** 隔离反控新实验\n')
        self.git('add', AUDIT)
        self.git('commit', '-m', 'docs(R1127): audit header control')
        result = self.scan()
        self.assertEqual(0, result.returncode, result.stdout)
        self.assertIn('实时 Git 行历史：' + AUDIT, result.stdout)
        self.assertIn('声明=R1127／实动档头=R1127', result.stdout)
        self.assertNotIn('初始档头逐字等于迁移基准', result.stdout)

    def test_missing_fixed_migration_history_is_invalid_not_green(self):
        other = self.root / 'without-history'
        other.mkdir()
        self.git('init', '--quiet', cwd=other)
        self.git('commit', '--allow-empty', '-m', 'docs(R150): isolated new repo', cwd=other)
        for name in [AUDIT] + ['scripts/verify/' + n for n in KNOWN]:
            target = other / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes((self.root / name).read_bytes())
        result = self.scan(root=other)
        self.assertEqual(9, result.returncode, result.stdout)
        self.assertIn('迁移基准 Git 原件不可读', result.stdout)

    def test_explicit_fixture_overrides_preserve_their_existing_contract(self):
        fixture = self.write('stub.md', '# 桩\n> 更新时间：2026-11-05（R150 批同步）\n'
                             '- 2026-11-05：**R150** 桩记录\n')
        git_log = self.write('stub-git.txt', 'docs(R150): fixture\n')
        result = self.scan({'VLB_OVERVIEW': str(fixture),
                            'VLB_GIT_LOG': str(git_log),
                            'VLB_HDR_SUBJECT': 'docs(R150): fixture'})
        self.assertEqual(0, result.returncode, result.stdout)
        self.assertIn('显式 VLB_HDR_SUBJECT 桩', result.stdout)
        self.assertIn('声明=R150／实动档头=R150', result.stdout)

    def test_original_criteria_and_ten_selftests_are_unchanged(self):
        before = embedded(original('scripts/verify/v_ledger_batch_consistency.sh').decode())
        after = embedded(SCRIPT.read_text())
        old_functions = {n.name: ast.dump(n, include_attributes=False)
                         for n in ast.parse(before).body if isinstance(n, ast.FunctionDef)}
        new_functions = {n.name: ast.dump(n, include_attributes=False)
                         for n in ast.parse(after).body if isinstance(n, ast.FunctionDef)}
        for name in ('read', 'git_subjects', 'self_r', 'git_rs', 'check',
                     'load_known', 'load_c5_known', 'verdict', 'show'):
            self.assertEqual(old_functions[name], new_functions[name], name)
        marker = "if MODE == 'selftest':"
        end = '\nif not os.path.isfile(OVERVIEW):'
        self.assertEqual(before.split(marker, 1)[1].split(end, 1)[0],
                         after.split(marker, 1)[1].split(end, 1)[0])
        for assignment in ("DOMAIN_MIN = int(os.environ.get('VLB_MIN_R', '140'))",
                           "LAG_MAX = int(os.environ.get('VLB_LAG_MAX', '2'))",
                           "FAIL_SET = os.environ.get('VLB_FAIL_SET', 'C2+C4+C5')"):
            self.assertIn(assignment, before)
            self.assertIn(assignment, after)


if __name__ == '__main__':
    unittest.main(verbosity=2)
