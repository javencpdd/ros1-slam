"""CLI regressions in disposable, same-commit Git repositories; no ROS/network."""
import copy
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from datetime import datetime, timedelta, timezone

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import crosshost as protocol


def env(extra=None):
    result = {k: v for k, v in os.environ.items() if not k.startswith(('GIT_', 'AI_BRIDGE_'))}
    result.update(GIT_CONFIG_NOSYSTEM='1', GIT_CONFIG_GLOBAL='/dev/null')
    result.update(extra or {})
    return result


def run(argv, expected=0, extra=None):
    result = subprocess.run(list(map(str, argv)), env=env(extra), text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=20)
    if expected is not None and result.returncode != expected:
        raise AssertionError((argv, result.returncode, result.stdout, result.stderr))
    return result


def git(root, *args):
    return run(['git', '-c', 'core.hooksPath=/dev/null', '-C', root, *args]).stdout.strip()


def files(root):
    return {str(p.relative_to(root)): (hashlib.sha256(p.read_bytes()).hexdigest(), p.stat().st_mtime_ns)
            for p in root.rglob('*') if p.is_file() and '.git' not in p.relative_to(root).parts}


def state(root):
    return json.loads((root / '.ai/state.json').read_text())


class CrosshostCLI(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='agent-owned-')
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.a, self.b = self.base / 'source', self.base / 'target'
        (self.a / 'scripts/agent').mkdir(parents=True)
        for name in ('bridge.py', 'crosshost.py'):
            shutil.copyfile(HERE / name, self.a / 'scripts/agent' / name)
        (self.a / 'note').mkdir()
        (self.a / 'note/09_跨工具协作契约.md').write_text('# owned contract\n')
        (self.a / '.gitignore').write_text('.ai/\nAGENTS.md\n__pycache__/\n*.pyc\n')
        (self.a / 'src').mkdir()
        (self.a / 'src/中文 [test].txt').write_text('baseline\n')
        (self.a / 'outside.txt').write_text('baseline\n')
        git(self.a, 'init', '-q'); git(self.a, 'config', 'user.name', 'Owned CLI')
        git(self.a, 'config', 'user.email', 'owned@example.invalid')
        git(self.a, 'add', '--', 'scripts/', 'note/', '.gitignore', 'src/', 'outside.txt')
        git(self.a, 'commit', '-qm', 'owned baseline')
        run(['git', '-c', 'core.hooksPath=/dev/null', 'clone', '-q', self.a, self.b])
        for root in (self.a, self.b):
            git(root, 'config', 'user.name', 'Owned CLI'); git(root, 'config', 'user.email', 'owned@example.invalid')
        self.cli(self.a, 'bootstrap', '--platform', 'x86_sim', '--as', 'codex')
        self.cli(self.b, 'bootstrap', '--platform', 'lite3', '--as', 'codex')
        self.ticket = self.base / 'host.json'
        self.packet = self.base / 'packet.json'
        self.cli(self.b, 'host', '--out', self.ticket)

    def cli(self, root, *args, expected=0, extra=None, shim=False):
        script = root / ('.ai/bridge.py' if shim else 'scripts/agent/bridge.py')
        # Deliberately omit -B: inspect/reject must not create module caches.
        return run([sys.executable, script, '--root', root, *args], expected, extra)

    def task(self, root=None, title='selected', scope='src/'):
        root = root or self.a
        output = self.cli(root, 'new', title, '--as', 'codex', '--scope', scope).stdout
        tid = re.search(r'T-\d{8}-\d+', output).group()
        self.cli(root, 'claim', tid, '--as', 'codex')
        return tid

    def export(self, tid=None, **kwargs):
        tid = tid or self.task()
        self.cli(self.a, 'export', tid, '--destination', self.ticket, '--to', 'codex',
                 '--summary', 'resume selected work', '--evidence', 'commit:path/hash',
                 '--out', self.packet, '--as', 'codex', **kwargs)
        return tid

    def edit_packet(self, edit, path=None):
        value = json.loads(self.packet.read_text()); edit(value['payload'])
        value['payload']['core_sha256'] = protocol.digest(protocol.canonical(value['payload']['core']))
        value = protocol.envelope('agent_handoff/v1', value['payload'])
        target = path or self.base / 'changed.json'
        target.write_bytes(protocol.canonical(value))
        return target

    def reject_unchanged(self, root, *args):
        before = files(root)
        result = self.cli(root, *args, expected=None)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertEqual(files(root), before)
        return result

    def mutate_local(self, root, edit):
        # Corrupt fixtures only; production state is never edited by tests.
        value = state(root); edit(value)
        (root / '.ai/state.json').write_text(json.dumps(value, ensure_ascii=False))

    def test_bootstrap_preserves_state_and_is_idempotent(self):
        self.task(); before = files(self.a)
        self.cli(self.a, 'bootstrap', '--platform', 'x86_sim', '--as', 'codex')
        self.assertEqual(files(self.a), before)
        self.cli(self.a, 'resume', '--as', 'codex', shim=True)
        self.cli(self.a, 'check', shim=True)
        self.assertFalse(list(self.a.rglob('__pycache__')))
        self.reject_unchanged(self.a, 'bootstrap', '--platform', 'm20')

    def test_unknown_runner_and_pointer_are_protected(self):
        runner = self.a / '.ai/bridge.py'; runner.write_text('# custom\n')
        self.reject_unchanged(self.a, 'bootstrap', '--platform', 'x86_sim')
        self.cli(self.a, 'bootstrap', '--platform', 'x86_sim', '--replace-legacy')
        self.assertEqual(next((self.a / '.ai/legacy').glob('bridge-*.py')).read_text(), '# custom\n')
        (self.a / 'AGENTS.md').write_text('custom without contract\n')
        self.reject_unchanged(self.a, 'bootstrap', '--platform', 'x86_sim')

    def test_legacy_commands_and_aliases(self):
        tid = self.task()
        self.cli(self.a, 'note', tid, 'note', '--as', 'codex')
        self.cli(self.a, 'handoff', tid, '--to', 'qoder', '--summary', 'review', '--as', 'codex')
        self.cli(self.a, 'claim', tid, '--as', 'qoder')
        for command in ('status', 'ls', 'json', 'check'):
            self.cli(self.a, command)
        self.cli(self.a, 'knowledge', 'knowledge'); self.cli(self.a, 'mem', 'alias')
        self.cli(self.a, 'done', tid, '--summary', 'finished')
        self.assertEqual(state(self.a)['tasks'][tid]['status'], 'done')

    def test_whole_handoff_and_no_other_task_or_dirty_file(self):
        tid = self.task(); other = self.cli(self.a, 'new', 'PRIVATE OTHER CARD', '--as', 'codex', '--scope', 'outside.txt').stdout.strip()
        self.cli(self.a, 'note', other, 'PRIVATE OTHER NOTE', '--as', 'codex')
        (self.a / 'outside.txt').write_text('PRIVATE DIRTY FILE\n')
        self.export(tid)
        source = state(self.a)
        self.assertEqual(source['tasks'][tid]['status'], 'transferred')
        self.assertIsNone(source['current_task'])
        packet = self.packet.read_text()
        for value in ('PRIVATE OTHER CARD', 'PRIVATE OTHER NOTE', 'PRIVATE DIRTY FILE', str(self.a)):
            self.assertNotIn(value, packet)
        self.cli(self.b, 'inspect', self.packet, '--show-card')
        self.cli(self.b, 'import', self.packet, '--as', 'codex')
        target = state(self.b)
        self.assertEqual(target['tasks'][tid]['status'], 'review')
        self.assertEqual(target['tasks'][tid]['task_uuid'], source['tasks'][tid]['task_uuid'])
        self.cli(self.b, 'check')
        self.assertEqual((self.a / 'outside.txt').read_text(), 'PRIVATE DIRTY FILE\n')

    def test_source_fence_all_legacy_writes_and_init(self):
        tid = self.export()
        for args in (('claim', tid, '--as', 'codex'), ('note', tid, 'bad', '--as', 'codex'),
                     ('handoff', tid, '--to', 'qoder', '--summary', 'bad'), ('done', tid), ('init', '--force')):
            with self.subTest(args=args): self.reject_unchanged(self.a, *args)
        self.cli(self.a, 'check')

    def test_collision_preserves_completed_task_and_orphan_card(self):
        old = self.task(self.b, 'old target'); self.cli(self.b, 'done', old)
        old_card = (self.b / '.ai/tasks' / (old + '.md')).read_bytes()
        orphan = self.b / '.ai/tasks' / (old.rsplit('-', 1)[0] + '-02.md')
        orphan.write_text('ORPHAN MUST SURVIVE')
        tid = self.export(); self.assertEqual(old, tid)
        output = json.loads(self.cli(self.b, 'import', self.packet, '--as', 'codex').stdout)
        self.assertNotEqual(output['local_id'], old)
        self.assertFalse(output['local_id'].endswith('-02'))
        self.assertEqual((self.b / '.ai/tasks' / (old + '.md')).read_bytes(), old_card)
        self.assertEqual(orphan.read_text(), 'ORPHAN MUST SURVIVE')
        self.assertEqual(state(self.b)['tasks'][old]['status'], 'done')

    def test_duplicate_after_progress_done_and_head_advance_is_readonly(self):
        tid = self.export(); self.cli(self.b, 'import', self.packet, '--as', 'codex')
        self.cli(self.b, 'claim', tid, '--as', 'codex'); self.cli(self.b, 'note', tid, 'progress')
        self.cli(self.b, 'done', tid)
        (self.b / 'outside.txt').write_text('next commit'); git(self.b, 'add', '--', 'outside.txt')
        git(self.b, 'commit', '-qm', 'advance')
        before = files(self.b)
        self.assertTrue(json.loads(self.cli(self.b, 'import', self.packet, '--as', 'codex').stdout)['already_imported'])
        self.assertEqual(files(self.b), before)

    def test_target_busy_wrong_agent_host_platform_and_commit(self):
        tid = self.export(); busy = self.task(self.b)
        self.reject_unchanged(self.b, 'import', self.packet, '--as', 'codex')
        self.cli(self.b, 'done', busy)
        self.reject_unchanged(self.b, 'import', self.packet, '--as', 'qoder')
        for edit in (lambda p: p['core']['destination'].update(host_id=str(__import__('uuid').uuid4())),
                     lambda p: p['core']['destination'].update(platform='m20'),
                     lambda p: p['core'].update(code_commit='0' * 40)):
            changed = self.edit_packet(edit)
            self.reject_unchanged(self.b, 'import', changed, '--as', 'codex')

    def test_dirty_scope_tracked_untracked_rename_and_literal_chinese(self):
        tid = self.task(scope='src/中文 [test].txt')
        tracked = self.a / 'src/中文 [test].txt'; tracked.write_text('changed')
        self.reject_unchanged(self.a, 'export', tid, '--destination', self.ticket, '--to', 'codex', '--summary', 'resume', '--out', self.packet, '--as', 'codex')
        git(self.a, 'restore', '--', 'src/中文 [test].txt')
        git(self.a, 'mv', '--', 'src/中文 [test].txt', 'outside-renamed.txt')
        self.reject_unchanged(self.a, 'export', tid, '--destination', self.ticket, '--to', 'codex', '--summary', 'resume', '--out', self.packet, '--as', 'codex')
        git(self.a, 'reset', '--hard', 'HEAD')
        (self.a / 'src/new.txt').write_text('untracked')
        self.mutate_local(self.a, lambda s: s['tasks'][tid].update(scope=['src/']))
        self.reject_unchanged(self.a, 'export', tid, '--destination', self.ticket, '--to', 'codex', '--summary', 'resume', '--out', self.packet, '--as', 'codex')

    def test_blocked_dependencies_not_silently_dropped(self):
        tid = self.task()
        self.mutate_local(self.a, lambda s: s['tasks'][tid].update(blocked_by=['T-20200101-01']))
        self.reject_unchanged(self.a, 'export', tid, '--destination', self.ticket, '--to', 'codex', '--summary', 'resume', '--out', self.packet, '--as', 'codex')

    def test_expired_and_future_lease_renew_keeps_core(self):
        tid = self.export()
        for delta in (-48, 48):
            def edit(p):
                t = datetime.now(timezone.utc).replace(microsecond=0) + timedelta(hours=delta)
                p['core']['created_at'] = protocol.timestamp(t)
                p['lease'].update(issued_at=protocol.timestamp(t), expires_at=protocol.timestamp(t + timedelta(hours=24)))
            changed = self.edit_packet(edit)
            self.reject_unchanged(self.b, 'import', changed, '--as', 'codex')
        original = json.loads(self.packet.read_text()); before = state(self.a)
        renewed = self.base / 'renewed.json'
        self.cli(self.a, 'export', tid, '--renew', '--ttl-hours', '2', '--out', renewed, '--as', 'codex')
        value = json.loads(renewed.read_text())
        self.assertEqual(value['payload']['core'], original['payload']['core'])
        self.assertEqual(value['payload']['lease']['revision'], 1)
        self.assertEqual(state(self.a), before)
        self.cli(self.b, 'import', renewed, '--as', 'codex')
        snap = files(self.b); self.cli(self.b, 'import', self.packet, '--as', 'codex')
        self.assertEqual(files(self.b), snap)

    def test_direct_roundtrip_and_replay_does_not_reactivate(self):
        tid = self.export(); self.cli(self.b, 'import', self.packet, '--as', 'codex')
        self.cli(self.b, 'claim', tid, '--as', 'codex')
        a_ticket = self.base / 'a-ticket.json'; returned = self.base / 'return.json'
        self.cli(self.a, 'host', '--out', a_ticket)
        self.cli(self.b, 'export', tid, '--destination', a_ticket, '--to', 'codex', '--summary', 'return', '--out', returned, '--as', 'codex')
        self.cli(self.a, 'import', returned, '--as', 'codex')
        self.assertEqual(state(self.a)['tasks'][tid]['transfer_hop'], 2)
        before = files(self.b); self.cli(self.b, 'import', self.packet, '--as', 'codex')
        self.assertEqual(files(self.b), before)
        self.assertEqual(state(self.b)['tasks'][tid]['status'], 'transferred')

    def test_json_hash_schema_types_paths_and_limits_readonly(self):
        self.export()
        invalid = [b'{"schema":1,"schema":2}', b'NaN', b'\xff', b'[' * 2000,
                   b'x' * (protocol.MAX_PACKAGE + 1)]
        value = json.loads(self.packet.read_text()); value['sha256'] = '0' * 64
        invalid.append(json.dumps(value).encode())
        for index, data in enumerate(invalid):
            path = self.base / ('invalid%d.json' % index); path.write_bytes(data)
            self.reject_unchanged(self.b, 'inspect', path)
        edits = [lambda p: p['core'].update(hop=True), lambda p: p['core'].update(extra='unknown'),
                 lambda p: p['core']['task'].update(scope=['../escape']),
                 lambda p: p['core']['task'].update(scope=['/tmp/path']),
                 lambda p: p['core']['task'].update(scope=['src\\bad']),
                 lambda p: p['core']['task'].update(scope=['.git/config']),
                 lambda p: p['core']['task'].update(scope=[]),
                 lambda p: p['core']['task'].update(id='../../escape'),
                 lambda p: p['core'].update(card='x' * (256 * 1024 + 1)),
                 lambda p: p['lease'].update(revision=False),
                 lambda p: p['lease'].update(issued_at='2026-01-01T00:00:00+08:00')]
        for edit in edits:
            path = self.edit_packet(edit)
            self.reject_unchanged(self.b, 'inspect', path)
        self.assertFalse(list(self.b.rglob('__pycache__')))

    def test_symlink_package_state_scope_and_nonroot(self):
        tid = self.export()
        linked = self.base / 'link.json'; linked.symlink_to(self.packet)
        self.reject_unchanged(self.b, 'inspect', linked)
        self.reject_unchanged(self.b, '--root', self.b / 'src', 'host')
        scope = self.b / 'src'; shutil.rmtree(scope); scope.symlink_to(self.a / 'src', target_is_directory=True)
        self.reject_unchanged(self.b, 'import', self.packet, '--as', 'codex')
        scope.unlink(); scope.mkdir()
        host = self.b / '.ai/host.json'; host.rename(self.base / 'real-host.json'); host.symlink_to(self.base / 'real-host.json')
        self.reject_unchanged(self.b, 'host')

    def test_git_environment_does_not_redirect_repository(self):
        tid = self.task()
        poisoned = {'GIT_DIR': str(self.b / '.git'), 'GIT_WORK_TREE': str(self.b),
                    'GIT_INDEX_FILE': str(self.base / 'fake-index')}
        result = self.cli(self.a, 'host', extra=poisoned)
        value = json.loads(result.stdout)
        self.assertEqual(value['payload']['host_id'], json.loads((self.a / '.ai/host.json').read_text())['host_id'])
        self.assertFalse((self.base / 'fake-index').exists())

    def test_publication_failure_fences_and_retry_uses_same_package(self):
        tid = self.task(); self.packet.write_text('do not overwrite')
        self.export(tid, expected=2)
        value = state(self.a)['tasks'][tid]
        self.assertEqual(value['status'], 'transferred')
        self.assertFalse((self.a / '.ai/transfer.pending.json').exists())
        self.assertEqual(self.packet.read_text(), 'do not overwrite')
        self.packet.unlink()
        self.cli(self.a, 'export', tid, '--out', self.packet, '--as', 'codex')
        self.assertEqual(json.loads(self.packet.read_text())['payload']['core']['package_id'], value['outgoing_transfer']['package_id'])
        self.reject_unchanged(self.a, 'recover', '--as', 'codex')

    def fault(self, root, command, tid=None, hard=False):
        # Instrument the in-memory fixture module, never alter committed tool code.
        driver = '''import argparse, importlib.util, os, sys
root, command, task_id, ticket, packet, hard = sys.argv[1:]
os.environ['AI_BRIDGE_ROOT'] = root
sys.path.insert(0, root + '/scripts/agent')
import bridge, crosshost
original = bridge.commit
def fail(st, line):
    bridge.save_state(st)
    if hard == 'yes': os._exit(77)
    raise OSError('owned injected commit failure')
bridge.commit = fail
if command == 'export':
    args = argparse.Namespace(id=task_id, destination=ticket, to='codex', summary='resume', evidence=[], ttl_hours=24, renew=False, out=packet, as_agent='codex', crosshost_function=crosshost.cmd_export)
else:
    args = argparse.Namespace(package=packet, as_agent='codex', crosshost_function=crosshost.cmd_import)
sys.exit(crosshost.dispatch(bridge, args))
'''
        return run([sys.executable, '-B', '-c', driver, root, command, tid or '', self.ticket, self.packet,
                    'yes' if hard else 'no'], expected=77 if hard else 2)

    def test_export_exception_rolls_back_and_crash_requires_recover(self):
        tid = self.task(); before = files(self.a)
        self.fault(self.a, 'export', tid)
        self.assertEqual({k: v[0] for k, v in files(self.a).items()}, {k: v[0] for k, v in before.items()})
        self.assertFalse(self.packet.exists())
        self.fault(self.a, 'export', tid, hard=True)
        self.assertTrue((self.a / '.ai/transfer.pending.json').exists())
        self.reject_unchanged(self.a, 'claim', tid, '--as', 'codex')
        self.reject_unchanged(self.a, 'resume', '--as', 'codex')
        self.reject_unchanged(self.a, 'init', '--force')
        self.cli(self.a, 'recover', '--as', 'codex')
        self.assertEqual(state(self.a)['tasks'][tid]['status'], 'in_progress')
        self.cli(self.a, 'check')
        self.assertFalse(self.packet.exists())

    def test_import_exception_and_crash_restore_without_half_task(self):
        self.export(); before = files(self.b)
        self.fault(self.b, 'import')
        # Rollback restores bytes; rewrites may update mtimes.
        self.assertEqual({k: v[0] for k, v in files(self.b).items()}, {k: v[0] for k, v in before.items()})
        self.fault(self.b, 'import', hard=True)
        self.cli(self.b, 'recover', '--as', 'codex')
        self.assertEqual({k: v[0] for k, v in files(self.b).items()}, {k: v[0] for k, v in before.items()})
        self.cli(self.b, 'import', self.packet, '--as', 'codex')
        self.cli(self.b, 'check')

    def test_copied_root_and_epoch_drift_fail_without_self_healing(self):
        self.task()
        self.mutate_local(self.a, lambda s: s.update(root=str(self.b)))
        self.reject_unchanged(self.a, 'resume', '--as', 'codex')
        self.reject_unchanged(self.a, 'bootstrap', '--platform', 'x86_sim')
        self.mutate_local(self.a, lambda s: s.update(root=str(self.a), epoch=s['epoch'] + 1))
        self.reject_unchanged(self.a, 'new', 'not allowed', '--scope', 'src/')


    def test_actual_expired_outbox_renew_after_source_head_advance(self):
        tid = self.task()
        driver = """import sys, os
from datetime import datetime, timedelta, timezone
root = sys.argv[1]
os.environ['AI_BRIDGE_ROOT'] = root
sys.path.insert(0, root + '/scripts/agent')
import bridge, crosshost
past = datetime.now(timezone.utc).replace(microsecond=0) - timedelta(hours=48)
crosshost.utc_now = lambda: past
sys.argv = ['bridge.py', '--root', root, *sys.argv[2:]]
sys.exit(bridge.main())
"""
        run([sys.executable, '-B', '-c', driver, self.a, 'export', tid,
             '--destination', self.ticket, '--to', 'codex', '--summary', 'resume',
             '--out', self.packet, '--as', 'codex'])
        original = json.loads(self.packet.read_text())
        self.reject_unchanged(self.b, 'import', self.packet, '--as', 'codex')
        (self.a / 'outside.txt').write_text('source advances')
        git(self.a, 'add', '--', 'outside.txt'); git(self.a, 'commit', '-qm', 'advance source')
        renewed = self.base / 'renewed.json'
        self.cli(self.a, 'export', tid, '--renew', '--out', renewed, '--as', 'codex')
        self.assertEqual(json.loads(renewed.read_text())['payload']['core'], original['payload']['core'])
        self.cli(self.b, 'import', renewed, '--as', 'codex')

    def test_receipt_core_collision_and_corrupt_private_inbox_rejected(self):
        tid = self.export(); self.cli(self.b, 'import', self.packet, '--as', 'codex')
        changed = self.edit_packet(lambda p: p['core'].update(summary='different core'))
        self.reject_unchanged(self.b, 'import', changed, '--as', 'codex')
        inbox = next((self.b / '.ai/inbox').glob('*.json')); inbox.write_text('{}')
        self.reject_unchanged(self.b, 'import', self.packet, '--as', 'codex')

    def test_lock_failure_and_bad_recovery_marker_do_not_write(self):
        tid = self.task(); before = files(self.a)
        driver = """import os, sys
root = sys.argv[1]
os.environ['AI_BRIDGE_ROOT'] = root
sys.path.insert(0, root + '/scripts/agent')
import bridge
def fail(*args): raise OSError('owned lock failure')
bridge.fcntl.flock = fail
sys.argv = ['bridge.py', '--root', root, 'claim', sys.argv[2], '--as', 'codex']
sys.exit(bridge.main())
"""
        run([sys.executable, '-B', '-c', driver, self.a, tid], expected=2)
        self.assertEqual(files(self.a), before)
        self.fault(self.a, 'export', tid, hard=True)
        marker = self.a / '.ai/transfer.pending.json'
        value = json.loads(marker.read_text()); value['payload']['../outside.txt'] = None
        marker.write_bytes(protocol.canonical(protocol.envelope('agent_recovery/v1', value['payload'])))
        self.reject_unchanged(self.a, 'recover', '--as', 'codex')
        self.assertTrue(marker.exists())
        value['payload'].pop('../outside.txt')
        value['payload'].pop('HANDOFF.md')
        marker.write_bytes(protocol.canonical(protocol.envelope('agent_recovery/v1', value['payload'])))
        self.reject_unchanged(self.a, 'recover', '--as', 'codex')

    def test_non_direct_three_host_return_rejected(self):
        tid = self.export(); self.cli(self.b, 'import', self.packet, '--as', 'codex')
        c = self.base / 'third'
        run(['git', '-c', 'core.hooksPath=/dev/null', 'clone', '-q', self.a, c])
        self.cli(c, 'bootstrap', '--platform', 'm20')
        ticket_c = self.base / 'c-ticket.json'; packet_c = self.base / 'c-packet.json'
        self.cli(c, 'host', '--out', ticket_c)
        self.cli(self.b, 'export', tid, '--destination', ticket_c, '--to', 'codex', '--summary', 'next', '--out', packet_c, '--as', 'codex')
        self.cli(c, 'import', packet_c, '--as', 'codex')
        ticket_a = self.base / 'a-ticket.json'; packet_a = self.base / 'a-packet.json'
        self.cli(self.a, 'host', '--out', ticket_a)
        self.cli(c, 'export', tid, '--destination', ticket_a, '--to', 'codex', '--summary', 'non-direct return', '--out', packet_a, '--as', 'codex')
        self.reject_unchanged(self.a, 'import', packet_a, '--as', 'codex')
        self.assertEqual(state(self.a)['tasks'][tid]['status'], 'transferred')

if __name__ == '__main__':
    unittest.main(verbosity=2)
