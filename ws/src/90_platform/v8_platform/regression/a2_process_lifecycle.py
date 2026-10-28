#!/usr/bin/env python3
"""A2 OS-process fault injection: crash, readiness/runtime timeout and cleanup.

No ROS or Gazebo is claimed by this harness. Real workers hold an exclusive file
lease and TCP listener; a TERM-ignoring descendant inherits both. The evaluator
checks process-group survivors and reacquires each resource after teardown.
Use: python3 -B a2_process_lifecycle.py --out /tmp/a2-process.json
The same command runs inside the fixed Noetic image for target-OS evidence.
"""
import argparse
import ctypes
import fcntl
import json
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE / 'orchestrator'))
import orchestrator as orch  # noqa: E402
from process_runner import ProcessRunner  # noqa: E402


def resource_child():
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    print('CHILD_READY', flush=True)
    while True:
        time.sleep(1)


def worker(lease, ready=True, with_child=False):
    with open(lease + '.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0))
            listener.listen(1)
            child = None
            if with_child:
                child = subprocess.Popen(
                    [sys.executable, '-B', str(Path(__file__).resolve()), '--resource-child'],
                    pass_fds=(lock.fileno(), listener.fileno()), stdout=subprocess.PIPE)
                if child.stdout.readline() != b'CHILD_READY\n':
                    raise RuntimeError('resource child did not become ready')
            Path(lease).write_text(json.dumps({
                'pid': os.getpid(), 'child_pid': child.pid if child else None,
                'port': listener.getsockname()[1], 'lock_path': lease + '.lock'}))
            if ready:
                print('READY', flush=True)
            while True:
                time.sleep(1)


def resources_released(lease):
    meta = json.loads(Path(lease).read_text())
    with open(meta['lock_path'], 'a') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return False
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)
    with socket.socket() as probe:
        try:
            probe.bind(('127.0.0.1', meta['port']))
        except OSError:
            return False
    return True


def make_runner(cid, lease, *, ready=True, with_child=True, role='local_planner'):
    spec = orch.ComponentSpec(cid, [role], stop_method='process_group_signal')
    cmd = [sys.executable, '-B', str(Path(__file__).resolve()), '--worker', '--lease', str(lease)]
    if not ready:
        cmd.append('--no-ready')
    if with_child:
        cmd.append('--with-child')
    class FixtureRunner(ProcessRunner):
        def start(self):
            result = super().start()
            if not ready:
                # Ensure this cell tests readiness *after resource acquisition*,
                # rather than whether worker startup can win a 0.2 s host race.
                wait_for(lambda: Path(lease).exists(), timeout=5)
            return result

    return FixtureRunner(spec, cmd, ready_marker='READY'), spec


def wait_for(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise RuntimeError('observation deadline expired')


def run_case(case, root):
    engine = orch.Orchestrator()
    lease = root / (case + '.json')
    runner, spec = make_runner('planner', lease, ready=case != 'ready_timeout')
    runners = {'planner': runner}
    specs = [spec]
    all_runners = [runner]
    other = None
    other_task = None
    if case == 'startup_error':
        bad = orch.ComponentSpec('bad', ['odometry'])

        class FailAfterResourceAcquired(ProcessRunner):
            def start(self):
                # Reproduce the costly case: first component already owns resources
                # before a later exec fails, rather than racing cleanup vs its boot.
                orch.wait_ready([runner], timeout=3)
                return super().start()

        runners['bad'] = FailAfterResourceAcquired(bad, ['/nonexistent/a2-program'])
        specs.append(bad)
        all_runners.append(runners['bad'])
    if case == 'isolation':
        other, other_spec = make_runner('peer', root / 'peer.json', with_child=False)
        all_runners.append(other)
        other_task = engine.submit('peer goal', [other_spec])
        engine.start(other_task, {'peer': other}, ready_timeout=3)
    task = engine.submit('fault test goal', specs)
    t0 = time.monotonic()
    error = None
    peer_unaffected = None
    try:
        try:
            engine.start(task, runners,
                         ready_timeout=0.2 if case == 'ready_timeout' else 3,
                         run_timeout=0.1 if case == 'run_timeout' else None)
        except orch.OrchestratorError as exc:
            error = exc.code
        if case in ('crash', 'isolation'):
            # Kill only the leader: its child keeps BOTH resources until group cleanup.
            os.kill(runner.proc.pid, signal.SIGKILL)
            wait_for(lambda: not runner.is_alive())
            if resources_released(lease):
                raise RuntimeError('fault did not preserve child-owned resources')
            engine.health_check(task, runners)
        elif case == 'run_timeout':
            while task.state == orch.TaskState.RUNNING:
                engine.health_check(task, runners)
                if time.monotonic() - t0 > 4:
                    raise RuntimeError('run deadline was not enforced')
                time.sleep(0.005)
        elif case == 'cancel':
            engine.cancel(task, runners)
        if other is not None:
            peer_unaffected = other.is_alive() and not resources_released(root / 'peer.json')
            engine.complete(other_task, {'peer': other})
        if not Path(lease).exists():
            raise RuntimeError('%s: resource fixture never acquired lease (state=%s, error=%s)' %
                               (case, task.state.value, error))
        meta = json.loads(Path(lease).read_text())
        survivors = runner.group_members()
        released = resources_released(lease)
        expected_error = {'ready_timeout': 'E_ORCH_READY_TIMEOUT',
                          'startup_error': 'E_ORCH_START_FAILED'}.get(case)
        cause = task.events[-1][3]
        ok = (task.state == (orch.TaskState.CANCELLED if case == 'cancel' else orch.TaskState.FAILED)
              and not survivors and released and error == expected_error)
        if case == 'run_timeout':
            ok = ok and cause == 'E_ORCH_RUN_TIMEOUT'
        if case in ('crash', 'isolation'):
            ok = ok and cause == 'died:planner'
        if other is not None:
            ok = (ok and peer_unaffected and other_task.state == orch.TaskState.COMPLETED
                  and resources_released(root / 'peer.json'))
        return {'case': case, 'verdict': 'PASS' if ok else 'FAIL',
                'state': task.state.value, 'cause': cause, 'error': error,
                'elapsed_wall_s': round(time.monotonic() - t0, 6),
                'pid': meta['pid'], 'child_pid': meta['child_pid'],
                'group_survivors': survivors, 'resources_released': released,
                'peer_unaffected': peer_unaffected, 'stop_record': task.stop_record}
    finally:
        for item in all_runners:
            item.reap()
        while True:
            try:
                child_pid, _status = os.waitpid(-1, os.WNOHANG)
                if child_pid == 0:
                    break
            except ChildProcessError:
                break


def run_matrix():
    # This Linux-only fault harness adopts and reaps its own orphaned workers.
    # Do not rely on container PID 1 (often a non-reaping shell/tail).
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(36, 1, 0, 0, 0) != 0:  # PR_SET_CHILD_SUBREAPER
        raise OSError(ctypes.get_errno(), 'cannot enable owned-child reaping')
    with tempfile.TemporaryDirectory(prefix='v8-a2-process-') as work:
        root = Path(work)
        cells = []
        for case in ('crash', 'ready_timeout', 'run_timeout', 'startup_error', 'cancel', 'isolation'):
            try:
                cells.append(run_case(case, root))
            except Exception as exc:
                cells.append({'case': case, 'verdict': 'FAIL', 'error': str(exc)})
    return {'scope': 'A2.real_os_processes;not_gazebo;no_151_budget_freeze',
            'verdict': 'PASS' if all(c['verdict'] == 'PASS' for c in cells) else 'FAIL',
            'cell_count': len(cells), 'cells': cells}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out')
    parser.add_argument('--worker', action='store_true')
    parser.add_argument('--resource-child', action='store_true')
    parser.add_argument('--lease')
    parser.add_argument('--no-ready', action='store_true')
    parser.add_argument('--with-child', action='store_true')
    args = parser.parse_args()
    if args.resource_child:
        resource_child()
        return 0
    if args.worker:
        worker(args.lease, ready=not args.no_ready, with_child=args.with_child)
        return 0
    result = run_matrix()
    encoded = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
    if args.out:
        Path(args.out).write_text(encoded)
    print(encoded, end='')
    return 0 if result['verdict'] == 'PASS' else 6


if __name__ == '__main__':
    sys.exit(main())
