"""A2 运行面测试：真进程停止链——就绪门（条件式）、停止序、TERM 挂起升级 KILL、
双任务隔离、与 Orchestrator 全链（真 PID 真信号真退净）。"""
import os
import subprocess
import sys
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import orchestrator as orch  # noqa: E402
from process_runner import ProcessRunner  # noqa: E402

PY = sys.executable
# 哑进程：打印 READY 后长眠（trap TERM 可控：hang 版忽略 TERM）
SLEEPER = "import time; print('READY', flush=True); time.sleep(3600)"
HANGER = ("import signal, time; signal.signal(signal.SIGTERM, signal.SIG_IGN); "
          "print('READY', flush=True); time.sleep(3600)")


def spec(cid, roles, stop='process_group_signal'):
    return orch.ComponentSpec(cid, roles, stop_method=stop)


def make(specs_cmds, **per_id):
    runners = {}
    for s, cmd in specs_cmds:
        kw = per_id.get(s.id, {})
        r = ProcessRunner(s, cmd, **kw)
        r.start()
        runners[s.id] = r
    return runners


class ProcessRunnerTests(unittest.TestCase):
    def addCleanupRunners(self, runners):
        for r in runners.values():
            self.addCleanup(r.reap)

    def test_ready_gate_by_marker(self):
        s = spec('a', ['odometry'])
        r = ProcessRunner(s, [PY, '-c', SLEEPER], ready_marker='READY')
        r.start()
        self.addCleanup(r.reap)
        orch.wait_ready([r], timeout=2)
        self.assertTrue(r.ready())          # 条件式：看到 READY 行
        self.assertFalse(r.stops)

    def test_ready_timeout_rejected(self):
        s = spec('a', ['odometry'])
        r = ProcessRunner(s, [PY, '-c', "import time; time.sleep(3600)"],
                          ready_marker='READY', ready_timeout=1.0)
        r.start()
        self.addCleanup(r.reap)
        t0 = time.monotonic()
        with self.assertRaises(orch.OrchestratorError) as caught:
            orch.wait_ready([r], timeout=0.1)
        self.assertEqual(caught.exception.code, 'E_ORCH_READY_TIMEOUT')
        self.assertLess(time.monotonic() - t0, 0.8)

    def test_stop_chain_order_and_clean(self):
        specs = [spec('scan', ['local_planner']), spec('dwa', ['controller']),
                 spec('mux', ['safety_arbiter'])]
        runners = make([(s, [PY, '-c', SLEEPER]) for s in specs])
        self.addCleanupRunners(runners)
        engine = orch.Orchestrator()
        task = engine.submit('a', specs)
        orch.wait_ready(list(runners.values()), timeout=10)
        task.transition(orch.TaskState.STARTING, 'start')
        task.transition(orch.TaskState.RUNNING, 'ready')
        record = engine.complete(task, runners)
        self.assertEqual([r['id'] for r in record], ['dwa', 'scan', 'mux'])  # 仲裁器最后
        self.assertTrue(all(r['outcome'] == 'stopped' for r in record))
        for r in runners.values():
            r.reap()
        self.assertTrue(all(not r.is_alive() for r in runners.values()))    # 真退净

    def test_term_hang_escalates_to_kill(self):
        s = spec('mux', ['safety_arbiter'])
        # ready_marker＝READY：条件式就绪门同步「SIG_IGN 已装好」（实测竞态：marker=None
        # 时固定等待下 TERM 可在 signal(SIG_IGN) 装好前到达→假死，A2 真实发现）
        r = ProcessRunner(s, [PY, '-c', HANGER], ready_marker='READY')
        r.start()
        self.addCleanup(r.reap)
        orch.wait_ready([r], timeout=2)
        r.stop('process_group_signal')
        deadline = time.monotonic() + 1.5
        while r.is_alive() and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertTrue(r.is_alive())               # TERM 被挂起
        r.force()
        r.reap()
        self.assertFalse(r.is_alive())              # KILL 退净
        self.assertEqual(r.stops, ['process_group_signal', 'force'])

    def test_stop_kills_descendant_in_own_session(self):
        """roslaunch starts each node in its own session; group-only stop strands them."""
        pidfile = Path('/tmp/v8-process-runner-child-%d.pid' % os.getpid())
        code = ("import os, time, sys\n"
                "print('READY', flush=True)\n"
                "p = os.fork()\n"
                "if p == 0:\n"
                "    os.setsid()\n"
                "    open(sys.argv[1], 'w').write(str(os.getpid()))\n"
                "    time.sleep(3600)\n"
                "else:\n"
                "    time.sleep(3600)\n")
        r = ProcessRunner(spec('nav', ['global_planner']),
                          [PY, '-c', code, str(pidfile)], ready_marker='READY')
        self.addCleanup(r.reap)
        self.addCleanup(lambda: pidfile.unlink() if pidfile.exists() else None)
        r.start()
        orch.wait_ready([r], timeout=3)
        deadline = time.monotonic() + 3
        while not pidfile.exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        child = int(pidfile.read_text())
        r.stop('process_group_signal')
        r.reap(timeout=3)
        self.assertTrue(r.is_stopped())
        stat = Path('/proc') / str(child) / 'stat'
        if stat.exists():
            self.assertEqual(stat.read_text().rpartition(')')[2].split()[0], 'Z')

    def test_repeated_start_keeps_original_process(self):
        r = ProcessRunner(spec('a', ['odometry']), [PY, '-c', SLEEPER], ready_marker='READY')
        self.addCleanup(r.reap)
        r.start()
        original = r.proc.pid
        r.start()
        self.assertEqual(r.proc.pid, original)
        orch.wait_ready([r], timeout=2)
        self.assertEqual(r.group_members(), [original])

    def test_partial_stdout_cannot_block_readiness_deadline(self):
        r = ProcessRunner(spec('a', ['odometry']),
                          [PY, '-c', "import time; print('READY', end='', flush=True); time.sleep(3600)"],
                          ready_marker='READY', ready_timeout=10)
        self.addCleanup(r.reap)
        r.start()
        t0 = time.monotonic()
        with self.assertRaises(orch.OrchestratorError):
            orch.wait_ready([r], timeout=0.1)
        self.assertLess(time.monotonic() - t0, 0.8)

    def test_stdout_after_ready_is_drained(self):
        # Pipe capacity is much smaller than 2 MiB; DONE appears only if logging is drained.
        r = ProcessRunner(spec('a', ['odometry']),
                          [PY, '-c', "import sys,time; print('READY',flush=True); "
                           "sys.stdout.write('x'*2097152+'\\nDONE\\n'); sys.stdout.flush(); time.sleep(3600)"],
                          ready_marker='DONE')
        self.addCleanup(r.reap)
        r.start()
        orch.wait_ready([r], timeout=3)
        self.assertTrue(r.ready())

    def test_start_failure_cleans_previous_real_process(self):
        specs = [spec('first', ['controller']), spec('bad', ['odometry'])]
        runners = {
            'first': ProcessRunner(specs[0], [PY, '-c', SLEEPER]),
            'bad': ProcessRunner(specs[1], ['/nonexistent/a2-runner-program'])}
        self.addCleanupRunners(runners)
        engine = orch.Orchestrator()
        task = engine.submit('g', specs)
        with self.assertRaises(orch.OrchestratorError) as caught:
            engine.start(task, runners)
        self.assertEqual(caught.exception.code, 'E_ORCH_START_FAILED')
        self.assertEqual(task.state, orch.TaskState.FAILED)
        self.assertTrue(all(r.is_stopped() for r in runners.values()))

    def test_runtime_timeout_cleans_processes(self):
        s = spec('scan', ['local_planner'])
        r = ProcessRunner(s, [PY, '-c', HANGER], ready_marker='READY')
        self.addCleanup(r.reap)
        engine = orch.Orchestrator()
        task = engine.submit('g', [s])
        engine.start(task, {'scan': r}, run_timeout=0.02, ready_timeout=2)
        deadline = time.monotonic() + 2
        while task.state == orch.TaskState.RUNNING and time.monotonic() < deadline:
            engine.health_check(task, {'scan': r})
            time.sleep(0.005)
        self.assertEqual(task.state, orch.TaskState.FAILED)
        self.assertEqual(task.events[-1][3], 'E_ORCH_RUN_TIMEOUT')
        self.assertTrue(r.is_stopped())

    def test_orchestrator_full_loop_real_processes(self):
        """全链：submit→start（就绪门）→运行中死亡→健康检查→停止链→FAILED＋退净。"""
        specs = [spec('odo', ['odometry'], stop='none'),
                 spec('scan', ['local_planner']),
                 spec('mux', ['safety_arbiter'])]
        runners = make([(s, [PY, '-c', SLEEPER]) for s in specs],
                       mux={'ready_marker': 'READY'})
        self.addCleanupRunners(runners)
        engine = orch.Orchestrator()
        task = engine.submit('a', specs)
        engine.start(task, runners, ready_timeout=10)   # 就绪门＝READY 行
        self.assertEqual(task.state, orch.TaskState.RUNNING)
        runners['scan'].proc.terminate()                 # 注入：真进程死亡
        runners['scan'].reap()
        dead = engine.health_check(task, runners)
        self.assertEqual(dead, ['scan'])
        self.assertEqual(task.state, orch.TaskState.FAILED)
        for r in runners.values():
            r.reap()
        self.assertTrue(all(not r.is_alive() for r in runners.values()))

    def test_two_task_isolation_real(self):
        specs_a = [spec('a.odo', ['odometry'])]
        specs_b = [spec('b.odo', ['odometry'])]
        runners = make([(specs_a[0], [PY, '-c', SLEEPER]),
                        (specs_b[0], [PY, '-c', SLEEPER])])
        self.addCleanupRunners(runners)
        engine = orch.Orchestrator()
        ta = engine.submit('A', specs_a)
        tb = engine.submit('B', specs_b)
        engine.start(ta, {'a.odo': runners['a.odo']}, ready_timeout=10)
        engine.start(tb, {'b.odo': runners['b.odo']}, ready_timeout=10)
        runners['a.odo'].proc.terminate()
        runners['a.odo'].reap()
        engine.health_check(ta, {'a.odo': runners['a.odo']})
        self.assertEqual(ta.state, orch.TaskState.FAILED)
        engine.complete(tb, {'b.odo': runners['b.odo']})
        self.assertEqual(tb.state, orch.TaskState.COMPLETED)   # B 不受 A 影响
        self.assertFalse(runners['b.odo'].is_alive())


if __name__ == '__main__':
    unittest.main(verbosity=2)
