"""A2 运行面测试：真进程停止链——就绪门（条件式）、停止序、TERM 挂起升级 KILL、
双任务隔离、与 Orchestrator 全链（真 PID 真信号真退净）。"""
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
        self.assertTrue(r.ready())          # 条件式：看到 READY 行
        self.assertFalse(r.stops)

    def test_ready_timeout_rejected(self):
        s = spec('a', ['odometry'])
        r = ProcessRunner(s, [PY, '-c', "import time; time.sleep(3600)"],
                          ready_marker='READY', ready_timeout=1.0)
        r.start()
        self.addCleanup(r.reap)
        t0 = time.monotonic()
        self.assertFalse(r.ready())         # 不打印＝超时拒（无固定等待冒充）
        self.assertLess(time.monotonic() - t0, 3.0)

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
        self.assertTrue(r.ready())
        r.stop('process_group_signal')
        deadline = time.monotonic() + 1.5
        while r.is_alive() and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertTrue(r.is_alive())               # TERM 被挂起
        r.force()
        r.reap()
        self.assertFalse(r.is_alive())              # KILL 退净
        self.assertEqual(r.stops, ['process_group_signal', 'force'])

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
