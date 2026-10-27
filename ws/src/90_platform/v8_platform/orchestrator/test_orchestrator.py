"""V8-06 编排与停止链测试：A2 三判据的设计层证据＋故障注入（SimulatedRunner）。

A2 映射：
  无固定等待冒充就绪 → wait_ready 只认条件（就绪即返、不睡满），永就绪＝超时进停止链；
  失效可停车         → 运行中死亡被健康检查捕获，停止链退净，任务 FAILED；
  其他任务不受影响   → 双任务隔离：A 的停止链不碰 B 的 runner，B 正常完成。
"""
import sys
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import orchestrator as orch  # noqa: E402


def spec(cid, roles, stop='none', resources=None):
    return orch.ComponentSpec(cid, roles, stop_method=stop, resources=resources)


def make(specs, **per_id):
    return {s.id: orch.SimulatedRunner(s, **per_id.get(s.id, {})) for s in specs}


STACK = [spec('odo', ['odometry']), spec('loc', ['global_localization']),
         spec('costmap', ['representation']), spec('scan', ['local_planner'], stop='none'),
         spec('dwa', ['controller'], stop='process_group_signal'),
         spec('mux', ['safety_arbiter'], stop='process_group_signal')]


class BudgetTests(unittest.TestCase):
    def test_pass_and_demand(self):
        specs = [spec('a', ['odometry'], resources=['cpu']),
                 spec('b', ['mapping'], resources=['cpu', 'gpu_optional'])]
        self.assertEqual(orch.check_budget(specs, {'cpu': 4, 'gpu': 1}),
                         {'cpu': 2, 'gpu_optional': 1})

    def test_gpu_conflict(self):
        specs = [spec('a', ['odometry'], resources=['gpu_required']),
                 spec('b', ['mapping'], resources=['gpu_required'])]
        with self.assertRaises(orch.OrchestratorError) as caught:
            orch.check_budget(specs, {'gpu_required': 1})
        self.assertEqual(caught.exception.code, 'E_ORCH_RESOURCE')

    def test_submit_fail_closed(self):
        engine = orch.Orchestrator(budget={'cpu': 1})
        with self.assertRaises(orch.OrchestratorError) as caught:
            engine.submit('g', [spec('a', ['odometry'], resources=['cpu']),
                                spec('b', ['mapping'], resources=['cpu'])])
        self.assertEqual(caught.exception.code, 'E_ORCH_RESOURCE')
        self.assertEqual(engine.tasks, {})  # 预算不过不入队


class LifecycleTests(unittest.TestCase):
    def test_happy_path_and_stop_order(self):
        engine = orch.Orchestrator()
        task = engine.submit('patrol', STACK)
        runners = make(STACK)
        engine.start(task, runners)
        self.assertEqual(task.state, orch.TaskState.RUNNING)
        record = engine.complete(task, runners)
        self.assertEqual(task.state, orch.TaskState.COMPLETED)
        # 停止序：dwa(controller) 最先，mux(safety_arbiter) 最后；全退净
        self.assertEqual(record[0]['id'], 'dwa')
        self.assertEqual(record[-1]['id'], 'mux')
        self.assertTrue(all(r['outcome'] == 'stopped' for r in record))
        self.assertEqual([e[2] for e in task.events],
                         ['starting', 'running', 'completed'])  # 事件留痕（状态值）

    def test_readiness_by_condition_not_fixed_wait(self):
        engine = orch.Orchestrator()
        task = engine.submit('g', STACK)
        runners = make(STACK, scan={'ready_polls': 3})  # scan 第 4 次轮询才就绪
        t0 = time.monotonic()
        engine.start(task, runners, ready_timeout=5.0)
        elapsed = time.monotonic() - t0
        self.assertLess(elapsed, 1.0)  # 条件成立即返，不睡满 5 s

    def test_ready_timeout_stops_chain(self):
        engine = orch.Orchestrator()
        task = engine.submit('g', STACK)
        runners = make(STACK, scan={'ready_polls': 10 ** 9})  # 永不就绪
        with self.assertRaises(orch.OrchestratorError) as caught:
            engine.start(task, runners, ready_timeout=0.1)
        self.assertEqual(caught.exception.code, 'E_ORCH_READY_TIMEOUT')
        self.assertEqual(task.state, orch.TaskState.FAILED)
        self.assertTrue(all(not r.is_alive() for r in runners.values()))  # 退净

    def test_die_in_running_detected_and_stopped(self):
        engine = orch.Orchestrator()
        task = engine.submit('g', STACK)
        runners = make(STACK)
        engine.start(task, runners)
        runners['scan'].die()  # 注入：局部规划进程死亡
        dead = engine.health_check(task, runners)
        self.assertEqual(dead, ['scan'])
        self.assertEqual(task.state, orch.TaskState.FAILED)
        self.assertTrue(all(not r.is_alive() for r in runners.values()))
        # 事件含失败原因
        self.assertTrue(any('died:scan' in e[3] for e in task.events))

    def test_cancel_stops_cleanly(self):
        engine = orch.Orchestrator()
        task = engine.submit('g', STACK)
        runners = make(STACK)
        engine.start(task, runners)
        engine.cancel(task, runners)
        self.assertEqual(task.state, orch.TaskState.CANCELLED)
        self.assertTrue(all(not r.is_alive() for r in runners.values()))

    def test_stop_hang_escalates_to_force(self):
        engine = orch.Orchestrator()
        task = engine.submit('g', STACK)
        runners = make(STACK, mux={'hang_stop': True})  # 仲裁器停止挂起
        engine.start(task, runners)
        record = engine.complete(task, runners)
        mux_record = next(r for r in record if r['id'] == 'mux')
        self.assertEqual(mux_record['outcome'], 'forced')
        self.assertTrue(all(not r.is_alive() for r in runners.values()))

    def test_invalid_transition_guarded(self):
        engine = orch.Orchestrator()
        task = engine.submit('g', STACK)
        runners = make(STACK, scan={'ready_polls': 10 ** 9})
        with self.assertRaises(orch.OrchestratorError):
            engine.start(task, runners, ready_timeout=0.05)
        with self.assertRaises(orch.OrchestratorError) as caught:
            engine.complete(task, runners)  # FAILED 不得再 COMPLETED
        self.assertEqual(caught.exception.code, 'E_ORCH_TRANSITION')

    def test_declared_stop_method_used(self):
        engine = orch.Orchestrator()
        task = engine.submit('g', STACK)
        runners = make(STACK)
        engine.start(task, runners)
        engine.complete(task, runners)
        self.assertEqual(runners['dwa'].stops, ['process_group_signal'])  # 声明式停止方式
        self.assertEqual(runners['scan'].stops, ['kill'])  # none→kill 兜底

    def test_task_isolation(self):
        """A2 第三判据：A 失败退净不影响 B。"""
        engine = orch.Orchestrator()
        task_a = engine.submit('a', [spec('a.odo', ['odometry']), spec('a.scan', ['local_planner'])])
        task_b = engine.submit('b', [spec('b.odo', ['odometry']), spec('b.scan', ['local_planner'])])
        runners_a = make(task_a.specs)
        runners_b = make(task_b.specs)
        engine.start(task_a, runners_a)
        engine.start(task_b, runners_b)
        runners_a['a.scan'].die()
        engine.health_check(task_a, runners_a)
        self.assertEqual(task_a.state, orch.TaskState.FAILED)
        # B 完全不受影响：仍活、可正常完成
        self.assertTrue(all(r.is_alive() for r in runners_b.values()))
        engine.complete(task_b, runners_b)
        self.assertEqual(task_b.state, orch.TaskState.COMPLETED)

    def test_lock_integration(self):
        """composer 锁文件 → 组件规格 → 编排全流程（消费流程6 产物）。"""
        sys.path.insert(0, str(HERE.parent / 'composer'))
        import composer as comp
        store = comp.load(HERE.parent / 'registry' / 'data' / 'manifests')
        ids = ['v8.odometry.faster_lio', 'v8.global_localization.lite3',
               'v8.representation.costmap_layers', 'v8.local_planner.scan_planner',
               'v8.controller.dwa_adapter', 'v8.safety_arbiter.lite3']
        lock = comp.lock(store, ids)
        specs = []
        for cid in ids:
            raw = store._composer_raws[cid]
            specs.append(orch.ComponentSpec(
                cid, raw['roles'], stop_method=raw['ownership']['stop'],
                resources=raw.get('requires', {}).get('resources', [])))
        engine = orch.Orchestrator(budget={'cpu': 8, 'gpu_required': 0})
        task = engine.submit('real-stack', specs)
        runners = {s.id: orch.SimulatedRunner(s) for s in specs}
        engine.start(task, runners)
        record = engine.complete(task, runners)
        self.assertEqual(record[0]['id'], 'v8.controller.dwa_adapter')
        self.assertEqual(record[-1]['id'], 'v8.safety_arbiter.lite3')
        self.assertEqual(task.state, orch.TaskState.COMPLETED)


if __name__ == '__main__':
    unittest.main(verbosity=2)
