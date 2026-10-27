"""V8-06 编排、任务 API 与停止链（设计层，R646）。

交付面（23 号 V8-06 行）：启停/取消（任务状态机）、就绪与健康检查（**可观测条件轮询，
禁止固定等待冒充就绪**——A2 判据的设计层落点）、进程清理（停止链：逆角色序＋仲裁器最后＋
挂起升级 force）、仲裁分层（/cmd_vel 唯一写者最后停，保证拆除期间输出仍被闸住）。
资源预算校验（requires.resources）在此闭合 V8-05 留白。

边界：真实进程/ROS/容器运行面与活体故障注入按 25 号活体窗纪律归仿真阶段（V8-07+/V8-13），
本层以 SimulatedRunner 注入等价故障（死进程/永就绪超时/停止挂起）验证停止链逻辑。
消费 composer 锁文件（流程6 产物）构造组件规格。
"""
import enum
import time
from collections import defaultdict

READY_TIMEOUT = 1.0
READY_INTERVAL = 0.005
STOP_GRACE = 0.05

# 停止序：消费方先停、生产方后停；safety_arbiter 永远最后（拆除期间 /cmd_vel 仍被闸住）
ROLE_STOP_ORDER = ['controller', 'trajectory_validator', 'local_planner', 'global_planner',
                   'representation', 'map_server', 'mapping', 'global_localization',
                   'odometry', 'simulator', 'evaluator']
SAFETY_ROLE = 'safety_arbiter'


class OrchestratorError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


class TaskState(enum.Enum):
    PENDING = 'pending'
    STARTING = 'starting'
    RUNNING = 'running'
    CANCELLING = 'cancelling'
    COMPLETED = 'completed'
    CANCELLED = 'cancelled'
    FAILED = 'failed'


TRANSITIONS = {
    TaskState.PENDING: {TaskState.STARTING, TaskState.CANCELLED},
    TaskState.STARTING: {TaskState.RUNNING, TaskState.CANCELLING, TaskState.FAILED},
    TaskState.RUNNING: {TaskState.CANCELLING, TaskState.COMPLETED, TaskState.FAILED},
    TaskState.CANCELLING: {TaskState.CANCELLED, TaskState.FAILED},
}


class ComponentSpec:
    """从 manifest／锁文件派生的组件规格（编排只消费声明面，不 import ROS）。"""

    def __init__(self, component_id, roles, stop_method='none', resources=None):
        self.id = component_id
        self.roles = list(roles)
        self.stop_method = stop_method
        self.resources = list(resources or [])


class Task:
    def __init__(self, task_id, goal, specs):
        self.id = task_id
        self.goal = goal
        self.specs = specs
        self.state = TaskState.PENDING
        self.events = []  # (timestamp, from, to, cause) —— 事件留痕（P8 面）

    def transition(self, new_state, cause):
        if new_state not in TRANSITIONS.get(self.state, set()):
            raise OrchestratorError('E_ORCH_TRANSITION',
                                    '%s→%s 非法（当前 %s，因 %s）' % (self.state.value, new_state.value,
                                                                      self.state.value, cause))
        old = self.state
        self.state = new_state
        self.events.append((time.time(), old.value, new_state.value, cause))


def check_budget(specs, budget):
    """V8-05 留白闭合：requires.resources 对预算的冲突检测（fail-closed）。

    预算键＝resources 枚举名（cpu／gpu_optional／gpu_required）逐件计数；
    gpu_required 逐件占 1 个独占 GPU。预算不足报 E_ORCH_RESOURCE，组合层不得静默超卖。
    """
    demand = defaultdict(int)
    for spec in specs:
        for resource in spec.resources:
            demand[resource] += 1
    for resource, quota in budget.items():
        if demand.get(resource, 0) > quota:
            raise OrchestratorError('E_ORCH_RESOURCE',
                                    '%s 需求 %d 超预算 %d' % (resource, demand[resource], quota))
    return dict(demand)


def wait_ready(runners, timeout=READY_TIMEOUT, interval=READY_INTERVAL):
    """就绪＝可观测条件（runner.ready()），条件轮询＋超时拒绝。

    A2 判据「无固定等待冒充就绪」：永不因睡满一段时间而宣布就绪，只认条件成立；
    条件在超时内不成立＝E_ORCH_READY_TIMEOUT（进入停止链，不冒充 RUNNING）。
    """
    deadline = time.monotonic() + timeout
    while True:
        pending = [r.id for r in runners if not r.ready()]
        if not pending:
            return
        if time.monotonic() >= deadline:
            raise OrchestratorError('E_ORCH_READY_TIMEOUT', '未就绪: %s' % ','.join(sorted(pending)))
        time.sleep(interval)


def stop_order(specs):
    def rank(spec):
        if SAFETY_ROLE in spec.roles:
            return len(ROLE_STOP_ORDER) + 1  # 仲裁器最后
        for index, role in enumerate(ROLE_STOP_ORDER):
            if role in spec.roles:
                return index
        return len(ROLE_STOP_ORDER)  # 未知角色：仲裁器之前、其余之后

    return sorted(specs, key=rank)


def stop_chain(runners, specs, grace=STOP_GRACE):
    """按序停止＋挂起升级：declared stop_method（none→kill 兜底）；grace 内未死→force。

    返回逐件停止记录（id, method, outcome）；终态仍有活件＝E_ORCH_STOP_FAILED。
    """
    record = []
    for spec in stop_order(specs):
        runner = runners.get(spec.id)
        if runner is None:
            raise OrchestratorError('E_ORCH_NO_RUNNER', '组件 %s 无运行件' % spec.id)
        method = spec.stop_method if spec.stop_method != 'none' else 'kill'
        runner.stop(method)
        deadline = time.monotonic() + grace
        while runner.is_alive() and time.monotonic() < deadline:
            time.sleep(0.002)
        if runner.is_alive():
            runner.force()
            outcome = 'forced'
        else:
            outcome = 'stopped'
        record.append({'id': spec.id, 'method': method, 'outcome': outcome})
    alive = [r.id for r in runners.values() if r.is_alive()]
    if alive:
        raise OrchestratorError('E_ORCH_STOP_FAILED', '无法退净: %s' % ','.join(sorted(alive)))
    return record


class ComponentRunner:
    """运行面抽象：真实进程/ROS 适配器与 SimulatedRunner 实现同一接口。"""

    def start(self):
        raise NotImplementedError

    def stop(self, method):
        raise NotImplementedError

    def force(self):
        raise NotImplementedError

    def ready(self):
        raise NotImplementedError

    def is_alive(self):
        raise NotImplementedError


class SimulatedRunner(ComponentRunner):
    """故障注入载体：ready_polls（就绪前轮询次数）、hang_stop（停止挂起）、手动 die()。"""

    def __init__(self, spec, ready_polls=0, hang_stop=False):
        self.id = spec.id
        self.spec = spec
        self.ready_polls = ready_polls
        self.hang_stop = hang_stop
        self.alive = False
        self.poll_count = 0
        self.stops = []

    def start(self):
        self.alive = True
        self.poll_count = 0

    def ready(self):
        self.poll_count += 1
        return self.alive and self.poll_count > self.ready_polls

    def is_alive(self):
        return self.alive

    def die(self):
        self.alive = False  # 注入：进程死亡

    def stop(self, method):
        self.stops.append(method)
        if not self.hang_stop:
            self.alive = False

    def force(self):
        self.stops.append('force')
        self.alive = False


class Orchestrator:
    """任务 API：submit（预算 fail-closed）→ start（就绪门）→ 健康检查 → 取消/完成。"""

    def __init__(self, budget=None):
        self.budget = dict(budget or {})
        self.tasks = {}
        self._next = 0

    def submit(self, goal, specs):
        check_budget(specs, self.budget)
        self._next += 1
        task = Task('T%d' % self._next, goal, list(specs))
        self.tasks[task.id] = task
        return task

    def start(self, task, runners, ready_timeout=READY_TIMEOUT):
        task.transition(TaskState.STARTING, 'start')
        try:
            for runner in runners.values():
                runner.start()
            wait_ready(list(runners.values()), timeout=ready_timeout)
        except OrchestratorError as error:
            stop_chain(runners, task.specs)
            task.transition(TaskState.FAILED, error.code)
            raise
        task.transition(TaskState.RUNNING, 'ready')
        return task

    def health_check(self, task, runners):
        dead = sorted(r.id for r in runners.values() if not r.is_alive())
        if dead:
            stop_chain(runners, task.specs)
            task.transition(TaskState.FAILED, 'died:%s' % ','.join(dead))
        return dead

    def cancel(self, task, runners):
        task.transition(TaskState.CANCELLING, 'cancel')
        record = stop_chain(runners, task.specs)
        task.transition(TaskState.CANCELLED, 'stopped')
        return record

    def complete(self, task, runners):
        if TaskState.COMPLETED not in TRANSITIONS.get(task.state, set()):
            raise OrchestratorError('E_ORCH_TRANSITION',
                                    '%s 状态不得完成（当前 %s）' % (task.id, task.state.value))
        record = stop_chain(runners, task.specs)
        task.transition(TaskState.COMPLETED, 'done')
        return record
