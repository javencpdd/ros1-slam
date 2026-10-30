"""V8-06/V8-13 运行面 A2 cell：ProcessRunner——Orchestrator 停止链的真进程落地。

SimulatedRunner（R646）验证的是编排逻辑；本件把 ComponentRunner 落到**真实 OS 进程**：
- start：subprocess.Popen（新会话组 start_new_session=True，为组信号做准备）；
- stop(method)：service_stop／process_group_signal→对进程组发 SIGTERM；none→同（kill 兜底语义）；
- force：SIGKILL 组；挂起进程（忽略 TERM）由 grace 超时升级 force 杀净；
- ready()：**条件式**——子进程 stdout 打印 READY 行才算就绪（不打印＝超时拒，无固定等待冒充）；
- is_alive：poll()。
与 Orchestrator/stop_chain 同接口即插（A2 运行面：真 PID、真信号语义、真退净核验）。
"""
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import orchestrator as orch  # noqa: E402


def process_group_members(pgid):
    """Live non-zombie members, including descendants of an exited leader."""
    if pgid is None:
        return []
    members = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / 'stat').read_text().rpartition(')')[2].split()
            if int(fields[2]) == pgid and fields[0] != 'Z':
                members.append(int(entry.name))
        except (OSError, ValueError, IndexError):
            continue
    return sorted(members)


class ProcessRunner(orch.ComponentRunner):
    """真实进程运行件：cmd 为 argv 列表；ready_marker 为就绪行（缺省进程即绪）。

    ready_timeout 保留为旧调用兼容参数；实际就绪期限由 wait_ready 的 timeout
    一处控制，ready() 每次只做非阻塞观测。Linux /proc 用于核验同进程组后代。
    """

    def __init__(self, spec, cmd, ready_marker=None, ready_timeout=10.0, env=None):
        self.id = spec.id
        self.spec = spec
        self.cmd = list(cmd)
        self.ready_marker = ready_marker
        self.ready_timeout = ready_timeout
        self.env = None if env is None else dict(env)
        self.proc = None
        self._ready_line_seen = False
        self.stops = []
        self.pgid = None
        self._reader = None
        self._tracked_pids = set()
        self._tracked_pgids = set()

    def start(self):
        # Orchestrator.start() owns startup. Repeated calls for an already live runner
        # must not overwrite the Popen handle and strand the original process group.
        if self.proc is not None:
            if self.is_alive():
                return self
            raise orch.OrchestratorError('E_ORCH_RUNNER_RESTART',
                                         '%s 运行件已用过，请创建新实例' % self.id)
        self.proc = subprocess.Popen(
            self.cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            start_new_session=True, env=self.env)
        self.pgid = self.proc.pid
        self._tracked_pids, self._tracked_pgids = set(), set()
        if self.ready_marker is None:
            self._ready_line_seen = True
        self._reader = threading.Thread(target=self._drain_stdout, daemon=True)
        self._reader.start()
        return self

    def _drain_stdout(self):
        """持续读出，避免 READY 后日志塞满管道反过来挂住任务。

        ready() 本身不等待；外层 wait_ready 的墙钟截止是唯一就绪预算。
        半行或很长的日志都不让主线程阻塞，也不无界保存日志。
        """
        pending = b''
        marker = self.ready_marker.encode() if self.ready_marker else None
        try:
            while True:
                chunk = self.proc.stdout.read1(4096)
                if not chunk:
                    break
                pending += chunk
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    if marker and marker in line:
                        self._ready_line_seen = True
                pending = pending[-65536:]
        except (OSError, ValueError):
            pass  # reap() may close the stream once all group members have stopped.

    def ready(self):
        return self.is_alive() and self._ready_line_seen

    def is_alive(self):
        return self.proc is not None and self.proc.poll() is None

    def group_members(self):
        """Linux 运行面列出同组非僵尸 PID；leader 已死仍计入存活后代。"""
        return process_group_members(self.pgid)

    def is_stopped(self):
        leader_dead = not self.is_alive()  # poll also reaps the direct child
        if not leader_dead or self.group_members():
            return False
        for pid in self._tracked_pids:
            try:
                fields = (Path('/proc') / str(pid) / 'stat').read_text().rpartition(')')[2].split()
            except (OSError, ValueError, IndexError):
                continue
            if fields[0] != 'Z':
                return False
        return not any(process_group_members(pgid)
                       for pgid in self._tracked_pgids - {self.pgid})

    @staticmethod
    def _descendants(root_pid):
        """Snapshot descendants before signalling: roslaunch puts each node in
        its own session, so killing the roslaunch group alone strands the nodes.
        """
        table = {}
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                fields = (entry / 'stat').read_text().rpartition(')')[2].split()
                table[int(entry.name)] = (int(fields[1]), int(fields[2]))
            except (OSError, ValueError, IndexError):
                continue
        children = {}
        for pid, (ppid, _pgrp) in table.items():
            children.setdefault(ppid, []).append(pid)
        found, stack = [], [root_pid]
        while stack:
            for pid in children.get(stack.pop(), []):
                if pid in found:
                    continue
                found.append(pid)
                stack.append(pid)
        return found, {table[pid][1] for pid in found if pid in table}

    def _track_tree(self):
        if self.proc is None:
            return
        self._tracked_pids, self._tracked_pgids = self._descendants(self.proc.pid)

    def _signal_group(self, sig):
        if self.pgid is not None:
            try:
                os.killpg(self.pgid, sig)
            except ProcessLookupError:
                pass

    def _signal_tree(self, sig):
        self._signal_group(sig)
        for pgid in sorted(self._tracked_pgids - {self.pgid}):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                pass
        for pid in sorted(self._tracked_pids):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass

    def stop(self, method):
        self.stops.append(method)
        if method in ('service_stop', 'process_group_signal', 'none', 'kill'):
            # 组信号：service_stop 语义上应走服务调用，此处以 TERM 组等价实现（运行面
            # ROS 适配器按 manifest 声明接入；纯进程面 TERM 即「请求停止」）
            self._track_tree()
            self._signal_tree(signal.SIGTERM)

    def force(self):
        self.stops.append('force')
        if not self._tracked_pids:
            self._track_tree()
        self._signal_tree(signal.SIGKILL)

    def reap(self, timeout=5.0):
        """收尸：等退出并 join，防僵尸。"""
        if self.proc is None:
            return
        try:
            self.proc.wait(timeout=min(timeout, 0.05))
        except subprocess.TimeoutExpired:
            pass
        if not self.is_stopped():
            self.force()
        try:
            self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.force()
            self.proc.wait(timeout=5)
        deadline = time.monotonic() + timeout
        while not self.is_stopped() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not self.is_stopped():
            raise orch.OrchestratorError('E_ORCH_STOP_FAILED', '%s 后代未退净' % self.id)
        if self._reader:
            self._reader.join(timeout=1.0)
        if self._reader and self._reader.is_alive():
            # A detached node may still hold the pipe write end; closing a stream
            # an active reader is blocked on would deadlock reap indefinitely.
            return
        if self.proc.stdout:
            self.proc.stdout.close()
