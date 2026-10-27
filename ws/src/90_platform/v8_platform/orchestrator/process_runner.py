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
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import orchestrator as orch  # noqa: E402


class ProcessRunner(orch.ComponentRunner):
    """真实进程运行件：cmd 为 argv 列表；ready_marker 为就绪行（缺省进程即绪）。"""

    def __init__(self, spec, cmd, ready_marker=None, ready_timeout=10.0):
        self.id = spec.id
        self.spec = spec
        self.cmd = list(cmd)
        self.ready_marker = ready_marker
        self.ready_timeout = ready_timeout
        self.proc = None
        self._ready_line_seen = False
        self.stops = []

    def start(self):
        self.proc = subprocess.Popen(
            self.cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, start_new_session=True)
        if self.ready_marker is None:
            self._ready_line_seen = True
        return self

    def _drain_ready(self, deadline):
        """非阻塞读 stdout 找就绪行（条件式就绪门）。"""
        import select
        while time.monotonic() < deadline and not self._ready_line_seen:
            ready, _, _ = select.select([self.proc.stdout], [], [], 0.2)
            if ready:
                line = self.proc.stdout.readline()
                if not line:
                    break
                if self.ready_marker in line:
                    self._ready_line_seen = True
        return self._ready_line_seen

    def ready(self):
        if self.proc is None:
            return False
        if self._ready_line_seen:
            return True
        return self._drain_ready(time.monotonic() + self.ready_timeout)

    def is_alive(self):
        return self.proc is not None and self.proc.poll() is None

    def stop(self, method):
        self.stops.append(method)
        if not self.is_alive():
            return
        if method in ('service_stop', 'process_group_signal', 'none', 'kill'):
            # 组信号：service_stop 语义上应走服务调用，此处以 TERM 组等价实现（运行面
            # ROS 适配器按 manifest 声明接入；纯进程面 TERM 即「请求停止」）
            try:
                os.killpg(self.proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    def force(self):
        self.stops.append('force')
        if self.is_alive():
            try:
                os.killpg(self.proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def reap(self, timeout=5.0):
        """收尸：等退出并 join，防僵尸。"""
        if self.proc is None:
            return
        try:
            self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.force()
            self.proc.wait(timeout=5)
        if self.proc.stdout:
            self.proc.stdout.close()
