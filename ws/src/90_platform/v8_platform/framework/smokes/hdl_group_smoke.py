#!/usr/bin/env python3
"""Real ROS smoke: hdl nodelet_group loads both hdl nodelets through a manager."""
import sys as _sys
from pathlib import Path as _Path

_BASE = _Path(__file__).resolve().parent.parent
for _entry in ('', 'nodes', 'tools', 'smokes'):
    _sys.path.insert(0, str(_BASE / _entry) if _entry else str(_BASE))
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
FRAMEWORK = HERE.parent
sys.path.insert(0, str(FRAMEWORK))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402

OUT = Path('/tmp/r1017_smoke.json')
PORT = 11495
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1017_ros',
                  ROS_LOG_DIR='/tmp/r1017_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1017_ros').mkdir(exist_ok=True)
Path('/tmp/r1017_logs').mkdir(exist_ok=True)


def start(argv, log):
    # ★2026-11-18：stream 原先没人 close —— 每 start 一次泄漏一个文件句柄。
    # 长跑/反复 start-stop 会一直涨（本项目 ulimit -n 1024，gz 类进程已用百余个）。
    # 挂到 Popen 对象上，由 stop() 统一关闭；进程对象生命周期与句柄一致。
    stream = open(log, 'ab')
    proc = subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT,
                            start_new_session=True, env=dict(os.environ))
    proc._log_stream = stream
    return proc


def stop(process):
    try:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
    finally:
        # 关闭 start() 里打开的日志流；即便 kill 路径抛异常也要关
        stream = getattr(process, '_log_stream', None)
        if stream is not None and not stream.closed:
            stream.close()


def main():
    catalog = config.load_components(FRAMEWORK / 'fixtures' / 'manifests')
    graph = config.load_graph(FRAMEWORK / 'fixtures' / 'graphs' / 'graph_hdl_group.yaml')
    plan = render.render_plan(graph, catalog, graph_mod.check(graph, catalog))
    result = {'startup_order': [item['id'] for item in plan['instances']]}
    master = start(['roscore', '-p', str(PORT)], '/tmp/r1017_master.log')
    assembly = None
    try:
        time.sleep(3)
        import rospy
        from nodelet.srv import NodeletList
        rospy.init_node('r1017_smoke', anonymous=True, disable_signals=True)
        assembly = runtime.Assembly(plan, catalog, ready_timeout=30)
        assembly.start()
        result['started'] = assembly.started
        service = '/v8/hdl/hdl_manager/list'
        rospy.wait_for_service(service, timeout=45)
        response = rospy.ServiceProxy(service, NodeletList)()
        result['nodelets'] = sorted(response.nodelets)
        nodes = subprocess.check_output(['rosnode', 'list'], text=True,
                                        env=dict(os.environ)).split()
        result['nodes'] = sorted(nodes)
        assembly.stop()
        result['stopped'] = all(r.is_stopped() for r in assembly.runners.values())
        time.sleep(.5)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        leftover = [row[2] for row in rows
                    if ('/opt/ros/noetic/lib/nodelet/nodelet' in row[2]
                        or 'v8_nodelet_group_' in row[2])
                    and not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        result['ok'] = (result['started']
                        and result['nodelets'] == ['/v8/hdl/globalmap_server_nodelet',
                                                   '/v8/hdl/hdl_localization_nodelet']
                        and '/v8/hdl/hdl_manager' in result['nodes']
                        and result['stopped'] and not leftover)
    finally:
        if assembly is not None and assembly.started:
            assembly.stop()
        stop(master)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
