#!/usr/bin/env python3
"""Real ROS smoke: hdl nodelet_group loads both hdl nodelets through a manager."""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
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
    stream = open(log, 'ab')
    return subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT,
                            start_new_session=True, env=dict(os.environ))


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def main():
    catalog = config.load_components(HERE / 'fixtures' / 'manifests')
    graph = config.load_graph(HERE / 'fixtures' / 'graphs' / 'graph_hdl_group.yaml')
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
