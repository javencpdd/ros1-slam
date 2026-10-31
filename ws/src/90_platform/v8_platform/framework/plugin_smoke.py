#!/usr/bin/env python3
"""Real ROS smoke: plugin (nodelet standalone) through graph/preflight/Assembly.

Starts the graph_plugin_nodelet graph with the default command builder, checks
parameters and Float64 in->out, then stops and verifies the node is gone.
"""
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

OUT = Path('/tmp/r1005_smoke.json')
PORT = 11474
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1005_ros',
                  ROS_LOG_DIR='/tmp/r1005_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1005_ros').mkdir(exist_ok=True)
Path('/tmp/r1005_logs').mkdir(exist_ok=True)


def start(argv, log):
    stream = open(log, 'ab')
    return subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT,
                            start_new_session=True, env=dict(os.environ))


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def main():
    catalog = config.load_components(HERE / 'fixtures' / 'manifests')
    graph = config.load_graph(HERE / 'fixtures' / 'graphs' / 'graph_plugin_nodelet.yaml')
    report = graph_mod.check(graph, catalog)
    plan = render.render_plan(graph, catalog, report)
    result = {'startup_order': report['startup_order'], 'digest': report['digest']}
    master = start(['roscore', '-p', str(PORT)], '/tmp/r1005_master.log')
    assembly = None
    try:
        time.sleep(3)
        import rospy
        from std_msgs.msg import Float64
        rospy.init_node('r1005_smoke', anonymous=True, disable_signals=True)
        assembly = runtime.Assembly(plan, catalog, ready_timeout=20)
        assembly.start()
        result['started'] = assembly.started
        result['safety'] = runtime.safety.check_plan(plan, catalog)
        deadline = time.monotonic() + 15
        value_param = None
        while time.monotonic() < deadline:
            value_param = rospy.get_param('/v8_plugin_smoke/plus/value', None)
            if value_param == 2.5:
                break
            time.sleep(.2)
        result['value_param'] = value_param
        publisher = rospy.Publisher('/v8_plugin_smoke/plus/in', Float64,
                                    queue_size=1, latch=True)
        deadline = time.monotonic() + 20
        output = None
        while time.monotonic() < deadline:
            publisher.publish(Float64(data=3.0))
            try:
                output = rospy.wait_for_message('/v8_plugin_smoke/plus/out',
                                                Float64, timeout=2).data
                break
            except rospy.ROSException:
                continue
        result['input'] = 3.0
        result['output'] = output
        result['node_name'] = '/v8_plugin_smoke/plus' if output is not None else None
        assembly.stop()
        result['stopped'] = all(r.is_stopped() for r in assembly.runners.values())
        # SIGTERM does not run ROS shutdown handlers, so master registrations may
        # stay stale; assert on processes, not on getSystemState.
        time.sleep(1.0)
        ps = subprocess.check_output(['ps', '-eo', 'pid,cmd'], text=True)
        leftover = [line.strip() for line in ps.splitlines()
                    if 'nodelet_tutorial_math/Plus' in line and 'grep' not in line]
        result['processes_after_stop'] = leftover
        result['process_gone'] = not leftover
        result['ok'] = (result['started'] and result['safety']['command_owner'] is None
                        and result['value_param'] == 2.5
                        and output is not None and abs(output - 5.5) < 1e-9
                        and result['stopped'] and result['process_gone'])
    finally:
        if assembly is not None and assembly.started:
            assembly.stop()
        stop(master)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
