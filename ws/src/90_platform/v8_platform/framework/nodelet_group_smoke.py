#!/usr/bin/env python3
"""Real ROS smoke: nodelet_group (external manager + loader) through Assembly."""
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

OUT = Path('/tmp/r1011_smoke.json')
PORT = 11488
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1011_ros',
                  ROS_LOG_DIR='/tmp/r1011_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1011_ros').mkdir(exist_ok=True)
Path('/tmp/r1011_logs').mkdir(exist_ok=True)


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
    graph = config.load_graph(HERE / 'fixtures' / 'graphs' / 'graph_nodelet_group.yaml')
    plan = render.render_plan(graph, catalog, graph_mod.check(graph, catalog))
    result = {'startup_order': plan['instances'][0]['id'],
              'command': runtime.default_command(
                  plan['instances'][0],
                  catalog['components']['v8.fw.nodelet_group_plus'])[0]}
    master = start(['roscore', '-p', str(PORT)], '/tmp/r1011_master.log')
    assembly = None
    try:
        time.sleep(3)
        import rospy
        from std_msgs.msg import Float64
        rospy.init_node('r1011_smoke', anonymous=True, disable_signals=True)
        assembly = runtime.Assembly(plan, catalog, ready_timeout=20)
        assembly.start()
        result['started'] = assembly.started
        deadline = time.monotonic() + 15
        value_param = None
        while time.monotonic() < deadline:
            value_param = rospy.get_param('/v8_group/plus/value', None)
            if value_param == 2.5:
                break
            time.sleep(.2)
        result['value_param'] = value_param
        publisher = rospy.Publisher('/v8_group/plus/in', Float64,
                                    queue_size=1, latch=True)
        deadline = time.monotonic() + 30
        output = None
        while time.monotonic() < deadline:
            publisher.publish(Float64(data=3.0))
            try:
                output = rospy.wait_for_message('/v8_group/plus/out', Float64,
                                                timeout=2).data
                break
            except rospy.ROSException:
                continue
        result['output'] = output
        nodes = subprocess.check_output(['rosnode', 'list'], text=True,
                                        env=dict(os.environ)).split()
        result['nodes'] = sorted(nodes)
        assembly.stop()
        result['stopped'] = all(r.is_stopped() for r in assembly.runners.values())
        time.sleep(1.0)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        matched = [row for row in rows
                   if ('/opt/ros/noetic/lib/nodelet/nodelet' in row[2]
                       or 'v8_nodelet_group_' in row[2])
                   and 'v8_group_mgr' in row[2]]
        leftover = [row[2] for row in matched if not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        result['ok'] = (result['started'] and result['value_param'] == 2.5
                        and output is not None and abs(output - 5.5) < 1e-9
                        and '/v8_group/v8_group_mgr' in result['nodes']
                        and '/v8_group/plus' in result['nodes']
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
