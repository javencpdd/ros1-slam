#!/usr/bin/env python3
"""Real ROS smoke: semantic provider through graph/Assembly (no bare node)."""
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

OUT = Path('/tmp/r1008_smoke.json')
MAP = Path('/tmp/r1008_sem_a.json')
PORT = 11481
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1008_ros',
                  ROS_LOG_DIR='/tmp/r1008_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1008_ros').mkdir(exist_ok=True)
Path('/tmp/r1008_logs').mkdir(exist_ok=True)


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
    value = json.loads((HERE / 'fixtures' / 'maps' / 'sem_a.json').read_text())
    MAP.write_text(json.dumps(value))
    catalog = config.load_components(HERE / 'fixtures' / 'manifests')
    graph = config.load_graph(HERE / 'fixtures' / 'graphs' / 'graph_semantic.yaml')
    report = graph_mod.check(graph, catalog)
    plan = render.render_plan(graph, catalog, report)
    plan['instances'][0]['params']['semantic_files'] = str(MAP)
    plan['instances'][0]['params']['keep_alive_s'] = 40
    result = {'startup_order': report['startup_order'], 'digest': report['digest'],
              'command_owner': runtime.safety.check_plan(plan, catalog)['command_owner']}
    master = start(['roscore', '-p', str(PORT)], '/tmp/r1008_master.log')
    assembly = None
    try:
        time.sleep(3)
        import rospy
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        rospy.init_node('r1008_smoke', anonymous=True, disable_signals=True)
        assembly = runtime.Assembly(plan, catalog, ready_timeout=20)
        assembly.start()
        result['started'] = assembly.started
        query = rospy.ServiceProxy('/v8/semantics/sem_a/query', Trigger)
        queried = None
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            try:
                response = query()
                if response.success:
                    queried = json.loads(response.message)
                    break
            except Exception:
                pass
            time.sleep(.2)
        result['query'] = None if queried is None else {
            'map_id': queried['map_id'], 'map_version': queried['map_version'],
            'features': len(queried['features'])}
        published = json.loads(rospy.wait_for_message('/v8/semantics/sem_a', String,
                                                      timeout=10).data)
        result['topic'] = {'map_id': published['map_id'],
                           'map_version': published['map_version'],
                           'features': len(published['features'])}
        assembly.stop()
        result['stopped'] = all(r.is_stopped() for r in assembly.runners.values())
        time.sleep(1.0)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        leftovers = [line.strip() for line in ps.splitlines()[1:]
                     if 'semantic_map_node.py' in line and 'r1008_smoke' not in line
                     and not line.split()[1].startswith('Z')]
        result['processes_after_stop'] = leftovers
        result['ok'] = (result['started'] and result['command_owner'] is None
                        and result['query'] == {'map_id': 'sem_a', 'map_version': 1,
                                                'features': 1}
                        and result['topic']['map_version'] == 1
                        and result['stopped'] and not leftovers)
    finally:
        if assembly is not None and assembly.started:
            assembly.stop()
        stop(master)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
