#!/usr/bin/env python3
"""Real ROS smoke: graph/Assembly with two map caches and one merger."""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
sys.path.insert(0, str(HERE.parent / 'representation'))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402
import ros_map_codec  # noqa: E402
import runtime  # noqa: E402

OUT = Path('/tmp/r1016_smoke.json')
PORT = 11494
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1016_ros',
                  ROS_LOG_DIR='/tmp/r1016_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1016_ros').mkdir(exist_ok=True)
Path('/tmp/r1016_logs').mkdir(exist_ok=True)


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
    graph = config.load_graph(HERE / 'fixtures' / 'graphs' / 'graph_map_cache_ros.yaml')
    plan = render.render_plan(graph, catalog, graph_mod.check(graph, catalog))
    result = {'startup_order': plan['instances'] and [item['id'] for item in plan['instances']]}
    master = start(['roscore', '-p', str(PORT)], '/tmp/r1016_master.log')
    assembly = None
    try:
        time.sleep(3)
        import rospy
        from std_srvs.srv import Trigger
        from v8_msgs.msg import MapDelta, SupportMap
        rospy.init_node('r1016_smoke', anonymous=True, disable_signals=True)
        assembly = runtime.Assembly(plan, catalog, ready_timeout=20)
        assembly.start()
        result['started'] = assembly.started
        merged_messages = []
        rospy.Subscriber('/v8/merge/delta', MapDelta,
                         lambda message: merged_messages.append(message), queue_size=10)
        time.sleep(1.5)
        value = json.loads((HERE.parent / 'contracts' / 'fixtures'
                            / 'map_ml_two_surfaces.json').read_text())
        support_map = ros_map_codec.support_map_msg(value)
        pub_a = rospy.Publisher('/v8/cache_a/base_map', SupportMap,
                                queue_size=1, latch=True)
        pub_b = rospy.Publisher('/v8/cache_b/base_map', SupportMap,
                                queue_size=1, latch=True)
        pub_a.publish(support_map)
        pub_b.publish(support_map)
        time.sleep(1.0)
        for name in ('/v8/cache_a/v8_map_cache/flush',
                     '/v8/cache_b/v8_map_cache/flush'):
            rospy.wait_for_service(name, timeout=15)
            response = rospy.ServiceProxy(name, Trigger)()
            result[name] = response.success
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not merged_messages:
            time.sleep(.1)
        merged = None if not merged_messages else \
            ros_map_codec.delta_value(merged_messages[-1])
        result['merged'] = None if merged is None else {
            'map_id': merged['map_id'], 'base_version': merged['base_version'],
            'surface_ids': sorted(op['surface']['surface_id'] for op in merged['ops']
                                  if op['op'] == 'upsert_surface'),
            'ops': len(merged['ops'])}
        assembly.stop()
        result['stopped'] = all(r.is_stopped() for r in assembly.runners.values())
        time.sleep(.5)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        leftover = [row[2] for row in rows
                    if ('map_cache_node.py' in row[2] or 'map_merge_node.py' in row[2])
                    and 'r1016_smoke' not in row[2] and not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        result['ok'] = (result['started']
                        and result.get('/v8/cache_a/v8_map_cache/flush') is True
                        and result.get('/v8/cache_b/v8_map_cache/flush') is True
                        and result['merged'] == {'map_id': 'svc_map', 'base_version': 1,
                                                 'surface_ids': ['ground', 'mezz'],
                                                 'ops': 2}
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
