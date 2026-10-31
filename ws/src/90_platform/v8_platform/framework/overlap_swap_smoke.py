#!/usr/bin/env python3
"""Real ROS smoke: zero-gap overlap swap of a read-only semantic provider.

Starts the old provider, starts the replacement alongside it, waits until the
new latched version is observed, then stops the old one and verifies the new
provider keeps serving.
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import hotswap  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402

OUT = Path('/tmp/r1009_smoke.json')
MAP_V1 = Path('/tmp/r1009_sem_v1.json')
MAP_V2 = Path('/tmp/r1009_sem_v2.json')
PORT = 11482
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1009_ros',
                  ROS_LOG_DIR='/tmp/r1009_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1009_ros').mkdir(exist_ok=True)
Path('/tmp/r1009_logs').mkdir(exist_ok=True)


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


def semantic_map(version, features):
    value = json.loads((HERE / 'fixtures' / 'maps' / 'sem_a.json').read_text())
    value['map_version'] = version
    value['features'] = features
    return value


def main():
    MAP_V1.write_text(json.dumps(semantic_map(1, [
        {'id': 'entry', 'kind': 'point',
         'geometry': {'type': 'point', 'point': [1., 2., 0.]}}])))
    MAP_V2.write_text(json.dumps(semantic_map(2, [
        {'id': 'entry', 'kind': 'point',
         'geometry': {'type': 'point', 'point': [1., 2., 0.]}},
        {'id': 'zone', 'kind': 'polygon',
         'geometry': {'type': 'polygon',
                      'points': [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.]]}}])))
    catalog = config.load_components(HERE / 'fixtures' / 'manifests')
    graph = config.load_graph(HERE / 'fixtures' / 'graphs' / 'graph_semantic.yaml')
    plan = render.render_plan(graph, catalog, graph_mod.check(graph, catalog))
    plan['instances'][0]['params'].update(
        {'semantic_files': str(MAP_V1), 'keep_alive_s': 60})
    record, new_plan = hotswap.prepare(
        plan, catalog, 'sem', 'v8.fw.semantic_provider',
        state={'map_version': 1}, epoch=0,
        params={'semantic_files': str(MAP_V2), 'keep_alive_s': 60})
    result = {}
    master = start(['roscore', '-p', str(PORT)], '/tmp/r1009_master.log')
    assembly = None
    try:
        time.sleep(3)
        import rospy
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        rospy.init_node('r1009_smoke', anonymous=True, disable_signals=True)
        assembly = runtime.Assembly(plan, catalog, ready_timeout=20)
        assembly.start()
        query = rospy.ServiceProxy('/v8/semantics/sem_a/query', Trigger)
        deadline = time.monotonic() + 20
        initial = None
        while time.monotonic() < deadline:
            try:
                response = query()
                if response.success:
                    initial = json.loads(response.message)
                    break
            except Exception:
                pass
            time.sleep(.2)
        result['initial_version'] = None if initial is None else initial['map_version']
        seen = {'version': 0}
        lock = threading.Lock()

        def on_map(message):
            version = json.loads(message.data)['map_version']
            with lock:
                seen['version'] = max(seen['version'], version)

        rospy.Subscriber('/v8/semantics/sem_a', String, on_map, queue_size=10)

        def ready():
            with lock:
                return seen['version'] >= 2

        old_runner = assembly.runners['sem']
        try:
            record, new_runner = hotswap.apply_overlap(
                assembly, catalog, record, new_plan, ready_timeout=20,
                ready_check=ready)
        except Exception as error:
            result['error'] = repr(error)
            result['preconditions'] = record.get('preconditions')
            result['old_runner_alive'] = old_runner.is_alive()
            result['seen_version'] = seen['version']
            result['ok'] = False
            return
        result['command'] = runtime.default_command(
            new_plan['instances'][0],
            catalog['components']['v8.fw.semantic_provider'])[0]
        result['preconditions'] = record['preconditions']
        result['postconditions'] = record['postconditions']
        result['old_runner_stopped'] = old_runner.is_stopped()
        result['new_runner_alive'] = new_runner.is_alive()
        result['new_runtime_name'] = new_plan['instances'][0]['runtime_name']
        result['state_file'] = new_runner.state_file
        result['state_file_ok'] = (Path(new_runner.state_file).is_file()
                                   and json.loads(Path(new_runner.state_file).read_text())
                                   == {'map_version': 1})
        failures = 0
        deadline = time.monotonic() + 10
        final = None
        while time.monotonic() < deadline:
            try:
                response = query()
                if response.success:
                    final = json.loads(response.message)
                    if final['map_version'] == 2:
                        break
            except Exception:
                failures += 1
            time.sleep(.2)
        result['query_failures_after_stop'] = failures
        result['final_version'] = None if final is None else final['map_version']
        result['final_features'] = None if final is None else len(final['features'])
        with lock:
            result['max_topic_version'] = seen['version']
        nodes = subprocess.check_output(['rosnode', 'list'], text=True,
                                        env=dict(os.environ)).split()
        result['nodes_after_swap'] = sorted(nodes)
        assembly.stop()
        result['stopped'] = all(r.is_stopped() for r in assembly.runners.values())
        result['ok'] = (result['initial_version'] == 1
                        and result['preconditions']['new_ready_before_stop']
                        and result['preconditions']['old_alive_before_stop']
                        and result['postconditions']['overlap']
                        and result['old_runner_stopped']
                        and result['new_runner_alive']
                        and result['state_file_ok']
                        and result['final_version'] == 2
                        and result['final_features'] == 2
                        and result['max_topic_version'] == 2
                        and '/sem_swap1' in result['nodes_after_swap']
                        and '/v8_semantic_map' not in result['nodes_after_swap']
                        and result['stopped'])
    finally:
        if assembly is not None and assembly.started:
            assembly.stop()
        stop(master)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
