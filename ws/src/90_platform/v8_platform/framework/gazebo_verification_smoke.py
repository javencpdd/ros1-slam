#!/usr/bin/env python3
"""Real Gazebo verification backend smoke (F6): select, launch, truth channel.

Starts the declared backend outside the algorithm graph, waits for its truth
topic, runs it through verification.run, then stops and checks cleanup.
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
import launch as launch_mod  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402
import verification  # noqa: E402

OUT = Path('/tmp/r1006_smoke.json')
ROS_PORT = 11478
GAZEBO_PORT = 11479
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % ROS_PORT,
                  GAZEBO_MASTER_URI='http://127.0.0.1:%d' % GAZEBO_PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1006_ros',
                  ROS_LOG_DIR='/tmp/r1006_logs', PYTHONDONTWRITEBYTECODE='1',
                  GAZEBO_MODEL_DATABASE_URI='')
Path('/tmp/r1006_ros').mkdir(exist_ok=True)
Path('/tmp/r1006_logs').mkdir(exist_ok=True)


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
    graph = config.load_graph(HERE / 'fixtures' / 'graphs' / 'graph_2d.yaml')
    plan = render.render_plan(graph, catalog, graph_mod.check(graph, catalog))
    spec = verification.load_config(HERE / 'fixtures' / 'verification_gazebo.yaml')
    backends = verification.load_backends(HERE / 'fixtures' / 'verification_manifests')
    packages = launch_mod.load_index(HERE / 'fixtures' / 'package_index.json')
    checked = verification.check(spec, plan, catalog, backends, packages)
    algorithm_topics = {port['topic'] for comp in catalog['components'].values()
                        for port in comp['inputs'] + comp['outputs']
                        if port.get('topic')}
    result = {'checked': checked,
              'truth_in_algorithm_topics': spec['truth_channel'] in algorithm_topics,
              'backend_in_plan': spec['backend'] in
                                 {item['component'] for item in plan['instances']}}
    master = start(['roscore', '-p', str(ROS_PORT)], '/tmp/r1006_master.log')
    runner = None
    try:
        time.sleep(3)
        import rospy
        from nav_msgs.msg import Odometry
        rospy.init_node('r1006_smoke', anonymous=True, disable_signals=True)
        backend = backends['components'][spec['backend']]
        argv, marker = runtime.default_command(
            {'id': 'verify', 'params': {}, 'namespace': None}, backend)
        result['command'] = argv
        runs = []

        def run_backend(_spec):
            nonlocal runner
            component_spec = runtime.orch.ComponentSpec(
                'verify', ['simulator'], stop_method='process_group_signal')
            runner = runtime.ProcessRunner(component_spec, argv, ready_marker=marker)
            runner.start()
            started = time.monotonic()
            frames = [rospy.wait_for_message(spec['truth_channel'], Odometry,
                                             timeout=90)]
            for _ in range(2):
                frames.append(rospy.wait_for_message(spec['truth_channel'],
                                                     Odometry, timeout=10))
            elapsed = time.monotonic() - started
            runs.append({'frames': len(frames), 'elapsed_s': round(elapsed, 3),
                         'frame_id': frames[0].header.frame_id,
                         'child_frame_id': frames[0].child_frame_id,
                         'seq': [frame.header.seq for frame in frames]})
            return runs[-1]

        outcome = verification.run(spec, checked, run_backend)
        result['run'] = outcome
        result['command_returncode'] = runner.proc.poll() if runner else None
        runner.stop('process_group_signal')
        runner.reap()
        result['runner_stopped'] = runner.is_stopped()
        time.sleep(1.0)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        matched = [row for row in rows
                   if ('gzserver' in row[2] or 'gazebo_truth.py' in row[2])
                   and 'r1006_smoke' not in row[2]]
        result['zombies_after_stop'] = [row[2] for row in matched
                                        if row[1].startswith('Z')]
        leftovers = [row[2] for row in matched if not row[1].startswith('Z')]
        result['processes_after_stop'] = leftovers
        result['ok'] = (outcome['status'] == 'ok'
                        and runs and runs[0]['frames'] == 3
                        and runs[0]['frame_id'] == 'world'
                        and checked['algorithm_digest'] == plan['digest']
                        and not result['truth_in_algorithm_topics']
                        and not result['backend_in_plan']
                        and result['runner_stopped'] and not leftovers)
    finally:
        if runner is not None and runner.is_alive():
            runner.stop('process_group_signal')
            runner.reap()
        stop(master)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
