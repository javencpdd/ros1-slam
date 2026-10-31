#!/usr/bin/env python3
"""Real MuJoCo verification backend smoke: in-repo drop world truth channel."""
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

OUT = Path('/tmp/r1010_smoke.json')
PORT = 11483
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1010_ros',
                  ROS_LOG_DIR='/tmp/r1010_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1010_ros').mkdir(exist_ok=True)
Path('/tmp/r1010_logs').mkdir(exist_ok=True)


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
    spec = verification.load_config(HERE / 'fixtures' / 'verification_mujoco.yaml')
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
    master = start(['roscore', '-p', str(PORT)], '/tmp/r1010_master.log')
    runner = None
    try:
        time.sleep(3)
        import rospy
        from nav_msgs.msg import Odometry
        rospy.init_node('r1010_smoke', anonymous=True, disable_signals=True)
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
            frames = [rospy.wait_for_message(spec['truth_channel'], Odometry,
                                             timeout=30)]
            for _ in range(3):
                frames.append(rospy.wait_for_message(spec['truth_channel'],
                                                     Odometry, timeout=10))
            zs = [round(frame.pose.pose.position.z, 4) for frame in frames]
            runs.append({'frames': len(frames), 'z': zs,
                         'frame_id': frames[0].header.frame_id,
                         'child_frame_id': frames[0].child_frame_id})
            return runs[-1]

        outcome = verification.run(spec, checked, run_backend)
        result['run'] = outcome
        runner.stop('process_group_signal')
        runner.reap()
        result['runner_stopped'] = runner.is_stopped()
        time.sleep(.5)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        matched = [row for row in rows
                   if ('mujoco_truth.py' in row[2] or 'verification_mujoco.launch' in row[2])
                   and 'r1010_smoke' not in row[2]]
        leftover = [row[2] for row in matched if not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        zs = runs[0]['z'] if runs else []
        result['ok'] = (outcome['status'] == 'ok'
                        and len(zs) == 4 and zs[0] > 0.5
                        and zs[-1] < zs[0] - 0.1
                        and runs[0]['frame_id'] == 'world'
                        and runs[0]['child_frame_id'] == 'probe'
                        and not result['truth_in_algorithm_topics']
                        and not result['backend_in_plan']
                        and result['runner_stopped'] and not leftover)
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
