#!/usr/bin/env python3
"""Real ROS smoke: FASTER-LIO backend through graph/Assembly on Gazebo m1."""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = Path('/tmp/r1028_smoke.json')
PORT = 11515
GAZEBO_PORT = 11516
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  GAZEBO_MASTER_URI='http://127.0.0.1:%d' % GAZEBO_PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1028_ros',
                  ROS_LOG_DIR='/tmp/r1028_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1028_ros').mkdir(exist_ok=True)
Path('/tmp/r1028_logs').mkdir(exist_ok=True)
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402


def start(argv, log):
    return subprocess.Popen(argv, stdout=open(log, 'ab'), stderr=subprocess.STDOUT,
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
    graph = config.load_graph(HERE / 'fixtures' / 'graphs' / 'graph_faster_lio.yaml')
    report = graph_mod.check(graph, catalog)
    plan = render.render_plan(graph, catalog, report)
    result = {'startup_order': report['startup_order'],
              'command_writers': report['command_writers'],
              'inputs': {key: value['source'] for key, value in
                         report['bindings']['loc'].items()}}
    master = start(['roscore', '-p', str(PORT)], '/tmp/r1028_master.log')
    gazebo = None
    assembly = None
    try:
        time.sleep(3)
        gazebo = start(['roslaunch',
                        '/repo/ws/src/95_simulation/lite3_gazebo_sim/launch/m1_gazebo.launch',
                        'gui:=false'], '/tmp/r1028_gazebo.log')
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        rospy.init_node('r1028_smoke', anonymous=True, disable_signals=True)
        cloud = rospy.wait_for_message('/rslidar_points', PointCloud2, timeout=90)
        result['sensor'] = {'cloud_frame': cloud.header.frame_id,
                            'cloud_points': cloud.width * cloud.height}
        odom = []
        mapped = []
        rospy.Subscriber('/Odometry', Odometry, lambda message: odom.append(message),
                         queue_size=50)
        rospy.Subscriber('/cloud_registered', PointCloud2,
                         lambda message: mapped.append(message.header.stamp), queue_size=5)
        assembly = runtime.Assembly(plan, catalog, ready_timeout=30)
        assembly.start()
        result['started'] = assembly.started
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline and not odom:
            time.sleep(.2)
        start_count = len(odom)
        time.sleep(5)
        result['odom_frames'] = len(odom)
        result['new_odom_in_5s'] = len(odom) - start_count
        result['mapped_clouds'] = len(mapped)
        if odom:
            first, last = odom[0], odom[-1]
            result['odom'] = {
                'frame_id': last.header.frame_id,
                'child_frame_id': last.child_frame_id,
                'first': [round(value, 4) for value in
                          (first.pose.pose.position.x, first.pose.pose.position.y,
                           first.pose.pose.position.z)],
                'last': [round(value, 4) for value in
                         (last.pose.pose.position.x, last.pose.pose.position.y,
                          last.pose.pose.position.z)],
            }
            truth = rospy.wait_for_message('/gazebo/model_states', __import__(
                'gazebo_msgs.msg', fromlist=['ModelStates']).ModelStates, timeout=5)
            names = list(truth.name)
            index = names.index('lite3') if 'lite3' in names else 0
            model = truth.pose[index].position
            result['truth'] = [round(model.x, 4), round(model.y, 4), round(model.z, 4)]
            result['truth_minus_odom_xy'] = [
                round(model.x - last.pose.pose.position.x, 4),
                round(model.y - last.pose.pose.position.y, 4)]
        runner = assembly.runners.get('loc')
        result['alive_before_stop'] = bool(runner and runner.is_alive())
        assembly.stop()
        result['stopped'] = all(item.is_stopped() for item in assembly.runners.values())
        time.sleep(.5)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        leftover = [row[2] for row in rows
                    if 'run_mapping_online' in row[2]
                    and 'r1028_smoke' not in row[2] and not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        result['ok'] = (result['started'] and result.get('new_odom_in_5s', 0) > 0
                        and result.get('mapped_clouds', 0) > 0
                        and result['stopped'] and not leftover)
    except Exception as error:
        import traceback
        result['error'] = repr(error)
        result['traceback'] = traceback.format_exc()
    finally:
        if assembly is not None and assembly.started:
            assembly.stop()
        if gazebo is not None:
            stop(gazebo)
        stop(master)
        time.sleep(.3)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
