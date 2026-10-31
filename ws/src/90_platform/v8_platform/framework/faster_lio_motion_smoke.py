#!/usr/bin/env python3
"""Real ROS smoke: FASTER-LIO odometry vs Gazebo truth while the m1 model moves.

Truth is used only for scoring after the run; it is never fed to the estimator.
"""
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = Path('/tmp/r1038_smoke.json')
PORT = 11524
GAZEBO_PORT = 11525
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  GAZEBO_MASTER_URI='http://127.0.0.1:%d' % GAZEBO_PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1038_ros',
                  ROS_LOG_DIR='/tmp/r1038_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1038_ros').mkdir(exist_ok=True)
Path('/tmp/r1038_logs').mkdir(exist_ok=True)
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def main():
    import rospy
    from gazebo_msgs.msg import ModelStates
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    catalog = config.load_components(HERE / 'fixtures' / 'manifests')
    graph = config.load_graph(HERE / 'fixtures' / 'graphs' / 'graph_faster_lio.yaml')
    report = graph_mod.check(graph, catalog)
    plan = render.render_plan(graph, catalog, report)
    result = {'startup_order': report['startup_order']}
    master = subprocess.Popen(['roscore', '-p', str(PORT)],
                              stdout=open('/tmp/r1038_master.log', 'ab'),
                              stderr=subprocess.STDOUT, start_new_session=True,
                              env=dict(os.environ))
    gazebo = None
    assembly = runtime.Assembly(plan, catalog, ready_timeout=30)
    try:
        time.sleep(3)
        gazebo = subprocess.Popen(['roslaunch',
                                   '/repo/ws/src/95_simulation/lite3_gazebo_sim/launch/m1_gazebo.launch',
                                   'gui:=false'], stdout=open('/tmp/r1038_gazebo.log', 'ab'),
                                  stderr=subprocess.STDOUT, start_new_session=True,
                                  env=dict(os.environ))
        rospy.init_node('r1038_smoke', anonymous=True, disable_signals=True)
        rospy.wait_for_message('/rslidar_points', PointCloud2, timeout=90)
        latest = {'odom': None, 'truth': None}
        rospy.Subscriber('/Odometry', Odometry,
                         lambda message: latest.update(odom=message), queue_size=5)

        def on_truth(message):
            names = list(message.name)
            if 'lite3_planar' in names:
                index = names.index('lite3_planar')
            else:
                index = next((i for i, name in enumerate(names) if 'lite3' in name), 0)
            latest['truth'] = message.pose[index].position
        rospy.Subscriber('/gazebo/model_states', ModelStates, on_truth, queue_size=2)
        assembly.start()
        result['started'] = assembly.started
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and latest['odom'] is None:
            time.sleep(.1)
        if latest['odom'] is None:
            raise RuntimeError('no odometry before motion')
        cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=2)

        def command(vx, wz):
            message = Twist()
            message.linear.x, message.angular.z = vx, wz
            cmd.publish(message)

        samples = []

        def sample(duration, vx, wz):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                command(vx, wz)
                if latest['odom'] is not None and latest['truth'] is not None:
                    p = latest['odom'].pose.pose.position
                    t = latest['truth']
                    samples.append((p.x, p.y, t.x, t.y))
                time.sleep(.1)
            command(0., 0.)
            time.sleep(.5)

        sample(3.0, .2, 0.)
        sample(2.0, 0., .3)
        sample(2.0, .2, 0.)
        result['samples'] = len(samples)
        if samples:
            ox, oy, tx, ty = samples[0]
            ox0, oy0, tx0, ty0 = ox, oy, tx, ty
            errors = []
            for ox, oy, tx, ty in samples:
                errors.append(math.hypot((ox - ox0) - (tx - tx0),
                                         (oy - oy0) - (ty - ty0)))
            result['truth_displacement'] = round(math.hypot(tx - tx0, ty - ty0), 4)
            result['odom_displacement'] = round(math.hypot(ox - ox0, oy - oy0), 4)
            result['mean_error_m'] = round(sum(errors) / len(errors), 4)
            result['max_error_m'] = round(max(errors), 4)
            result['final_error_m'] = round(errors[-1], 4)
    except Exception as error:
        import traceback
        result['error'] = repr(error)
        result['traceback'] = traceback.format_exc()
    finally:
        if assembly.started:
            assembly.stop()
        if gazebo is not None:
            stop(gazebo)
        stop(master)
        time.sleep(.3)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        leftover = [row[2] for row in rows
                    if 'run_mapping_online' in row[2]
                    and 'r1038_smoke' not in row[2] and not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        result['ok'] = (result.get('started')
                        and result.get('samples', 0) > 50
                        and result.get('truth_displacement', 0) > .3
                        and result.get('mean_error_m', 99) < .5
                        and result.get('max_error_m', 99) < 1.0
                        and not leftover)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
