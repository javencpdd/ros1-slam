#!/usr/bin/env python3
"""A6 evidence: hdl + faster_lio against Gazebo truth, then a pose_mux switch."""
import copy
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = Path('/tmp/r1042_smoke.json')
PORT = 11527
GAZEBO_PORT = 11528
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  GAZEBO_MASTER_URI='http://127.0.0.1:%d' % GAZEBO_PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1042_ros',
                  ROS_LOG_DIR='/tmp/r1042_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1042_ros').mkdir(exist_ok=True)
Path('/tmp/r1042_logs').mkdir(exist_ok=True)
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
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, PointCloud2
    from std_msgs.msg import String
    catalog = config.load_components(HERE / 'fixtures' / 'manifests')
    catalog['components']['v8.fw.hdl_nodelet_group']['launch']['plugins'][0][
        'params']['globalmap_pcd'] = ('/repo/note/进度/v8/M5/26-10-27/evidence/'
                                      'R773/m1_world_synth_v4.pcd')
    graph = config.load_graph(HERE / 'fixtures' / 'graphs'
                              / 'graph_localization_compare.yaml')
    report = graph_mod.check(graph, catalog)
    plan = render.render_plan(graph, catalog, report)
    result = {'startup_order': report['startup_order']}
    master = subprocess.Popen(['roscore', '-p', str(PORT)],
                              stdout=open('/tmp/r1042_master.log', 'ab'),
                              stderr=subprocess.STDOUT, start_new_session=True,
                              env=dict(os.environ))
    gazebo = None
    assembly = runtime.Assembly(plan, catalog, ready_timeout=30)
    mux_runner = None
    running = threading.Event()
    try:
        time.sleep(3)
        gazebo = subprocess.Popen(['roslaunch',
                                   '/repo/ws/src/95_simulation/lite3_gazebo_sim/launch/m1_gazebo.launch',
                                   'gui:=false'], stdout=open('/tmp/r1042_gazebo.log', 'ab'),
                                  stderr=subprocess.STDOUT, start_new_session=True,
                                  env=dict(os.environ))
        rospy.init_node('r1042_smoke', anonymous=True, disable_signals=True)
        cloud = rospy.wait_for_message('/rslidar_points', PointCloud2, timeout=90)
        imu = rospy.wait_for_message('/livox/imu', Imu, timeout=90)
        cloud_pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=5)
        imu_pub = rospy.Publisher('/gpsimu_driver/imu_data', Imu, queue_size=20)
        leg_pub = rospy.Publisher('/leg_odom2', Odometry, queue_size=5)
        running.set()

        def feed_loop():
            while running.is_set() and not rospy.is_shutdown():
                fresh_cloud = copy.deepcopy(cloud)
                fresh_cloud.header.stamp = rospy.Time.now()
                cloud_pub.publish(fresh_cloud)
                fresh_imu = copy.deepcopy(imu)
                fresh_imu.header.stamp = rospy.Time.now()
                imu_pub.publish(fresh_imu)
                leg = Odometry()
                leg.header.frame_id, leg.header.stamp = 'map', rospy.Time.now()
                leg.child_frame_id = 'base_link'
                leg.pose.pose.orientation.w = 1.0
                leg_pub.publish(leg)
                time.sleep(.1)

        threading.Thread(target=feed_loop, daemon=True).start()
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import StaticTransformBroadcaster
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'base_link', 'rslidar'
        transform.transform.rotation.w = 1.0
        StaticTransformBroadcaster().sendTransform(transform)
        latest = {'hdl': None, 'lio': None, 'truth': None}
        hdl_samples, lio_samples, errors_hdl, errors_lio = [], [], [], []
        rospy.Subscriber('/odom', Odometry, lambda m: latest.update(hdl=m), queue_size=5)
        rospy.Subscriber('/Odometry', Odometry, lambda m: latest.update(lio=m), queue_size=5)

        def on_truth(message):
            names = list(message.name)
            index = names.index('lite3_planar') if 'lite3_planar' in names else 0
            latest['truth'] = message.pose[index].position
        rospy.Subscriber('/gazebo/model_states', ModelStates, on_truth, queue_size=2)
        assembly.start()
        result['started'] = assembly.started
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline and (latest['hdl'] is None or latest['lio'] is None):
            time.sleep(.1)
        result['outputs_present'] = {'hdl': latest['hdl'] is not None,
                                     'lio': latest['lio'] is not None}
        if latest['hdl'] is None or latest['lio'] is None:
            raise RuntimeError('missing localization output')
        compare_end = time.monotonic() + 3
        while time.monotonic() < compare_end:
            if latest['truth'] is not None:
                h = latest['hdl'].pose.pose.position
                l = latest['lio'].pose.pose.position
                t = latest['truth']
                errors_hdl.append(math.hypot(h.x - t.x, h.y - t.y))
                errors_lio.append(math.hypot(l.x - t.x, l.y - t.y))
            time.sleep(.1)
        result['hdl_frames'] = len(errors_hdl)
        result['lio_frames'] = len(errors_lio)
        if errors_hdl:
            result['hdl_error'] = {'mean': round(sum(errors_hdl) / len(errors_hdl), 4),
                                   'max': round(max(errors_hdl), 4),
                                   'last': round(errors_hdl[-1], 4)}
        if errors_lio:
            result['lio_error'] = {'mean': round(sum(errors_lio) / len(errors_lio), 4),
                                   'max': round(max(errors_lio), 4),
                                   'last': round(errors_lio[-1], 4)}
        # Pose mux switch between the two live backends (PoseStamped adapters).
        mux_catalog = config.load_components(HERE / 'fixtures' / 'manifests')
        mux_graph = config.load_graph(HERE / 'fixtures' / 'graphs' / 'graph_pose_mux.yaml')
        mux_report = graph_mod.check(mux_graph, mux_catalog)
        mux_plan = render.render_plan(mux_graph, mux_catalog, mux_report)
        mux_plan['instances'][0]['params'].update(max_gap_m=1.0, max_age_s=1.0)
        instance = mux_plan['instances'][0]
        command, marker = runtime.default_command(
            instance, mux_catalog['components'][instance['component']])
        mux_runner = runtime.ProcessRunner(runtime.build_specs(mux_plan, mux_catalog)[0],
                                          command, ready_marker=marker)
        mux_runner.start()
        pose_a = rospy.Publisher('/v8/local/pose_in_a', PoseStamped, queue_size=2)
        pose_b = rospy.Publisher('/v8/local/pose_in_b', PoseStamped, queue_size=2)
        select = rospy.Publisher('/v8/local/pose_select', String, queue_size=2)
        outputs, statuses = [], []
        rospy.Subscriber('/v8/local/pose', PoseStamped,
                         lambda m: outputs.append(time.monotonic()), queue_size=20)
        rospy.Subscriber('/v8/local/pose_mux/status', String,
                         lambda m: statuses.append(json.loads(m.data)), queue_size=20)

        def adapt(odom):
            message = PoseStamped()
            message.header.frame_id, message.header.stamp = 'map', rospy.Time.now()
            message.pose.position = odom.pose.pose.position
            message.pose.orientation.w = 1.
            return message

        def adapt_loop():
            while running.is_set() and not rospy.is_shutdown():
                if latest['hdl'] is not None:
                    pose_a.publish(adapt(latest['hdl']))
                if latest['lio'] is not None:
                    pose_b.publish(adapt(latest['lio']))
                time.sleep(.05)

        threading.Thread(target=adapt_loop, daemon=True).start()
        time.sleep(1.0)
        select.publish(String(data='a'))
        time.sleep(1.0)
        select.publish(String(data='b'))
        deadline = time.monotonic() + 3
        accepted = False
        while time.monotonic() < deadline:
            accepted = any(item.get('action') == 'select' and item.get('ok')
                           and item.get('active') == 'b' for item in statuses)
            if accepted:
                break
            time.sleep(.05)
        result['mux_switch_accepted'] = accepted
        result['mux_max_gap_s'] = round(max((b - a for a, b in zip(outputs, outputs[1:])),
                                            default=0.0), 4)
        result['mux_outputs'] = len(outputs)
    except Exception as error:
        import traceback
        result['error'] = repr(error)
        result['traceback'] = traceback.format_exc()
    finally:
        running.clear()
        if mux_runner is not None and not mux_runner.is_stopped():
            mux_runner.stop('process_group_signal')
            time.sleep(.3)
        if assembly.started:
            assembly.stop()
        if gazebo is not None:
            stop(gazebo)
        stop(master)
        time.sleep(.3)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        leftover = [row[2] for row in rows
                    if ('run_mapping_online' in row[2] or 'nodelet' in row[2]
                        or 'pose_mux_node.py' in row[2])
                    and 'r1042_smoke' not in row[2] and not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        result['ok'] = (result.get('started')
                        and result.get('hdl_frames', 0) > 0
                        and result.get('lio_frames', 0) > 0
                        and result.get('mux_switch_accepted')
                        and result.get('mux_max_gap_s', 1) < .25
                        and not leftover)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
