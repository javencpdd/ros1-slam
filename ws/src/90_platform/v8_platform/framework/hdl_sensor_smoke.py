#!/usr/bin/env python3
"""Real ROS smoke: hdl nodelets fed by Gazebo m1 points/IMU, watch /odom."""
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

OUT = Path('/tmp/r1018_smoke.json')
PORT = 11496
GAZEBO_PORT = 11497
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  GAZEBO_MASTER_URI='http://127.0.0.1:%d' % GAZEBO_PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1018_ros',
                  ROS_LOG_DIR='/tmp/r1018_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1018_ros').mkdir(exist_ok=True)
Path('/tmp/r1018_logs').mkdir(exist_ok=True)


def start(argv, log):
    stream = open(log, 'ab')
    return subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT,
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
    graph = config.load_graph(HERE / 'fixtures' / 'graphs' / 'graph_hdl_group.yaml')
    plan = render.render_plan(graph, catalog, graph_mod.check(graph, catalog))
    result = {'startup_order': [item['id'] for item in plan['instances']]}
    master = start(['roscore', '-p', str(PORT)], '/tmp/r1018_master.log')
    gazebo = None
    assembly = None
    try:
        time.sleep(3)
        gazebo = start(['roslaunch',
                        '/repo/ws/src/95_simulation/lite3_gazebo_sim/launch/m1_gazebo.launch',
                        'gui:=false'], '/tmp/r1018_gazebo.log')
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, PointCloud2
        from nodelet.srv import NodeletList
        rospy.init_node('r1018_smoke', anonymous=True, disable_signals=True)
        cloud = rospy.wait_for_message('/rslidar_points', PointCloud2, timeout=90)
        imu = rospy.wait_for_message('/livox/imu', Imu, timeout=90)
        result['sensor'] = {'cloud_points': len(cloud.data), 'imu_frame': imu.header.frame_id}
        cloud_pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=5)
        imu_pub = rospy.Publisher('/gpsimu_driver/imu_data', Imu, queue_size=20)
        counts = {'cloud': 0, 'imu': 0}

        def on_cloud(message):
            counts['cloud'] += 1
            cloud_pub.publish(message)

        def on_imu(message):
            counts['imu'] += 1
            imu_pub.publish(message)

        rospy.Subscriber('/rslidar_points', PointCloud2, on_cloud, queue_size=5)
        rospy.Subscriber('/livox/imu', Imu, on_imu, queue_size=20)
        assembly = runtime.Assembly(plan, catalog, ready_timeout=30)
        assembly.start()
        result['started'] = assembly.started
        rospy.wait_for_service('/v8/hdl/hdl_manager/list', timeout=45)
        result['nodelets'] = sorted(
            rospy.ServiceProxy('/v8/hdl/hdl_manager/list', NodeletList)().nodelets)
        odom_messages = []
        rospy.Subscriber('/odom', Odometry,
                         lambda message: odom_messages.append(message), queue_size=20)
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline and not odom_messages:
            time.sleep(.2)
        result['counts'] = dict(counts)
        result['odom_frames'] = len(odom_messages)
        if odom_messages:
            message = odom_messages[-1]
            result['odom'] = {'frame_id': message.header.frame_id,
                              'child_frame_id': message.child_frame_id,
                              'position': [round(value, 4) for value in
                                           (message.pose.pose.position.x,
                                            message.pose.pose.position.y,
                                            message.pose.pose.position.z)]}
        assembly.stop()
        result['stopped'] = all(r.is_stopped() for r in assembly.runners.values())
        result['ok'] = (result['started'] and result['nodelets']
                        and result['counts']['cloud'] > 0
                        and result['counts']['imu'] > 0
                        and result['odom_frames'] > 0
                        and result['stopped'])
    finally:
        if assembly is not None and assembly.started:
            assembly.stop()
        if gazebo is not None:
            stop(gazebo)
        stop(master)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
