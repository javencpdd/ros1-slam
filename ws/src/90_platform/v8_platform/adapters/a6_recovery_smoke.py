#!/usr/bin/env python3
"""Live ROS wiring smoke for the A6 recovery supervisor (fake relocalize/candidate)."""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
PORT = 11590
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT, ROS_IP='127.0.0.1',
                  ROS_HOME='/tmp/a6_recovery_ros', ROS_LOG_DIR='/tmp/a6_recovery_logs',
                  PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/a6_recovery_ros').mkdir(exist_ok=True)
Path('/tmp/a6_recovery_logs').mkdir(exist_ok=True)


def main():
    import rospy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String, UInt64
    from std_srvs.srv import Empty, EmptyResponse
    master = subprocess.Popen(['roscore', '-p', str(PORT)], start_new_session=True,
                              stdout=open('/tmp/a6_recovery_master.log', 'ab'),
                              stderr=subprocess.STDOUT, env=dict(os.environ))
    node = None
    try:
        time.sleep(3)
        rospy.init_node('a6_recovery_smoke', anonymous=True, disable_signals=True)
        relocalize_calls = [0]

        def relocalize(_request):
            relocalize_calls[0] += 1
            return EmptyResponse()

        rospy.Service('/relocalize', Empty, relocalize)
        odom_pub = rospy.Publisher('/Odometry', Odometry, queue_size=20)
        candidate_pub = rospy.Publisher('/a6/recovery/candidate', String, queue_size=5)
        lease_pub = rospy.Publisher('/a6/recovery/lease_in', UInt64, queue_size=5)
        latest = {'status': None, 'allowed': None, 'lease': None, 'backend': None}
        rospy.Subscriber('/a6/recovery/status', String,
                         lambda m: latest.update(status=json.loads(m.data)), queue_size=20)
        rospy.Subscriber('/a6/recovery/command_allowed', Bool,
                         lambda m: latest.update(allowed=m.data), queue_size=20)
        rospy.Subscriber('/a6/recovery/lease', UInt64,
                         lambda m: latest.update(lease=m.data), queue_size=5)
        rospy.Subscriber('/a6/recovery/backend', String,
                         lambda m: latest.update(backend=json.loads(m.data)), queue_size=5)
        node = subprocess.Popen([sys.executable, '-B', str(HERE / 'a6_recovery_node.py'),
                                 '--odom-topic', '/Odometry',
                                 '--candidate-topic', '/a6/recovery/candidate',
                                 '--lease-in-topic', '/a6/recovery/lease_in'],
                                start_new_session=True,
                                stdout=open('/tmp/a6_recovery_node.log', 'ab'),
                                stderr=subprocess.STDOUT, env=dict(os.environ))
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and latest['status'] is None:
            time.sleep(.05)
        if latest['status'] is None:
            raise RuntimeError('recovery node status missing')

        def publish_odom(x, y, stamp):
            message = Odometry()
            message.header.frame_id, message.header.stamp = 'map', rospy.Time.from_sec(stamp)
            message.pose.pose.position.x, message.pose.pose.position.y = x, y
            message.pose.pose.orientation.w = 1.
            message.pose.covariance[0] = message.pose.covariance[7] = 0.01
            odom_pub.publish(message)

        stamp = 100.0
        for _ in range(12):
            publish_odom(0., 0., stamp); stamp += .05; time.sleep(.05)
        lease_pub.publish(UInt64(data=1))
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and not latest.get('allowed'):
            publish_odom(0., 0., stamp); stamp += .05; time.sleep(.05)
        result = {'initial_tracking': bool(latest.get('allowed')),
                  'initial_lease': latest.get('lease')}
        # stop odometry -> stale -> relocalize request
        time.sleep(.5)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and (latest.get('status') or {}).get('phase') != 'RELOCALIZING':
            time.sleep(.05)
        result['relocalizing'] = (latest.get('status') or {}).get('phase') == 'RELOCALIZING'
        result['relocalize_calls_before'] = relocalize_calls[0]
        # reject a low-score candidate, then accept a valid one
        candidate_pub.publish(String(data=json.dumps({'pose': [1., 2., 0.], 'score': 0.1})))
        time.sleep(.3)
        candidate_pub.publish(String(data=json.dumps({'pose': [1., 2., 0.], 'score': 0.9})))
        time.sleep(.3)
        result['backend_switch'] = latest.get('backend')
        # backend recovers; healthy samples at the new pose -> new lease
        stamp += 1.0
        for _ in range(24):
            publish_odom(1., 2., stamp); stamp += .05; time.sleep(.05)
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline and not latest.get('allowed'):
            publish_odom(1., 2., stamp); stamp += .05; time.sleep(.05)
        result.update({'recovered': bool(latest.get('allowed')),
                       'new_lease': latest.get('lease'),
                       'final_phase': (latest.get('status') or {}).get('phase'),
                       'relocalize_calls': relocalize_calls[0]})
        result['ok'] = bool(result.get('initial_tracking') and result.get('relocalizing')
                            and result.get('backend_switch') and result.get('recovered')
                            and result.get('new_lease') == 2)
        print(json.dumps(result, ensure_ascii=False, indent=1))
        return 0 if result['ok'] else 6
    finally:
        for proc in (node, master):
            if proc is not None and proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=5)


if __name__ == '__main__':
    raise SystemExit(main())
