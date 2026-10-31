#!/usr/bin/env python3
"""Real ROS smoke: mux hold gate across a command-source handoff.

The old source is stopped and a new one started while mux hold is asserted;
/cmd_vel must stay zero until hold is released. This is a safe handoff, not a
zero-interruption swap.
"""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

OUT = Path('/tmp/r1020_smoke.json')
PORT = 11498
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1020_ros',
                  ROS_LOG_DIR='/tmp/r1020_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1020_ros').mkdir(exist_ok=True)
Path('/tmp/r1020_logs').mkdir(exist_ok=True)
LAUNCH = '/repo/ws/src/60_safety/lite3_safety/launch/60_safety.launch'


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


def source(value):
    return ['rostopic', 'pub', '-r', '20', '/cmd_vel_raw',
            'geometry_msgs/Twist', '{linear: {x: %s}}' % value]


def read_speed(rospy, Twist, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            message = rospy.wait_for_message('/cmd_vel', Twist, timeout=1)
            return message.linear.x
        except Exception:
            continue
    return None


def main():
    import rospy
    from std_msgs.msg import Bool
    from geometry_msgs.msg import Twist
    result = {'launch': LAUNCH}
    master = start(['roscore', '-p', str(PORT)], '/tmp/r1020_master.log')
    mux = source_a = source_b = None
    try:
        time.sleep(3)
        rospy.init_node('r1020_smoke', anonymous=True, disable_signals=True)
        hold_pub = rospy.Publisher('/lite3/supervisor/hold', Bool,
                                   queue_size=1, latch=True)
        mux = start(['roslaunch', LAUNCH], '/tmp/r1020_mux.log')
        time.sleep(4)
        result['mux_zero'] = read_speed(rospy, Twist)
        source_a = start(source(0.4), '/tmp/r1020_a.log')
        time.sleep(2.0)
        result['pre_speed'] = read_speed(rospy, Twist)
        hold_pub.publish(Bool(data=True))
        time.sleep(.5)
        result['held_zero'] = read_speed(rospy, Twist)
        stop(source_a)
        source_a = None
        time.sleep(.5)
        result['held_zero_after_stop'] = read_speed(rospy, Twist)
        source_b = start(source(0.3), '/tmp/r1020_b.log')
        time.sleep(2.0)
        result['held_zero_with_new'] = read_speed(rospy, Twist)
        hold_pub.publish(Bool(data=False))
        result['post_speed'] = read_speed(rospy, Twist)
        stop(source_b)
        source_b = None
        stop(mux)
        mux = None
        time.sleep(.5)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        leftover = [row[2] for row in rows
                    if ('safe_cmd_vel_mux.py' in row[2] or 'rostopic pub' in row[2])
                    and not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        result['ok'] = (result['pre_speed'] is not None
                        and abs(result['pre_speed'] - 0.4) < 0.02
                        and result['held_zero'] is not None
                        and abs(result['held_zero']) < 1e-9
                        and result['held_zero_after_stop'] is not None
                        and abs(result['held_zero_after_stop']) < 1e-9
                        and result['held_zero_with_new'] is not None
                        and abs(result['held_zero_with_new']) < 1e-9
                        and result['post_speed'] is not None
                        and abs(result['post_speed'] - 0.3) < 0.02
                        and not leftover)
    finally:
        for process in (source_a, source_b, mux):
            if process is not None:
                stop(process)
        stop(master)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
