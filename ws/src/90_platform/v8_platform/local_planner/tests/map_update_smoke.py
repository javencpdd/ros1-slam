#!/usr/bin/env python3
"""Real ROS smoke: a live map update replaces a feasible 3D plan.

The planner is fed pose/goal/obstacles/localization so it returns a trajectory.
After a delta raises the mezzanine, fresh feedback on the new support must
produce a trajectory carrying the new map version and the new height.
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
REP = ROOT / 'representation'
OUT = Path('/tmp/r1024_smoke.json')
STATE = Path('/tmp/r1024_state.json')
PORT = 11503
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1024_ros',
                  ROS_LOG_DIR='/tmp/r1024_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1024_ros').mkdir(exist_ok=True)
Path('/tmp/r1024_logs').mkdir(exist_ok=True)


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


def wait_status(rospy, String, version, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            status = json.loads(rospy.wait_for_message('/v8/local/status', String,
                                                       timeout=2).data)
            if status.get('map_version') == version:
                return status
        except Exception:
            continue
    return None


def wait_trajectory(rospy, String, latest, version, z, timeout=25):
    deadline = time.monotonic() + timeout
    seen = []
    while time.monotonic() < deadline:
        status = latest.get('status')
        if status is not None and (not seen or seen[-1].get('status') != status):
            seen.append({'status': status})
        try:
            payload = json.loads(rospy.wait_for_message('/v8/local/trajectory', String,
                                                        timeout=1).data)
        except Exception:
            continue
        seen.append({'map_version': payload.get('map_version'),
                     'last': payload.get('trajectory', [[0, 0, 0]])[-1]})
        if (payload.get('map_version') == version
                and abs(payload['trajectory'][-1][2] - z) < 1e-6):
            return payload, seen
    return None, seen


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool, String
    fixture = json.loads((ROOT / 'contracts' / 'fixtures'
                          / 'map_ml_two_surfaces.json').read_text())
    mezz = next(item for item in fixture['surfaces'] if item['surface_id'] == 'mezz')
    raised = json.loads(json.dumps(mezz))
    raised['z_reference'] = 0.8
    if STATE.exists():
        STATE.unlink()
    master = start(['roscore', '-p', str(PORT)], '/tmp/r1024_master.log')
    nodes = []
    result = {'fixture_version': fixture['map_version']}
    try:
        time.sleep(3)
        rospy.init_node('r1024_smoke', anonymous=True, disable_signals=True)
        delta_node = start([sys.executable, '-B', str(REP / 'delta_map_node.py'),
                            '_map_json:=' + str(ROOT / 'contracts' / 'fixtures'
                                                / 'map_ml_two_surfaces.json'),
                            '_state_path:=' + str(STATE),
                            '_delta_topic:=/map_delta',
                            '_map_topic:=/v8_support_map',
                            '_status_topic:=/v8_map_status'],
                           '/tmp/r1024_delta.log')
        planner = start([sys.executable, '-B', str(Path(__file__).resolve().parent
                                                  / 'ros_node.py'),
                         '_map_topic:=/v8_support_map',
                         '_pose_topic:=/v8/local/pose',
                         '_goal_topic:=/v8/local/goal',
                         '_obstacles_topic:=/v8/local/obstacles',
                         '_localization_topic:=/v8/local/localization_ok',
                         '_command_topic:=/v8/local/cmd_vel_3d',
                         '_trajectory_topic:=/v8/local/trajectory',
                         '_status_topic:=/v8/local/status'],
                        '/tmp/r1024_planner.log')
        nodes = [delta_node, planner]
        pose_pub = rospy.Publisher('/v8/local/pose', PoseStamped, queue_size=2)
        goal_pub = rospy.Publisher('/v8/local/goal', PoseStamped, queue_size=2, latch=True)
        obstacles_pub = rospy.Publisher('/v8/local/obstacles', String, queue_size=2)
        localization_pub = rospy.Publisher('/v8/local/localization_ok', Bool, queue_size=2)
        feed = {'pose': (3.0, 2.0, 0.6), 'goal': (3.5, 2.0, 0.6)}
        feed_lock = threading.Lock()
        running = threading.Event()
        running.set()
        latest = {'status': None}

        def on_status(message):
            latest['status'] = json.loads(message.data)
        rospy.Subscriber('/v8/local/status', String, on_status, queue_size=1)

        def signed(x, y, z):
            message = PoseStamped()
            message.header.frame_id, message.header.stamp = 'map', rospy.Time.now()
            message.pose.position.x, message.pose.position.y, message.pose.position.z = x, y, z
            message.pose.orientation.w = 1.
            return message

        def feed_loop():
            obstacles = String(data='[]')
            localization = Bool(data=True)
            sent_goal = None
            while running.is_set() and not rospy.is_shutdown():
                with feed_lock:
                    pose, goal = feed['pose'], feed['goal']
                pose_pub.publish(signed(*pose))
                # A goal is a one-shot decision: repeating it would invalidate
                # every in-flight plan through the generation counter.
                if goal != sent_goal:
                    goal_pub.publish(signed(*goal))
                    sent_goal = goal
                obstacles_pub.publish(obstacles)
                localization_pub.publish(localization)
                time.sleep(.1)

        thread = threading.Thread(target=feed_loop, daemon=True)
        thread.start()

        result['initial'] = wait_status(rospy, String, 2)
        first, first_seen = wait_trajectory(rospy, String, latest, 2, .6)
        result['first'] = None
        if first is not None:
            result['first'] = {'map_version': first['map_version'],
                               'reason': first['reason'],
                               'last': first['trajectory'][-1],
                               'surfaces': sorted(set(first['surfaces']))}
        result['first_seen'] = first_seen[-5:]
        publisher = rospy.Publisher('/map_delta', String, queue_size=10, latch=True)
        delta = {'schema_version': 1, 'map_id': 'two_level_boxes',
                 'base_version': 2, 'delta_id': 'r1024-d1',
                 'source': {'uri': 'smoke://r1024', 'sha256': 'a' * 64},
                 'ops': [{'op': 'upsert_surface', 'surface': raised}]}
        time.sleep(.5)
        publisher.publish(String(data=json.dumps(delta)))
        result['updated'] = wait_status(rospy, String, 3)
        with feed_lock:
            feed.update(pose=(3.0, 2.0, .8), goal=(3.5, 2.0, .8))
        second, second_seen = wait_trajectory(rospy, String, latest, 3, .8)
        result['second'] = None
        if second is not None:
            result['second'] = {'map_version': second['map_version'],
                                'reason': second['reason'],
                                'last': second['trajectory'][-1],
                                'surfaces': sorted(set(second['surfaces']))}
        result['second_seen'] = second_seen[-5:]
        result['trajectory_changed'] = bool(
            first is not None and second is not None
            and first['trajectory'] != second['trajectory'])
        result['ok'] = (result.get('initial') is not None
                        and result['initial'].get('map_version') == 2
                        and result.get('first') is not None
                        and result['first']['reason'] == 'arrived'
                        and abs(result['first']['last'][2] - .6) < 1e-6
                        and abs(result['first']['last'][0] - 3.5) < 1e-6
                        and result.get('updated') is not None
                        and result['updated'].get('map_version') == 3
                        and result.get('second') is not None
                        and result['second']['reason'] == 'arrived'
                        and abs(result['second']['last'][2] - .8) < 1e-6
                        and abs(result['second']['last'][0] - 3.5) < 1e-6
                        and result['trajectory_changed'])
    finally:
        try:
            running.clear()
        except NameError:
            pass
        for node in nodes:
            stop(node)
        stop(master)
        time.sleep(.3)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        leftover = [row[2] for row in rows
                    if ('ros_node.py' in row[2] or 'delta_map_node.py' in row[2])
                    and not row[1].startswith('Z') and 'r1024_smoke' not in row[2]]
        result['processes_after_stop'] = leftover
        result['ok'] = bool(result.get('ok')) and not leftover
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
