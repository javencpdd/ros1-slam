#!/usr/bin/env python3
"""Real ROS smoke: local planner consumes /v8_support_map and sees version bump."""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
REP = ROOT / 'representation'
OUT = Path('/tmp/r1023_smoke.json')
STATE = Path('/tmp/r1023_state.json')
PORT = 11502
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1023_ros',
                  ROS_LOG_DIR='/tmp/r1023_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1023_ros').mkdir(exist_ok=True)
Path('/tmp/r1023_logs').mkdir(exist_ok=True)


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


def wait_version(rospy, String, version, timeout=15):
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


def main():
    import rospy
    from std_msgs.msg import String
    fixture = json.loads((ROOT / 'contracts' / 'fixtures'
                          / 'map_ml_two_surfaces.json').read_text())
    mezz = next(item for item in fixture['surfaces'] if item['surface_id'] == 'mezz')
    mezz['z_reference'] = 0.8
    if STATE.exists():
        STATE.unlink()
    master = start(['roscore', '-p', str(PORT)], '/tmp/r1023_master.log')
    nodes = []
    result = {'fixture_version': fixture['map_version']}
    try:
        time.sleep(3)
        rospy.init_node('r1023_smoke', anonymous=True, disable_signals=True)
        delta_node = start([sys.executable, '-B', str(REP / 'delta_map_node.py'),
                            '_map_json:=' + str(ROOT / 'contracts' / 'fixtures'
                                                / 'map_ml_two_surfaces.json'),
                            '_state_path:=' + str(STATE),
                            '_delta_topic:=/map_delta',
                            '_map_topic:=/v8_support_map',
                            '_status_topic:=/v8_map_status'],
                           '/tmp/r1023_delta.log')
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
                        '/tmp/r1023_planner.log')
        nodes = [delta_node, planner]
        result['initial'] = wait_version(rospy, String, 2)
        publisher = rospy.Publisher('/map_delta', String, queue_size=10, latch=True)
        delta = {'schema_version': 1, 'map_id': 'two_level_boxes',
                 'base_version': 2, 'delta_id': 'r1023-d1',
                 'source': {'uri': 'smoke://r1023', 'sha256': 'a' * 64},
                 'ops': [{'op': 'upsert_surface', 'surface': mezz}]}
        time.sleep(.5)
        publisher.publish(String(data=json.dumps(delta)))
        result['updated'] = wait_version(rospy, String, 3)
        result['ok'] = (result.get('initial') is not None
                        and result['initial'].get('map_version') == 2
                        and result.get('updated') is not None
                        and result['updated'].get('map_version') == 3)
    finally:
        for node in nodes:
            stop(node)
        stop(master)
        time.sleep(.3)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        leftover = [row[2] for row in rows
                    if ('ros_node.py' in row[2] or 'delta_map_node.py' in row[2])
                    and not row[1].startswith('Z') and 'r1023_smoke' not in row[2]]
        result['processes_after_stop'] = leftover
        result['ok'] = bool(result.get('ok')) and not leftover
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
