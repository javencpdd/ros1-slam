#!/usr/bin/env python3
"""Real ROS smoke for multi_map_node: two maps, isolated deltas, restart restore."""
import copy
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
FIXTURE = HERE.parent / 'contracts/fixtures/map_ml_two_surfaces.json'
NODE = HERE / 'multi_map_node.py'
OUT = Path('/tmp/r998_smoke.json')
STATE = Path('/tmp/r998_state')
MAP_A = Path('/tmp/r998_map_a.json')
MAP_B = Path('/tmp/r998_map_b.json')
PORT = 11470
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r998_ros',
                  ROS_LOG_DIR='/tmp/r998_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r998_ros').mkdir(exist_ok=True)
Path('/tmp/r998_logs').mkdir(exist_ok=True)


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


def wait_version(rospy, String, topic, expected, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            payload = json.loads(rospy.wait_for_message(topic, String, timeout=2).data)
            if payload.get('map_version') == expected:
                return payload
        except Exception:
            pass
    raise RuntimeError('%s version %s timeout' % (topic, expected))


def delta(map_id, base, z):
    value = json.loads(FIXTURE.read_text())
    surface = copy.deepcopy(next(s for s in value['surfaces'] if s['surface_id'] == 'mezz'))
    surface['z_reference'] = z
    return json.dumps({'schema_version': 1, 'map_id': map_id, 'base_version': base,
                       'delta_id': 'd1', 'source': {'uri': 'local://r998', 'sha256': 'b' * 64},
                       'ops': [{'op': 'upsert_surface', 'surface': surface}]})


def main():
    import rospy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    if STATE.exists():
        for path in STATE.glob('*.json'):
            path.unlink()
    STATE.mkdir(exist_ok=True)
    base = json.loads(FIXTURE.read_text())
    for path, map_id in ((MAP_A, 'map_a'), (MAP_B, 'map_b')):
        value = copy.deepcopy(base)
        value['map_id'] = map_id
        path.write_text(json.dumps(value))
    master = start(['roscore', '-p', str(PORT)], '/tmp/r998_master.log')
    node = None
    result = {'maps': ['map_a', 'map_b']}
    try:
        time.sleep(3)
        rospy.init_node('r998_smoke', anonymous=True, disable_signals=True)
        node = start([sys.executable, '-B', str(NODE),
                      '_state_dir:=' + str(STATE),
                      '_map_files:=%s,%s' % (MAP_A, MAP_B),
                      '_keep_alive_s:=40'], '/tmp/r998_node1.log')
        initial_a = wait_version(rospy, String, '/v8/maps/map_a', 2)
        initial_b = wait_version(rospy, String, '/v8/maps/map_b', 2)
        result['initial_versions'] = [initial_a['map_version'], initial_b['map_version']]
        publisher = rospy.Publisher('/map_delta', String, queue_size=1)
        time.sleep(0.5)
        publisher.publish(String(data=delta('map_a', 2, .8)))
        after_a = wait_version(rospy, String, '/v8/maps/map_a', 3)
        after_b = wait_version(rospy, String, '/v8/maps/map_b', 2)
        result['after_delta_versions'] = [after_a['map_version'], after_b['map_version']]
        query = rospy.ServiceProxy('/v8/maps/map_a/query', Trigger)
        result['query_version'] = json.loads(query().message)['map_version']
        status = json.loads(rospy.wait_for_message('/v8/maps/status', String, timeout=5).data)
        result['status_applied'] = {key: value['applied_deltas']
                                    for key, value in status['maps'].items()}
        stop(node)
        node = start([sys.executable, '-B', str(NODE),
                      '_state_dir:=' + str(STATE),
                      '_map_files:=%s,%s' % (MAP_A, MAP_B),
                      '_keep_alive_s:=40'], '/tmp/r998_node2.log')
        restored_a = wait_version(rospy, String, '/v8/maps/map_a', 3)
        restored_b = wait_version(rospy, String, '/v8/maps/map_b', 2)
        result['restored_versions'] = [restored_a['map_version'], restored_b['map_version']]
        publisher.publish(String(data=delta('map_a', 3, .8)))
        time.sleep(1.0)
        replay = json.loads(rospy.wait_for_message('/v8/maps/map_a', String, timeout=5).data)
        result['replay_version'] = replay['map_version']
        result['ok'] = (result['initial_versions'] == [2, 2]
                        and result['after_delta_versions'] == [3, 2]
                        and result['query_version'] == 3
                        and result['restored_versions'] == [3, 2]
                        and result['replay_version'] == 3)
    finally:
        if node is not None:
            stop(node)
        stop(master)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
