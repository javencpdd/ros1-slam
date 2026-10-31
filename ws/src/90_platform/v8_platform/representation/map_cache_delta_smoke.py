#!/usr/bin/env python3
"""Real ROS smoke: typed MapDelta -> JSON relay -> delta_map_node version bump."""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = Path('/tmp/r1022_smoke.json')
STATE = Path('/tmp/r1022_state.json')
PORT = 11501
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1022_ros',
                  ROS_LOG_DIR='/tmp/r1022_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1022_ros').mkdir(exist_ok=True)
Path('/tmp/r1022_logs').mkdir(exist_ok=True)


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
    import rospy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    fixture = json.loads((HERE.parent / 'contracts' / 'fixtures'
                          / 'map_ml_two_surfaces.json').read_text())
    mezz = next(item for item in fixture['surfaces'] if item['surface_id'] == 'mezz')
    mezz['z_reference'] = 0.8
    if STATE.exists():
        STATE.unlink()
    master = start(['roscore', '-p', str(PORT)], '/tmp/r1022_master.log')
    nodes = []
    result = {'fixture_version': fixture['map_version']}
    try:
        time.sleep(3)
        rospy.init_node('r1022_smoke', anonymous=True, disable_signals=True)
        delta_node = start([sys.executable, '-B', str(HERE / 'delta_map_node.py'),
                            '_map_json:=' + str(HERE.parent / 'contracts' / 'fixtures'
                                                / 'map_ml_two_surfaces.json'),
                            '_state_path:=' + str(STATE),
                            '_delta_topic:=/map_delta',
                            '_map_topic:=/v8_support_map',
                            '_status_topic:=/v8_map_status'],
                           '/tmp/r1022_delta.log')
        relay = start([sys.executable, '-B', str(HERE / 'map_delta_relay.py'),
                       '_delta_topic:=/v8/cache/delta',
                       '_json_topic:=/map_delta'], '/tmp/r1022_relay.log')
        cache = start([sys.executable, '-B', str(HERE / 'map_cache_node.py'),
                       '_map_id:=two_level_boxes', '_base_version:=2',
                       '_ops_topic:=/v8/cache/ops',
                       '_delta_topic:=/v8/cache/delta',
                       '_ack_topic:=/v8/cache/ack',
                       '_status_topic:=/v8/cache/status'],
                      '/tmp/r1022_cache.log')
        nodes = [delta_node, relay, cache]
        time.sleep(2.5)
        initial = json.loads(rospy.wait_for_message('/v8_map_status', String,
                                                    timeout=10).data)
        result['initial_version'] = initial['map_version']
        publisher = rospy.Publisher('/v8/cache/ops', String, queue_size=10)
        time.sleep(.5)
        publisher.publish(String(data=json.dumps({'op': 'upsert', 'surface': mezz})))
        time.sleep(.3)
        rospy.wait_for_service('/v8_map_cache/flush', timeout=10)
        response = rospy.ServiceProxy('/v8_map_cache/flush', Trigger)()
        result['flush_ok'] = response.success
        deadline = time.monotonic() + 15
        status = None
        while time.monotonic() < deadline:
            status = json.loads(rospy.wait_for_message('/v8_map_status', String,
                                                       timeout=2).data)
            if status.get('map_version') == 3:
                break
            time.sleep(.1)
        result['final_version'] = None if status is None else status.get('map_version')
        query = rospy.ServiceProxy('/v8_delta_map/query', Trigger)
        snapshot = json.loads(query().message)
        result['query_version'] = snapshot.get('map_version')
        surfaces = {item['surface_id']: item for item in snapshot['map']['surfaces']}
        result['mezz_z'] = surfaces['mezz']['z_reference']
        result['ok'] = (result['initial_version'] == 2
                        and result['flush_ok']
                        and result['final_version'] == 3
                        and result['query_version'] == 3
                        and abs(result['mezz_z'] - 0.8) < 1e-9)
    finally:
        for node in nodes:
            stop(node)
        stop(master)
        time.sleep(.3)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        leftover = [row[2] for row in rows
                    if ('map_delta_relay.py' in row[2] or 'map_cache_node.py' in row[2]
                        or 'delta_map_node.py' in row[2])
                    and not row[1].startswith('Z') and 'r1022_smoke' not in row[2]]
        result['processes_after_stop'] = leftover
        result['ok'] = bool(result.get('ok')) and not leftover
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
