#!/usr/bin/env python3
"""Real ROS smoke for semantic_map_node: two maps, latched publish, query, status."""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
NODE = HERE / 'semantic_map_node.py'
OUT = Path('/tmp/r1004_smoke.json')
MAP_A = Path('/tmp/r1004_sem_a.json')
MAP_B = Path('/tmp/r1004_sem_b.json')
PORT = 11471
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1004_ros',
                  ROS_LOG_DIR='/tmp/r1004_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1004_ros').mkdir(exist_ok=True)
Path('/tmp/r1004_logs').mkdir(exist_ok=True)


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


def make_map(map_id, version, features):
    return {'schema': 'semantic_map/v1', 'map_id': map_id,
            'map_version': version, 'frame': 'map', 'features': features}


def feature(feature_id, kind, geometry):
    return {'id': feature_id, 'kind': kind, 'geometry': geometry,
            'attributes': {'source': 'r1004'}}


def main():
    import rospy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    MAP_A.write_text(json.dumps(make_map('sem_a', 2, [
        feature('entry', 'point', {'type': 'point', 'point': [1., 2., 0.]}),
        feature('rail', 'line', {'type': 'line',
                                 'points': [[0., 0., 0.], [2., 0., 0.]]})])))
    MAP_B.write_text(json.dumps(make_map('sem_b', 1, [
        feature('platform', 'box', {'type': 'box',
                                    'min': [0., 0., 0.], 'max': [2., 1., .3]}),
        feature('zone', 'polygon', {'type': 'polygon',
                                    'points': [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.]]})])))
    master = start(['roscore', '-p', str(PORT)], '/tmp/r1004_master.log')
    node = None
    result = {'maps': ['sem_a', 'sem_b']}
    try:
        time.sleep(3)
        rospy.init_node('r1004_smoke', anonymous=True, disable_signals=True)
        rejected = subprocess.run(
            [sys.executable, '-B', str(NODE), '_semantic_files:=%s,%s' % (MAP_A, MAP_A),
             '_keep_alive_s:=5'], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            env=dict(os.environ), timeout=20)
        result['duplicate_rc'] = rejected.returncode
        result['duplicate_error'] = b'E_SEMANTIC_DUPLICATE' in rejected.stderr
        node = start([sys.executable, '-B', str(NODE),
                      '_semantic_files:=%s,%s' % (MAP_A, MAP_B),
                      '_keep_alive_s:=40'], '/tmp/r1004_node.log')
        payload_a = json.loads(rospy.wait_for_message('/v8/semantics/sem_a', String,
                                                      timeout=10).data)
        payload_b = json.loads(rospy.wait_for_message('/v8/semantics/sem_b', String,
                                                      timeout=10).data)
        result['sem_a'] = {'map_version': payload_a['map_version'],
                           'features': len(payload_a['features']),
                           'kinds': sorted(item['kind'] for item in payload_a['features'])}
        result['sem_b'] = {'map_version': payload_b['map_version'],
                           'features': len(payload_b['features']),
                           'kinds': sorted(item['kind'] for item in payload_b['features'])}
        query = rospy.ServiceProxy('/v8/semantics/sem_b/query', Trigger)
        queried = json.loads(query().message)
        result['query'] = {'map_id': queried['map_id'], 'frame': queried['frame'],
                           'features': len(queried['features'])}
        status = json.loads(rospy.wait_for_message('/v8/semantics/status', String,
                                                   timeout=5).data)
        result['status'] = status
        result['ok'] = (result['duplicate_rc'] != 0 and result['duplicate_error']
                        and result['sem_a'] == {'map_version': 2, 'features': 2,
                                                'kinds': ['line', 'point']}
                        and result['sem_b'] == {'map_version': 1, 'features': 2,
                                                'kinds': ['box', 'polygon']}
                        and result['query'] == {'map_id': 'sem_b', 'frame': 'map',
                                                'features': 2}
                        and status['sem_a']['features'] == 2
                        and status['sem_b']['kinds'] == {'box': 1, 'polygon': 1})
    finally:
        if node is not None:
            stop(node)
        stop(master)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
