#!/usr/bin/env python3
"""Real ROS smoke: v8_msgs MapDelta through map_cache_node and map_merge_node."""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import ros_map_codec  # noqa: E402

OUT = Path('/tmp/r1014_smoke.json')
PORT = 11492
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1014_ros',
                  ROS_LOG_DIR='/tmp/r1014_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1014_ros').mkdir(exist_ok=True)
Path('/tmp/r1014_logs').mkdir(exist_ok=True)


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


def surface(surface_id, z, edges=None):
    return {'surface_id': surface_id, 'z_reference': z, 'thickness_m': 0.05,
            'representation': 'elevation_grid', 'data_uri': surface_id + '.json',
            'traversability': 'supported', 'clearance_min_m': 0.4,
            'footprint': {'min_x': 0, 'min_y': 0, 'max_x': 1, 'max_y': 1},
            'edges': edges or []}


def delta(delta_id, z):
    return {'schema_version': 1, 'map_id': 'svc_map', 'base_version': 1,
            'delta_id': delta_id,
            'source': {'uri': 'smoke://' + delta_id, 'sha256': 'a' * 64},
            'ops': [{'op': 'upsert_surface', 'surface': surface('mezz', z)}]}


def main():
    import rospy
    from std_msgs.msg import String, UInt32
    from std_srvs.srv import Trigger
    from v8_msgs.msg import MapDelta
    master = start(['roscore', '-p', str(PORT)], '/tmp/r1014_master.log')
    cache = merge = None
    result = {'ros_master': PORT}
    try:
        time.sleep(3)
        rospy.init_node('r1014_smoke', anonymous=True, disable_signals=True)
        cache = start([sys.executable, '-B', str(HERE / 'map_cache_node.py'),
                       '_map_id:=svc_map', '_base_version:=1',
                       '_ops_topic:=/v8/cache/ops',
                       '_delta_topic:=/v8/cache/delta',
                       '_ack_topic:=/v8/cache/ack',
                       '_status_topic:=/v8/cache/status'],
                      '/tmp/r1014_cache.log')
        ops_pub = rospy.Publisher('/v8/cache/ops', String, queue_size=10)
        ack_pub = rospy.Publisher('/v8/cache/ack', UInt32, queue_size=10)
        deltas = []
        rospy.Subscriber('/v8/cache/delta', MapDelta,
                         lambda message: deltas.append(message), queue_size=10)
        time.sleep(1.5)
        ops_pub.publish(String(data=json.dumps({'op': 'upsert',
                                                'surface': surface('mezz', 0.8)})))
        time.sleep(.2)
        flush = rospy.ServiceProxy('/v8_map_cache/flush', Trigger)
        response = flush()
        result['flush_ok'] = response.success
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not deltas:
            time.sleep(.1)
        if not deltas:
            raise RuntimeError('no delta after flush')
        delta1 = deltas[-1]
        value1 = ros_map_codec.delta_value(delta1)
        result['delta1'] = {'map_id': value1['map_id'],
                            'base_version': value1['base_version'],
                            'op': value1['ops'][0]['op'],
                            'surface_id': value1['ops'][0]['surface']['surface_id'],
                            'z': value1['ops'][0]['surface']['z_reference']}
        ack_pub.publish(UInt32(data=2))
        deadline = time.monotonic() + 5
        status = None
        while time.monotonic() < deadline:
            status = json.loads(rospy.wait_for_message('/v8/cache/status', String,
                                                       timeout=2).data)
            if status.get('base_version') == 2:
                break
            time.sleep(.1)
        result['cache_base_after_ack'] = None if status is None else status.get('base_version')
        ops_pub.publish(String(data=json.dumps({'op': 'remove',
                                                'surface_id': 'mezz'})))
        time.sleep(.2)
        response = flush()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and len(deltas) < 2:
            time.sleep(.1)
        if len(deltas) < 2:
            raise RuntimeError('no second delta after flush')
        delta2 = deltas[-1]
        value2 = ros_map_codec.delta_value(delta2)
        result['delta2'] = {'base_version': value2['base_version'],
                            'op': value2['ops'][0]['op']}
        merge = start([sys.executable, '-B', str(HERE / 'map_merge_node.py'),
                       '_delta_a:=/v8/delta_a', '_delta_b:=/v8/delta_b',
                       '_delta_topic:=/v8/merged',
                       '_status_topic:=/v8/merge/status'],
                      '/tmp/r1014_merge.log')
        time.sleep(1.0)
        merged_messages = []
        rospy.Subscriber('/v8/merged', MapDelta,
                         lambda message: merged_messages.append(message), queue_size=10)
        pub_a = rospy.Publisher('/v8/delta_a', MapDelta, queue_size=10)
        pub_b = rospy.Publisher('/v8/delta_b', MapDelta, queue_size=10)
        time.sleep(1.0)
        pub_a.publish(ros_map_codec.delta_msg(delta('a', 0.7)))
        pub_b.publish(ros_map_codec.delta_msg(delta('b', 0.9)))
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not merged_messages:
            time.sleep(.1)
        if not merged_messages:
            raise RuntimeError('no merged delta')
        merged = merged_messages[-1]
        merged_value = ros_map_codec.delta_value(merged)
        result['merged'] = {'map_id': merged_value['map_id'],
                            'base_version': merged_value['base_version'],
                            'surface_id': merged_value['ops'][0]['surface']['surface_id'],
                            'z': merged_value['ops'][0]['surface']['z_reference']}
        result['merged_status'] = json.loads(
            rospy.wait_for_message('/v8/merge/status', String, timeout=5).data)
        result['ok'] = (result['flush_ok']
                        and result['delta1'] == {'map_id': 'svc_map', 'base_version': 1,
                                                 'op': 'upsert_surface',
                                                 'surface_id': 'mezz', 'z': 0.8}
                        and result['cache_base_after_ack'] == 2
                        and result['delta2'] == {'base_version': 2,
                                                 'op': 'remove_surface'}
                        and result['merged'] == {'map_id': 'svc_map',
                                                 'base_version': 1,
                                                 'surface_id': 'mezz', 'z': 0.9}
                        and result['merged_status'].get('error') is None)
    finally:
        for process in (cache, merge):
            if process is not None:
                stop(process)
        time.sleep(.5)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        leftover = [row[2] for row in rows
                    if ('map_cache_node.py' in row[2] or 'map_merge_node.py' in row[2])
                    and 'r1014_smoke' not in row[2] and not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        result['ok'] = bool(result.get('ok')) and not leftover
        stop(master)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
