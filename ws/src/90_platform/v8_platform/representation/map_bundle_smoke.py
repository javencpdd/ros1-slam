#!/usr/bin/env python3
"""Real ROS smoke: relocatable map bundle with multi_map_node.

Apply a delta in state dir A, export a bundle (state + map files), import it
into a new root B, then start a fresh provider on B and verify the restored
versions and the updated surface height.
"""
import copy
import json
import os
import signal
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
FIXTURE = HERE.parent / 'contracts' / 'fixtures' / 'map_ml_two_surfaces.json'
OUT = Path('/tmp/r1031_smoke.json')
ROOT = Path('/tmp/r1031')
PORT = 11517
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/r1031_ros',
                  ROS_LOG_DIR='/tmp/r1031_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/r1031_ros').mkdir(exist_ok=True)
Path('/tmp/r1031_logs').mkdir(exist_ok=True)
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import map_bundle  # noqa: E402


def start(argv, log):
    return subprocess.Popen(argv, stdout=open(log, 'ab'), stderr=subprocess.STDOUT,
                            start_new_session=True, env=dict(os.environ))


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def wait_version(rospy, String, topic, expected, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            payload = json.loads(rospy.wait_for_message(topic, String, timeout=2).data)
            if payload.get('map_version') == expected:
                return payload
        except Exception:
            continue
    return None


def delta(map_id, base, z):
    value = json.loads(FIXTURE.read_text())
    surface = copy.deepcopy(next(item for item in value['surfaces']
                                 if item['surface_id'] == 'mezz'))
    surface['z_reference'] = z
    return json.dumps({'schema_version': 1, 'map_id': map_id, 'base_version': base,
                       'delta_id': 'd1', 'source': {'uri': 'local://r1031',
                                                    'sha256': 'b' * 64},
                       'ops': [{'op': 'upsert_surface', 'surface': surface}]})


def main():
    import rospy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    if ROOT.exists():
        shutil.rmtree(ROOT)
    state_a = ROOT / 'state_a'
    maps_a = ROOT / 'maps_a'
    state_b = ROOT / 'state_b'
    maps_b = ROOT / 'maps_b'
    for path in (state_a, maps_a):
        path.mkdir(parents=True)
    base = json.loads(FIXTURE.read_text())
    map_files = []
    for name, map_id in (('map_a.json', 'map_a'), ('map_b.json', 'map_b')):
        value = copy.deepcopy(base)
        value['map_id'] = map_id
        path = maps_a / name
        path.write_text(json.dumps(value))
        map_files.append(path)
    master = start(['roscore', '-p', str(PORT)], '/tmp/r1031_master.log')
    nodes = []
    result = {'bundle': str(ROOT / 'bundle')}
    try:
        time.sleep(3)
        rospy.init_node('r1031_smoke', anonymous=True, disable_signals=True)
        nodes.append(start([sys.executable, '-B', str(HERE / 'multi_map_node.py'),
                            '_state_dir:=' + str(state_a),
                            '_map_files:=%s,%s' % (maps_a / 'map_a.json',
                                                   maps_a / 'map_b.json'),
                            '_keep_alive_s:=40'], '/tmp/r1031_node_a.log'))
        initial = wait_version(rospy, String, '/v8/maps/map_a', 2)
        publisher = rospy.Publisher('/map_delta', String, queue_size=1)
        time.sleep(.5)
        publisher.publish(String(data=delta('map_a', 2, .8)))
        updated = wait_version(rospy, String, '/v8/maps/map_a', 3)
        result['before_relocation'] = None if initial is None or updated is None else {
            'map_a': [initial['map_version'], updated['map_version']]}
        stop(nodes.pop())
        time.sleep(1.)
        exported = map_bundle.export_bundle(state_a, ROOT / 'bundle', map_files)
        imported = map_bundle.import_bundle(ROOT / 'bundle', state_b, maps_b)
        result['exported'] = exported
        result['imported'] = imported
        nodes.append(start([sys.executable, '-B', str(HERE / 'multi_map_node.py'),
                            '_state_dir:=' + str(state_b),
                            '_map_files:=%s,%s' % (maps_b / 'map_a.json',
                                                   maps_b / 'map_b.json'),
                            '_keep_alive_s:=40'], '/tmp/r1031_node_b.log'))
        restored_a = wait_version(rospy, String, '/v8/maps/map_a', 3)
        restored_b = wait_version(rospy, String, '/v8/maps/map_b', 2)
        result['after_relocation'] = None if restored_a is None or restored_b is None else {
            'map_a': restored_a['map_version'], 'map_b': restored_b['map_version'],
            'map_a_mezz_z': next(item['z_reference'] for item in restored_a['map']['surfaces']
                                 if item['surface_id'] == 'mezz')}
        query = rospy.ServiceProxy('/v8/maps/map_a/query', Trigger)
        result['query_version'] = json.loads(query().message)['map_version']
    except Exception as error:
        import traceback
        result['error'] = repr(error)
        result['traceback'] = traceback.format_exc()
    finally:
        for node in reversed(nodes):
            stop(node)
        stop(master)
        time.sleep(.3)
        ps = subprocess.check_output(['ps', '-eo', 'pid,stat,cmd'], text=True)
        rows = [line.split(None, 2) for line in ps.splitlines()[1:]]
        leftover = [row[2] for row in rows
                    if 'multi_map_node.py' in row[2]
                    and 'r1031_smoke' not in row[2] and not row[1].startswith('Z')]
        result['processes_after_stop'] = leftover
        result['ok'] = (result.get('before_relocation') == {'map_a': [2, 3]}
                        and result.get('after_relocation') == {
                            'map_a': 3, 'map_b': 2, 'map_a_mezz_z': .8}
                        and result.get('query_version') == 3
                        and not leftover)
        OUT.write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=1))
        sys.exit(0 if result.get('ok') else 6)


if __name__ == '__main__':
    raise SystemExit(main())
