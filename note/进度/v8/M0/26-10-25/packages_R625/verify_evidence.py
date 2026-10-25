import csv
import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
REQUIRED = {'samples_ready', 'tf_single_writer', 'tf_connected', 'ray_geometry', 'scan_time',
            'imu_gravity', 'imu_time', 'clock_monotonic', 'map_nonempty', 'costmap_nonempty',
            'built_plugin_loaded', 'no_old_workspace_libraries', 'system_sensors_loaded',
            'processes_alive', 'resources', 'basic_stop'}
LIB = '/opt/ros1-slam/r625/ws/devel/lib/liblite3_costmap_layers.so'


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def load():
    stage = json.loads((HERE / 'runtime-attempt3.json').read_text())
    assert stage['result']['path'] == 'runtime-' + stage['token'] + '/result.json'
    assert stage['execution_snapshot'] == 'execution-' + stage['token']
    raw = (HERE / stage['result']['path']).read_bytes()
    snapshot = HERE / stage['execution_snapshot']
    return {'stage': stage, 'runtime': json.loads(raw), 'result_sha256': sha(raw),
            'snapshot_hashes': {p.name: sha(p.read_bytes()) for p in snapshot.iterdir() if p.is_file()},
            'build': json.loads((HERE / 'build-result.json').read_text()),
            'build_stage': json.loads((HERE / 'build-attempt1.json').read_text()),
            'library_sha256': sha((HERE / 'liblite3_costmap_layers.so').read_bytes()),
            'source_manifest': json.loads((HERE / 'source-manifest.json').read_text())['files'],
            'source_hashes': {str(p.relative_to(HERE / 'source-snapshot/lite3_costmap_layers')): sha(p.read_bytes())
                              for p in (HERE / 'source-snapshot/lite3_costmap_layers').rglob('*') if p.is_file()},
            'fixture_hashes': {name: sha((HERE / stage['result']['path']).parent.joinpath(name).read_bytes())
                               for name in ('scene.world', 'map.pgm', 'map.yaml')}}


def validate(data):
    s, r, b = data['stage'], data['runtime'], data['build_stage']
    assert s['stage_ok'] is True and s['errors'] == []
    for stage in (s, b):
        assert stage['stage_ok'] is True and stage['errors'] == []
        assert stage['execution']['rc'] == 0 and stage['exit_state']['ExitCode'] == 0
        assert stage['exit_state']['Status'] == 'exited' and stage['exit_state']['OOMKilled'] is False
        assert stage['remaining'] == [] and stage['remove']['rc'] == 0 and 'forced_stop' not in stage
        assert stage['before'] == stage['after']
        assert stage['input_before'] == stage['input_after']
        assert stage['source_before'] == stage['source_after'] == data['source_manifest']
        assert stage['isolation']['Image'] == 'sha256:59f759dd5001f3b40da2c2678644ab9f578e7d45a62e757c9a7ba44195caf7d7'
        assert stage['isolation']['HostConfig']['NetworkMode'] == 'none'
        assert stage['isolation']['HostConfig']['ReadonlyRootfs'] is True
    assert len(data['source_manifest']) == 10 and data['source_manifest'] == data['source_hashes']
    assert s['execution_inputs_before'] == s['execution_inputs_after'] == data['snapshot_hashes']
    assert set(data['snapshot_hashes']) == {'runtime.py', 'runtime.sh', 'probe.py', 'run_stage.py'}
    assert s['driver_sha256'] == data['snapshot_hashes']['run_stage.py']
    assert s['result']['sha256'] == data['result_sha256']
    assert r['run'] == '/opt/ros1-slam/r625/runtime-' + s['token']
    assert r['accepted'] is True and r['errors'] == []
    assert set(r['required_checks']) == REQUIRED and set(r['checks']) == REQUIRED
    assert all(r['checks'][k] is True for k in REQUIRED)
    assert data['fixture_hashes'] == r['fixture_hashes']
    assert data['build']['sha256'] == data['library_sha256'] == r['loaded_files']['costmap'][LIB]
    assert not any('/opt/ros1-slam/ws/devel/' in p for group in r['loaded_files'].values() for p in group)
    for lib in ('libgazebo_ros_laser.so', 'libgazebo_ros_imu_sensor.so'):
        assert '/opt/ros/noetic/lib/' + lib in r['loaded_files']['gazebo']
    assert set(r['tf_writers']) == {'map->odom', 'odom->base_link', 'base_link->lidar', 'base_link->imu'}
    assert all(len(v) == 1 and v[0] for v in r['tf_writers'].values())
    for kind, minimum in (('scan', 50), ('imu', 300)):
        stamps = r[kind]['stamps']
        assert len(stamps) >= minimum and all(math.isfinite(v) for v in stamps)
        assert all(y > x for x, y in zip(stamps, stamps[1:]))
        assert r[kind]['publisher'] == '/gazebo'
    scan = r['scan']
    assert scan['frame'] == 'lidar' and len(scan['ranges']) == 181
    finite = [(i, v) for i, v in enumerate(scan['ranges']) if v is not None and math.isfinite(v) and 0.1 < v < 10]
    assert len(finite) > 10 and abs(scan['ranges'][90] - 2.5) < 0.05
    for i, distance in finite:
        angle = scan['angle_min'] + i * scan['angle_increment']
        x, y = distance * math.cos(angle), distance * math.sin(angle)
        assert -5 <= x < 5 and -5 <= y < 5
    acc = r['imu']['acceleration']
    assert r['imu']['frame'] == 'imu' and len(acc) == 3
    assert all(math.isfinite(v) for v in acc) and abs(math.sqrt(sum(v*v for v in acc)) - 9.8) < 0.2
    assert r['clock']['count'] > 10 and r['clock']['last'] - r['clock']['first'] > 4
    for kind in ('map', 'costmap'):
        assert r[kind]['width'] == r[kind]['height'] == 100 and r[kind]['resolution'] > 0
        assert r[kind]['frame'] == 'map' and r[kind]['occupied'] > 0 and r[kind]['free'] > 0
    assert r['resource']['load'][0] <= r['resource']['cpu_count'] * 0.5
    assert r['resource']['statvfs_free'] >= 15 * 1024**3
    assert len(r['stopped']) == 9 and all(not p['forced'] and p['rc'] in (0, -2) for p in r['stopped'])
    return len(finite)


def versions():
    observed = json.loads((HERE / 'observations.json').read_text())
    assert observed['capture_complete'] is True and observed['errors'] == []
    status = {}
    for block in observed['dpkg_status']['text'].split('\n\n'):
        fields = dict(line.split(': ', 1) for line in block.splitlines() if ': ' in line and not line.startswith(' '))
        if 'Package' in fields:
            status[fields['Package']] = fields
    with (HERE.parent / 'environment_R617/image-packages.tsv').open() as stream:
        old = {r['Package']: r for r in csv.DictReader(stream, delimiter='\t')}
    historical = json.loads((HERE.parent / 'provenance_R622/inventory-final.json').read_text())['historical_binary_baseline']['selected_packages']
    historical = {r['Package']: r for r in historical}
    names = [p['package'] for p in observed['packages']]
    assert len(names) == len(set(names)) == 12
    for name in names:
        for field in ('Version', 'Architecture', 'Status', 'Source'):
            assert status[name].get(field, '') == old[name][field] == historical[name][field], (name, field)
    print('PASS 12 historical binary packages: Version/Architecture/Status/Source equal R617 and R622')


if __name__ == '__main__':
    count = validate(load())
    print(f'PASS third-run evidence bindings and raw observations; {count} finite ray endpoints in map')
    print('LIMIT clock monotonicity and process readiness retain runtime checks; this is not a full bag replay')
    versions()
