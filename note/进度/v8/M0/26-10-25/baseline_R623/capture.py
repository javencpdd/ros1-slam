import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import subprocess
import sys
import tarfile
import tempfile
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[6]
IMAGE = 'sha256:59f759dd5001f3b40da2c2678644ab9f578e7d45a62e757c9a7ba44195caf7d7'
VOLUMES = ['docker_ros1_ws_build', 'docker_ros1_ws_devel']
COPY_INPUTS = ['opt/3rdparty/Livox_SDK2', 'ws/src', 'docker/catkin_build.sh',
               'docker/entrypoint.sh', 'docker/env.sh']


def now():
    return datetime.now(timezone.utc).isoformat()


def run(args):
    p = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return {'command': args, 'rc': p.returncode, 'stdout': p.stdout, 'stderr': p.stderr}


def checked(args):
    result = run(args)
    if result['rc']:
        raise RuntimeError(result)
    return result['stdout']


def digest(stream):
    h = hashlib.sha256()
    for chunk in iter(lambda: stream.read(1024 * 1024), b''):
        h.update(chunk)
    return h.hexdigest()


def source_inventory(root):
    rows = []
    for source in COPY_INPUTS:
        top = root / source
        paths = [top] if not top.is_dir() else sorted(top.rglob('*'))
        for path in paths:
            if path.is_symlink():
                rows.append({'path': str(path.relative_to(root)), 'kind': 'symlink',
                             'target': os.readlink(path), 'target_exists_on_host': path.exists()})
            elif path.is_file():
                with path.open('rb') as f:
                    sha = digest(f)
                rows.append({'path': str(path.relative_to(root)), 'kind': 'file',
                             'size': path.stat().st_size, 'sha256': sha})
        if not top.exists() and not top.is_symlink():
            rows.append({'path': source, 'kind': 'absent'})
    return rows


def read_archive(stream, prefix, expected_root):
    rows = []
    with tarfile.open(fileobj=stream, mode='r|*') as archive:
        for member in archive:
            parts = PurePosixPath(member.name).parts
            if '..' in parts or member.name.startswith('/'):
                raise ValueError('unsafe archive member: ' + member.name)
            if not parts or parts[0] != expected_root:
                raise ValueError('unexpected archive root: ' + member.name)
            relative = '/'.join(parts[1:])
            path = prefix.rstrip('/') + ('/' + relative if relative else '')
            row = {'path': path, 'size': member.size}
            if member.isfile():
                row['kind'] = 'file'
                f = archive.extractfile(member)
                row['sha256'] = digest(f)
            elif member.issym() or member.islnk():
                row.update(kind='symlink' if member.issym() else 'hardlink', target=member.linkname)
            else:
                continue
            rows.append(row)
    return rows


def copy_inventory(container, path, prefix):
    args = ['docker', 'cp', container + ':' + path, '-']
    with tempfile.TemporaryFile() as errors:
        p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=errors)
        try:
            rows = read_archive(p.stdout, prefix, PurePosixPath(path).name)
            parse_error = None
        except (tarfile.TarError, ValueError) as e:
            rows, parse_error = [], str(e)
            for _ in iter(lambda: p.stdout.read(1024 * 1024), b''):
                pass
        rc = p.wait()
        errors.seek(0)
        stderr = errors.read().decode(errors='replace')
    return {'command': args, 'rc': rc, 'stderr': stderr, 'parse_error': parse_error, 'rows': rows}


def copy_text(container, path):
    args = ['docker', 'cp', container + ':' + path, '-']
    p = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    result = {'command': args, 'rc': p.returncode, 'stderr': p.stderr.decode(errors='replace')}
    if p.returncode == 0:
        import io
        with tarfile.open(fileobj=io.BytesIO(p.stdout)) as archive:
            regular = [m for m in archive if m.isfile()]
            if len(regular) != 1:
                raise ValueError('expected one regular file')
            content = archive.extractfile(regular[0]).read()
        result.update(sha256=hashlib.sha256(content).hexdigest(), text=content.decode(errors='replace'))
    return result


def container_snapshot():
    rows = json.loads(checked(['docker', 'inspect', 'ros1-slam-sim', 'ros1-slam-sim-viz']))
    return [{k: row[k] for k in ('Id', 'Name', 'Image', 'State', 'Mounts')} for row in rows]


def compare(left, right):
    if not left or not right:
        raise ValueError('empty inventory is INVALID')
    for row in [*left, *right]:
        if not isinstance(row.get('path'), str) or not row['path']:
            raise ValueError('missing inventory path')
        if row.get('kind') == 'file':
            sha = row.get('sha256')
            if not isinstance(sha, str) or len(sha) != 64 or any(c not in '0123456789abcdef' for c in sha):
                raise ValueError('invalid file digest')
        elif row.get('kind') in ('symlink', 'hardlink'):
            if not isinstance(row.get('target'), str) or not row['target']:
                raise ValueError('missing link target')
        else:
            raise ValueError('invalid inventory kind')
    a = {r['path']: r for r in left}
    b = {r['path']: r for r in right}
    if len(a) != len(left) or len(b) != len(right):
        raise ValueError('duplicate inventory paths')
    result = {'equal': [], 'different': [], 'only_left': sorted(a.keys() - b.keys()),
              'only_right': sorted(b.keys() - a.keys())}
    for path in sorted(a.keys() & b.keys()):
        fields = ('kind', 'sha256', 'target')
        key = 'equal' if all(a[path].get(k) == b[path].get(k) for k in fields) else 'different'
        result[key].append(path)
    return result


def collect():
    result = {'started_at_utc': now(), 'status': 'static_inventory_not_build_or_runtime_acceptance',
              'source_scope': 'raw COPY roots including Docker-ignored entries; not an effective Docker context',
              'main_repo_head': checked(['git', 'rev-parse', 'HEAD']).strip(),
              'existing_before': container_snapshot(), 'host_load': list(os.getloadavg()),
              'host_cpus': os.cpu_count(), 'local_inputs': source_inventory(ROOT)}
    result['input_rules'] = {p: (ROOT / p).read_text() for p in ['.dockerignore', 'docker/Dockerfile']}
    result['volume_metadata'] = json.loads(checked(['docker', 'volume', 'inspect', *VOLUMES]))
    result['image_metadata'] = json.loads(checked(['docker', 'image', 'inspect', IMAGE]))
    result['image_history'] = run(['docker', 'history', '--no-trunc', '--format', '{{json .}}', IMAGE])
    created = []
    result['temporary_containers'] = []
    try:
        for label, mount_args in [('image', []), ('volumes', [
            '--mount', 'type=volume,src=' + VOLUMES[0] + ',dst=/evidence/build,readonly,volume-nocopy',
            '--mount', 'type=volume,src=' + VOLUMES[1] + ',dst=/evidence/devel,readonly,volume-nocopy'])]:
            args = ['docker', 'create', '--network', 'none', '--read-only',
                    '--label', 'ros1-slam.evidence=R623', '--entrypoint', '/bin/true', *mount_args, IMAGE]
            cid = checked(args).strip()
            created.append(cid)
            inspection = json.loads(checked(['docker', 'inspect', cid]))[0]
            result['temporary_containers'].append({'command': args, 'id': cid,
                'state': inspection['State'], 'host_config': inspection['HostConfig'], 'mounts': inspection['Mounts']})
            if label == 'image':
                result['image_sources'] = copy_inventory(cid, '/opt/ros1-slam/ws/src', 'ws/src')
                result['image_artifacts'] = copy_inventory(cid, '/opt/ros1-slam/ws/devel/lib', 'devel/lib')
                result['image_cache'] = copy_text(cid, '/opt/ros1-slam/ws/build/CMakeCache.txt')
                result['image_sdk'] = copy_inventory(cid, '/usr/local/lib/liblivox_lidar_sdk_static.a', 'liblivox_lidar_sdk_static.a')
                result['image_copyright'] = {name: copy_text(cid, '/usr/share/doc/' + name + '/copyright')
                    for name in ['ros-noetic-global-planner', 'ros-noetic-navfn', 'ros-noetic-teb-local-planner',
                                 'ros-noetic-dwa-local-planner', 'ros-noetic-mbf-costmap-nav']}
            else:
                result['volume_artifacts'] = copy_inventory(cid, '/evidence/devel/lib', 'devel/lib')
                result['volume_cache'] = copy_text(cid, '/evidence/build/CMakeCache.txt')
    finally:
        result['cleanup'] = [run(['docker', 'rm', cid]) for cid in reversed(created)]
    result['existing_after'] = container_snapshot()
    result['existing_metadata_unchanged'] = result['existing_before'] == result['existing_after']
    result['local_inputs_unchanged'] = result['local_inputs'] == source_inventory(ROOT)
    if result['image_sources']['rc'] == 0 and result['image_sources']['parse_error'] is None:
        result['source_comparison'] = compare([r for r in result['local_inputs'] if r['path'].startswith('ws/src/')], result['image_sources']['rows'])
    if all(result[k]['rc'] == 0 and result[k]['parse_error'] is None for k in ['image_artifacts', 'volume_artifacts']):
        result['artifact_comparison'] = compare(result['image_artifacts']['rows'], result['volume_artifacts']['rows'])
    result['completed_at_utc'] = now()
    return result


if __name__ == '__main__':
    output = Path(sys.argv[1])
    data = collect()
    with output.open('x') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write('\n')
    checked(['setfacl', '-m', 'u:ros1-slam:rwx', str(output)])
    print(json.dumps({k: data[k] for k in ['started_at_utc', 'completed_at_utc',
        'existing_metadata_unchanged', 'local_inputs_unchanged']}, ensure_ascii=False))
    for key in ['image_sources', 'image_artifacts', 'volume_artifacts', 'image_cache', 'volume_cache']:
        value = data[key]
        print(key, 'rc', value['rc'], 'rows', len(value.get('rows', [])), value['stderr'])
    for key in ['source_comparison', 'artifact_comparison']:
        print(key, {k: len(v) for k, v in data.get(key, {}).items()})
