#!/usr/bin/env python3
import argparse
import ast
from datetime import datetime, timezone
import hashlib
import io
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import tarfile
import uuid


def run(*argv):
    result = subprocess.run(argv, capture_output=True, text=True, check=True,
                            env={**os.environ, 'LC_ALL': 'C'})
    return result.stdout.strip()


def inspect(kind, ref):
    result = subprocess.run(['docker', kind, 'inspect', ref], capture_output=True, text=True)
    if result.returncode:
        return {'ref': ref, 'rc': result.returncode, 'error': result.stderr.strip()}
    data = json.loads(result.stdout)[0]
    if kind == 'image':
        fields = ['Id', 'RepoTags', 'RepoDigests', 'Created', 'Os', 'Architecture', 'Size']
        return {'ref': ref, **{key: data[key] for key in fields}}
    return {
        'ref': ref, 'Id': data['Id'], 'Image': data['Image'],
        'image_ref': data['Config']['Image'], 'State': data['State'],
        'Mounts': data['Mounts'],
        'HostConfig': {key: data['HostConfig'][key] for key in
                       ['NetworkMode', 'ReadonlyRootfs', 'Memory', 'NanoCpus', 'CpusetCpus', 'ShmSize']},
        'environment': [value for value in data['Config']['Env'] if value.split('=', 1)[0] in
                        ['ROS_DISTRO', 'ROS_MASTER_URI', 'ROS1_MASTER_URI',
                         'GAZEBO_MASTER_URI', 'PYTHONNOUSERSITE', 'LIBGL_ALWAYS_SOFTWARE']],
    }


def copy_file(container, path):
    result = subprocess.run(['docker', 'cp', '-L', f'{container}:{path}', '-'], capture_output=True)
    record = {'path': path, 'rc': result.returncode}
    if result.returncode:
        record['error'] = result.stderr.decode().strip()
        return record, None
    with tarfile.open(fileobj=io.BytesIO(result.stdout)) as archive:
        members = [member for member in archive.getmembers() if member.isfile()]
        if len(members) != 1:
            raise ValueError(f'Expected one regular file for {path}, got {len(members)}')
        content = archive.extractfile(members[0]).read()
    record.update(bytes=len(content), sha256=hashlib.sha256(content).hexdigest())
    return record, content


def parse_packages(content):
    packages = []
    for paragraph in content.decode().strip().split('\n\n'):
        fields = dict(line.split(': ', 1) for line in paragraph.splitlines()
                      if ': ' in line and not line[0].isspace())
        if 'Package' not in fields:
            continue
        packages.append({key: fields.get(key, '') for key in
                         ['Package', 'Version', 'Architecture', 'Status', 'Source']})
    return sorted(packages, key=lambda row: (row['Package'], row['Architecture']))


def static_versions(content):
    versions = {}
    for node in ast.parse(content.decode()).body:
        if isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant):
            for target in node.targets:
                if isinstance(target, ast.Name) and 'version' in target.id:
                    versions[target.id] = node.value.value
    return versions


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--container', default='ros1-slam-sim')
    args = parser.parse_args()
    root = args.root.resolve()
    out = args.output.resolve()
    if not out.is_dir():
        parser.error('Output directory must already exist')
    if any((out / name).exists() for name in ['environment-lock.json', 'image-packages.tsv']):
        parser.error('Refusing to overwrite an existing snapshot')
    started = datetime.now(timezone.utc).isoformat()
    container = inspect('container', args.container)
    if container.get('rc') or container['State']['Running']:
        parser.error('The selected existing container must be stopped')
    image = inspect('image', container['Image'])
    if image.get('rc'):
        parser.error('The exact image used by the container must exist locally')
    config_paths = ['docker/Dockerfile', 'docker/docker-compose.yml', 'docker/catkin_build.sh',
                    'docker/build.sh', 'docker/entrypoint.sh', 'docker/env.sh']
    config_hashes = {name: hashlib.sha256((root / name).read_bytes()).hexdigest()
                     for name in config_paths}
    env_keys = ['ROS_BASE_IMAGE', 'ROS_BASE_TAG', 'IMAGE', 'SIM_IMAGE', 'SLIM', 'JOBS']
    selected_env = {}
    for line in (root / 'docker/.env').read_text().splitlines():
        if '=' in line and not line.lstrip().startswith('#'):
            key, value = line.split('=', 1)
            if key.strip() in env_keys:
                selected_env[key.strip()] = value.split(' #', 1)[0].strip()
    refs = [f"{selected_env['ROS_BASE_IMAGE']}:{selected_env['ROS_BASE_TAG']}",
            selected_env['IMAGE'], selected_env['SIM_IMAGE']]
    existing_copy, _ = copy_file(container['Id'], '/var/lib/dpkg/status')
    specimen_name = f'v8-env-inventory-{uuid.uuid4().hex[:12]}'
    specimen_id = run('docker', 'create', '--name', specimen_name, '--network', 'none',
                      '--read-only', '--entrypoint', '/bin/true', image['Id'])
    try:
        specimen = inspect('container', specimen_id)
        if specimen['State']['Running'] or specimen['Mounts']:
            raise RuntimeError('Specimen must be unstarted and have no mounts')
        files = {}
        packages = []
        for path in ['/var/lib/dpkg/status', '/etc/os-release',
                     '/usr/local/lib/python3.8/dist-packages/numpy/version.py',
                     '/usr/local/lib/python3.8/dist-packages/scipy/version.py',
                     '/usr/local/bin/catkin_build.sh',
                     '/opt/ros1-slam/ws/build/CMakeCache.txt',
                     '/opt/ros1-slam/ws/devel/setup.bash']:
            record, content = copy_file(specimen_id, path)
            if content is not None:
                if path == '/var/lib/dpkg/status':
                    packages = parse_packages(content)
                elif path == '/etc/os-release':
                    record['text'] = content.decode()
                elif path.endswith('/version.py'):
                    record['static_literals'] = static_versions(content)
                elif path.endswith('CMakeCache.txt'):
                    record['selected_entries'] = [line for line in content.decode().splitlines()
                        if line.startswith(('CMAKE_BUILD_TYPE:', 'CMAKE_CXX_COMPILER:',
                                            'CMAKE_C_COMPILER:', 'ROS_EDITION:',
                                            'CATKIN_WHITELIST_PACKAGES:', 'CMAKE_HOME_DIRECTORY:',
                                            'PYTHON_EXECUTABLE:', 'CATKIN_DEVEL_PREFIX:'))]
            files[path] = record
        if not packages:
            raise RuntimeError('Image package database is empty or unavailable')
    finally:
        run('docker', 'rm', specimen_id)
    after = inspect('container', args.container)
    if container != after:
        raise RuntimeError('Existing container metadata changed during capture; do not publish snapshot')
    package_text = 'Package\tVersion\tArchitecture\tStatus\tSource\n' + ''.join(
        '\t'.join(row.values()) + '\n' for row in packages)
    cpu = json.loads(run('lscpu', '--json'))['lscpu']
    mem = dict(line.split(':', 1) for line in Path('/proc/meminfo').read_text().splitlines())
    usage = shutil.disk_usage(root)
    snapshot = {
        'schema_version': 1, 'status': 'partial_observed_baseline',
        'captured_at_utc': started,
        'source': {'root': str(root), 'head': run('git', '-C', str(root), 'rev-parse', 'HEAD'),
                   'trees': {name: run('git', '-C', str(root), 'rev-parse', f'HEAD:{name}')
                             for name in ['ws/src', 'docker', 'opt/3rdparty']},
                   'config_sha256': config_hashes,
                   'code_dirty': run('git', '-C', str(root), 'status', '--porcelain', '--',
                                     'ws/src', 'docker', 'opt/3rdparty')},
        'host': {'os_release': Path('/etc/os-release').read_text(), 'kernel': platform.release(),
                 'architecture': platform.machine(),
                 'cpu': [row for row in cpu if row['field'] in
                         ['CPU(s):', 'Model name:', 'Socket(s):', 'Core(s) per socket:',
                          'Thread(s) per core:']],
                 'memory_total': mem['MemTotal'].strip(),
                 'disk_bytes': {'total': usage.total, 'free': usage.free},
                 'ros_directories': sorted(p.name for p in Path('/opt/ros').iterdir()),
                 'workspace': {name: {'exists': (root / name).exists(),
                                      'is_symlink': (root / name).is_symlink()}
                               for name in ['ws/src/CMakeLists.txt', 'ws/build', 'ws/devel']},
                 'tool_packages': run('dpkg-query', '-W', '-f=${Package}\t${Version}\n',
                                      'gcc', 'g++', 'cmake', 'python3', 'git'),
                 'xauthority_is_directory': Path('/run/user/1000/gdm/Xauthority').is_dir()},
        'docker': {'version': json.loads(run('docker', 'version', '--format', '{{json .}}')),
                   'compose_version': run('docker', 'compose', 'version', '--short'),
                   'selected_env': selected_env, 'images': [inspect('image', ref) for ref in refs],
                   'existing_container': container, 'existing_container_copy': existing_copy,
                   'existing_container_metadata_unchanged': True},
        'image_baseline': {'image': image, 'specimen_id': specimen_id,
                           'specimen_never_started': True, 'specimen_removed': True,
                           'files': files, 'package_records': len(packages),
                           'installed_packages': sum(row['Status'] == 'install ok installed' for row in packages),
                           'packages_file': 'image-packages.tsv',
                           'packages_sha256': hashlib.sha256(package_text.encode()).hexdigest()},
        'unverified': ['existing_container_writable_layer_packages', 'named_volume_build_and_devel',
                       'runtime_python_import_resolution', 'image_build_source_commit',
                       'registry_digest_fetchability', 'apt_and_pip_artifact_availability',
                       'clean_rebuild', 'ros_gazebo_runtime', 'upstream_source_commits'],
    }
    snapshot['capture_completed_at_utc'] = datetime.now(timezone.utc).isoformat()
    (out / 'image-packages.tsv').write_text(package_text, encoding='utf-8')
    (out / 'environment-lock.json').write_text(json.dumps(snapshot, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'status': snapshot['status'], 'image': image['Id'],
                      'packages': len(packages), 'installed': snapshot['image_baseline']['installed_packages'],
                      'existing_container_copy_rc': existing_copy['rc'],
                      'specimen_removed': True}, ensure_ascii=False))


if __name__ == '__main__':
    main()
