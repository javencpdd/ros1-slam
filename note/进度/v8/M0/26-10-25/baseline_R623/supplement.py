import hashlib
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys
from datetime import datetime, timezone

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
spec = importlib.util.spec_from_file_location('capture', HERE / 'capture.py')
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def sha(path):
    with path.open('rb') as f:
        return capture.digest(f)


def collect():
    result = {'started_at_utc': capture.now()}
    dockerfile = ROOT / 'docker/Dockerfile'
    copies = []
    for i, line in enumerate(dockerfile.read_text().splitlines(), 1):
        if re.match(r'^(COPY|ADD)\s', line):
            copies.append({'line': i, 'text': line, 'scope': 'stage' if '--from=' in line else 'host'})
    result['docker_copy_instructions'] = copies
    result['dockerignore'] = {'sha256': sha(ROOT / '.dockerignore'),
        'excluded_toplevel_cmake_line': 18,
        'scope': 'raw inventory does not implement Docker matching; effective context unverified'}
    package = ROOT / 'ws/src/30_perception/deeprobotics_local_height_map'
    cmake = package / 'CMakeLists.txt'
    paths = []
    for i, line in enumerate(cmake.read_text().splitlines(), 1):
        clean = line.split('#', 1)[0].strip()
        if re.fullmatch(r'(src|test)/[A-Za-z0-9_./]+\.cpp', clean):
            paths.append({'line': i, 'path': str((package / clean).relative_to(ROOT)),
                          'is_file': (package / clean).is_file(),
                          'class': 'test' if clean.startswith('test/') else 'production'})
    result['height_map'] = {'cmake_sha256': sha(cmake), 'declared_sources': paths,
        'src_exists': (package / 'src').exists(), 'include_exists': (package / 'include').exists(),
        'catkin_ignore': (package / 'CATKIN_IGNORE').is_file(),
        'catkin_ignore_size': (package / 'CATKIN_IGNORE').stat().st_size,
        'source_files': [str(p.relative_to(ROOT)) for p in package.rglob('*')
                         if p.suffix.lower() in {'.cpp', '.cc', '.c', '.h', '.hpp', '.py'} and p.is_file()]}
    packages = []
    for path in sorted((ROOT / 'ws/src').rglob('package.xml')):
        ancestors = [path.parent, *path.parent.parents]
        ignores = [str((p / 'CATKIN_IGNORE').relative_to(ROOT)) for p in ancestors
                   if p.is_relative_to(ROOT / 'ws/src') and (p / 'CATKIN_IGNORE').is_file()]
        packages.append({'path': str(path.relative_to(ROOT)), 'ignore_markers': ignores})
    result['package_ignore_inventory'] = packages
    result['existing_writable_layer_diff'] = capture.run(['docker', 'diff', 'ros1-slam-sim'])
    result['volume_metadata'] = json.loads(capture.checked(['docker', 'volume', 'inspect',
        'docker_ros1_ws_build', 'docker_ros1_ws_devel', 'docker_ros1_logs', 'docker_ros1_ros_home']))
    xauth = Path('/run/user/1000/gdm/Xauthority')
    result['xauthority_host_type'] = {'path': str(xauth), 'is_file': xauth.is_file(), 'is_dir': xauth.is_dir()}
    result['source_status'] = capture.run(['git', 'status', '--porcelain', '--untracked-files=normal', '--',
                                         'ws/src', 'docker', 'opt/3rdparty'])
    result['temporary_containers_remaining'] = capture.run(['docker', 'ps', '-a', '--filter',
        'label=ros1-slam.evidence=R623', '--format', '{{.ID}}'])
    result['completed_at_utc'] = capture.now()
    return result


if __name__ == '__main__':
    output = Path(sys.argv[1])
    data = collect()
    with output.open('x') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write('\n')
    subprocess.run(['setfacl', '-m', 'u:ros1-slam:rwx', str(output)], check=True)
    print(json.dumps({'copy_instructions': data['docker_copy_instructions'],
        'height_map': data['height_map'], 'xauthority_host_type': data['xauthority_host_type'],
        'writable_diff_rc': data['existing_writable_layer_diff']['rc'],
        'writable_diff_lines': len(data['existing_writable_layer_diff']['stdout'].splitlines()),
        'source_status': data['source_status']['stdout'],
        'temporary_remaining': data['temporary_containers_remaining']['stdout']}, ensure_ascii=False, indent=2))
