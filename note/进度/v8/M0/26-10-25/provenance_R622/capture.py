import hashlib
import json
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path


def git(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args], text=True).strip()


def metadata(path):
    tree = ET.parse(path).getroot()
    return {
        'name': tree.findtext('name'),
        'version': tree.findtext('version'),
        'licenses_declared': [e.text for e in tree.findall('license')],
        'urls_declared': [{'type': e.get('type'), 'text': e.text} for e in tree.findall('url')],
    }


def own_git(path):
    if not (path / '.git').exists():
        return {'status': 'no_own_git' if path.exists() else 'directory_absent'}
    top = Path(git(path, 'rev-parse', '--show-toplevel')).resolve()
    if top != path.resolve():
        return {'status': 'toplevel_mismatch'}
    return {'status': 'local_git_identity', 'head': git(path, 'rev-parse', 'HEAD'),
            'changes': git(path, 'status', '--porcelain', '--untracked-files=normal'),
            'remote_retrieval': 'not_tested', 'upstream_authenticity': 'not_verified'}


def collect(root):
    started = datetime.now(timezone.utc).isoformat()
    hashes = {}

    def evidence(path):
        relative = str(path.relative_to(root))
        hashes[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
        return relative

    source = root / 'note/03_外部文件来源清单.md'
    mappings = []
    for number, line in enumerate(source.read_text().splitlines(), 1):
        if not line.startswith('|'):
            continue
        for target in re.findall(r'/home/ros1-slam/(ws/src/[^（\s|`]+)', line):
            mappings.append((target.rstrip('/'), number))
    packages = []
    for path in sorted((root / 'ws/src').rglob('package.xml')):
        row = metadata(path)
        folder = path.parent
        relative = str(folder.relative_to(root))
        row.update(path=relative, metadata=evidence(path), build_entry=evidence(folder / 'CMakeLists.txt'),
                   main_repo_tree=git(root, 'rev-parse', 'HEAD:' + relative), own_git=own_git(folder))
        matches = [(p, n) for p, n in mappings if relative == p or relative.startswith(p + '/')]
        row['copy_record_lines'] = sorted({n for p, n in matches})
        row['copy_record_meaning'] = 'historical_record_not_current_equivalence_or_upstream_revision'
        candidates = []
        for parent in [folder, *folder.parents]:
            if not parent.is_relative_to(root):
                break
            for item in sorted(parent.iterdir()):
                if item.is_file() and re.match(r'^(LICENSE|COPYING|NOTICE)([._-].*)?$', item.name, re.I):
                    candidates.append(evidence(item))
        row['ancestor_license_candidates_not_grants'] = candidates
        row['upstream_revision_binding'] = 'not_established_by_this_capture'
        packages.append(row)
    references = {name: own_git(root / 'opt/upstream' / name) for name in (
        'Lite3_SLAM', 'Lite3_Navigation', 'deep-robotics-simulation', 'deep_robotics_model',
        'Mid360_simulation_plugin', 'livox_laser_simulation_Mid360', 'lvr2')}
    sdk = root / 'opt/3rdparty/Livox_SDK2'
    sdk_identity = own_git(sdk)
    sdk_identity['main_repo_tree'] = git(root, 'rev-parse', 'HEAD:opt/3rdparty/Livox_SDK2')
    selected = [
        'note/03_外部文件来源清单.md', 'docker/Dockerfile', 'docker/build.sh',
        'docker/docker-compose.yml', 'docker/catkin_build.sh', '.dockerignore',
        'ws/src/50_task/pipeline_tracking/scripts/TaskPoint.py',
        'ws/src/10_slam/faster_lio/LICENSE', 'ws/src/10_slam/faster_lio/src/laser_mapping.cc',
        'ws/src/10_slam/faster_lio/cmake/packages.cmake',
        'ws/src/95_simulation/Mid360_simulation_plugin/README.md',
        'ws/src/00_drivers/livox_ros_driver2/package_ROS1.xml',
        'ws/src/00_drivers/livox_ros_driver2/package_ROS2.xml',
        'opt/3rdparty/Livox_SDK2/README.md', 'opt/3rdparty/Livox_SDK2/CHANGELOG.md',
        'opt/3rdparty/Livox_SDK2/LICENSE.txt', 'opt/3rdparty/Livox_SDK2/sdk_core/CMakeLists.txt',
        'note/进度/v8/M0/26-10-25/environment_R617/environment-lock.json',
        'note/进度/v8/M0/26-10-25/environment_R617/image-packages.tsv',
    ]
    for path in selected:
        evidence(root / path)
    baseline = json.loads((root / selected[-2]).read_text())
    lines = (root / selected[-1]).read_text().splitlines()
    wanted = {'ros-noetic-' + name for name in (
        'move-base', 'move-base-flex', 'mbf-costmap-nav', 'global-planner', 'navfn',
        'base-local-planner', 'dwa-local-planner', 'teb-local-planner', 'map-server',
        'costmap-2d', 'spatio-temporal-voxel-layer', 'sob-layer')}
    binaries = [dict(zip(lines[0].split('\t'), line.split('\t'))) for line in lines[1:]
                if line.split('\t')[0] in wanted]
    result = {
        'status': 'offline_provenance_inventory_not_license_or_runtime_acceptance',
        'started_at_utc': started,
        'main_repo_head': git(root, 'rev-parse', 'HEAD'),
        'source_changes': git(root, 'status', '--porcelain', '--untracked-files=normal', '--', 'ws/src', 'opt/3rdparty', 'docker'),
        'tree_semantics': 'main_repo_tree identifies tracked HEAD content only; not ignored files, upstream identity or build-context lock',
        'packages': packages, 'reference_repositories': references, 'livox_sdk2': sdk_identity,
        'historical_binary_baseline': {'captured_at_utc': baseline['captured_at_utc'],
            'images': baseline['docker']['images'], 'selected_packages': binaries,
            'meaning': 'R617 evidence re-read; no current Docker, source-package, copyright or retrieval observation'},
        'source_sha256': hashes,
        'not_executed': ['network_fetch', 'docker', 'build', 'ros', 'gazebo', 'hardware'],
    }
    result['completed_at_utc'] = datetime.now(timezone.utc).isoformat()
    return result


if __name__ == '__main__':
    print(json.dumps(collect(Path(sys.argv[1]).resolve()), ensure_ascii=False, indent=2))
