#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
cd /opt/ros1-slam/r633/ws
test ! -e build/CMakeCache.txt
test ! -e devel/setup.bash
catkin_make -j2 -l2 -DCMAKE_BUILD_TYPE=Release > /opt/ros1-slam/r633/logs/build.log 2>&1
tail -5 /opt/ros1-slam/r633/logs/build.log
source devel/setup.bash
python3 -B - <<'PY'
import hashlib, json, subprocess
from pathlib import Path
ws = Path('/opt/ros1-slam/r633/ws')
node = ws / 'devel/lib/scan_planner/scan_planner_node'
assert node.is_file() and node.read_bytes()[:4] == b'\x7fELF', str(node)
ldd = subprocess.run(['ldd', str(node)], capture_output=True, text=True, check=True).stdout
assert 'not found' not in ldd
libs = sorted((ws / 'devel/lib').glob('*.so'))
result = {
    'node': str(node.relative_to(ws)), 'node_sha256': hashlib.sha256(node.read_bytes()).hexdigest(),
    'libs': {str(p.relative_to(ws)): hashlib.sha256(p.read_bytes()).hexdigest() for p in libs},
    'ldd_scan_planner_node': ldd,
    'rospack_scan_planner': subprocess.run(['rospack', 'find', 'scan_planner'],
                                           capture_output=True, text=True).stdout.strip(),
}
Path('/opt/ros1-slam/r633/build-result.json').write_text(json.dumps(result, indent=2) + '\n')
print('BUILD_ARTIFACT', result['node_sha256'], 'libs', len(result['libs']))
PY
