#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
cd /opt/ros1-slam/r625/ws
test ! -e build/CMakeCache.txt
test ! -e devel/lib/liblite3_costmap_layers.so
catkin_make -j2 -l2 -DROS_EDITION=ROS1 -DCATKIN_WHITELIST_PACKAGES= -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
python3 -B - <<'PY'
import hashlib, json, os, subprocess
from pathlib import Path
import numpy, rospy
lib = Path('/opt/ros1-slam/r625/ws/devel/lib/liblite3_costmap_layers.so')
assert lib.read_bytes()[:4] == b'\x7fELF'
ldd = subprocess.run(['ldd', str(lib)], capture_output=True, text=True, check=True).stdout
assert 'not found' not in ldd
result = {'library': str(lib), 'sha256': hashlib.sha256(lib.read_bytes()).hexdigest(),
          'ldd': ldd, 'numpy': [numpy.__version__, numpy.__file__], 'rospy': rospy.__file__,
          'environment': {k: os.environ.get(k) for k in ('CMAKE_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH', 'ROS_PACKAGE_PATH')},
          'cache': Path('build/CMakeCache.txt').read_text()}
Path('/opt/ros1-slam/r625/build-result.json').write_text(json.dumps(result, indent=2)+'\n')
print('BUILD_ARTIFACT', result['sha256'])
PY
