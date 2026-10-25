import hashlib
from pathlib import Path
import shlex
import sys

import probe

original = probe.create_args


def create_args(token):
    args = original(token)
    code = ('from pathlib import Path; '
            'paths=["/home/ros1-slam", "/opt/ros1-slam", "/home/ros1-slam/ws", '
            '"/home/ros1-slam/ws/src", "/home/ros1-slam/ws/src/40_navigation", '
            '"/opt/ros1-slam/ws/src/40_navigation"]; '
            'print([(p,Path(p).is_dir(),Path(p).is_symlink(),str(Path(p).resolve())) for p in paths])')
    args[-1] = 'python3 -B -c ' + shlex.quote(code)
    return args


if __name__ == '__main__':
    print('path_driver_sha256=' + hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    probe.create_args = create_args
    probe.accepted = lambda data: False
    sys.exit(probe.main(Path(sys.argv[1])))
