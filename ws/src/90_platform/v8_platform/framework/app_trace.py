"""Opt-in bounded navigation observation; no ROS imports or runtime decisions."""
import json
import math
from pathlib import Path
import threading
import time


class Trace:
    def __init__(self, metadata, limit=250000):
        self.metadata = dict(metadata)
        self.limit = limit
        self.events = []
        self.errors = []
        self.overflow = False
        self.lock = threading.Lock()

    def record(self, kind, payload, ros_time, wall_time=None):
        # Check serializability now; malformed capture must not silently disappear.
        try:
            event = {'kind': kind, 'payload': payload, 'ros_time': float(ros_time),
                     'wall_monotonic': time.monotonic() if wall_time is None else float(wall_time)}
            json.dumps(event, allow_nan=False)
            if not all(math.isfinite(event[k]) for k in ('ros_time', 'wall_monotonic')):
                raise ValueError('nonfinite capture clock')
        except (TypeError, ValueError) as error:
            self.error(kind, error)
            return
        with self.lock:
            if len(self.events) >= self.limit:
                self.overflow = True
            else:
                self.events.append(event)

    def error(self, kind, error):
        with self.lock:
            if len(self.errors) < 100:
                self.errors.append({'kind': kind, 'error': repr(error)})

    def snapshot(self):
        with self.lock:
            return {'schema': 'navigation_app_trace/v1', 'metadata': self.metadata,
                    'events': list(self.events), 'errors': list(self.errors),
                    'overflow': self.overflow, 'capture_ok': not self.errors and not self.overflow}

    def write(self, path):
        value = self.snapshot()
        Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=1, allow_nan=False) + '\n',
                              encoding='utf-8')
        import shutil
        import subprocess
        if shutil.which('setfacl'):
            subprocess.run(['setfacl', '-m', 'u:ros1-slam:rwx', str(path)], check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return value


def prepare_logs(trace_path):
    """Allocate a fresh opt-in observation runtime, preserving all legacy logs."""
    import subprocess
    import shutil
    directory = Path(str(Path(trace_path).absolute()) + '.runtime')
    directory.mkdir()
    def permissions(path):
        if shutil.which('setfacl'):
            subprocess.run(['setfacl', '-m', 'u:ros1-slam:rwx', str(path)], check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if path.is_dir():
                subprocess.run(['setfacl', '-d', '-m', 'u:ros1-slam:rwx', str(path)], check=False,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    permissions(directory)
    home, logs = directory / 'ros_home', directory / 'ros_logs'
    for path in (home, logs):
        path.mkdir(); permissions(path)
    paths = {}
    for name in ('master', 'gazebo', 'viz', 'picker', 'rviz', 'tf_static', 'robot_state', 'joint_state', 'pose_tf'):
        path = directory / (name + '.log')
        path.touch(exist_ok=False); permissions(path)
        paths[name] = str(path)
    return paths, str(home), str(logs)
