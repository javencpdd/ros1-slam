"""Single place that resolves the repository root.

Precedence:
  1. ROS1_SLAM_ROOT environment variable (per-host override, also exported by
     scripts/env.sh so every child process sees the same root);
  2. the git top level of this file (works in a container and on the Jetson);
  3. the nearest parent that looks like this repository (ws/src + note);
  4. /home/ros1-slam as the historical default.
"""
import os
import subprocess
from pathlib import Path

FALLBACK = Path('/home/ros1-slam')


def repo_root(start=None):
    configured = os.environ.get('ROS1_SLAM_ROOT')
    if configured:
        return Path(configured).expanduser().resolve()
    here = Path(start or __file__).resolve()
    base = here if here.is_dir() else here.parent
    try:
        out = subprocess.run(['git', '-C', str(base), 'rev-parse', '--show-toplevel'],
                             capture_output=True, text=True, timeout=3)
        if out.returncode == 0 and out.stdout.strip():
            return Path(out.stdout.strip()).resolve()
    except (OSError, subprocess.SubprocessError):
        pass
    for parent in [base] + list(base.parents):
        if (parent / 'ws' / 'src').is_dir() and (parent / 'note').is_dir():
            return parent
    return FALLBACK
