"""Single place that resolves the repository root.

Precedence:
  1. ROS1_SLAM_ROOT environment variable (per-host override, also exported by
     scripts/env.sh so every child process sees the same root);
  2. the nearest repository or source/runtime-image marker (ws/src with note,
     or with scripts/build and tooling/build; no .git requirement);
  3. the git top level when no repository marker is found;
  4. /home/ros1-slam as the historical default.
"""
import os
import subprocess
from pathlib import Path

FALLBACK = Path('/home/ros1-slam')


def repo_root(start=None, allow_git=True):
    configured = os.environ.get('ROS1_SLAM_ROOT')
    if configured:
        return Path(configured).expanduser().resolve()
    here = Path(start or __file__).resolve()
    base = here if here.is_dir() else here.parent
    # App imports must not create a git subprocess before execution preflight.
    for parent in [base] + list(base.parents):
        if ((parent / 'ws' / 'src').is_dir()
                and ((parent / 'note').is_dir()
                     or ((parent / 'scripts/build').is_dir() and (parent / 'tooling/build').is_dir()))):
            return parent
    if not allow_git:
        return FALLBACK
    try:
        out = subprocess.run(['git', '-C', str(base), 'rev-parse', '--show-toplevel'],
                             capture_output=True, text=True, timeout=3)
        if out.returncode == 0 and out.stdout.strip():
            return Path(out.stdout.strip()).resolve()
    except (OSError, subprocess.SubprocessError):
        pass
    return FALLBACK
