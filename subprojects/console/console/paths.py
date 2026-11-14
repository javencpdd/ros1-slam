"""Repository roots for the standalone console repo (host-only)."""
from __future__ import annotations

import os
from pathlib import Path

CONSOLE_DIR = Path(__file__).resolve().parent
P3_ROOT = CONSOLE_DIR.parent
_PARENT = P3_ROOT.parent

REPO_ROOT = Path(os.environ.get('V8_REPO_ROOT') or (_PARENT / 'ros1-slam')).resolve()
SIM_ROOT = Path(os.environ.get('V8_SIM_ROOT') or (_PARENT / 'ros1-slam-simulation')).resolve()
CONTAINER_ROOT = os.environ.get('V8_CONTAINER_ROOT', '/opt/ros1-slam')
DATA_DIR = Path(os.environ.get('V8_CONSOLE_DATA') or (REPO_ROOT / 'data' / 'console'))
RUN_DIR = Path(os.environ.get('V8_CONSOLE_RUN') or '/tmp/v8-console')
FOXGLOVE_DIR = Path(os.environ.get('V8_FOXGLOVE_DIR') or (REPO_ROOT / 'docker' / 'foxglove'))
