#!/usr/bin/env bash
# Builder and pure-image compatibility entry; all builds use the x86_sim wrapper.
set -eo pipefail
HERE="$(cd "$(/usr/bin/dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_ROOT="${HERE%/*}"
if [ ! -f "${DEFAULT_ROOT}/scripts/build/target_workspace.py" ]; then
    DEFAULT_ROOT=/opt/ros1-slam
fi
WS_ROOT="${WS_ROOT:-$DEFAULT_ROOT}"
export WS_ROOT
exec bash "${WS_ROOT}/docker/build.sh" "$@"
