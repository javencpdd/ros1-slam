#!/usr/bin/env bash
# x86 simulation build wrapper; target selection and cache ownership live in the helper.
# Usage: bash docker/build.sh [--plan | --clean] (WS_DIR/JOBS are explicit inputs).
set -eo pipefail

HERE="$(cd "$(/usr/bin/dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="${WS_ROOT:-${HERE%/*}}"
WS_DIR="${WS_DIR:-${WS_ROOT}/ws}"
JOBS="${JOBS:-4}"
ACTION=build
CLEAN=0
while [ "$#" -gt 0 ]; do
    case "$1" in
        --plan) ACTION=plan ;;
        --clean) CLEAN=1 ;;
        *) echo "[build] unknown argument: $1 (use --plan or --clean)" >&2; exit 2 ;;
    esac
    shift
done
if [ "$ACTION" = plan ] && [ "$CLEAN" = 1 ]; then
    echo "[build] --plan is read-only and cannot be combined with --clean" >&2
    exit 2
fi
if ! [[ "$JOBS" =~ ^[1-9][0-9]*$ ]]; then
    echo "[build] JOBS must be a positive integer" >&2
    exit 2
fi
case "${PROFILE:-sim}" in
    sim|dev) ;;
    *) echo "[build] x86_sim requires PROFILE=sim (dev is an alias); core is unsupported. Lite3 uses the native target helper." >&2; exit 2 ;;
esac
if [ "${SLIM:-0}" != 0 ]; then
    echo "[build] SLIM is retired; only SLIM=0 is accepted. Package roots are selected by profiles.json." >&2
    exit 2
fi
HELPER="${WS_ROOT}/scripts/build/target_workspace.py"
PROFILES="${WS_ROOT}/tooling/build/profiles.json"
if [ ! -f "$HELPER" ] || [ ! -f "$PROFILES" ]; then
    echo "[build] missing target helper or profiles under ${WS_ROOT}" >&2
    exit 2
fi
ARGS=("$HELPER" "$ACTION" --target x86_sim --workspace "$WS_DIR" --profiles "$PROFILES" --jobs "$JOBS")
if [ "$CLEAN" = 1 ]; then ARGS+=(--clean); fi
if [ "$ACTION" = plan ]; then
    exec python3 -B "${ARGS[@]}"
fi
if [ "${ROS_DISTRO:-noetic}" != noetic ] || [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "[build] ROS1 Noetic is required; --plan works on a host without ROS" >&2
    exit 2
fi
# Preserve system SDKs via default compiler paths/ldconfig, not ambient overlays.
# Only these inputs enter the compiler shell; workspace/jobs/flags stay in argv.
exec /usr/bin/env -i HOME="${HOME}" \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    LANG=C.UTF-8 LC_ALL=C.UTF-8 CPATH=/opt/ros/noetic/include \
    /bin/bash --noprofile --norc -c '
        set -eo pipefail
        source /opt/ros/noetic/setup.bash
        exec /usr/bin/python3 -B "$@"
    ' target-build "${ARGS[@]}"
