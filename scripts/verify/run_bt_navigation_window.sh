#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="${1:?evidence directory required}"
SENTINEL=0
finish() { rc=$?; if [ "$SENTINEL" = 0 ]; then printf 'BT_NATIVE_RUNTIME_RC=%s\n' "$rc"; fi; }
trap finish EXIT
LOCAL_PLANNER="${2:-teb}"
GLOBAL_PLANNER="${3:-global_planner}"
ELEVATION_SOURCE="${4:-native}"
ELEVATION_TTL="${5:-1.0}"
GOAL_DISTANCE="${6:-1.0}"
LAUNCH_ARGS="local_planner:=$LOCAL_PLANNER global_planner:=$GLOBAL_PLANNER elevation_source:=$ELEVATION_SOURCE elevation_ttl_s:=$ELEVATION_TTL"
DRIVER_ARGS="--local-planner $LOCAL_PLANNER --global-planner $GLOBAL_PLANNER --goal-distance $GOAL_DISTANCE"
[ -d "$OUT" ] || exit 9
[ ! -e "$OUT/runtime.json" ] || exit 9
bash "$ROOT/scripts/verify/v8_window_preflight.sh" > "$OUT/preflight.log" 2>&1 || exit 5
python3 - <<'PY'
import socket
for port in (11313, 11345):
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', port))
PY
set +u
source /opt/ros/noetic/setup.bash
source "$ROOT/ws/devel/setup.bash"
[ -n "${TEB_OVERLAY:-}" ] && source "$TEB_OVERLAY/devel/setup.bash"
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
export PYTHONNOUSERSITE=1
export ROS_HOME="$OUT/ros_home"
mkdir -p "$ROS_HOME"
LAUNCH_PID=""
cleanup() {
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID"
    wait "$LAUNCH_PID" || true
  fi
}
trap cleanup EXIT
sha256sum "$ROOT/ws/devel/lib/lite3_bt/bt_console" "$ROOT/ws/src/90_platform/lite3_bt/scripts/navigation_executor.py" > "$OUT/runtime_fingerprints.txt"
roslaunch lite3_bringup mode_sim_teb.launch $LAUNCH_ARGS > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
python3 - <<'PY'
import socket,time,xmlrpc.client
socket.setdefaulttimeout(1.0)
master=xmlrpc.client.ServerProxy('http://127.0.0.1:11313')
end=time.monotonic()+60
while time.monotonic()<end:
    try:
        result=master.getParam('/bt_runtime_preflight','/use_sim_time')
        if result[0]==1 and result[2] is True:
            break
    except (OSError, xmlrpc.client.Error):
        pass
    time.sleep(0.1)
else:
    raise SystemExit('simulation master did not become ready')
PY
set +e
python3 "$ROOT/scripts/verify/test_bt_navigation_runtime.py" --output "$OUT/runtime.json" --lease-loss $DRIVER_ARGS > "$OUT/driver.log" 2>&1
RC=$?
set -e
cleanup
LAUNCH_PID=""
python3 - <<'PY'
import socket
for port in (11313, 11345):
    with socket.socket() as sock:
        assert sock.connect_ex(('127.0.0.1', port)) != 0, 'port still listening: %s' % port
print('CLEANUP_PORTS_FREE')
PY
printf 'BT_NATIVE_RUNTIME_RC=%s\n' "$RC"
SENTINEL=1
exit "$RC"
