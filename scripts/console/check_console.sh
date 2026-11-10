#!/usr/bin/env bash
# Console self-check (host). Read-only except starting nothing.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
FAIL=0
ok() { printf 'PASS  %s\n' "$1"; }
bad() { printf 'FAIL  %s\n' "$1"; FAIL=1; }

command -v docker >/dev/null && ok 'docker' || bad 'docker'
for name in ros1-slam-roscore ros1-slam-rosbridge; do
  if docker ps --format '{{.Names}}' | grep -qx "$name"; then ok "container $name"; else bad "container $name"; fi
done
for path in web/console.html web/assets/console/console.js web/assets/console/rosbridge.js \
            web/assets/data/maps.json web/assets/data/3d_runs.json \
            web/assets/data/worlds/worlds.json; do
  [[ -f "$ROOT/$path" ]] && ok "$path" || bad "$path"
done
if curl -fsS --max-time 2 http://127.0.0.1:8780/api/health >/dev/null 2>&1; then
  ok 'agent /api/health'
  curl -fsS --max-time 3 http://127.0.0.1:8780/api/stacks >/dev/null 2>&1 \
    && ok 'agent /api/stacks' || bad 'agent /api/stacks'
  curl -fsS --max-time 2 http://127.0.0.1:8780/api/whoami >/dev/null 2>&1 \
    && ok 'agent /api/whoami' || bad 'agent /api/whoami'
  if curl -fsS --max-time 2 http://127.0.0.1:8780/api/voxels/meta >/dev/null 2>&1; then
    ok 'agent /api/voxels/meta'
    curl -fsS --max-time 2 http://127.0.0.1:8780/api/voxels/latest >/dev/null 2>&1 \
      && ok 'agent /api/voxels/latest' || bad 'agent /api/voxels/latest'
  else
    echo 'SKIP  agent /api/voxels（voxel_stream 未运行或无快照）'
  fi
else
  bad 'agent /api/health（bash scripts/console/start_agent.sh --start）'
fi
for url in http://127.0.0.1/web/console.html http://127.0.0.1/web/assets/data/maps.json \
           http://127.0.0.1/web/assets/data/3d_runs.json \
           http://127.0.0.1/web/assets/data/worlds/worlds.json; do
  code=$(curl -s -o /dev/null -w '%{http_code}' --max-time 3 "$url" || true)
  [[ "$code" == 200 ]] && ok "$url" || bad "$url -> $code"
done
if command -v node >/dev/null; then
  node -e "new Function(require('fs').readFileSync('$ROOT/web/assets/console/console.js','utf8'))" \
    && ok 'console.js syntax' || bad 'console.js syntax'
fi
python3 -m py_compile "$ROOT/scripts/console/agent.py" "$ROOT/scripts/console/voxel_stream.py" \
  "$ROOT/scripts/ops/collect_3d_runs.py" "$ROOT/scripts/ops/build_maps_index.py" \
  "$ROOT/scripts/ops/build_world_scenes.py" "$ROOT/scripts/ops/regrid_map.py" \
  "$ROOT/scripts/ops/world_registry.py" \
  "$ROOT/ws/src/90_platform/lite3_bt/scripts/goal_gate.py" \
  "$ROOT/ws/src/90_platform/lite3_bt/scripts/goal_gate_policy.py" \
  "$ROOT/ws/src/90_platform/lite3_bt/scripts/test_goal_gate_policy.py" \
  && ok 'python syntax' || bad 'python syntax'
python3 -B -m unittest discover -s "$ROOT/ws/src/90_platform/lite3_bt/scripts" \
  -p 'test_goal_gate_policy.py' >/dev/null 2>&1 \
  && ok 'goal gate ACL policy + wiring' || bad 'goal gate ACL policy + wiring'
BT_TEST_BIN="${HOME}/.cache/lite3_bt_goal_queue_test"
mkdir -p "${HOME}/.cache" 2>/dev/null || true
g++ -std=c++11 -Wall -Wextra -I"$ROOT/ws/src/90_platform/lite3_bt/include" \
  "$ROOT/ws/src/90_platform/lite3_bt/test/test_goal_queue.cpp" -o "$BT_TEST_BIN" \
  >/dev/null 2>&1 && "$BT_TEST_BIN" >/dev/null 2>&1 \
  && ok 'goal queue C++ tests' || bad 'goal queue C++ tests'
if docker ps --format '{{.Names}}' | grep -qx ros1-slam-sim-viz; then
  docker exec ros1-slam-sim-viz bash -c 'source /opt/ros/noetic/setup.bash && B=/opt/ros1-slam && g++ -std=c++14 -I$B/opt/btcpp_v3/include $B/ws/src/90_platform/lite3_bt/test/test_tree_load.cpp -L$B/opt/btcpp_v3/lib -lbehaviortree_cpp_v3 -Wl,-rpath,$B/opt/btcpp_v3/lib -pthread -o /tmp/lite3_bt_tree_load && /tmp/lite3_bt_tree_load $B/ws/src/90_platform/lite3_bt/trees/navigate_with_planner.xml && /tmp/lite3_bt_tree_load $B/ws/src/90_platform/lite3_bt/trees/navigate_single_goal.xml' \
    >/dev/null 2>&1 && ok 'BT XML tree load (BT.CPP)' || bad 'BT XML tree load (BT.CPP)'
else
  printf 'SKIP  BT XML tree load (sim-viz 未运行；未验不等于通过)\n'
fi
python3 "$ROOT/scripts/ops/world_registry.py" --check >/dev/null 2>&1 \
  && ok 'world registry' || bad 'world registry'
if [[ -f "$ROOT/opt/btcpp_v3/lib/libbehaviortree_cpp_v3.so" ]]; then
  bash "$ROOT/scripts/ops/bootstrap_bt_demo.sh" --check >/dev/null 2>&1 \
    && ok 'BT demo bootstrap' || bad 'BT demo bootstrap（bash scripts/ops/bootstrap_bt_demo.sh --build）'
else
  echo 'SKIP  BT demo bootstrap（未构建；scripts/ops/bootstrap_bt_demo.sh --build）'
fi
python3 - "$ROOT" <<'PY'
import json, os, sys
root = sys.argv[1]
for path in ('web/assets/data/maps.json', 'web/assets/data/3d_runs.json',
             'web/assets/data/worlds/worlds.json'):
    data = json.load(open(os.path.join(root, path), encoding='utf-8'))
    assert data.get('schema'), path
print('PASS  json schema')
PY
[[ $FAIL -eq 0 ]] && echo 'ALL PASS' || echo 'HAS FAILURES'
exit $FAIL
