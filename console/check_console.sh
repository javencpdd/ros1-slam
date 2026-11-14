#!/usr/bin/env bash
# Console self-check (host, standalone P3). Read-only: starts nothing.
# P1/P2 integration checks run only when the sibling repos are present.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # P3 repo root
REPO_ROOT="${V8_REPO_ROOT:-$ROOT/../ros1-slam}"
SIM_ROOT="${V8_SIM_ROOT:-$ROOT/../ros1-slam-simulation}"
export V8_REPO_ROOT="$REPO_ROOT" V8_SIM_ROOT="$SIM_ROOT"
FAIL=0
ok() { printf 'PASS  %s\n' "$1"; }
bad() { printf 'FAIL  %s\n' "$1"; FAIL=1; }

command -v docker >/dev/null && ok 'docker' || bad 'docker'
for name in ros1-slam-roscore ros1-slam-rosbridge; do
  if docker ps --format '{{.Names}}' | grep -qx "$name"; then ok "container $name"; else bad "container $name"; fi
done
for path in console/ui/console.html console/ui/assets/console/console.js \
            console/ui/assets/console/rosbridge.js console/ui/assets/data/maps.json \
            console/ui/assets/data/worlds/worlds.json; do
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
  bad 'agent /api/health（bash console/start_agent.sh --start）'
fi
for url in http://127.0.0.1:8780/console.html http://127.0.0.1:8780/assets/data/maps.json \
           http://127.0.0.1:8780/assets/data/worlds/worlds.json; do
  code=$(curl -s -o /dev/null -w '%{http_code}' --max-time 3 "$url" || true)
  [[ "$code" == 200 ]] && ok "$url" || bad "$url -> $code"
done
if command -v node >/dev/null; then
  node -e "new Function(require('fs').readFileSync('$ROOT/console/ui/assets/console/console.js','utf8'))" \
    && ok 'console.js syntax' || bad 'console.js syntax'
  node -e "new Function(require('fs').readFileSync('$ROOT/console/ui/assets/console/mission_events.js','utf8'))" \
    && ok 'mission_events.js syntax' || bad 'mission_events.js syntax'
  node "$ROOT/console/test_mission_events.js" >/dev/null 2>&1 \
    && ok 'mission events tests' || bad 'mission events tests'
fi
python3 -m py_compile "$ROOT/console/agent.py" "$ROOT/console/voxel_stream.py" \
  "$ROOT/console/ops/build_maps_index.py" "$ROOT/console/ops/build_world_scenes.py" \
  "$ROOT/console/ops/world_registry.py" \
  && ok 'python syntax' || bad 'python syntax'
python3 "$ROOT/console/ops/world_registry.py" --check >/dev/null 2>&1 \
  && ok 'world registry' || bad 'world registry'
python3 - "$ROOT" <<'PY'
import json, os, sys
root = sys.argv[1]
for path in ('console/ui/assets/data/maps.json', 'console/ui/assets/data/worlds/worlds.json'):
    data = json.load(open(os.path.join(root, path), encoding='utf-8'))
    assert data.get('schema'), path
print('PASS  json schema')
PY
# --- P1 integration checks (only when the platform repo is beside us) ---
if [[ -d "$REPO_ROOT/ws/src/90_platform/lite3_bt" ]]; then
  python3 "$ROOT/console/test_bt_config.py" >/dev/null 2>&1 \
    && ok 'BT config profile keys' || bad 'BT config profile keys'
  python3 - "$REPO_ROOT" <<'PY'
import json, os, sys
root = sys.argv[1]
path = os.path.join(root, 'web/assets/data/3d_runs.json')
if os.path.exists(path):
    assert json.load(open(path, encoding='utf-8')).get('schema')
    print('PASS  web 3d_runs schema')
else:
    print('SKIP  web 3d_runs')
PY
else
  echo 'SKIP  P1 integration（lite3_bt 不在 V8_REPO_ROOT）'
fi
[[ $FAIL -eq 0 ]] && echo 'ALL PASS' || echo 'HAS FAILURES'
exit $FAIL
