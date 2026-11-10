#!/usr/bin/env bash
# Reproducible bootstrap/check for the host-only BehaviorTree/Groot2 demo.
#
#   bash scripts/ops/bootstrap_bt_demo.sh --check
#   bash scripts/ops/bootstrap_bt_demo.sh --build   # build BT.CPP 3.8.8 + lite3_bt in sim-viz
#   bash scripts/ops/bootstrap_bt_demo.sh --groot2 # fetch/extract Groot2 1.9.0 on the host
#   bash scripts/ops/bootstrap_bt_demo.sh --all
#
# BT.CPP is built inside ros1-slam-sim-viz because the host libstdc++ may differ
# from the container. The library and catkin binary are installed under the
# mounted opt/btcpp_v3 and ws/devel, so a container recreate does not require a
# network download (but may require the catkin rebuild step).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CONTAINER="${BT_CONTAINER:-ros1-slam-sim-viz}"
BTCPP_TAG='3.8.8'
BTCPP_SHA='7b87e4819f0c482639c30fa6a6f06ab4c83c10e3362032cfc3beff4d827205af'
BTCPP_URL="https://gh-proxy.com/https://github.com/BehaviorTree/BehaviorTree.CPP/archive/refs/tags/${BTCPP_TAG}.tar.gz"
GROOT2_VERSION='1.9.0'
GROOT2_SHA='26d040255cb1462c6511954f345ee73280d72c7f88a2f9d13bbaff9902de6c70'
GROOT2_URL="https://pub-32cef6782a9e411e82222dee82af193e.r2.dev/Groot2-v${GROOT2_VERSION}-x86_64.AppImage"
GROOT2_DIR="${GROOT2_DIR:-$HOME/.cache/groot2}"
BT_LIB="$ROOT/opt/btcpp_v3/lib/libbehaviortree_cpp_v3.so"
BT_BIN_CONTAINER='/opt/ros1-slam/ws/devel/lib/lite3_bt/bt_console'

ok() { printf 'PASS  %s\n' "$1"; }
warn() { printf 'WARN  %s\n' "$1"; }
bad() { printf 'FAIL  %s\n' "$1"; FAIL=1; }
FAIL=0

container_running() {
  docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER"
}

check() {
  if [[ -f "$BT_LIB" ]]; then ok "BT.CPP library $BT_LIB"; else bad "BT.CPP library missing（先 --build）"; fi
  if container_running; then
    if docker exec "$CONTAINER" test -x "$BT_BIN_CONTAINER" 2>/dev/null; then
      ok "bt_console binary"
    else
      bad "bt_console binary missing（先 --build）"
    fi
  else
    bad "container $CONTAINER is not running"
  fi
  if [[ -x "$GROOT2_DIR/squashfs-root/AppRun" ]]; then
    ok "Groot2 $GROOT2_VERSION extracted"
  else
    warn "Groot2 not extracted（宿主执行 --groot2；仅本机 GUI）"
  fi
  [[ $FAIL -eq 0 ]]
}

build() {
  if ! container_running; then
    echo "container $CONTAINER is not running" >&2
    return 1
  fi
  if [[ -f "$BT_LIB" && "${BT_FORCE:-0}" != "1" ]]; then
    ok 'BT.CPP library already installed (set BT_FORCE=1 to rebuild)'
  else
    docker exec "$CONTAINER" bash -lc "
      set -e
      cd /tmp
      rm -rf btcpp-bootstrap btcpp-${BTCPP_TAG}.tar.gz
      curl -L --fail --retry 2 -o btcpp-${BTCPP_TAG}.tar.gz '${BTCPP_URL}'
      echo '${BTCPP_SHA}  btcpp-${BTCPP_TAG}.tar.gz' | sha256sum -c -
      tar xzf btcpp-${BTCPP_TAG}.tar.gz
      cd BehaviorTree.CPP-${BTCPP_TAG}
      rm -rf build
      cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/ros1-slam/opt/btcpp_v3 \
        -DBUILD_EXAMPLES=OFF -DBUILD_UNIT_TESTS=OFF -DBUILD_TOOLS=OFF \
        -DBUILD_MANUAL_SELECTOR=OFF
      cmake --build build -j4
      cmake --install build
    "
    ok 'BT.CPP installed'
  fi
  docker exec "$CONTAINER" bash -lc "
    set -e
    source /opt/ros/noetic/setup.bash
    cd /opt/ros1-slam/ws
    catkin_make --pkg lite3_bt -DCMAKE_BUILD_TYPE=Release \
      -Dbehaviortree_cpp_v3_DIR=/opt/ros1-slam/opt/btcpp_v3/lib/cmake/behaviortree_cpp_v3
  "
  ok 'lite3_bt built'
}

fetch_groot2() {
  mkdir -p "$GROOT2_DIR"
  if [[ -x "$GROOT2_DIR/squashfs-root/AppRun" ]]; then
    ok "Groot2 already extracted at $GROOT2_DIR/squashfs-root/AppRun"
    return 0
  fi
  local image="$GROOT2_DIR/Groot2-v${GROOT2_VERSION}-x86_64.AppImage"
  if [[ ! -f "$image" ]]; then
    if ! curl -L --fail --retry 2 -o "$image" "$GROOT2_URL"; then
      echo "Groot2 download failed; set HTTPS_PROXY (host proxy) or predownload $GROOT2_URL" >&2
      return 1
    fi
  fi
  echo "${GROOT2_SHA}  $image" | sha256sum -c -
  chmod +x "$image"
  (cd "$GROOT2_DIR" && "$image" --appimage-extract >/dev/null)
  [[ -x "$GROOT2_DIR/squashfs-root/AppRun" ]] || {
    echo "Groot2 extraction failed" >&2
    return 1
  }
  ok "Groot2 $GROOT2_VERSION extracted at $GROOT2_DIR/squashfs-root/AppRun"
}

case "${1:---check}" in
  --check) check ;;
  --build) build ;;
  --groot2) fetch_groot2 ;;
  --all) build; fetch_groot2 ;;
  *) echo "usage: $0 [--check|--build|--groot2|--all]" >&2; exit 2 ;;
esac
