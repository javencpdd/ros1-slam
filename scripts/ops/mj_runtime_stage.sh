#!/usr/bin/env bash
# =============================================================================
# mj_runtime_stage.sh —— 把一个容器里的 MuJoCo/ORT Python 运行时搬到另一个容器
#
#   scripts/ops/mj_runtime_stage.sh --target ros1-slam-sim-viz
#   scripts/ops/mj_runtime_stage.sh --source v8codex-20261028 --target my_foxy_env --dest /tmp/mjros2
#
# 为什么不是直接装进镜像：当前镜像没有 mujoco/onnxruntime 的离线 wheel；本脚本用
# docker cp 从已有运行时容器搬运，宿主机只做中转（容器之间 docker cp 不支持）。
# 目标容器里用 PYTHONPATH=<dest> 生效；镜像固化仍待有网络时补 Dockerfile。
# =============================================================================
set -euo pipefail

SOURCE=${SOURCE:-v8codex-20261028}
TARGET=${TARGET:-}
DEST=${DEST:-/tmp/mjrt}
WITH_NUMPY=${WITH_NUMPY:-auto}

die() { echo "[mj-runtime] $*" >&2; exit 2; }

while [ $# -gt 0 ]; do
    case "$1" in
        --source) SOURCE=$2; shift 2 ;;
        --target) TARGET=$2; shift 2 ;;
        --dest) DEST=$2; shift 2 ;;
        --with-numpy) WITH_NUMPY=1; shift ;;
        --no-numpy) WITH_NUMPY=0; shift ;;
        -h|--help) sed -n '1,20p' "$0"; exit 0 ;;
        *) die "unknown argument: $1" ;;
    esac
done
[ -n "$TARGET" ] || die "--target container is required"
docker inspect "$SOURCE" >/dev/null 2>&1 || die "source container not found: $SOURCE"
docker inspect "$TARGET" >/dev/null 2>&1 || die "target container not found: $TARGET"

STAGE=$(mktemp -d /tmp/mj-runtime-XXXXXX)
trap 'rm -rf "$STAGE"' EXIT

echo "[mj-runtime] staging from $SOURCE to $TARGET:$DEST"
docker exec "$TARGET" mkdir -p "$DEST"

copy_path() {
    local path=$1 name=$2
    docker cp "$SOURCE:$path" "$STAGE/$name" >/dev/null
    docker cp "$STAGE/$name" "$TARGET:$DEST/$name" >/dev/null
    echo "  + $name"
}

copy_path /usr/local/lib/python3.8/dist-packages/mujoco mujoco
copy_path /usr/local/lib/python3.8/dist-packages/glfw glfw
if docker exec "$SOURCE" test -d /tmp/r854-policy-runtime/onnxruntime; then
    copy_path /tmp/r854-policy-runtime/onnxruntime onnxruntime
else
    echo "  ! onnxruntime not found in $SOURCE (skip)"
fi

if [ "$WITH_NUMPY" = 1 ] || [ "$WITH_NUMPY" = auto ]; then
    # 仅当目标 numpy 低于 mujoco 需要的 ABI 时搬运；用 import 探测。
    if ! docker exec "$TARGET" bash -lc "PYTHONPATH=$DEST python3 -c 'import mujoco' >/dev/null 2>&1"; then
        copy_path /usr/local/lib/python3.8/dist-packages/numpy numpy
        docker cp "$SOURCE:/usr/local/lib/python3.8/dist-packages/numpy.libs" "$STAGE/numpy.libs" >/dev/null
        docker cp "$STAGE/numpy.libs" "$TARGET:$DEST/numpy.libs" >/dev/null
        echo "  + numpy + numpy.libs"
    fi
fi

docker exec "$TARGET" bash -lc "PYTHONPATH=$DEST python3 -c 'import mujoco; print(\"mujoco\", mujoco.__version__)'"
docker exec "$TARGET" bash -lc "PYTHONPATH=$DEST python3 -c 'import onnxruntime; print(\"onnxruntime\", onnxruntime.__version__)'" 2>/dev/null || \
    echo "  ! onnxruntime import failed (optional)"
echo "[mj-runtime] done: export PYTHONPATH=$DEST:\$PYTHONPATH"
