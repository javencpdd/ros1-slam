#!/usr/bin/env bash
# 把 /web 发布到宿主端口（默认 80），容器内跑 web/serve.py。
# 用法：bash web/publish.sh [端口]     例：bash web/publish.sh 8080
set -euo pipefail

PORT="${1:-80}"
NAME="${NAME:-ros1-slam-web}"
IMAGE="${IMAGE:-ros1-slam:noetic-sim}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"

docker rm -f "$NAME" >/dev/null 2>&1 || true
docker run -d --name "$NAME" --restart unless-stopped \
  -p "${PORT}:8000" \
  -v "${REPO}:/srv:ro" \
  --entrypoint python3 \
  "$IMAGE" -u /srv/web/serve.py --port 8000 --bind 0.0.0.0 >/dev/null

echo "published ${REPO}/web  ->  ${NAME} (host :${PORT})"
if [ "$PORT" = "80" ]; then
  echo "open  http://<本机IP>/"
else
  echo "open  http://<本机IP>:${PORT}/"
fi
echo "stop  docker stop ${NAME}   |   remove  docker rm -f ${NAME}"
