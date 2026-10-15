#!/usr/bin/env bash
# 把 tooling/skill 镜像恢复到工具真实读取路径（两处：.zcode 真身 ＋ .qoder 的符号链接）。
# 用途：换机／重装工具／技能包被误改后的复原。只读镜像，不带任何生产侧内容。
set -u
SRC="$(cd "$(dirname "$0")" && pwd)"
DEST_ROOT="${HOME}/.zcode/skill"
LINK_ROOT="${HOME}/.qoder/skills"
echo "[1] 校验镜像自身（件数与哈希）"; ( cd "$SRC" && sha256sum -c _sha256_manifest.txt 2>/dev/null | grep -c ': OK$' )
echo "[2] 恢复到 $DEST_ROOT（存在则先备份）"
if [ -d "$DEST_ROOT" ]; then cp -a "$DEST_ROOT" "$DEST_ROOT.orig.$(date +%Y%m%d_%H%M)" && echo "  已备份旧件"; fi
mkdir -p "$DEST_ROOT"; cp -a "$SRC/software-company" "$SRC/senior-developer" "$SRC/note" "$SRC/window-scheduler" "$DEST_ROOT/"
echo "[3] 目录位（775）与文件位（664）"; find "$DEST_ROOT" -type d -exec chmod 775 {} + ; find "$DEST_ROOT" -type f -exec chmod 664 {} +
echo "[4] 工具读取路径需要的符号链接"; mkdir -p "$LINK_ROOT"; for d in software-company senior-developer note window-scheduler; do [ -e "$LINK_ROOT/$d" ] || ln -s "$DEST_ROOT/$d" "$LINK_ROOT/$d"; done
echo "[5] 结果核对"; ls -l "$LINK_ROOT" | tail -4
