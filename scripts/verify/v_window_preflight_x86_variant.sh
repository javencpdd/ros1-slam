#!/usr/bin/env bash
# v_window_preflight_x86_variant.sh —— x86 侧**联调**预检变体（R356 到签①，2026-10-18）
#
# 身份与边界（★先读再跑）：
#   · 在册 `v_window_preflight.sh`（A35＝丙，主理人-佳文 2026-10-14 裁）**一字不动**，
#     仍是唯一在册许可式；本件是 x86 容器路径形差下的**联调变体**。
#   · 按真人到签①：本件跑通（rc=0）即**视为预检 rc=0**（联调口径）。
#   · 本件**不重抄任何判据**：三条判据、rc 语义（0 放行／5 拒窗／9 前置缺失）、负载门的
#     核归一化基（R355 改判）**全部走在册本体**——唯一差别是把本体的 VPF_ISO 换成同目录
#     乙式包装件 `_iso_alt_wrapper.sh`（它接住"预检自带 source 顶掉 /home 形"这一跳）。
#     ★单一真身仍在在册件；在册件若改，本件自动跟随（无第二份实现、无漂移面）。
#   · ★本件不自设门值、不改判据语义、不入门禁串、不申请入任何面清单（入面归主理人）。
#
# 与在册本体的分界＝只此一处：
#   在册本体：. source（noetic→devel）→ bash v3_isolation.sh        ⇒ x86 上 [4] 项 /home 形败（甲式已知态）
#   本件：    . source（同在册本体）→ bash _iso_alt_wrapper.sh       ⇒ 包装件把 /home 形插回队头＋ISO_ALLOW 补全 ⇒ 乙式口径
#
# 用法（★x86 容器内）：
#   bash v_window_preflight_x86_variant.sh            # 真跑（放行句形状与在册本体逐字同形）
#   bash v_window_preflight_x86_variant.sh --selftest # 透传在册本体五例自测（T0 的 ISO 走本件包装件）
# rc 语义与在册本体同：0＝放行；5＝拒窗（任一判据不成立）；9＝前置缺失。
set -u
SELF=$(readlink -f "${BASH_SOURCE[0]}")
HERE=$(cd "$(dirname "$SELF")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
BASE=$ROOT/scripts/verify/v_window_preflight.sh
WRAP=$HERE/_iso_alt_wrapper.sh
[ -f "$BASE" ] || { echo "[前置缺失] 在册预检本体不在位＝$BASE"; exit 9; }
[ -f "$WRAP" ] || { echo "[前置缺失] 乙式包装件不在位＝$WRAP"; exit 9; }
export VPF_ISO="$WRAP"
if [ "${1:-}" = "--selftest" ]; then
  exec bash "$BASE" --selftest
fi
exec bash "$BASE" "$@"
