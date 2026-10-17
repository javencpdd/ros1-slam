#!/usr/bin/env bash
# _iso_alt_wrapper.sh —— 乙式 ISO 包装件（★只服务 x86 容器路径形差；R356 到签①）
#
# 存在理由（在册事实，R354 登记）＝在册 v_window_preflight.sh 自带 source
#   （noetic → ws/devel），会把 devel 的 `/opt/ros1-slam/ws/src` **重新插到
#   ROS_PACKAGE_PATH 队头** ⇒ 调用方预设的 `/home/ros1-slam/ws/src` 形被顶掉，
#   rospack 解析回 `/opt/ros1-slam`，`v3_isolation.sh` 第 [4] 项的 `/home` 形
#   要求即败（＝x86 上 ISO 甲式已知态的机理，不是环境坏）。
#   直接跑 `v3_isolation.sh` 时先设乙式旋钮则 ISOLATION OK rc=0（R354 双旋钮实测），
#   ★但旋钮穿不过预检本体的自带 source —— 本包装件补的正是这一跳。
#
# 本件做什么＝被预检本体调用时（★此时 source 已完成）：
#   ①把 `/home/ros1-slam/ws/src` 插回 ROS_PACKAGE_PATH 队头（/home 形重新赢）；
#   ②ISO_ALLOW 显式给全量白名单并补 `/opt/ros1-slam` 前缀（devel 的 pwd -P 解析形，
#     ★防"未设时空展开覆盖脚本自带缺省面"那一族假红——R354 实录）；
#   ③exec 在册 v3_isolation.sh（★在册 ISO 件一字不动、参数原样透传）。
#
# 边界＝★不自设任何判据、不改任何门值、不改 ISO 件本身；只做环境重排。
# ★只适用于 x86 容器内（/home/ros1-slam 软链由镜像运行期建立）；宿主机裸跑不支持。
set -u
SELF=$(readlink -f "${BASH_SOURCE[0]}")
HERE=$(cd "$(dirname "$SELF")" && pwd)
ISO=${ISO_TARGET:-$HERE/v3_isolation.sh}
[ -f "$ISO" ] || { echo "[前置缺失] 在册 ISO 件不在位＝$ISO"; exit 9; }
export ROS_PACKAGE_PATH="/home/ros1-slam/ws/src${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
export ISO_ALLOW="/opt/ros1-slam:/home/ros1-slam:/opt/ros/noetic:/usr:/opt/3rdparty:/etc"
exec bash "$ISO" "$@"
