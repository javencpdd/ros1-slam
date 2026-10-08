#!/usr/bin/env bash
# v_pose_source.sh —— 位姿源身份断言（F-D 前置断言的可执行化，小明 2026-10-08）
# 设计出处：note/进度/SCAN集成/设计/fd_双写者与前馈源裁定.md §10.5（小张）＋判据 FD-2（同件 §6 表）。
#   主判 = 「/Odometry 的 Publishers 段行数恰 1」**加**「消息头帧对去重集合符合该窗口径」——
#   两条都要，因为单看发布者数抓不到「同一进程发两系」，单看帧对抓不到「两个进程发同一名」。
# 分档（**必须分档，否则 F-D 的正确结果会被判成红**，小张原话）：
#   pre        = F-D 落地**前**（改前阳性对照）：/Odometry 帧对去重集合应含 {odom, camera_init} 两族（双写实证）
#   post_a     = F-D 落地**后**、有 faster_lio 的窗（stageA/stageC/SCAN 链）：/Odometry 只剩 {camera_init}，且 /sim/odom_gt 为 {odom}
#   post_b     = F-D 落地**后**、无 faster_lio 的窗（stageB/W4）：/Odometry 无发布者（不该有读者依赖它），/sim/odom_gt 为 {odom}
#   legacy_b   = F-D 落地**前**、无 faster_lio 的窗：/Odometry 为 {odom} 单族（现状 stageB）
# 用法：bash v_pose_source.sh <档> <窗口秒=8>（须在被检话题已在位的窗内调用；本件不起链，避免与被试仿真争资源）
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
set -u
MODE=${1:?用法: v_pose_source.sh pre|post_a|post_b|legacy_b [秒]}
WIN=${2:-8}
FAIL=0
say() { [ "$1" = "0" ] && echo "  [PASS] $2" || { echo "  [FAIL] $2"; FAIL=1; }; }

pubs_of() { timeout 15 rostopic info "$1" 2>/dev/null | awk '/^Publishers:/{f=1;next} /^Subscribers:/{f=0} f&&NF' | sed 's/^ *[*] *//;s/ (http.*//' | sort -u; }
frames_of() { # $1=话题 $2=秒
  timeout $(( $2 + 15 )) rostopic echo -n 60 "$1" 2>/dev/null | awk -v w="$2" '
    /^header:/{grp=1} grp&&/frame_id:/{if(!seen[$0]++){print}} /^child_frame_id:/{if(!cf[$0]++){print}}' | sort -u | tr '\n' ' '
}

for T in /Odometry /sim/odom_gt; do
  if ! rostopic list 2>/dev/null | grep -qx -- "$T"; then
    echo "  话题 $T：不存在"
    # F-D 未落地档（pre／legacy_b）里 /sim/odom_gt 本就不该存在 ⇒ 缺失是**符合预期**，不是 FAIL。
    #   （首版把"不存在"一律判 FAIL，等于用改后的世界去要求改前的现场——假红。）
    case "$MODE:$T" in
      post_a:/sim/odom_gt|post_b:/sim/odom_gt) say 1 "话题 $T 在位（$MODE 档要求 F-D 已落地）";;
      pre:/sim/odom_gt|legacy_b:/sim/odom_gt)  say 0 "话题 $T 尚未引入（$MODE 档＝F-D 改前态，符合预期）";;
      pre:/Odometry)                           say 1 "话题 /Odometry 缺失 ⇒ 改前双写阳性对照无从取（本档必须存在）";;
      legacy_b:/Odometry)                      say 1 "话题 /Odometry 缺失 ⇒ 现状单写阳性对照无从取（本档必须存在）";;
      *) say 0 "话题 $T 意外缺失（$MODE 档）";;
    esac
    continue
  fi
  NP=$(pubs_of "$T" | grep -c .)
  PL=$(pubs_of "$T" | paste -sd, -)
  FR=$(frames_of "$T" "$WIN")
  echo "  $T：Publishers=$NP [$PL]  帧对集合={$FR}"
  case "$MODE:$T" in
    pre:/Odometry)      echo "$FR" | grep -q odom && echo "$FR" | grep -q camera_init; say $? "pre：/Odometry 含双族（odom 与 camera_init）＝双写阳性对照在案";;
    post_a:/Odometry)   [ "$NP" = "1" ] && ! echo "$FR" | grep -qw odom;  say $? "post_a：/Odometry 发布者恰 1 且不再含 odom 族（真值已让位）";;
    post_a:/sim/odom_gt)[ "$NP" = "1" ] && echo "$FR" | grep -q odom;     say $? "post_a：/sim/odom_gt 恰 1 发布者且为 odom 族（真值另名可读）";;
    post_b:/Odometry)   [ "$NP" = "0" ]; say $? "post_b：无 faster_lio 窗内 /Odometry 无发布者（不得有隐性读者）";;
    post_b:/sim/odom_gt)[ "$NP" = "1" ] && echo "$FR" | grep -q odom;     say $? "post_b：stageB 位姿源＝/sim/odom_gt 单写";;
    legacy_b:/Odometry) [ "$NP" = "1" ] && echo "$FR" | grep -q odom;    say $? "legacy_b：现状 stageB 为 odom 单族单写";;
    *) echo "  （$MODE 档不检 $T）";;
  esac
done
[ "$FAIL" = "0" ] && echo "POSE SOURCE VERDICT: PASS ($MODE)" || echo "POSE SOURCE VERDICT: FAIL ($MODE)"
exit "$FAIL"
