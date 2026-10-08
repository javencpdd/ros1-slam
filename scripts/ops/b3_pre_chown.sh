#!/usr/bin/env bash
# b3_pre_chown.sh —— 批3 前置（甲新1）的执行与复验件（小明 2026-10-08，工单 note/进度/SCAN集成/设计/b3_甲案工单_属主侧待办.md）
#
# 为何要有这件：configure 报的是 **chmod 类**（file COPY cannot set permissions / os.chmod PermissionError），
#   而 chmod 只认文件属主 ⇒ 补 ACL 不解决（甲案第一版口径不足，已如实更正）。要动的只有 catkin 每次构建都要
#   **重写并 chmod** 的那一族生成件；ws/devel/lib 与 ws/build 下的编译产物、devel/include 生成头**一律不动**。
# 谁跑：chown 改他人属主需要 root ⇒ 由属主/主理人侧执行；本件默认只 dry-run，不自己动（甲案：本仓不代跑 chown）。
#
# 用法：
#   bash b3_pre_chown.sh              仅列清单与计数（零改动）
#   sudo bash b3_pre_chown.sh --apply 改属主到 ysc:ysc（36 件，清单与 dry-run 同一表达式取，绝不手抄）
#   bash b3_pre_chown.sh --verify    复跑增量 catkin_make 并给四条复验（configure rc／关键产物 sha／白名单／属主面 diff）
set -u
ROOT=/home/ros1-slam
cd "$ROOT" || exit 9
TARGET_USER=ysc
LIST(){ find ws/devel ws/build/catkin_generated ws/build/atomic_configure -maxdepth 3 -type f \
          -user ros1-slam ! -path '*/devel/lib/*' ! -path '*/devel/include/*' ! -name '*.so*' 2>/dev/null | sort; }
MODE=${1:-dry}
if [ "$MODE" = "--dry" ] || [ "$MODE" = "dry" ]; then
  echo "【dry-run】待改属主件（$USER → $TARGET_USER）："
  LIST | sed 's/^/  /'
  N=$(LIST | wc -l); echo "计数=$N"
  echo "确认无误后执行： sudo bash scripts/ops/b3_pre_chown.sh --apply"
  echo "两处当场报错的件是否在清单里：$(LIST | grep -cE 'ws/devel/\.rosinstall$|ws/build/catkin_generated/setup_cached\.sh$')/2"
  exit 0
fi
if [ "$MODE" = "--apply" ]; then
  if [ "$(id -u)" != "0" ]; then
    echo "【拒跑】chown 需 root（当前 $(id -un)）⇒ 请用： sudo bash scripts/ops/b3_pre_chown.sh --apply"
    exit 9
  fi
  N=0; while IFS= read -r f; do chown ysc:ysc "$f" && N=$((N+1)); done <<EOL
$(LIST)
EOL
  echo "【apply】已改属主 $N 件（清单同一表达式取，未手抄）"
  LIST | wc -l | sed 's/^/  仍属 ros1-slam 的同类件数（应 0）=/'
  exit 0
fi
if [ "$MODE" = "--verify" ]; then
  echo "【verify】$(date '+%F %T')  属主面：ysc 件数=$(find ws/devel ws/build -type f -user $TARGET_USER 2>/dev/null | wc -l) ros1-slam 件数=$(find ws/devel ws/build -type f -user ros1-slam 2>/dev/null | wc -l)"
  SOK=$(sha256sum ws/devel/lib/liblite3_planar_drive.so 2>/dev/null | cut -d' ' -f1)
  BOK=$(sha256sum data/maps/lite3.pcd | cut -d' ' -f1)
  echo "  改前 .so sha=${SOK:-无}  样张 sha=${BOK:0:12}…"
  set +u; unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION; source /opt/ros/noetic/setup.bash; set -u
  ( cd "$ROOT/ws" && catkin_make -j2 > /tmp/b3_verify_catkin.log 2>&1 ); rc=$?
  echo "  ① 增量 catkin_make rc=$rc（须 0；日志 /tmp/b3_verify_catkin.log 尾：）"
  tail -3 /tmp/b3_verify_catkin.log | sed 's/^/      /'
  echo "  ② 关键产物是否被顺带重编译：改后 .so sha=$(sha256sum ws/devel/lib/liblite3_planar_drive.so 2>/dev/null | cut -d' ' -f1) ⇒ 与改前 $([ "$SOK" = "$(sha256sum ws/devel/lib/liblite3_planar_drive.so 2>/dev/null | cut -d' ' -f1)" ] && echo 同值=未动 || echo "不等=动了(须登记)")"
  WL=$(grep -E '^CATKIN_WHITELIST_PACKAGES:' ws/build/CMakeCache.txt 2>/dev/null | head -1)
  WV=${WL#*=}   # 缓存行的形态是 键:TYPE=值；判空要看等号后面，不能拿整行等值比（上一版就把它误判成"非空须登记"）
  echo "  ③ 白名单必须是空值（只数出现次数会把「键在但被赋值」也判成过）：现值=[$WL] ⇒ $([ -z "$WV" ] && echo "OK（值确为空）" || echo "非空＝$WV，须登记")；样张 $([ "$BOK" = "$(sha256sum data/maps/lite3.pcd | cut -d' ' -f1)" ] && echo 未变 || echo 变了!!)"
  echo "  ④ 属主面 diff：构建后 ysc 件数=$(find ws/devel ws/build -type f -user $TARGET_USER 2>/dev/null | wc -l)（若比改前跳增，说明我的构建又改了共享件属主，须当场公开＝坑30 纪律）"
  [ "$rc" = "0" ] || echo "  ⇒ configure 仍未通过：批3 本体不可开工（报错原文见日志）"
  exit "$rc"
fi
echo "未知模式：$MODE（可用：无参|--dry|--apply|--verify）"; exit 9
