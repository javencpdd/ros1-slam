#!/usr/bin/env bash
# 孤儿 gzserver 检测与定点清理。
#
# 事故模型（2026-11-25 实测）：gzserver 进程活着、命令行里也带 `-s libgazebo_ros_api_plugin.so`，
# 但它注册的那个 rosmaster 已经随上一次 roslaunch 退出了。于是当前 master 上
#   `rosnode list` 没有 `/gazebo`、`rosservice list | grep gazebo` = 0、`/clock` 无发布者；
# 而它仍占着 GAZEBO_MASTER_URI 的端口（11345），导致新起的 gzserver **exit 255** 直接死掉。
# 表象极具误导性：`pgrep gzserver` 有进程、`/rslidar_points` 也有"发布者"（其实是空壳 advertise）。
#
# 判据（不数进程，看 ROS 面）：**有 gzserver 进程 且 当前 master 上没有 `/gazebo` 节点** ⇒ 孤儿。
# 清理范围（保守）：只杀命令行同时含 `worlds/m1_world.world` 与 `__name:=gazebo` 的进程，
# 以及监听 11313 的 rosmaster；绝不按 `pkill -f gzserver` 全杀。
#
# 用法：
#   bash scripts/verify/orphan_gzserver.sh            # 检测 + 清理
#   DRY=1 bash scripts/verify/orphan_gzserver.sh      # 只打印计划，不动手
# 退出码：0 无需清理或清理后已干净；1 仍有残留
set -uo pipefail

WS_=${WS_:-/opt/ros1-slam}
ROS_MASTER_PORT_=${ROS_MASTER_PORT_:-11313}
# 测试注入点：允许用假命令替换真实探测，从而在没有真 gzserver 的情况下验证判据
PGREP_CMD_=${PGREP_CMD_:-"pgrep -x gzserver"}
ROSNODE_CMD_=${ROSNODE_CMD_:-"rosnode list"}
DRY=${DRY:-0}

say() { printf '[orphan] %s\n' "$*"; }

gz_pids=$($PGREP_CMD_ 2>/dev/null || true)
if [ -z "${gz_pids// /}" ]; then
  say "无 gzserver 进程，无需清理"
  exit 0
fi

# 只看"当前 master 上有没有 /gazebo 节点"——这是判活体的唯一可靠信号
if $ROSNODE_CMD_ 2>/dev/null | grep -qx "/gazebo"; then
  say "gzserver 在跑且 /gazebo 节点在册（pids=$(echo $gz_pids | tr '\n' ' ')），属正常底座，不动"
  exit 0
fi

say "检出孤儿：gzserver 进程存在（$(echo $gz_pids | tr '\n' ' ')）但 master 无 /gazebo 节点"

targets=""
for p in $gz_pids; do
  cmd=$(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null || true)
  case "$cmd" in
    *"worlds/m1_world.world"*"__name:=gazebo"*) targets="$targets $p" ;;
    *) say "跳过 pid=$p（不属于 m1_world 仿真：$(printf '%s' "$cmd" | cut -c1-80)）" ;;
  esac
done

if [ -z "${targets// /}" ]; then
  say "无匹配到 m1_world 的 gzserver，不动手"
else
  say "计划清理 pid:$targets"
  for p in $targets; do
    ps -o pid,lstart,args -p "$p" 2>/dev/null | tail -1 | cut -c1-120
  done
  if [ "$DRY" = "1" ]; then
    say "DRY=1：只打印，不执行"
  else
    for p in $targets; do kill -TERM "$p" 2>/dev/null || true; done
    sleep 3
    for p in $targets; do kill -KILL "$p" 2>/dev/null || true; done
    # 同 master 的 rosmaster 一并清掉（它已无 gzserver 归属，留着会让下一次 base 误判"已有底座"）
    mp=$(pgrep -f "rosmaster --core -p ${ROS_MASTER_PORT_}" 2>/dev/null || true)
    if [ -n "${mp// /}" ]; then
      say "清理该域 rosmaster pid: $mp"
      for p in $mp; do kill -TERM "$p" 2>/dev/null || true; done
      sleep 2
    fi
  fi
fi

# 干跑到此为止（不做复核，否则会把"故意没杀"误报成失败）
if [ "$DRY" = "1" ]; then
  say "DRY=1 结束（未执行清理）"
  exit 0
fi

# 清完复核：进程清零、端口释放
left=$($PGREP_CMD_ 2>/dev/null || true)
if [ -n "${left// /}" ]; then
  say "FAIL 仍有 gzserver 残留：$(echo $left | tr '\n' ' ')"
  exit 1
fi
say "已清干净（gzserver=0）"
exit 0
