#!/usr/bin/env bash
# premap_check.sh — 采图（mapping）开工前环境体检
# 用法: sudo bash /home/ros1-slam/scripts/verify/premap_check.sh [雷达IP]
# 退出码: 0=全绿可开工  1=阻塞项(必须处置)  2=仅警告(可开工但注意)
# 依据: v4 P0.5 双查(扩充正则) + 坑5(UDP端口占用) + 坑7(interval语义) + 2026-10-05 案例(foxy侧livox驱动抢端口)
set -u
LIDAR_IP=${1:-192.168.1.201}
BLOCK=0; WARN=0
ok(){ printf "  ✓ %s\n" "$1"; }
bad(){ printf "  ✗ [阻塞] %s\n" "$1"; BLOCK=$((BLOCK+1)); }
warn(){ printf "  ⚠ [警告] %s\n" "$1"; WARN=$((WARN+1)); }

echo "=== 采图前环境体检 $(date '+%F %T') ==="

echo "[1] ROS1 残留进程（应无）"
P=$(ps aux | grep -E "roscore|rosmaster|run_mapping_online|map_server|hdl_localization" | grep -v grep || true)
if [ -n "$P" ]; then bad "残留 ROS1 进程：先 pkill -f 'roscore|rosmaster|livox|faster_lio|run_mapping'"; echo "$P" | awk '{print "      "$1, $11, $12}'; else ok "无"; fi

echo "[2] 任何侧的雷达驱动进程（应无——雷达 UDP 端口只允许一个进程绑定，ROS1/ROS2/foxy 侧都算）"
P=$(ps aux | grep -E "livox_ros_driver2_node|lslidar_driver|lslidar.*_node" | grep -v grep || true)
if [ -n "$P" ]; then bad "已有雷达驱动在跑（下方列表，含 ROS2/foxy 侧也算）：先停掉它"; echo "$P" | awk '{print "      "$1, $11, $12}'; else ok "无"; fi

echo "[3] 雷达数据端口占用（56100–56599 应无任何绑定）"
P=$(ss -ulnp 2>/dev/null | grep -E ":56[1-5]0[0-9] " || true)
if [ -n "$P" ]; then bad "雷达端口被占（下方为占用者；无 sudo 时看不到进程名）："; echo "$P" | sed 's/^/      /'; else ok "空闲"; fi

echo "[4] master 端口 11312（应空闲——本次将起全新 master，无陈旧注册）"
P=$(ss -tlnp 2>/dev/null | grep ":11312 " || true)
if [ -n "$P" ]; then bad "11312 已被占用：旧 master 活着，先清（pkill -f roscore|rosmaster）"; echo "$P" | sed 's/^/      /'; else ok "空闲"; fi

echo "[5] 运动链路侧进程（期望恰好 4 行常态：jetson2motion/jetson2app/transfer_launch/lite3_ros_bridge）"
N=$(ps aux | grep -E "ros2qnx|message_transformer|lite3_ros_bridge|jetson2motion|jetson2app|transfer_launch" | grep -v grep | wc -l)
if [ "$N" -eq 4 ]; then ok "4 行常态（采图模式不发 /cmd_vel，无需处置）"
elif [ "$N" -gt 4 ]; then bad "运动链路侧 $N 行 >4：出现未知进程，触 Q8 停机线，先处置再采图"
else warn "运动链路侧 $N 行 <4（常态 4 行有变化，信息性记录）"; fi

echo "[6] 雷达可达性"
if ping -c1 -W1 "$LIDAR_IP" >/dev/null 2>&1; then ok "$LIDAR_IP reachable"; else bad "$LIDAR_IP 不可达（查雷达供电/网线）"; fi

echo "[7] 磁盘余量 ≥15G"
A=$(df -BG /home/ros1-slam | awk 'NR==2{gsub("G","",$4); print $4}')
if [ "${A:-0}" -ge 15 ]; then ok "可用 ${A}G"; else bad "仅剩 ${A}G（19 号反判据④）"; fi

echo "[8] 内存可用 ≥2G（interval=-1 全程累积预估 ~0.5–1G）"
M=$(free -m | awk 'NR==2{print $7}')
if [ "${M:-0}" -ge 2000 ]; then ok "可用 ${M}MB"; else warn "仅 ${M}MB——走位控制在 15 分钟内，防内存压力"; fi

echo "[9] 编译产物与 launch 在位"
if [ -f /home/ros1-slam/ws/devel/setup.bash ] && [ -f /home/ros1-slam/ws/src/00_drivers/lite3_drivers/launch/01_sensors.launch ] && [ -f /home/ros1-slam/ws/src/10_slam/lite3_slam/launch/10_slam_mapping.launch ]; then ok "devel + 两 launch 在位"; else bad "devel 或 launch 缺失（先完成 P5 / 检查文件）"; fi

echo "[10] 采图参数核验（faster_lio_outdoor.yaml）"
Y=/home/ros1-slam/ws/src/10_slam/lite3_slam/config/faster_lio_outdoor.yaml
if grep -q "interval: -1" "$Y" && grep -q "imu_topic: /livox/imu" "$Y"; then ok "interval=-1（全量落盘，停序须等日志打完）/ imu=/livox/imu"; else bad "关键参数与预期不符（interval 必须 -1、imu_topic 必须 /livox/imu）——勿凭记忆改回，走 04 台账"; fi
if grep -q "filter_size_surf: 0.3" "$Y"; then ok "filter_size_surf 0.3（CPU 安全档）"; else warn "filter_size_surf 非 0.3（0.15 实测 load11 超载）"; fi

echo "[11] 历史底图归档份数（防覆盖丢失）"
N=$(ls /home/ros1-slam/note/进度/P3b/证据/ 2>/dev/null | grep -c "底图v1\|首采样张")
if [ "$N" -ge 2 ]; then ok "已归档 $N 份，本次覆盖无忧"; else warn "归档仅 $N 份（<2），覆盖 data/maps 前先备份到证据目录"; fi

echo
echo "[Foxglove] ROS1 模式 → http://192.168.1.103:11312 ｜ ROS_HOSTNAME=笔记本局域网IP ｜ 订阅 /cloud_registered + /path（/rslidar_points 可订，~2MB/s 易丢帧）｜ 只订阅不发布"
echo
if [ $BLOCK -gt 0 ]; then echo "=== PREMAP BLOCKED（$BLOCK 个阻塞项，处置后重跑）==="; exit 1
elif [ $WARN -gt 0 ]; then echo "=== PREMAP OK（$WARN 个警告，可开工）==="; exit 2
else echo "=== PREMAP OK（全绿，可开工）==="; exit 0
fi
