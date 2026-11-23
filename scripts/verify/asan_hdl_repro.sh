#!/usr/bin/env bash
# 用途：hdl_localization 定位链的分级复现。MODE=normal 跑正式库做基线，MODE=asan 跑 ASAN 版取崩溃现场。
#
# 设计约束（每一条都对应一次踩过的坑）：
#   1) 正式库 devel/lib 全程只读：ASAN 版放 devel/.asan/，靠 LD_LIBRARY_PATH 前置注入。
#   2) 注入是否真的命中不靠推理：加载后 grep /proc/<mgr>/maps 里 .asan/ 的绝对路径。
#      历史上 cp 覆盖 devel/lib 后忘记还原，导致「正常版」长期是 ASAN 版，结论全部作废。
#   3) 订阅是否真的建立不靠推理：nodelet 实例是 /mgr/hdl，不是 /mgr；
#      查 rosnode info /mgr/hdl 的 Subscriptions 与 rostopic info 的 Subscribers 计数。
#   4) 分级闸门：闸门不过立即退出并打印诊断，不把「跑满时长没崩」当成通过。
#   5) 全部 stdout 落 evidence 目录，脚本自带 tee，不依赖调用方重定向。
set -uo pipefail

MODE=${MODE:-normal}
DUR=${DUR:-60}
WAIT_SCAN=${WAIT_SCAN:-60}
MAP_PCD=${MAP_PCD:-/opt/ros1-slam/data/maps/mapped_m1.pcd}
WS=${WS:-/opt/ros1-slam/ws}
NOTE=/opt/ros1-slam/note/进度/00_专题/v8_map_loc_2026-11-24
EV=${EV:-$NOTE/evidence/${MODE}_$(date +%Y%m%d_%H%M%S)}
ASAN_LIB=/usr/lib/x86_64-linux-gnu/libasan.so.4
ASAN_DIR=$WS/devel/.asan
DBG_DIR=$WS/devel/.dbg
LIVE_LIB=$WS/devel/lib/libhdl_localization_nodelet.so
BACKUP_LIB=$WS/devel/.asan_backup/libhdl_localization_nodelet.so

mkdir -p "$EV"
exec > >(tee "$EV/run.log") 2>&1

say() { printf '%s\n' "$*"; }
gate_fail() { say "GATE_FAIL: $*"; exit 9; }

# 话题是否有数据流。两个坑叠在一起过：
#   1) 不能用 timeout 3：容器内 rospy 冷启动就要 2-3 秒，3 秒窗口把「有数据」判成「没数据」；
#   2) 不能写成 timeout N rostopic hz | grep -q：rostopic hz 不会自己退出，timeout 必然以 124
#      杀掉它，而本脚本开着 pipefail ⇒ 整个管道返回 124 ⇒ if 永远判假。
#      实测症状就是 Gazebo 明明在发 10 Hz 点云，脚本却卡在 [1] 等 450 秒。
hz_probe() {
  local out
  out=$(timeout 8 rostopic hz "$1" 2>/dev/null || true)
  printf '%s' "$out" | grep -q "average rate"
}

say "===== MODE=$MODE DUR=${DUR}s MAP=$MAP_PCD ====="
say "EV=$EV"
say "live_lib_sha=$(sha256sum "$LIVE_LIB" | cut -c1-16) backup_sha=$(sha256sum "$BACKUP_LIB" 2>/dev/null | cut -c1-16)"

source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11313}

[ -f "$MAP_PCD" ] || gate_fail "地图不存在 $MAP_PCD"

# ---------- 0. 清理 ----------
# rosnode kill -a 只发 shutdown 请求，杀不掉 rosrun 起的进程；残留的旧 manager / relay
# 会占住 /velodyne_points 与 /mgr 名字，让后续闸门全部失真。所以按命令行匹配直接杀。
say "--- [0] 清理陈旧节点与 gzserver ---"
for pat in 'nodelet manager' 'nodelet load' 'topic_tools.*relay'; do
  pkill -f "$pat" 2>/dev/null
done
sleep 2
# rosnode kill -a 会连 Gazebo 自己的 /gazebo /sim_adapter 一起关掉，复用 Gazebo 时必须跳过它；
# 进程层面的 pkill 已经足够，master 里的僵尸注册在下次同名注册时被顶替。
if [ "${SKIP_GAZEBO:-0}" != "1" ]; then
  rosnode kill -a >/dev/null 2>&1
  sleep 3
  for pid in $(ps -eo pid,args | awk '/gzserver/ && /worlds\// {print $1}'); do kill -TERM "$pid" 2>/dev/null; done
  sleep 3
fi
say "after_clean_nodes=$(rosnode list 2>/dev/null | tr '\n' ' ')"
say "after_clean_leftover=$(pgrep -fc 'nodelet|topic_tools' 2>/dev/null || true)"

rosparam set use_sim_time true

# ★崩溃现场日志必须走 rosconsole 文件 appender★：manager 的 stdout 重定向到文件后是块缓冲，
# SIGSEGV 时缓冲区整段丢失，manager.log 会是空的（2026-11-24 第一次正常版复现就是这样，
# 只知道崩了，不知道崩前走到哪）。FileAppender 配 bufferedIO=false 会逐条落盘。
#
# ★崩溃前的日志不能靠 rosconsole 配置★：试过 ROSCONSOLE_CONFIG_FILE + FileAppender，
# console.log 落地 0 字节，且 manager 的 screen 输出也一起没了 —— DOMConfigurator 解析失败后
# rosconsole 干脆不挂任何 appender，连带把默认 ConsoleAppender 覆盖掉，结果两条路都静默。
# 改用 libstdbuf 让 stdout/stderr 行缓冲：不碰日志框架，输出照旧走 stdout，
# 只是 SIGSEGV 之前每条都已经 write 到文件，缓冲区不会整段丢。
CONSOLE_DIR=$WS/devel/.asan
mkdir -p "$CONSOLE_DIR"
STDBUF_LIB=/usr/lib/x86_64-linux-gnu/coreutils/libstdbuf.so
[ -f "$STDBUF_LIB" ] || gate_fail "缺 $STDBUF_LIB，拿不到崩溃前的行缓冲输出"

# ---------- 1. Gazebo ----------
# ★map_to_odom:=false 是必须的★：m1_gazebo.launch 里的 map_to_odom_static 是「无定位节点」的
# 占位恒等桥，launch 注释写明「凡本台引入 hdl／faster_lio 等发 map 系的节点，必须注释掉本节点」。
# 不退位就会出现两个源同时发 map→odom，TF 树成环/多父。
say "--- [1] 启动 Gazebo（map_to_odom=false）---"
if [ "${SKIP_GAZEBO:-0}" = "1" ] && hz_probe /rslidar_points; then
  say "复用已在跑的 Gazebo（SKIP_GAZEBO=1 且 /rslidar_points 有数据）"
else
  setsid roslaunch lite3_gazebo_sim m1_gazebo.launch map_to_odom:=false \
    > "$EV/gazebo.log" 2>&1 < /dev/null &
  for i in $(seq 1 45); do
    sleep 2
    if hz_probe /rslidar_points; then say "rslidar_ready_after=${i}x2s"; break; fi
  done
fi
hz_probe /rslidar_points || gate_fail "Gazebo 未提供有数据的 /rslidar_points"
timeout 8 rostopic hz /rslidar_points 2>/dev/null | grep -E "average rate" | head -1
if ! rostopic list 2>/dev/null | grep -qx /rslidar_points; then gate_fail "/rslidar_points 未出现"; fi

# ---------- 2. 点云桥接 ----------
say "--- [2] relay /rslidar_points -> /velodyne_points ---"
setsid rosrun topic_tools relay /rslidar_points /velodyne_points > "$EV/relay.log" 2>&1 < /dev/null &
sleep 3
hz_probe /velodyne_points || { say "relay.log:"; tail -10 "$EV/relay.log"; gate_fail "/velodyne_points 无数据"; }
timeout 8 rostopic hz /velodyne_points 2>/dev/null | grep -E "average rate" | head -1

# ---------- 3. manager ----------
say "--- [3] 启动 nodelet manager ---"
# 用 nodelet 二进制直接起，不经 rosrun 的 bash 层：env 变量能确定地只作用于 manager 进程。
# MODE 三档：
#   normal —— 正式库 devel/lib，做基线；
#   dbg    —— devel/.dbg 的 -O1 -g 库，外面套 catchsegv(libSegFault) 抓回溯，再用 addr2line 解行号；
#   asan   —— devel/.asan 的 ASAN 库。★这条已知不可用★：把 libasan 预加载进未用 ASAN 编译的
#             nodelet manager，进程会单线程自旋（/proc/<pid>/status: State R、Threads 1），
#             XMLRPC 完全不响应，load 请求静默挂死 40 秒以上。保留只为留证。
MGR_CMD=(/opt/ros/noetic/lib/nodelet/nodelet manager __name:=mgr)
MGR_RUN=(env _STDBUF_O=L _STDBUF_E=L)
case "$MODE" in
  asan)
    MGR_RUN+=(LD_PRELOAD=$ASAN_LIB:$STDBUF_LIB
              "ASAN_OPTIONS=detect_leaks=0:abort_on_error=0:log_path=$CONSOLE_DIR/asan"
              LD_LIBRARY_PATH=$ASAN_DIR:$LD_LIBRARY_PATH)
    ;;
  dbg)
    # ★注入方式：LD_PRELOAD 预加载，不是 LD_LIBRARY_PATH★
    # class_loader 用 rospack 拼出绝对路径 devel/lib/...，LD_LIBRARY_PATH 前置对它无效
    # （实测命中 0，maps 里永远是 devel/lib 那份，拿 dbg 偏移解行号会得到假结论）。
    # 而 dlopen 按 soname 去重：两份 soname 都是 libhdl_localization_nodelet.so，
    # 预加载 dbg 之后 class_loader 再 dlopen 绝对路径会直接复用已加载的这份。
    # 这样既不覆盖 devel/lib，也不改插件 XML，还能在崩溃前就从 maps 验证到。
    MGR_RUN+=(LD_PRELOAD=$STDBUF_LIB:$DBG_DIR/libhdl_localization_nodelet.so)
    MGR_CMD=(catchsegv "${MGR_CMD[@]}")
    ;;
  *)
    MGR_RUN+=(LD_PRELOAD=$STDBUF_LIB)
    ;;
esac
setsid "${MGR_RUN[@]}" "${MGR_CMD[@]}" > "$EV/manager.log" 2>&1 < /dev/null &
# ★等 load_nodelet 服务就绪，不能固定 sleep★：ASAN 版启动慢，固定 sleep 6 会让 load 请求
# 落在 manager 还没 advertise 服务的窗口里，客户端不报错只是静默挂着，看起来像「加载失败」。
MGR_SVC_OK=0
for _i in $(seq 1 40); do
  sleep 1
  if rosservice list 2>/dev/null | grep -qx '/mgr/load_nodelet'; then MGR_SVC_OK=1; break; fi
done
if [ "$MGR_SVC_OK" != "1" ]; then
  say "manager.log tail:"; tail -20 "$EV/manager.log"
  gate_fail "manager 的 /mgr/load_nodelet 未就绪"
fi
# catchsegv 会多一层 /bin/sh，用精确进程名拿真正的 nodelet pid
MGR_PID=$(pgrep -x nodelet | head -1)
[ -n "$MGR_PID" ] || gate_fail "manager 未起来"
say "mgr_pid=$MGR_PID libasan_in_maps=$(grep -c 'libasan.so' /proc/$MGR_PID/maps 2>/dev/null || true)"
[ "$MODE" = "asan" ] && [ "$(grep -c 'libasan.so' /proc/$MGR_PID/maps 2>/dev/null || true)" = "0" ] \
  && gate_fail "LD_PRELOAD libasan 未生效，后续结论无意义"
# dbg 的注入必须在崩溃之前验证：进程一死 /proc/<pid>/maps 就没了，
# 放在 [6] 就只能得到 0 而无法区分「没注入」和「已经崩了」。
if [ "$MODE" = "dbg" ]; then
  PRE=$(grep -c "$DBG_DIR/libhdl_localization_nodelet.so" /proc/$MGR_PID/maps 2>/dev/null || true)
  say "preload_dbg_in_maps=$PRE"
  [ "$PRE" = "0" ] && gate_fail "调试库预加载未生效（maps 未命中 .dbg），回溯的行号不可用"
fi

# ---------- 4. globalmap server（始终用正式库，用于喂地图）----------
say "--- [4] 加载 GlobalmapServerNodelet（正式库）---"
rosparam set /gms/globalmap_pcd "$MAP_PCD"
rosparam set /gms/convert_utm_to_local false
rosparam set /gms/downsample_resolution 0.1
setsid rosrun nodelet nodelet load hdl_localization/GlobalmapServerNodelet mgr __name:=gms \
  > "$EV/gms.log" 2>&1 < /dev/null &
# ★判据必须是「有活数据」，不是「话题出现在 rostopic list 里」★：上一轮就是把上一轮残留的
# /globalmap 注册当成加载成功（globalmap_width 读出来是空、nodes 里没有 /mgr/gms，gms 压根没起）。
gm_width() {
  timeout 8 rostopic echo -n1 /globalmap/width 2>/dev/null | grep -oE '[0-9]+' | head -1
}
GM_W=""
for i in $(seq 1 20); do
  sleep 2
  GM_W=$(gm_width)
  if [ -n "$GM_W" ] && [ "$GM_W" != "0" ]; then say "globalmap_ready_after=${i}x2s width=$GM_W"; break; fi
done
if [ -z "$GM_W" ] || [ "$GM_W" = "0" ]; then
  say "gms.log:"; tail -20 "$EV/gms.log"
  say "nodes=$(rosnode list 2>/dev/null | grep -E '/mgr' | tr '\n' ' ')"
  gate_fail "/globalmap 无活数据（话题注册≠有 publisher 在发）"
fi
say "globalmap_width=$GM_W"

# ---------- 5. hdl（被测对象）----------
say "--- [5] 加载 HdlLocalizationNodelet（MODE=$MODE）---"
rosparam set /hdl/odom_child_frame_id base_link
# ★robot_odom_frame_id 必须＝odom★：publish_odometry 的两个分支按它选边——
# 能查到 robot_odom→base_link 就发「map→robot_odom」，查不到就发「map→base_link」。
# 默认值是 robot_odom，Gazebo 树里没有这条边，于是走 else 发 map→base_link，
# 与 Gazebo 的 odom→base_link 撞成双父。设成 odom 后走 if 分支，hdl 独占 map→odom。
rosparam set /hdl/robot_odom_frame_id odom
rosparam set /hdl/use_imu false
rosparam set /hdl/specify_init_pose true
# 初始位姿可配：机器人的真实起点未必是建图原点（雷达近距离地面点被过滤后，(0,0) 在地图里
# 是未观测区，全局规划器到不了任何地方）。这里传的是机器人**实际所在**的位置，不是伪造。
rosparam set /hdl/init_pos_x ${INIT_X:-0.0}
rosparam set /hdl/init_pos_y ${INIT_Y:-0.0}
rosparam set /hdl/init_pos_z 0.0
rosparam set /hdl/init_ori_w 1.0
rosparam set /hdl/reg_method NDT_OMP
rosparam set /hdl/ndt_resolution 1.0
rosparam set /hdl/ndt_neighbor_search_method DIRECT1
rosparam set /hdl/ndt_neighbor_search_radius 3.0
rosparam set /hdl/downsample_resolution 0.3
rosparam set /hdl/cool_time_duration 1.0
# 仿真里 now 与 header.stamp 的差包含 relay 转发与排队延迟，默认 200 ms 阈值会把近一半帧
# 判成 timeout 丢掉（实测 WARN: t_diss 210.000000 ms，align 频率只剩 ~5 Hz）。
# 放宽到 500 ms 只为不丢帧，不改变任何配准算法参数。
rosparam set /hdl/lidar_timeout_ms 500
setsid rosrun nodelet nodelet load hdl_localization/HdlLocalizationNodelet mgr __name:=hdl /odom:=/hdl_odom_raw \
  > "$EV/hdl.log" 2>&1 < /dev/null &
# 不固定 sleep：崩溃是即时的，等 8 秒既浪费时间又可能错过现场。轮询「实例出现 or manager 已死」。
LOAD_AT=""
for _i in $(seq 1 20); do
  sleep 1
  if ! [ -d "/proc/$MGR_PID" ]; then LOAD_AT="mgr_died_at=${_i}s"; break; fi
  if rosnode list 2>/dev/null | grep -qxE '/hdl|/mgr/(hdl|HdlLocalizationNodelet)'; then LOAD_AT="instance_at=${_i}s"; break; fi
done
say "load_result=${LOAD_AT:-timeout_20s}"

if [ -d "/proc/$MGR_PID" ]; then
  say "mgr_alive=yes"
else
  say "mgr_alive=NO（加载即崩）"
  say "--- manager.log 尾 40（崩溃现场；空＝崩在任何 NODELET_INFO 之前）---"
  tail -40 "$EV/manager.log" 2>/dev/null || say "(manager.log 不存在)"
fi

# ---------- 6. 注入有效性闸门 ----------
say "--- [6] 注入有效性：实际加载的是哪一份 .so ---"
MAPS_HIT_ASAN=0; MAPS_HIT_DBG=0; MAPS_HIT_LIVE=0
if [ -d "/proc/$MGR_PID" ]; then
  MAPS_HIT_ASAN=$(grep -c "$ASAN_DIR/libhdl_localization_nodelet.so" /proc/$MGR_PID/maps 2>/dev/null || true)
  MAPS_HIT_DBG=$(grep -c "$DBG_DIR/libhdl_localization_nodelet.so" /proc/$MGR_PID/maps 2>/dev/null || true)
  MAPS_HIT_LIVE=$(grep -c "$WS/devel/lib/libhdl_localization_nodelet.so" /proc/$MGR_PID/maps 2>/dev/null || true)
fi
say "maps_asan=$MAPS_HIT_ASAN maps_dbg=$MAPS_HIT_DBG maps_live=$MAPS_HIT_LIVE"
# 进程已死时 /proc/<pid>/maps 不存在，三项必然全 0；这不是「没加载」，而是「已经崩了」。
# dbg 的注入在 [3] 之后已经用 preload_dbg_in_maps 验过，这里不能因为崩了再否一遍。
if [ ! -d "/proc/$MGR_PID" ]; then
  say "进程已退出，跳过 maps 复核（dbg 的注入已由启动时的 preload 检查确认）"
else
  case "$MODE" in
    asan) [ "$MAPS_HIT_ASAN" = "0" ] && { grep -E 'hdl_localization|ndt_omp|libasan' /proc/$MGR_PID/maps 2>/dev/null | awk '{print $NF}' | sort -u; gate_fail "ASAN 版未被加载，本次运行不能用于取崩溃证据"; } ;;
    dbg)  [ "$MAPS_HIT_DBG" = "0" ]  && { grep -E 'hdl_localization|ndt_omp' /proc/$MGR_PID/maps 2>/dev/null | awk '{print $NF}' | sort -u; gate_fail "调试版未被加载，回溯会指向正式库，行号失真"; } ;;
    *)    { [ "$MAPS_HIT_ASAN" != "0" ] || [ "$MAPS_HIT_DBG" != "0" ]; } && gate_fail "正常版跑法却加载了调试/ASAN 库，环境被污染" ;;
  esac
fi

# ---------- 7. 实例与订阅闸门 ----------
# ★两个实测坑★：
#   1) nodelet 实例不是 /mgr 本身。/mgr 是 manager 进程，自身永远 Subscriptions: None；
#   2) load 时用 __name:=hdl，实例在 master 里注册成**顶层 /hdl**，不是 /mgr/hdl。
#      之前按 /mgr/hdl 找，永远找不到，把「已加载」误判成「加载失败」——这也是上一轮
#      「Subscriptions: None」结论的第二个来源（第一个是查错日志落点）。
# 所以判据落在「回调链路是否通」而不是名字上：/velodyne_points 必须有真订阅者。
say "--- [7] 实例与订阅闸门 ---"
say "nodes=$(rosnode list 2>/dev/null | grep -iE 'hdl|mgr' | tr '\n' ' ')"
HDL_NODE=""
for cand in /hdl /mgr/hdl /mgr/HdlLocalizationNodelet; do
  if rosnode list 2>/dev/null | grep -qx "$cand"; then HDL_NODE=$cand; break; fi
done
say "hdl_node=${HDL_NODE:-未找到}"

SUB_BLOCK=$(rostopic info /velodyne_points 2>/dev/null | sed -n '/Subscribers/,$p')
say "-- /velodyne_points Subscribers --"
printf '%s\n' "$SUB_BLOCK" | head -5
SUB_OK=0
if printf '%s\n' "$SUB_BLOCK" | grep -q '\*' && ! printf '%s\n' "$SUB_BLOCK" | grep -q 'None'; then SUB_OK=1; fi
say "velodyne_has_subscriber=$SUB_OK"
if [ "$SUB_OK" = "0" ]; then
  say "hdl.log:"; tail -20 "$EV/hdl.log"
  say "manager.log tail:"; tail -20 "$EV/manager.log"
  gate_fail "/velodyne_points 没有真订阅者（回调不可能被调用）"
fi
if [ -n "$HDL_NODE" ]; then
  rosnode info "$HDL_NODE" > "$EV/hdl_info.txt" 2>&1
  say "-- $HDL_NODE Subscriptions --"
  sed -n '/^Subscriptions/,/^$/p' "$EV/hdl_info.txt"
fi

# ---------- 8. 进 scan/align 闸门 ----------
# ★判据落点★：nodelet 的 NODELET_INFO 走 rosconsole console backend，打在 **manager 进程 stdout**，
# 已重定向到 $EV/manager.log。而 /opt/ros1-slam/logs/latest/ 是 roslaunch 的 run 目录，
# 只有 roslaunch 托管的节点才写进去；rosrun 起的 manager 不写那里。
# 历史上一直 grep logs/latest 得到全 0，据此判「回调没进」——那是查错了地方。
say "--- [8] 等待进入 scan -> align（最多 ${WAIT_SCAN}s）---"
logcnt() {
  local n
  n=$(grep -c "$1" "$EV/manager.log" 2>/dev/null)
  echo "${n:-0}"
}
GOT_SCAN=0; GOT_ALIGN=0
for i in $(seq 1 $WAIT_SCAN); do
  sleep 1
  if [ "$GOT_SCAN" = "0" ] && [ "$(logcnt 'scan: raw')" != "0" ]; then
    GOT_SCAN=1; say "scan_seen_after=${i}s"
  fi
  if [ "$GOT_SCAN" = "1" ] && [ "$(logcnt 'align done')" != "0" ]; then
    GOT_ALIGN=1; say "align_seen_after=${i}s"; break
  fi
  if ! [ -d "/proc/$MGR_PID" ]; then say "mgr 在等待中退出（第 ${i}s）"; break; fi
done
say "got_scan=$GOT_SCAN got_align=$GOT_ALIGN"
say "manager_log_lines=$(wc -l < "$EV/manager.log" 2>/dev/null || true)"
say "对照 logs/latest 计数（历史误判来源）: scan=$(grep -rh 'scan: raw' /opt/ros1-slam/logs/latest/ 2>/dev/null | wc -l) align=$(grep -rh 'align done' /opt/ros1-slam/logs/latest/ 2>/dev/null | wc -l)"

if [ "$GOT_SCAN" = "0" ] || [ "$GOT_ALIGN" = "0" ]; then
  say "--- 未进入 align，诊断 ---"
  say "mgr_alive=$([ -d /proc/$MGR_PID ] && echo yes || echo no)"
  for pat in "globalmap has not been received" "cannot be transformed" "waiting for initial pose" "cloud is empty" "filtered scan too small" "align begin" "align done"; do
    printf '%-40s %s\n' "$pat" "$(logcnt "$pat")"
  done
  say "--- manager.log 尾 40 ---"; tail -40 "$EV/manager.log" 2>/dev/null
  say "--- hdl.log 尾 20 ---"; tail -20 "$EV/hdl.log" 2>/dev/null
  say "GATE_FAIL: 未进入 align，不进入崩溃观察"
  exit 9
fi

# ---------- 9. 崩溃观察 ----------
say "--- [9] 已进入 align，观察 ${DUR}s 崩溃 ---"
CRASH_AT=""
for i in $(seq 1 "$DUR"); do
  sleep 1
  if ! [ -d "/proc/$MGR_PID" ]; then CRASH_AT=$i; say "mgr_died_at=${i}s"; break; fi
done
say "crash_at=${CRASH_AT:-none}"
say "asan_reports=$(ls $CONSOLE_DIR/asan.* 2>/dev/null | wc -l)"
if [ -n "$CRASH_AT" ]; then
  say "--- ASAN 报告（若有）---"
  head -60 $CONSOLE_DIR/asan.* 2>/dev/null
  say "--- manager.log 尾部 ---"
  tail -40 "$EV/manager.log"
fi

# ---------- 10. 汇总 ----------
cp -f $CONSOLE_DIR/asan.* "$EV/" 2>/dev/null
say "===== 汇总 ====="
say "mode=$MODE crash_at=${CRASH_AT:-none}"
say "scan_lines=$(logcnt 'scan: raw')"
say "align_lines=$(logcnt 'align done')"
say "align_begin=$(logcnt 'align begin')"
say "lidar_timeout_drops=$(logcnt 'Disgarding a timeout lidar msg')"
say "globalmap_msgs=$(logcnt 'globalmap received:')"
say "live_lib_sha_now=$(sha256sum "$LIVE_LIB" | cut -c1-16)"

# ---------- 定位数据闸门（可注入失效）----------
# hdl 现在发 /hdl_odom_raw，由这个命名 relay 转发成 /hdl_odom，loc 只订后者。
# 杀掉 relay 即可"只断定位数据"：hdl、manager、roslaunch、9 个导航节点全部不受影响。
# 之前试过的三种手段都不行 —— pkill manager 关栈、rosnode kill /hdl 触发 required 关停、
# nodelet unload 被活动服务拒绝。串一层 relay 是唯一干净且可复现的注入点。
say "--- [8b] 定位数据闸门 /hdl_odom_raw -> /hdl_odom ---"
setsid rosrun topic_tools relay /hdl_odom_raw /hdl_odom __name:=hdl_gate > "$EV/hdl_gate.log" 2>&1 < /dev/null &
sleep 2
hz_probe /hdl_odom || { say "hdl_gate.log:"; tail -10 "$EV/hdl_gate.log"; gate_fail "/hdl_odom 无数据（闸门未通）"; }

# /hdl_odom 是 P3 的验收对象：必须有稳定输出。
ODOM_HZ=$(timeout 8 rostopic hz /hdl_odom 2>/dev/null | grep -E "average rate" | tail -1)
say "hdl_odom_hz=${ODOM_HZ:-none}"
if [ -z "$ODOM_HZ" ]; then
  say "P3_VERDICT=FAIL（/hdl_odom 无输出）"
else
  say "P3_VERDICT=PASS（/hdl_odom 有输出：$ODOM_HZ）"
fi
say "EV=$EV"
