#!/usr/bin/env bash
# x2_ablate_stvl.sh —— stvl **归因差分**段启动器（R384 佳文裁定＝M4 并取、次序 R2→R1 的 R2 执行件；施工侧-小明）
#
# 为何存在：`stvl_标注列` 的顶档缺"归因"通道（只读件 derive_stvl_mark.py 按定形 ③ 表 A 行只能落到 N3）。
#   归因的一条零改在册件路径＝单层对照差分——同参各去掉一枚插件、看足印内痕迹是否随之消失。
# ★在册硬约束逐条：①**不改 probe_x2_fourcells.sh**（M2 不落；它是 T3 四枚 sha 见证之一，现值 cbf06584e453）
#   ②**不改 costmap_common.yaml**（红线件）——插件参数块用 sed **逐字拷**自该件，★不读内存值、不重写
#   ③段名与在册六段错开（x2_d1／x2_d2），★避免撞在册档名
#   ④窗型＝联调·旁证（★不是判据窗、★不入分子、★不写转正句）
#   ⑤teardown 只按本件 PID 与端口持有者收、★不 kill 他人进程；无 sudo、无 FORCE=1
# 用法：ABLATE_SEG=x2_d1 X2_RUN=/tmp/x2_attr bash note/进度/SCAN集成/设计/scripts/x2_ablate_stvl.sh
#   ABLATE_SEG＝x2_d1（去 sob_layer，留 stvl）‖ x2_d2（去 stvl_obstacle_layer，留 sob）
# rc：0＝本段原始件合格（data: 计数＝1 且格数＝1600）｜5＝不合格（记 N3、该段不判被试）｜9＝前置缺失
set -u
d=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd "$d/../../../../.." && pwd)
RUN=${X2_RUN:-/tmp/x2_attr}
SEG=${ABLATE_SEG:-x2_d1}
GAP=${X2_GAP:-12}
PUBPY=$ROOT/note/进度/SCAN集成/设计/scripts/x2_pub_two_frame.py
CLOUDPYP=$ROOT/note/进度/SCAN集成/设计/scripts/x2_pub_cloud.py
CCY=$ROOT/ws/src/40_navigation/lite3_navigation/config/costmap_common.yaml
LOG=$RUN/run.log
mkdir -p "$RUN"
export ROS_MASTER_URI=http://127.0.0.1:11313
set +u; . /opt/ros/noetic/setup.bash; [ -f "$ROOT/ws/devel/setup.bash" ] && . "$ROOT/ws/devel/setup.bash"; set -u

say() { echo "[$(date '+%T')] $*" | tee -a "$LOG"; }
note() { echo "$*" | tee -a "$LOG"; }

case "$SEG" in
  x2_d1) KEEP="stvl_obstacle_layer"; PLUGINS='[{name: static_a, type: "costmap_2d::StaticLayer"}, {name: static_b, type: "costmap_2d::StaticLayer"}, {name: stvl_obstacle_layer, type: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"}]' ;;
  x2_d2) KEEP="sob_layer";           PLUGINS='[{name: static_a, type: "costmap_2d::StaticLayer"}, {name: static_b, type: "costmap_2d::StaticLayer"}, {name: sob_layer, type: "sob_layer::SobLayer"}]' ;;
  *) echo "[前置缺失] 未知段名：$SEG（只认 x2_d1‖x2_d2）"; exit 9 ;;
esac
[ -f "$CCY" ] || { echo "[前置缺失] 在册参数件不在位：$CCY"; exit 9; }
[ -f "$CLOUDPYP" ] || { echo "[前置缺失] 点云发布器不在位"; exit 9; }

# ---------------- 参数件：基键照在册探针同式，插件块从 costmap_common.yaml 逐字 sed 拷 ----------------
gen_params() {
  local out="$RUN/params_$SEG.yaml"
  {
    echo "global_frame: map"
    echo "robot_base_frame: base_link"
    echo "robot_radius: 0.35"
    echo "resolution: 0.1"
    echo "width: 4.0"
    echo "height: 4.0"
    echo "transform_tolerance: 10.0"
    echo "update_frequency: 5.0"
    echo "publish_frequency: 5.0"
    echo "always_send_full_costmap: true"
    echo "track_unknown_space: false"
    echo "plugins: $PLUGINS"
    echo "static_a: {map_topic: /pmap_a, first_map_only: true, subscribe_to_updates: false, track_unknown_space: false, use_maximum: true, lethal_cost_threshold: 100, unknown_cost_value: -1, trinary_costmap: true}"
    echo "static_b: {map_topic: /pmap_b, first_map_only: true, subscribe_to_updates: false, track_unknown_space: false, use_maximum: true, lethal_cost_threshold: 100, unknown_cost_value: -1, trinary_costmap: true}"
  } > "$out"
  if [ "$KEEP" = "stvl_obstacle_layer" ]; then
    sed -n '/^stvl_obstacle_layer:/,/^sob_layer:/p' "$CCY" | sed '$d' >> "$out"
  else
    sed -n '/^sob_layer:/,/^[[:space:]]*$/p' "$CCY" | sed '$d' >> "$out"
  fi
  printf '%s' "$out"
}

RS="" ALLPIDS=""
cleanup() {
  for p in $ALLPIDS "$RS"; do [ -n "$p" ] && kill "$p" 2>/dev/null; done
  wait 2>/dev/null
  for q in $(ss -lntup 2>/dev/null | grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u); do
    cl=$(tr '\0' ' ' < "/proc/$q/cmdline" 2>/dev/null)
    case "$cl" in *x2_ablate_stvl*|*costmap_2d_node*|*x2_pub_*|*static_transform_publisher*) ;;
      *rosmaster*|*roscore*|*rosout*) kill "$q" 2>/dev/null ;;
    esac
  done
  say "本件 teardown 结束（★只按本件 PID 与端口持有者收）"
}
trap cleanup EXIT

pf=$(gen_params)
say "== 差分段 $SEG｜去=$([ "$KEEP" = stvl_obstacle_layer ] && echo sob_layer || echo stvl_obstacle_layer)｜留=$KEEP =="
say "参数件＝$pf（插件行数回读 $(grep -c 'name:' <<<"$PLUGINS") 声明／实拷块 $(grep -c "^${KEEP}:" "$pf")）"

roscore -p 11313 > "$RUN/roscore.log" 2>&1 & RS=$!
for i in $(seq 1 40); do rosnode list >/dev/null 2>&1 && break; sleep 1; done
rosnode list >/dev/null 2>&1 || { echo "[前置缺失] roscore 40s 未起（见 $RUN/roscore.log）"; exit 9; }
rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 10 > "$RUN/tf_base.log" 2>&1 & ALLPIDS="$ALLPIDS $!"

rosparam load "$pf" "/$SEG/costmap" || { note "ABLATE_VERDICT rc=9（rosparam load 失败）"; exit 9; }
X2_GAP="$GAP" X2_TAG="$SEG" python3 "$PUBPY" > "$RUN/pub_$SEG.log" 2>&1 & ALLPIDS="$ALLPIDS $!"
sleep 2
X2_CLOUD_TAG="$SEG" python3 "$CLOUDPYP" > "$RUN/cloud_$SEG.log" 2>&1 & ALLPIDS="$ALLPIDS $!"
rosrun tf static_transform_publisher 0 0 0.3 0 0 0 base_link rslidar 10 > "$RUN/tf_r_$SEG.log" 2>&1 & ALLPIDS="$ALLPIDS $!"
rosrun costmap_2d costmap_2d_node __name:="$SEG" > "$RUN/node_$SEG.log" 2>&1 & NODEPID=$!
ALLPIDS="$ALLPIDS $NODEPID"

sleep $(( GAP + 12 ))
say "主话题数：$(rostopic list 2>/dev/null | grep -c "/$SEG/costmap/costmap$")（须 1）"
timeout 40 rostopic echo -n1 "/$SEG/costmap/costmap" > "$RUN/cost_$SEG.txt" 2>&1
dh=$(grep -c 'data:' "$RUN/cost_$SEG.txt")
CNT=$(python3 -c "import re;t=open('$RUN/cost_$SEG.txt',encoding='utf-8',errors='ignore').read();print(len(re.findall(r'-?\d+',t.split('data:',1)[1])) if 'data:' in t else 0)" 2>/dev/null || echo 0)
note "  段指纹：seg=$SEG 留=$KEEP 去=$([ "$KEEP" = stvl_obstacle_layer ] && echo sob_layer || echo stvl_obstacle_layer) data_行数=$dh 解析值数=$CNT"
if [ "$dh" = 1 ] && [ "$CNT" = 1600 ]; then
  note "ABLATE_VERDICT rc=0（$SEG 原始件合格）｜★顶档判定交只读件 derive_stvl_mark.py（本件不自判）"
  exit 0
else
  note "ABLATE_VERDICT rc=5（$SEG 记 N3＝原始件不合格 data_行数=$dh 解析值数=$CNT）"
  exit 5
fi
