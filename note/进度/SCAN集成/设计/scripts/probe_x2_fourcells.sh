#!/usr/bin/env bash
# probe_x2_fourcells.sh —— X2「恢复该方向前的待补测四格」窗包探针（R332 建；来源＝主理人 2026-10-16 03:5x 勾复① 案 **乙**＝四格一次跑齐、一扇连续窗、6 段、格③ 走合成点云）
#
# 工单出处＝note/进度/SCAN集成/设计/at_X2四格呈报输入单_供窗包.md §②（＃166）；被试面出处＝v5.3 §16.9 X2 条「恢复该方向前的待补测四格」四格原文。
# ★本件只出"读数"，不自判 X2 能否叠加，也不写「已解决／被推翻／确证」级结论（判定归验收工程师-小美、落册笔权归主理人）。
# ★与 r59 旧探针的五处差别（＝在册缺口逐条补）：
#   ① 参数面＝**在册 costmap_common.yaml 的 static_layer 块八键逐字照抄**（唯一改动＝map_topic 指向本探针自己的测试图话题，理由＝双实例必须各有各的 map_topic）
#   ② **不再出现 `combination_method`**（承 R276 在册更正＝StaticLayer 根本没有该键，键挂在没人读的层上）
#   ③ 两张测试图**在同一格各给不同非零代价**（A=100、B=50）＝能把"取大"与"后加载覆盖"判开的**判别区**；r59 两图无同格冲突 ⇒ 判不开
#   ④ 发布器改**两帧**（第二帧新增 80 区）＝first_map_only 语义当期可测
#   ⑤ 格③ 另起一段＝现役三插件构成（stvl＋sob 参数块从在册 yaml 逐字拷）＋合成点云发布器
#
# 六段：g1 格①（在册参数面 T,T）｜g2 (T,F)｜g3 (F,T)｜g4 (F,F)＝格② 四组合之其余三组｜g5 (T,T) 复跑（同参数面换 ns＝测可重复性）｜g6 格③ 三插件构成
# 格④（横切）＝每段四件原始件：cost_gN.txt（rostopic echo -n1 落盘）＋params_gN.yaml＋node_gN.log＋run.log 里的计数断言行与时间戳。
#
# rc 语义（本件自报；★不入门禁串＝勾复③「不入」，将来要入第 11 件须另走 #112/#114 同款明令）：
#   0＝六段全部原始件合格（`data:` 计数断言＝1 且 格数＝1600）｜5＝有段不合格（该段记 N3、不判被试）｜9＝前置缺失（发布器不在位／source 后非 noetic／roscore 起不来＝★空读数不得当绿读）
# 用法：bash probe_x2_fourcells.sh                     RUN=${X2_RUN:-/tmp/x2_fourcells}  两帧间隔=${X2_GAP:-12}s
#       bash probe_x2_fourcells.sh --parse-selftest    离线自检（不碰 ROS，四例：两例须过、两例须失败）
# 开窗前置由 scripts/verify/v_window_preflight.sh 把关（在册唯一许可式）；本件不自设负载门槛、不 kill 非本件进程。
set -u
SELF=$(readlink -f "${BASH_SOURCE[0]}")
d=$(dirname "$SELF")
while [ "$d" != "/" ] && [ ! -f "$d/scripts/verify/v_window_preflight.sh" ]; do d=$(dirname "$d"); done
ROOT=$d
RUN=${X2_RUN:-/tmp/x2_fourcells}
GAP=${X2_GAP:-12}
LOG=$RUN/run.log
PUBPY=$ROOT/note/进度/SCAN集成/设计/scripts/x2_pub_two_frame.py
CLOUDPYP=$ROOT/note/进度/SCAN集成/设计/scripts/x2_pub_cloud.py
CCY=$ROOT/ws/src/40_navigation/lite3_navigation/config/costmap_common.yaml
mkdir -p "$RUN"
export ROS_MASTER_URI=http://127.0.0.1:11313

say() { echo "[$(date '+%T')] $*" | tee -a "$LOG"; }
note() { echo "$*" | tee -a "$LOG"; }

write_parser() {
  cat > "$RUN/_parse.py" <<'PYPARSE'
import sys, re
p = sys.argv[1]
t = open(p, encoding='utf-8', errors='ignore').read()
if 'data:' not in t:
    print("N3|未采到 data 面"); sys.exit(1)
vals = [int(x) for x in re.findall(r'-?\d+', t.split('data:', 1)[1])][:1600]
if len(vals) != 1600:
    print("N3|格数=%d（须 1600）" % len(vals)); sys.exit(1)
g = lambda r, c: vals[r * 40 + c]
print("data_行数=%d 格数=%d 判别区=%s A独占50点=%s B独占100区=%s 未知区=%s 第二帧新置区=%s distinct=%s 254_格数=%d"
      % (t.count('data:'), len(vals), [g(10, c) for c in range(10, 15)], g(20, 20),
         [g(12, c) for c in range(12, 15)], [g(30, c) for c in range(0, 5)],
         [g(25, c) for c in range(25, 30)], sorted(set(vals)), vals.count(254)))
PYPARSE
}

# ---------------- 离线解析自检 ----------------
if [ "${1:-}" = "--parse-selftest" ]; then
  write_parser
  PF="$RUN/_parse.py"
  python3 - "$RUN" <<'PYGEN'
import sys, os
run = sys.argv[1]
def mk(name, band, keep_all=True):
    v = [0] * 1600
    for c in range(10, 15):
        v[400 + c] = band
    txt = "header:\n  frame_id: map\ninfo:\n  width: 40\n  height: 40\ndata: %s\n" % str(v).replace(",", "")
    p = os.path.join(run, name)
    open(p, 'w').write(txt if keep_all else txt[:400])
    return p
mk('_st100.txt', 100)
mk('_st50.txt', 50)
mk('_stshort.txt', 100, keep_all=False)
open(os.path.join(run, '_stnodata.txt'), 'w').write("WARNING: topic does not appear\n")
PYGEN
  A=$(python3 "$PF" "$RUN/_st100.txt" 2>&1); RA=$?
  B=$(python3 "$PF" "$RUN/_st50.txt" 2>&1); RB=$?
  C=$(python3 "$PF" "$RUN/_stnodata.txt" 2>&1); RC=$?
  D=$(python3 "$PF" "$RUN/_stshort.txt" 2>&1); RD=$?
  echo "例一 判别区=100 → rc=$RA｜$A"
  echo "例二 判别区=50  → rc=$RB｜$B"
  echo "例三 无 data 面 → rc=$RC｜$C"
  echo "例四 格数不足   → rc=$RD｜$D"
  if [ "$RA" = 0 ] && [ "$RB" = 0 ] && [ "$RC" = 1 ] && [ "$RD" = 1 ] \
     && printf '%s' "$A" | grep -qF '判别区=[100, 100, 100, 100, 100]' \
     && printf '%s' "$B" | grep -qF '判别区=[50, 50, 50, 50, 50]'; then
    echo "X2_PARSE_SELFTEST PASS=4 FAIL=0 rc=0"; exit 0
  fi
  echo "X2_PARSE_SELFTEST FAIL＝例一/二 须 rc=0 且判别区读对、例三/四 须 rc=1（★自检坏＝活体读数不可信）"; exit 9
fi

# ---------------- 前置在位性 ----------------
[ -f "$PUBPY" ]    || { echo "[前置缺失] 两帧发布器不在位＝$PUBPY"; exit 9; }
[ -f "$CLOUDPYP" ] || { echo "[前置缺失] 合成点云发布器不在位＝$CLOUDPYP"; exit 9; }
[ -f "$CCY" ]      || { echo "[前置缺失] 在册 costmap_common.yaml 不在位＝$CCY"; exit 9; }
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
[ -f "$ROOT/ws/devel/setup.bash" ] && source "$ROOT/ws/devel/setup.bash"
set -u
export PYTHONNOUSERSITE=1
[ "${ROS_DISTRO:-}" = "noetic" ] || { echo "[前置缺失] ROS_DISTRO=${ROS_DISTRO:-空} 非 noetic"; exit 9; }

RS="" ALLPIDS=""
port_pid() { ss -lntp 2>/dev/null | grep -o ":$1.*pid=[0-9]*" | grep -o 'pid=[0-9]*' | cut -d= -f2 | sort -u; }
cleanup() {
  say "-- teardown（只按本件 PID 与端口持有者收；★不用未锚定的 -f 全局匹配、不 kill 他人进程）--"
  for p in $ALLPIDS "$RS"; do [ -n "$p" ] && kill "$p" 2>/dev/null; done
  sleep 3
  for _ in 1 2 3 4 5 6; do
    n=$(port_pid 11313); [ -z "$n" ] && break
    for q in $n; do
      cl=$(tr '\0' ' ' < /proc/$q/cmdline 2>/dev/null)
      case "$cl" in
        *rosmaster*|*roscore*|*rosout*) say "  端口持有者 $q 确认为本件 ROS 件（${cl:0:40}）⇒ 收"; kill "$q" 2>/dev/null ;;
        *) say "  端口持有者 $q 不是 ROS 件（${cl:0:40}）⇒ ★不收" ;;
      esac
    done
    sleep 2
  done
  say "退净：11313=$(ss -lnt 2>/dev/null | grep -c ':11313') 11311=$(ss -lnt 2>/dev/null | grep -c ':11311')（均须 0）"
}
trap cleanup EXIT

write_parser
say "== X2 四格窗包开跑｜RUN=$RUN｜身份=$(whoami)｜load1=$(cut -d' ' -f1 /proc/loadavg)｜两帧间隔=${GAP}s｜在册 yaml=$CCY =="
roscore -p 11313 > "$RUN/roscore.log" 2>&1 & RS=$!
for _ in $(seq 1 40); do rosnode list >/dev/null 2>&1 && break; sleep 1; done
rosnode list >/dev/null 2>&1 || { echo "[前置缺失] roscore 40s 未起（见 $RUN/roscore.log）"; exit 9; }
say "roscore 在位（PID=$RS）"
rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 10 > "$RUN/tf_base.log" 2>&1 & ALLPIDS="$ALLPIDS $!"
sleep 3

gen_params() {  # $1=段名 $2=use_maximum $3=first_map_only $4=构成(static2|four_plugin) → 打印参数件路径
  local seg="$1" um="$2" fm="$3" kind="$4" out="$RUN/params_$1.yaml"
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
    if [ "$kind" = "four_plugin" ]; then
      echo 'plugins: [{name: static_a, type: "costmap_2d::StaticLayer"}, {name: static_b, type: "costmap_2d::StaticLayer"}, {name: stvl_obstacle_layer, type: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"}, {name: sob_layer, type: "sob_layer::SobLayer"}]'
    else
      echo 'plugins: [{name: static_a, type: "costmap_2d::StaticLayer"}, {name: static_b, type: "costmap_2d::StaticLayer"}]'
    fi
    echo "static_a: {map_topic: /pmap_a, first_map_only: $fm, subscribe_to_updates: false, track_unknown_space: false, use_maximum: $um, lethal_cost_threshold: 100, unknown_cost_value: -1, trinary_costmap: true}"
    echo "static_b: {map_topic: /pmap_b, first_map_only: $fm, subscribe_to_updates: false, track_unknown_space: false, use_maximum: $um, lethal_cost_threshold: 100, unknown_cost_value: -1, trinary_costmap: true}"
  } > "$out"
  if [ "$kind" = "four_plugin" ]; then
    sed -n '/^stvl_obstacle_layer:/,/^sob_layer:/p' "$CCY" | sed '$d' >> "$out"
    sed -n '/^sob_layer:/,/^[[:space:]]*$/p' "$CCY" | sed '$d' >> "$out"
  fi
  printf '%s' "$out"
}

OKSEG=0; FAILSEG=0; N3LIST=""
run_seg() {  # $1=段名 $2=use_maximum $3=first_map_only $4=构成 $5=格名
  local seg="$1" um="$2" fm="$3" kind="$4" cell="$5" pf dh rd rc
  pf=$(gen_params "$seg" "$um" "$fm" "$kind")
  say "---- 段 $seg｜格＝$cell｜use_maximum=$um｜first_map_only=$fm｜构成=$kind ----"
  rosparam load "$pf" "/$seg/costmap" || { say "  [$seg] ★rosparam load 失败 ⇒ 记 N3"; FAILSEG=$((FAILSEG+1)); N3LIST="$N3LIST $seg"; return; }
  local plc=$(rosparam get "/$seg/costmap/plugins" 2>&1 | grep -c 'name:')
  say "  参数回读：global_frame=$(rosparam get "/$seg/costmap/global_frame" 2>&1 | tail -1)｜plugins=$plc（须 2 或 4）｜static_a.use_maximum=$(rosparam get "/$seg/costmap/static_a/use_maximum" 2>&1 | tail -1)"
  X2_GAP="$GAP" X2_TAG="$seg" python3 "$PUBPY" > "$RUN/pub_$seg.log" 2>&1 &
  local pubpid=$!; ALLPIDS="$ALLPIDS $pubpid"
  sleep 2
  if [ "$kind" = "four_plugin" ]; then
    X2_CLOUD_TAG="$seg" python3 "$CLOUDPYP" > "$RUN/cloud_$seg.log" 2>&1 &
    local clpid=$!; ALLPIDS="$ALLPIDS $clpid"
    rosrun tf static_transform_publisher 0 0 0.3 0 0 0 base_link rslidar 10 > "$RUN/tf_r_$seg.log" 2>&1 &
    local trpid=$!; ALLPIDS="$ALLPIDS $trpid"
  fi
  rosrun costmap_2d costmap_2d_node __name:="$seg" > "$RUN/node_$seg.log" 2>&1 &
  local nodepid=$!; ALLPIDS="$ALLPIDS $nodepid"
  sleep $(( GAP + 12 ))
  say "  插件装载行：$(grep -aoE 'static_a|static_b|SpatioTemporalVoxel|SobLayer' "$RUN/node_$seg.log" 2>/dev/null | sort -u | tr '\n' ' ')"
  say "  主话题数：$(rostopic list 2>/dev/null | grep -c "/$seg/costmap/costmap$")（须 1）"
  timeout 40 rostopic echo -n1 "/$seg/costmap/costmap" > "$RUN/cost_$seg.txt" 2>&1
  dh=$(grep -c 'data:' "$RUN/cost_$seg.txt")
  rd=$(python3 "$RUN/_parse.py" "$RUN/cost_$seg.txt" 2>&1); rc=$?
  if [ "$rc" = 0 ] && [ "$dh" = 1 ]; then
    OKSEG=$((OKSEG + 1)); say "  [$seg] ★原始件合格（data_行数=$dh）→ $rd"
  else
    FAILSEG=$((FAILSEG + 1)); N3LIST="$N3LIST $seg"; say "  [$seg] ★N3＝原始件不合格（data_行数=$dh｜解析 rc=$rc｜$rd）⇒ 该段不判被试"
  fi
  note "  段指纹：seg=$seg 格=\"$cell\" use_maximum=$um first_map_only=$fm 构成=$kind data_行数=$dh 解析rc=$rc"
  kill "$nodepid" "$pubpid" 2>/dev/null
  [ -n "${clpid:-}" ] && kill "$clpid" 2>/dev/null
  [ -n "${trpid:-}" ] && kill "$trpid" 2>/dev/null
  trpid=""; clpid=""
  sleep 4
}

run_seg x2_g1 true  true  static2     "格①在册参数面"
run_seg x2_g2 true  false static2     "格②(T,F)"
run_seg x2_g3 false true  static2     "格②(F,T)"
run_seg x2_g4 false false static2     "格②(F,F)"
run_seg x2_g5 true  true  static2     "格②(T,T) 复跑"
run_seg x2_g6 true  true  four_plugin "格③三插件构成"

say "== 汇总 =="
note "X2 四格读数 = 段数=6｜原始件合格=$OKSEG／6｜不合格=$FAILSEG／6｜N3 段=${N3LIST:-无}（★合格判据＝data: 计数断言=1 且 格数=1600）"
note "★读法写死：判别区=[100×5]⇒A 的 lethal 保住（取大语义）；=[0×5]⇒被后加载者的自由格抹掉（覆盖语义）；其余值⇒两档都不成立、须逐段看；第二帧新置区=[100×5]⇒后续图进层（first_map_only 未拦），=[0×5]⇒只吃首帧；A独占50点 读回 0＝在册 trinary 把 50 抹成 0"
note "★本件不自判 X2 能否叠加、不写已解决／被推翻／确证；四格是否跑齐只看上面两个数"
if [ "$FAILSEG" = 0 ]; then
  echo "X2_FOURCELL VERDICT rc=0（六段原始件齐＝四格已跑齐）"; exit 0
else
  echo "X2_FOURCELL VERDICT rc=5（有段记 N3＝四格未跑齐）"; exit 5
fi
