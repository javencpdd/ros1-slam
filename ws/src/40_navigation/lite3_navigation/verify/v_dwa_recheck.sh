#!/usr/bin/env bash
# v_dwa_recheck.sh —— W2 遗留项「DWA 经 mbf nav_core 包装不可用」的重核驱动（小明 2026-10-08）
# 前置已满足的证据链：W2 三败签名 = 位姿获取域（age 5.372s > tolerance 3s），而 M2R5/U1 已根除
#   该家族根因（base_link 双父 + 静态 map->odom 零锚点）⇒ 小美「非算法否定、待后续重核」的条件成立。
# 判据（roadmap C #4 原句）：mbf 试加载插件成功 ＋ ExePath outcome=0 ×3（三连）。
# 入口走 TOP 编排件 lite3_bringup/mode_sim_teb.launch（K3：平级模块禁互 include；与 m2_smoke.sh 同款）。
# 用法：bash v_dwa_recheck.sh [轮数=3]
# rc 语义表（R40，小美 M2R8 裁定 2；本件自有层，与 DWP／SM／BR／RD 各层互不认，引用必须连件名＋代际）：
#   0 = PASS       DWA 判据 N 连全过且 teb 同场对照过
#   5 = FAIL       确有判据不过（有效跑内 state/outcome/dt/dev 不过）⇒ 判 DWA 不可用成立
#   6 = INVALID    纯前置不成立或有效性门不过（/clock 未出、DWA 未装载、RTF<0.25、teb 对照不过）⇒ 作废重跑，不判可用性
#   7 = 并存档     INVALID 与判据 FAIL 同批并存 ⇒ 停手上报，不得由重试消化（与 SM 的 rc=4 同型不同值）
#   4 = INCOMPLETE 凑不满 N 又不属上述（须人工看日志），既不判可用也不判不可用
# 取一序：7 > 5 > 6 > 4 > 0。旧版此处恒 exit 0（判据 FAIL 改不动批级 rc）＝假判据，她 R40 判停，
#   并由 H13 批给出活体反例：teb 对照步卡死被我手工终止，件仍打 dwa rc=0。
# 注：catkin 的 profile.d/1.ros_distro.sh 读 ROS_DISTRO 时不防未绑定 ⇒ 必须在 source 段关 -u，
#   source 完再开（02 既有坑，m2_smoke.sh 同款处置）。
set +u
N=${1:-3}
PKG=/home/ros1-slam/ws/src/40_navigation/lite3_navigation
export DWA_OUT=${DWA_OUT:-/tmp/dwa_recheck}
mkdir -p "$DWA_OUT"
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313 GAZEBO_MASTER_URI=http://127.0.0.1:11345
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib

cleanup() { rosnode kill -a >/dev/null 2>&1; sleep 3
  pkill -f gzserver >/dev/null 2>&1; pkill -f gzclient >/dev/null 2>&1
  pkill -f "roslaunch.*mode_sim_" >/dev/null 2>&1; pkill -f move_base_flex >/dev/null 2>&1
  pkill -f sim_adapter >/dev/null 2>&1; pkill -f "rosmaster --core -p 11313" >/dev/null 2>&1
  sleep 4
  { echo "# 退净核验 @$(date '+%F %T')"
    echo "-- 命中进程原样 ps 行 --"
    # 旧写法把「调用本件的那条命令行」也列为命中（命令行里带着 dwa_recheck 字样＝匹配散文，R40 后续修订项）：
    #   现在按两条规则剔除自身面——①剔除本 shell 及其全部祖先 PID；②剔除 args 里出现本判据件名的行。
    #   这是**证据可读性**修正，不改任何判据门值；剩余命中仍是真残留。
    _anc=$$; while [ "$_anc" != "1" ] && [ -n "$_anc" ]; do _anc=$(ps -o ppid= -p "$_anc" 2>/dev/null | tr -d ' '); [ -n "$_anc" ] && _ex="${_ex:-} $_anc"; done
    # 上一版我把这条改成「管道三段 + || echo（零命中）」——结果 **|| 永远不触发**：
    #   流水线的退出码取最后一个命令（awk），awk 收到空输入照样返回 0 ⇒ 零命中时什么都不打，
    #   「空输出」冒充「零命中」（自测 T3 当场抓住，属坑29 那一族）。现改为先取串再显式二分支。
    _hits=$(pgrep -af 'roslaunch|rosmaster|rosout|gzserver|gzclient|move_base_flex|sim_adapter|dwa_recheck' 2>/dev/null \
      | grep -v "v_dwa_recheck.sh" | awk -v ex="$$ ${_ex:-}" 'BEGIN{split(ex,a," "); for(i in a) drop[a[i]]=1} !($1 in drop)')
    if [ -n "$_hits" ]; then printf '%s\n' "$_hits"; else echo "  （零命中）"; fi
    echo "-- 端口 TCP/UDP 分查 --"
    for p in 11311 11312 11313 11345; do echo "  port $p TCP=$(ss -lnt 2>/dev/null | grep -c ":$p\b") UDP=$(ss -lnu 2>/dev/null | grep -c ":$p\b")"; done
  } > "$DWA_OUT/exitclean.txt" 2>&1
  sed 's/^/[exitclean] /' "$DWA_OUT/exitclean.txt"; }
trap cleanup EXIT

echo "### 一、mbf 是否真把 DWA 装进控制器表（加载判据，零节点侧改动）"
roslaunch lite3_bringup mode_sim_teb.launch > "$DWA_OUT/mbf.log" 2>&1 &
LP=$!
T=0
until [ $T -ge 300 ]; do
  if rostopic list 2>/dev/null | grep -qx /clock && timeout 10 rostopic echo -n 1 /clock >/dev/null 2>&1; then break; fi
  sleep 2; T=$((T+2))
done
[ $T -lt 300 ] || { echo "  [INVALID] /clock 300s 未出 ⇒ 前置不成立，本批作废重跑（不判 DWA 可用性，R40）"; exit 6; }
echo "  /clock 就绪 t=${T}s"
# 加载判据必须**有界轮询**：mbf 装插件晚于 /clock 出现数十秒，单次即查会把「还没装」误判成「装不上」，
#   更糟的是随即 exit → trap 关掉整台仿真（首版即踩此坑，log 见 Reason given for shutdown: user request）。
# 就绪信号必须等**目标那一条**：planners（GlobalPlanner/NavfnROS）先于 controllers 装载，
#   拿「loaded successfully 计数 >=2」当就绪会在 DWA 还没轮到时就读到 2 而提前判定（首版即踩）。
# R86 等待上界重定标（件面改动，**不动任何判据门值**）：原 300s 是按「快环境」拍的，实测两次装载时刻差一倍以上——
#   try1：装载行落在墙钟 +约340s（sim=1.268s，RTF≈0.24）⇒ 300s 判 INVALID 属**假红**；
#   try2：bound 已放到 600s 仍未装载，事后看日志实际到货约 +690s（含迟到自诊的 30s 也没赶上）
#     ⇒ 同一次窗里我并发跑了 sha256/读 npy 这类重活，把装载拖长了一倍（02 坑54：**独占窗里并发离线活会制造假 INVALID**）。
#   结论：装载时延对机器负载敏感，上界必须按**最慢观测值再加裕量**定 ⇒ 默认 900s；`DWA_LOAD_BOUND` 可覆写。
#   纪律面同步：跑本件期间**不得并发跑离线重活**（已在 05 的 G 段与本注释留名）。
# R88 就绪判据重做（任务 #76；**件面改动，不动任何判据门值**）。三跑全打 rc=6 的根因不是"上界不够大"：
#   try1 的 mbf.log 时间戳重建出「首行 sim=0.001 → DWA 装载行 sim=1.268」＝**事件由仿真进度驱动**，
#   而当时这台机器 RTF 只有 ≈0.026（sim 推进 1.267s 花了 48s 墙钟）⇒ 装载需要的墙钟时长 = sim 阈值 ÷ RTF，
#   **RTF 随负载波动 ⇒ 拿墙钟上界等它是无界的**（340s / 690s / >1020s 三次"都太短"就是这么来的）。
#   改法三条：①上界降级为**防挂死**用途（默认 2400s，不再是判定值）；②轮询期间打印 sim 与 RTF，
#   让"慢"看得见、可区分环境负载与件坏；③**装载迟到不再直接作废整批**——只要本窗内最终装上就继续跑判据，
#   ③(R88 第二版) 到界**不再 exit 6**——日志全缓冲会造"假否定"，只打印警示并继续进判据阶段。
LOAD_BOUND=${DWA_LOAD_BOUND:-2400}
sim_now() { timeout 12 rostopic echo -n 1 /clock 2>/dev/null \
  | awk '/^ *secs:/{s=$2} /^ *nsecs:/{n=$2} END{if(s==""){print "-1"} else {printf "%.2f", s+(n==""?0:n)/1000000000}}'; }
W0=$(date +%s); S0=$(sim_now)
DWA_LOADED=0; TEB_LOADED=0; T2=0
while [ $T2 -lt $LOAD_BOUND ]; do
  DWA_LOADED=$(grep -ac 'dwa_local_planner/DWAPlannerROS" has been loaded successfully' "$DWA_OUT/mbf.log" 2>/dev/null | head -1); DWA_LOADED=${DWA_LOADED:-0}
  TEB_LOADED=$(grep -ac 'teb_local_planner/TebLocalPlannerROS" has been loaded successfully' "$DWA_OUT/mbf.log" 2>/dev/null | head -1); TEB_LOADED=${TEB_LOADED:-0}
  [ "$DWA_LOADED" -ge 1 ] && [ "$TEB_LOADED" -ge 1 ] && break
  if [ $((T2 % 60)) -eq 0 ]; then
    SN=$(sim_now); EL=$(( $(date +%s) - W0 )); RTF="—"
    if [ "${SN%.*}" -gt 0 ] 2>/dev/null && [ "$EL" -gt 0 ]; then
      RTF=$(awk -v a="$SN" -v b="$S0" -v e="$EL" 'BEGIN{d=a-b; if(d>0&&e>0) printf "%.3f", d/e; else print "—"}')
    fi
    echo "  …等待装载 轮询=${T2}s｜墙钟=${EL}s｜sim=${SN}s（起于 ${S0}s）｜RTF≈${RTF}｜DWA=$DWA_LOADED teb=$TEB_LOADED"
  fi
  sleep 4; T2=$((T2+4))
done
LOAD=$(grep -ac 'has been loaded successfully' "$DWA_OUT/mbf.log" 2>/dev/null | head -1)
SN=$(sim_now); EL=$(( $(date +%s) - W0 ))
echo "  装载等待结束：轮询 ${T2}s｜墙钟 ${EL}s｜sim=${SN}s（起于 ${S0}s）｜DWA 命中=$DWA_LOADED teb 命中=$TEB_LOADED（loaded 总累计=$LOAD，含 planners/recovery）"
if [ "${DWA_LOADED:-0}" -lt 1 ] || [ "${TEB_LOADED:-0}" -lt 1 ]; then
  # R88 第二版（根因已证＝重定向日志全缓冲，见证据 r88_b1_w4_try4/index.txt）：
  #   日志里"看不见装载行"不等于"没装载"——子进程 stdio 全缓冲，行会晚于事件几十分钟才落盘，
  #   甚至要到进程退出才刷出来。⇒ **本门只警示，不再有权 exit 6**；就绪与否改由功能结果判定
  #   （下面 dwa_recheck.py 会真发 goal、读 outcome；装不上它自然拿不到 outcome，rc 语义已覆盖）。
  echo "  【警示】到 ${LOAD_BOUND}s 仍未在日志里看到控制器装载行 ⇒ 不据此作废（日志全缓冲会造假否定），"
  echo "          继续进判据阶段，以 goal 的 outcome 为准（墙钟 ${EL}s｜sim ${SN}s｜起始 ${S0}s）"
fi
LIST=$(timeout 15 rosparam get /move_base_flex/controllers 2>/dev/null | tr -d '\n' | grep -o "name: '[A-Za-z]*'")
echo "  控制器注册表: $LIST"
echo "  （注：root-inertia WARN 现指 base_link，小张 M2R5 风险①预测成立；已 grep 全仓无脚本消费该串）"

PASS=0; FAIL=0; INVALID=0; BOTH=0
for i in $(seq 1 "$N"); do
  echo "### 二、DWA 第 $i 跑（三连判据）"
  python3 "$PKG/verify/dwa_recheck.py" DWAPlannerROS "dwa$i" > "$DWA_OUT/dwa$i.txt" 2>&1
  RC=$?
  sed 's/^/    /' "$DWA_OUT/dwa$i.txt"
  POSEERR=$(grep -acE "Could not get the current robot pose|robot pose is .* old" "$DWA_OUT/mbf.log" | head -1)
  echo "  rc=$RC ；mbf 日志内 pose 类命中累计=${POSEERR:-0}"
  # 单跑 rc 四态（DWP 侧 R40 新增 7＝有效性不过与判据不过并存，不再被 6 吸收）
  if [ $RC -eq 0 ]; then PASS=$((PASS+1)); echo "  [PASS] dwa 第 $i 跑"
  elif [ $RC -eq 6 ]; then INVALID=$((INVALID+1)); echo "  [INVALID] dwa 第 $i 跑（RTF 不达标，作废重跑，不计 FAIL）"
  elif [ $RC -eq 7 ]; then INVALID=$((INVALID+1)); FAIL=$((FAIL+1)); BOTH=1
    echo "  [INVALID+FAIL 并存] dwa 第 $i 跑（有效性门不过且判据亦不过 ⇒ 停手上报，不由重试消化、不计 PASS）"
  else FAIL=$((FAIL+1)); echo "  [FAIL] dwa 第 $i 跑 rc=$RC"; fi
  sleep 3
done
echo "### 三、同场 teb 对照（证 harness 本身没坏）"
python3 "$PKG/verify/dwa_recheck.py" TebLocalPlannerROS teb_ctrl > "$DWA_OUT/teb_ctrl.txt" 2>&1
TC=$?; sed 's/^/    /' "$DWA_OUT/teb_ctrl.txt"
if [ $TC -eq 0 ]; then echo "  [PASS] teb 对照"
else INVALID=$((INVALID+1)); echo "  [INVALID] teb 同场对照不过 rc=$TC ⇒ harness 本身存疑，本批不作可用性结论（R40：对照位是前置，不是被测件判据）"; fi
echo "### 汇总 DWA PASS=$PASS FAIL=$FAIL INVALID=$INVALID（目标 PASS=$N）；teb 对照 rc=$TC"
kill $LP >/dev/null 2>&1
if [ "$FAIL" -gt 0 ]; then echo "DWA RECHECK VERDICT: FAIL（确有判据不过，非环境因素）"
elif [ "$PASS" = "$N" ] && [ "$TC" = "0" ]; then echo "DWA RECHECK VERDICT: PASS（DWA 可用，判据三连全过）"
elif [ "$PASS" -lt "$N" ] && [ "$INVALID" -gt 0 ]; then echo "DWA RECHECK VERDICT: INCOMPLETE（$INVALID 次因 RTF 作废须重取；已 $PASS 次判据全过、0 次判据失败 ⇒ 不判 DWA 不可用）"
else echo "DWA RECHECK VERDICT: INCOMPLETE（凑不满 $N 次有效跑，且不属 RTF 作废，须人工看日志）"; fi
# ---- 批级 rc 显式分档（R40 核心：判据 FAIL 必须能改批级 rc，否则整件是假判据）----
if   [ "$BOTH" = "1" ]; then _RD=7
elif [ "$FAIL" -gt 0 ];  then _RD=5
elif [ "$INVALID" -gt 0 ]; then
  if [ "$PASS" = "$N" ] && [ "$TC" = "0" ]; then _RD=0; else _RD=6; fi
elif [ "$PASS" = "$N" ] && [ "$TC" = "0" ]; then _RD=0
else _RD=4
fi
echo "DWA RECHECK 批级 rc=$_RD（0 PASS／5 FAIL／6 INVALID／7 并存停手／4 INCOMPLETE；本件自有层，引用须连件名）"
exit "$_RD"
