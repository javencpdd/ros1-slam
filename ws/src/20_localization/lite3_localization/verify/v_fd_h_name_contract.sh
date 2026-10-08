#!/usr/bin/env bash
# v_fd_h_name_contract.sh —— 批4（FD-H 名字契约）的**静态对账件**（零节点，小明 2026-10-09）
#
# 背景一句话：hdl 第三方 nodelet 实发绝对名 /odom，而在册声明与 Stage C 读者要的是 /hdl_odom
#   ⇒ 不做契约，即使 hdl 完全健康，gateC 的 hdl_odom 键也恒 False（"跑不动"冒充"跑不过"，坑29／R40 那一族）。
# 处置按小张裁定 (a)：只在**自有壳包** hdl_localization.launch 加一行 remap，第三方一字不动。
# 本件证的是"契约在位且没越界"；**活体出帧核对留 G8 同场**（段 A 不给绿、也不给红，见末段 ABSENT）。
#
# 用法：bash v_fd_h_name_contract.sh     退出码：0=全过；5=有 FAIL；9=前置缺失（件不在位／git 不可用）
set -u
ROOT=/home/ros1-slam
cd "$ROOT" || exit 9
HDL=ws/src/20_localization/lite3_localization/launch/hdl_localization.launch
NODE=ws/src/20_localization/hdl_localization/apps/hdl_localization_nodelet.cpp
MH=ws/src/95_simulation/lite3_gazebo_sim/launch/m2_hdl.launch
SM=ws/src/95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh
BASE=note/进度/SCAN集成/证据/scan_src_baseline_49.sha256
NP=0; NF=0; NA=0
mark(){ if [ "$1" = "0" ]; then NP=$((NP+1)); printf '[PASS] %s\n' "$2"; else NF=$((NF+1)); printf '[FAIL] %s\n' "$2"; fi; }
absent(){ NA=$((NA+1)); printf '[ABSENT] %s\n' "$1"; }
for f in "$HDL" "$NODE" "$MH" "$SM" "$BASE"; do [ -r "$f" ] || { echo "[FAIL] 前置件不可读：$f"; echo "FDH VERDICT: rc=9"; exit 9; }; done

# 把"remap 是否在 hdl 节点块内且 from/to 逐字对"做成函数，好让**被污染的副本**也走同一判定（阴性对照不另写一套逻辑）
check_contract(){ # $1=launch 路径  ⇒ 输出 OK / 三种失败原因
  local f="$1" blk
  blk=$(awk '/name="hdl_localization_nodelet"/{p=1} p{print} p&&/<\/node>/{exit}' "$f")
  [ -n "$blk" ] || { echo "NOBLOCK"; return; }
  echo "$blk" | grep -qE '<remap[[:space:]]+from="/odom"[[:space:]]+to="/hdl_odom"[[:space:]]*/>' \
    && echo "OK" || { echo "$blk" | grep -q 'from="/odom"' && echo "BADMAPPING" || echo "NOREMAP"; }
}

# N1 阳性对照：发布点探针必须读得到（读不到则后面全是空集真命题）
PUB=$(grep -c 'advertise<nav_msgs::Odometry>("/odom"' "$NODE")
[ "$PUB" = "1" ] && mark 0 "N1 阳性对照：第三方发布点探针命中 nodelet.cpp 的 advertise(\"/odom\")" \
                 || mark 1 "N1 探针读不到发布点（命中=$PUB，应 1）⇒ 本件其余判定不可信"

# N2 绝对名 remap 不会伤输入：nodelet 的订阅里不得有 /odom，且订阅总数必须 >0（否则 N2 是空集真命题）
SUBN=$(grep -c 'nh\.subscribe' "$NODE"); SUBODOM=$(grep -c 'subscribe<[^>]*>("/odom"\|subscribe("/odom"' "$NODE")
[ "$SUBN" -ge 3 ] && [ "$SUBODOM" = "0" ] && mark 0 "N2 订阅面安全：订阅=$SUBN 条、其中无 /odom ⇒ remap 只改输出" \
                   || mark 1 "N2 订阅面不安全（订阅=$SUBN 含 /odom=$SUBODOM）⇒ remap 可能改到输入，须改方案"

# N3 在册读者与声明仍以 /hdl_odom 为契约（我没顺手把读者改名）
R1=$(grep -c '/hdl_odom' "$MH"); R2=$(grep -c '/hdl_odom' "$SM")
[ "$R1" -ge 1 ] && [ "$R2" -ge 3 ] && mark 0 "N3 读者侧未动：m2_hdl.launch 命中=$R1、m2_smoke.sh 命中=$R2（≥3）" \
                                    || mark 1 "N3 读者侧被改或声明漂移（launch=$R1 smoke=$R2）⇒ 属判据载体变更，须走 A3"

# N4 契约本体：remap 行必须**在 hdl 节点块内**且 from/to 逐字正确（写在块外＝nodelet 载进 manager 后不生效）
C=$(check_contract "$HDL")
[ "$C" = "OK" ] && mark 0 "N4 契约在位：hdl_localization_nodelet 块内 from=/odom to=/hdl_odom 逐字匹配" \
                 || mark 1 "N4 契约不符（$C）"

# N5 同块既有绝对名 remap 先例（说明这种写法在本壳包里已被实证，不是新发明的招）
ABS=$(awk '/name="hdl_localization_nodelet"/{p=1} p{print} p&&/<\/node>/{exit}' "$HDL" | grep -cE '<remap from="/')
[ "$ABS" -ge 2 ] && mark 0 "N5 块内绝对名 remap 共 $ABS 条（含本次）⇒ 写法先例在位" \
                 || mark 1 "N5 块内绝对名 remap 仅 $ABS 条，缺先例支撑"

# N6 第三方零越界：①git 面上第三方各包零改动 ②SCAN 30_3rd 基线双条件（内容全等＋集合计数）
GITDIRS="ws/src/20_localization/hdl_localization ws/src/20_localization/hdl_global_localization ws/src/20_localization/fast_gicp ws/src/20_localization/ndt_omp ws/src/30_3rd ws/src/10_slam/faster_lio"
CHG=$(git status --porcelain -- $GITDIRS 2>/dev/null | wc -l)
NB=$( ( cd ws/src/30_3rd && sha256sum -c "$ROOT/$BASE" 2>/dev/null | grep -c ': OK$' ) )
CNT=$(find ws/src/30_3rd -type f ! -name '*.sha256' | wc -l)
[ "$CHG" = "0" ] && [ "$NB" = "49" ] && [ "$CNT" = "49" ] \
  && mark 0 "N6 第三方零越界：六处第三方树 git 改动=$CHG、30_3rd 基线可验=$NB 且集合计数=$CNT（双条件，D2 口径）" \
  || mark 1 "N6 越界或基线不符（git 改动=$CHG 可验=$NB/49 计数=$CNT/49）"

# N7 帧名走 param 不受话题名影响（FD-G 双父那族坑的边界条件，钉住它别和名字混改）
# 注意匹配面：launch 里 frame_id 出现两次是**合法结构**（arg 声明 + param 引用），首版我按 `name="odom_child_frame_id"` 计数
#   得 2 就报 FAIL——那是我自己的断言匹配面比目标宽（同 DWR 那句"命令行里带过这个字样"那一族），不是件有问题。
FRA=$(grep -c '<arg name="odom_child_frame_id"' "$HDL")
FRP=$(grep -c '<param name="odom_child_frame_id"' "$HDL")
[ "$FRA" = "1" ] && [ "$FRP" = "1" ] && mark 0 "N7 帧名走 param 且只此一处（arg 声明=$FRA、param=$FRP）⇒ 改话题名不动 TF 边" \
                 || mark 1 "N7 帧名结构异常（arg=$FRA 应1，param=$FRP 应1）"

# N8 阴性对照：把 to= 改坏一个字母，同一判定函数必须真红（不这么测，N4 就是"少写断言也能绿"的假件）
mkdir -p /tmp/fdh_neg && cp -p "$HDL" /tmp/fdh_neg/hdl_tampered.launch
sed -i 's|to="/hdl_odom"|to="/hdl odom"|' /tmp/fdh_neg/hdl_tampered.launch
CN=$(check_contract /tmp/fdh_neg/hdl_tampered.launch)
rm -rf /tmp/fdh_neg
[ "$CN" != "OK" ] && mark 0 "N8 阴性对照：to 被改坏后同一判定给 $CN（不是照旧绿）⇒ N4 有分辨力" \
                   || mark 1 "N8 阴性对照不过：改坏契约仍判 OK ⇒ N4 是假绿灯"

absent "活体面（G8 同场）：rostopic hz /hdl_odom 出帧、stageC 的 hdl_odom 键由恒 False 转 True——本件不以此段冒充已验"
echo "=================================================="
echo "FDH VERDICT: PASS=$NP FAIL=$NF ABSENT=$NA（活体面按设计未跑，不计入通过数）"
[ "$NF" = "0" ] && exit 0 || exit 5
