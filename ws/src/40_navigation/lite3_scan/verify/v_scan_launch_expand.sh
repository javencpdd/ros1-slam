#!/usr/bin/env bash
# v_scan_launch_expand.sh (SLE) —— 70_scan.launch 的 **roslaunch 展开级** 断言（小美 M2R8 第 22 条／R41 之一 的欠项）
#
# 为何单立一件：批2 把 70_scan.launch 的 include 目标改指壳包件 RSIM，当时只过了 xmllint。
#   她当场指出：**xmllint 不验 $(find lite3_scan) 能不能解析**，而 $(find) 的可解析性正是批2 唯一的新拓扑风险；
#   且共同门禁 [5]/[6] 段自记「跨行 include 不在扫描面内」⇒ 静态面也不覆盖。⇒ 批2 活体之前本应做一次展开断言（这一步我当时漏了，见 04 更正句）。
#   本件补的是「展开级」证据：走 roslaunch 自己的解析器与替换引擎，不是再 grep 一遍文本。
#
# 零节点、零 roscore、零仿真：只用 --deps／rospkg／substitution_args，不起任何节点、不占任何端口。
# 用法：（先按三步环境 source）bash v_scan_launch_expand.sh     退出码 0=全过；5=有 FAIL；9=前置缺失（rospack 面为空＝没环境，不判被试）
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
set -u
ROOT=/home/ros1-slam
L70=$ROOT/ws/src/40_navigation/lite3_scan/launch/70_scan.launch
NP=0; NF=0
mark(){ if [ "$1" = "0" ]; then NP=$((NP+1)); printf '[PASS] %s\n' "$2"; else NF=$((NF+1)); printf '[FAIL] %s\n' "$2"; fi; }
[ -f "$L70" ] || { echo "[FAIL] 被测 launch 不在位：$L70"; echo "SLE VERDICT: rc=9"; exit 9; }
# 前置闸：rospack 面为空 ⇒ 是没 source 环境，不是隔离违规（她 R40／R46 那一族：前置不成立走独立档，别打假红灯）
if [ -z "$(rospack find lite3_scan 2>/dev/null)" ]; then
  echo "[INVALID] rospack 解析不到 lite3_scan ⇒ 三步环境未生效，本件未起跑（不判被试）"
  echo "SLE VERDICT: rc=9"; exit 9
fi
echo "  环境自证：lite3_scan＝$(rospack find lite3_scan)｜ROS_MASTER_URI=$ROS_MASTER_URI"

# L1 阳性对照：展开必须先能解析出 ≥1 个 include（零命中不许当通过，坑29 纪律）
INC=$(python3 - "$L70" <<'PYEOF'
import io,re,sys
s=io.open(sys.argv[1],encoding='utf-8').read()
print(len(re.findall(r'<include\b[^>]*',s,re.S)))
PYEOF
)
[ "${INC:-0}" -ge 1 ] && mark 0 "L1 阳性对照：文本面解析出 include 数=$INC（≥1 才谈后面的零命中）" \
                      || mark 1 "L1 阳性对照不过：include 数=${INC:-0} ⇒ 探针或件面失效"

# L2 roslaunch --files 全展开必须 rc=0（noetic 的 roslaunch 已无 --deps，--files/--nodes 才是现役展开面——
#    本条首版写 --deps 得 rc=2 usage 错，属「探针自己不会用」的假红，改正后用真展开面）
FILES=$(roslaunch --files "$L70" 2>/tmp/sle_files_err.txt); frc=$?
NFILE=$(echo "$FILES" | grep -c '^/')
[ "$frc" = "0" ] && [ "$NFILE" -ge 2 ] && mark 0 "L2 roslaunch --files 展开 rc=0，实读文件数=$NFILE（含跨包 include，均被替换引擎解析成功）" \
                 || { mark 1 "L2 --files 展开不过（rc=$frc 文件数=$NFILE）"; sed 's/^/      [files-err] /' /tmp/sle_files_err.txt | head -3; }

# L3 $(find lite3_scan) 在替换引擎里落到现役壳包件，且该文件真实存在
RES=$(python3 - <<'PYEOF'
from roslaunch.substitution_args import resolve_args
print(resolve_args('$(find lite3_scan)/launch/lio_relay_sim.launch'))
PYEOF
)
[ -f "$RES" ] && case "$RES" in *40_navigation/lite3_scan/launch/lio_relay_sim.launch) mark 0 "L3 替换引擎解析目标＝$RES（在位且属壳包）";; *) mark 1 "L3 解析到意外路径：$RES";; esac \
              || mark 1 "L3 解析结果不是在册文件：${RES:-空}"

# L4 展开级零命中：70 的 include 目标串里不得再指 30_3rd 的旧 relay（上游引用消失，展开后再核一次）
OLD=$(python3 - "$L70" <<'PYEOF'
import io,re,sys
s=io.open(sys.argv[1],encoding='utf-8').read()
tg=[m.group(1).strip() for m in re.finditer(r'<include\b[^>]*?file="([^"]*)"',s,re.S)]
print(sum(1 for t in tg if '30_3rd' in t or ('lite3_slam' in t and 'lio_relay.launch' in t)), len(tg))
PYEOF
)
NOLD=$(echo "$OLD" | awk '{print $1}'); NTOT=$(echo "$OLD" | awk '{print $2}')
[ "${NOLD:-1}" = "0" ] && [ "${NTOT:-0}" -ge 1 ] && mark 0 "L4 include 目标零命中旧件（目标数=$NTOT，指 30_3rd 者=$NOLD）" \
                       || mark 1 "L4 展开级仍有旧件引用（目标数=$NTOT 命中=$NOLD）"

# L5 批1 行 B 守恒：odom_in 的 remap 目标逐字仍是 /Odometry（接线改动没顺手动到源名）
ODO=$(python3 - "$L70" <<'PYEOF'
import io,re,sys
s=io.open(sys.argv[1],encoding='utf-8').read()
m=re.findall(r'name="odom_in"\s+value="([^"]*)"',s)
print(m[0] if m else 'ABSENT')
PYEOF
)
[ "$ODO" = "/Odometry" ] && mark 0 "L5 odom_in 逐字＝/Odometry（批1 行 B 未被批2 改动波及）" \
                         || mark 1 "L5 odom_in 变了或缺失：$ODO"

# L6 展开清单里壳包 relay 必须以**绝对路径**在位，且旧 relay 不得出现（展开级证据，比文本 grep 强一档）
HRSIM=$(echo "$FILES" | grep -c '40_navigation/lite3_scan/launch/lio_relay_sim.launch')
H3RD=$(echo "$FILES" | grep -c '30_3rd.*lio_relay[.]launch')
[ "$HRSIM" = "1" ] && [ "$H3RD" = "0" ] \
  && mark 0 "L6 展开读到的 relay 只有壳包件（rsim=1 旧件=0）；30_3rd 侧仍被读的是 advanced_param.xml（参数件，非 relay）" \
  || mark 1 "L6 展开面 relay 指认异常（rsim=$HRSIM 应1，旧件=$H3RD 应0）"

# L7 展开后的节点面：relay 与 SCAN 消费者都必须在拓扑里（零 roscore，只看 --nodes 的展开结果）
NODES=$(roslaunch --nodes "$L70" 2>/dev/null | tr -d ' ')
NNODE=$(echo "$NODES" | grep -c '^/')
echo "$NODES" | grep -q '^/lio_odom_vehicle_relay$' && echo "$NODES" | grep -q '^/scan_planner_node$' && [ "$NNODE" -ge 5 ] \
  && mark 0 "L7 展开节点数=$NNODE，含 /lio_odom_vehicle_relay 与 /scan_planner_node（批2 接线在展开态成立）" \
  || mark 1 "L7 展开节点面缺件（节点数=$NNODE 清单=[$NODES]）"

echo "=================================================="
echo "SLE VERDICT: PASS=$NP FAIL=$NF"
[ "$NF" = "0" ] && exit 0 || exit 5
