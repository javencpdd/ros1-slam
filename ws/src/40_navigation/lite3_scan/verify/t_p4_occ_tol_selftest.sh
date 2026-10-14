#!/bin/bash
# t_p4_occ_tol_selftest.sh —— #111／T-R34 的施工侧自测件（作业单 22 号 批1 步骤2 点名的新件）
# 性质＝★施工自测、不入门禁串（新增判据自测件入门仍归主理人）；★不改任何判据／门值／rc 语义／串成员
# 判据权威＝v5.3 §16.4 的 Z6 格原文＋`ap` 稿 #7 行（两者逐字均写：`min_dist_to_any_occ` 纳入判定、门值 0.10、
#            来源＝env 覆盖 `P4_OCC_TOL`（缺省 0.10、单一真身在代码）；本件只验"名实是否相符"，不新增语义）
# 被验件＝ws/src/40_navigation/lite3_scan/verify/p4_replan.py（本件对它零写入）
# 用法＝bash t_p4_occ_tol_selftest.sh    退出码＝0 全过／9 有 FAIL（自测件语义，与门禁串无关）
set +u
cd /home/ros1-slam || exit 9
export PYTHONNOUSERSITE=1
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
[ -f ws/devel/setup.bash ] && source ws/devel/setup.bash >/dev/null 2>&1
P=ws/src/40_navigation/lite3_scan/verify/p4_replan.py
NPASS=0; NTOT=0
chk() { NTOT=$((NTOT+1)); if [ "$2" = "$3" ]; then NPASS=$((NPASS+1)); echo "[PASS] $1：期望=$3／实得=$2"; else echo "[FAIL] $1：期望=$3／实得=$2"; fi; }
rd() { python3 -c "
import importlib.util, sys
sp='$P'
m=importlib.util.spec_from_file_location('p4r', sp); mod=importlib.util.module_from_spec(m)
m.loader.exec_module(mod)
exec(sys.argv[1])" "$1"; }
echo "== 档一 未设 env ⇒ 缺省读 0.10（单一真身在代码）=="
unset P4_OCC_TOL; chk "档一 OCC_BREACH_TOL 缺省" "$(rd 'print(mod.OCC_BREACH_TOL)')" "0.1"
echo "== 档二 env P4_OCC_TOL=0.25 ⇒ 读到 0.25 =="
export P4_OCC_TOL=0.25; chk "档二 env 覆盖生效" "$(rd 'print(mod.OCC_BREACH_TOL)')" "0.25"
echo "== 档三 两侧用例（恰在门值两侧，判定必须不同）=="
export P4_OCC_TOL=0.10
L=$(rd 'print(mod.a23_breach(0.0999)[0])'); R=$(rd 'print(mod.a23_breach(0.1001)[0])')
chk "档三 下侧 0.0999 应判 breach" "$L" "True"
chk "档三 上侧 0.1001 应不判" "$R" "False"
chk "档三 两面必须不同" "$([ "$L" != "$R" ] && echo diff || echo same)" "diff"
echo "== 档四 撤 env ⇒ 回缺省 0.10 =="
unset P4_OCC_TOL; chk "档四 撤例回缺省" "$(rd 'print(mod.OCC_BREACH_TOL)')" "0.1"
echo "== 反向对照 撤掉判据（tol=0）⇒ 同一穿墙样本必须转绿（Z6 硬要求：不反向就证明不了判据在起作用）=="
S=$(rd 'print(mod.a23_breach(0.020, 0.0)[0])'); T=$(rd 'print(mod.a23_breach(0.020)[0])')
chk "反向 撤判据转绿" "$S" "False"
chk "反向 带判据判红" "$T" "True"
echo "== 判定链是否真引用（名实相符检查，只读）=="
G=$(grep -c 'breach, btol = a23_breach(dmin)' "$P"); chk "主链调用 a23_breach 处在位" "$G" "1"
echo "T SELFTEST VERDICT: PASS=$NPASS/$NTOT（rc=0 全过／9 有 FAIL；★本件不入门禁串）"
[ "$NPASS" -eq "$NTOT" ] && exit 0 || exit 9
