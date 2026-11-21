#!/usr/bin/env bash
# v_task_scan_primitive.sh —— M5.2 scan_navigate 原语的**契约面**判据件（零节点，小明 2026-10-08）
#
# 为何单立一件（不扩 v_task_smach.sh 的 13 CHECK）：她那件已冻结在册（13/13 是她的复核基线），
#   我加判同改数＝她的基线漂移。新件只管「注册表 ↔ 实现」这一族——Q11 当年就是栽在话题名与实现漂移上。
# 段 A 静态契约（现在就能跑，零节点零 roscore）；段 B 活体 log 判据（给 log 才跑，缺 log 明确打「未跑」不落绿）。
# 用法：bash v_task_scan_primitive.sh [task_smach 运行 log]
#   退出码（本件自有层，不与 SM／BR／v_task_smach 各层互认）：0=段 A 全过（且给了 log 则段 B 也过）；
#   5=有 FAIL；9=前置缺失（件不在位／log 不可读＝不判被试）。
set -u
ROOT=/home/ros1-slam
TP=${TP_OVERRIDE:-$ROOT/ws/src/50_task/lite3_task/config/task_primitives.yaml}   # 覆写口只为阴性对照用（默认值即现役件）
TS=${TS_OVERRIDE:-$ROOT/ws/src/50_task/lite3_task/scripts/task_smach_min.py}
LOG=${1:-}
NP=0; NF=0; NA=0
mark(){ if [ "$1" = "0" ]; then NP=$((NP+1)); printf '[PASS] %s\n' "$2"; else NF=$((NF+1)); printf '[FAIL] %s\n' "$2"; fi; }
skip(){ NA=$((NA+1)); printf '[ABSENT] %s\n' "$1"; }
for f in "$TP" "$TS"; do [ -r "$f" ] || { echo "[FAIL] 前置件不可读：$f"; echo "TASK SCAN PRIMITIVE VERDICT: rc=9"; exit 9; }; done
command -v roslaunch >/dev/null 2>&1 || echo "  [注] 未 source ROS 环境——本件段 A 全是文件级检查，不需 ROS"

# ---- 段 A 静态契约 ----
# S1 阳性对照：注册表必须先解析出 ≥5 条原语（零命中不许当通过，坑29 纪律）
NPRIM=$(python3 - "$TP" <<'PYEOF'
import io,sys,yaml
d=yaml.safe_load(io.open(sys.argv[1],encoding='utf-8'))
print(len(d.get('primitives') or {}))
PYEOF
)
[ "${NPRIM:-0}" -ge 5 ] && mark 0 "S1 阳性对照：注册表解析出原语数=$NPRIM（≥5 才谈后面的在位与对账）" \
                        || mark 1 "S1 阳性对照不过：原语数=${NPRIM:-解析失败} ⇒ 解析器或件面失效"

# S2 scan_navigate 在位且 target 落在既有前缀空间
S2=$(python3 - "$TP" <<'PYEOF'
import io,sys,yaml
d=yaml.safe_load(io.open(sys.argv[1],encoding='utf-8'))
p=(d.get('primitives') or {}).get('scan_navigate') or {}
t=p.get('target','ABSENT')
print('%s|%s' % ('INPLACE' if p else 'ABSENT', 'OK' if t.startswith(('action:mbf_msgs/','local:')) else 'BAD:%s' % t))
PYEOF
)
[ "$S2" = "INPLACE|OK" ] && mark 0 "S2 scan_navigate 在位且 target 前缀在契约空间（$S2）" \
                        || mark 1 "S2 契约面不过：$S2"

# S3 二态口径逐字＝wait 原语的既有口径（不自立新语义）
S3=$(python3 - "$TP" <<'PYEOF'
import io,sys,yaml
d=yaml.safe_load(io.open(sys.argv[1],encoding='utf-8'))
pr=d['primitives']
a=pr.get('scan_navigate',{}).get('outcome_semantics','ABSENT')
b=pr.get('wait',{}).get('outcome_semantics','ABSENT')
print('%s|%s|%s' % (a, 'SAME' if a==b and a!='ABSENT' else 'DIFF', b))
PYEOF
)
case "$S3" in *"|SAME|"*) mark 0 "S3 outcome_semantics 与 wait 逐字同值（二态口径复用，未自造失败语义）";; *) mark 1 "S3 语义漂移：$S3";; esac

# S4 实现侧类与挂载点在位（类定义＋mode 分支＋transitions）
S4=$(python3 - "$TS" <<'PYEOF'
import io,re,sys
s=io.open(sys.argv[1],encoding='utf-8').read()
print('%d|%d|%d|%d' % (s.count('class ScanNavigateState'), s.count("reg['primitives']['scan_navigate']"),
                       s.count("'SCAN_NAVIGATE'"), s.count("mode == 'scan'")))
PYEOF
)
if [ "${S4%%|*}" = "1" ] && [ "$(echo "$S4" | cut -d'|' -f2)" = "1" ] && [ "$(echo "$S4" | cut -d'|' -f3)" -ge 2 ] && [ "$(echo "$S4" | cut -d'|' -f4)" -ge 1 ]; then
  mark 0 "S4 实现面在位：类定义/注册表读取/状态挂载(≥2)/mode 分支(≥1)（计数=$S4）"
else
  mark 1 "S4 实现面不齐（计数=$S4，应 1|1|≥2|≥1）"
fi

# S5 契约↔注入口逐字对账（注册表 goal_fields 的每一维都必须在实现里有对应 get_param，防 Q11 那族漂移）
S5=$(python3 - "$TP" "$TS" <<'PYEOF'
import io,sys,yaml,re
reg=yaml.safe_load(io.open(sys.argv[1],encoding='utf-8'))
src=io.open(sys.argv[2],encoding='utf-8').read()
f=reg['primitives']['scan_navigate'].get('goal_fields') or []
need={'target_pose':r'~goal_x|~goal_y','timeout_s':r'~scan_timeout_s','tol_m':r'~scan_tol_m','frame_id':r'~scan_goal_frame'}
miss=[k for k in f if k not in need or not re.search(need[k],src)]
print('%s|%s' % ('ALL' if not miss and f else 'MISS', ','.join(miss) or '-'))
PYEOF
)
[ "${S5%%|*}" = "ALL" ] && mark 0 "S5 goal_fields 四维与实现注入口逐字对账通过（target_pose/timeout_s/tol_m/frame_id）" \
                        || mark 1 "S5 契约与实现漂移：$S5"

# S6 门值不放宽：实现里的缺省 timeout/tol 必须等于 A7 判据③④ 原文（0.35／120）
S6=$(python3 - "$TS" <<'PYEOF'
import io,re,sys
s=io.open(sys.argv[1],encoding='utf-8').read()
t=re.search(r"~scan_timeout_s',\s*([0-9.]+)",s); o=re.search(r"~scan_tol_m',\s*([0-9.]+)",s)
print('%s|%s' % (t.group(1) if t else 'ABSENT', o.group(1) if o else 'ABSENT'))
PYEOF
)
[ "$S6" = "120.0|0.35" ] && mark 0 "S6 门值零放宽：实现缺省 timeout=120.0s／tol=0.35m ＝ A7 判据④③ 原文" \
                         || mark 1 "S6 门值与原文不符（$S6，应 120.0|0.35）"

# S7 失败侧不替上游造事件：aborted 只能由超时产生（静态数返回点＋看它带的字段）
S7=$(python3 - "$TS" <<'PYEOF'
import io,sys
s=io.open(sys.argv[1],encoding='utf-8').read()
seg=s[s.index('class ScanNavigateState'):s.index('\ndef main(')]
ret=[l.strip() for l in seg.split('\n') if l.strip() == "return 'aborted'"]
# 口径：类内**只允许一个** aborted 返回点，且类内必须出现超时判定/日志——
#   即「失败侧只有超时档」，不许把 arrived=False 当成上游失败事件转成 aborted（J1 明写不造失败信号）。
ok=(len(ret)==1 and ('timeout' in seg.lower()) and ('超时' in seg))
print('%d|%s' % (len(ret), 'CLEAN' if ok else 'CHECK'))
PYEOF
)
[ "${S7##*|}" = "CLEAN" ] && mark 0 "S7 失败侧口径：类内 aborted 返回点恰一处且带超时判定与超时日志（未把 arrived=False 当上游失败事件；读数=$S7）" \
                          || mark 1 "S7 失败侧出现非超时分支或缺超时档：$S7"

# S8 旧原语零改动（前 5 条目的规范化 dump 与 git HEAD 版逐字同）——防止我加东西时顺手动到别人在册契约
S8=$(python3 - "$TP" "$TS" <<'PYEOF'
import io,subprocess,sys,yaml
def norm(txt):
    d=yaml.safe_load(txt)
    pr=d.get('primitives') or {}
    return yaml.dump({k:pr[k] for k in sorted(pr) if k!='scan_navigate'}, sort_keys=True)
new=norm(io.open(sys.argv[1],encoding='utf-8').read())
old=subprocess.run(['git','-C','/home/ros1-slam','show','HEAD:ws/src/50_task/lite3_task/config/task_primitives.yaml'],
                   capture_output=True,text=True).stdout
if not old.strip():
    print('GITMISS|-'); raise SystemExit(0)
print('SAME' if norm(old)==new else 'DIFF')
PYEOF
)
[ "$S8" = "SAME" ] && mark 0 "S8 旧原语（除 scan_navigate）与 HEAD 逐字同值 ⇒ 我没顺手动到在册契约" \
                   || mark 1 "S8 旧原语被改动或 git 取不到（$S8）"

# S9 默认模式零回归：mode 缺省仍是 chain（不改既有 P6/联调口径）
S9=$(python3 - "$TS" <<'PYEOF'
import io,re,sys
s=io.open(sys.argv[1],encoding='utf-8').read()
m=re.search(r"~mode',\s*'([a-z]+)'",s)
print(m.group(1) if m else 'ABSENT')
PYEOF
)
[ "$S9" = "chain" ] && mark 0 "S9 mode 缺省仍=chain（scan 需显式点，默认行为零变化）" \
                    || mark 1 "S9 mode 缺省被改（$S9）"

# ---- 段 B 活体 log 判据（给 log 才跑）----
if [ -n "$LOG" ]; then
  if [ -r "$LOG" ]; then
    b1=$(grep -c '原语 scan_navigate outcome=succeeded' "$LOG")
    b2=$(grep -c 'task outcome: succeeded' "$LOG")
    b3=$(grep -oE '到达偏差 [0-9.]+m' "$LOG" | tail -1)
    b4=$(grep -oE '耗时 [0-9.]+s' "$LOG" | tail -1)
    [ "$b1" = "1" ] && mark 0 "L1 原语行在册且 succeeded（$b3 $b4）" || mark 1 "L1 原语 succeeded 行数=$b1（应 1）"
    [ "$b2" = "1" ] && mark 0 "L2 task outcome=succeeded" || mark 1 "L2 未见 task outcome: succeeded"
    echo "  [注] 段 B 只证「任务层原语跑通并按 J1 到达」；规划层的避让／穿墙分辨力属 O-8，不在本件职责"
  else
    mark 1 "L0 log 不可读：$LOG"
  fi
else
  skip "段 B 活体 log 判据未跑（未给 log）⇒ 本节不给绿，也不给红"
fi

echo "=================================================="
# 旧写法把「（log 未给＝活体面未跑）」当成固定后缀印——于是段 B **真跑过**时（ABSENT=0）判定行仍说"活体面未跑"，
# 与同一行的 ABSENT=0 自相矛盾（坑62 那一族：两份陈述互相打架，且都语法合法）。改成随 ABSENT 计数条件印。
if [ "$NA" -gt 0 ]; then NOTE="（log 未给＝活体面未跑，本节不给绿也不给红）"; else NOTE="（段 B 活体 log 判据已跑）"; fi
echo "TASK SCAN PRIMITIVE VERDICT: PASS=$NP FAIL=$NF ABSENT=$NA$NOTE"
[ "$NF" = "0" ] && exit 0 || exit 5
