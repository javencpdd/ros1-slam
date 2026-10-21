#!/usr/bin/env bash
# w3 预检 ③④ 活体探针：costmap_2d 双 StaticLayer 实例同栈（§8.3）＋ unknown=-1 的解释差异（§8.4）
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313
RUN=/tmp/w3probe; LOG=$RUN/run.log; : > "$LOG"
say(){ echo "[$(date '+%T')] $*" | tee -a "$LOG"; }
CN="";PN="";RS=""
cleanup(){ say "-- teardown --"; [ -n "$CN" ] && kill "$CN" 2>/dev/null; [ -n "$PN" ] && kill "$PN" 2>/dev/null; [ -n "$RS" ] && kill "$RS" 2>/dev/null; sleep 3
  say "退净：11313 监听=$(ss -lnt 2>/dev/null|grep -c ':11313')（须 0）"; }
trap cleanup EXIT
say "== [1] roscore 11313 =="
roscore -p 11313 > "$RUN/roscore.log" 2>&1 & RS=$!
for _ in $(seq 1 30); do rostopic list >/dev/null 2>&1 && break; sleep 1; done
say "== [2] 双图发布器（map_a 含 100/50，map_b 含一行 -1）=="
python3 $RUN/pub.py > "$RUN/pub.log" 2>&1 & PN=$!
sleep 5
say "== [3] 按节点命名空间装载参数 =="
rosparam load $RUN/params.yaml costmap_probe && say "  rosparam load rc=0（参数键数=$(rosparam list 2>/dev/null | grep -c '^costmap_probe/')）"
say "== [4] costmap_2d_node =="
rosrun costmap_2d costmap_2d_node __name:=costmap_probe > "$RUN/costmap.log" 2>&1 & CN=$!
sleep 12
say "  节点在位=$(rosnode list 2>/dev/null | grep -c costmap_probe)"
say "  话题面：$(rostopic list 2>/dev/null | grep -E 'costmap_probe/' | tr '\n' ' ')"
say "== [5] 采一次 /costmap_probe/costmap 并解格 =="
timeout 25 rostopic echo -n1 /costmap_probe/costmap > "$RUN/cost.txt" 2>&1
say "  采到行数=$(wc -l < "$RUN/cost.txt")；data 面=$(grep -c 'data:' "$RUN/cost.txt")"
python3 - <<'PY'
import re
t=open('/tmp/w3probe/cost.txt',encoding='utf-8',errors='ignore').read()
if 'data:' not in t:
    print("  [FAIL] 没采到 costmap 的 data 面 ⇒ 双实例探针未成立（看 costmap.log）"); raise SystemExit(1)
body=t.split('data:',1)[1]
vals=[int(x) for x in re.findall(r'(-?\d+)',body) if True][:40*40]
print("  格子数=%d（期望 40*40=1600）" % len(vals))
def at(r,c): return vals[r*40+c] if len(vals)>r*40+c else None
print("  map_a 的 100 区(行10 列10..14) 读回：", [at(10,10+i) for i in range(5)])
print("  map_a 的 50 点(行20 列20)   读回：", at(20,20))
print("  map_b 的 -1 区(行30 列0..4) 读回：", [at(30,i) for i in range(5)])
print("  自由区(行5 列5)             读回：", at(5,5))
print("  未标注区(行35 列35)         读回：", at(35,35))
print("  值域：min=%s max=%s；出现过的 distinct 值=%s"%(min(vals),max(vals),sorted(set(vals))[:12]))
print("  判读口径（设计稿 §4.3 的禁令）：costmap 侧若把源图的 -1 读成 254/负一等『未知』码 ⇒ 禁令被自动尊重；")
print("                    若读成 0（自由）⇒ 『未覆盖≠自由』必须由我方层自己保证，不能指望 costmap。")
PY
say "  插件装载与告警：$(grep -iE 'static_a|static_b|unknown|error|warn' "$RUN/costmap.log" | head -4 | tr '\n' '|')"
