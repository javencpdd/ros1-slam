#!/usr/bin/env bash
# b18_farcorner_diag.sh —— v4.4 X1「远角红线一次性诊断豁免」窗：1 对远角 × 2 次尝试
#   目的（唯一）＝回答 B19 剩下的那一小块：运行时放宽 planner_patience 之后，perlin3d 远角对到底能不能规划出来。
#   授权链：主理人 2026-10-11 裁「同批出计划书补丁稿」⇒ 补丁稿＝note/计划书v4/21_…-v4.4-…补丁.md（X1 定范围与不变式）；本脚本与补丁同批。
#   范围红线（照 X1）：只跑 goal[0] 这一对；不跑第 2/3/4 对；不改任何门值；不动任何在册件（本脚本在证据目录，起链只用现成 launch 的 arg）。
#   安全：起窗前 load≤4；A1 现取红线基准（见证覆盖面＝4）；样张 sha 起前＝退后；foxy 侧 PID 逐位一致；teardown 残留清零；FORCE=1 禁用。
set -u
RUN="/home/ros1-slam/note/进度/SCAN集成/证据/log/raw/r174_b18_farcorner_diag"
SAMPLE="/home/ros1-slam/data/maps/lite3.pcd"
GOALIDX=0
mkdir -p "$RUN"

say() { printf '%s %s\n' "$(date '+%H:%M:%S')" "$*"; }

# ---------- 0. 前置（不过就 INVALID 退出，不判被试）----------
L=$(awk '{print $1}' /proc/loadavg)
ok=$(python3 -c "print(1 if float('$L')<=4.0 else 0)")
[ "$ok" = "1" ] || { echo "前置不成立：load=$L＞4 ⇒ 不起窗（rc=9 语义）"; exit 9; }
ss -lnt 2>/dev/null | grep -qE ':1131[34]' && { echo "前置不成立：11313/11314 已有监听 ⇒ 端口面不干净（rc=9）"; exit 9; }
pgrep -f 'roslaunch lite3_bringup|gzserver|move_base' | grep -q . && { echo "前置不成立：已有在册链在跑 ⇒ 互斥（rc=9）"; exit 9; }
date '+%F %T' > "$RUN/preflight_time.txt"
uptime >> "$RUN/preflight_time.txt" 2>&1
ps -o pid,lstart,cmd -p $(pgrep -d, -f 'ros2 launch transfer|jetson2app|jetson2motion|sensor_checker' | head -c 200) > "$RUN/foxy_start.txt" 2>&1
sha256sum "$SAMPLE" > "$RUN/sample_before.sha" 2>&1

# 轨迹源：原 /tmp/matrix_w3/ 工作集与 launch 默认件均不在盘上 ⇒ 只用在册证据树里的存档副本（静止于 0.05,0.05）
TRAJ_SRC="/home/ros1-slam/note/进度/仿真测试台/证据/log/matrix_w3/traj_perlin3d.yaml"
TRAJ="$RUN/traj_perlin3d.yaml"
TRAJ_WANT="9d40854314764843383d849e439541660fcc908e77ba79fa9ae67302c5c8a97e"   # 出处＝matrix_w3/w3_证据对账.sha256 第 34 行
[ -f "$TRAJ_SRC" ] || { echo "前置不成立：存档轨迹源不在盘 ⇒ 无法复现在册形状（rc=9）"; exit 9; }
cp "$TRAJ_SRC" "$TRAJ"
got=$(sha256sum "$TRAJ" | awk '{print $1}')
[ "$got" = "$TRAJ_WANT" ] || { echo "前置不成立：轨迹源 sha 不符（$got ≠ $TRAJ_WANT）⇒ 不起窗（rc=9）"; exit 9; }
echo "$got  $TRAJ" > "$RUN/traj_sha_ok.txt"

# ROS 的 profile.d/1.ros_distro.sh 会展开 $ROS_DISTRO，而本件在 set -u 下刚 unset 过它 ⇒ 标准 source 写法在本档会当场 abort（R174 首跑实证）。只在 source 段临时关 -u。
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313

# ---------- 1. 起链（照在册 w3_t1_probe2.sh 的 perlin3d arg，一字不改那些参数）----------
setsid roslaunch lite3_bringup mode_nav_sim.launch rviz:=false use_gpu:=false \
  map_type:=1 map_seed:=511 map_x:=40 map_y:=20 map_z:=5 map_res:=0.15 \
  perlin_complexity:=0.07 perlin_fill:=0.1 perlin_fractal:=1 perlin_attenuation:=0.1 \
  map_grid:=40 map_traj:="$TRAJ" > "$RUN/launch.log" 2>&1 &
# setsid 会 fork 后即退 ⇒ $! 不是 roslaunch 本体，据它 kill 会打空、留下孤儿 master（坑24）⇒ 起后按名取真 PID 与其组长
sleep 5
RL_PID=$(pgrep -f 'roslaunch.*mode_nav_sim' | head -1 || true)
RL_PG=$(ps -o pgid= -p "${RL_PID:-0}" 2>/dev/null | tr -d ' ' || true)
echo "rl_pid=${RL_PID:-none} rl_pg=${RL_PG:-none} started=$(date '+%T')" > "$RUN/launch_pid.txt"
say "roslaunch 真 PID=${RL_PID:-未取到}（组长 ${RL_PG:-?}）；等就绪（上限 300s）"

# ---------- 2. 客户端：基线一次 → 放宽 patience → 再一次 → 还原 ----------
python3 - "$RUN" "$GOALIDX" <<'PYEOF' 2>&1 | tee "$RUN/probe_result.txt"
# -*- coding: utf-8 -*-
import json, math, os, sys, time
import rospy, actionlib
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from mbf_msgs.msg import GetPathAction, GetPathGoal
from actionlib_msgs.msg import GoalStatus
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.msg import Config as DRConfig

RUN, GOALIDX = sys.argv[1], int(sys.argv[2])
goals = [[-19.55, -9.55], [19.65, -9.75], [19.65, 9.25], [-19.65, 9.25]]
start = [0.05, 0.05]
gx, gy = goals[GOALIDX]
out = {"pair": {"start": start, "goal": [gx, gy], "index": GOALIDX,
               "euclid_xy": round(math.hypot(gx - start[0], gy - start[1]), 2)},
       "attempts": [], "patience": {}, "note": []}

def line(msg):
    print(msg); sys.stdout.flush()

rospy.init_node('b18_farcorner_diag')
t0 = time.time()
client = actionlib.SimpleActionClient('move_base_flex/get_path', GetPathAction)
line("[gate] 等 get_path action 服务器（上限 300s）…")
if not client.wait_for_server(rospy.Duration(300)):
    out["verdict"] = "INVALID|前置未成立：get_path 服务器 300s 未就绪 ⇒ 不判被试"
    line("B18 诊断窗结论 = " + out["verdict"]); json.dump(out, open(os.path.join(RUN, 'b18.json'), 'w'), ensure_ascii=False, indent=1)
    sys.exit(9)
line("[gate] 服务器就绪，用时 %.1fs" % (time.time() - t0))

try:
    dr = Client('/move_base_flex', timeout=30)
    cfg0 = dr.get_configuration()
    keys = sorted(k for k in cfg0.keys() if 'patience' in k.lower())
    out["patience"]["keys_found"] = {k: cfg0[k] for k in keys}
    line("[DR] 含 patience 的键与现值：%s" % out["patience"]["keys_found"])
    pk = [k for k in keys if k.endswith('planner_patience')] or [k for k in keys if 'planner' in k]
    if not pk:
        out["verdict"] = "INVALID|前置未成立：DR 里找不到 planner_patience 类键（%s）⇒ 不判被试" % keys[:6]
        line("B18 诊断窗结论 = " + out["verdict"]); json.dump(out, open(os.path.join(RUN, 'b18.json'), 'w'), ensure_ascii=False, indent=1)
        sys.exit(9)
except Exception as e:
    out["verdict"] = "INVALID|前置未成立：DR 客户端不可用（%s）⇒ 不判被试" % e
    line("B18 诊断窗结论 = " + out["verdict"]); json.dump(out, open(os.path.join(RUN, 'b18.json'), 'w'), ensure_ascii=False, indent=1)
    sys.exit(9)

def ps(x, y):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = x; p.pose.position.y = y; p.pose.orientation.w = 1.0
    return p

GOAL_STATE = {0:'PENDING',1:'ACTIVE',2:'PREEMPTED',3:'SUCCEEDED',4:'ABORTED',
              5:'REJECTED',6:'PREEMPTING',7:'RECALLING',8:'RECALLED',9:'LOST'}
OUTCOME = {0:'SUCCESS',50:'FAILURE',51:'CANCELED',52:'INVALID_START',53:'INVALID_GOAL',
           54:'NO_PATH_FOUND',55:'PAT_EXCEEDED',56:'EMPTY_PATH',57:'TF_ERROR',
           58:'NOT_INITIALIZED',59:'INVALID_PLUGIN',60:'INTERNAL_ERROR',61:'OUT_OF_MAP',62:'MAP_ERROR',63:'STOPPED'}

def run_once(tag, budget):
    g = GetPathGoal()
    g.use_start_pose = True            # 钉死起点＝在册那对，不让"机器人当下在哪"混进读数
    g.start_pose = ps(start[0], start[1])
    g.target_pose = ps(gx, gy)
    g.tolerance = 0.0
    t = time.time()
    client.send_goal(g)
    done = client.wait_for_result(rospy.Duration(budget))
    dt = time.time() - t
    rr_err = ''
    try:
        r = client.get_result()
    except Exception as e:
        r, rr_err = None, 'get_result 抛错 %s' % e
    if r is None and not rr_err:
        rr_err = 'result 为 None（等待超时或服务器未回话）'
    oc, st, msg, npath, herr = None, '', '', None, rr_err
    if r is not None:
        # 逐字段独立取：一个字段取错不得把已拿到的 outcome 覆盖掉（attempt1 正是这么废掉的）
        try: oc = r.outcome
        except Exception as e: herr += ' | outcome 取失败 %s' % e
        try: msg = (r.message or '')[:80]
        except Exception as e: herr += ' | message 取失败 %s' % e
        try: npath = len(r.path.poses)
        except Exception as e: herr += ' | path 取失败 %s' % e
    try: st = str(client.get_state())   # 状态在 actionlib 客户端侧，不在 GetPathResult 里
    except Exception as e: st = 'state 取失败 %s' % e
    cancelled = False
    if not done:
        # 未回话 ⇒ 目标仍是 ACTIVE：既占着规划线程，又卡住 mbf 的 reconfigure 锁（attempt2 的 DR 就是这么落不了地的）
        # 在册 SimpleActionClient 只有 wait_for_server／wait_for_result／get_state／cancel_goal／get_result，没有 wait_for_state ⇒ 轮询到终态
        client.cancel_goal()
        cancelled = True
        TERMINAL = {GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                    GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST}
        tc = time.time()
        while time.time() - tc < 20:
            if int(client.get_state()) in TERMINAL:
                break
            time.sleep(0.5)
        try: st = str(client.get_state())
        except Exception as e: st = 'state 取失败 %s' % e
        _reached = int(st) in TERMINAL if st.lstrip('-').isdigit() else False
        out.setdefault("cancel", []).append(
            {"tag": tag, "terminal_reached": _reached, " waited_s": round(time.time() - tc, 1),
             "state_name": GOAL_STATE.get(int(st), st) if st.lstrip('-').isdigit() else st})
        line('[cancel] 目标未回话 ⇒ 已 cancel_goal；取消后状态=%s（%s，等 %.1fs，到终态＝%s）' % (
            st, GOAL_STATE.get(int(st), '?') if st.lstrip('-').isdigit() else '?',
            time.time() - tc, _reached))
    name = 'NO-OUTCOME' if oc is None else OUTCOME.get(oc, 'UNKNOWN(%s)' % oc)
    rec = {"tag": tag, "outcome": oc, "outcome_name": name,
           "state": st, "state_name": GOAL_STATE.get(int(st), st) if str(st).lstrip('-').isdigit() else st,
           "message": msg, "path_poses": npath, "wall_s": round(dt, 1),
           "result_returned": (r is not None), "wait_returned": bool(done),
           "cancelled_by_harness": cancelled, "harness_error": herr.strip()}
    out["attempts"].append(rec)
    line("[attempt %s] outcome=%s(%s) state=%s path_poses=%s 墙钟=%.1fs msg=%s%s" % (
        tag, oc, name, st, npath, dt, msg, ('  ERR=' + herr.strip()) if herr else ''))
    return rec

def patience_now():
    try:
        c = dr.get_configuration()
        return {k: c[k] for k in pk}
    except Exception as e:
        return {"error": str(e)}

_cap = {"n": 0, "vals": {}}
def _cap_cb(cfg):          # 名字不能与 _cap 字典同名，否则 def 会把字典顶掉（本轮差点又犯）
    _cap["n"] += 1
    for p_ in cfg.doubles:
        if p_.name in pk:
            _cap["vals"][p_.name] = p_.value

# ① 地图就绪门：global_costmap 的 static_layer 吃的就是 map_topic: map ⇒ 没有 /map 就没有"规划"这回事，门不过即 INVALID
line("\n=== 前置门：等 /map（上限 60s）===")
try:
    m = rospy.wait_for_message('/map', OccupancyGrid, timeout=60)
    out["map_gate"] = {"ok": True, "w": m.info.width, "h": m.info.height, "res": m.info.resolution}
    line("[地图门] /map 就绪 %dx%d @%.3fm（launch 请求的是 40x20 @0.15m ⇒ 若不同，本窗的\"远角\"是在别的图上测的，必须记下来）"
         % (m.info.width, m.info.height, m.info.resolution))
    occ = []
    for tname, ttype in rospy.get_published_topics():
        if ttype == 'nav_msgs/OccupancyGrid':
            occ.append(tname)
            try:
                mm = rospy.wait_for_message(tname, OccupancyGrid, timeout=3)
                occ[-1] = '%s = %dx%d @%.3fm' % (tname, mm.info.width, mm.info.height, mm.info.resolution)
            except Exception:
                occ[-1] = '%s（无新消息／latched 未命中）' % tname
    out["map_gate"]["occupancy_topics"] = occ
    line("[地图普查] 盘上所有 OccupancyGrid 话题：%s" % occ)
except Exception as e:
    out["map_gate"] = {"ok": False, "err": str(e)}
    out["verdict"] = "INVALID|前置未成立：/map 60s 未就绪（%s）⇒ 不发 goal、不判被试" % e
    line("B18 诊断窗结论 = " + out["verdict"])
    json.dump(out, open(os.path.join(RUN, 'b18.json'), 'w'), ensure_ascii=False, indent=1)
    sys.exit(9)

# ② 放宽：先赶在**一次 goal 都没发过**的时候写（attempt1 空闲时落地过、attempt2/4 有 goal 挂着时没落地）。
#    但 attempt5 已把我上一版的说法**证伪了一半**：真空闲（goal 数＝0）时按客户端缓存看仍报"未落地"
#    ⇒ 所以"空闲"未必是充分条件，也可能一直是**回执载体**读错了；本窗用双路回执把这两件事分开。
pat_default = patience_now()
out["patience"]["before_set"] = pat_default
line("\n=== 放宽 planner_patience 到 60.0s（此刻已发过的 goal 数＝%d，即真空闲）===" % len(out["attempts"]))
line("[空闲门] 写 DR 前客户端状态=%s（GOAL_STATE %s）；未发过目标时应为 PENDING" % (
    client.get_state(), GOAL_STATE.get(int(client.get_state()), '?')))
_upd_sub = rospy.Subscriber('/move_base_flex/parameter_updates', DRConfig,
                            lambda cfg: _cap_cb(cfg), queue_size=10)
time.sleep(0.5)                      # 先收 latched 那份（＝开机配置），它**不是**"写之后"的回执
_base_n = _cap["n"]
try: rp_before = rospy.get_param('/move_base_flex/planner_patience')
except Exception as e: rp_before = 'rosparam 取失败 %s' % e
try:
    dr.update_configuration({k: 60.0 for k in pk})
except Exception as e:
    out["note"].append("DR set 抛错：%s" % e)
tp = time.time()
while time.time() - tp < 15 and _cap["n"] <= _base_n:
    time.sleep(0.3)
try: rp_after = rospy.get_param('/move_base_flex/planner_patience')
except Exception as e: rp_after = 'rosparam 取失败 %s' % e
pat_after = patience_now()
tp = time.time()
while time.time() - tp < 10:
    if pk and all(abs(float(v) - 60.0) < 1e-6 for v in pat_after.values()):
        break
    time.sleep(1.0)
    pat_after = patience_now()
bcast_new = dict(_cap["vals"]) if _cap["n"] > _base_n else None
out["patience"].update({"rosparam_before_set": rp_before, "rosparam_after_set": rp_after,
                        "broadcast_before_set_latched": dict(_cap["vals"]),
                        "broadcast_after_set": bcast_new, "broadcast_msgs_seen": _cap["n"],
                        "after_set_client_cache": pat_after})
_upd_sub.unregister()
def _is60(d):
    return bool(d) and bool(pk) and all(abs(float(d.get(k, -1)) - 60.0) < 1e-6 for k in pk)
got_rp = isinstance(rp_after, float) and abs(rp_after - 60.0) < 1e-6
got_bc = _is60(bcast_new)
got_ca = bool(pk) and all(abs(float(v) - 60.0) < 1e-6 for v in pat_after.values())
got = got_rp or got_bc or got_ca
line("[DR 回执三取] rosparam %s→%s｜写后广播=%s（共见 %d 条）｜客户端缓存=%s ⇒ 落地＝%s（相符通道＝%s）" % (
    rp_before, rp_after, bcast_new, _cap["n"], pat_after, got,
    "+".join([n for n, f in (('rosparam', got_rp), ('写后广播', got_bc), ('缓存', got_ca)) if f]) or '无'))
if not got:
    out["verdict"] = ("INVALID|前置未成立：patience 未确认放宽（服务器广播＝%s／客户端缓存＝%s，两路都不是 60.0）⇒ 不判被试"
                      % (srv, pat_after))
    line("B18 诊断窗结论 = " + out["verdict"])
    try:
        dr.update_configuration(pat_default)
    except Exception:
        pass
    json.dump(out, open(os.path.join(RUN, 'b18.json'), 'w'), ensure_ascii=False, indent=1)
    sys.exit(9)

# ③ 唯一一次被试尝试（X1 的 ②「放宽后重试」；①「默认 patience 基线」本轮不占额度——它已由在册 W3 那 8 行与我方 attempt1 的 3.2s 墙钟带着）
line("\n=== 被试尝试：放宽后（60s patience）发该对 ===")
a2 = run_once('after_relaxed_patience_idle_set_first', 90)

# ④ 尽力还原（链随即拆掉，还原只是不留悬空态；goal 若卡住则还原必然落不了地，如实记）
try:
    dr.update_configuration(pat_default)
except Exception as e:
    out["note"].append("还原 set 抛错：%s" % e)
out["patience"]["restored"] = patience_now()
line("[DR 还原] %s" % out["patience"]["restored"])

# 判定：先自证探针、再判被试（探针废了就不许把脏读数算成被试的 FAIL）
# 但**"服务器整窗没回 result"不是探针故障**——那是被试侧的读数（第三型），别拿自证门把它挡成 INVALID（attempt7 就是这么误判的）
_read_err = [a["tag"] for a in (a2,)
             if a["result_returned"] and (a["outcome"] is None or '取失败' in a["harness_error"])]
if _read_err:
    v = ("INVALID|前置未成立：探针在 %s **拿到了 result 却没读出 outcome**（harness_error 见 attempts）"
         "⇒ 本窗不判被试，也不得记为远角失败" % "、".join(_read_err))
elif a2["outcome"] == 0:
    v = ("PASS(诊断意义=改述)|空闲时放宽 patience 到 60s 后该对规划出来了：outcome=0、路径点=%s、墙钟 %.1fs"
         "⇒ 在册『不可终止』那半句应升为『已证：长 patience 下可规划』；"
         "**不改 perlin3d 标准档裁撤定性**（其依据是 60 点单里其余确定性 55 与长程能力边界）"
         % (a2["path_poses"], a2["wall_s"]))
elif a2["outcome"] == 55:
    v = ("FAIL(诊断意义=坐实)|patience 已确认放宽到 60s（回查值见 patience.after_set），该对仍在 %.1fs 拿到 PAT_EXCEEDED(55)、路径点=%s"
         "⇒ 在册『长 patience 仍无规划』这一表述由『未证』升为『已证（本对、本图、本 boot）』"
         "＝t 稿 §4.1 那句『再等一等等不等得到』现在有答案了：**等不到**"
         % (a2["wall_s"], a2["path_poses"]))
elif not a2["result_returned"]:
    v = ("第三型(诊断意义=服务面)|patience 60s 已落地，但服务器在 %.1fs 内**连一次 result 都没回**（state=%s，取消是否到终态见 cancel）"
         "⇒ 这**不是** patience 问题，是 t 稿 §4.2 分列的服务/挂起型；"
         "『不可终止』若继续写成 patience 超时＝归因错，该改述为『服务器对本对不回话』"
         % (a2["wall_s"], a2["state_name"]))
else:
    v = ("第四型(如实报，不硬套三档)|patience 60s 已落地，该对回了话但码既非 0 也非 55："
         "outcome=%s(%s)、message=%r、路径点=%s、墙钟 %.1fs ⇒ 需要按这个码重新定归因，不由我猜"
         % (a2["outcome"], a2["outcome_name"], a2["message"], a2["path_poses"], a2["wall_s"]))
out["verdict"] = v
line("\nB18 诊断窗结论 = " + v)
json.dump(out, open(os.path.join(RUN, 'b18.json'), 'w'), ensure_ascii=False, indent=1)
PYEOF
PRC=${PIPESTATUS[0]}

# ---------- 3. teardown 与红线复算（关窗手法照在册 w3_t1_probe2.sh：rosnode kill -a ＋ 按名 pkill）----------
say "关窗：rosnode kill -a → 结束 roslaunch 组长 → 按名兜底"
rosnode kill -a >/dev/null 2>&1; sleep 3
[ -n "${RL_PG:-}" ] && kill -TERM "-$RL_PG" 2>/dev/null
sleep 6
pkill -f 'roslaunch.*mode_nav_sim' 2>/dev/null; pkill -f move_base_flex 2>/dev/null
pkill -f sim_adapter 2>/dev/null; pkill -f mockamap 2>/dev/null; pkill -f render_node 2>/dev/null
sleep 3
[ -n "${RL_PG:-}" ] && kill -KILL "-$RL_PG" 2>/dev/null
sleep 2
pgrep -af 'roslaunch lite3_bringup|gzserver|move_base|mode_nav_sim|sim_adapter|mockamap|render_node' > "$RUN/teardown_residue.txt" 2>&1 || true
[ -s "$RUN/teardown_residue.txt" ] || echo "（零残留）" > "$RUN/teardown_residue.txt"
ss -lnt 2>/dev/null | grep -E ':1131[0-9]|:1134[0-9]' > "$RUN/teardown_ports.txt" 2>&1 || echo "端口零监听" > "$RUN/teardown_ports.txt"
grep -q ':1131' "$RUN/teardown_ports.txt" && echo "警告：11311–11314 仍有监听 ⇒ 孤儿 master 风险（坑24），须人工确认后再开下一窗" >> "$RUN/teardown_ports.txt"
ps -o pid,lstart,cmd -p $(pgrep -d, -f 'ros2 launch transfer|jetson2app|jetson2motion|sensor_checker' | head -c 200) > "$RUN/foxy_end.txt" 2>&1
sha256sum "$SAMPLE" > "$RUN/sample_after.sha" 2>&1
{ echo "== 样张 sha 起前 vs 退后 =="; diff "$RUN/sample_before.sha" "$RUN/sample_after.sha" && echo "  同（未被触碰）" || echo "  异 ⇒ 本窗不可引用";
  echo "== foxy 侧 PID 起前 vs 退后 =="; diff "$RUN/foxy_start.txt" "$RUN/foxy_end.txt" >/dev/null && echo "  逐位一致（零触碰；覆盖面＝4 unit 见证，monitor 两 unit 主理人裁定不复原）" || echo "  有差异 ⇒ 本窗不可引用";
  echo "== 残留进程 =="; cat "$RUN/teardown_residue.txt"; echo "== 端口 =="; cat "$RUN/teardown_ports.txt"; } > "$RUN/redline_summary.txt" 2>&1
say "探针退出码=$PRC"; cat "$RUN/redline_summary.txt"
exit "$PRC"
