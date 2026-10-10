#!/usr/bin/env bash
# v_scan_tf_parent_probe.sh (STP) —— SCAN 窗内的 base_link 父集合只读探针（小明 2026-10-10，批2 条1／条4 的取数件）
#
# 为何单立一件（先说清它补的是哪个洞）：批2（FD-G 双父根除，行 F2 已于 2026-10-08 11:54 落地 98d6a72）的复跑清单
#   第 1 条「E3 与双父签名」与第 4 条「FD-6 明细」**从来没有过 SCAN 窗的活体读数**。在册那五份
#   base_link_parents=['odom']（证据/log/raw/r81_b1_stageB5/b1..b5.log:25）出自 **stageB 窗**：其被试链是
#   m2_smoke.sh（m2_batch_runner.sh:43），roslaunch 面只有 m2_gazebo／m2_mapping（m2_smoke.sh:95/100）
#   ⇒ **不含 70_scan.launch、不含 relay**。而 FD-G 的发作条件恰是「relay 的静态边 body→base_link 与插件
#   的动态边 odom→base_link 同窗」（设计/fd_双写者与前馈源裁定.md:66、FD-6 表 :212），
#   该窗结构上不可能发作 ⇒ 拿它抵批2 的义务＝**假承接**（02 坑55：真读数＋错被测面）。
#
# 读数纪律（三条写死，都在册有出处）：
#   ① **只打印不判据**：小美 M2R7 结案 四节加项二原文「批1 内 FD 断言保持只打印」＋批1 行 C「正式入 gate 随批1b」。
#      ⇒ 本件不新增任何 gate 行、不进 m2_smoke.sh 的 gate 字典、不动任何门值；rc 只表「取没取到／有没有发作」。
#   ② **空集不得判绿**（02 坑29 一族）：窗内一条变换都没扫到 ⇒ rc=9 INVALID，绝不打印成"没双父"。
#   ③ **就绪信号不用 grep 日志**（02 坑53／坑54：重定向日志全缓冲会造假否定）⇒ 本件等的是话题／TF 数据本身。
#
# 两相序（由调用方驱动，见 设计/r_批2复取承接对账.md §3；本件只负责"在一个已起的窗里取一次"）：
#   对照相：只读地另起一次上游基线件（roslaunch plan_manage lio_relay.launch），期望父集合 **两元素** {odom, body}
#           ⇒ 先证明这探针抓得到 FD-G，才有资格谈实照相的零命中。
#   实照相：恢复在册 70_scan.launch（include 指壳包 lio_relay_sim.launch），期望父集合 **恰 ['odom']**
#           且 /tf_static 里以 base_link 为子的边数 = 0（FD-6 表 :212 第二句）。
#   停手条（照 fd :212 原文，不新设）：实照相若见 {odom, body} ⇒ **立即停手上报、判 TF 结构回归**，
#           不走 INVALID 通道、不静默继续 ⇒ 本件打 rc=7 并打印 STOP-FLAG 行。
#
# 用法：bash v_scan_tf_parent_probe.sh --selftest            # 零节点：解析层正反例＋空集中止＋元检＋缺席防呆（须 8 全绿）
#       bash v_scan_tf_parent_probe.sh <phase标签>            # 活体只读采样（须已有链与 master），缺省标签 live
#       环境变量：STP_WIN=3.0（采样窗秒数；/tf 与 /tf_static **同窗**累计）｜STP_CHILD=base_link
# 退出码：0 取到有效读数且未见双父（＝"未见发作"，**不称通过判据**）｜7 双父发作（停手）｜9 INVALID 前置缺失｜5 自测有 FAIL
# 性质：零 ws 写、零编译、零 /cmd_vel、无 sudo；只读 /tf 与 /tf_static 与 rosnode list。
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
set -u
MODE="live"; PHASE="live"
case "${1:-}" in
  --selftest) MODE="selftest"; PHASE="-" ;;
  "")          ;;
  *)           PHASE="$1" ;;
esac
WIN="${STP_WIN:-3.0}"; CHILD="${STP_CHILD:-base_link}"
echo "STP 环境自证：mode=${MODE} phase=${PHASE} ROS_MASTER_URI=${ROS_MASTER_URI:-未设} win=${WIN} child=${CHILD}"
OUT=$(python3 - "${MODE}" "${WIN}" "${CHILD}" "${PHASE}" <<'PYEOF'
# -*- coding: utf-8 -*-
# 解析核（两模式共用；自测就是在喂它）：rows = [(parent, child, stamp_float, is_static)]
import sys, time, os

MODE, WIN, CHILD, PHASE = sys.argv[1], float(sys.argv[2]), sys.argv[3], sys.argv[4]


def norm(f):
    # tf2 帧名可带前导斜杠；带与不带必须是同一帧，否则父集合会假分成两元素（也假合并不相关帧）
    return (f or '').strip().lstrip('/') or ''


def parse(rows, child=CHILD):
    par, stat_in, total = {}, 0, 0
    for p, c, s, is_static in rows:
        p, c = norm(p), norm(c)
        if p == '' or c == '':
            continue
        total += 1
        par.setdefault(c, set()).add(p)
        if c == child and is_static:
            stat_in += 1
    child_seen = child in par
    return sorted(par.get(child, [])), stat_in, total, sorted(par.keys()), child_seen


def report(rows, phase, src):
    par, stat_in, total, frames, child_seen = parse(rows)
    print('PROBE phase=%s 来源=%s 采样：变换行=%d 帧数=%d' % (phase, src, total, len(frames)))
    if total == 0 or not frames:
        print('PROBE INVALID：窗内一条变换都没扫到（扫描面空集）⇒ **不得读作"没有双父"**（坑29）')
        return 9
    if not child_seen:
        print('PROBE INVALID：目标帧 %s 根本不在 TF 图里（扫到 %d 行／%d 个帧，但没有它的父边）'
              '⇒ 父集合为空是**缺席**不是"没有双父"，与绿是两回事（同坑29 的另一种形态：'
              '被试名写错／链没起全也会"零命中"）' % (CHILD, total, len(frames)))
        return 9
    print('PROBE %s_parents=%s' % (CHILD, par))
    print('PROBE static_edges_into_%s=%d' % (CHILD, stat_in))
    agg = {}
    for p, c, s, is_static in rows:
        if norm(c) != CHILD:
            continue
        key = (norm(p), 'static' if is_static else 'dynamic')
        t, n = agg.get(key, (s, 0))
        agg[key] = (max(t, s), n + 1)
    for (p, kind), (s, n) in sorted(agg.items()):
        # last_stamp=0 ⇒ 该边来自被规范化成 Time(0) 的静态缓存（02 坑21／U1 一族），照 E3 打 [ZERO] 标注
        print('PROBE edge %s->%s %s hits=%d last_stamp=%.3f%s'
              % (p, CHILD, kind, n, s, ' [ZERO]' if s == 0.0 else ''))
    if len(par) >= 2:
        print('PROBE STOP-FLAG：base_link 有 %d 个父 %s ⇒ FD-G 发作签名（fd 稿 :212 的 {odom, body} 一族）'
              '⇒ 立即停手上报、判 TF 结构回归；不走 INVALID、不静默继续' % (len(par), par))
        return 7
    print('PROBE 未见发作（恰单父；**这是读数陈述，不是 gate 判定**——本件不入 gate、不动门值）')
    return 0


def fixtures_selftest():
    cnt = {'pass': 0, 'fail': 0}

    def check(rows, want_par, want_rc, want_static):
        got_par, got_static, _, _, _ = parse(rows)
        got_rc = report(rows, 'selftest', 'fixture')
        return (got_par == want_par and got_rc == want_rc and got_static == want_static), got_par, got_static, got_rc

    def t(name, rows, want_par, want_rc, want_static):
        ok, got_par, got_static, got_rc = check(rows, want_par, want_rc, want_static)
        print('[%s] %s（期望 parents=%s static=%d rc=%d｜实得 parents=%s static=%d rc=%d）'
              % ('PASS' if ok else 'FAIL', name, want_par, want_static, want_rc,
                 got_par, got_static, got_rc))
        cnt['pass' if ok else 'fail'] += 1

    dbl = [('odom', 'base_link', 10.0, False), ('body', 'base_link', 0.0, True),
           ('base_link', 'rslidar', 0.0, True)]
    t('T-1 阳性对照·上游 relay 在位（真实发作形态，须被认出双父并打 7）',
      dbl, ['body', 'odom'], 7, 1)
    t('T-2 实照相·壳包 relay（批2 期望态：恰 odom 单父、静态入边 0）',
      [('odom', 'base_link', 10.0, False), ('base_link', 'rslidar', 0.0, True),
       ('base_link', 'TORSO', 0.0, True), ('world', 'camera_init', 0.0, True)], ['odom'], 0, 0)
    t('T-3 前导斜杠归一（"/odom"+base_link 与 odom+"/base_link" 都是同一条边，不许假分成两元素）',
      [('/odom', '/base_link', 10.0, False), ('odom', 'base_link', 11.0, False),
       ('world', 'camera_init', 0.0, True)], ['odom'], 0, 0)
    t('T-4 双父的另一型（body 侧是动态而非静态，仍须判发作；静态入边计数只用于 FD-6 第二句）',
      [('odom', 'base_link', 10.0, False), ('body', 'base_link', 10.5, False)], ['body', 'odom'], 7, 0)
    t('T-5 空集必须 INVALID（扫描面为空的"零命中"不是绿）', [], [], 9, 0)
    t('T-6 坏行不进面（父名为空的畸形行不计入 total，防它把 total 顶成非零骗过 ②）',
      [('', 'base_link', 1.0, False), ('odom', 'base_link', 10.0, False)], ['odom'], 0, 0)
    t('T-8 目标帧缺席（图里有别的帧、就是没有 base_link 的父边）⇒ INVALID 不是"没有双父"',
      [('world', 'camera_init', 0.0, True), ('odom', 'some_other_frame', 5.0, False)], [], 9, 0)
    ok_meta, g_par, g_st, g_rc = check(dbl, ['odom'], 0, 0)
    print('[%s] T-7 元检（拿 T-1 那份双父 rows 去比"单父＋rc 0"的错期望，harness 必须判不成立；'
          '实得 parents=%s static=%d rc=%d ⇒ 错期望被认出＝绿）'
          % ('PASS' if ok_meta is False else 'FAIL', g_par, g_st, g_rc))
    cnt['pass' if ok_meta is False else 'fail'] += 1
    print('STP SELFTEST VERDICT: PASS=%d FAIL=%d' % (cnt['pass'], cnt['fail']))
    return 0 if cnt['fail'] == 0 else 5


def master_reachable(tmo=2.0):
    # 有界预检：没 master 时 rospy.init_node 会**一直挂着**（实测 60s 超时才回来）。
    # "跑不动"必须当场落成 INVALID(9)，不能占着窗等——02 坑53 同族（等待类上界必须显式写）。
    import socket
    from urllib.parse import urlparse
    u = urlparse(os.environ.get('ROS_MASTER_URI', ''))
    if not u.hostname:
        return False, 'ROS_MASTER_URI 不可解析'
    try:
        s = socket.create_connection((u.hostname, u.port or 11311), timeout=tmo)
        s.close()
        return True, '%s:%s' % (u.hostname, u.port or 11311)
    except Exception as e:
        return False, '%s:%s 不可连（%s）' % (u.hostname, u.port or 11311, type(e).__name__)


def live_sample():
    import rospy
    from tf2_msgs.msg import TFMessage
    ok, det = master_reachable()
    if not ok:
        print('PROBE INVALID：master 前置缺失＝%s ⇒ 链没起／起不动，属"跑不动"而非"跑不过"，'
              '**不判被试、不作结论**（细节见 02 坑53）' % det)
        return 9
    print('PROBE master 可达：%s（有界预检 2s）' % det)
    rospy.init_node('stp_tf_parent_probe', anonymous=True, disable_signals=True)
    rows = []
    nstat_msg = [0]

    def cb(m):
        for tr in m.transforms:
            rows.append((tr.header.frame_id, tr.child_frame_id, tr.header.stamp.to_sec(), False))

    def cb_static(m):
        nstat_msg[0] += 1
        for tr in m.transforms:
            rows.append((tr.header.frame_id, tr.child_frame_id, tr.header.stamp.to_sec(), True))

    # 静态面**不能**用 wait_for_message：/tf_static 是 latched，而 latching 是**逐发布者**各存一份，
    # wait_for_message 只返回最先到的那一条 ⇒ 多发布者时其余静态边整批漏读（本件首版就是这么漏掉
    # /body_to_base_link 的，实测见 证据/.../r93_batch2_live_20261010_153503）。改成窗开头建订阅、
    # 每个发布者连接时各投递一次 latch，窗内累计全部消息。
    sub = rospy.Subscriber('/tf', TFMessage, cb, queue_size=200)
    subS = rospy.Subscriber('/tf_static', TFMessage, cb_static, queue_size=200)
    w = time.time()
    while time.time() - w < WIN and not rospy.is_shutdown():
        time.sleep(0.05)
    sub.unregister()
    subS.unregister()
    print('PROBE 静态面投递：/tf_static 收到消息数=%d（latched 逐发布者各一条，>1 才算收全）' % nstat_msg[0])
    if nstat_msg[0] == 0:
        print('PROBE static=ERR 窗内一条 /tf_static 都没收到 ⇒ 静态入边计数不可用（**不得**读作"零条静态边"）')
    try:
        import subprocess
        nodes = subprocess.check_output(['rosnode', 'list']).decode().split()
        rel = [n for n in nodes if 'relay' in n or 'static_transform' in n or 'to_' in n]
        print('PROBE 窗型自证：TF 相关节点=%s（总节点=%d）' % (sorted(rel), len(nodes)))
    except Exception as e:
        print('PROBE 窗型自证=ERR %s' % type(e).__name__)
    return report(rows, PHASE, 'live /tf+/tf_static')


if MODE == 'selftest':
    sys.exit(fixtures_selftest())
try:
    sys.exit(live_sample())
except SystemExit:
    raise
except Exception as e:
    print('PROBE INVALID：活体前置缺失（%s: %s）⇒ 没链／没 master 都属"跑不动"，'
          '**不判被试、不作结论**（05 复验作业单的三态纪律）' % (type(e).__name__, e))
    sys.exit(9)
PYEOF
)
PYRC=$?
echo "${OUT}"
echo "=================================================="
echo "STP VERDICT rc=${PYRC}（0 未见发作／7 发作停手／9 INVALID 前置缺失／5 自测有 FAIL）"
exit ${PYRC}
