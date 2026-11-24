#!/usr/bin/env python3
"""运动中断探针：单进程完成「等运动 → 注入 → 观察窗口 → 进程内判据 + 原始序列落盘」。

判据全部在内存里算，不经过 rosbag。后者在这一项上坑过四轮，且都不是被测系统的问题：
  · `pkill -f` 模式匹配到执行它的 shell 自身，把脚本杀了、bag 没关闭；
  · `setsid rosbag & BAGPID=$!` 里 `$!` 是 setsid 的 pid，SIGINT 打不到 rosbag，
    读出来 318 KB 却 0 条消息；
  · `.active` 未索引、SIGINT 与 hold 不同步；
  · `rosbag` 的 duration 按**仿真时间**统计（rtf 0.46），15 s 墙钟只录出 6.5 s，
    于是「5 s 窗口」怎么设都不够 —— 这是判据口径问题，不是采样不够。
现在每条样本同时带 wall 与 sim 时间戳，判据显式指定口径。

时间锚点（这一项最容易假绿的地方）：
  · 注入时机等「首帧非零」再延迟 inject_delay_s，不靠固定 sleep —— 运动窗口常只有 2 s；
  · 停止延迟以**失效生效时刻**为锚：mode=stop 取闸门最后一条 `/hdl_odom` 的时刻。
    `rosnode kill` 是阻塞调用，返回时刻不等于断流时刻；把时间戳打在下发之前，
    会把命令行自身的耗时算进延迟（实测虚高约 1 s，曾把 0.24 s 报成 1.24 s）。
    mode=cancel 取取消报文发出时刻。
  · 零速窗口按**仿真时间**算：按 wall 算会在 rtf 0.46 下把 5 s sim 缩成 2.3 s sim。

配套：`goal_until_motion.py` 负责"目标只存在到车动起来为止"（否则取消后新目标
让系统合法重新起步，会被误读成取消没生效）；`cancel_replay_probe.py` 负责旧/新
路径的成对反控。

用法：
  motion_interrupt_probe.py --mode stop|cancel --hold-s 24 --out result.json --dump series.tsv
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

STOP_SPEED = 0.02

# /cmd_vel 是 geometry_msgs/Twist，**没有 header**（只有 linear/angular）。
# 想给每条命令配一个仿真时刻只能另订 /clock 并把最近一次 sim 时间抄进去。
# 曾经的写法 msg.header.stamp.to_sec() 直接 AttributeError 打死探针。
CLK = {'sim': None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=('stop', 'cancel'), required=True)
    ap.add_argument('--bag', default='')
    ap.add_argument('--on-speed', type=float, default=STOP_SPEED)
    ap.add_argument('--hold-s', type=float, default=10.0)
    ap.add_argument('--wait-motion-s', type=float, default=60.0)
    # 检测到运动后延迟多久再注入。默认 0；配合 goal_until_motion.py 时需要 >0，
    # 保证「目标已停发 → 再取消」，否则取消后随后的新目标会让系统合法重新起步。
    ap.add_argument('--inject-delay-s', type=float, default=0.0)
    ap.add_argument('--action', default='rosnode kill /hdl_gate')
    ap.add_argument('--out', default='')
    ap.add_argument('--dump', default='', help='原始序列落盘（TSV），判据可复核')
    args = ap.parse_args()

    bag_path = args.bag or os.path.abspath('motion.bag')

    # 1) 不启 rosbag：改为进程内记录（rosbag 的 SIGINT/reindex/sim-wall 混淆已四次坑死本项）
    samples = []

    state = {'frames': 0, 'first': None}

    def on_cmd(msg):
        state['frames'] += 1
        v = math.hypot(msg.linear.x, msg.linear.y)
        samples.append({'w': time.monotonic(), 's': CLK['sim'], 'v': v})
        if state['first'] is None and v >= args.on_speed:
            state['first'] = (time.monotonic(), v)

    from nav_msgs.msg import Odometry
    gt_samples = []

    def on_gt(msg):
        p = msg.pose.pose.position
        gt_samples.append({'w': time.monotonic(), 's': msg.header.stamp.to_sec(),
                           'x': p.x, 'y': p.y})

    ack_samples = []

    def on_ack(msg):
        ack_samples.append({'w': time.monotonic(), 's': CLK['sim'], 't': msg.data})

    # 闸门真正停发的时刻：`rosnode kill` 是阻塞调用，返回值不代表"这一刻才断"，
    # 用 /hdl_odom 的最后一条消息作为失效生效点，比给命令行打时间戳准得多。
    HDL = {'w': None, 'n': 0}

    def on_hdl(_msg):
        HDL['w'] = time.monotonic()
        HDL['n'] += 1

    def on_clock(msg):
        CLK['sim'] = msg.clock.to_sec()

    rospy.init_node('motion_interrupt_probe', anonymous=True)
    rospy.Subscriber('/clock', Clock, on_clock, queue_size=20)
    rospy.Subscriber('/cmd_vel', Twist, on_cmd, queue_size=50)
    rospy.Subscriber('/sim/odom_gt', Odometry, on_gt, queue_size=50)
    rospy.Subscriber('/v8/local/cancel_ack', String, on_ack, queue_size=20)
    rospy.Subscriber('/hdl_odom', Odometry, on_hdl, queue_size=50)

    # 2) 等首帧非零（车真的动了）
    t0 = time.monotonic()
    while not rospy.is_shutdown() and state['first'] is None:
        if time.monotonic() - t0 > args.wait_motion_s:
            break
        time.sleep(0.01)

    fired = state['first'] is not None
    t_inject = None
    t_effective = None
    if fired:
        fire_at, fire_v = state['first']
        if args.inject_delay_s > 0:
            time.sleep(args.inject_delay_s)
        if args.mode == 'stop':
            subprocess.call(args.action.split())
            act = args.action
        else:
            pub = rospy.Publisher('/v8/bt/cancel', String, queue_size=5)
            rospy.sleep(0.3)
            text = '1:all:operator'
            pub.publish(String(data=text))
            rospy.sleep(0.2)
            pub.publish(String(data=text))
            act = 'cancel ' + text
        # 注入动作**做完**才打时间戳：`rosnode kill` 是阻塞调用（实测耗时近 1 s），
        # 打在下发之前会把命令行自身的耗时算成"停止延迟"。
        t_inject = time.monotonic()
        print('INJECT at v=%.3f action=%s' % (fire_v, act), flush=True)
        # 3) 注入后继续观察 hold_s
        # ★必须用纯 time.sleep，不能带 rospy.is_shutdown() 条件★：
        # 实测注入后 rospy 的 shutdown 标志会变 True，循环立刻退出，
        # rosbag 随即被 SIGINT —— bag 的结束时刻正好等于"末非零命令"时刻，
        # 零速窗口覆盖 0.00 s，5 s 判据永远测不出来。
        t_fire = time.monotonic()
        time.sleep(args.hold_s)
        print('HOLD done %.2fs after inject' % (time.monotonic() - t_fire), flush=True)
    else:
        print('NO_MOTION: %.0f s 内未见非零 cmd（frames=%d）' % (args.wait_motion_s, state['frames']))

    # 4) 直接在内存里算判据（不碰 rosbag）
    result = {'mode': args.mode, 'motion_seen': fired,
              'cmd_frames': state['frames'], 'hold_s': args.hold_s,              'cmd_samples': len(samples), 'gt_samples': len(gt_samples),
              'ack_count': len(ack_samples)}
    if ack_samples:
        result['ack_last'] = ack_samples[-1]['t']
        fire_w = state['first'][0] if fired else None
        if fire_w is not None:
            result['ack_latency_wall_s'] = round(ack_samples[0]['w'] - fire_w, 3)
    if fired and samples and gt_samples:
        fire_w = state['first'][0]
        nz = [x for x in samples if x['v'] >= 0.02]
        last = nz[-1] if nz else None
        result['max_speed'] = round(max(x['v'] for x in samples), 3)
        if last:
            after_v = [x['v'] for x in samples if x['w'] > last['w']]
            result['nonzero_frames'] = len(nz)
            result['last_nonzero_wall_s'] = round(last['w'] - fire_w, 3)
            result['zero_window_wall_s'] = round(samples[-1]['w'] - last['w'], 3)
            result['max_speed_after'] = round(max(after_v), 3) if after_v else 0.0
            # ★停止延迟以"失效生效时刻"为锚★：mode=stop 时闸门的最后一条
            # /hdl_odom 就是数据真正断掉的时刻（比 rosnode kill 的返回时刻准）；
            # 取消时以取消报文发出时刻为锚。
            if t_inject is not None:
                t_eff = t_inject
                if args.mode == 'stop' and HDL['w'] is not None:
                    t_eff = HDL['w']
                result['inject_wall_s'] = round(t_inject - fire_w, 3)
                if args.mode == 'stop' and HDL['w'] is not None:
                    result['hdl_last_wall_s'] = round(HDL['w'] - fire_w, 3)
                    result['hdl_msgs'] = HDL['n']
                result['stop_delay_s'] = round(max(0.0, last['w'] - t_eff), 3)
                result['inject_to_last_nonzero_s'] = round(last['w'] - t_inject, 3)
                result['moving_at_inject'] = bool(last['w'] >= t_eff)
                # 注入后真值滑行：从失效生效时刻算到观测结束，这才是"车又走了多远"
                base_i = [g for g in gt_samples if g['w'] <= t_eff]
                after_i = [g for g in gt_samples if g['w'] > t_eff]
                if base_i and after_i:
                    b_i = base_i[-1]
                    result['gt_coast_after_inject_m'] = round(math.hypot(
                        after_i[-1]['x'] - b_i['x'], after_i[-1]['y'] - b_i['y']), 4)
            base = [g for g in gt_samples if g['w'] <= last['w']]
            after_g = [g for g in gt_samples if g['w'] > last['w']]
            if base and after_g:
                b = base[-1]
                result['gt_travel_after_m'] = round(
                    math.hypot(after_g[-1]['x'] - b['x'], after_g[-1]['y'] - b['y']), 4)
                # ★窗口一律按仿真时间算★：rosbag/duration 与 rtf 都按 sim 统计，
                # 用 wall 窗口会在 rtf=0.46 时把 5 s sim 缩成 2.3 s sim，判据假严；
                # gt 自带 sim 时间戳，以「末非零命令时刻对齐的最后一帧 gt」为锚点。
                anchor_s = b['s']
                result['anchor_sim_s'] = round(anchor_s, 3)
                result['zero_window_sim_s'] = round(after_g[-1]['s'] - anchor_s, 3)
                for win in (2.0, 5.0, 10.0):
                    seg = [g for g in after_g if g['s'] <= anchor_s + win]
                    if seg:
                        result['gt_move_within_%.0fs_sim_m' % win] = round(max(
                            math.hypot(g['x'] - b['x'], g['y'] - b['y']) for g in seg), 4)
                        result['gt_covers_%.0fs_sim' % win] = bool(
                            after_g[-1]['s'] - anchor_s >= win - 0.05)
    # 存活核查：注入之后再数一次节点，证明"归零"不是整栈被关掉换来的。
    try:
        out = subprocess.check_output(['rosnode', 'list'],
                                      stderr=subprocess.DEVNULL).decode('utf-8', 'replace')
        names = [ln for ln in out.splitlines() if ln.startswith('/')]
        result['v8_nodes_after'] = sum(1 for n in names if n.startswith('/v8'))
        result['gazebo_alive'] = '/gazebo' in names
        result['hdl_alive'] = any(n.startswith('/hdl') for n in names)
    except Exception as exc:  # 数不出来要如实标 NA，不能默认活着
        result['v8_nodes_after'] = None
        result['alive_error'] = str(exc)[:120]

    if args.out:
        open(args.out, 'w').write(json.dumps(result, ensure_ascii=False) + '\n')
    # ★探针必须落盘原始序列★：只留汇总表等于自报，复核者无法重算窗口。
    if args.dump:
        fire_w = state['first'][0] if fired else None
        with open(args.dump, 'w') as fh:
            fh.write('# kind\tt_rel_wall\tsim\tspeed_or_x\ty\n')
            for x in samples:
                rel = '' if fire_w is None else '%.3f' % (x['w'] - fire_w)
                fh.write('cmd\t%s\t%s\t%.4f\t\n' % (
                    rel, '' if x['s'] is None else '%.3f' % x['s'], x['v']))
            for g in gt_samples:
                rel = '' if fire_w is None else '%.3f' % (g['w'] - fire_w)
                fh.write('gt\t%s\t%.3f\t%.4f\t%.4f\n' % (rel, g['s'], g['x'], g['y']))
            for a in ack_samples:
                rel = '' if fire_w is None else '%.3f' % (a['w'] - fire_w)
                fh.write('ack\t%s\t%s\t%s\t\n' % (
                    rel, '' if a['s'] is None else '%.3f' % a['s'], a['t']))
    print(json.dumps(result, ensure_ascii=False))
    return 0 if fired else 2


if __name__ == '__main__':
    sys.exit(main())
