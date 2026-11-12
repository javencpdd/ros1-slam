#!/usr/bin/env python3
"""SCAN+BT 真机装配的只读预检（对应 SCAN真机接入清单.md §2）。

不起栈、不发目标、不发速度；只订阅/查询，逐项打印 PASS/FAIL。
判据：
  1. 关键节点在位（BT/桥/规划/跟踪/到达/门禁/安全 mux）
  2. /v8/bt/status 存在且 planner_mode=signal（arrival signal 生效）
  3. /lite3/scan/arrived 在线且当前 False（无残留到达锁存）
  4. /cmd_vel 速率 ≥10 Hz 且采样窗内线速度/角速度全 0（不动车）
  5. /global_path/status 在线（桥→BT 路径就绪判据）
退出码：0 全 PASS；1 有 FAIL；9 前置不成立（拿不到 /v8/bt/status）。
"""
import argparse
import json
import sys

REQUIRED_NODES = (
    'scan_planner_node',
    'closed_loop_controller',
    'scan_arrival_monitor',
    'scan_bt_bridge',
    'bt_console',
    'v8_goal_gate',
    'safe_cmd_vel_mux',
)


def parse_bt_status(text):
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def main():
    import rospy
    import rosgraph
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool, String

    parser = argparse.ArgumentParser()
    parser.add_argument('--window', type=float, default=4.0, help='采样窗秒数')
    args = parser.parse_args()

    rospy.init_node('scan_real_preflight', anonymous=True)
    master = rosgraph.Master('/scan_real_preflight')
    checks = []

    def record(name, ok, detail):
        checks.append((name, ok, detail))
        print('%s %-28s %s' % ('PASS' if ok else 'FAIL', name, detail))

    states = master.getSystemState()
    nodes = set(states[0]) | set(states[1]) | set(states[2])
    missing = [n for n in REQUIRED_NODES if '/' + n not in nodes]
    record('nodes', not missing, 'missing=' + (','.join(missing) or '-'))

    try:
        status_msg = rospy.wait_for_message('/v8/bt/status', String, timeout=8)
    except rospy.ROSException:
        print('INVALID 拿不到 /v8/bt/status（BT 未起或 master 不对）')
        return 9
    status = parse_bt_status(status_msg.data)
    if status is None:
        record('bt_status_json', False, 'status 不是 JSON')
        return 1
    mode = status.get('planner_mode', '?')
    record('planner_mode', mode == 'signal', 'planner_mode=%s' % mode)
    record('bt_idle', status.get('outcome', '') in ('', None) and status.get('queue_depth', 0) == 0,
           'outcome=%r queue_depth=%s' % (status.get('outcome'), status.get('queue_depth')))

    try:
        arrived = rospy.wait_for_message('/lite3/scan/arrived', Bool, timeout=5)
        record('arrived_false', not arrived.data, 'arrived=%s' % arrived.data)
    except rospy.ROSException:
        record('arrived_false', False, '话题离线')

    speeds = []

    def on_cmd(message):
        twist = message
        speeds.append(max(abs(twist.linear.x), abs(twist.linear.y), abs(twist.linear.z),
                          abs(twist.angular.x), abs(twist.angular.y), abs(twist.angular.z)))

    sub = rospy.Subscriber('/cmd_vel', Twist, on_cmd, queue_size=50)
    rospy.sleep(args.window)
    sub.unregister()
    if not speeds:
        record('cmd_vel_zero', False, '未收到 /cmd_vel')
    else:
        rate = len(speeds) / args.window
        peak = max(speeds)
        record('cmd_vel_zero', rate >= 10 and peak < 1e-3,
               '%.1f Hz peak=%.4f n=%d' % (rate, peak, len(speeds)))

    topics = set(states[1])
    record('path_status_topic', '/global_path/status' in topics, 'present' if '/global_path/status' in topics else 'missing')

    failed = [name for name, ok, _ in checks if not ok]
    print('PREFLIGHT %s (%d/%d)' % ('PASS' if not failed else 'FAIL',
                                    len(checks) - len(failed), len(checks)))
    return 0 if not failed else 1


if __name__ == '__main__':
    sys.exit(main())
