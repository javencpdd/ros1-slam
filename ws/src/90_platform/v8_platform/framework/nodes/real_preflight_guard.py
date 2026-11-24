#!/usr/bin/env python3
"""真机启动前置闸：外参未标定、或 quality_min 仍是占位 ⇒ **非零退出**。

★拦住的机制不是打印警告★：本节点在 launch 里带 `required="true"`，roslaunch 见到
required 节点退出就把整个 launch 带走。所以"未标定必须阻断"成立的条件是两件事同时成立：
  ① 本进程真的返回非零；
  ② launch 里确实写了 required="true"（有静态用例钉住这一条，见
     framework/tests/test_real_preflight.py）。
另外它**常驻**而不是一次性退出：既能让 required 语义可用（正常退出也会带走整窗），
又能定期复核——有人热改参数把阈值改回占位，同样会被带走。

仿真域不走本节点（模式入口里不 include 它），阈值为 0 在离线开发里是合法的。
"""
import os
import sys
import time

import rospy
import tf2_ros

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))   # .../v8_platform，common 在其下
from common.real_gate import check_extrinsic, load_attestation  # noqa: E402


def _resolve(path, repo_root):
    if not path:
        return ''
    return path if os.path.isabs(path) else os.path.join(repo_root, path)


def evaluate(get_param):
    """把参数折成一次判定；纯读参数，不做 IO 之外的事，便于单测直接调。"""
    repo_root = get_param('~repo_root', '') or os.environ.get('ROS1_SLAM_ROOT', os.getcwd())
    att_path = _resolve(get_param('~extrinsic_attestation',
                                  'data/calib/rslidar_extrinsic.json'), repo_root)
    allow = bool(get_param('~allow_uncalibrated', False))
    qmin = get_param('~quality_min', 0.0)
    raw = [get_param('~tf_x', None), get_param('~tf_y', None), get_param('~tf_z', None)]
    xyz = None
    if all(v is not None for v in raw):
        try:
            xyz = [float(v) for v in raw]
        except (TypeError, ValueError):
            xyz = None
    require_calibrated = bool(get_param('~require_calibrated', True))
    verdict = check_extrinsic(load_attestation(att_path), xyz, qmin, allow,
                              require_calibrated=require_calibrated)
    verdict['attestation_path'] = att_path
    verdict['quality_min'] = qmin
    verdict['require_calibrated'] = require_calibrated
    return verdict


def check_tf_identity(wait_s, parent='base_link', child='rslidar'):
    """运行时读真实 TF 的平移：全零即"雷达装在 base_link 原点"=未标定。

    只看配置里的 args 不够——驱动侧那条 TF 是写死在第三方 launch 里的恒等值，
    配置文件里根本没有这个字段。所以这里必须读 TF 树本身。读不到按未标定处理。
    """
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer)
    deadline = time.time() + wait_s
    while not rospy.is_shutdown() and time.time() < deadline:
        try:
            tfm = buffer.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(0.1))
        except Exception:                      # noqa: BLE001 —— 任何查询失败都按未知处理
            time.sleep(0.2)
            continue
        t = tfm.transform.translation
        if all(abs(v) < 1e-9 for v in (t.x, t.y, t.z)):
            return False, [t.x, t.y, t.z]
        return True, [t.x, t.y, t.z]
    return None, None                          # 查询超时 = 未知 = 不通过


def main():
    rospy.init_node('v8_real_preflight_guard')
    recheck_s = float(rospy.get_param('~recheck_s', 5.0))
    check_tf = bool(rospy.get_param('~check_tf', True))
    tf_wait_s = float(rospy.get_param('~tf_wait_s', 20.0))
    while not rospy.is_shutdown():
        verdict = evaluate(lambda key, default=None: rospy.get_param(key, default))
        # 不运动的入口不必等 TF（等 20 s 只会白占一个周期），它也不会动
        if check_tf and not verdict.get('not_required'):
            non_identity, xyz = check_tf_identity(tf_wait_s)
            if non_identity is not True:
                verdict['ok'] = False
                verdict.setdefault('reasons', []).append(
                    'E_EXTRINSIC_IDENTITY_TF' if non_identity is False
                    else 'E_EXTRINSIC_TF_UNKNOWN')
            else:
                verdict['tf_xyz'] = xyz
        if verdict.get('not_required'):
            rospy.logwarn_throttle(30.0,
                                   '[real_preflight] 本入口不运动，只告警不阻断：%s',
                                   '，'.join(verdict['reasons']) or '—')
            rospy.sleep(max(1.0, recheck_s))
            continue
        if not verdict['ok']:
            rospy.logfatal('[real_preflight] 拒绝启动真机导航：%s｜标定记录=%s｜quality_min=%s',
                           '，'.join(verdict['reasons']), verdict['attestation_path'],
                           verdict['quality_min'])
            sys.exit(1)
        if verdict['degraded']:
            rospy.logwarn_throttle(30.0, '[real_preflight] 开发口放行（未标定/未对表）：%s',
                                   verdict['reasons'])
        else:
            rospy.loginfo_throttle(60.0, '[real_preflight] 准入通过（外参已标定、阈值已确值）')
        rospy.sleep(max(1.0, recheck_s))


if __name__ == '__main__':
    main()
