#!/usr/bin/env python3
""""未标定必须阻断"的**进程级**证据，而不是"日志里出现了 WARN"。

被测对象是真节点 `framework/nodes/real_preflight_guard.py`。测试用一套最小 rospy/tf2_ros
桩把它跑成真实子进程，然后断言**退出码**：
  - 未标定或阈值占位 ⇒ 必须非零退出（配合 launch 里的 required="true" 就会带走整窗）；
  - 达标 ⇒ 必须**不退出**（常驻才复用得了 required 语义，否则正常退出也会带走整窗）。
另有三条静态断言，钉住 launch 侧的两个前提：required 真的写了、仿真入口真的没接它。
"""
import os
import re
import subprocess
import sys
import tempfile
import textwrap
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
NODE = os.path.join(REPO, 'ws/src/90_platform/v8_platform/framework/nodes/real_preflight_guard.py')
LAUNCH = os.path.join(REPO, 'ws/src/70_bringup/lite3_bringup/launch')
NAV = os.path.join(LAUNCH, 'mode_navigation.launch')
LOCALIZE = os.path.join(LAUNCH, 'mode_localize.launch')
SIM = os.path.join(LAUNCH, 'mode_sim_hdl.launch')

STUB_ROSPY = '''
PARAMS = %(params)r
LOG = []
def init_node(*a, **k):
    pass
def get_param(key, default=None):
    return PARAMS.get(key, default)
def logfatal(fmt, *a):
    LOG.append(('fatal', fmt %% a if a else fmt))
def logwarn(fmt, *a):
    pass
def logwarn_throttle(*a, **k):
    pass
def loginfo(*a, **k):
    pass
def loginfo_throttle(*a, **k):
    pass
def is_shutdown():
    return False
def sleep(seconds):
    import time
    time.sleep(min(float(seconds), 0.05))
class Duration(object):
    def __init__(self, value):
        self.value = value
class Time(object):
    def __init__(self, value=0.0):
        self.value = value
'''

STUB_TF = '''
class LookupGone(Exception):
    pass
class Buffer(object):
    def __init__(self, *a, **k):
        pass
    def lookup_transform(self, *a, **k):
        raise LookupGone('stub: no tf')
class TransformListener(object):
    def __init__(self, *a, **k):
        pass
'''


class BlockingTest(unittest.TestCase):

    def _run(self, params, timeout=5):
        tmp = tempfile.mkdtemp(prefix='guard_')
        with open(os.path.join(tmp, 'rospy.py'), 'w', encoding='utf-8') as fh:
            fh.write(STUB_ROSPY % {'params': params})
        with open(os.path.join(tmp, 'tf2_ros.py'), 'w', encoding='utf-8') as fh:
            fh.write(STUB_TF)
        env = dict(os.environ)
        env['PYTHONPATH'] = tmp + os.pathsep + env.get('PYTHONPATH', '')
        proc = subprocess.Popen([sys.executable, NODE], env=env,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            out, err = proc.communicate(timeout=timeout)
            return proc.returncode, out.decode('utf-8', 'ignore') + err.decode('utf-8', 'ignore')
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.communicate()
            return 'timeout', ''

    def test_uncalibrated_blocks_with_nonzero_exit(self):
        """未标定 + 会动的入口 ⇒ 必须非零退出（这是"阻断启动"的唯一硬证据）。"""
        rc, _ = self._run({'~require_calibrated': True, '~check_tf': False,
                           '~quality_min': 0.0, '~extrinsic_attestation': 'nope.json',
                           '~repo_root': REPO})
        self.assertNotEqual('timeout', rc, '未标定时节点没退出，也就阻止不了启动')
        self.assertNotEqual(0, rc, '未标定时退出码是 0 ⇒ required 语义不会带走整窗')

    def test_warn_only_entry_does_not_exit(self):
        """反控：不运动的入口（require_calibrated=false）必须**不退出**，否则采不到标定数据。"""
        rc, _ = self._run({'~require_calibrated': False, '~check_tf': False,
                           '~quality_min': 0.0, '~extrinsic_attestation': 'nope.json',
                           '~repo_root': REPO})
        self.assertEqual('timeout', rc, '只告警的入口退出了，建图/定位干跑会被打死')

    def test_calibrated_motion_entry_stays_alive(self):
        """达标 ⇒ 常驻不退出（required 语义要求它活着）。"""
        tmp = tempfile.mkdtemp(prefix='att_')
        att = os.path.join(tmp, 'att.json')
        with open(att, 'w', encoding='utf-8') as fh:
            fh.write('{"xyz":[0.1,0,0.3],"measured_at":"2026-11-24","measured_by":"G8"}')
        rc, _ = self._run({'~require_calibrated': True, '~check_tf': False,
                           '~quality_min': 0.35, '~extrinsic_attestation': att,
                           '~repo_root': REPO})
        self.assertEqual('timeout', rc, '达标后节点退出了：required 会误杀整窗')


class LaunchStaticTest(unittest.TestCase):
    """launch 侧的两个前提也必须钉住，否则节点判得再对也拦不住。"""

    def test_navigation_guard_is_required_and_bound_to_motion(self):
        body = open(NAV, encoding='utf-8').read()
        self.assertIn('real_preflight_guard.py', body)
        guard = body[body.index('real_preflight_guard.py'):]
        guard = guard[:guard.index('</node>')]
        self.assertRegex(guard, r'required="true"',
                         '前置闸没写 required="true" ⇒ 非零退出不会带走整窗')
        self.assertRegex(guard, r'name="require_calibrated"\s+value="\$\(arg enable_motion\)"',
                         'require_calibrated 没绑 enable_motion ⇒ 建图/定位干跑会被阻断')

    def test_motion_gate_cancels_into_bt(self):
        body = open(NAV, encoding='utf-8').read()
        self.assertIn('loc_motion_gate_node.py', body)
        gate = body[body.index('loc_motion_gate_node.py'):]
        gate = gate[:gate.index('</node>')]
        self.assertIn('/v8/bt/cancel', gate, '运动门控没有指向 BT 取消入口')

    def test_localize_warns_only(self):
        body = open(LOCALIZE, encoding='utf-8').read()
        guard = body[body.index('real_preflight_guard.py'):]
        guard = guard[:guard.index('</node>')]
        self.assertRegex(guard, r'name="require_calibrated"\s+value="false"')
        self.assertNotRegex(guard, r'required="true"')

    def test_simulation_entry_is_independent(self):
        """仿真域必须完全不含这两个真机闸（域独立）。"""
        body = open(SIM, encoding='utf-8').read()
        self.assertNotIn('real_preflight_guard', body)
        self.assertNotIn('loc_motion_gate', body)


if __name__ == '__main__':
    unittest.main(verbosity=2)
