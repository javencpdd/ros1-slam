"""gait_guard 节点接线单测（stub ROS，不起节点、不连真机）。

覆盖：默认开关、影子通道确实产出并发布、shadow 只报不拦、
assist 否决误切、contract 门值反算真的进 GaitDecision、模板态拒用回退。

本件用最小 ROS stub 加载节点，绝不触碰 /v8/chassis/gait 的真实下游，
也不加载 grid_map_msgs（缺省走 ImportError 降级分支，接线由 on_elevation
直接喂假消息验证）。
"""
import importlib.util
import json
import sys
import time
import types
import unittest
from importlib.machinery import SourceFileLoader
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

NODE_PATH = HERE / 'stair_gait_guard_node.py'

SUPPORT_MAP = {
    'schema_version': 1, 'kind': 'support_map', 'frame': 'map',
    'resolution': 0.1, 'map_id': 'test', 'map_version': 1,
    'surfaces': [
        {'surface_id': 'm0', 'z_reference': 0.0, 'traversability': 'supported',
         'footprint': {'min_x': 0.0, 'min_y': 0.0, 'max_x': 2.0, 'max_y': 4.0},
         'edges': [{'kind': 'step', 'to': 'm1'}]},
        {'surface_id': 'm1', 'z_reference': 0.15, 'traversability': 'supported',
         'footprint': {'min_x': 2.0, 'min_y': 0.0, 'max_x': 4.0, 'max_y': 4.0},
         'edges': [{'kind': 'step', 'to': 'm0'}]},
    ],
}

FILLED = {'status': 'FILLED', 'effective': True,
          'gaits': {'flat_mid': {
              'latency_jitter': {'cmd_lag_p95_s': 0.20, 'switch_settle_s': 0.80},
              'failure': {'stop_distance_m': 0.40},
              'envelope': {'v_max_mps': 2.0, 'step_max_m': 0.12}}}}

STUB_KEYS = ('rospy', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs',
             'nav_msgs.msg', 'std_msgs', 'std_msgs.msg')


def _stub_modules(params=None):
    fake = dict(params or {})

    class Pub(object):
        def __init__(self, topic, *a, **k):
            self.topic, self.sent = topic, []

        def publish(self, msg):
            self.sent.append(msg.data if hasattr(msg, 'data') else msg)

    class String(object):
        def __init__(self, data=None):
            self.data = data

    rospy = types.ModuleType('rospy')
    rospy.get_param = lambda name, default=None: fake.get(name, default)
    rospy.Publisher = Pub
    rospy.Subscriber = lambda *a, **k: None
    rospy.loginfo = rospy.logwarn = lambda *a, **k: None
    rospy.init_node = lambda *a, **k: None
    rospy.is_shutdown = lambda: True
    rospy.Rate = type('Rate', (object,), {'__init__': lambda s, *a, **k: None,
                                          'sleep': lambda s: None})
    out = {'rospy': rospy}
    for pkg, names in {'geometry_msgs': ['PoseStamped'],
                       'nav_msgs': ['Path'],
                       'std_msgs': ['String']}.items():
        m = types.ModuleType(pkg)
        sub = types.ModuleType(pkg + '.msg')
        for name in names:
            setattr(sub, name, String if name == 'String'
                    else type(name, (object,), {}))
        m.msg = sub
        out[pkg], out[pkg + '.msg'] = m, sub
    return out


def load_node(mod_name, params=None):
    saved = {k: sys.modules.get(k) for k in STUB_KEYS}
    for k, v in _stub_modules(params).items():
        sys.modules[k] = v
    try:
        loader = SourceFileLoader(mod_name, str(NODE_PATH))
        spec = importlib.util.spec_from_loader(mod_name, loader)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[mod_name] = mod
        loader.exec_module(mod)
        return mod.GaitGuardNode()
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v


class _Dim(object):
    def __init__(self, size):
        self.size = size


def gridmap_message(matrix, res=0.05, cx=0.0, cy=0.0):
    n_rows, n_cols = len(matrix), len(matrix[0])
    flat = [matrix[r][c] for c in range(n_cols) for r in range(n_rows)]
    layer = types.SimpleNamespace(
        data=flat,
        layout=types.SimpleNamespace(dim=[_Dim(n_cols), _Dim(n_rows)]))
    info = types.SimpleNamespace(
        resolution=res,
        pose=types.SimpleNamespace(position=types.SimpleNamespace(x=cx, y=cy)),
        header=types.SimpleNamespace(frame_id='map'))
    return types.SimpleNamespace(data=[layer], info=info)


WORLD_PATH = [((c - 9.5) * 0.05, 0.0, 0.0) for c in range(20)]
STEP_MATRIX = [[(0.10 if c >= 10 else 0.0) for c in range(20)] for _ in range(9)]
SLOPE_MATRIX = [[0.10 * c / 19.0 for c in range(20)] for _ in range(9)]


def gait_sequence(node):
    seq = []
    node.map_value = SUPPORT_MAP
    for pose, path in (
            ((1.0, 2.0, 0.0), [(1.0, 2.0, 0.0), (1.5, 2.0, 0.0)]),
            ((1.0, 2.0, 0.0), [(1.0, 2.0, 0.0), (3.0, 2.0, 0.15)]),
            ((1.0, 2.0, 0.0), [(1.0, 2.0, 0.0), (1.5, 2.0, 0.0)]),
            ((1.0, 2.0, 0.0), [(1.0, 2.0, 0.0), (3.0, 2.0, 0.15)])):
        node.pose, node.path = pose, path
        node.step()
        seq.append(node.published)
    return seq


BASELINE_SEQ = ['flat_mid', 'high_step', 'flat_mid', 'high_step']


class GaitGuardWiringTests(unittest.TestCase):
    def test_defaults_are_conservative(self):
        node = load_node('wiring_defaults')
        self.assertEqual(node.gate_source, 'declared')
        self.assertEqual(node.perception_mode, 'shadow')
        self.assertEqual(node.gate_resolution['source'], 'declared')
        self.assertEqual(node.elevation_topic, '/elevation_mapping/elevation_map')

    def test_default_gait_sequence_unchanged(self):
        self.assertEqual(gait_sequence(load_node('wiring_seq')), BASELINE_SEQ)

    def test_shadow_channel_publishes_terrain(self):
        node = load_node('wiring_shadow')
        node.pose, node.path = WORLD_PATH[0], WORLD_PATH
        node.on_elevation(gridmap_message(STEP_MATRIX))
        self.assertIsNotNone(node.latest_terrain)
        self.assertEqual(node.latest_terrain['discontinuity_type'], 'vertical_step')
        self.assertEqual(len(node.terrain_pub.sent), 1)
        payload = json.loads(node.terrain_pub.sent[0])
        self.assertTrue(payload['ahead'])
        self.assertAlmostEqual(payload['step_m'], 0.10, places=4)
        self.assertEqual(payload['header_frame'], 'map')

    def test_shadow_never_changes_gait(self):
        node = load_node('wiring_shadow_seq')
        node.pose, node.path = WORLD_PATH[0], WORLD_PATH
        node.on_elevation(gridmap_message(STEP_MATRIX))
        self.assertEqual(gait_sequence(node), BASELINE_SEQ)

    def test_assist_vetoes_slope_misclassification(self):
        node = load_node('wiring_assist', {'~perception_mode': 'assist'})
        node.pose, node.path = WORLD_PATH[0], WORLD_PATH
        node.on_elevation(gridmap_message(SLOPE_MATRIX))
        self.assertEqual(node.latest_terrain['discontinuity_type'],
                         'continuous_slope')
        node.map_value = SUPPORT_MAP
        node.pose = (1.0, 2.0, 0.0)
        node.path = [(1.0, 2.0, 0.0), (3.0, 2.0, 0.15)]
        node.step()
        self.assertEqual(node.published, 'flat_mid')

    def test_contract_gates_reach_decision(self):
        node = load_node('wiring_contract', {
            '~gate_source': 'contract', '~gait_contract': FILLED,
            '~gate_margin_m': 0.20})
        self.assertEqual(node.gate_resolution['source'], 'contract')
        self.assertEqual(node.lookahead_m, 2.6)
        self.assertEqual(node.max_step_m, 0.12)
        self.assertEqual(node.decision.max_step_m, 0.12)

    def test_template_contract_falls_back(self):
        node = load_node('wiring_fallback', {
            '~gate_source': 'contract',
            '~gait_contract': {'status': 'TEMPLATE_ONLY', 'effective': False}})
        self.assertEqual(node.gate_resolution['source'], 'declared')
        self.assertEqual(node.lookahead_m, 1.5)
        self.assertEqual(node.max_step_m, 0.15)
        self.assertEqual(gait_sequence(node), BASELINE_SEQ)

    def test_gait_change_republishes_inside_window_only(self):
        node = load_node('wiring_republish', {'~republish_hz': 2.0,
                                              '~republish_window_s': 5.0})
        node.map_value = SUPPORT_MAP
        node.pose = (1.0, 2.0, 0.0)
        node.path = [(1.0, 2.0, 0.0), (3.0, 2.0, 0.15)]
        with mock.patch.object(time, 'monotonic',
                               side_effect=[100.0, 100.1, 100.6, 106.0]):
            node.step()   # change: publish, window closes at 105.0
            node.step()   # 100.1: interval not reached
            node.step()   # 100.6: republish
            node.step()   # 106.0: window closed
        self.assertEqual(node.gait_pub.sent, ['high_step', 'high_step'])
        self.assertEqual(json.loads(node.status_pub.sent[-1])['republished'],
                         False)

    def test_moving_pose_blocks_republish(self):
        node = load_node('wiring_republish_moving', {'~republish_hz': 2.0,
                                                     '~republish_window_s': 5.0})
        node.map_value = SUPPORT_MAP
        node.path = [(1.0, 2.0, 0.0), (3.0, 2.0, 0.15)]
        node.pose = (1.0, 2.0, 0.0)
        with mock.patch.object(time, 'monotonic',
                               side_effect=[100.0, 100.1, 100.6, 101.1]):
            node.step()                              # change: publish
            node.pose = (1.1, 2.0, 0.0); node.step()  # still moving
            node.pose = (1.2, 2.0, 0.0); node.step()
            node.pose = (1.3, 2.0, 0.0); node.step()
        self.assertEqual(node.gait_pub.sent, ['high_step'])

    def test_status_reports_source_and_mode(self):
        node = load_node('wiring_status')
        node.map_value = SUPPORT_MAP
        node.pose = (1.0, 2.0, 0.0)
        node.path = [(1.0, 2.0, 0.0), (3.0, 2.0, 0.15)]
        node.step()
        payload = json.loads(node.status_pub.sent[-1])
        self.assertEqual(payload['gate_source'], 'declared')
        self.assertEqual(payload['perception_mode'], 'shadow')
        self.assertEqual(payload['gait'], 'high_step')


if __name__ == '__main__':
    unittest.main(verbosity=2)
