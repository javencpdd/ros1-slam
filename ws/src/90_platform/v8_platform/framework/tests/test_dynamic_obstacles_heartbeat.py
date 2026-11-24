"""perception 心跳不变量：**没真看过点云，就不许发"无障碍"**。

背景（2026-11-25 实测）：`/v8/local/status` 长期 `obstacles_cb=0` 且 `/cmd_vel` 恒零。
根因是 blind 守卫在"从未收到点云"时从第一拍就成立，唯一的障碍发布点被 `continue` 跳过；
反过来更危险：`no_cloud / no_pose / tf_missing / error` 这些分支**原本会发空表**——
等于把"感知没起来"伪装成"这条路是通的"，让 local 放心往前走。

真人裁定（同日）：六态一律不得刷新有效障碍心跳，诊断走独立话题；只有
"真实、有效、新鲜点云 + 处理成功 + 判定无障碍"才允许空列表。

三连结构：
  主测   —— 四种"没真看过"的情形都不许发（blind×2、stale_frame、非 ok 状态）；
  反控   —— ①真的 ok 时必须允许发（否则本闸把正常心跳也掐了）；
            ②同一帧重复到达不算新数据（否则上游卡住重发旧帧就能维持"健康"）；
  正向   —— ok 且 objects=0 时允许发空表（这才是合法的"确认无障碍"）。
"""
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

# 节点模块顶部 import 了 rospy / tf2_ros / 三类消息；这里用桩顶掉，只测纯函数。
for _name in ('rospy', 'tf2_ros', 'geometry_msgs', 'geometry_msgs.msg',
              'std_msgs', 'std_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg'):
    if _name not in sys.modules:
        sys.modules[_name] = MagicMock()

HERE = Path(__file__).resolve().parent.parent          # .../v8_platform/framework
sys.path.insert(0, str(HERE))
import nodes.dynamic_obstacles_node as node  # noqa: E402

import unittest  # noqa: E402


def tearDownModule():
    for _name in ('rospy', 'tf2_ros', 'geometry_msgs', 'geometry_msgs.msg',
                  'std_msgs', 'std_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg'):
        if isinstance(sys.modules.get(_name), MagicMock):
            del sys.modules[_name]


def _gate_obj(last_cloud_wall, cloud_stamp, processed_stamp, timeout=1.0):
    """绕过 __init__（会读 ROS 参数），只装配 _gate 需要的四个属性。"""
    o = object.__new__(node.DynamicObstaclesNode)
    o.blind_timeout_s = timeout
    o.last_cloud_wall = last_cloud_wall
    o.cloud_stamp = cloud_stamp
    o.processed_stamp = processed_stamp
    return o


class GateTest(unittest.TestCase):
    # ---------- 主测：没真看过就不许发 ----------
    def test_never_received_cloud_is_blind(self):
        """①启动期从未收云：不许发（原实现从第一拍就 blind，正是车不走的直接原因）。"""
        publish, state = _gate_obj(None, None, None)._gate(now=100.0)
        self.assertFalse(publish)
        self.assertEqual(state['state'], 'blind')

    def test_cloud_timeout_is_blind(self):
        """②曾收过云但断流超时：不许发。"""
        publish, state = _gate_obj(last_cloud_wall=90.0, cloud_stamp=90.0,
                                   processed_stamp=90.0)._gate(now=100.0)
        self.assertFalse(publish)
        self.assertEqual(state['state'], 'blind')

    def test_duplicate_frame_not_new_data(self):
        """⑤同一 stamp 重复到达：不算新数据，不许借此维持"健康"。"""
        publish, state = _gate_obj(last_cloud_wall=99.5, cloud_stamp=99.0,
                                   processed_stamp=99.0)._gate(now=100.0)
        self.assertFalse(publish)
        self.assertEqual(state['state'], 'stale_frame')

    # ---------- 反控：真 ok 必须放行 ----------
    def test_fresh_frame_passes_gate(self):
        publish, state = _gate_obj(last_cloud_wall=99.9, cloud_stamp=99.8,
                                   processed_stamp=99.0)._gate(now=100.0)
        self.assertTrue(publish, '新鲜帧被闸掉了 —— 正常心跳会被掐死')
        self.assertIsNone(state)


class ShouldPublishTest(unittest.TestCase):
    # ---------- 主测：六态一律不发 ----------
    def test_six_states_never_publish(self):
        for state in ('no_cloud', 'no_pose', 'unsupported_cloud', 'tf_missing',
                      'error', 'blind', 'disabled', 'stale_frame'):
            self.assertFalse(node.should_publish({'state': state}),
                             '%s 竟然允许发有效障碍表' % state)

    # ---------- 反控：ok 必须发 ----------
    def test_ok_publishes_even_when_empty(self):
        self.assertTrue(node.should_publish({'state': 'ok', 'objects': 0}),
                        'ok 且无障碍时被拦 —— 合法的空表发不出去')

    def test_ok_with_objects_publishes(self):
        self.assertTrue(node.should_publish({'state': 'ok', 'objects': 3}))

    def test_none_and_garbage_are_rejected(self):
        self.assertFalse(node.should_publish(None))
        self.assertFalse(node.should_publish('ok'))     # 形状不对也不许发


if __name__ == '__main__':
    unittest.main(verbosity=2)
