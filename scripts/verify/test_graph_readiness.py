#!/usr/bin/env python3
"""直接执行 graph_readiness 的真实函数（不是看字符串）。

回归点（2026-11-27）：`owns = any(any(...))` 把 bool 当可迭代，直接 TypeError。
只读脚本外壳永远发现不了，必须**调用**里面的函数。
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import graph_readiness as gr  # noqa: E402

INFO_MULTI = """Type: std_msgs/String

Publishers:
 * /v8/3d/loc/loc (http://a:1234/)
 * /sim_pose_source (http://a:5678/)

Subscribers: None
"""

INFO_ONE = """Type: std_msgs/String

Publishers:
 * /v8/3d/loc/loc (http://a:1234/)

Subscribers: None
"""

INFO_NONE = """Type: std_msgs/String

Publishers:
None

Subscribers: None
"""


class ParseTest(unittest.TestCase):
    def test_parse_returns_full_node_names(self):
        self.assertEqual(gr.parse_publishers(INFO_ONE), ['/v8/3d/loc/loc'])

    def test_parse_two_publishers(self):
        self.assertEqual(gr.parse_publishers(INFO_MULTI),
                         ['/v8/3d/loc/loc', '/sim_pose_source'])

    def test_parse_none(self):
        self.assertEqual(gr.parse_publishers(INFO_NONE), [])


class PublisherVerdictTest(unittest.TestCase):
    def test_single_allowed_is_authoritative(self):
        self.assertTrue(gr.publisher_verdict(['/v8/3d/loc/loc'], ['/v8/3d/loc/loc'])[0])

    def test_second_publisher_makes_it_non_authoritative(self):
        # 这就是 TypeError 那行的真实语义：出现第二个发布者即判不权威
        ok, why = gr.publisher_verdict(['/v8/3d/loc/loc', '/sim_pose_source'],
                                       ['/v8/3d/loc/loc'])
        self.assertFalse(ok)
        self.assertIn('不唯一', why)

    def test_wrong_publisher(self):
        ok, why = gr.publisher_verdict(['/sim_pose_source'], ['/v8/3d/loc/loc'])
        self.assertFalse(ok)
        self.assertIn('白名单', why)

    def test_no_publisher(self):
        self.assertFalse(gr.publisher_verdict([], ['/x'])[0])


class LatchedTest(unittest.TestCase):
    """地图是 latch 单发：不能要求窗内多条，只能查有效负载 + 唯一发布者。"""

    def test_latched_ok(self):
        ok, _ = gr.latched_verdict(True, True, ['/v8/3d/map/map'], ['/v8/3d/map/map'])
        self.assertTrue(ok)

    def test_latched_missing_payload(self):
        ok, why = gr.latched_verdict(False, False, ['/v8/3d/map/map'], ['/v8/3d/map/map'])
        self.assertFalse(ok)
        self.assertIn('latch', why)

    def test_latched_bad_fields(self):
        ok, why = gr.latched_verdict(True, False, ['/v8/3d/map/map'], ['/v8/3d/map/map'])
        self.assertFalse(ok)
        self.assertIn('字段', why)

    def test_latched_secondary_publisher_rejected(self):
        ok, why = gr.latched_verdict(True, True, ['/v8/3d/map/map', '/delta_map'],
                                    ['/v8/3d/map/map'])
        self.assertFalse(ok)

    def test_interface_kinds_declared(self):
        # 地图必须是 latched，POSE 必须是 stream——类型写错会重新退回"数条数"的坑
        self.assertEqual(gr.INTERFACES['map'][0][1], 'latched')
        self.assertEqual(gr.INTERFACES['loc'][0][1], 'stream')


class IncludeTest(unittest.TestCase):
    def test_far_include_nodes_are_explicit_names(self):
        # FAR 的真实节点与实例 namespace 无关，必须按名字登记
        self.assertIn('/far_planner', gr.INCLUDE_NODES['far_stack.launch'])
        self.assertIn('/v8_global_far', gr.INCLUDE_NODES['far_stack.launch'])


if __name__ == '__main__':
    unittest.main()
