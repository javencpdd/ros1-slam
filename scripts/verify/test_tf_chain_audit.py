#!/usr/bin/env python3
"""tf_chain_audit 纯逻辑负控：缺边、双父、双写、环路、未来戳、重复源戳、超龄；静态边不按心跳判超龄。

2026-11-27：这些正是旧版漏掉的六类（旧版 stale 不进 problems、静态边被当动态、缺 root 也 PASS 等）。
喂合成边集，不依赖 ROS。
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tf_chain_audit as A  # noqa: E402


def E(parent, child, dynamic=True, stamps=(1.0, 2.0), wall=None, pubs=None):
    return {child: {parent: {'dynamic': dynamic, 'stamps': list(stamps),
                             'last_wall': wall, 'publishers': pubs or []}}}


def merge(*ds):
    out = {}
    for d in ds:
        for c, ps in d.items():
            out.setdefault(c, {}).update(ps)
    return out


CONTRACT = {'root': 'map', 'production_target': 'rslidar',
            'gt_frames': ['odom', 'gt_odom'], 'stale_s': 1.0}


def kinds(res):
    return sorted({p['kind'] for p in res['problems']})


class AuditTest(unittest.TestCase):
    def test_clean_chain_passes(self):
        edges = merge(E('map', 'camera_init', dynamic=False),
                      E('camera_init', 'body'),
                      E('body', 'base_link', dynamic=False),
                      E('base_link', 'rslidar', dynamic=False))
        res = A.audit(edges, CONTRACT, now=3.0)
        self.assertEqual(res['verdict'], 'PASS', res['problems'])
        self.assertEqual(res['chain'][-1], 'map')

    def test_chain_not_reaching_root_fails(self):
        edges = merge(E('camera_init', 'body'), E('body', 'base_link', dynamic=False),
                      E('base_link', 'rslidar', dynamic=False))
        res = A.audit(edges, CONTRACT, now=3.0)
        self.assertIn('no_root', kinds(res))
        self.assertEqual(res['verdict'], 'FAIL')

    def test_double_parent_fails(self):
        edges = merge(E('body', 'base_link', dynamic=False),
                      E('odom', 'base_link', dynamic=False),
                      E('map', 'body', dynamic=False))
        res = A.audit(edges, CONTRACT, now=3.0)
        self.assertIn('double_parent', kinds(res))

    def test_production_through_gt_fails(self):
        edges = merge(E('map', 'odom', dynamic=False), E('odom', 'base_link'),
                      E('base_link', 'rslidar', dynamic=False))
        res = A.audit(edges, CONTRACT, now=3.0)
        self.assertIn('production_uses_gt', kinds(res))

    def test_stale_dynamic_edge_fails(self):
        edges = merge(E('map', 'camera_init', dynamic=False),
                      E('camera_init', 'body', wall=0.0),
                      E('body', 'base_link', dynamic=False),
                      E('base_link', 'rslidar', dynamic=False))
        res = A.audit(edges, CONTRACT, now=10.0)
        self.assertIn('stale', kinds(res), '超龄必须进 problems 并影响 PASS')

    def test_static_edge_never_stale(self):
        edges = merge(E('map', 'camera_init', dynamic=False, wall=0.0),
                      E('camera_init', 'body', wall=9.5),
                      E('body', 'base_link', dynamic=False, wall=0.0),
                      E('base_link', 'rslidar', dynamic=False, wall=0.0))
        res = A.audit(edges, CONTRACT, now=10.0)
        self.assertNotIn('stale', kinds(res), '静态边不按心跳判超龄')

    def test_stalled_stamp_fails(self):
        edges = merge(E('map', 'camera_init', dynamic=False),
                      E('camera_init', 'body', stamps=(5.0, 5.0)),
                      E('body', 'base_link', dynamic=False),
                      E('base_link', 'rslidar', dynamic=False))
        res = A.audit(edges, CONTRACT, now=6.0)
        self.assertIn('stamp_stalled', kinds(res))

    def test_regressing_stamp_fails(self):
        edges = merge(E('map', 'camera_init', dynamic=False),
                      E('camera_init', 'body', stamps=(5.0, 4.0)),
                      E('body', 'base_link', dynamic=False))
        res = A.audit(edges, CONTRACT, now=6.0)
        self.assertIn('stamp_regression', kinds(res))

    def test_future_stamp_fails(self):
        edges = merge(E('map', 'camera_init', dynamic=False),
                      E('camera_init', 'body', stamps=(99.0,)),
                      E('body', 'base_link', dynamic=False))
        res = A.audit(edges, CONTRACT, now=6.0)
        self.assertIn('future_stamp', kinds(res))

    def test_cycle_fails(self):
        edges = merge(E('a', 'b', dynamic=False), E('b', 'a', dynamic=False))
        res = A.audit(edges, dict(CONTRACT, production_target='b'), now=3.0)
        self.assertIn('cycle', kinds(res))

    def test_duplicate_publisher_fails(self):
        edges = merge(E('map', 'camera_init', dynamic=False, pubs=['/a', '/b']))
        res = A.audit(edges, dict(CONTRACT, production_target='camera_init'), now=3.0)
        self.assertIn('duplicate_publisher', kinds(res))

    def test_gt_frames_are_configurable(self):
        # 改了帧名，只要契约跟着改，就不会假 PASS
        edges = merge(E('map', 'gt_odom_x', dynamic=False), E('gt_odom_x', 'base_link'),
                      E('base_link', 'rslidar', dynamic=False))
        res = A.audit(edges, dict(CONTRACT, gt_frames=['gt_odom_x']), now=3.0)
        self.assertIn('production_uses_gt', kinds(res))


if __name__ == '__main__':
    unittest.main()
