#!/usr/bin/env python3
"""端到端（进程内）负控：歧义/无匹配/错高度必须在 **plan 与 track 的出口**上拒绝运动。

2026-11-27：之前只断言 associate() 的字段，等于只测诊断不测控制。
这里直接调 plan()/track_segment()，看它们是否真的拒绝、并且速度出口为零。
"""
import json
import os
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
import sys
sys.path.insert(0, os.path.dirname(HERE))

import local_native as ln  # noqa: E402


def _real_map():
    d = HERE
    for _ in range(8):
        p = os.path.join(d, 'data', 'maps', 'generated', 'v8_m1_hdl_v1.json')
        if os.path.isfile(p):
            with open(p, encoding='utf-8') as fh:
                return json.load(fh)
        d = os.path.dirname(d)
    raise unittest.SkipTest('缺少 data/maps/generated/v8_m1_hdl_v1.json')


def _two_floor(m):
    m = json.loads(json.dumps(m))
    base = next(s for s in m['surfaces'] if s.get('traversability') == 'supported')
    up = json.loads(json.dumps(base))
    up['surface_id'] = base['surface_id'] + '_upper'
    up['z_reference'] = base['z_reference'] + 0.05     # 只有 5 cm 的层差
    m['surfaces'].append(up)
    # 单层图不允许出现第二个面（E_MAP_ML_COLLAPSE）；要造多层必须显式声明
    m['multiple_surfaces_per_xy'] = True
    return m


class OutletTest(unittest.TestCase):
    def test_ambiguous_map_is_refused_somewhere_upstream(self):
        """两层相距 5cm 的图会被**地图契约**先拒（E_MAP_ML_COLLAPSE / E_MAP_DISCONNECTED），
        根本到不了 local 的歧义分支。这说明"歧义"在当前契约下不是随手能造的形态，
        真正的多层歧义负控需要一张**合法**的上下层连通图（带坡道/楼梯），单独立项。

        这里不断言具体错误码，只断言"拒绝发生且不产出轨迹"，避免把上游拒绝写成假通过。
        """
        m = _two_floor(_real_map())
        cfg = ln.Config(support_assoc_tol_m=0.08)
        with self.assertRaises(ln.LocalError) as ctx:
            ln.plan(m, (5.2, 2.45, 0.0), (6.0, 2.45, 0.0),
                    obstacles=(), now=100.0, config=cfg)
        self.assertTrue(str(ctx.exception.code).startswith(('E_MAP_', 'E_LOCAL_')),
                        '必须是明确拒绝，不能静默返回轨迹：%s' % ctx.exception.code)

    def test_single_floor_still_plans(self):
        cfg = ln.Config(support_assoc_tol_m=0.08)
        out = ln.plan(_real_map(), (5.2, 2.45, 0.0), (6.0, 2.45, 0.0),
                      obstacles=(), now=100.0, config=cfg)
        self.assertTrue(out['ok'])
        self.assertTrue(out['support_association']['start']['valid'])

    def test_track_zero_velocity_when_pose_leaves_support(self):
        cfg = ln.Config(support_assoc_tol_m=1e-6)
        m = _real_map()
        out = ln.plan(m, (5.2, 2.45, 0.0), (6.0, 2.45, 0.0),
                      obstacles=(), now=100.0, config=cfg)
        # 反馈位姿被抬高 0.5 m：既不在任何支撑面上，也不在容差内
        step = ln.track_segment(m, out, (5.2, 2.45, 0.5), 100.0, (),
                                config=cfg, feedback_pose=(5.2, 2.45, 0.5))
        self.assertTrue(step['stop'], '失关联必须停车')
        self.assertEqual(list(step['velocity']), [0.0, 0.0, 0.0])

    def test_track_blocks_on_map_version_change(self):
        cfg = ln.Config()
        m = _real_map()
        out = ln.plan(m, (5.2, 2.45, 0.0), (6.0, 2.45, 0.0),
                      obstacles=(), now=100.0, config=cfg)
        m2 = json.loads(json.dumps(m))
        m2['map_version'] = m['map_version'] + 1
        step = ln.track_segment(m2, out, (5.2, 2.45, 0.0), 100.0, (), config=cfg)
        self.assertTrue(step['stop'])
        self.assertEqual(step['reason'], 'E_LOCAL_MAP_STALE')


if __name__ == '__main__':
    unittest.main()
