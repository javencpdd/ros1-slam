#!/usr/bin/env python3
"""真机迁移门禁（组合件）的三连自测：正向 + 三条逐项反控 + 一个解析器用例。

这三条是**合取**：任一不成立都必须整体不通过。只写正向用例的话，把比较写成 `or`
或者漏掉其中一条，正向照样绿——所以每条都要有把它单独改成"不满足"的用例。
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import v_real_migration_gate as rg  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class AggregateTest(unittest.TestCase):
    def test_all_three_pass(self):
        r = rg.aggregate(True, True, 0.35, 0.35)
        self.assertTrue(r['ok'], r)
        self.assertEqual([], r['failed'])

    def test_control_tf_fails(self):
        r = rg.aggregate(False, True, 0.35, 0.35)
        self.assertFalse(r['ok'])
        self.assertIn('TF', r['failed'])

    def test_control_attestation_fails(self):
        r = rg.aggregate(True, False, 0.35, 0.35)
        self.assertFalse(r['ok'])
        self.assertIn('attestation', r['failed'])

    def test_control_quality_placeholder_fails(self):
        """反控：任一处 quality_min 还是 0.0 占位就必须不通过（两处都要判）。"""
        for graph_q, launch_q in ((0.0, 0.35), (0.35, 0.0), (0.0, 0.0)):
            r = rg.aggregate(True, True, graph_q, launch_q)
            self.assertFalse(r['ok'], '占位没被判红：图=%s launch=%s' % (graph_q, launch_q))
            self.assertIn('quality_min', r['failed'])

    def test_control_none_quality_fails(self):
        r = rg.aggregate(True, True, None, None)
        self.assertFalse(r['ok'])
        self.assertIn('quality_min', r['failed'])


class ParserTest(unittest.TestCase):
    def test_reads_real_graph(self):
        q = rg.graph_quality_min(os.path.join(REPO, rg.GRAPH))
        self.assertIsNotNone(q, '真机图里没读到 quality_min')
        self.assertEqual(0.0, q, '本批把 quality_min 留在 0.0 占位，读数应对得上')

    def test_reads_real_launch(self):
        q = rg.launch_quality_min(os.path.join(REPO, rg.NAV_LAUNCH))
        self.assertIsNotNone(q, '真机 launch 里没读到 quality_min')
        self.assertEqual(0.0, q)

    def test_missing_file_returns_none(self):
        self.assertIsNone(rg.graph_quality_min('/nonexistent.yaml'))
        self.assertIsNone(rg.launch_quality_min('/nonexistent.launch'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
