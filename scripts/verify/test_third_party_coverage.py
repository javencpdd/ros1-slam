#!/usr/bin/env python3
"""third_party_coverage 的三连自测：正向 + 两条反控 + 对真树的实时校验。

为什么要有反控：这类"集合对表"判据最容易退化成恒真——只要实现里少写一半比较，
正向用例照样绿。所以必须有把输入改成**应当失败**的用例，且要能证明判据真会翻。
"""
import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import third_party_coverage as cov  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TREE = os.path.join(REPO, 'ws/src/30_3rd')
BASE = os.path.join(REPO, 'note/进度/v7/S1/SCAN集成/证据/scan_src_baseline_49.sha256')
REG = os.path.join(REPO, 'scripts/verify/third_party_unbaselined.tsv')


class _Tmp(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix='tpc_')
        self.tree = os.path.join(self.dir, 'tree')
        self.base = os.path.join(self.dir, 'base.sha256')
        self.reg = os.path.join(self.dir, 'reg.tsv')
        for d in ('alpha', 'beta', 'gamma'):
            os.makedirs(os.path.join(self.tree, d))
            open(os.path.join(self.tree, d, 'f.txt'), 'w').write(d)
        # 基线只覆盖 alpha/beta
        with open(self.base, 'w') as fh:
            for d in ('alpha', 'beta'):
                fh.write('%s  %s/f.txt\n' % ('0' * 64, d))
        # 登记 gamma
        with open(self.reg, 'w') as fh:
            fh.write('目录\t入库提交\t日期\t定性\n')
            fh.write('gamma\tdeadbee\t2026-11-24\t测试夹具\n')

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)


class CoverageTest(_Tmp):

    def test_positive_registered_matches(self):
        ok, d = cov.audit(self.tree, self.base, self.reg)
        self.assertTrue(ok, d)
        self.assertEqual(['gamma'], d['未纳入'])
        self.assertEqual([], d['多登记'])

    def test_control_new_package_unregistered_must_fail(self):
        """反控一：新增一个未登记的顶层包 ⇒ 必须判不一致。"""
        os.makedirs(os.path.join(self.tree, 'delta'))
        open(os.path.join(self.tree, 'delta', 'f.txt'), 'w').write('delta')
        ok, d = cov.audit(self.tree, self.base, self.reg)
        self.assertFalse(ok, '新增未登记包却没翻红')
        self.assertIn('delta', d['未纳入'])

    def test_control_extra_registration_must_fail(self):
        """反控二：登记件里写了树里不存在的包（幽灵登记）⇒ 必须判不一致。"""
        with open(self.reg, 'a') as fh:
            fh.write('phantom\tdeadbee\t2026-11-24\t幽灵\n')
        ok, d = cov.audit(self.tree, self.base, self.reg)
        self.assertFalse(ok, '多登记却没翻红')
        self.assertIn('phantom', d['多登记'])

    def test_missing_inputs_raise(self):
        with self.assertRaises(FileNotFoundError):
            cov.audit(os.path.join(self.dir, 'nope'), self.base, self.reg)
        with self.assertRaises(FileNotFoundError):
            cov.audit(self.tree, os.path.join(self.dir, 'nope.sha256'), self.reg)


class RealTreeTest(unittest.TestCase):
    """对真树跑一次：这是 N6 的实际判据面，不能只在夹具上绿。"""

    def test_repo_tree_matches_registry(self):
        ok, d = cov.audit(TREE, BASE, REG)
        self.assertTrue(ok, '未入基线包与登记件不一致：%s' % d)
        self.assertEqual([], d['多登记'], '登记件里有树中不存在的包')


if __name__ == '__main__':
    unittest.main(verbosity=2)
