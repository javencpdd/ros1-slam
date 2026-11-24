#!/usr/bin/env python3
"""v_tf_extrinsic_gate 的三连自测：正向 + 四条反控 + 一个条件节点用例。

反控的意义：这条判据的三个子条件（唯一／非恒等／与标定记录一致）任何一条写漏，
正向用例都还会绿。所以每条子条件都要有"改成应当失败"的用例，并断言它真的翻。
"""
import json
import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import v_tf_extrinsic_gate as gate  # noqa: E402

EDGE = ('base_link', 'rslidar')


def _node(parent='base_link', child='rslidar', xyz=(0.1, 0.0, 0.3), name='base_link_to_rslidar',
          cond=None):
    args = '%s %s %s 0.0 0.13 0.0 /%s /%s' % (xyz[0], xyz[1], xyz[2], parent, child)
    attr = ''
    if cond is not None:
        attr = ' if="$(arg %s)"' % cond
    return ('  <node pkg="tf2_ros" type="static_transform_publisher" name="%s"%s '
            'args="%s" />' % (name, attr, args))


class GateTest(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix='tfg_')
        self.pkg = os.path.join(self.dir, 'fakepkg')
        os.makedirs(os.path.join(self.pkg, 'launch'))
        self.roots = {'fakepkg': self.pkg}
        self.att = os.path.join(self.dir, 'att.json')

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def _launch(self, name, body, args=''):
        path = os.path.join(self.pkg, 'launch', name)
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write('<launch>\n%s\n%s\n</launch>\n' % (args, body))
        return path

    # ── 正向 ──────────────────────────────────────────────────────────
    def test_single_measured_publisher_passes(self):
        e = self._launch('ok.launch', _node())
        ok, d = gate.audit(e, self.roots, EDGE)
        self.assertTrue(ok, d)
        self.assertEqual(1, len(d['publishers']))
        self.assertFalse(d['publishers'][0]['identity'])

    def test_attestation_match_passes(self):
        e = self._launch('ok.launch', _node(xyz=(0.1, 0.0, 0.3)))
        with open(self.att, 'w') as fh:
            json.dump({'xyz': [0.1, 0.0, 0.3], 'measured_at': '2026-11-24',
                       'measured_by': 'G8', 'method': 'tape+level'}, fh)
        ok, d = gate.audit(e, self.roots, EDGE, gate.load_attestation(self.att))
        self.assertTrue(ok, d)
        self.assertIn('measured_at', d['attested'])

    # ── 反控 ──────────────────────────────────────────────────────────
    def test_control_identity_placeholder_fails(self):
        """反控一：唯一但恒等（未标定）⇒ 必须 FAIL。这是真机现状。"""
        e = self._launch('ident.launch', _node(xyz=(0.0, 0.0, 0.0)))
        ok, d = gate.audit(e, self.roots, EDGE)
        self.assertFalse(ok, '恒等占位没被判红')
        self.assertTrue(d['publishers'][0]['identity'])

    def test_control_two_publishers_fails(self):
        """反控二：同一条边两个发布者 ⇒ 必须 FAIL（重复 TF）。"""
        body = _node() + '\n' + _node(name='another_tf', xyz=(0.2, 0.0, 0.3))
        e = self._launch('dup.launch', body)
        ok, d = gate.audit(e, self.roots, EDGE)
        self.assertFalse(ok, '双发布者没被判红')
        self.assertEqual(2, len(d['publishers']))

    def test_control_unparsed_args_fail_closed(self):
        """反控三：args 含未展开变量 ⇒ 不可判定，必须 FAIL 而不是放过。"""
        e = self._launch('var.launch',
                         '  <node pkg="tf2_ros" type="static_transform_publisher" name="t" '
                         'args="$(arg tf_x) 0 0 0 0 0 /base_link /rslidar" />')
        ok, d = gate.audit(e, self.roots, EDGE)
        self.assertFalse(ok, '不可判定被当成通过')
        self.assertTrue(d['unknown'])

    def test_control_attestation_mismatch_fails(self):
        """反控四：标定记录与现值不一致 ⇒ 必须 FAIL（标过但配错）。"""
        e = self._launch('ok.launch', _node(xyz=(0.1, 0.0, 0.3)))
        with open(self.att, 'w') as fh:
            json.dump({'xyz': [0.1, 0.0, 0.9]}, fh)
        ok, d = gate.audit(e, self.roots, EDGE, gate.load_attestation(self.att))
        self.assertFalse(ok, '记录与现值不一致却没翻红')

    def test_conditional_node_respects_include_args(self):
        """被 include 关掉的条件节点不该算发布者；打开时要算。

        这条对应真机现状：两个入口给 hdl launch 传 publish_extrinsic:=false，
        若把它的声明也算进来，正确的入口会被判成"双发布者"。
        """
        inner = self._launch('inner.launch', _node(cond='publish_extrinsic'),
                             args='  <arg name="publish_extrinsic" default="true" />')
        outer_off = self._launch(
            'outer_off.launch',
            '  <include file="$(find fakepkg)/launch/inner.launch">'
            '<arg name="publish_extrinsic" value="false"/></include>')
        outer_on = self._launch(
            'outer_on.launch',
            '  <include file="$(find fakepkg)/launch/inner.launch">'
            '<arg name="publish_extrinsic" value="true"/></include>')
        self.assertEqual([], gate.walk(outer_off, self.roots, EDGE),
                         '关掉的条件节点仍被计入')
        self.assertEqual(1, len(gate.walk(outer_on, self.roots, EDGE)),
                         '打开的条件节点没被计入')
        self.assertTrue(inner)

    def test_other_edge_ignored(self):
        """不相干的边不该被算进来（判据不能只看 type）。"""
        e = self._launch('other.launch', _node(parent='base_link', child='laser'))
        self.assertEqual([], gate.walk(e, self.roots, EDGE))


if __name__ == '__main__':
    unittest.main(verbosity=2)
