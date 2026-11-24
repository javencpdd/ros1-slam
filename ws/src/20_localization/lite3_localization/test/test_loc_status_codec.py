#!/usr/bin/env python3
"""loc_status_codec 的自测：正向 + 四条反控。

反控要证明的是"读不到/读坏了不会被当成好"——这类解码最典型的退化就是
`getattr(msg, 'x', 0.0)`：字段缺失时悄悄给 0.0，而 0.0 在质量判据里可能恰好过关。
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/scripts')

import loc_status_codec as codec  # noqa: E402


class _Msg(object):
    def __init__(self, converged=True, frac=0.8, err=0.02, drop=()):
        self.has_converged = converged
        self.inlier_fraction = frac
        self.matching_error = err
        for name in drop:
            delattr(self, name)


class CodecTest(unittest.TestCase):
    def test_good_message(self):
        out = codec.decode(_Msg())
        self.assertTrue(out['converged'])
        self.assertAlmostEqual(0.8, out['quality'])
        self.assertEqual('inlier_fraction', out['quality_name'])

    def test_not_converged_is_carried_through(self):
        self.assertFalse(codec.decode(_Msg(converged=False))['converged'])

    def test_missing_converged_raises(self):
        with self.assertRaises(codec.StatusDecodeError):
            codec.decode(_Msg(drop=('has_converged',)))

    def test_missing_fraction_raises(self):
        with self.assertRaises(codec.StatusDecodeError):
            codec.decode(_Msg(drop=('inlier_fraction',)))

    def test_fraction_out_of_range_raises(self):
        for bad in (-0.1, 1.5):
            with self.assertRaises(codec.StatusDecodeError):
                codec.decode(_Msg(frac=bad))

    def test_nan_fraction_raises(self):
        with self.assertRaises(codec.StatusDecodeError):
            codec.decode(_Msg(frac=float('nan')))

    def test_matching_error_optional(self):
        out = codec.decode(_Msg(drop=('matching_error',)))
        self.assertNotIn('matching_error', out)
        self.assertIn('matching_error', codec.decode(_Msg()))


if __name__ == '__main__':
    unittest.main(verbosity=2)
