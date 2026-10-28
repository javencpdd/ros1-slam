import unittest
import numpy as np
from support_shift import closest_shift_target


class SupportShiftTests(unittest.TestCase):
    def test_projection_keeps_margin_and_reduces_backward_motion(self):
        triangle = [[-.2, -.2], [.2, -.2], [-.2, .2]]
        target = closest_shift_target([0., 0.], triangle, .05, 9.)
        # Oblique boundary: x+y <= -margin*sqrt(2).
        self.assertLessEqual(sum(target), -.05 * np.sqrt(2) + 1e-9)
        self.assertGreaterEqual(min(target), -.15 - 1e-9)
        self.assertGreater(target[0], -.01)

    def test_already_safe_does_not_move(self):
        target = closest_shift_target([-.1, -.1], [[-.2,-.2],[.2,-.2],[-.2,.2]], .02)
        np.testing.assert_allclose(target, [-.1, -.1])

    def test_infeasible_triangle_refuses(self):
        self.assertIsNone(closest_shift_target([0., 0.], [[0.,0.],[.01,0.],[0.,.01]], .05))
        self.assertIsNone(closest_shift_target([0., 0.], [[0.,0.],[0.,0.],[0.,.01]], .05))

    def test_orientation_and_translation_do_not_change_feasibility(self):
        triangle = np.array([[0.,0.],[.4,0.],[0.,.4]]) + [2., -3.]
        forward = closest_shift_target([2.2, -2.8], triangle, .05)
        reverse = closest_shift_target([2.2, -2.8], triangle[::-1], .05)
        np.testing.assert_allclose(forward, reverse)


if __name__ == '__main__':
    unittest.main()
