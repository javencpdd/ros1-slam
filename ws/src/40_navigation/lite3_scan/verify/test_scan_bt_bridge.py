#!/usr/bin/env python3
"""Pure-function checks for scan_bt_bridge (needs the ROS env only for msgs)."""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import scan_bt_bridge as bridge  # noqa: E402


def bspline(ctrl, knots):
    pts = [SimpleNamespace(x=x, y=y) for x, y in ctrl]
    return SimpleNamespace(pos_pts=pts, knots=knots, order=3)


def check(cond, what):
    print('%s  %s' % ('PASS' if cond else 'FAIL', what))
    return 0 if cond else 1


def main():
    failures = 0
    quad = bspline([(0., 0.), (1., 2.), (2., 0.)], [0., 0., 0., 1., 1., 1.])
    points = bridge.sample_bspline(quad, 120)
    failures += check(len(points) == 120, 'quadratic B-spline samples 120 points')
    failures += check(abs(points[0][0]) < 1e-9 and abs(points[0][1]) < 1e-9,
                      'curve starts at first control point')
    failures += check(abs(points[-1][0] - 2.) < 1e-9 and abs(points[-1][1]) < 1e-9,
                      'curve ends at last control point')
    failures += check(abs(points[60][0] - 1.) < 0.05 and points[60][1] > 0.9,
                      'curve midpoint stays inside the hull')

    failures += check(bridge.sample_bspline(bspline([], [0., 0., 0., 1.])) == [],
                      'empty control points rejected')
    failures += check(bridge.sample_bspline(bspline([(0., 0.), (1., 1.)], [0., 0.])) == [],
                      'short knot vector rejected')
    failures += check(bridge.sample_bspline(bspline([(0., 0.), (1., 1.)], [1., 0., 0., 1., 1.])) == [],
                      'non-monotonic knots rejected')

    message = bridge.path_message(points[:2], 'camera_init', None)
    failures += check(len(message.poses) == 2, 'path message keeps sampled points')
    failures += check(message.header.frame_id == 'camera_init', 'path frame set')
    failures += check(abs(message.poses[0].pose.orientation.w - 1.) < 1e-9, 'path quaternion set')

    ok = bridge.status_message(True, 12).data
    running = bridge.status_message(False).data
    failures += check('"ok": true' in ok and '"points": 12' in ok, 'ready status json')
    failures += check('E_PLAN_RUNNING' in running, 'running status json')

    print('FAILED (%d)' % failures if failures else 'ALL PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
