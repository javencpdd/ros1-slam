"""Bounded, seeded map->odom perturbation for the estimated-localization track.

This is a test-input generator, not LIO/NDT and not a sensor model: it produces
a deterministic, slowly varying offset so the navigation stack runs with a
localization belief that is not Gazebo ground truth. Evaluation still uses
/sim/odom_gt (truth), so the perturbation cannot turn a miss into a hit.
"""
import math
import random

AMP_XY_M = 0.05
AMP_YAW_RAD = 0.03


def offset(t, seed, amp_xy=AMP_XY_M, amp_yaw=AMP_YAW_RAD):
    if not all(isinstance(v, (int, float)) and math.isfinite(v)
               for v in (t, amp_xy, amp_yaw)):
        raise ValueError('finite t, amp_xy and amp_yaw required')
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ValueError('integer seed required')
    if t < 0 or amp_xy < 0 or amp_yaw < 0:
        raise ValueError('t and amplitudes must be nonnegative')
    rnd = random.Random(seed)
    phases = [rnd.uniform(0.0, 2.0 * math.pi) for _ in range(6)]
    x = amp_xy * (0.6 * math.sin(0.31 * t + phases[0])
                  + 0.4 * math.sin(0.73 * t + phases[1]))
    y = amp_xy * (0.6 * math.sin(0.27 * t + phases[2])
                  + 0.4 * math.sin(0.67 * t + phases[3]))
    yaw = amp_yaw * (0.6 * math.sin(0.19 * t + phases[4])
                     + 0.4 * math.sin(0.41 * t + phases[5]))
    return x, y, yaw
