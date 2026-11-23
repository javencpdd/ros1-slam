#!/usr/bin/env python3
"""碰撞模型回归：AABB 细化与速度相关膨胀的定量对比。

四个场景（前两个与"过度包围"直接相关，后两个守住安全侧）：
  A 复杂障碍   真实点云帧；同一组候选直线段在细化前后各判一次，算通行率变化
  B 狭窄通道   合成两侧墙，扫间距，找"能通过的最小净宽"，与理论下界对照
  C 动态障碍   同一障碍位置，扫速度，看安全距离随速度增长（fail-closed）
  D 正常通行   空旷场景，确认没有被误判成不可通行

判定全部走 local_native 的真实函数（_swept_hit / dynamic_margin_extra），
不复制一份判据，避免"回归测的是回归自己的实现"。
"""
import argparse
import json
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'ws/src/90_platform/v8_platform/framework'))
sys.path.insert(0, os.path.join(REPO, 'ws/src/80_algorithm/v8_algorithm/local_planner'))
import dynamic_obstacles as do  # noqa: E402
import local_native as native  # noqa: E402

PARAMS = {
    'floor_z': 0.0, 'roi_forward_m': 6.0, 'roi_lateral_m': 3.0,
    'min_height_m': 0.15, 'max_height_m': 1.6, 'voxel_m': 0.10,
    'min_points': 8, 'max_objects': 32, 'self_radius_m': 0.45,
}
# 被检验的细化开关：默认 0.45（已交付配置）。传 1.0 可做反控 —— 判定必须随之变 FAIL，
# 否则说明"判定"根本没在看细化效果。
REFINE_FILL = 0.45
BASE_CFG = dict(radius_m=0.15, clearance_m=0.10, max_speed_mps=0.6,
                height_m=0.35, reaction_s=0.10, brake_decel_mps2=1.0,
                dynamic_margin_cap_m=0.30)


def cfg(**over):
    return native.Config(**dict(BASE_CFG, **over))


def as_obstacle(box, active=(0.0, 1e9)):
    return (tuple(box['min']), tuple(box['max']), (0.0, 0.0, 0.0), 0.0,
            active[0], active[1])


def seg(x0, y0, x1, y1, z=0.0, duration=4.0):
    return ((x0, y0, z, 0.0), (x1, y1, z, duration))


def blocked(segment, boxes, config, speed=None):
    extra = native.dynamic_margin_extra(config, speed)
    a, b = segment
    for box in boxes:
        if native._swept_hit(a, b, as_obstacle(box), config, margin_extra=extra):
            return True
    return False


def min_gap(segment, boxes):
    """线段到各框的 xy 最短距离（框外为正；穿过为 0）。"""
    (x0, y0, _, _), (x1, y1, _, _) = segment
    best = float('inf')
    for box in boxes:
        lo, hi = box['min'], box['max']
        worst = 0.0
        still_out = True
        for k in range(41):
            t = k / 40.0
            x, y = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            dx = max(lo[0] - x, 0.0, x - hi[0])
            dy = max(lo[1] - y, 0.0, y - hi[1])
            dist = math.hypot(dx, dy)
            if dist <= 0.0:
                still_out = False
                break
            worst = max(worst, dist)
        if still_out:
            best = min(best, worst)
        else:
            best = 0.0
            break
    return best


def boxes_from_cloud(points, center, min_fill):
    params = dict(PARAMS)
    roi = do.roi_filter(points, center[:2], params['floor_z'], params['roi_forward_m'],
                        params['roi_lateral_m'], params['min_height_m'], params['max_height_m'])
    roi = do.self_filter(roi, center[:2], params['self_radius_m'])
    if not len(roi):
        return []
    centers, keys = do.voxel_downsample(roi, params['voxel_m'])
    return do.cluster_boxes(centers, keys, params['min_points'], params['max_objects'],
                            min_fill=min_fill, min_extent_m=0.45, max_depth=4,
                            voxel_m=params['voxel_m'])


def scene_a_complex(frame_path, center):
    """真实帧：算出"能走的候选方向"在细化前后的比例。"""
    points = np.load(frame_path)
    before = boxes_from_cloud(points, center, 1.0)
    after = boxes_from_cloud(points, center, REFINE_FILL)
    config = cfg()
    total = ok_before = ok_after = 0
    gap_before = gap_after = float('inf')
    for k in range(72):
        angle = 2.0 * math.pi * k / 72.0
        for reach in (1.0, 1.5, 2.0):
            x1 = center[0] + reach * math.cos(angle)
            y1 = center[1] + reach * math.sin(angle)
            segment = seg(center[0], center[1], x1, y1, z=0.0, duration=max(reach / 0.25, 0.1))
            total += 1
            b = blocked(segment, before, config, speed=0.25)
            a = blocked(segment, after, config, speed=0.25)
            ok_before += 0 if b else 1
            ok_after += 0 if a else 1
            if not b:
                gap_before = min(gap_before, min_gap(segment, before))
            if not a:
                gap_after = min(gap_after, min_gap(segment, after))
    return {
        'scene': 'A_complex_real_frame',
        'boxes_before': len(before), 'boxes_after': len(after),
        'segments': total,
        'passable_before': ok_before, 'passable_after': ok_after,
        'pass_rate_before': round(ok_before / total, 3),
        'pass_rate_after': round(ok_after / total, 3),
        'min_gap_before_m': None if gap_before == float('inf') else round(gap_before, 3),
        'min_gap_after_m': None if gap_after == float('inf') else round(gap_after, 3),
    }


def wall_pair(gap_m, length=2.0, thickness=0.3):
    """两侧墙，净宽 gap_m，沿 x 方向留出通道。"""
    half = gap_m / 2.0
    boxes = []
    for sign in (-1.0, 1.0):
        y0 = sign * half if sign > 0 else -half - thickness
        boxes.append({'min': [0.8, y0, 0.0], 'max': [0.8 + length, y0 + thickness, 1.0],
                      'points': 50})
    return boxes


def l_shape_cloud():
    """L 形墙：横臂 x∈[1,3] y∈[0,0.3]，竖臂 x∈[1,1.3] y∈[0.3,2]。"""
    pts = []
    for x in np.arange(1.0, 3.0, 0.05):
        for y in np.arange(0.02, 0.30, 0.05):
            pts.append([x, y, 0.30])
    for x in np.arange(1.0, 1.30, 0.05):
        for y in np.arange(0.30, 2.00, 0.05):
            pts.append([x, y, 0.30])
    return np.array(pts, dtype=float)


def scene_a2_lshape():
    """受控 L 形：一条真实可走、却被单个 AABB 判死的通道。

    走 (2.0,1.0) -> (2.0,2.5)：竖臂在 x∈[1,1.3]，横臂在 y∈[0,0.3]，两者都不挡；
    但 L 的 AABB 是 x∈[1,3] y∈[0,2]，这条线整段落在盒内 —— 细化前后应给出相反的判定。
    """
    points = l_shape_cloud()
    center = (0.5, 1.0)
    before = boxes_from_cloud(points, center, 1.0)
    after = boxes_from_cloud(points, center, REFINE_FILL)
    config = cfg()
    segment = seg(2.0, 1.0, 2.0, 2.5, z=0.30, duration=6.0)
    hit_before = blocked(segment, before, config, speed=0.25)
    hit_after = blocked(segment, after, config, speed=0.25)

    def area(bs):
        return round(sum((b['max'][0] - b['min'][0]) * (b['max'][1] - b['min'][1]) for b in bs), 3)

    return {
        'scene': 'A2_lshape_controlled',
        'boxes_before': len(before), 'boxes_after': len(after),
        'area_before_m2': area(before), 'area_after_m2': area(after),
        'probe_segment': [[2.0, 1.0], [2.0, 2.5]],
        'truly_free': True,
        'blocked_before': hit_before, 'blocked_after': hit_after,
        'false_block_removed': bool(hit_before and not hit_after),
    }


def scene_b_narrow():
    """扫净宽，找"实际能通过的最小净宽"，与 2*(radius+clearance+extra) 对照。"""
    config = cfg()
    speed = 0.25
    extra = native.dynamic_margin_extra(config, speed)
    need = 2.0 * (config.radius_m + config.clearance_m + extra)
    smallest = None
    rows = []
    for gap_cm in range(20, 141, 5):
        gap = gap_cm / 100.0
        segment = seg(0.0, 0.0, 3.0, 0.0, duration=3.0 / max(speed, 1e-6))
        hit = blocked(segment, wall_pair(gap), config, speed=speed)
        rows.append([gap, not hit])
        if not hit and smallest is None:
            smallest = gap
    return {
        'scene': 'B_narrow_corridor',
        'required_clearance_m': round(need, 3),
        'smallest_passable_gap_m': smallest,
        'margin_over_required_m': None if smallest is None else round(smallest - need, 3),
        'sweep': rows,
    }


def scene_c_dynamic():
    """障碍在路上：随速度增大，原本可行的段必须变成不可行（安全侧增长）。"""
    config = cfg()
    box = {'min': [1.0, -0.3, 0.0], 'max': [1.4, 0.3, 1.0], 'points': 50}
    rows = []
    for speed in (0.05, 0.10, 0.20, 0.30, 0.40, 0.60):
        segment = seg(0.0, 0.0, 2.0, 0.0, duration=max(2.0 / speed, 1e-6))
        rows.append([speed, round(native.dynamic_margin_extra(config, speed), 3),
                     blocked(segment, [box], config, speed=speed)])
    # 侧向绕行 1.0 m 的段，看它在各速度下是否仍可通过
    side = []
    for speed in (0.05, 0.20, 0.60):
        segment = seg(0.0, 1.0, 2.0, 1.0, duration=max(2.0 / speed, 1e-6))
        side.append([speed, not blocked(segment, [box], config, speed=speed)])
    unknown = blocked(seg(0.0, 0.0, 2.0, 0.0, duration=8.0), [box], config, speed=None)
    return {
        'scene': 'C_dynamic_obstacle',
        'head_on_by_speed': rows,
        'side_pass_by_speed': side,
        'blocked_when_speed_unknown': unknown,
    }


def scene_d_open():
    config = cfg()
    segment = seg(0.0, 0.0, 2.0, 0.0, duration=8.0)
    far = {'min': [3.0, 3.0, 0.0], 'max': [3.4, 3.4, 1.0], 'points': 50}
    return {
        'scene': 'D_open_space',
        'passable_no_obstacles': not blocked(segment, [], config, speed=0.6),
        'passable_far_obstacle': not blocked(segment, [far], config, speed=0.6),
    }


def judge(results):
    """把四个场景的读数折成机器判定；任一不成立即整体 FAIL（调用方据此返回非零）。

    判据只表达"优化不该变差、且安全侧必须成立"，不引入新的阈值：
      A  复杂障碍（真实帧）：细化后的通行率不得低于细化前（允许相等）；
      A2 受控 L 形：必须确实把误判去掉（before 碰、after 不碰）；
      B  狭窄通道：实测最小可通过净宽不得小于理论下界（否则说明膨胀没生效）；
      C  动态障碍：余量随速度单调不减，且速度未知时判碰（fail-closed）；
      D  空旷：无障碍与远处障碍都必须可通过。
    skipped 的场景（如没有 --frame）记为 INVALID，不参与通过性判定但会让整体不是 PASS。
    """
    checks = {}
    verdicts = {}
    for item in results:
        scene = item.get('scene')
        if item.get('skipped'):
            verdicts[scene] = 'INVALID'
            continue
        ok = None
        if scene == 'A_complex_real_frame':
            ok = item['pass_rate_after'] >= item['pass_rate_before'] - 1e-9
            checks[scene] = 'pass_rate_after %.3f >= before %.3f' % (
                item['pass_rate_after'], item['pass_rate_before'])
        elif scene == 'A2_lshape_controlled':
            ok = bool(item.get('false_block_removed'))
            checks[scene] = 'blocked_before=%s blocked_after=%s' % (
                item.get('blocked_before'), item.get('blocked_after'))
        elif scene == 'B_narrow_corridor':
            need = item.get('required_clearance_m')
            got = item.get('smallest_passable_gap_m')
            ok = (got is not None) and (need is not None) and got >= need - 1e-9
            checks[scene] = 'smallest_passable %.3f >= required %.3f' % (got or -1, need or -1)
        elif scene == 'C_dynamic_obstacle':
            rows = item.get('head_on_by_speed') or []
            extras = [r[1] for r in rows]
            monotone = all(extras[i + 1] >= extras[i] - 1e-9 for i in range(len(extras) - 1))
            failclosed = bool(item.get('blocked_when_speed_unknown'))
            ok = monotone and failclosed
            checks[scene] = 'extra_monotone=%s failclosed=%s' % (monotone, failclosed)
        elif scene == 'D_open_space':
            ok = bool(item.get('passable_no_obstacles')) and bool(item.get('passable_far_obstacle'))
            checks[scene] = 'open=%s far=%s' % (item.get('passable_no_obstacles'),
                                                item.get('passable_far_obstacle'))
        if ok is not None:
            verdicts[scene] = 'PASS' if ok else 'FAIL'

    overall = ('PASS' if verdicts and all(v == 'PASS' for v in verdicts.values())
               else 'INVALID' if any(v == 'INVALID' for v in verdicts.values())
               else 'FAIL')
    return {'verdict': overall, 'per_scene': verdicts, 'checks': checks}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frame', default='', help='scene A 用的点云 npy')
    ap.add_argument('--center', default='0.927,-3.098')
    ap.add_argument('--out', default='')
    ap.add_argument('--min-fill', type=float, default=0.45,
                    help='细化开关；1.0 = 不细化，用于反控（判定应变 FAIL 且退出码非零）')
    args = ap.parse_args()

    global REFINE_FILL
    REFINE_FILL = args.min_fill
    center = tuple(float(v) for v in args.center.split(','))
    results = []
    if args.frame and os.path.isfile(args.frame):
        results.append(scene_a_complex(args.frame, center))
    else:
        results.append({'scene': 'A_complex_real_frame', 'skipped': 'no --frame'})
    results.append(scene_a2_lshape())
    results.append(scene_b_narrow())
    results.append(scene_c_dynamic())
    results.append(scene_d_open())

    for item in results:
        print(json.dumps(item, ensure_ascii=False))

    verdict = judge(results)
    print(json.dumps({'verdict': verdict['verdict'], **verdict}, ensure_ascii=False, indent=1))
    if args.out:
        with open(args.out, 'w') as stream:
            stream.write(json.dumps({'results': results, 'judgement': verdict},
                                    ensure_ascii=False, indent=1) + '\n')
    # 失败必须非零退出：否则 CI/回归脚本会把 FAIL 当成功
    return 0 if verdict['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
