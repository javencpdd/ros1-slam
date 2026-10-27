"""flat2d 场景几何预检：目标净空 + 最宽走廊瓶颈。

几何预检只是必要条件，不能预测规划器行为（R775 实证：v1 全部目标都过预检，
但 SC02 仍出现净空违规与卡死）。定标前仍须 pilot 实跑。

用法：python3 validate_goals.py [scenarios.json]
输出 JSON；全部目标达标 exit 0，否则 exit 1。
"""
import heapq
import json
import math
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
GOAL_CLEARANCE_MIN_M = 0.40
PATH_BOTTLENECK_MIN_M = 0.55


def box_dist(px, py, cx, cy, sx, sy, yaw):
    c, s = math.cos(-yaw), math.sin(-yaw)
    dx, dy = px - cx, py - cy
    lx, ly = dx * c - dy * s, dx * s + dy * c
    return max(abs(lx) - sx / 2, abs(ly) - sy / 2, 0.0)


class World:
    def __init__(self, model):
        self.boxes = [tuple(b) for b in model['boxes']]
        self.res = model['resolution']
        self.ox, self.oy = model['origin']
        self.w, self.h = model['size']
        self.start = tuple(model['start'])
        self._clear = [[self.clear(*self.center(c, r)) for r in range(self.h)]
                       for c in range(self.w)]

    def clear(self, px, py):
        return min(box_dist(px, py, *b) for b in self.boxes)

    def cell(self, x, y):
        return int((x - self.ox) / self.res), int((y - self.oy) / self.res)

    def center(self, c, r):
        return self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res

    def widest_path(self, goal):
        sc, sr = self.cell(*self.start)
        gc, gr = self.cell(*goal)
        if not (0 <= gc < self.w and 0 <= gr < self.h):
            return None, None
        best = [[-1.0] * self.h for _ in range(self.w)]
        best[sc][sr] = self._clear[sc][sr]
        pq = [(-best[sc][sr], sc, sr)]
        prev = {}
        while pq:
            negb, c, r = heapq.heappop(pq)
            b = -negb
            if b < best[c][r] - 1e-12:
                continue
            if (c, r) == (gc, gr):
                break
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nc, nr = c + dc, r + dr
                if not (0 <= nc < self.w and 0 <= nr < self.h):
                    continue
                nb = min(b, self._clear[nc][nr])
                if nb > best[nc][nr] + 1e-9:
                    best[nc][nr] = nb
                    prev[(nc, nr)] = (c, r)
                    heapq.heappush(pq, (-nb, nc, nr))
        if best[gc][gr] < 0:
            return None, None
        path = [(gc, gr)]
        cur = (gc, gr)
        while cur != (sc, sr):
            cur = prev[cur]
            path.append(cur)
        pts = [self.center(*p) for p in path]
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        return best[gc][gr], length


def check_goals(model, scenarios):
    world = World(model)
    rows = []
    for scenario in scenarios['scenarios']:
        for group, goal in enumerate(scenario['goals']):
            bottleneck, length = world.widest_path(goal)
            row = {
                'scenario_id': scenario['id'],
                'group': group,
                'goal': list(goal),
                'goal_clearance_m': round(world.clear(*goal), 3),
                'path_bottleneck_m': None if bottleneck is None else round(bottleneck, 3),
                'path_len_m': None if length is None else round(length, 2),
            }
            row['ok'] = bool(row['goal_clearance_m'] >= GOAL_CLEARANCE_MIN_M
                             and bottleneck is not None
                             and bottleneck >= PATH_BOTTLENECK_MIN_M)
            rows.append(row)
    return {
        'thresholds': {'goal_clearance_min_m': GOAL_CLEARANCE_MIN_M,
                       'path_bottleneck_min_m': PATH_BOTTLENECK_MIN_M,
                       'clearance_definition': 'robot_center_to_box_surface_m'},
        'all_ok': all(r['ok'] for r in rows),
        'rows': rows,
    }


def main(argv):
    path = pathlib.Path(argv[0]) if argv else HERE / 'scenarios_v2.json'
    model = json.loads((HERE / 'world_model.json').read_text())
    scenarios = json.loads(path.read_text())
    report = check_goals(model, scenarios)
    print(json.dumps(report, ensure_ascii=False, indent=1))
    return 0 if report['all_ok'] else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
