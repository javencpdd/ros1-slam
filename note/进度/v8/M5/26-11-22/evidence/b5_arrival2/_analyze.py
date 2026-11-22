"""把 NDJSON 事件流切成场景，逐场景出读数与判据结论。

判据都写在 `verdicts` 里、逐条带理由——改判据先改这里再看数字，不靠「看着像通过」。

★ 读结论时必须连着 C1 看 ★ C2／C4 的「arrived 一直是 false」只在「这条通道确实能发出
true」成立时才有意义，C1 就是那个正向对照。没有 C1 的零，与「判据坏了、永远不发」
分不开——v1 的读数正是这么被误读的。

★ 证据分两档，报告里必须写明是哪一档 ★
  STRONG       上游 FAR 侧也有动静（`/far_reach_goal_status` 出帧、`/way_point` 或
               `/global_path` 有产出）⇒ 目标不仅到了适配器，也到了上游规划器。
  ADAPTER_ONLY 只有适配器侧的动静（新目标当场发的 arrived=false ＋ 位姿在跳）
               ⇒ 对「到达判据」这个待验对象已经够：判据的输入就是 pose 与 goal，
               两者都有当场证据。但不主张「上游也认这个目标」。
"""
import json
import sys

SHIFT_M = 3.0          # C4 的位姿抬升量（米）
SHIFT_TOL = 0.05       # 位姿抬升回读容差（米）


def load(path):
    records = []
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def slice_by_mark(records):
    """按 mark 切片，**连续同名标记合成一片**。

    v2 一开始每个场景发 3 条相同的标记（防丢），结果每个场景被切成 3 片，
    读数被摊平（见 attemptB 的日志）。同名连续标记显然是同一次切换。
    """
    slices, current = [], None
    for record in records:
        if record['kind'] == 'mark':
            if current is not None and current['tag'] == record['tag']:
                continue
            current = {'tag': record['tag'], 't0': record['t'], 'events': []}
            slices.append(current)
            continue
        if current is not None:
            current['events'].append(record)
    return slices


def summarize(name, slc):
    arrived_true = arrived_false = reach_true = reach_false = waypoints = 0
    poses = 0
    stamps = set()
    pose_z = []
    reasons = set()
    for event in slc['events']:
        kind = event['kind']
        if kind == 'arrived':
            if event['data']:
                arrived_true += 1
            else:
                arrived_false += 1
        elif kind == 'reach':
            if event['data']:
                reach_true += 1
            else:
                reach_false += 1
        elif kind == 'waypoint':
            waypoints += 1
        elif kind == 'path':
            stamps.add(event['stamp'])
        elif kind == 'pose':
            poses += 1
            pose_z.append(event['p'][2])
        elif kind == 'status':
            try:
                payload = json.loads(event['text'])
            except ValueError:
                continue
            if payload.get('reason'):
                reasons.add(payload['reason'])
    return {
        'name': name,
        'arrived_true': arrived_true,
        'arrived_false': arrived_false,
        'reach_true': reach_true,
        'reach_false': reach_false,
        'waypoints': waypoints,
        'paths': len(stamps),
        'poses': poses,
        'pose_z_max': max(pose_z) if pose_z else float('nan'),
        'reason': ','.join(sorted(reasons)),
    }


def upstream_alive(row):
    return row['reach_false'] > 0 or row['waypoints'] > 0 or row['paths'] > 0


def main():
    rows = [summarize(slc['tag'], slc) for slc in slice_by_mark(load(sys.argv[1]))]
    for row in rows:
        print('%(name)-16s arrived_true=%(arrived_true)d arrived_false=%(arrived_false)d '
              'reach_true=%(reach_true)d reach_false=%(reach_false)d '
              'waypoints=%(waypoints)d path_stamps=%(paths)d poses=%(poses)d '
              'pose_z_max=%(pose_z_max).3f reasons=%(reason)s' % row, flush=True)

    by_name = {row['name']: row for row in rows}
    verdicts = []

    c0 = by_name.get('C0_liveness')
    if c0:
        if c0['arrived_true'] != 0:
            state, why = 'FAIL', '3.0 m 远的目标被判到达'
        elif not upstream_alive(c0):
            state, why = 'INCONCLUSIVE', '上游从未接受目标（无 reach、无航点、无路径）'
        else:
            state, why = 'PASS', '目标被上游接受且 3.0 m 远不判到达'
        verdicts.append(('C0_liveness', state, why, 'STRONG' if upstream_alive(c0) else '-'))

    c1 = by_name.get('C1_arrival')
    channel_ok = bool(c1 and c1['arrived_true'] >= 1)
    if c1:
        verdicts.append(('C1_arrival', 'PASS' if channel_ok else 'FAIL',
                         '正向对照：0.2 m 目标必须出现 arrived_true>=1',
                         'STRONG' if upstream_alive(c1) else 'ADAPTER_ONLY'))

    for name, expect_z in (('C2_cross_floor', None), ('C4_pose_shift', SHIFT_M)):
        row = by_name.get(name)
        if not row:
            continue
        if row['arrived_true'] != 0:
            state, why = 'FAIL', '出现 arrived_true=%d' % row['arrived_true']
        elif not channel_ok:
            state, why = 'INCONCLUSIVE', 'C1 没拿到 true，无法区分「判据对」与「通道报废」'
        elif row['arrived_false'] < 1 or row['poses'] < 1:
            state, why = 'INCONCLUSIVE', '没有「新目标落定」或「位姿在跳」的证据'
        elif expect_z is not None and row['pose_z_max'] < expect_z - SHIFT_TOL:
            state, why = 'INCONCLUSIVE', '位姿没真抬上去（pose_z_max=%.3f）' % row['pose_z_max']
        else:
            state, why = 'PASS', '整个窗口 arrived 一直 false，且目标与位姿都有当场证据'
        verdicts.append((name, state, why, 'STRONG' if upstream_alive(row) else 'ADAPTER_ONLY'))

    print('', flush=True)
    for name, state, why, tier in verdicts:
        print('VERDICT %-16s %-12s [%-12s] %s' % (name, state, tier, why), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
