#!/usr/bin/env python3
"""TerrainObserver：从高程栅格产出地形特征，纯 Python 无 ROS。

定位（在册事实）：
  - 感知源 = elevation_gridmap_publisher.py 发到 /elevation_mapping/elevation_map
    的 grid_map_msgs/GridMap，层名 elevation，frame=map，2.5D 离地高，
    data 为列主序扁平（与 grid_map::Matrix / Eigen 列主序一致），NaN = 无效格。
  - 声明几何地图前视 = stair_ahead.py:89-94，输出 ahead / distance_m / step_m。
    本模块输出与之同形同单位，便于影子通道逐条对照。

为什么要新增判别维（F10）：
  stairstudy/fixtures.py:86-87 的 stairs_mid 与 ramp_envelope 包络逐点同量
  （rise 0.80 / run 2.24），但阶面是竖直不连续、斜坡是连续梯度，
  步态需求不同。只把高程源塞进 stair_ahead 会把两者判成同一件事。
  因此必须输出 discontinuity_type：vertical_step / continuous_slope / none。

明确局限（不得当能力证书）：
  - clearance_m 是「沿路径前方窗口内、横向连续有效格数 × resolution」的
    保守估计，不是真实侧向净宽或可通过性证明；侧壁薄板、悬空、负障碍
    在 2.5D 高程里不可见。
  - 本模块只产出特征，不写 /v8/chassis/gait，不证明底盘已执行。
"""
import math

DISCONTINUITY_TYPES = ('none', 'vertical_step', 'continuous_slope',
                       'blocked', 'insufficient_data')


def gridmap_to_matrix(flat, n_rows, n_cols):
    """列主序扁平 -> matrix[row][col]，与 elevation_gridmap_publisher.py 的
    fields_to_matrix（reshape(n_cols, n_rows).T）逐字同口径。

    flat[col * n_rows + row] 即 matrix[row][col]。NaN 原样保留（无效格）。
    """
    if n_rows <= 0 or n_cols <= 0:
        raise ValueError('grid dimensions must be positive')
    if len(flat) != n_rows * n_cols:
        raise ValueError('flat length %d != %d x %d' % (len(flat), n_rows, n_cols))
    return [[float(flat[col * n_rows + row]) for col in range(n_cols)]
            for row in range(n_rows)]


def world_to_cell(x, y, center_x, center_y, n_rows, n_cols, resolution):
    """GridMap 世界坐标 -> (row, col)。越界返回 None。

    x 随 col 增大，y 随 row 增大而减小（图像行序）。
    """
    col = int(math.floor((x - center_x) / resolution + n_cols / 2.0))
    row = int(math.floor((center_y - y) / resolution + n_rows / 2.0))
    if 0 <= row < n_rows and 0 <= col < n_cols:
        return (row, col)
    return None


def _finite(value):
    return value is not None and math.isfinite(value)


def _cell(matrix, row, col):
    if 0 <= row < len(matrix) and 0 <= col < len(matrix[0]):
        return matrix[row][col]
    return None


def _trace_path(matrix, path_cells, lookahead_m, resolution):
    """沿格坐标路径按格步长展开成采样序列 [(row, col, travelled_m), ...]。"""
    if not path_cells:
        return []
    samples = []
    travelled = 0.0
    prev = path_cells[0]
    samples.append((prev[0], prev[1], 0.0))
    for cell in path_cells[1:]:
        step = math.hypot(cell[0] - prev[0], cell[1] - prev[1]) * resolution
        travelled += step
        if travelled > lookahead_m + 1e-9:
            break
        samples.append((cell[0], cell[1], round(travelled, 4)))
        prev = cell
    return samples


def _lateral_clearance(matrix, row, col, half_width, max_step_m):
    """横向连续有效格数（保守）：以 (row,col) 为中心向两侧扫，
    遇 NaN / 越界 / 高差超过 max_step_m 即停止。返回格数。
    """
    center_h = _cell(matrix, row, col)
    if not _finite(center_h):
        return 0
    width = 1
    for sign in (-1, 1):
        for offset in range(1, half_width + 1):
            c = col + sign * offset
            h = _cell(matrix, row, c)
            if not _finite(h) or abs(h - center_h) > max_step_m:
                break
            width += 1
    return width


def observe(matrix, path_cells, resolution=0.05, lookahead_m=1.5,
            min_step_m=0.05, max_step_m=0.15, half_width=5,
            min_valid_cells=3):
    """产出地形特征 dict。path_cells 为格坐标 [(row, col), ...]。

    空集拒判：有效采样少于 min_valid_cells 时返回
    discontinuity_type='insufficient_data'，不印绿、不给 ahead=True。
    """
    samples = _trace_path(matrix, path_cells, lookahead_m, resolution)
    heights = []
    clearances = []
    travelled_at = []
    for row, col, dist in samples:
        h = _cell(matrix, row, col)
        if _finite(h):
            heights.append((h, row, col, dist))
            clearances.append(_lateral_clearance(matrix, row, col,
                                                half_width, max_step_m))
            travelled_at.append(dist)

    out = {
        'discontinuity_type': 'insufficient_data',
        'ahead': False,
        'step_m': None,
        'distance_m': None,
        'clearance_m': None,
        'slope_deg': None,
        'valid_cells': len(heights),
        'sampled_cells': len(samples),
        'scanned_m': round(travelled_at[-1], 4) if travelled_at else 0.0,
        'reason': '',
        'source': 'elevation_gridmap',
        'frame': 'map',
    }
    if len(heights) < min_valid_cells:
        out['reason'] = 'valid_cells_below_min:%d<%d' % (len(heights), min_valid_cells)
        return out

    out['clearance_m'] = round(min(clearances) * resolution, 4)

    # 相邻有效采样点的高程序列 -> 差分
    diffs = []
    for i in range(1, len(heights)):
        dh = heights[i][0] - heights[i - 1][0]
        ds = heights[i][3] - heights[i - 1][3]
        if ds > 0:
            diffs.append((abs(dh), dh, ds, heights[i][3]))

    if not diffs:
        out['discontinuity_type'] = 'none'
        out['reason'] = 'no_positive_forward_step'
        return out

    total_rise = heights[-1][0] - heights[0][0]
    peak = max(d[0] for d in diffs)
    # 竖直不连续：单步跳变既超过 min_step_m，又占了总升高的主要部分
    # 连续梯度：单步跳变小，但多步同号累积出总升高
    significant = [d for d in diffs if d[0] >= min_step_m]
    if peak >= min_step_m and abs(total_rise) > 1e-9 \
            and peak / abs(total_rise) >= 0.6 and len(significant) <= 2:
        out['discontinuity_type'] = 'vertical_step'
        idx = max(range(len(diffs)), key=lambda i: diffs[i][0])
        out['ahead'] = True
        out['step_m'] = round(diffs[idx][0], 4)
        out['distance_m'] = round(diffs[idx][3], 3)
    elif abs(total_rise) >= min_step_m and len(significant) == 0:
        out['discontinuity_type'] = 'continuous_slope'
        run = heights[-1][3] - heights[0][3]
        out['slope_deg'] = round(math.degrees(math.atan2(abs(total_rise), run)), 3) \
            if run > 0 else None
        out['ahead'] = False  # 斜坡不是台阶，不应触发 high_step 切换
        out['reason'] = 'continuous_gradient_not_a_step_edge'
    else:
        out['discontinuity_type'] = 'none'
        out['reason'] = 'no_dominant_discontinuity'

    # 超过可达阶高 = 堵头（与 GaitDecision 的 step_too_high 同义，但来自感知侧）
    if out['step_m'] is not None and out['step_m'] > max_step_m:
        out['discontinuity_type'] = 'blocked'
        out['reason'] = 'step_exceeds_max_step_m'

    out['total_rise_m'] = round(total_rise, 4)
    out['peak_diff_m'] = round(peak, 4)
    return out


def shadow_compare(declared_result, terrain_result):
    """影子通道对照：只产出分歧记录，不修改任何决策输入。

    返回 dict：consistent / disagreements / note。
    """
    d_step = declared_result.get('step_m') if declared_result.get('ahead') else None
    t_step = terrain_result.get('step_m') if terrain_result.get('ahead') else None
    disagreements = []
    if bool(declared_result.get('ahead')) != bool(terrain_result.get('ahead')):
        disagreements.append('ahead_flag_differs')
    if d_step is not None and t_step is not None and abs(d_step - t_step) > min_step_tolerance():
        disagreements.append('step_m_differs')
    if terrain_result.get('discontinuity_type') == 'continuous_slope' \
            and declared_result.get('ahead'):
        disagreements.append('declared_step_but_terrain_is_slope')
    if terrain_result.get('discontinuity_type') == 'blocked':
        disagreements.append('terrain_blocked')
    return {
        'consistent': not disagreements,
        'disagreements': disagreements,
        'declared_ahead': bool(declared_result.get('ahead')),
        'terrain_type': terrain_result.get('discontinuity_type'),
        'note': 'shadow only; never writes /v8/chassis/gait',
    }


def min_step_tolerance():
    """阶高对照容差（米）：与 min_step_m 默认量级一致，避免浮点噪声误报。"""
    return 0.02


# ── 三连自测 ──────────────────────────────────────────────────────────────
def _flat_matrix(n_rows=9, n_cols=20, height=0.0):
    return [[height for _ in range(n_cols)] for _ in range(n_rows)]


def _selftest():
    nan = float('nan')
    res = 0.05

    # 0) 列主序还原口径：flat[col*n_rows+row] == matrix[row][col]
    n_rows, n_cols = 3, 4
    flat = [float(col * n_rows + row) for col in range(n_cols) for row in range(n_rows)]
    m = gridmap_to_matrix(flat, n_rows, n_cols)
    for row in range(n_rows):
        for col in range(n_cols):
            assert m[row][col] == float(col * n_rows + row)
    print('列主序 OK：flat[col*n_rows+row] == matrix[row][col]')

    # 1) 主测：平地 -> none，且 clearance 可测
    # clearance 是「横向连续有效格数 × res」，不是纵向行数 n_rows。
    # 路径起点 col=0：向左越界即停、向右最多 half_width=5 格，故 1+5=6 格。
    flat_ground = _flat_matrix()
    path = [(4, c) for c in range(20)]
    r = observe(flat_ground, path, resolution=res)
    assert r['valid_cells'] == 20 and r['discontinuity_type'] == 'none'
    assert r['clearance_m'] == round(6 * res, 4), ('clearance', r['clearance_m'])
    print('主测 OK：平地判 none，valid_cells=%d，clearance=%s m'
          % (r['valid_cells'], r['clearance_m']))

    # 注入有效性：放宽横向半宽，净宽读数必须真的跟着变（防「净宽是常数」空控）
    r_wide = observe(flat_ground, path, resolution=res, half_width=8)
    assert r_wide['clearance_m'] == round(9 * res, 4), \
        ('净宽注入 FAIL', r_wide['clearance_m'])
    assert r_wide['clearance_m'] > r['clearance_m']
    print('净宽注入 OK：half_width 5->8，clearance %s -> %s m'
          % (r['clearance_m'], r_wide['clearance_m']))

    # 2) 空集拒判：全 NaN 必须拒绝给绿
    nan_ground = [[nan for _ in range(20)] for _ in range(9)]
    r = observe(nan_ground, path, resolution=res)
    assert r['discontinuity_type'] == 'insufficient_data'
    assert r['ahead'] is False and r['clearance_m'] is None
    print('空集拒判 OK：全 NaN 判 insufficient_data，不印绿')

    # 3) F10 判别对照：包络同量（总升高 0.10）但阶面 vs 斜坡必须分开
    step_ground = _flat_matrix()
    for c in range(10, 20):
        for row in range(9):
            step_ground[row][c] = 0.10
    rs = observe(step_ground, path, resolution=res)
    assert rs['discontinuity_type'] == 'vertical_step', ('阶面误判', rs)
    assert rs['ahead'] is True and abs(rs['step_m'] - 0.10) < 1e-6

    slope_ground = _flat_matrix()
    for c in range(20):
        for row in range(9):
            slope_ground[row][c] = 0.10 * c / 19.0
    rp = observe(slope_ground, path, resolution=res)
    assert rp['discontinuity_type'] == 'continuous_slope', ('斜坡误判', rp)
    assert rp['ahead'] is False and rp['slope_deg'] is not None
    print('F10 对照 OK：同总升高 0.10 m -> %s / %s（slope=%.2f deg）'
          % (rs['discontinuity_type'], rp['discontinuity_type'], rp['slope_deg']))

    # 4) 注入有效性：把阶面改成超过 max_step_m 的墙，必须翻成 blocked
    wall_ground = _flat_matrix()
    for c in range(10, 20):
        for row in range(9):
            wall_ground[row][c] = 0.45
    rw = observe(wall_ground, path, resolution=res, max_step_m=0.15)
    assert rw['discontinuity_type'] == 'blocked', ('注入有效性 FAIL', rw)
    print('注入有效性 OK：0.45 m 墙 -> blocked（超 max_step_m=0.15）')

    # 5) 反控：把墙撤回 0.10 m，必须恢复 vertical_step
    for c in range(10, 20):
        for row in range(9):
            wall_ground[row][c] = 0.10
    rr = observe(wall_ground, path, resolution=res, max_step_m=0.15)
    assert rr['discontinuity_type'] == 'vertical_step', ('反控 FAIL', rr)
    print('反控 OK：撤回注入后恢复 vertical_step')

    # 6) 影子对照只报不拦
    declared = {'ahead': True, 'step_m': 0.10, 'distance_m': 0.5}
    cmp_ok = shadow_compare(declared, rs)
    cmp_bad = shadow_compare(declared, rp)
    assert cmp_ok['consistent'] is True, ('影子对照误报', cmp_ok)
    assert 'declared_step_but_terrain_is_slope' in cmp_bad['disagreements']
    print('影子对照 OK：阶面一致 / 斜坡报分歧，且不写步态话题')
    print('SELFTEST PASS (terrain_observer)')
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(_selftest())
