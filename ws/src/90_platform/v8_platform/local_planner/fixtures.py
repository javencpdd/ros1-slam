"""local_planner 的共享测试夹具（2026-11-18，M7 架构解耦）。

★为什么单独成文件★
这三个夹具原先定义在 `test_local_native.py`（测试文件）里，但引用它们的不只是测试：

    local_planner/runtime_probe.py    ← 实现文件
    local_planner/demo.py             ← 实现文件
    regression/a45_3d_test.py         ← 跨包

「实现 import 测试」是架构倒置：测试文件本应是最下游的消费者，却成了上游依赖源，
导致（a）删改测试会牵连实现，（b）本包测试无法整体移入 tests/ 子目录。
抽成中性模块后，实现与测试**都依赖 fixtures**，方向恢复正常。

这里的三个夹具保持与抽出前**逐字等价**（含 flat 的默认值与 layers 的 z=0.15 等常量），
任何数值漂移都会让既有反例失去意义。
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str((Path(__file__).resolve().parent / '../../..' / '80_algorithm/v8_algorithm/representation').resolve()))
import support_map  # noqa: E402  夹具里造双层支撑面要用它的构造器


def flat(ymin=-2.0, ymax=2.0):
    """单层平面场景：把 contract 夹具的 footprint 改成指定 y 范围。"""
    value = json.loads((HERE.parent / 'contracts/fixtures/map_single_floor.json').read_text())
    value['surfaces'][0]['footprint'].update(min_x=-2.0, max_x=3.0, min_y=ymin, max_y=ymax)
    return value


def layers():
    """双层支撑面（ground z=0 / deck z=0.15，带台阶边）——越层规划用。"""
    return support_map.build_support_map(
        'local_layers', .2, [-2., -2., 0.],
        [{'surface_id': 'ground', 'z_reference': 0.0, 'data_uri': 'fixture://ground',
          'footprint': {'min_x': -2., 'max_x': 0.2, 'min_y': -1., 'max_y': 1.},
          'edges': [{'to': 'deck', 'kind': 'step', 'max_step_height_m': .2, 'bidirectional': True}]},
         {'surface_id': 'deck', 'z_reference': .15, 'data_uri': 'fixture://deck',
          'footprint': {'min_x': 0., 'max_x': 3., 'min_y': -1., 'max_y': 1.}, 'edges': []}],
        None, {'uri': 'fixture://local_layers', 'sha256': 'a' * 64})


BOX = {'id': 'box', 'min': [.6, -.2, -.05], 'max': [1.0, .2, .8]}
