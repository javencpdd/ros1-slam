"""Support-aware piecewise integration controls for the Gazebo carrier."""
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
PLATFORM = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(PLATFORM / 'representation'))
import carrier_kinematics


def scene_map():
    path = PLATFORM.parents[1] / '95_simulation/lite3_gazebo_sim/scenarios/two_floor_nav/maps/open.json'
    return json.loads(path.read_text())


def surface(surface_id, cells, footprint=None, z=0.):
    return {'surface_id': surface_id, 'z_reference': z, 'traversability': 'supported',
            'footprint': footprint or {'min_x': 0., 'max_x': 1.6,
                                       'min_y': 0., 'max_y': 1.6},
            'cells': cells}


class CarrierKinematicsTests(unittest.TestCase):
    def test_same_height_command_cannot_integrate_past_step_edge(self):
        value = scene_map()
        position = (-0.7752064741284673, 1.2000000000000002, 0.96)
        velocity = (-0.41322459785886934, 0., 0.)
        out = carrier_kinematics.piecewise_advance(
            position, velocity, 0.061, value['surfaces'], resolution=value['resolution'],
            snap_m=.03)
        self.assertAlmostEqual(out[0], -0.8, places=12)
        self.assertAlmostEqual(out[2], 0.96, places=12)

    def test_contiguous_cells_do_not_stop_at_internal_grid_line(self):
        value = scene_map()
        out = carrier_kinematics.piecewise_advance(
            (-0.6, 1.2, 0.96), (-0.6, 0., 0.), 0.05, value['surfaces'],
            resolution=value['resolution'], snap_m=.03)
        self.assertAlmostEqual(out[0], -0.63, places=12)

    def test_explicit_vertical_transfer_may_cross_xy_edge(self):
        value = scene_map()
        out = carrier_kinematics.piecewise_advance(
            (-0.4, 1.2, 0.96), (-0.557086, 0., 0.222834), 1.2,
            value['surfaces'], resolution=value['resolution'], snap_m=.03)
        self.assertLess(out[0], -0.8)
        self.assertGreater(out[2], 0.96)

    def test_sparse_next_cell_stops_at_current_cell_edge(self):
        cells = {'encoding': 'rle', 'runs': [[0, 0, 1], [1, 1, 1]]}
        out = carrier_kinematics.piecewise_advance(
            (0.3, 0.2, 0.), (1., 0., 0.), 1., [surface('sparse', cells)],
            resolution=0.4, snap_m=.03)
        self.assertAlmostEqual(out[0], 0.4, places=12)

    def test_contiguous_cell_run_allows_full_internal_crossing(self):
        cells = {'encoding': 'rle', 'runs': [[0, 0, 2]]}
        out = carrier_kinematics.piecewise_advance(
            (0.3, 0.2, 0.), (1., 0., 0.), 1., [surface('run', cells)],
            resolution=0.4, snap_m=.03)
        self.assertAlmostEqual(out[0], 0.8, places=12)
        out = carrier_kinematics.piecewise_advance(
            (0.4, 0.2, 0.), (1., 0., 0.), 1., [surface('run', cells)],
            resolution=0.4, snap_m=.03)
        self.assertAlmostEqual(out[0], 0.8, places=12)

    def test_exact_half_open_boundary_does_not_reintegrate_into_missing_cell(self):
        cells = {'encoding': 'rle', 'runs': [[0, 0, 1], [1, 1, 1]]}
        out = carrier_kinematics.piecewise_advance(
            (0.4, 0.2, 0.), (1., 0., 0.), 0.1, [surface('sparse', cells)],
            resolution=0.4, snap_m=.03)
        self.assertAlmostEqual(out[0], 0.4, places=12)
        out = carrier_kinematics.piecewise_advance(
            (0.4000000005, 0.2, 0.), (1., 0., 0.), 0.1,
            [surface('sparse', cells)], resolution=0.4, snap_m=.03)
        self.assertAlmostEqual(out[0], 0.4, places=12)

    def test_nonfinite_motion_input_holds_finite_position(self):
        cells = {'encoding': 'rle', 'runs': [[0, 0, 2]]}
        surfaces = [surface('run', cells)]
        for velocity in ((float('inf'), 0., 0.), (0., float('nan'), 0.)):
            with self.subTest(velocity=velocity):
                out = carrier_kinematics.piecewise_advance(
                    (0.3, 0.2, 0.), velocity, 1., surfaces, resolution=0.4, snap_m=.03)
                self.assertEqual(out, (0.3, 0.2, 0.))
        out = carrier_kinematics.piecewise_advance(
            (0.3, 0.2, 0.), (1., 0., 0.), float('inf'), surfaces,
            resolution=0.4, snap_m=.03)
        self.assertEqual(out, (0.3, 0.2, 0.))

    def test_cellless_surface_and_unknown_height_keep_original_integration(self):
        plain = surface('plain', None, footprint={'min_x': 0., 'max_x': .4,
                                                  'min_y': 0., 'max_y': .4})
        out = carrier_kinematics.piecewise_advance(
            (0.3, 0.2, 0.), (1., 0., 0.), 1., [plain], resolution=None, snap_m=.03)
        self.assertAlmostEqual(out[0], 0.4, places=12)
        out = carrier_kinematics.piecewise_advance(
            (0.3, 0.2, 1.), (1., 0., 0.), 1., [plain], resolution=None, snap_m=.03)
        self.assertAlmostEqual(out[0], 1.3, places=12)

    def test_tied_candidates_on_shared_boundary_height_snap_without_dict_compare(self):
        value = scene_map()
        # 楼梯口 step10/upper_floor 同高 1.6；x=-4.0 是两者 cell 线，±1e-9 邻域
        # 使两个面同时入选、(dz, z_reference) 完全并列。真实探针窗在此 TypeError。
        for x in (-4.0, -4.0 + 5e-10, -4.0 - 5e-10):
            with self.subTest(x=x):
                out = carrier_kinematics.snap_position(
                    (x, 1.2, 1.6), (0.6, 0., 0.), value['surfaces'], snap_m=.03)
                self.assertAlmostEqual(out[2], 1.6, places=12)

    def test_same_height_cross_surface_walk_continues_onto_top_tread(self):
        value = scene_map()
        # 下楼入口：upper_floor(-4.0 边缘) 与 step10 同高 1.6 且 cells 连续，
        # 零 z 速度直行应走上踏面而不是被钉在 -4.0（真实探针窗死锁根因）。
        out = carrier_kinematics.piecewise_advance(
            (-4.0, 1.2, 1.6), (0.6, 0., 0.), 0.5, value['surfaces'],
            resolution=value['resolution'], snap_m=.03)
        self.assertGreater(out[0], -3.9)
        self.assertAlmostEqual(out[2], 1.6, places=12)

    def test_true_edge_without_same_height_successor_still_pins(self):
        value = scene_map()
        # step10 前缘 x=-3.2 之外是同高度真空区（step09 仅 1.44），
        # 无同高后继 cell，仍须在边界截断。
        out = carrier_kinematics.piecewise_advance(
            (-3.21, 1.2, 1.6), (0.6, 0., 0.), 0.5, value['surfaces'],
            resolution=value['resolution'], snap_m=.03)
        self.assertAlmostEqual(out[0], -3.2, places=12)
        self.assertAlmostEqual(out[2], 1.6, places=12)


class CarrierNodeSourceTests(unittest.TestCase):
    """静态守卫：节点内被引用的名字必须可解析（无需 ROS 运行主循环）。

    回归：真实 upstairs-r14 窗口因循环内残留旧变量名，首个 tick 的
    status 发布即 NameError，节点进程 0.7 s 内 atexit 退出。
    """

    def test_node_has_no_unbound_referenced_names(self):
        import builtins
        import symtable
        source = (HERE / 'gazebo_carrier3d_node.py').read_text()
        root = symtable.symtable(source, 'gazebo_carrier3d_node.py', 'exec')
        module_names = {symbol.get_name() for symbol in root.get_symbols()}
        errors = []

        def check(scope):
            for symbol in scope.get_symbols():
                if not symbol.is_referenced():
                    continue
                if (symbol.is_assigned() or symbol.is_parameter()
                        or symbol.is_free() or symbol.is_declared_global()):
                    continue
                name = symbol.get_name()
                if (name in module_names or name in dir(builtins)
                        or name.startswith('__')):
                    continue
                errors.append('%s: %s' % (scope.get_name(), name))
            for child in scope.get_children():
                check(child)

        check(root)
        self.assertEqual(errors, [])


if __name__ == '__main__':
    unittest.main()
