#!/usr/bin/env python3
"""Gait decision node: stair look-ahead -> /v8/chassis/gait.

Pure decision logic lives in GaitDecision (unit-tested); this wrapper only moves
ROS messages. It never publishes a velocity: the chassis bridge is the only
consumer of the gait topic and the safety mux stays the only /cmd_vel writer.
"""
import json

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

import stair_ahead


from gait_decision import GaitDecision


class GaitGuardNode(object):
    def __init__(self):
        self.enable = bool(rospy.get_param('~enable', True))
        self.enter_m = float(rospy.get_param('~enter_m', 1.0))
        self.exit_m = float(rospy.get_param('~exit_m', 1.5))
        self.min_step_m = float(rospy.get_param('~min_step_m', 0.05))
        self.max_step_m = float(rospy.get_param('~max_step_m', 0.15))
        self.lookahead_m = float(rospy.get_param('~lookahead_m', 1.5))
        self.rate_hz = float(rospy.get_param('~rate_hz', 5.0))
        self.map_topic = rospy.get_param('~map_topic', '/v8_support_map')
        self.path_topic = rospy.get_param('~path_topic', '/v8/3d/global/path')
        self.pose_topic = rospy.get_param('~pose_topic', '/v8/local/pose')
        self.gait_topic = rospy.get_param('~gait_topic', '/v8/chassis/gait')
        self.status_topic = rospy.get_param('~status_topic', '/v8/gait_guard/status')

        self.decision = GaitDecision(self.enter_m, self.exit_m, self.min_step_m,
                                     self.max_step_m,
                                     rospy.get_param('~flat_gait', 'flat_mid'),
                                     rospy.get_param('~step_gait', 'high_step'))
        self.map_value = None
        self.path = None
        self.pose = None
        self.published = None

        self.gait_pub = rospy.Publisher(self.gait_topic, String, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(self.map_topic, String, self.on_map, queue_size=1)
        rospy.Subscriber(self.path_topic, Path, self.on_path, queue_size=1)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.on_pose, queue_size=1)

        # ── ②感知接线 + ③门值机制（全部后置，默认逐字零回归）──────────────
        # 在册多处引用本件 :38-39「只发 flat_mid / high_step 两个步态名」，
        # 故上面 1—39 行不得改动；新逻辑一律追加在此之后。
        # 默认 gate_source=declared、perception_mode=shadow 时，
        # /v8/chassis/gait 的输出与改造前逐字一致（可用双版本对表证明）。
        import gait_gates
        import terrain_observer
        self._gait_gates = gait_gates
        self._terrain = terrain_observer

        # ③门值来源：declared（在册基线，默认）| contract（实测契约反算）。
        # 契约字段当前全为 null 且 status=TEMPLATE_ONLY，contract 会自动拒用
        # 并回退声明基线，不会凭感觉改写任何判据门值。
        self.gate_source = rospy.get_param('~gate_source', 'declared')
        self.gait_contract = rospy.get_param('~gait_contract', {})
        self.contract_gait = rospy.get_param('~contract_gait', 'flat_mid')
        self.gate_margin_m = float(rospy.get_param('~gate_margin_m', 0.0))
        declared = {
            'lookahead_m': self.lookahead_m,
            'enter_m': self.enter_m,
            'exit_m': self.exit_m,
            'min_step_m': self.min_step_m,
            'max_step_m': self.max_step_m,
        }
        self.gate_resolution = self._gait_gates.resolve_gates(
            declared, source=self.gate_source, contract=self.gait_contract,
            margin_m=self.gate_margin_m, gait=self.contract_gait)
        if self.gate_resolution['source'] == 'contract':
            gates = self.gate_resolution['gates']
            self.lookahead_m = gates['lookahead_m']
            self.enter_m = gates['enter_m']
            self.exit_m = gates['exit_m']
            self.min_step_m = gates['min_step_m']
            self.max_step_m = gates['max_step_m']
            self.decision = GaitDecision(
                self.enter_m, self.exit_m, self.min_step_m, self.max_step_m,
                self.decision.flat_gait, self.decision.step_gait)
            rospy.loginfo('[gait_guard] contract gates applied: %s'
                          % json.dumps(self.gate_resolution['applied']))
        elif self.gate_source == 'contract':
            rospy.logwarn('[gait_guard] contract gates refused, fallback to declared: %s'
                          % json.dumps(self.gate_resolution['refusals']))

        # ②感知接线：订阅高程 GridMap -> TerrainObserver 影子通道。
        # off | shadow（默认，只报不拦）| assist（感知只做否决、不驱动）。
        self.perception_mode = rospy.get_param('~perception_mode', 'shadow')
        self.elevation_topic = rospy.get_param(
            '~elevation_topic', '/elevation_mapping/elevation_map')
        self.terrain_topic = rospy.get_param('~terrain_topic',
                                             '/v8/gait_guard/terrain')
        self.terrain_resolution = float(rospy.get_param('~terrain_resolution', 0.05))
        self.terrain_lookahead_m = float(
            rospy.get_param('~terrain_lookahead_m', self.lookahead_m))
        self.terrain_half_width = int(rospy.get_param('~terrain_half_width', 5))
        self.terrain_pub = rospy.Publisher(self.terrain_topic, String,
                                           queue_size=1, latch=True)
        self.latest_terrain = None
        self._GridMap = None
        if self.perception_mode != 'off':
            try:
                from grid_map_msgs.msg import GridMap
                self._GridMap = GridMap
                rospy.Subscriber(self.elevation_topic, GridMap,
                                 self.on_elevation, queue_size=1)
                rospy.loginfo('[gait_guard] terrain observer wired: %s -> %s mode=%s'
                              % (self.elevation_topic, self.terrain_topic,
                                 self.perception_mode))
            except ImportError as exc:
                rospy.logwarn('[gait_guard] grid_map_msgs unavailable, '
                              'terrain observer disabled: %s' % exc)

    def on_map(self, msg):
        try:
            value = json.loads(msg.data)
        except ValueError:
            return
        self.map_value = value.get('map', value)

    def on_path(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
                     for p in msg.poses]

    def on_pose(self, msg):
        self.pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def on_elevation(self, msg):
        """GridMap -> 地形特征。只发影子诊断话题，不写 /v8/chassis/gait。"""
        try:
            layer = msg.data[0]
            n_rows = int(layer.layout.dim[1].size)
            n_cols = int(layer.layout.dim[0].size)
            matrix = self._terrain.gridmap_to_matrix(list(layer.data), n_rows, n_cols)
            info = msg.info
            center_x, center_y = info.pose.position.x, info.pose.position.y
            resolution = float(info.resolution or self.terrain_resolution)
            path_cells = []
            points = self.path or ([self.pose] if self.pose else [])
            for point in points:
                cell = self._terrain.world_to_cell(
                    point[0], point[1], center_x, center_y,
                    n_rows, n_cols, resolution)
                if cell:
                    path_cells.append(cell)
            if not path_cells:
                return
            features = self._terrain.observe(
                matrix, path_cells, resolution=resolution,
                lookahead_m=self.terrain_lookahead_m,
                min_step_m=self.min_step_m, max_step_m=self.max_step_m,
                half_width=self.terrain_half_width)
            features['header_frame'] = info.header.frame_id
            features['perception_mode'] = self.perception_mode
            self.latest_terrain = features
            self.terrain_pub.publish(String(data=json.dumps(features)))
        except (ValueError, IndexError, AttributeError, TypeError) as exc:
            rospy.logwarn('[gait_guard] terrain observer parse failed: %s' % exc)

    def step(self):
        if not self.enable or self.map_value is None or self.pose is None:
            return
        result = stair_ahead.stair_ahead(self.map_value, self.pose, self.path,
                                         lookahead_m=self.lookahead_m,
                                         min_step_m=self.min_step_m)
        decision_result = result
        compare = None
        if self.latest_terrain is not None and self.perception_mode != 'off':
            compare = self._terrain.shadow_compare(result, self.latest_terrain)
            # assist 档：感知只做否决、不做驱动。与声明地图分歧（斜坡/堵头）
            # 时保持平地步态，绝不把未验证的感知结果写成 high_step。
            if self.perception_mode == 'assist' and \
                    self.latest_terrain.get('discontinuity_type') in \
                    ('blocked', 'continuous_slope'):
                decision_result = dict(result)
                decision_result['ahead'] = False
        gait = self.decision.update(decision_result)
        if gait != self.published:
            self.published = gait
            self.gait_pub.publish(String(data=gait))
        status = {
            'gait': gait,
            'climbing': self.decision.climbing,
            'refusal': self.decision.refusal,
            'ahead': result.get('ahead', False),
            'step_m': result.get('step_m'),
            'distance_m': result.get('distance_m'),
            'gate_source': self.gate_resolution['source'],
            'contract_refusals': self.gate_resolution['refusals'],
            'perception_mode': self.perception_mode,
        }
        if compare is not None:
            status['terrain_type'] = compare['terrain_type']
            status['terrain_consistent'] = compare['consistent']
            status['terrain_disagreements'] = compare['disagreements']
        self.status_pub.publish(String(data=json.dumps(status)))

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('v8_gait_guard')
    GaitGuardNode().spin()
