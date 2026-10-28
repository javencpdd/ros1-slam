#!/usr/bin/env python3
"""Straight-strip in-process pilot: ray -> global -> local -> latch -> policy torque.

No ROS, hardware, contact-layer transfer or formal #151 acceptance. The floor is
known/declared, not sensed. World +X guidance uses the current lateral coordinate
inside the global route's straight corridor; local XY is rotated into body XY,
never projected onto X, and nonzero local Z is rejected.
Ray bounds describe visible surfaces plus uncertainty, not unseen ground-truth size.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

PLATFORM = Path(__file__).resolve().parents[3] / '90_platform' / 'v8_platform'
for directory in ('planner', 'representation', 'local_planner'):
    sys.path.insert(0, str(PLATFORM / directory))
import support_planner
import local_native as local
from mj_step_baseline import analyze_braking, rollout

SCOPE = 'inprocess_ray_global_local_policy_torque_full_chain_pilot'
BARRIER = 'navigation_pilot_barrier'


def support_fixture():
    surface = {'surface_id': 'floor', 'z_reference': 0., 'data_uri': 'fixture://known_flat_strip',
               'footprint': {'min_x': -1., 'max_x': 4., 'min_y': -.4, 'max_y': .4},
               'thickness_m': .05, 'edges': [], 'representation': 'elevation_grid',
               'traversability': 'supported', 'clearance_min_m': .4}
    source = {'uri': 'fixture://declared_flat_floor_z0_x[-1,4]_y[-.4,.4]',
              'sha256': hashlib.sha256(b'known flat floor z=0 x=[-1,4] y=[-.4,.4]').hexdigest()}
    value = json.loads((PLATFORM / 'contracts/fixtures/map_single_floor.json').read_text())
    value.update(map_id='mj_known_flat_strip', resolution=.2, origin=[-1., -.4, 0.],
                 extent={'width': 25, 'height': 4}, surfaces=[surface], source=source,
                 multiple_surfaces_per_xy=False)
    support_planner.contract.validate('map', value)
    return value


def wrapper_xml(xml):
    """Absolute include; original compiler asset roots survive the temporary location."""
    xml = Path(xml).resolve()
    root = ET.parse(xml).getroot()
    root.set('model', 'navigation_stop_pilot')
    compiler = root.find('compiler')
    if compiler is None:
        compiler = ET.SubElement(root, 'compiler')
    for key in ('meshdir', 'texturedir'):
        compiler.set(key, str((xml.parent / compiler.get(key, '')).resolve()))
    world = root.find('worldbody')
    if world is None:
        world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'geom', name=BARRIER, type='box', pos='2.0 0 0.5',
                  size='0.10 1.0 0.5', rgba='0.8 0.2 0.1 1')
    return ET.tostring(root, encoding='unicode')


class BarrierInjection:
    """The geom id is used only to hide/spawn the owned fixture, never for sensing."""
    def __init__(self, at):
        self.at, self.geom, self.spawn_time = at, None, None
        self.events = []

    def __call__(self, model, data):
        import mujoco
        if self.geom is None:
            self.geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, BARRIER)
            if self.geom < 0:
                raise ValueError('owned barrier missing')
            model.geom_pos[self.geom, 2] = -10.
            self.events.append({'time': float(data.time), 'event': 'barrier_hidden'})
            mujoco.mj_forward(model, data)
        if self.spawn_time is None and data.time >= self.at:
            model.geom_pos[self.geom, 2] = .5
            self.spawn_time = float(data.time)
            self.events.append({'time': self.spawn_time, 'event': 'barrier_spawn'})
            mujoco.mj_forward(model, data)


def ray_scan(model, data):
    """Nine horizontal front origins span the known strip; accept any geom hit."""
    import mujoco
    points, rays = [], []
    direction = np.array([1., 0., 0.])
    for lateral in np.linspace(-.4, .4, 9):
        origin = np.array([data.qpos[0] + .45, data.qpos[1] + lateral, data.qpos[2]])
        geom = np.array([-1], dtype=np.int32)
        distance = float(mujoco.mj_ray(model, data, origin, direction, None, 1, -1, geom))
        valid = math.isfinite(distance) and (distance < 0 or geom[0] >= 0)
        hit = valid and 0 <= distance <= 3.
        if hit:
            points.append((origin + distance * direction).tolist())
        rays.append({'origin': origin.tolist(), 'distance': distance,
                     'valid': bool(valid), 'hit': bool(hit)})
    return {'stamp': float(data.time), 'xyz': points, 'rays': rays,
            'ok': all(ray['valid'] for ray in rays)}


def sensed_obstacles(scan):
    points = np.asarray(scan['xyz'], dtype=float)
    if not points.size:
        return []
    if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
        raise ValueError('non-finite or malformed sensing')
    # Explicit floor support; visible horizontal samples do not establish box depth.
    lo, hi = points.min(axis=0) - .12, points.max(axis=0) + .12
    lo[2] = 0.
    return [{'min': lo.tolist(), 'max': hi.tolist(), 'stamp': scan['stamp'],
             'velocity': [0., 0., 0.]}]


class NavigationController:
    def __init__(self, sensor=ray_scan, injection=None):
        self.sensor, self.injection = sensor, injection
        self.map = support_fixture()
        self.config = local.Config(max_speed_mps=.3, resolution=.1)
        self.result, self.obstacles, self.scan = None, [], None
        self.last_tick, self.last_scan = None, -math.inf
        self.latch, self.first_zero, self.previous_command = None, None, np.zeros(3)
        self.clouds, self.global_plans, self.local_plans, self.decisions = [], [], [], []
        self.faults = []

    def latch_zero(self, now, reason):
        if reason in ('watchdog', 'missing_sensing', 'sensor_watchdog', 'carrier_heading'):
            self.faults.append({'time': now, 'reason': reason})
        if self.latch is None:
            self.latch = {'time': now, 'reason': reason}
        return np.zeros(3)

    def replan(self, pose, now):
        try:
            route = support_planner.plan(self.map, pose, (3.5, pose[1], 0.))
            self.global_plans.append(dict(route, stamp=now))
            ahead = [p for p in route['path'] if p[0] >= pose[0] + .8]
            # Grid centres need not coincide with body y. This declared straight
            # corridor guides forward, but every local XY correction is executed.
            goal = (ahead[0][0] if ahead else 3.5, pose[1], 0.)
        except support_planner.PlannerError as error:
            self.global_plans.append({'stamp': now, 'ok': False, 'reason': error.code})
            self.result = None
            self.latch_zero(now, error.code)
            return
        if (self.result and now < self.result['valid_until'] and
                math.dist(pose, self.result['trajectory'][1][:3]) > .04):
            return  # retain the committed first segment, not a new search result
        try:
            self.result = local.plan(self.map, pose, goal, self.obstacles, now, self.config)
            self.local_plans.append(dict(self.result, stamp=now, goal=list(goal)))
            if not self.result.get('ok'):
                self.latch_zero(now, self.result.get('reason', 'E_LOCAL_NO_TRAJECTORY'))
        except local.LocalError as error:
            self.result = None
            self.local_plans.append({'stamp': now, 'ok': False, 'reason': error.code})
            self.latch_zero(now, error.code)

    def __call__(self, model, data):
        now = float(data.time)
        pose = (float(data.qpos[0]), float(data.qpos[1]), 0.)  # declared flat support, not base z
        w, x, y, z = data.qpos[3:7]
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        if not math.isfinite(yaw) or abs(yaw) > .15:
            self.latch_zero(now, 'carrier_heading')
        if self.injection is not None:
            self.injection(model, data)
        if self.last_tick is not None and (now < self.last_tick or now - self.last_tick > .4):
            self.latch_zero(now, 'watchdog')
        self.last_tick = now
        refresh = now - self.last_scan >= .2 - 1e-9
        if refresh:
            self.last_scan = now
            try:
                self.scan = self.sensor(model, data)
                if (not self.scan or not self.scan.get('ok') or
                        len(self.scan.get('rays', [])) != 9 or
                        not all(r.get('valid') for r in self.scan['rays']) or
                        not math.isfinite(self.scan['stamp']) or
                        not 0 <= now - self.scan['stamp'] <= .4):
                    raise ValueError('missing or stale sensing')
                self.clouds.append(self.scan)
                obstacles = sensed_obstacles(self.scan)
                # A repeated cloud's insignificant float jitter cannot preserve a
                # revoked trajectory. Replan on obstacle input at each sensor tick.
                if obstacles or self.obstacles:
                    self.result = None
                self.obstacles = obstacles
            except (ValueError, TypeError, KeyError) as error:
                self.clouds.append({'stamp': now, 'ok': False, 'error': str(error)})
                self.latch_zero(now, 'missing_sensing')
        if self.scan is None or now - self.scan.get('stamp', -math.inf) > .4:
            self.latch_zero(now, 'sensor_watchdog')
        if refresh and self.latch is None:
            self.replan(pose, now)
        if self.result and (now >= self.result['valid_until'] or
                            math.dist(pose, self.result['trajectory'][1][:3]) <= .04):
            self.result = None
        if self.result is None and self.latch is None:
            self.replan(pose, now)
        command, reason, incoming = np.zeros(3), self.latch['reason'] if self.latch else 'no_trajectory', None
        if self.latch is None and self.result:
            # track_segment analytically checks the ENTIRE remaining first segment
            # at each policy tick with actual activation times, not just endpoints.
            tracked = local.track_segment(self.map, self.result, pose, now, self.obstacles,
                                          self.config, control_period_s=.02)
            incoming, reason = tracked['velocity'], tracked['reason']
            try:
                world = np.asarray(incoming, dtype=float)
                supported = (world.shape == (3,) and np.all(np.isfinite(world)) and
                             world[2] == 0. and np.linalg.norm(world) <= .4)
            except (TypeError, ValueError, OverflowError):
                supported = False
            if not supported:
                reason = 'unsupported_axis_or_velocity'
                self.latch_zero(now, reason)
            elif reason == 'E_LOCAL_COLLISION' or (tracked['stop'] and not tracked.get('complete') and reason != 'hold'):
                self.latch_zero(now, reason)
            else:
                c, s = math.cos(yaw), math.sin(yaw)
                command = np.array([c * world[0] + s * world[1],
                                    -s * world[0] + c * world[1],
                                    np.clip(-yaw, -.15, .15)])
        if self.latch is not None:
            command, reason = np.zeros(3), self.latch['reason']
        if self.first_zero is None and np.any(self.previous_command != 0.) and np.all(command == 0.):
            self.first_zero = {'time': now, 'reason': reason}
        self.decisions.append({'stamp': now, 'vx': float(command[0]), 'body_command': command.tolist(),
                               'reason': reason, 'local_xyz': incoming})
        self.previous_command = command.copy()
        return command


def collision_evaluation(samples, wrapper, spawn):
    """Independent truth sweep of collision-geometry bounds, never planner input."""
    geom = ET.fromstring(wrapper).find(".//geom[@name='navigation_pilot_barrier']")
    centre = np.array([float(v) for v in geom.get('pos').split()])
    half = np.array([float(v) for v in geom.get('size').split()])
    collisions, minimum_gap = [], math.inf
    if any(s.get('robot_aabb') is None for s in samples):
        return {'ok': False, 'reason': 'geometry_missing', 'collisions': []}
    if spawn is not None:
        lo, hi = centre - half, centre + half
        for a, b in zip(samples, samples[1:]):
            box_a, box_b = np.asarray(a['robot_aabb']), np.asarray(b['robot_aabb'])
            count = max(1, math.ceil(float(np.max(np.abs(box_b - box_a))) / .01),
                        math.ceil((b['time'] - a['time']) / .01))
            for u in np.linspace(0., 1., count + 1):
                stamp = a['time'] + u * (b['time'] - a['time'])
                if stamp < spawn:
                    continue
                box = box_a + u * (box_b - box_a)
                minimum_gap = min(minimum_gap, float(lo[0] - box[1, 0]))
                if np.all(box[1] >= lo) and np.all(box[0] <= hi):
                    collisions.append({'stamp': stamp, 'robot_aabb': box.tolist()})
                    break
    return {'ok': not collisions, 'collisions': collisions,
            'min_forward_geometry_gap_m': minimum_gap if math.isfinite(minimum_gap) else None,
            'scope': 'sampled collision-geometry AABB sweep; no fixed centre-radius substitute'}


def report(run, controller, wrapper):
    spawn = controller.injection.spawn_time if controller.injection else None
    first = controller.first_zero
    braking = analyze_braking(run['physics_trace'], first['time'] if first else None)
    injection_measurement = analyze_braking(run['physics_trace'], spawn)
    for measurement, semantics in ((braking, 'planner_first_nonzero_to_zero'),
                                   (injection_measurement, 'barrier_spawn_not_command_trigger')):
        measurement['measurement_origin'] = semantics
        if measurement['completed']:
            measurement['stop_time_s'] = measurement['confirmation_time'] - measurement['trigger_time']
    before = lambda p: spawn is not None and p['stamp'] < spawn
    global_before = any(p.get('path') and before(p) for p in controller.global_plans)
    local_before = any(p.get('trajectory') and before(p) for p in controller.local_plans)
    hits = any(c.get('xyz') and c['stamp'] >= spawn for c in controller.clouds) if spawn is not None else False
    no_route = any(not p['ok'] and p.get('reason') in ('E_LOCAL_NO_ROUTE', 'E_PLAN_NO_ROUTE')
                   and spawn is not None and p['stamp'] >= spawn
                   for p in controller.local_plans + controller.global_plans)
    collision = collision_evaluation(run['physics_trace'], wrapper, spawn)
    checks = {'moving_observed': bool(braking.get('pre_trigger_moving')),
              'sensor_hits': hits, 'global_route_before': global_before,
              'forward_command_before': any(d['vx'] > 0. and spawn is not None and d['stamp'] < spawn
                                            for d in controller.decisions),
              'sensed_planner_zero': bool(first and spawn is not None and first['time'] >= spawn and
                  first['reason'] in ('E_LOCAL_NO_ROUTE', 'E_PLAN_NO_ROUTE', 'E_LOCAL_COLLISION') and
                  any(c.get('xyz') and spawn <= c['stamp'] <= first['time'] for c in controller.clouds)),
              'local_trajectory_before': local_before, 'planner_blocked_no_route': no_route,
              'latched_stop': controller.latch is not None, 'no_fall': not run['fell'],
              'collision_free': collision['ok'], 'braking_completed': braking['completed'],
              'supported_execution': not any(d['reason'] == 'unsupported_axis_or_velocity'
                                             for d in controller.decisions),
              'sensing_watchdog_ok': not controller.faults and all(c.get('ok') for c in controller.clouds)
                  and not any(d['reason'] in ('watchdog', 'missing_sensing', 'sensor_watchdog', 'carrier_heading')
                              for d in controller.decisions)}
    verdict = 'PASS' if all(checks.values()) else 'FAIL'
    if (not any(any(v != 0. for v in d['body_command']) for d in controller.decisions) and
            all(checks[k] for k in ('no_fall', 'collision_free', 'supported_execution', 'sensing_watchdog_ok'))):
        verdict = 'INVALID'
    return {'scope': SCOPE, 'no_ros': True, 'hardware': False, 'budget_frozen': False,
            'formal_151_acceptance': False, 'verdict': verdict, 'checks': checks,
            'known_support_map': controller.map, 'global_plans': controller.global_plans,
            'local_plans': controller.local_plans, 'raw_sensor': controller.clouds,
            'decisions': controller.decisions, 'faults': controller.faults,
            'latch': controller.latch, 'planner_first_zero': first,
            'events': controller.injection.events if controller.injection else [],
            'planner_braking': braking, 'injection_to_stop': injection_measurement,
            'truth_collision_evaluation': collision, 'rollout': run}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xml', required=True)
    parser.add_argument('--policy', required=True)
    parser.add_argument('--seconds', type=float, default=15.)
    parser.add_argument('--inject-at', type=float, default=5.)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.seconds, args.inject_at)) or not 0 < args.inject_at < args.seconds:
        parser.error('require finite 0 < inject-at < seconds')
    xml = wrapper_xml(args.xml)
    controller = NavigationController(injection=BarrierInjection(args.inject_at))
    with tempfile.TemporaryDirectory(prefix='mj-navigation-') as directory:
        path = Path(directory) / 'pilot.xml'
        path.write_text(xml)
        run = rollout(str(path), args.policy, args.seconds, controller=controller)
    result = report(run, controller, xml)
    result.update(original_xml=str(Path(args.xml).resolve()),
                  original_model_sha256=hashlib.sha256(Path(args.xml).read_bytes()).hexdigest(),
                  model_hash_scope='original + wrapper XML bytes; referenced assets excluded')
    text = json.dumps(result, ensure_ascii=False, indent=1)
    Path(args.out).write_text(text + '\n')
    print(json.dumps({'verdict': result['verdict'], 'scope': SCOPE, 'checks': result['checks']}))
    return 0 if result['verdict'] == 'PASS' else 6


if __name__ == '__main__':
    raise SystemExit(main())
