#!/usr/bin/env python3
"""Declared-map obstacle cases on a MuJoCo step deck, initialized on its top.

This is torque/contact navigation on the step face, NOT a successful ascent.
The unchanged third-party robot XML is composed in a temporary wrapper. Both
the support map and obstacle input are read back from its compiled MuJoCo geoms.
No sensed-map, ROS, hardware, #151, or four-foot ascent acceptance is claimed.
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

from mj_step_baseline import rollout

PLATFORM = Path(__file__).resolve().parents[3] / '90_platform' / 'v8_platform'
sys.path.insert(0, str(PLATFORM / 'planner'))
import support_planner  # noqa: E402

DECK = 'step_obstacles_deck'
BOX = 'step_obstacles_box'
CASES = ('clear', 'offset_box', 'blocked_box')
RADIUS = .5
BODY_HEIGHT = .5
STEP_HEIGHT = .06
START = (.05, .05, STEP_HEIGHT)
GOAL = (3.55, .05, STEP_HEIGHT)
SCOPE = 'declared_geom_map_global_route_policy_torque_on_step_deck'


def wrapper_xml(xml, case):
    """Compose owned test geoms without rewriting a third-party asset."""
    if case not in CASES:
        raise ValueError('unknown step obstacle case')
    xml = Path(xml).resolve()
    root = ET.parse(xml).getroot()
    root.set('model', 'step_obstacles_' + case)
    compiler = root.find('compiler')
    if compiler is None:
        compiler = ET.SubElement(root, 'compiler')
    for key in ('meshdir', 'texturedir'):
        compiler.set(key, str((xml.parent / compiler.get(key, '')).resolve()))
    world = root.find('worldbody')
    if world is None:
        raise ValueError('robot worldbody is required')
    for geom in world.findall('geom'):
        if geom.get('type') != 'plane':
            raise ValueError('use the original flat robot XML, not an existing step world')
    ET.SubElement(world, 'geom', name=DECK, type='box', pos='2 0 .03',
                  size='3 1.6 .03', rgba='.4 .5 .6 1', friction='1 .01 .01')
    if case != 'clear':
        position = '1.8 .45 .36' if case == 'offset_box' else '1.8 0 .36'
        size = '.2 .35 .3' if case == 'offset_box' else '.2 1.6 .3'
        ET.SubElement(world, 'geom', name=BOX, type='box', pos=position, size=size,
                      rgba='.8 .2 .1 1', friction='1 .01 .01')
    return ET.tostring(root, encoding='unicode')


def geom_bounds(model, data, name):
    """Read a compiled static box as a world AABB; rotated boxes are conservative."""
    import mujoco
    index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if index < 0:
        raise ValueError('missing owned geom: ' + name)
    if model.geom_type[index] != mujoco.mjtGeom.mjGEOM_BOX or model.geom_bodyid[index] != 0:
        raise ValueError('owned fixture must be a static box')
    centre = np.asarray(data.geom_xpos[index])
    extent = np.abs(np.asarray(data.geom_xmat[index]).reshape(3, 3)) @ model.geom_size[index]
    return {'geom_name': name, 'min': (centre - extent).tolist(),
            'max': (centre + extent).tolist()}


def support_map(model, data, case):
    deck = geom_bounds(model, data, DECK)
    boxes = [] if case == 'clear' else [geom_bounds(model, data, BOX)]
    source = {'deck': deck, 'boxes': boxes}
    digest = hashlib.sha256(json.dumps(source, sort_keys=True).encode()).hexdigest()
    lo, hi = deck['min'], deck['max']
    footprint = {'min_x': lo[0] + RADIUS, 'max_x': hi[0] - RADIUS,
                 'min_y': lo[1] + RADIUS, 'max_y': hi[1] - RADIUS}
    value = json.loads((PLATFORM / 'contracts/fixtures/map_single_floor.json').read_text())
    value.update(schema_version=2, map_id='mj_step_deck_' + case, map_version=1,
                 resolution=.1, origin=[footprint['min_x'], footprint['min_y'], 0.],
                 extent={'width': round((footprint['max_x'] - footprint['min_x']) / .1),
                         'height': round((footprint['max_y'] - footprint['min_y']) / .1)},
                 source={'uri': 'fixture://compiled_mujoco_step_geoms', 'sha256': digest},
                 surfaces=[{'surface_id': 'deck', 'z_reference': hi[2],
                            'thickness_m': hi[2] - lo[2], 'representation': 'elevation_grid',
                            'data_uri': 'fixture://mujoco_geom/' + DECK,
                            'traversability': 'supported', 'clearance_min_m': 2.,
                            'footprint': footprint, 'edges': []}],
                 obstacles=[{'obstacle_id': item['geom_name'], 'min': item['min'],
                             'max': item['max']} for item in boxes])
    return value, source


class RouteController:
    """Track a precomputed global route with body vx/vy commands and yaw hold."""
    def __init__(self, plan):
        self.plan = plan
        self.points = np.asarray([p[:2] for p in plan.get('path', [])], dtype=float)
        if plan.get('ok') and (self.points.ndim != 2 or self.points.shape[1:] != (2,)
                               or not len(self.points) or not np.all(np.isfinite(self.points))):
            raise ValueError('successful global route must contain finite XY points')
        self.index, self.arrived = 0, False
        self.decisions = []

    def __call__(self, model, data):
        pose = np.asarray(data.qpos[:2])
        command = np.zeros(3)
        if self.plan.get('ok'):
            while self.index < len(self.points) - 1 and np.linalg.norm(self.points[self.index] - pose) < .12:
                self.index += 1
            delta = self.points[self.index] - pose
            if self.index == len(self.points) - 1 and np.linalg.norm(delta) <= .12:
                self.arrived = True
            if not self.arrived:
                norm = float(np.linalg.norm(delta))
                world = delta / max(1., norm / .23)
                w, x, y, z = data.qpos[3:7]
                yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                c, s = math.cos(yaw), math.sin(yaw)
                command = np.array([c * world[0] + s * world[1],
                                    -s * world[0] + c * world[1],
                                    np.clip(-yaw, -.15, .15)])
        self.decisions.append({'time': float(data.time), 'command': command.tolist(),
                               'route_index': self.index, 'arrived': self.arrived})
        return command


class PhysicsObserver:
    def __init__(self, case):
        self.case, self.box_id, self.feet = case, None, None

    def __call__(self, model, data, snapshot):
        import mujoco
        if self.feet is None:
            self.feet = {leg: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, leg + '_FOOT')
                         for leg in ('FL', 'FR', 'HL', 'HR')}
            if any(index < 0 for index in self.feet.values()):
                raise ValueError('four-foot body identities are required')
            self.box_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, BOX)
        contacts = []
        for index in range(data.ncon):
            contact = data.contact[index]
            if self.box_id in (contact.geom1, contact.geom2) and self.box_id >= 0:
                other = contact.geom2 if contact.geom1 == self.box_id else contact.geom1
                if model.geom_bodyid[other] != 0:
                    contacts.append({'geom1': int(contact.geom1), 'geom2': int(contact.geom2),
                                     'distance': float(contact.dist)})
        return {'feet': {leg: snapshot.xpos[index].tolist() for leg, index in self.feet.items()},
                'box_contacts': contacts}


def finite_array(value, shape):
    """Preserve type evidence: strings and bools cannot masquerade as measurements."""
    try:
        raw = np.asarray(value, dtype=object)
        if raw.shape != shape or not all(
                isinstance(v, (int, float, np.integer, np.floating))
                and not isinstance(v, (bool, np.bool_)) for v in raw.flat):
            return None
        result = raw.astype(float)
        return result if np.all(np.isfinite(result)) else None
    except (ValueError, TypeError, OverflowError):
        return None


def validate_measurements(run, geometry, decisions, case):
    """Reject incomplete/nonfinite observations before any comparison can go green."""
    problems = []

    def check(value, shape, field, sample=None):
        result = finite_array(value, shape)
        if result is None:
            problems.append({'sample': sample, 'field': field,
                             'reason': 'missing, malformed, non-real or nonfinite measurement'})
        return result

    trace = run.get('physics_trace')
    if not isinstance(trace, list) or len(trace) < 2:
        problems.append({'field': 'physics_trace', 'reason': 'at least two physics samples required'})
        return problems
    if type(run.get('fell')) is not bool:
        problems.append({'field': 'fell', 'reason': 'must be a boolean observation'})
    previous = None
    for index, sample in enumerate(trace):
        if not isinstance(sample, dict):
            problems.append({'sample': index, 'field': 'sample', 'reason': 'must be an object'})
            continue
        stamp = check(sample.get('time'), (), 'time', index)
        if stamp is not None:
            if float(stamp) < 0 or (previous is not None and float(stamp) <= previous):
                problems.append({'sample': index, 'field': 'time', 'reason': 'must increase strictly from nonnegative time'})
            previous = float(stamp)
        for field, shape in (('xyz', (3,)), ('base_vxy', (2,)), ('qpos', (19,)),
                             ('command', (3,)), ('tau_max', ()), ('robot_aabb', (2, 3))):
            value = check(sample.get(field), shape, field, index)
            if field == 'robot_aabb' and value is not None and np.any(value[0] > value[1]):
                problems.append({'sample': index, 'field': field, 'reason': 'inverted collision bounds'})
        observation = sample.get('observation')
        if not isinstance(observation, dict):
            problems.append({'sample': index, 'field': 'observation', 'reason': 'must be an object'})
            continue
        feet = observation.get('feet')
        if not isinstance(feet, dict) or set(feet) != {'FL', 'FR', 'HL', 'HR'}:
            problems.append({'sample': index, 'field': 'feet', 'reason': 'exactly four identified feet required'})
        else:
            for leg, point in feet.items():
                check(point, (3,), 'feet.' + leg, index)
        contacts = observation.get('box_contacts')
        if not isinstance(contacts, list):
            problems.append({'sample': index, 'field': 'box_contacts', 'reason': 'must be an observed list'})
        else:
            for contact in contacts:
                if not isinstance(contact, dict):
                    problems.append({'sample': index, 'field': 'box_contacts', 'reason': 'malformed contact'})
                    continue
                check(contact.get('distance'), (), 'contact.distance', index)
                for field in ('geom1', 'geom2'):
                    identity = contact.get(field)
                    if type(identity) is not int or identity < 0:
                        problems.append({'sample': index, 'field': 'contact.' + field, 'reason': 'nonnegative geom identity required'})
    if not isinstance(geometry, dict) or not isinstance(geometry.get('deck'), dict) or not isinstance(geometry.get('boxes'), list):
        problems.append({'field': 'geometry', 'reason': 'deck and boxes must be observed'})
    else:
        if (case not in CASES or len(geometry['boxes']) != (0 if case == 'clear' else 1)):
            problems.append({'field': 'geometry.boxes', 'reason': 'owned obstacle geometry does not match case'})
        for index, box in enumerate([geometry['deck']] + geometry['boxes']):
            if not isinstance(box, dict):
                problems.append({'field': 'geometry', 'reason': 'malformed geom bounds'})
                continue
            lo = check(box.get('min'), (3,), 'geometry.min', index)
            hi = check(box.get('max'), (3,), 'geometry.max', index)
            if lo is not None and hi is not None and np.any(lo >= hi):
                problems.append({'field': 'geometry', 'reason': 'geom bounds must have positive volume'})
    if not isinstance(decisions, list) or not decisions:
        problems.append({'field': 'decisions', 'reason': 'empty controller history cannot establish zero commands'})
    else:
        previous = None
        for index, decision in enumerate(decisions):
            if not isinstance(decision, dict):
                problems.append({'field': 'decisions', 'sample': index, 'reason': 'must be an object'})
                continue
            check(decision.get('command'), (3,), 'decision.command', index)
            stamp = check(decision.get('time'), (), 'decision.time', index)
            if stamp is not None:
                if float(stamp) < 0 or (previous is not None and float(stamp) <= previous):
                    problems.append({'field': 'decision.time', 'sample': index, 'reason': 'must increase strictly'})
                previous = float(stamp)
    return problems


def evaluate_physics(run, plan, geometry, controller, case):
    """Independent body-bounds/contact evaluation; never adjusts the route or motion."""
    problems = validate_measurements(run, geometry, controller.decisions, case)
    if problems:
        trace = run.get('physics_trace') or []
        present = bool(trace) and not any(p['field'] in ('physics_trace', 'robot_aabb', 'observation') for p in problems)
        return {'verdict': 'FAIL', 'checks': {'physics_trace_present': present,
                                             'physics_measurements_valid': False},
                'initialized_on_deck': True, 'ascent_completed': False,
                'four_foot_ascent_acceptance': 'NOT_TESTED', 'reason': 'invalid_physics_measurements',
                'measurement_problems': problems}
    trace = run['physics_trace']
    boxes = geometry['boxes']
    penetrations, contacts, missing, envelope_violations, deck_departures = [], [], [], [], []
    deck_lo, deck_hi = geometry['deck']['min'], geometry['deck']['max']
    minimum_gap = math.inf
    for sample in trace:
        bounds = sample.get('robot_aabb')
        observation = sample.get('observation')
        if bounds is None or observation is None:
            missing.append(sample['time'])
            continue
        aabb = np.asarray(bounds)
        position = np.asarray(sample['xyz'])
        if np.any(aabb[0, :2] < np.asarray(deck_lo[:2]) - 1e-6) or np.any(
                aabb[1, :2] > np.asarray(deck_hi[:2]) + 1e-6):
            deck_departures.append(sample['time'])
        # Validate the planning body envelope against the actual collision geometry.
        if (np.any(aabb[0, :2] < position[:2] - RADIUS - 1e-6) or
                np.any(aabb[1, :2] > position[:2] + RADIUS + 1e-6) or
                aabb[0, 2] < STEP_HEIGHT - .005 or
                aabb[1, 2] > STEP_HEIGHT + BODY_HEIGHT + 1e-6):
            envelope_violations.append(sample['time'])
        if observation['box_contacts']:
            contacts.append({'time': sample['time'], 'contacts': observation['box_contacts']})
        for box in boxes:
            lo, hi = np.asarray(box['min']), np.asarray(box['max'])
            gap = float(np.linalg.norm(np.maximum(np.maximum(lo - aabb[1], aabb[0] - hi), 0.)))
            minimum_gap = min(minimum_gap, gap)
            if np.all(aabb[1] >= lo) and np.all(aabb[0] <= hi):
                penetrations.append({'time': sample['time'], 'robot_aabb': bounds})
    final = trace[-1]
    feet = final.get('observation', {}).get('feet', {})
    feet_on_deck = len(feet) == 4 and all(
        deck_lo[0] <= p[0] <= deck_hi[0] and deck_lo[1] <= p[1] <= deck_hi[1]
        and abs(p[2] - (deck_hi[2] + .022)) <= .03 for p in feet.values())
    displacement = math.dist(trace[0]['xyz'][:2], final['xyz'][:2])
    maximum_excursion = max(math.dist(trace[0]['xyz'][:2], s['xyz'][:2]) for s in trace)
    cumulative_distance = sum(math.dist(a['xyz'][:2], b['xyz'][:2]) for a, b in zip(trace, trace[1:]))
    nonzero = any(any(v != 0. for v in d['command']) for d in controller.decisions)
    actual_nonzero = any(any(v != 0. for v in s['command']) for s in trace)
    expected_planning = (plan.get('reason') == 'E_PLAN_NO_ROUTE' if case == 'blocked_box' else plan.get('ok') is True)
    goal_error = math.dist(final['xyz'][:2], GOAL[:2])
    moved_as_expected = (not nonzero and not actual_nonzero and maximum_excursion < .10
                         if case == 'blocked_box' else
                         nonzero and actual_nonzero and controller.arrived and goal_error <= .18 and displacement > 3.)
    checks = {'expected_planner_behavior': expected_planning,
              'physics_trace_present': bool(trace) and not missing,
              'physics_measurements_valid': True,
              'no_fall': not run['fell'] and min(s['xyz'][2] for s in trace) >= deck_hi[2] + .18,
              'actual_body_within_planning_envelope': not envelope_violations,
              'no_box_contact': not contacts, 'no_box_aabb_overlap': not penetrations,
              'actual_body_stays_over_deck': not deck_departures,
              'all_four_feet_end_on_deck': feet_on_deck,
              'motion_or_rejection_verified': moved_as_expected}
    return {'verdict': 'PASS' if all(checks.values()) else 'FAIL', 'checks': checks,
            'final_goal_error_m': goal_error, 'base_displacement_m': displacement,
            'maximum_xy_excursion_m': maximum_excursion, 'cumulative_xy_m': cumulative_distance,
            'controller_nonzero_command_observed': nonzero, 'physics_nonzero_command_observed': actual_nonzero,
            'minimum_geometry_gap_m': minimum_gap if math.isfinite(minimum_gap) else None,
            'contact_sample_count': len(contacts), 'aabb_overlap_sample_count': len(penetrations),
            'envelope_violation_sample_count': len(envelope_violations),
            'deck_departure_sample_count': len(deck_departures),
            'first_contact': contacts[:1], 'first_overlap': penetrations[:1],
            'world_feet_end': feet, 'initialized_on_deck': True,
            'ascent_completed': False, 'four_foot_ascent_acceptance': 'NOT_TESTED',
            'collision_scope': 'all recorded physics steps: actual collision-geom AABB and contacts'}


def run_case(xml, policy, case, seconds=24.):
    import mujoco
    wrapper = wrapper_xml(xml, case)
    with tempfile.TemporaryDirectory(prefix='mj-step-obstacles-') as directory:
        path = Path(directory) / 'world.xml'
        path.write_text(wrapper)
        model = mujoco.MjModel.from_xml_path(str(path))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        value, geometry = support_map(model, data, case)
        try:
            plan = support_planner.plan(value, START, GOAL, robot_radius_m=RADIUS,
                                        robot_height_m=BODY_HEIGHT)
        except support_planner.PlannerError as error:
            plan = {'ok': False, 'reason': error.code, 'detail': error.detail, 'path': []}
        controller = RouteController(plan)
        observer = PhysicsObserver(case)
        run = rollout(str(path), policy, seconds, controller=controller,
                      initial_xyz=[START[0], START[1], STEP_HEIGHT + .35], physics_observer=observer)
    result = evaluate_physics(run, plan, geometry, controller, case)
    result.update(case=case, scope=SCOPE, hardware=False, sensed_map=False,
                  formal_151_acceptance=False, calibration_changed=False,
                  support_map=value, compiled_fixture_geometry=geometry, global_plan=plan,
                  robot_envelope={'radius_m': RADIUS, 'height_m': BODY_HEIGHT,
                                  'xy_shape': 'conservative square',
                                  'ground_contact_penetration_tolerance_m': .005},
                  start=list(START), goal=list(GOAL), decisions=controller.decisions,
                  rollout=run, composed_xml_sha256=hashlib.sha256(wrapper.encode()).hexdigest(),
                  original_xml=str(Path(xml).resolve()),
                  original_xml_sha256=hashlib.sha256(Path(xml).read_bytes()).hexdigest(),
                  asset_hash_scope='input and composed XML bytes plus policy; referenced meshes excluded',
                  existing_ascent_baseline={'status': 'NOT_PASSED', 'reference': 'R855',
                                           'reason': 'rear-foot support/slide bottleneck not solved by these cases'})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xml', required=True)
    parser.add_argument('--policy', required=True)
    parser.add_argument('--case', choices=CASES + ('all',), default='all')
    parser.add_argument('--seconds', type=float, default=24.)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    cases = CASES if args.case == 'all' else (args.case,)
    results = [run_case(args.xml, args.policy, case, args.seconds) for case in cases]
    result = {'scope': SCOPE, 'hardware': False, 'initialized_on_deck': True,
              'ascent_completed': False, 'verdict': 'PASS' if all(r['verdict'] == 'PASS' for r in results) else 'FAIL',
              'cases': results}
    Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps({'verdict': result['verdict'], 'scope': SCOPE,
                      'cases': [{'case': r['case'], 'verdict': r['verdict'], 'checks': r['checks']}
                                for r in results]}, ensure_ascii=False))
    return 0 if result['verdict'] == 'PASS' else 6


if __name__ == '__main__':
    raise SystemExit(main())
