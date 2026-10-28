#!/usr/bin/env python3
"""Reversible MuJoCo force-control experiment; no hardware or acceptance authority.

World-frame contact force f is the ground reaction ON the robot.  Static balance is
M*qdd+bias=tau+J.T*f, hence joint feed-forward is bias-J.T*f, NOT bias+J.T*f.
The experiment adds body pose feedback, stance-foot world anchors and operational
space swing feedback.  Fixed natural frequencies determine gains from mass/inertia;
there is no kp/kd sweep.  XML changes, including a step and foot-only contact, exist
only in the loaded model.  The inherited 5 cm support gate is never relaxed.
"""
import argparse
from contextlib import contextmanager
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

LEGS = ('FL', 'FR', 'HL', 'HR')


def skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def wrench_matrix(points, com):
    """[sum f, sum (p-com) cross f], columns ordered contact XYZ."""
    return np.block([[np.eye(3) for _ in points],
                     [skew(np.asarray(point) - com) for point in points]])


def project_friction(forces, mu):
    """Euclidean projection onto fz>=0, ||fxy||<=mu*fz (flat step treads)."""
    result = np.asarray(forces, dtype=float).reshape(-1, 3).copy()
    for force in result:
        radius, normal = float(np.linalg.norm(force[:2])), float(force[2])
        if normal >= 0 and radius <= mu * normal:
            continue
        projected_normal = max(0., (mu * radius + normal) / (1 + mu * mu))
        force[:2] *= mu * projected_normal / max(radius, 1e-12)
        force[2] = projected_normal
    return result


def allocate_contact_forces(points, com, desired_wrench, mu=0.8, warm=None, iterations=180):
    """Small convex projected-gradient wrench fit; reports infeasibility as residual.

    No tensile force or implicit suction is available.  This is a force allocator,
    not a certificate of dynamic stability or of a completed stair traversal.
    """
    points, com = np.asarray(points, dtype=float), np.asarray(com, dtype=float)
    desired = np.asarray(desired_wrench, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3 or desired.shape != (6,):
        raise ValueError('at least three XYZ supports and a six-dimensional wrench required')
    if mu <= 0 or not all(np.all(np.isfinite(value)) for value in (points, com, desired)):
        raise ValueError('finite geometry/wrench and positive friction required')
    matrix = wrench_matrix(points, com)
    length = max(0.10, float(np.mean(np.linalg.norm(points - com, axis=1))))
    weights = np.diag([1., 1., 1., 1 / length, 1 / length, 1 / length])
    a, b = weights @ matrix, weights @ desired
    force = np.linalg.lstsq(a, b, rcond=None)[0].reshape(-1, 3) if warm is None else np.array(warm).reshape(-1, 3)
    force = project_friction(force, mu)
    step = 1 / (np.linalg.norm(a, 2) ** 2 + 1e-8)
    for _ in range(iterations):
        gradient = (a.T @ (a @ force.ravel() - b)).reshape(-1, 3)
        updated = project_friction(force - step * gradient, mu)
        if np.linalg.norm(updated - force) < 1e-8:
            force = updated
            break
        force = updated
    residual = matrix @ force.ravel() - desired
    return force, {'force_residual_N': float(np.linalg.norm(residual[:3])),
                   'moment_residual_Nm': float(np.linalg.norm(residual[3:])),
                   'min_normal_N': float(np.min(force[:, 2])),
                   'max_friction_ratio': float(np.max(np.linalg.norm(force[:, :2], axis=1)
                                                       / np.maximum(force[:, 2], 1e-9)))}


def reaction_feedforward(jacobian, reaction, bias):
    """Actuated coordinates only; negative sign follows static virtual work."""
    return np.asarray(bias) - np.asarray(jacobian).T @ np.asarray(reaction)


def triangle_margin(point, supports):
    point, supports = np.asarray(point)[:2], np.asarray(supports)[:, :2]
    if len(supports) != 3:
        raise ValueError('three supports required')
    distances = []
    for index in range(3):
        left, right, other = supports[index], supports[(index + 1) % 3], supports[(index + 2) % 3]
        edge = right - left
        normal = np.array([-edge[1], edge[0]]) / max(np.linalg.norm(edge), 1e-12)
        if normal @ (other - left) < 0:
            normal = -normal
        distances.append(float(normal @ (point - left)))
    return min(distances)


def load_model(xml_path, step_h=0., step_x=1.):
    """Build from an in-memory XML tree; never modify source XML or mesh assets."""
    import mujoco
    path = Path(xml_path).resolve()
    tree = ET.fromstring(path.read_text())
    compiler_paths = {'meshdir': path.parent, 'texturedir': path.parent}
    for compiler in tree.findall('compiler'):
        for key in ('meshdir', 'texturedir'):
            if compiler.get(key):
                compiler_paths[key] = (path.parent / compiler.get(key)).resolve()
                compiler.set(key, str(compiler_paths[key]))
    # XML-string compilation has no source directory.  The original Lite3 XML has
    # no meshdir and uses ./meshes/*.STL, so every external asset needs its origin.
    for tag, key in (('mesh', 'meshdir'), ('texture', 'texturedir')):
        for element in tree.iter(tag):
            if element.get('file'):
                element.set('file', str((compiler_paths[key] / element.get('file')).resolve()))
    for element in tree.iter('include'):
        element.set('file', str((path.parent / element.get('file')).resolve()))
    if step_h > 0:
        ET.SubElement(tree.find('worldbody'), 'geom', name='force_experiment_step', type='box',
                      pos='%g 0 %g' % (step_x + 2., step_h / 2.),
                      size='2 2 %g' % (step_h / 2.), friction='1 0.005 0.0001')
    model = mujoco.MjModel.from_xml_string(ET.tostring(tree, encoding='unicode'))
    for geom in range(model.ngeom):
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[geom]) or ''
        if body.endswith('_SHANK'):
            model.geom_contype[geom] = model.geom_conaffinity[geom] = 0
    return model


class ForceController:
    """Ground reaction feed-forward plus body/foot feedback for a fixed support set."""
    def __init__(self, model, data, swing=(), mu=0.8, frequency_hz=1.2):
        import mujoco
        import crawl_gait as crawl
        self.model, self.data, self.swing, self.mu = model, data, tuple(swing), mu
        self.support = [name for name in LEGS if name not in swing]
        self.ids = {name: crawl._ids(model, name)[1] for name in LEGS}
        self.torso = crawl._ids(model, 'FL')[2]
        self.anchors = {name: data.xpos[self.ids[name]].copy() for name in self.support}
        self.reference_quat = data.xquat[self.torso].copy()
        self.reference_rotation = crawl._rotmat(self.reference_quat)
        self.mass = float(np.sum(model.body_mass))
        self.weight = self.mass * abs(float(model.opt.gravity[2]))
        self.omega = 2 * math.pi * frequency_hz
        self.com_offset = self.reference_rotation.T @ (data.subtree_com[self.torso] - data.xpos[self.torso])
        com = data.subtree_com[self.torso]
        self.inertia = np.zeros((3, 3))
        for body in range(1, model.nbody):
            rotation = data.ximat[body].reshape(3, 3)
            offset = data.xipos[body] - com
            self.inertia += rotation @ np.diag(model.body_inertia[body]) @ rotation.T \
                + model.body_mass[body] * ((offset @ offset) * np.eye(3) - np.outer(offset, offset))
        self.scratch = mujoco.MjData(model)
        self.previous_body_goal = None
        self.previous_foot_goals = {}
        self.last_time = -1e6
        self.cached = np.zeros(12)
        self.warm = None
        self.logs = []
        self.saturations = 0
        if len(self.support) == 3:
            margin = triangle_margin(data.xpos[self.torso], list(self.anchors.values()))
            if margin < 0.05:
                raise ValueError('force experiment refuses lift: support margin %.6f < 0.05 m' % margin)

    def _jacobian(self, name):
        import mujoco
        jacobian = np.zeros((3, self.model.nv))
        rotation = np.zeros_like(jacobian)
        mujoco.mj_jacBody(self.model, self.data, jacobian, rotation, self.ids[name])
        return jacobian

    def _normal_forces(self):
        import mujoco
        loads = {name: 0. for name in LEGS}
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, index, force)
            world = contact.frame.reshape(3, 3).T @ force[:3]
            for geom in (contact.geom1, contact.geom2):
                name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                         self.model.geom_bodyid[geom]) or ''
                if name.endswith('_FOOT') and name[:2] in loads:
                    loads[name[:2]] += abs(float(world[2]))
        return loads

    def command(self, targets, **_ignored):
        import mujoco
        import crawl_gait as crawl
        model, data = self.model, self.data
        if data.time - self.last_time >= 0.01 - 1e-9:
            elapsed = max(0.01, data.time - self.last_time) if self.last_time > -1e5 else 0.01
            self.last_time = float(data.time)
            self.scratch.qpos[:] = data.qpos
            self.scratch.qpos[:3] = 0
            self.scratch.qpos[3:7] = self.reference_quat
            self.scratch.qpos[7:19] = targets
            mujoco.mj_forward(model, self.scratch)  # Separate data; no contact/IK state pollution.
            body_goal = np.mean([self.anchors[name] - self.scratch.xpos[self.ids[name]]
                                 for name in self.support], axis=0)
            com_goal = body_goal + self.reference_rotation @ self.com_offset
            velocity = np.zeros(6)
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, self.torso, velocity, 0)
            goal_velocity = np.zeros(3) if self.previous_body_goal is None else \
                (body_goal - self.previous_body_goal) / elapsed
            goal_velocity = np.clip(goal_velocity, -.25, .25)
            self.previous_body_goal = body_goal.copy()
            body_force = self.mass * (self.omega ** 2 * (com_goal - data.subtree_com[self.torso])
                                     + 2 * self.omega * (goal_velocity - velocity[3:]))
            body_force[2] += self.weight
            body_force[2] = np.clip(body_force[2], .5 * self.weight, 1.5 * self.weight)
            horizontal = np.linalg.norm(body_force[:2])
            if horizontal > .5 * self.mu * self.weight:
                body_force[:2] *= .5 * self.mu * self.weight / horizontal
            rotation = crawl._rotmat(data.xquat[self.torso])
            relative = self.reference_rotation @ rotation.T
            error = .5 * np.array([relative[2, 1] - relative[1, 2],
                                   relative[0, 2] - relative[2, 0], relative[1, 0] - relative[0, 1]])
            moment = self.inertia @ (self.omega ** 2 * error - 2 * self.omega * velocity[:3])
            points = np.array([data.xpos[self.ids[name]] for name in self.support])
            forces, diagnostics = allocate_contact_forces(points, data.subtree_com[self.torso],
                np.r_[body_force, moment], mu=self.mu, warm=self.warm, iterations=100)
            self.warm = forces.copy()
            torque = data.qfrc_bias[6:18].copy()
            span = max(.15, float(np.mean(np.linalg.norm(points[:, :2]
                                    - data.subtree_com[self.torso][:2], axis=1))))
            anchor_k = self.weight / span
            full_mass = np.zeros((model.nv, model.nv))
            mujoco.mj_fullM(model, full_mass, data.qM)
            for name, force in zip(self.support, forces):
                jacobian = self._jacobian(name)
                torque = reaction_feedforward(jacobian[:, 6:18], force, torque)
                foot_error = self.anchors[name] - data.xpos[self.ids[name]]
                foot_velocity = jacobian @ data.qvel
                anchor_force = anchor_k * foot_error - 2 * math.sqrt(anchor_k * .15) * foot_velocity
                norm = np.linalg.norm(anchor_force)
                if norm > .2 * self.weight:
                    anchor_force *= .2 * self.weight / norm
                torque += jacobian[:, 6:18].T @ anchor_force
            for name in self.swing:
                index = LEGS.index(name)
                dofs = slice(6 + 3 * index, 9 + 3 * index)
                jacobian = self._jacobian(name)
                leg_jacobian = jacobian[:, dofs]
                effective_inverse = leg_jacobian @ np.linalg.solve(full_mass[dofs, dofs], leg_jacobian.T)
                inertia = np.linalg.pinv(effective_inverse, rcond=1e-4)
                eigen, basis = np.linalg.eigh(inertia)
                inertia = basis @ np.diag(np.clip(eigen, .02, 2.0)) @ basis.T
                goal = body_goal + self.scratch.xpos[self.ids[name]]
                speed = np.zeros(3) if name not in self.previous_foot_goals else \
                    (goal - self.previous_foot_goals[name]) / elapsed
                speed = np.clip(speed, -.4, .4)
                self.previous_foot_goals[name] = goal.copy()
                swing_omega = 2 * math.pi * 2.5
                force = inertia @ (swing_omega ** 2 * (goal - data.xpos[self.ids[name]])
                                   + 2 * swing_omega * (speed - jacobian @ data.qvel))
                torque += jacobian[:, 6:18].T @ force
            self.cached = torque
            diagnostics.update(time_s=round(float(data.time), 3),
                               body_goal=body_goal.tolist(),
                               desired_fz={name: float(force[2]) for name, force in zip(self.support, forces)},
                               actual_fz=self._normal_forces())
            if len(self.support) == 3:
                diagnostics['support_margin_m'] = triangle_margin(data.xpos[self.torso], points)
            if not self.logs or data.time - self.logs[-1]['time_s'] >= .10 - 1e-8:
                self.logs.append(diagnostics)
        torque = self.cached - 1.0 * data.qvel[6:18]
        self.saturations += int(np.any(np.abs(torque) > 30.0))
        torque = np.clip(torque, -30.0, 30.0)
        if not np.all(np.isfinite(torque)):
            raise FloatingPointError('nonfinite force-control torque')
        data.qfrc_applied[6:18] = torque
        return torque

    def report(self):
        return {'support': self.support, 'swing': list(self.swing), 'natural_frequency_hz': self.omega / (2 * math.pi),
                'torque_limit_Nm': 30., 'saturated_steps': self.saturations,
                'feedforward_sign': 'bias - J_world.T @ ground_reaction', 'samples': self.logs}


def run_minimal(xml_path, mode='stand', leg='HR', backend='force', seconds=2.):
    import mujoco
    import crawl_gait as crawl
    model = load_model(xml_path)
    data = mujoco.MjData(model)
    data.qpos[2], data.qpos[7:19] = .35, crawl.DEF
    mujoco.mj_forward(model, data)
    target = crawl.DEF.copy()
    for _ in range(int(.6 / model.opt.timestep)):
        crawl.pd_hold(model, data, target, kp=150., kd=1.)
        mujoco.mj_step(model, data)
    foot_ids = {name: crawl._ids(model, name)[1] for name in LEGS}
    torso = crawl._ids(model, leg)[2]
    start = {name: data.xpos[foot_ids[name]].copy() for name in LEGS}
    zmin, max_slip, phases, force_reports = float(data.qpos[2]), 0., [], []

    def execute(name, destination, duration, swing=()):
        nonlocal target, zmin, max_slip
        begin = target.copy()
        controller = ForceController(model, data, swing=swing) if backend == 'force' else None
        anchors = {name: data.xpos[foot_ids[name]].copy() for name in LEGS if name not in swing}
        nsteps = max(1, int(duration / model.opt.timestep))
        for index in range(nsteps):
            u = min(1., (index + 1) / nsteps)
            smooth = 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5
            target = begin + smooth * (destination - begin)
            if controller:
                controller.command(target)
            else:
                gain = np.array([100. if name in swing else 150. for name in LEGS for _ in range(3)])
                crawl.pd_hold(model, data, target, kp=gain, kd=1.)
            mujoco.mj_step(model, data)
            zmin = min(zmin, float(data.qpos[2]))
            max_slip = max(max_slip, max(float(np.linalg.norm((data.xpos[foot_ids[name]] - anchor)[:2]))
                                        for name, anchor in anchors.items()))
            if data.qpos[2] < .18:
                break
        if controller:
            force_reports.append(controller.report())
        phases.append({'phase': name, 'z': float(data.qpos[2]), 'max_support_slip_m': max_slip})

    margin = None
    if mode == 'lift':
        supports = [start[name][:2] for name in LEGS if name != leg]
        lengths = [np.linalg.norm(supports[(i + 1) % 3] - supports[(i + 2) % 3]) for i in range(3)]
        incenter = sum(length * point for length, point in zip(lengths, supports)) / sum(lengths)
        shift = incenter - data.xpos[torso][:2]
        rotation = crawl._rotmat(data.xquat[torso])
        goal = target.copy()
        for index, name in enumerate(LEGS):
            relative = crawl.foot_body_pos(model, data, name) - rotation.T @ np.r_[shift, 0.]
            goal[3 * index:3 * index + 3] = crawl.solve_leg(model, data, name, relative)
        execute('shift_for_lift', goal, 1.)
        margin = triangle_margin(data.xpos[torso], [data.xpos[foot_ids[name]] for name in LEGS if name != leg])
        if margin < .05 or zmin < .18:
            return {'mode': mode, 'backend': backend, 'pass': False, 'reason': 'support_margin_gate',
                    'margin_m': margin, 'zmin': zmin, 'force': force_reports, 'phases': phases}
        goal = target.copy()
        index = LEGS.index(leg)
        relative = crawl.foot_body_pos(model, data, leg) + np.array([0., 0., .10])
        goal[3 * index:3 * index + 3] = crawl.solve_leg(model, data, leg, relative)
        lift_start = float(data.xpos[foot_ids[leg]][2])
        execute('lift', goal, seconds, swing=(leg,))
        rise = float(data.xpos[foot_ids[leg]][2]) - lift_start
    else:
        execute('stand', target, seconds)
        rise = 0.
    return {'mode': mode, 'backend': backend, 'leg': leg, 'margin_m': margin,
            'zmin': zmin, 'fell': zmin < .18, 'support_slip_max_m': max_slip,
            'swing_rise_m': rise, 'pass': zmin >= .18 and max_slip <= .02
                and (mode == 'stand' or rise >= .07),
            'force': force_reports, 'phases': phases}


@contextmanager
def rear_force_bridge(swing_pd=False, stance_force=False):
    """Swap only the rear swing bridge controller in this process; restore on exit."""
    import crawl_gait as crawl
    original_hold = crawl.pd_hold
    active = {'key': None, 'controller': None, 'reports': [],
              'swing_pd': swing_pd, 'stance_force': stance_force}

    def wrapped(model, data, targets, kp=40., kd=1.5, max_tau=30.):
        gains = np.asarray(kp)
        swing = tuple(LEGS[index] for index in range(4)
                      if gains.ndim and gains[3 * index] < np.max(gains) - 1e-6)
        front_height = min(float(data.xpos[crawl._ids(model, name)[1]][2]) for name in ('FL', 'FR'))
        rear_height = min(float(data.xpos[crawl._ids(model, name)[1]][2]) for name in ('HL', 'HR'))
        enabled = front_height - rear_height > .03 and (
            (len(swing) == 1 and swing[0] in ('HL', 'HR')) or (stance_force and not swing))
        if not enabled:
            if active['controller'] is not None:
                active['reports'].append(active['controller'].report())
            active['key'], active['controller'] = None, None
            return original_hold(model, data, targets, kp=kp, kd=kd, max_tau=max_tau)
        key = (id(model), id(data), swing)
        if key != active['key']:
            if active['controller'] is not None:
                active['reports'].append(active['controller'].report())
            active['key'] = key
            active['controller'] = ForceController(model, data, swing=swing)
        if not swing_pd or not swing:
            return active['controller'].command(targets)
        pd_torque = original_hold(model, data, targets, kp=kp, kd=kd, max_tau=max_tau).copy()
        torque = active['controller'].command(targets).copy()
        first = 3 * LEGS.index(swing[0])
        torque[first:first + 3] = pd_torque[first:first + 3]
        data.qfrc_applied[6:18] = torque
        return torque

    crawl.pd_hold = wrapped
    try:
        yield active
    finally:
        if active['controller'] is not None:
            active['reports'].append(active['controller'].report())
        crawl.pd_hold = original_hold


def run_stair(xml_path, backend='force', step_h=.06, step_x=1., start_x=.80):
    import mujoco
    import crawl_gait as crawl
    # crawl.run reads an XML path.  Replace only that loader within this serial process,
    # with a proxy that compiles our in-memory XML; never write an alternate XML file.
    original_model = mujoco.MjModel
    class ModelProxy:
        @staticmethod
        def from_xml_path(_path):
            return load_model(xml_path, step_h=step_h, step_x=step_x)
        from_xml_string = staticmethod(original_model.from_xml_string)
    kwargs = dict(xml=xml_path, start_x=start_x, kp=150., kd=1., swing_kp=100., swing_kd=1.,
                  lift_h=.10, swing_fwd=.04, drive_fwd=.05, margin_min=.05, step_swing=.14,
                  rear_margin=.05, rear_shift=.05, step_x=step_x, step_h=step_h,
                  climb_x=.10, climb_dz=.03, rear_rounds=4, rear_swing=.08,
                  climb2_x=.06, climb2_dz=.02)
    try:
        mujoco.MjModel = ModelProxy
        if backend == 'force':
            with rear_force_bridge() as experiment:
                result = crawl.run('stair', **kwargs)
            result['force_experiment'] = experiment['reports']
        else:
            result = crawl.run('stair', **kwargs)
    finally:
        mujoco.MjModel = original_model
    feet = result.get('world_feet_end', {})
    result['full_stair_completed'] = len(feet) == 4 and all(
        point[0] >= step_x + .02 and abs(point[2] - (step_h + .022)) <= .03
        for point in feet.values())
    result['pass'] = bool(result.get('pass') and result['full_stair_completed'])
    result['backend'] = backend
    return result


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=('stand', 'lift', 'stair'))
    parser.add_argument('--xml', required=True)
    parser.add_argument('--backend', choices=('force', 'pd', 'compare'), default='force')
    parser.add_argument('--leg', choices=LEGS, default='HR')
    parser.add_argument('--seconds', type=float, default=2.)
    parser.add_argument('--step-h', type=float, default=.06)
    parser.add_argument('--start-x', type=float, default=.80)
    parser.add_argument('--out', default='')
    args = parser.parse_args(argv)
    backends = ('pd', 'force') if args.backend == 'compare' else (args.backend,)
    results = []
    for backend in backends:
        try:
            result = run_stair(args.xml, backend, args.step_h, start_x=args.start_x) if args.mode == 'stair' else \
                run_minimal(args.xml, args.mode, args.leg, backend, args.seconds)
        except (ValueError, FloatingPointError) as error:
            result = {'mode': args.mode, 'backend': backend, 'pass': False,
                      'aborted': True, 'reason': str(error)}
        results.append(result)
    result = {'experimental': True, 'hardware_validated': False, 'results': results}
    text = json.dumps(result, ensure_ascii=False, indent=1)
    if args.out:
        Path(args.out).write_text(text + '\n')
    print(text)
    return 0 if all(item.get('pass') for item in results) else 6


if __name__ == '__main__':
    raise SystemExit(main())
