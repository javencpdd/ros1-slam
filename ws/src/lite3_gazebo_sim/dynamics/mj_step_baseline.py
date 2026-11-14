#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V8-X1 首切片：MuJoCo 接触动力学基线与 10 cm 台阶尝试。

载入 DeepRobotics Lite3 MJCF（第三方原样，不入库）与 ppo 策略（onnx），用与
`/work/mj_stop.py` 相同的观测/动作口径（45 维 obs、50 Hz 策略、30 N·m 力矩限幅、
qfrc_applied 直接驱动）在「平地」与「0.1 m 台阶世界」各跑一次，记录：
机身轨迹、最低高度、是否跌落、是否越过台阶、接触数峰值与模型参数（时间步/摩擦）。
本件只读路径参数，不改任何第三方资产。

用法（容器内）：
  python3 mj_step_baseline.py \
    --xml /work/Lite3_rl_deploy-main/third_party/deep_robotics_model/Lite3/Lite3_mjcf/mjcf/Lite3.xml \
    --step-xml /work/Lite3_step10.xml \
    --policy /work/Lite3_rl_deploy-main/policy/ppo/policy.onnx \
    --out /work/v8x1_baseline.json
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


DEFAULT_POSE = np.array([0, -0.65, 1.3] * 4)


def rot(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def analyze_braking(samples, trigger_time, speed_limit=0.02,
                    xy_range_limit=0.01, hold_seconds=0.5):
    """Sampled component measurement, not frozen #151 acceptance criteria."""
    parameters = {'speed_limit_m_s': speed_limit, 'xy_range_limit_m': xy_range_limit,
                  'hold_seconds': hold_seconds, 'motion_epsilon_m': 1e-9,
                  'pre_motion_window_s': 0.5, 'base_height_floor_m': 0.18,
                  'post_confirmation_motion': 'reject through trace end',
                  'xy_range_definition': 'norm of XY axis peak-to-peak ranges',
                  'interpolation': 'piecewise linear; window starts at recorded samples',
                  'distance_interval': 'trigger through confirmation, or trace end',
                  'forward_axis': 'world +X', 'criteria_151_frozen': False}
    if not all(np.isfinite(x) and x > 0 for x in
               (speed_limit, xy_range_limit, hold_seconds)):
        raise ValueError('measurement parameters must be finite and positive')
    result = {'scope': 'torque_policy_braking_component', 'hardware': False,
              'full_chain': False, 'parameters': parameters, 'completed': False,
              'status': 'INVALID', 'reason': 'trigger_not_observed',
              'trigger_time': trigger_time, 'first_stationary_time': None,
              'confirmation_time': None, 'peak_forward_m': None,
              'cumulative_xy_m': None}
    if trigger_time is None or not samples:
        return result
    times = np.array([s['time'] for s in samples], dtype=float)
    xyz = np.array([s['xyz'] for s in samples], dtype=float)
    xy = xyz[:, :2]
    velocity = np.array([s['base_vxy'] for s in samples], dtype=float)
    if (not np.isfinite(trigger_time) or not np.all(np.isfinite(times)) or
            not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(velocity)) or
            np.any(np.diff(times) <= 0)):
        result['reason'] = 'invalid_samples'
        return result
    if not times[0] < trigger_time <= times[-1]:
        return result

    def interpolate(values, time):
        return np.array([np.interp(time, times, values[:, k]) for k in range(2)])

    origin = interpolate(xy, trigger_time)
    pre_start = max(times[0], trigger_time - parameters['pre_motion_window_s'])
    pre = (times > pre_start) & (times < trigger_time)
    pre_xy = np.vstack((interpolate(xy, pre_start), xy[pre], origin))
    moved = (np.linalg.norm(interpolate(velocity, trigger_time)) > speed_limit and
             np.linalg.norm(np.ptp(pre_xy, axis=0)) > parameters['motion_epsilon_m'])
    result['pre_trigger_moving'] = bool(moved)
    if not moved:
        result['reason'] = 'no_pre_trigger_motion'
        return result
    post_times = np.r_[trigger_time, times[times > trigger_time]]
    post_xy = np.vstack((origin, xy[times > trigger_time]))
    post_v = np.vstack((interpolate(velocity, trigger_time), velocity[times > trigger_time]))
    result.update(status='FAIL', reason='stop_not_completed')
    end = times[-1]
    for i, start in enumerate(post_times):
        confirmation = start + hold_seconds
        if confirmation > times[-1]:
            break
        inside = (post_times >= start) & (post_times < confirmation)
        window_xy = np.vstack((post_xy[inside], interpolate(xy, confirmation)))
        window_v = np.vstack((post_v[inside], interpolate(velocity, confirmation)))
        if (np.all(np.linalg.norm(window_v, axis=1) <= speed_limit) and
                np.linalg.norm(np.ptp(window_xy, axis=0)) <= xy_range_limit):
            result.update(completed=True, status='PASS', reason='stationary_window_observed',
                          first_stationary_time=float(post_times[i]),
                          confirmation_time=float(confirmation))
            end = confirmation
            break
    if result['completed']:
        stationary = post_times >= result['first_stationary_time']
        if (np.any(np.linalg.norm(post_v[stationary], axis=1) > speed_limit) or
                np.linalg.norm(np.ptp(post_xy[stationary], axis=0)) > xy_range_limit):
            result.update(completed=False, status='FAIL', reason='motion_resumed')
            end = times[-1]
    if np.any(xyz[:, 2] < parameters['base_height_floor_m']):
        result.update(completed=False, status='FAIL', reason='base_height_below_floor')
        end = times[-1]
    measured_xy = np.vstack((post_xy[post_times < end], interpolate(xy, end)))
    result.update(peak_forward_m=float(max(0.0, np.max(measured_xy[:, 0] - origin[0]))),
                  cumulative_xy_m=float(np.linalg.norm(np.diff(measured_xy, axis=0), axis=1).sum()),
                  measurement_end_time=float(end))
    return result


def _controller_command(value):
    """Accept legacy scalar vx or exactly three real body vx/vy/wz commands."""
    error = 'controller must return finite scalar vx or shape (3,) body vx/vy/wz in [-0.4, 0.4]'
    numeric = (int, float, np.integer, np.floating)
    if isinstance(value, numeric) and not isinstance(value, (bool, np.bool_)):
        value = (value, 0., 0.)
    elif not isinstance(value, (list, tuple, np.ndarray)):
        raise ValueError(error)
    try:
        values = np.asarray(value, dtype=object)  # preserve mixed-list bools before validation
        if (values.shape != (3,) or
                not all(isinstance(v, numeric) and not isinstance(v, (bool, np.bool_)) for v in values)):
            raise ValueError(error)
        command = values.astype(float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(error) from exc
    if not np.all(np.isfinite(command)) or np.any(np.abs(command) > .4):
        raise ValueError(error)
    return command


def rollout(xml, policy, seconds=10.0, v_cmd=0.3, stop_at=None, seed=0, controller=None,
            initial_xyz=None, physics_observer=None):
    """Run torque dynamics, with an optional read-only observation callback.

    ``initial_xyz`` is applied once, before the first physics step. The observer
    receives (model, dynamics_data, kinematics_snapshot) at every recorded step;
    its JSON-compatible return value is stored as ``observation``.
    """
    if controller is not None and (stop_at is not None or not callable(controller)):
        raise ValueError('controller must be callable and is mutually exclusive with stop_at')
    if physics_observer is not None and not callable(physics_observer):
        raise ValueError('physics_observer must be callable')
    if initial_xyz is None:
        initial_xyz = [0., 0., .35]
    raw_initial = np.asarray(initial_xyz, dtype=object)
    if (raw_initial.shape != (3,) or
            not all(isinstance(v, (int, float, np.integer, np.floating))
                    and not isinstance(v, (bool, np.bool_)) for v in raw_initial)):
        raise ValueError('initial_xyz must contain three finite real coordinates, z > 0')
    initial_xyz = raw_initial.astype(float)
    if not np.all(np.isfinite(initial_xyz)) or initial_xyz[2] <= 0:
        raise ValueError('initial_xyz must contain three finite real coordinates, z > 0')
    if stop_at is not None and not (np.isfinite(stop_at) and 0 < stop_at < seconds):
        raise ValueError('stop_at must be finite and satisfy 0 < stop_at < seconds')
    import mujoco
    import onnxruntime as ort

    ort.set_seed(seed)
    model_sha256 = hashlib.sha256(Path(xml).read_bytes()).hexdigest()
    policy_sha256 = hashlib.sha256(Path(policy).read_bytes()).hexdigest()
    sess = ort.InferenceSession(policy, providers=['CPUExecutionProvider'])
    inp = sess.get_inputs()[0].name
    model = mujoco.MjModel.from_xml_path(xml)
    data = mujoco.MjData(model)
    geometry_data = mujoco.MjData(model)
    robot_geoms = np.flatnonzero((model.geom_bodyid != 0) &
                                ((model.geom_contype != 0) | (model.geom_conaffinity != 0)))
    default_pose = DEFAULT_POSE
    scale = np.array([0.125, 0.25, 0.25] * 4)
    data.qpos[:3] = initial_xyz
    data.qpos[7:19] = default_pose
    mujoco.mj_forward(model, data)
    dt = model.opt.timestep
    stride = max(1, int(round(0.02 / dt)))
    last = np.zeros(12)
    target = default_pose.copy()
    steps = int(seconds / dt)
    zmin, xmax, ncon_peak = 9.0, -9.0, 0
    trace = []
    command = np.array([float(v_cmd), 0., 0.]) if controller is None else np.zeros(3)
    trigger_time = None
    trigger_boundary = None

    def sample():
        geometry_data.qpos[:] = data.qpos
        mujoco.mj_kinematics(model, geometry_data)
        bounds = None
        if len(robot_geoms):
            rotation = geometry_data.geom_xmat[robot_geoms].reshape(-1, 3, 3)
            local_box = model.geom_aabb[robot_geoms]
            centre = geometry_data.geom_xpos[robot_geoms] + np.einsum('nij,nj->ni', rotation, local_box[:, :3])
            extent = np.einsum('nij,nj->ni', np.abs(rotation), local_box[:, 3:])
            bounds = [(centre - extent).min(axis=0).tolist(),
                      (centre + extent).max(axis=0).tolist()]
        value = {'time': float(data.time), 'xyz': data.qpos[:3].tolist(),
                 'base_vxy': data.qvel[:2].tolist(), 'command': command.tolist(),
                 'robot_aabb': bounds, 'qpos': data.qpos.tolist(),
                 'tau_max': float(np.max(np.abs(data.qfrc_applied[6:18])))}
        if physics_observer is not None:
            value['observation'] = physics_observer(model, data, geometry_data)
        return value

    physics_trace = [sample()]
    for i in range(steps):
        triggering = False
        if i % stride == 0:
            if controller is not None:
                next_command = _controller_command(controller(model, data))
                if trigger_time is None and np.any(command != 0.) and np.all(next_command == 0.):
                    trigger_time = float(data.time)
                    trigger_boundary = {'before': sample()}
                    triggering = True
                command = next_command
            if stop_at is not None and trigger_time is None and data.time >= stop_at:
                trigger_time = float(data.time)
                trigger_boundary = {'before': sample()}
                command[:] = 0.
                triggering = True
            obs = np.concatenate([
                data.qvel[3:6] * 0.25,
                rot(data.qpos[3:7]).T @ np.array([0, 0, -1.0]),
                command / 0.8,
                data.qpos[7:19] - default_pose,
                data.qvel[6:18] * 0.05,
                last]).astype(np.float32)
            out = sess.run(None, {inp: obs.reshape(1, -1)})[0].reshape(-1)
            last = out.astype(np.float32)
            target = out * scale + default_pose
        tau = np.clip(30.0 * (target - data.qpos[7:19]) - 1.0 * data.qvel[6:18], -30, 30)
        data.qfrc_applied[6:18] = tau
        if triggering:
            trigger_boundary['after'] = sample()
            physics_trace[-1] = sample()
        mujoco.mj_step(model, data)
        physics_trace.append(sample())
        zmin = min(zmin, float(data.qpos[2]))
        xmax = max(xmax, float(data.qpos[0]))
        ncon_peak = max(ncon_peak, int(data.ncon))
        if i % int(round(0.5 / dt)) == 0:
            trace.append((round(i * dt, 1), round(float(data.qpos[0]), 3),
                          round(float(data.qpos[2]), 3)))
        if data.qpos[2] < 0.12:
            break
    return {
        'xml': str(xml), 'policy': str(policy),
        'model_sha256': model_sha256, 'policy_sha256': policy_sha256,
        'model_hash_scope': 'input XML bytes only; excludes referenced assets',
        'scope': 'torque_policy_braking_component', 'hardware': False, 'full_chain': False,
        'seed': seed, 'stop_at': stop_at, 'trigger_time': trigger_time,
        'initial_xyz': initial_xyz.tolist(),
        'trigger_semantics': ('first nonzero-to-zero controller policy tick' if controller is not None
                              else 'first scheduled policy tick at or after stop_at'),
        'trigger_boundary': trigger_boundary, 'physics_trace': physics_trace,
        'braking': analyze_braking(physics_trace, trigger_time)
                   if stop_at is not None or controller is not None else None,
        'seconds': seconds, 'v_cmd': v_cmd,
        'timestep': float(model.opt.timestep),
        'geom_friction_row0': [float(x) for x in model.geom_friction[0]],
        'nq': int(model.nq), 'nv': int(model.nv), 'nu': int(model.nu),
        'final_x': round(float(data.qpos[0]), 3),
        'final_z': round(float(data.qpos[2]), 3),
        'xmax': round(xmax, 3), 'zmin': round(zmin, 3),
        'fell': bool(zmin < 0.18),
        'ncon_peak': ncon_peak,
        'trace': trace,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--xml', default='/work/Lite3_rl_deploy-main/third_party/'
                                         'deep_robotics_model/Lite3/Lite3_mjcf/mjcf/Lite3.xml')
    parser.add_argument('--step-xml', default='/work/Lite3_step10.xml')
    parser.add_argument('--policy', default='/work/Lite3_rl_deploy-main/policy/ppo/policy.onnx')
    parser.add_argument('--seconds', type=float, default=10.0)
    parser.add_argument('--stop-at', type=float)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--flat-only', action='store_true')
    parser.add_argument('--out', default='')
    args = parser.parse_args()
    if args.stop_at is not None and not (np.isfinite(args.stop_at) and 0 < args.stop_at < args.seconds):
        parser.error('--stop-at must satisfy 0 < stop-at < seconds')
    flat = rollout(args.xml, args.policy, args.seconds, stop_at=args.stop_at, seed=args.seed)
    result = {'flat': flat,
              'scope': 'torque_policy_braking_component', 'hardware': False, 'full_chain': False,
              'verdict': 'PASS' if (not flat['fell']) else 'FAIL',
              'note': 'verdict 只判平地基线（策略不掉）；台阶是否通过看 step10.step_cleared'}
    if not args.flat_only:
        step = rollout(args.step_xml, args.policy, args.seconds, stop_at=args.stop_at, seed=args.seed)
        # 台阶前沿 x=1.0；越过判定：机身中心 x>1.6 且未跌落。
        step['step_front_x'] = 1.0
        step['step_cleared'] = bool((not step['fell']) and step['xmax'] > 1.6)
        result['step10'] = step
    if args.stop_at is not None:
        result['verdict'] = flat['braking']['status'] if not flat['fell'] else 'FAIL'
        result['note'] = 'PASS 仅表示制动分量的静止窗测得；不冻结 #151，不代全链或真机'
    text = json.dumps(result, ensure_ascii=False, indent=1)
    print(text)
    if args.out:
        Path(args.out).write_text(text + '\n')
    return 0 if result['verdict'] == 'PASS' else 6


if __name__ == '__main__':
    raise SystemExit(main())
