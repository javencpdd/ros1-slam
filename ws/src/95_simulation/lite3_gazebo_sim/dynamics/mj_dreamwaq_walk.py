#!/usr/bin/env python3
"""DreamWaQ 270-state MuJoCo adapter: 45 current + 5x45 history -> 12 actions.

This is a deployment probe, not a real-robot controller and not a frozen gait
acceptance. It reads only the supplied XML/ONNX and writes JSON evidence.
"""
import argparse
import hashlib
import json
import os
import shutil
import tempfile
from pathlib import Path

import numpy as np

OBS_DIM = 45
HISTORY = 5
STATE_DIM = OBS_DIM * (1 + HISTORY)
ACTION_SCALE = np.array([0.125, 0.25, 0.25] * 4)
COMMAND_SCALE = np.array([2.0, 2.0, 0.25])
DEFAULT_POSE = np.array([0.0, -1.0, 1.8] * 4)
TORQUE_LIMIT = 30.0


def rot(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def observation(qpos, qvel, command, last_action, default_pose=DEFAULT_POSE):
    """DreamWaQ 45: body ang vel, body gravity, scaled command, joints, last action."""
    rotation = rot(qpos[3:7])
    value = np.concatenate([
        (rotation.T @ qvel[3:6]) * 0.25,
        rotation.T @ np.array([0., 0., -1.]),
        np.asarray(command, dtype=float) * COMMAND_SCALE,
        qpos[7:19] - default_pose,
        qvel[6:18] * 0.05,
        last_action]).astype(np.float32)
    if value.shape != (OBS_DIM,):
        raise ValueError('observation must be %d-dimensional' % OBS_DIM)
    return value


def state_vector(obs, history, newest_first=True):
    """270 = current obs + five 45-dim history frames (newest first by default)."""
    if len(history) != HISTORY or any(np.asarray(frame).shape != (OBS_DIM,) for frame in history):
        raise ValueError('history must contain %d frames of %d values' % (HISTORY, OBS_DIM))
    frames = list(history) if not newest_first else list(reversed(history))
    value = np.concatenate([obs] + [np.asarray(frame, dtype=np.float32) for frame in frames])
    if value.shape != (STATE_DIM,):
        raise ValueError('state must be %d-dimensional' % STATE_DIM)
    return value.astype(np.float32)


def joint_target(action, default_pose=DEFAULT_POSE, action_scale=ACTION_SCALE):
    return np.asarray(action, dtype=float) * action_scale + default_pose


def make_stairs_xml(flat_xml, step_h, steps=3, depth=.5, width=3.0, front=1.0):
    """Create a temporary 3-step world that includes the supplied flat model."""
    flat = Path(flat_xml).resolve()
    if steps < 1 or step_h <= 0 or depth <= 0 or width <= 0:
        raise ValueError('stairs need positive step height/depth/width and steps >= 1')
    work = Path(tempfile.mkdtemp(prefix='lite3_stairs_'))
    shutil.copy2(flat, work / flat.name)
    for entry in flat.parent.iterdir():
        if entry == flat:
            continue
        target = work / entry.name
        if target.exists():
            continue
        try:
            os.symlink(entry, target, target_is_directory=entry.is_dir())
        except OSError:
            if entry.is_dir():
                shutil.copytree(entry, target)
            else:
                shutil.copy2(entry, target)
    geoms = []
    for i in range(steps):
        z = step_h * (i + 1) / 2.
        x = front + depth * i + depth / 2.
        geoms.append('    <geom name="step%d" type="box" size="%g %g %g" pos="%g 0 %g" '
                     'friction="1 0.01 0.01" rgba="0.4 0.7 0.4 1"/>'
                     % (i + 1, depth / 2., width / 2., z, x, z))
    xml = ('<mujoco model="Lite3_stairs_auto">\n'
           '  <compiler meshdir="%s"/>\n'
           '  <include file="%s"/>\n'
           '  <worldbody>\n%s\n  </worldbody>\n</mujoco>\n'
           % (work, flat.name, '\n'.join(geoms)))
    path = work / 'stairs.xml'
    path.write_text(xml)
    return str(path)


def run_rollout(xml, policy, seconds, v_cmd, default_pose=DEFAULT_POSE,
                kp=25.0, kd=.5, initial_z=.40, history_newest_first=True,
                stair=None):
    import mujoco
    import onnxruntime as ort

    session = ort.InferenceSession(policy, providers=['CPUExecutionProvider'])
    input_name = session.get_inputs()[0].name
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    default_pose = np.asarray(default_pose, dtype=float)
    data.qpos[:3] = [0., 0., initial_z]
    data.qpos[3:7] = [1., 0., 0., 0.]
    data.qpos[7:19] = default_pose
    mujoco.mj_forward(model, data)
    dt = model.opt.timestep
    stride = max(1, int(round(.02 / dt)))
    history = [np.zeros(OBS_DIM, dtype=np.float32) for _ in range(HISTORY)]
    last_action = np.zeros(12, dtype=np.float32)
    target = default_pose.copy()
    command = np.array([v_cmd, 0., 0.], dtype=np.float32)
    trace = []
    marks = {}
    zmin, xmax, max_abs_y, fell, unstable = 9., -9., 0., False, False
    mark_x = []
    if stair:
        mark_x = [stair['front'] + stair['depth'] * (i + 1) for i in range(stair['steps'])]
        mark_x.append(mark_x[-1] + .2)
    for step in range(int(seconds / dt)):
        if step % stride == 0:
            obs = observation(data.qpos, data.qvel, command, last_action, default_pose)
            state = state_vector(obs, history, newest_first=history_newest_first)
            action = session.run(None, {input_name: state.reshape(1, -1)})[0].reshape(-1)
            last_action = action.astype(np.float32)
            target = joint_target(action, default_pose)
            history = history[1:] + [obs]
        tau = np.clip(kp * (target - data.qpos[7:19]) - kd * data.qvel[6:18],
                      -TORQUE_LIMIT, TORQUE_LIMIT)
        data.qfrc_applied[6:18] = tau
        mujoco.mj_step(model, data)
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            unstable = True
            break
        zmin = min(zmin, float(data.qpos[2]))
        xmax = max(xmax, float(data.qpos[0]))
        max_abs_y = max(max_abs_y, abs(float(data.qpos[1])))
        for x in mark_x:
            key = '%.3f' % x
            if key not in marks and data.qpos[0] > x:
                marks[key] = [round(float(data.time), 3), round(float(data.qpos[1]), 4),
                              round(float(data.qpos[2]), 4)]
        if step % max(1, int(round(.05 / dt))) == 0:
            trace.append([round(float(data.time), 3), round(float(data.qpos[0]), 4),
                          round(float(data.qpos[1]), 4), round(float(data.qpos[2]), 4),
                          round(float(np.max(np.abs(action))), 4), int(data.ncon)])
        if data.qpos[2] < .12:
            fell = True
            break
    final = [round(float(x), 4) for x in data.qpos[:3]]
    result = {
        'policy_sha256': hashlib.sha256(Path(policy).read_bytes()).hexdigest(),
        'xml_sha256': hashlib.sha256(Path(xml).read_bytes()).hexdigest(),
        'scope': 'mujoco_dreamwaq_270_stair_component',
        'hardware': False, 'full_chain': False, 'criteria_151_frozen': False,
        'policy': str(policy), 'xml': str(xml),
        'seconds': seconds, 'v_cmd': v_cmd, 'control_hz': 50,
        'default_pose': default_pose.tolist(), 'action_scale': ACTION_SCALE.tolist(),
        'command_scale': COMMAND_SCALE.tolist(), 'kp': kp, 'kd': kd,
        'torque_limit': TORQUE_LIMIT, 'history': HISTORY,
        'history_newest_first': bool(history_newest_first),
        'observation_layout': ['body_ang_vel*0.25', 'body_gravity', 'command*[2,2,0.25]',
                               'joint_pos-default', 'joint_vel*0.05', 'last_action'],
        'final_xyz': final, 'xmax': round(xmax, 4), 'zmin': round(zmin, 4),
        'max_abs_y': round(max_abs_y, 4), 'fell': bool(fell), 'unstable': bool(unstable),
        'marks': marks, 'trace': trace,
    }
    if stair:
        boundaries = mark_x[:-1]
        crossed = xmax > mark_x[-1]
        within = all(('%.3f' % x) in marks and abs(marks['%.3f' % x][1]) < 1.0
                     for x in boundaries)
        clean = not fell and not unstable and crossed and within
        stable = clean and final[0] > mark_x[-1] and abs(final[1]) < stair['width'] / 2
        result.update(stair=stair, crossed_once=bool(crossed),
                      clean_climb=bool(clean), stable_finish=bool(stable))
        result['pass'] = bool(clean)
    elif v_cmd == 0:
        result['pass'] = bool(not fell and not unstable and abs(final[0]) < .15 and
                              abs(final[1]) < .15 and final[2] > .15)
    else:
        result['pass'] = bool(not fell and not unstable and xmax > .5 and max_abs_y < .8)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy', required=True)
    parser.add_argument('--xml', default='/tmp/mjcf/Lite3.xml')
    parser.add_argument('--mode', choices=('stand', 'walk', 'stairs'), default='walk')
    parser.add_argument('--seconds', type=float)
    parser.add_argument('--v-cmd', type=float, default=.3)
    parser.add_argument('--step-h', type=float, default=.06)
    parser.add_argument('--steps', type=int, default=3)
    parser.add_argument('--step-depth', type=float, default=.5)
    parser.add_argument('--step-width', type=float, default=3.0)
    parser.add_argument('--step-front', type=float, default=1.0)
    parser.add_argument('--out')
    args = parser.parse_args()
    seconds = args.seconds if args.seconds is not None else {
        'stand': 2., 'walk': 3., 'stairs': 10.}[args.mode]
    stairs = None
    xml = args.xml
    if args.mode == 'stairs':
        stairs = {'step_h': args.step_h, 'steps': args.steps, 'depth': args.step_depth,
                  'width': args.step_width, 'front': args.step_front}
        xml = make_stairs_xml(args.xml, args.step_h, args.steps, args.step_depth,
                              args.step_width, args.step_front)
    try:
        result = run_rollout(xml, args.policy, seconds,
                             0. if args.mode == 'stand' else args.v_cmd, stair=stairs)
    finally:
        if stairs:
            shutil.rmtree(Path(xml).parent, ignore_errors=True)
    text = json.dumps(result, ensure_ascii=False, indent=1)
    print(text)
    if args.out:
        Path(args.out).write_text(text + '\n')
    return 0 if result['pass'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
