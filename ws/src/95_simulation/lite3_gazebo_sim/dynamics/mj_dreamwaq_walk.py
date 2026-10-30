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
import xml.etree.ElementTree as ET
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


def yaw_of(q):
    w, x, y, z = q
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def heading_command(v_cmd, yaw, y, yaw_gain=1.0, y_gain=.5, max_wz=.4):
    """Hold world +X heading and the y=0 line with a bounded body yaw-rate command."""
    wz = float(np.clip(-yaw_gain * yaw - y_gain * y, -max_wz, max_wz))
    return np.array([v_cmd, 0., wz], dtype=np.float32)


def sample_initial(seed, perturb=True):
    """Reproducible initial base perturbation; seed alone changes the start state."""
    if not perturb:
        return {'dx': 0., 'dy': 0., 'dyaw': 0., 'dz': 0.}
    rng = np.random.default_rng(seed)
    return {'dx': float(rng.uniform(-.05, .05)),
            'dy': float(rng.uniform(-.10, .10)),
            'dyaw': float(rng.uniform(-.0873, .0873)),
            'dz': float(rng.uniform(-.005, .005))}


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


def make_stairs_xml(flat_xml, step_h, steps=3, depth=.5, width=3.0, front=1.0,
                    landing_depth=0.0, contact_profile=None):
    """Create a temporary 3-step world that includes the supplied flat model.

    contact_profile optionally overrides the model option (timestep/integrator)
    and the stair geoms' contact softness (solref/solimp); defaults unchanged.
    """
    flat = Path(flat_xml).resolve()
    if steps < 1 or step_h <= 0 or depth <= 0 or width <= 0 or landing_depth < 0:
        raise ValueError('stairs need positive step height/depth/width and steps >= 1')
    work = Path(tempfile.mkdtemp(prefix='lite3_stairs_'))
    shutil.copy2(flat, work / flat.name)
    if contact_profile:
        tree = ET.parse(work / flat.name)
        root = tree.getroot()
        option = root.find('option')
        if option is None:
            option = ET.Element('option')
            root.insert(1 if root.find('compiler') is not None else 0, option)
        for key in ('timestep', 'integrator'):
            if contact_profile.get(key) is not None:
                option.set(key, str(contact_profile[key]))
        tree.write(work / flat.name, encoding='utf-8', xml_declaration=True)
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
    contact = ''
    if contact_profile:
        if contact_profile.get('solref'):
            contact += ' solref="%s"' % contact_profile['solref']
        if contact_profile.get('solimp'):
            contact += ' solimp="%s"' % contact_profile['solimp']
    geoms = []
    for i in range(steps):
        z = step_h * (i + 1) / 2.
        x = front + depth * i + depth / 2.
        geoms.append('    <geom name="step%d" type="box" size="%g %g %g" pos="%g 0 %g" '
                     'friction="1 0.01 0.01"%s rgba="0.4 0.7 0.4 1"/>'
                     % (i + 1, depth / 2., width / 2., z, x, z, contact))
    if landing_depth:
        top = step_h * steps
        geoms.append('    <geom name="landing" type="box" size="%g %g %g" pos="%g 0 %g" '
                     'friction="1 0.01 0.01"%s rgba="0.4 0.7 0.4 1"/>'
                     % (landing_depth / 2., width / 2., top / 2.,
                        front + depth * steps + landing_depth / 2., top / 2., contact))
    xml = ('<mujoco model="Lite3_stairs_auto">\n'
           '  <compiler meshdir="%s"/>\n'
           '  <include file="%s"/>\n'
           '  <worldbody>\n%s\n  </worldbody>\n</mujoco>\n'
           % (work, flat.name, '\n'.join(geoms)))
    path = work / 'stairs.xml'
    path.write_text(xml)
    return str(path)


def nonfinite_state(streak, finite, limit=5):
    """Count consecutive non-finite actions; terminal only after the limit."""
    if finite:
        return 0, False
    streak += 1
    return streak, streak >= limit


def stair_verdict(final, xmax, marks, stair, fell, unstable):
    """Classify a stair rollout; a stable finish must remain on the top level.

    No-landing stairs: the 0.5 m top step cannot be crossed by the old
    "top edge + 0.2 m" rule without walking off the support, so reach/stand on
    the top step is the crossing condition and the top edge itself is not
    required as a mark. With a landing, the old edge+0.2 m rule is kept.
    """
    boundaries = [stair['front'] + stair['depth'] * (i + 1) for i in range(stair['steps'])]
    landing = float(stair.get('landing_depth', 0.) or 0.)
    top_start = boundaries[-2] if len(boundaries) > 1 else stair['front']
    if landing > 0:
        required_x = boundaries[-1] + .2
        required_marks = boundaries
    else:
        required_x = top_start + min(.2, stair['depth'] / 2.)
        required_marks = boundaries[:-1]
    crossed = xmax > required_x
    within = all(('%.3f' % x) in marks and abs(marks['%.3f' % x][1]) < 1.0
                 for x in required_marks)
    clean = not fell and not unstable and crossed and within
    stable = (clean and final[0] > required_x
              and abs(final[1]) < stair['width'] / 2
              and final[2] > stair['step_h'] * stair['steps'] + .1)
    return {'crossed_once': bool(crossed), 'clean_climb': bool(clean),
            'stable_finish': bool(stable)}


def run_rollout(xml, policy, seconds, v_cmd, default_pose=DEFAULT_POSE,
                kp=25.0, kd=.5, initial_z=.40, history_newest_first=True,
                stair=None, heading_hold=False, yaw_gain=1.0, y_gain=.5, max_wz=.4,
                initial_perturbation=None):
    import mujoco
    import onnxruntime as ort

    session = ort.InferenceSession(policy, providers=['CPUExecutionProvider'])
    input_name = session.get_inputs()[0].name
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    default_pose = np.asarray(default_pose, dtype=float)
    perturb = initial_perturbation or {'dx': 0., 'dy': 0., 'dyaw': 0., 'dz': 0.}
    data.qpos[:3] = [perturb['dx'], perturb['dy'], initial_z + perturb['dz']]
    data.qpos[3:7] = [np.cos(perturb['dyaw'] / 2.), 0., 0., np.sin(perturb['dyaw'] / 2.)]
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
            if heading_hold and v_cmd:
                command = heading_command(v_cmd, yaw_of(data.qpos[3:7]), float(data.qpos[1]),
                                          yaw_gain, y_gain, max_wz)
            obs = observation(data.qpos, data.qvel, command, last_action, default_pose)
            state = state_vector(obs, history, newest_first=history_newest_first)
            action = session.run(None, {input_name: state.reshape(1, -1)})[0].reshape(-1)
            target = joint_target(action, default_pose)
            if not np.all(np.isfinite(action)) or not np.all(np.isfinite(target)):
                unstable = True
                break
            last_action = action.astype(np.float32)
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
        'heading_hold': bool(heading_hold), 'yaw_gain': yaw_gain, 'y_gain': y_gain,
        'max_wz_command': max_wz, 'initial_perturbation': perturb,
        'observation_layout': ['body_ang_vel*0.25', 'body_gravity', 'command*[2,2,0.25]',
                               'joint_pos-default', 'joint_vel*0.05', 'last_action'],
        'final_xyz': final, 'xmax': round(xmax, 4), 'zmin': round(zmin, 4),
        'max_abs_y': round(max_abs_y, 4), 'fell': bool(fell), 'unstable': bool(unstable),
        'marks': marks, 'trace': trace,
    }
    if stair:
        verdict = stair_verdict(final, xmax, marks, stair, fell, unstable)
        result.update(stair=stair, **verdict)
        result['pass'] = verdict['clean_climb']
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
    parser.add_argument('--landing-depth', type=float, default=0.0)
    parser.add_argument('--heading-hold', action='store_true')
    parser.add_argument('--yaw-gain', type=float, default=1.0)
    parser.add_argument('--y-gain', type=float, default=.5)
    parser.add_argument('--max-wz', type=float, default=.4)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--perturb', action='store_true')
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
                              args.step_width, args.step_front, args.landing_depth)
    try:
        result = run_rollout(xml, args.policy, seconds,
                             0. if args.mode == 'stand' else args.v_cmd, stair=stairs,
                             heading_hold=args.heading_hold, yaw_gain=args.yaw_gain,
                             y_gain=args.y_gain, max_wz=args.max_wz,
                             initial_perturbation=sample_initial(args.seed, args.perturb))
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
