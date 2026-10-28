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
import json
from pathlib import Path

import numpy as np


def rot(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def rollout(xml, policy, seconds=10.0, v_cmd=0.3):
    import mujoco
    import onnxruntime as ort

    sess = ort.InferenceSession(policy, providers=['CPUExecutionProvider'])
    inp = sess.get_inputs()[0].name
    model = mujoco.MjModel.from_xml_path(xml)
    data = mujoco.MjData(model)
    default_pose = np.array([0, -0.65, 1.3] * 4)
    scale = np.array([0.125, 0.25, 0.25] * 4)
    data.qpos[2] = 0.35
    data.qpos[7:19] = default_pose
    mujoco.mj_forward(model, data)
    dt = model.opt.timestep
    stride = max(1, int(round(0.02 / dt)))
    last = np.zeros(12)
    target = default_pose.copy()
    steps = int(seconds / dt)
    zmin, xmax, ncon_peak = 9.0, -9.0, 0
    trace = []
    for i in range(steps):
        if i % stride == 0:
            obs = np.concatenate([
                data.qvel[3:6] * 0.25,
                rot(data.qpos[3:7]).T @ np.array([0, 0, -1.0]),
                [v_cmd / 0.8, 0, 0],
                data.qpos[7:19] - default_pose,
                data.qvel[6:18] * 0.05,
                last]).astype(np.float32)
            out = sess.run(None, {inp: obs.reshape(1, -1)})[0].reshape(-1)
            last = out.astype(np.float32)
            target = out * scale + default_pose
        tau = np.clip(30.0 * (target - data.qpos[7:19]) - 1.0 * data.qvel[6:18], -30, 30)
        data.qfrc_applied[6:18] = tau
        mujoco.mj_step(model, data)
        zmin = min(zmin, float(data.qpos[2]))
        xmax = max(xmax, float(data.qpos[0]))
        ncon_peak = max(ncon_peak, int(data.ncon))
        if i % int(round(0.5 / dt)) == 0:
            trace.append((round(i * dt, 1), round(float(data.qpos[0]), 3),
                          round(float(data.qpos[2]), 3)))
        if data.qpos[2] < 0.12:
            break
    return {
        'xml': xml,
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
    parser.add_argument('--out', default='')
    args = parser.parse_args()
    flat = rollout(args.xml, args.policy, args.seconds)
    step = rollout(args.step_xml, args.policy, args.seconds)
    # 台阶前沿 x=1.0（box 0.6 半长、中心 1.6）；越过判定：机身中心 x>1.6 且未跌落
    step['step_front_x'] = 1.0
    step['step_cleared'] = bool((not step['fell']) and step['xmax'] > 1.6)
    result = {'flat': flat, 'step10': step,
              'verdict': 'PASS' if (not flat['fell']) else 'FAIL',
              'note': 'verdict 只判平地基线（策略不掉）；台阶是否通过看 step10.step_cleared'}
    text = json.dumps(result, ensure_ascii=False, indent=1)
    print(text)
    if args.out:
        Path(args.out).write_text(text + '\n')
    return 0 if result['verdict'] == 'PASS' else 6


if __name__ == '__main__':
    raise SystemExit(main())
