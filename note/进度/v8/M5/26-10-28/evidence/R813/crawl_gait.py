#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V8-X1 准静态爬行前置：数值腿 IK＋站立保持（MuJoCo）。

模型：DeepRobotics Lite3 MJCF（第三方 /work，不入库）。12 个关节（每腿 HipX/HipY/Knee），
关节 qpos 7..18，足端 body FL_FOOT/FR_FOOT/HL_FOOT/HR_FOOT。IK 用数值阻尼最小二乘
（目标＝TORSO 系足端位置），不依赖手推几何符号。

用法（容器内）：
  python3 crawl_gait.py stand  --seconds 3
  python3 crawl_gait.py lift   --leg FL --height 0.05 --seconds 4
  python3 crawl_gait.py step   --leg FL --height 0.10 --seconds 6   # 单腿上 0.1 m 台阶（平地世界近似）
"""
import argparse
import json
from pathlib import Path

import numpy as np

XML = '/work/Lite3_rl_deploy-main/third_party/deep_robotics_model/Lite3/Lite3_mjcf/mjcf/Lite3.xml'
LEGS = ('FL', 'FR', 'HL', 'HR')
DEF = np.array([0, -0.65, 1.3] * 4)


def _ids(model, leg):
    import mujoco
    j0 = 7 + 3 * LEGS.index(leg)
    foot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, leg + '_FOOT')
    torso = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'TORSO')
    return j0, foot, torso


def foot_body_pos(model, data, leg):
    """足端在 TORSO 系的位置（当前位置）。"""
    import mujoco
    j0, foot, torso = _ids(model, leg)
    mujoco.mj_forward(model, data)
    R = data.xpos[torso]
    rel = data.xpos[foot] - R
    # TORSO 姿态（站立时接近单位阵）
    quat = data.xquat[torso]
    w, x, y, z = quat
    rot = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return rot.T @ rel


def leg_ik(model, data, leg, target, iters=80, damping=0.02):
    """把一条腿的足端解到 TORSO 系 target；返回关节角（不改动 data 的最终状态之外的语义）。"""
    import mujoco
    j0, _foot, _torso = _ids(model, leg)
    jq = 1 + 3 * LEGS.index(leg)   # 关节索引（0=floating_base），与 qpos 地址 7+3i 不同
    q = data.qpos[j0:j0 + 3].copy()
    lo = model.jnt_range[jq:jq + 3, 0]
    hi = model.jnt_range[jq:jq + 3, 1]
    for _ in range(iters):
        pos = foot_body_pos(model, data, leg)
        err = target - pos
        if np.linalg.norm(err) < 1e-4:
            break
        jac = np.zeros((3, 3))
        for k in range(3):
            probe = data.qpos[j0:j0 + 3].copy()
            probe[k] += 1e-4
            data.qpos[j0:j0 + 3] = probe
            jac[:, k] = (foot_body_pos(model, data, leg) - pos) / 1e-4
            data.qpos[j0:j0 + 3] = q
        dq = jac.T @ np.linalg.solve(jac @ jac.T + damping * np.eye(3), err)
        q = np.clip(q + dq, lo, hi)
        data.qpos[j0:j0 + 3] = q
    return q


def pd_hold(model, data, targets, kp=40.0, kd=1.5, max_tau=30.0):
    tau = np.clip(kp * (targets - data.qpos[7:19]) - kd * data.qvel[6:18], -max_tau, max_tau)
    data.qfrc_applied[6:18] = tau
    return tau


def run(mode='stand', leg='FL', height=0.05, seconds=3.0, kp=40.0, kd=1.5, shift_y=0.0, shift_x=0.0):
    import mujoco
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    data.qpos[2] = 0.35
    data.qpos[7:19] = DEF
    mujoco.mj_forward(model, data)
    nominal = {name: foot_body_pos(model, data, name) for name in LEGS}
    targets = DEF.copy()
    if shift_y or shift_x:
        for name in LEGS:
            goal = nominal[name] + np.array([shift_x, shift_y, 0.0])
            q = leg_ik(model, data, name, goal)
            k = 3 * LEGS.index(name)      # targets 是 12 维关节向量（qpos 7..18 去掉偏移）
            targets[k:k + 3] = q
        data.qpos[7:19] = DEF
        mujoco.mj_forward(model, data)
    if mode in ('lift', 'step'):
        goal = nominal[leg] + np.array([0.0, 0.0, height])
        q = leg_ik(model, data, leg, goal)
        k = 3 * LEGS.index(leg)
        targets[k:k + 3] = q
        data.qpos[7:19] = DEF
        mujoco.mj_forward(model, data)
    dt = model.opt.timestep
    steps = int(seconds / dt)
    zmin, trace = 9.0, []
    for i in range(steps):
        pd_hold(model, data, targets, kp=kp, kd=kd)
        mujoco.mj_step(model, data)
        zmin = min(zmin, float(data.qpos[2]))
        if i % int(round(0.25 / dt)) == 0:
            trace.append((round(i * dt, 2), round(float(data.qpos[2]), 3),
                          round(float(data.qpos[0]), 3)))
        if data.qpos[2] < 0.12:
            break
    feet = {name: [round(float(x), 3) for x in foot_body_pos(model, data, name)]
            for name in LEGS}
    return {'mode': mode, 'leg': leg if mode != 'stand' else None, 'height': height,
            'seconds': seconds, 'final_z': round(float(data.qpos[2]), 3),
            'final_x': round(float(data.qpos[0]), 3), 'zmin': round(zmin, 3),
            'fell': bool(zmin < 0.18),
            'nominal_feet': {k: [round(float(x), 3) for x in v] for k, v in nominal.items()},
            'final_feet': feet, 'trace': trace}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['stand', 'lift', 'step'])
    parser.add_argument('--leg', default='FL')
    parser.add_argument('--height', type=float, default=0.05)
    parser.add_argument('--seconds', type=float, default=3.0)
    parser.add_argument('--kp', type=float, default=40.0)
    parser.add_argument('--kd', type=float, default=1.5)
    parser.add_argument('--shift-y', type=float, default=0.0)
    parser.add_argument('--shift-x', type=float, default=0.0)
    parser.add_argument('--out', default='')
    args = parser.parse_args()
    result = run(args.mode, args.leg, args.height, args.seconds,
                 kp=args.kp, kd=args.kd, shift_y=args.shift_y, shift_x=args.shift_x)
    text = json.dumps(result, ensure_ascii=False, indent=1)
    print(text)
    if args.out:
        Path(args.out).write_text(text + '\n')
    return 0 if not result['fell'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
