import sys, json, numpy as np, mujoco, onnxruntime as ort

XML = '/work/Lite3_rl_deploy-main/third_party/deep_robotics_model/Lite3/Lite3_mjcf/mjcf/Lite3.xml'
ONNX = '/work/Lite3_rl_deploy-main/policy/ppo/policy.onnx'

def rot(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])

def yaw_of(q):
    w, x, y, z = q
    return np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

def run(seed, v_cmd=0.3, spin_up=6.0, avg_win=0.2, stand_thr=0.02, hold=0.3, max_stop=5.0):
    sess = ort.InferenceSession(ONNX, providers=['CPUExecutionProvider'])
    inp = sess.get_inputs()[0].name
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    DEF = np.array([0, -0.65, 1.3] * 4)
    SCALE = np.array([0.125, 0.25, 0.25] * 4)
    rng = np.random.RandomState(seed)
    d.qpos[2] = 0.35
    d.qpos[7:19] = DEF + rng.uniform(-0.01, 0.01, 12)
    mujoco.mj_forward(m, d)
    dt = m.opt.timestep
    stride = max(1, int(round(0.02 / dt)))
    last = np.zeros(12)
    target = DEF.copy()
    t = 0.0
    switch_i = int(round(spin_up / dt))
    t0 = x0 = y0 = v0 = yaw0 = None
    win = []
    stand_start = None
    t_confirmed = None
    t_first_below = None
    t_peak = None
    peak_along = -9.0
    zmin = 9.0; ymax = 0.0; yaw_drift = 0.0
    nan = False
    trace = []
    steps = int((spin_up + max_stop + 1.0) / dt)
    for i in range(steps):
        if i % stride == 0:
            cmd_v = 0.0 if i >= switch_i else v_cmd
            obs = np.concatenate([
                d.qvel[3:6] * 0.25,
                rot(d.qpos[3:7]).T @ np.array([0, 0, -1.0]),
                [cmd_v / 0.8, 0, 0],
                d.qpos[7:19] - DEF,
                d.qvel[6:18] * 0.05,
                last]).astype(np.float32)
            if not np.all(np.isfinite(obs)):
                nan = True; break
            out = sess.run(None, {inp: obs.reshape(1, -1)})[0].reshape(-1)
            if not np.all(np.isfinite(out)):
                nan = True; break
            last = out.astype(np.float32)
            target = out * SCALE + DEF
            if i == switch_i:
                t0 = t; x0 = float(d.qpos[0]); y0 = float(d.qpos[1])
                v0 = float(d.qvel[0]); yaw0 = yaw_of(d.qpos[3:7])
        tau = np.clip(30.0 * (target - d.qpos[7:19]) - 1.0 * d.qvel[6:18], -30, 30)
        d.qfrc_applied[6:18] = tau
        mujoco.mj_step(m, d)
        t += dt
        zmin = min(zmin, float(d.qpos[2]))
        if not np.isfinite(d.qpos[0]):
            nan = True; break
        if t0 is not None:
            ymax = max(ymax, abs(float(d.qpos[1]) - y0))
            dx = float(d.qpos[0]) - x0
            dy = float(d.qpos[1]) - y0
            along = dx * np.cos(yaw0) + dy * np.sin(yaw0)
            if along > peak_along:
                peak_along = along
                t_peak = t
            yaw_drift = max(yaw_drift, abs(np.degrees(yaw_of(d.qpos[3:7]) - yaw0)))
        if t0 is not None and t_confirmed is None:
            win.append((t, float(d.qvel[0])))
            while win and win[0][0] < t - avg_win:
                win.pop(0)
            if t - t0 >= avg_win:
                v_avg = sum(v for _, v in win) / len(win)
                if abs(v_avg) < stand_thr:
                    if t_first_below is None:
                        t_first_below = t
                    if stand_start is None:
                        stand_start = t
                    elif t - stand_start >= hold:
                        t_confirmed = t
                else:
                    stand_start = None
            if i % 25 == 0:
                trace.append((round(t - t0, 2), round(float(d.qvel[0]), 3), round(along, 3)))
        if d.qpos[2] < 0.18:
            break
    if not nan and d.qpos[2] >= 0.18 and t_confirmed is None:
        return {'seed': seed, 'ok': False, 'reason': 'no_confirmed_standstill',
                'v0': v0, 'nan': nan, 'zmin': round(zmin, 3), 'trace': trace[-12:]}
    return {'seed': seed, 'ok': not nan, 'v_cmd': v_cmd, 'v0': round(v0, 3),
            'stop_distance_max_forward_m': round(peak_along, 4),
            'stop_time_to_peak_s': round(t_peak - t0, 3) if t_peak else None,
            'stop_time_first_below_s': round(t_first_below - t0, 3) if t_first_below else None,
            'stop_time_confirmed_s': round(t_confirmed - t0, 3) if t_confirmed else None,
            'yaw_drift_deg': round(yaw_drift, 2),
            'y_drift_m': round(ymax, 3), 'zmin': round(zmin, 3),
            'fell': zmin < 0.18, 'nan': nan, 'trace': trace}

if __name__ == '__main__':
    seeds = [int(s) for s in sys.argv[1:]] or [0, 1, 2]
    rows = [run(s) for s in seeds]
    print(json.dumps({'xml': 'Lite3.xml(flat,floor)', 'policy': 'policy.onnx',
                      'ctrl_hz': 50, 'cut_based_on': 'cmd=0 at policy input',
                      'rows': rows}, ensure_ascii=False))
