"""抓 MuJoCo 仿真画面：站立/行进/切零/停住 + 风险深度视图。"""
import json, numpy as np, mujoco, onnxruntime as ort, cv2

XML = '/work/Lite3_rl_deploy-main/third_party/deep_robotics_model/Lite3/Lite3_mjcf/mjcf/Lite3.xml'
ONNX = '/work/Lite3_rl_deploy-main/policy/ppo/policy.onnx'
OUT = '/work/frames'
import os
os.makedirs(OUT, exist_ok=True)
DEF = np.array([0, -0.65, 1.3] * 4)
SCALE = np.array([0.125, 0.25, 0.25] * 4)


def rot(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


sess = ort.InferenceSession(ONNX, providers=['CPUExecutionProvider'])
inp = sess.get_inputs()[0].name
m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)
jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'floating_base')
adr = m.jnt_qposadr[jid]
d.qpos[adr + 2] = 0.35
d.qpos[adr + 7:adr + 19] = DEF.copy()
mujoco.mj_forward(m, d)
dt = m.opt.timestep
stride = max(1, int(round(0.02 / dt)))
last = np.zeros(12)
target = DEF.copy()
r = mujoco.Renderer(m, 480, 640)
cam = mujoco.MjvCamera()
mujoco.mjv_defaultCamera(cam)
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
cam.distance = 2.4
cam.azimuth = 132
cam.elevation = -16
shots = {0.6: '01_standing', 5.98: '02_walking', 6.12: '03_hold', 6.45: '04_braking', 8.6: '05_stopped'}
t = 0.0
taken = set()
steps = int(9.5 / dt)
sim_switch = 6.0
for i in range(steps):
    if i % stride == 0:
        cmd_v = 0.0 if t >= sim_switch else 0.3
        obs = np.concatenate([
            d.qvel[3:6] * 0.25, rot(d.qpos[3:7]).T @ np.array([0, 0, -1.0]),
            [cmd_v / 0.8, 0, 0], d.qpos[adr + 7:adr + 19] - DEF,
            d.qvel[6:18] * 0.05, last]).astype(np.float32)
        out = sess.run(None, {inp: obs.reshape(1, -1)})[0].reshape(-1)
        if np.all(np.isfinite(out)):
            last = out.astype(np.float32); target = out * SCALE + DEF
    tau = np.clip(30.0 * (target - d.qpos[adr + 7:adr + 19]) - 1.0 * d.qvel[6:18], -30, 30)
    d.qfrc_applied[6:18] = tau
    mujoco.mj_step(m, d)
    t += dt
    for ts, name in list(shots.items()):
        if name not in taken and t >= ts:
            cam.lookat[:] = [float(d.qpos[0]), float(d.qpos[1]), float(d.qpos[2]) + 0.05]
            r.update_scene(d, camera=cam)
            img = r.render()
            cv2.imwrite(os.path.join(OUT, name + '.png'), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            taken.add(name)
print('flat_frames', sorted(taken), 'x_end', round(float(d.qpos[0]), 3))

# 风险深度视图：相机前墙 1.2 m
m2 = mujoco.MjModel.from_xml_path('/work/Lite3_hazcam.xml')
d2 = mujoco.MjData(m2)
rb = m2.jnt_qposadr[mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_JOINT, 'floating_base')]
wj = m2.jnt_qposadr[mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_JOINT, 'wall_free')]
d2.qpos[rb + 2] = 0.32
d2.qpos[rb + 7:rb + 19] = DEF.copy()
camx = float(d2.site_xpos[mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_SITE, 'depth_camera_site')][0])
wall_x = camx + 0.1 + 1.2
d2.qpos[wj] = wall_x; d2.qpos[wj + 2] = 0.5; d2.qpos[wj + 3:wj + 7] = [1, 0, 0, 0]
mujoco.mj_forward(m2, d2)
r2 = mujoco.Renderer(m2, 240, 320)
r2.enable_depth_rendering()
r2.update_scene(d2, camera='haz_cam')
dep = r2.render()
r2.disable_depth_rendering()
r2.update_scene(d2, camera='haz_cam')
rgb = r2.render()
cv2.imwrite(os.path.join(OUT, '06_depth_rainbow.png'),
            cv2.applyColorMap(cv2.convertScaleAbs(np.clip(dep, 0, 3.0), alpha=255/3.0), cv2.COLORMAP_TURBO))
cv2.imwrite(os.path.join(OUT, '07_hazcam_rgb.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
print(json.dumps({'wall_range_m': 1.2, 'depth_min': round(float(np.nanmin(dep)), 3),
                  'frames': sorted(os.listdir(OUT))}))
