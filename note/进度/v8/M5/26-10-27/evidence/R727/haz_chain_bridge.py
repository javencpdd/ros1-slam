"""同载体全链桥：MuJoCo Lite3 + 墙 + 深度渲染线程 + /clock + cmd 订阅。"""
import json, os, sys, threading, time
import numpy as np, mujoco, onnxruntime as ort, rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock

XML = '/work/Lite3_hazcam.xml'
ONNX = '/work/Lite3_rl_deploy-main/policy/ppo/policy.onnx'
DEF = np.array([0, -0.65, 1.3] * 4)
SCALE = np.array([0.125, 0.25, 0.25] * 4)
WALL_FRONT = 3.2 - 0.1
CAM_LOCAL_X = 0.252


def rot(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


class B(object):
    def __init__(self):
        self.cmd = 0.0
        self.events = []
        self.lock = threading.Lock()
        rospy.Subscriber('/cmd_vel', Twist, self._cb, queue_size=5)

    def _cb(self, m):
        with self.lock:
            self.cmd = float(m.linear.x)
            self.events.append((time.time(), float(m.linear.x)))

    def get_cmd(self):
        with self.lock:
            return self.cmd


def main():
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    out = sys.argv[2] if len(sys.argv) > 2 else '/work/haz_chain.json'
    dur = float(sys.argv[3]) if len(sys.argv) > 3 else 16.0
    rospy.init_node('haz_chain_bridge', disable_signals=True)
    rospy.set_param('/use_sim_time', True)
    br = B()
    threading.Thread(target=rospy.spin, daemon=True).start()
    clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)
    depth_pub = rospy.Publisher('/haz/depth', Image, queue_size=1)

    sess = ort.InferenceSession(ONNX, providers=['CPUExecutionProvider'])
    inp = sess.get_inputs()[0].name
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    fid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'floating_base')
    rb = m.jnt_qposadr[fid]
    rd = m.jnt_dofadr[fid]
    wj = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'wall_free')]
    rng = np.random.RandomState(seed)
    d.qpos[rb + 2] = 0.35
    d.qpos[rb + 7:rb + 19] = DEF + rng.uniform(-0.01, 0.01, 12)
    d.qpos[wj], d.qpos[wj + 1], d.qpos[wj + 2] = 3.2, 0.0, 0.5
    d.qpos[wj + 3:wj + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    dt = m.opt.timestep
    stride = max(1, int(round(0.02 / dt)))

    state = {'qpos': d.qpos.copy(), 'qvel': d.qvel.copy(), 'sim_t': 0.0,
             'base_x': float(d.qpos[rb]), 'stop': False}
    lock = threading.Lock()
    frames = []
    W, H = 96, 72

    class RT(threading.Thread):
        def run(self):
            d2 = mujoco.MjData(m)
            r = mujoco.Renderer(m, H, W)
            r.enable_depth_rendering()
            while True:
                with lock:
                    if state['stop']:
                        break
                    qpos = state['qpos'].copy(); qvel = state['qvel'].copy()
                    sim_t = state['sim_t']; bx = state['base_x']
                d2.qpos[:] = qpos; d2.qvel[:] = qvel
                mujoco.mj_forward(m, d2)
                tc = time.time()
                r.update_scene(d2, camera='haz_cam')
                dep = r.render().copy()
                te = time.time()
                img = Image()
                img.header.stamp = rospy.Time.from_sec(sim_t)
                img.height, img.width = H, W
                img.encoding = '32FC1'; img.step = W * 4
                img.data = np.ascontiguousarray(dep, dtype=np.float32).tobytes()
                depth_pub.publish(img)
                roi = dep[int(H * 0.15):int(H * 0.50), int(W * 0.3):int(W * 0.7)]
                ok = np.isfinite(roi) & (roi > 0.05) & (roi < 5.0)
                dmin = float(roi[ok].min()) if ok.any() else None
                frames.append({'sim_t': sim_t, 'base_x': bx, 'gt_range': WALL_FRONT - (bx + CAM_LOCAL_X),
                               'dmin': dmin, 'render_ms': round((te - tc) * 1000, 1),
                               'wall_t': time.time()})
                time.sleep(0.02)
            state['renderer'] = None

    rt = RT(); rt.daemon = True; rt.start()
    last = np.zeros(12); target = DEF.copy()
    samples = []; t = 0.0
    t0 = time.time(); i = 0
    steps = int(dur / dt)
    while i < steps:
        if i % stride == 0:
            cv = br.get_cmd()
            obs = np.concatenate([
                d.qvel[rd + 3:rd + 6] * 0.25, rot(d.qpos[rb + 3:rb + 7]).T @ np.array([0, 0, -1.0]),
                [cv / 0.8, 0, 0], d.qpos[rb + 7:rb + 19] - DEF,
                d.qvel[rd + 6:rd + 18] * 0.05, last]).astype(np.float32)
            if np.all(np.isfinite(obs)):
                out_ = sess.run(None, {inp: obs.reshape(1, -1)})[0].reshape(-1)
                if np.all(np.isfinite(out_)):
                    last = out_.astype(np.float32); target = out_ * SCALE + DEF
        tau = np.clip(30.0 * (target - d.qpos[rb + 7:rb + 19]) - 1.0 * d.qvel[rd + 6:rd + 18], -30, 30)
        d.qfrc_applied[rd + 6:rd + 18] = tau
        mujoco.mj_step(m, d)
        i += 1; t += dt
        if i % 5 == 0:
            c = Clock(); c.clock = rospy.Time.from_sec(t); clock_pub.publish(c)
        if i % 5 == 0:
            samples.append((round(t, 4), round(float(d.qpos[rb]), 5), round(float(d.qpos[rb + 2]), 4),
                            round(float(d.qvel[rd]), 4), br.get_cmd(), time.time()))
        with lock:
            state['qpos'] = d.qpos.copy(); state['qvel'] = d.qvel.copy()
            state['sim_t'] = t; state['base_x'] = float(d.qpos[rb])
        lag = (t0 + t) - time.time()
        if lag > 0.002:
            time.sleep(min(lag, 0.01))
    with lock:
        state['stop'] = True
    rt.join(timeout=3)
    json.dump({'seed': seed, 'frames': frames, 'samples': samples,
               'cmd_events': br.events, 'wall_front': WALL_FRONT,
               'final_x': float(d.qpos[rb]), 'zmin': float(min(s[2] for s in samples))},
              open(out, 'w'))
    print('BRIDGE_DONE frames=%d samples=%d x=%.3f' % (len(frames), len(samples), float(d.qpos[rb])))


if __name__ == '__main__':
    main()
