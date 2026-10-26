"""ROS/MuJoCo 全链停车测量桥：/cmd_vel -> policy -> 基座轨迹。

- 物理按 500 Hz 步进，用墙钟配速到 1x 实时（与真实机器人时间尺度一致）。
- 记录 /cmd_vel 每条消息的到达时刻与值，以及每 10 ms 的基座位姿/速度。
- 10 s 后把 JSON 写到 ~out。
"""
import json, os, sys, threading, time
import numpy as np, mujoco, onnxruntime as ort, rospy
from geometry_msgs.msg import Twist

XML = os.environ.get('MJ_XML', '/work/Lite3_rl_deploy-main/third_party/deep_robotics_model/Lite3/Lite3_mjcf/mjcf/Lite3.xml')
FRIC = os.environ.get('FRICTION_SCALE')
ONNX = '/work/Lite3_rl_deploy-main/policy/ppo/policy.onnx'
DEF = np.array([0, -0.65, 1.3] * 4)
SCALE = np.array([0.125, 0.25, 0.25] * 4)


def rot(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def yaw_of(q):
    w, x, y, z = q
    return np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


class Bridge(object):
    def __init__(self):
        self.cmd = 0.0
        self.cmd_events = []
        self.lock = threading.Lock()
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self._cb, queue_size=5)

    def _cb(self, msg):
        with self.lock:
            self.cmd = float(msg.linear.x)
            self.cmd_events.append((time.time(), float(msg.linear.x)))

    def get_cmd(self):
        with self.lock:
            return self.cmd


def main():
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    out = sys.argv[2] if len(sys.argv) > 2 else '/work/mj_chain.json'
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 11.0
    rospy.init_node('mj_bridge', disable_signals=True)
    br = Bridge()
    threading.Thread(target=rospy.spin, daemon=True).start()

    sess = ort.InferenceSession(ONNX, providers=['CPUExecutionProvider'])
    inp = sess.get_inputs()[0].name
    m = mujoco.MjModel.from_xml_path(XML)
    if FRIC:
        m.geom_friction[:, 0] *= float(FRIC)
    d = mujoco.MjData(m)
    rng = np.random.RandomState(seed)
    d.qpos[2] = 0.35
    d.qpos[7:19] = DEF + rng.uniform(-0.01, 0.01, 12)
    mujoco.mj_forward(m, d)
    dt = m.opt.timestep
    stride = max(1, int(round(0.02 / dt)))
    last = np.zeros(12)
    target = DEF.copy()
    samples = []
    wall0 = time.time()
    sim_t = 0.0
    i = 0
    while sim_t < duration:
        if i % stride == 0:
            cv = br.get_cmd()
            obs = np.concatenate([
                d.qvel[3:6] * 0.25,
                rot(d.qpos[3:7]).T @ np.array([0, 0, -1.0]),
                [cv / 0.8, 0, 0],
                d.qpos[7:19] - DEF,
                d.qvel[6:18] * 0.05,
                last]).astype(np.float32)
            if np.all(np.isfinite(obs)):
                out_ = sess.run(None, {inp: obs.reshape(1, -1)})[0].reshape(-1)
                if np.all(np.isfinite(out_)):
                    last = out_.astype(np.float32)
                    target = out_ * SCALE + DEF
        tau = np.clip(30.0 * (target - d.qpos[7:19]) - 1.0 * d.qvel[6:18], -30, 30)
        d.qfrc_applied[6:18] = tau
        mujoco.mj_step(m, d)
        i += 1
        sim_t += dt
        if i % 5 == 0:
            samples.append((time.time(), round(sim_t, 4), round(float(d.qpos[0]), 5),
                            round(float(d.qpos[1]), 5), round(float(yaw_of(d.qpos[3:7])), 5),
                            round(float(d.qvel[0]), 4), round(float(d.qpos[2]), 4),
                            br.get_cmd()))
        # 1x real-time pacing
        lag = (wall0 + sim_t) - time.time()
        if lag > 0.002:
            time.sleep(min(lag, 0.01))
    with open(out, 'w') as f:
        json.dump({'seed': seed, 'xml': XML, 'policy': ONNX, 'ctrl_hz': 50,
                   'samples': samples, 'cmd_events': br.cmd_events}, f)
    print('BRIDGE_DONE %s samples=%d' % (out, len(samples)))


if __name__ == '__main__':
    main()
