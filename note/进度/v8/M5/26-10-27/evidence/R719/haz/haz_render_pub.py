"""感知静台：MuJoCo 深度相机 10 Hz 渲染，墙以 0.3 m/s 沿 -x 接近；发布深度图与真值日志。"""
import json, sys, time
import numpy as np, mujoco, rospy
from sensor_msgs.msg import Image

XML = '/work/Lite3_hazcam.xml'
DEF = np.array([0, -0.65, 1.3] * 4)

rospy.init_node('haz_render_pub', disable_signals=True)
pub = rospy.Publisher('/haz/depth', Image, queue_size=1)
rate_hz = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
out = sys.argv[2] if len(sys.argv) > 2 else '/work/haz_frames.json'
m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)
RB = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'floating_base')]
WJ = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'wall_free')]
d.qpos[RB + 2] = 0.32
d.qpos[RB + 7:RB + 19] = DEF.copy()
first = None
W, H = 96, 72
r = mujoco.Renderer(m, H, W)
r.enable_depth_rendering()
# 相机在 TORSO 前 0.252 m、离地约 0.394 m；墙前表面 x = body_x - 0.1
frames = []
t0 = time.time()
dur = 6.0
i = 0
while time.time() - t0 < dur:
    t = time.time() - t0
    wall_x = 2.0 - 0.3 * t
    # free joint qpos: x y z qw qx qy qz
    mj = WJ
    d.qpos[mj] = wall_x
    d.qpos[mj + 1] = 0.0
    d.qpos[mj + 2] = 0.5
    d.qpos[mj + 3:mj + 7] = [1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(m, d)
    cam_x = float(d.site_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'depth_camera_site')][0])
    range_x = (wall_x - 0.1) - cam_x
    tc = time.time()
    r.update_scene(d, camera='haz_cam')
    dep = r.render().copy()
    te = time.time()
    img = Image()
    img.header.stamp = rospy.Time.now()
    img.height, img.width = H, W
    img.encoding = '32FC1'
    img.step = W * 4
    img.data = np.ascontiguousarray(dep, dtype=np.float32).tobytes()
    tp = time.time()
    pub.publish(img)
    frames.append({'i': i, 't_cross': tc, 't_capture': tc, 't_render_done': te,
                   't_publish': tp, 'wall_x': wall_x, 'range_x': range_x})
    i += 1
    nxt = t0 + i / rate_hz
    lag = nxt - time.time()
    if lag > 0:
        time.sleep(lag)
with open(out, 'w') as f:
    json.dump({'rate_hz': rate_hz, 'frames': frames,
               'render_ms_mean': round(1000 * np.mean([f['t_render_done'] - f['t_capture'] for f in frames]), 2),
               'cam_x': cam_x}, f)
print('RENDER_DONE frames=%d render_ms=%.1f' % (len(frames), 1000 * np.mean([f['t_render_done'] - f['t_capture'] for f in frames])))
