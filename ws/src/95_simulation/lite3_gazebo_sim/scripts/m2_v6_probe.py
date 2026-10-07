#!/usr/bin/env python3
# m2_v6_probe.py —— M2R5/U1 结构等价判据（V6）的运行时取证件（小张设计 6 节 V6；小明施工 2026-10-08）
# 为何存在：root 由 TORSO 改为 base_link 后，「只动拓扑、不动物理」不得是断言，须 measurable。
#   取两态：改前件经 m2_gazebo.launch 的 payload_dir 口子另起一台（源文件不翻转），两份输出逐字段可比。
# 字段名以本仓 noetic 的 gazebo_msgs srv 定义为准（GetLinkProperties 回 com/mass/ixx..izz/无 bbox；
#   GetModelProperties 回 parent_model_name/canonical_body_name/body_names/geom_names/joint_names/is_static）。
# 用法：python3 m2_v6_probe.py <标签=pre|post>
import sys, time, rospy
from gazebo_msgs.srv import GetModelProperties, GetLinkProperties
from gazebo_msgs.msg import ModelStates

TAG = sys.argv[1] if len(sys.argv) > 1 else 'post'
rospy.init_node('m2_v6_probe')

POSE = []
def cb(m):
    for p, n in zip(m.pose, m.name):
        if n == 'lite3_planar':
            POSE.append((p.position.x, p.position.y, p.position.z,
                         p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
rospy.Subscriber('/gazebo/model_states', ModelStates, cb, queue_size=50)

t_end = time.time() + 6.0
while time.time() < t_end and not rospy.is_shutdown():
    time.sleep(0.1)
if not POSE:
    print('V6 tag=%s model_states=ABSENT' % TAG); raise SystemExit(1)

zs = [s[2] for s in POSE]
print('V6 tag=%s samples=%d' % (TAG, len(POSE)))
print('V6 tag=%s pose_x=%.6f pose_y=%.6f' % (TAG, POSE[-1][0], POSE[-1][1]))
print('V6 tag=%s z_min=%.6f z_max=%.6f z_mean=%.6f z_span=%.6f' % (TAG, min(zs), max(zs), sum(zs)/len(zs), max(zs)-min(zs)))
print('V6 tag=%s orient_xyzw=%s' % (TAG, ['%.6f' % v for v in POSE[-1][3:7]]))

try:
    mp = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
    mp.wait_for_service(10)
    r = mp(model_name='lite3_planar')
    print('V6 tag=%s model_ok=%s canonical_body_name=%s is_static=%s parent=%s'
          % (TAG, r.success, r.canonical_body_name, r.is_static, r.parent_model_name))
    print('V6 tag=%s body_names=%s' % (TAG, sorted(r.body_names)))
    print('V6 tag=%s geom_names=%s' % (TAG, sorted(r.geom_names)))
    print('V6 tag=%s joint_names=%s' % (TAG, sorted(r.joint_names)))
except Exception as e:
    print('V6 tag=%s model_props=ERR %s: %s' % (TAG, type(e).__name__, e))

lp = None
try:
    lp = rospy.ServiceProxy('/gazebo/get_link_properties', GetLinkProperties)
    lp.wait_for_service(10)
except Exception as e:
    print('V6 tag=%s link_props_service=ERR %s' % (TAG, type(e).__name__))
for link in ('lite3_planar::base_link', 'lite3_planar::TORSO', 'lite3_planar::rslidar'):
    if lp is None: break
    try:
        r = lp(link_name=link)
        if not r.success:
            print('V6 tag=%s link=%s found=False msg=%s' % (TAG, link, r.status_message.strip())); continue
        print('V6 tag=%s link=%s found=True mass=%.6f ixx=%.6f iyy=%.6f izz=%.6f ixy=%.6f ixz=%.6f iyz=%.6f'
              % (TAG, link, r.mass, r.ixx, r.iyy, r.izz, r.ixy, r.ixz, r.iyz))
        print('V6 tag=%s link=%s com=%.6f,%.6f,%.6f gravity_mode=%s'
              % (TAG, link, r.com.position.x, r.com.position.y, r.com.position.z, r.gravity_mode))
    except Exception as e:
        print('V6 tag=%s link=%s ERR %s: %s' % (TAG, link, type(e).__name__, e))
