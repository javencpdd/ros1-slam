import json, time, rospy, struct
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnModel
rospy.init_node('obst_diag', anonymous=True, disable_signals=True)
st={'cloud':None,'status':None,'obst':None,'pose':None}
rospy.Subscriber('/cloud_registered', PointCloud2, lambda m: st.update(cloud=m), queue_size=1)
rospy.Subscriber('/v8/3d/perception/status', String, lambda m: st.update(status=m.data[:200]))
rospy.Subscriber('/v8/local/obstacles', String, lambda m: st.update(obst=m.data[:200]))
time.sleep(2)
sdf=('<sdf version="1.6"><model name="p4_box2"><static>true</static><link name="link">'
     '<collision name="c"><geometry><box><size>0.6 0.6 1.0</size></box></geometry></collision>'
     '<visual name="v"><geometry><box><size>0.6 0.6 1.0</size></box></geometry></visual></link></model></sdf>')
rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=5)
sp=rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
p=Pose(); p.position.x=1.1; p.position.y=-3.06; p.position.z=0.5; p.orientation.w=1.0
print('spawn', sp('p4_box2', sdf, '', p, 'world').success)
time.sleep(3)
m=st['cloud']
pts=None
if m is not None:
    n=m.width*m.height; step=m.point_step
    data=m.data; near=0
    for i in range(0, min(n, 3000)):
        off=i*step
        try:
            x,y,z=struct.unpack_from('fff', data, off)
        except Exception:
            break
        if abs(x)<2.0 and abs(y)<2.0:
            near+=1
    pts={'w':m.width,'h':m.height,'frame':m.header.frame_id,'near_2m':near}
print(json.dumps({'cloud':pts,'status':st['status'],'obst':st['obst']}, ensure_ascii=False))
