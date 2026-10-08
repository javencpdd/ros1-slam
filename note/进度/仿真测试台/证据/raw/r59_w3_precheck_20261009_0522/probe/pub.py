import rospy, numpy as np
from nav_msgs.msg import OccupancyGrid
def mk(frame, vals):
    m=OccupancyGrid()
    m.header.frame_id=frame
    m.info.resolution=0.1; m.info.width=40; m.info.height=40
    m.info.origin.position.x=-2.0; m.info.origin.position.y=-2.0
    m.data=[int(v) for v in vals]
    return m
rospy.init_node('w3_precheck_pub')
a=np.zeros(40*40,dtype=np.int8); a[10*40+10:10*40+15]=100      # map_a：一块 100 的必杀区
a[20*40+20]=50                                                 # 以及一个 50 的代价点
b=np.zeros(40*40,dtype=np.int8); b[30*40:30*40+5]=-1           # map_b：一行 -1 未知
pa=rospy.Publisher('/pmap_a',OccupancyGrid,queue_size=1,latch=True); pb=rospy.Publisher('/pmap_b',OccupancyGrid,queue_size=1,latch=True)
r=rospy.Rate(5)
while not rospy.is_shutdown():
    pa.publish(mk('map',a)); pb.publish(mk('map',b)); r.sleep()
