import json, time, rospy, traceback
from sensor_msgs.msg import PointCloud2
rospy.init_node('sub_test', anonymous=True, disable_signals=True)
hits={'q1':0,'q10':0,'err':[]}
def mk(k):
    def cb(m):
        hits[k]+=1
        if hits[k]==1: hits[k+'_meta']={'frame':m.header.frame_id,'w':m.width,'h':m.height,'bytes':len(m.data)}
    return cb
rospy.Subscriber('/cloud_registered', PointCloud2, mk('q1'), queue_size=1)
rospy.Subscriber('/cloud_registered', PointCloud2, mk('q10'), queue_size=10, tcp_nodelay=True)
time.sleep(6)
# 时钟与连接
info=rospy.get_published_topics()
print(json.dumps({'hits':{k:v for k,v in hits.items() if k!='err'},'timesync':rospy.get_param('/use_sim_time',None),
                  'now':round(rospy.get_time(),2)}, ensure_ascii=False))
