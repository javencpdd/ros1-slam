"""在 m1 走廊加两条静态侧墙（y=±0.7，x 0—5.5），供全链停车试验。"""
import sys
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

rospy.init_node('spawn_corridor', disable_signals=True)


def box_sdf(name, sx, sy, sz):
    return ('<sdf version="1.6"><model name="%s"><static>true</static><link name="link">'
            '<collision name="c"><geometry><box><size>%g %g %g</size></box></geometry></collision>'
            '<visual name="v"><geometry><box><size>%g %g %g</size></box></geometry>'
            '<material><script><name>Gazebo/Grey</name></script></material></visual></link></model></sdf>'
            % (name, sx, sy, sz, sx, sy, sz))


def spawn(name, x, y, z, sx, sy, sz):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.w = 1.0
    rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=10)
    sp = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    try:
        r = sp(name, box_sdf(name, sx, sy, sz), '', pose, 'world')
        return r.success
    except Exception as e:
        print('spawn fail', name, e)
        return False


if len(sys.argv) > 1 and sys.argv[1] == 'delete':
    rospy.wait_for_service('/gazebo/delete_model', timeout=10)
    de = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    for n in ('corr_north', 'corr_south'):
        try:
            print(n, de(n).success)
        except Exception as e:
            print(n, 'skip', e)
else:
    print('north', spawn('corr_north', 2.75, 0.75, 0.5, 5.5, 0.1, 1.0))
    print('south', spawn('corr_south', 2.75, -0.75, 0.5, 5.5, 0.1, 1.0))
