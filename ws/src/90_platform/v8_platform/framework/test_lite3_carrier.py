"""Real carrier adapter over ROS/message stubs; no ROS/master or subprocess."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest import mock

HERE=Path(__file__).resolve().parent


class Stamp:
    def __init__(self,value=0.):self.value=value
    def to_sec(self):return self.value


class Pose:
    def __init__(self):
        self.header=types.SimpleNamespace(frame_id='map',stamp=Stamp(100.))
        self.pose=types.SimpleNamespace(position=types.SimpleNamespace(x=1.,y=2.,z=0.),
                                       orientation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.))


class Data:
    def __init__(self,data=None):self.data=data


class Publisher:
    def __init__(self,*args,**kwargs):self.messages=[];self.hook=None
    def publish(self,message):
        if self.hook:self.hook(message)
        self.messages.append(copy.deepcopy(message))


class CarrierTests(unittest.TestCase):
    def setUp(self):
        self.clock={'ros':100.,'wall':10.};self.params={};self.lookup_hook=None
        self.transform=types.SimpleNamespace(header=types.SimpleNamespace(frame_id='map',stamp=Stamp(100.)),
            child_frame_id='base_link',transform=types.SimpleNamespace(
                translation=types.SimpleNamespace(x=1.,y=2.,z=0.),
                rotation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
        outer=self
        class Buffer:
            def lookup_transform(self,*args):
                outer.lookup_args=args
                if outer.lookup_hook:outer.lookup_hook()
                if outer.transform is None:raise LookupError()
                return outer.transform
        rospy=types.ModuleType('rospy')
        rospy.get_param=lambda key,default=None:self.params.get(key,default)
        rospy.Publisher=Publisher;rospy.Subscriber=lambda *a,**kw:None
        rospy.Time=type('Time',(Stamp,),{'now':staticmethod(lambda:Stamp(self.clock['ros']))})
        rospy.Duration=lambda value:value
        rospy.is_shutdown=lambda:False
        rospy.Rate=mock.Mock(side_effect=AssertionError('ROS Rate must not schedule the watchdog'))
        rospy.sleep=mock.Mock(side_effect=AssertionError('ROS sleep must not schedule the watchdog'))
        tf2=types.ModuleType('tf2_ros');tf2.Buffer=Buffer;tf2.TransformListener=lambda b:None
        tf2.LookupException=LookupError;tf2.ConnectivityException=ConnectionError;tf2.ExtrapolationException=ArithmeticError
        geometry=types.ModuleType('geometry_msgs.msg');geometry.PoseStamped=Pose
        nav=types.ModuleType('nav_msgs.msg');nav.Odometry=object
        std=types.ModuleType('std_msgs.msg');std.Bool=Data;std.String=Data
        modules={'rospy':rospy,'tf2_ros':tf2,'geometry_msgs':types.ModuleType('geometry_msgs'),
                 'geometry_msgs.msg':geometry,'nav_msgs':types.ModuleType('nav_msgs'),'nav_msgs.msg':nav,
                 'std_msgs':types.ModuleType('std_msgs'),'std_msgs.msg':std}
        with mock.patch.dict(sys.modules,modules):
            spec=importlib.util.spec_from_file_location('carrier_under_test',HERE/'lite3_carrier_node.py')
            self.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.module)
        self.rospy=rospy
        self.mono=mock.patch.object(self.module.time,'monotonic',side_effect=lambda:self.clock['wall'])
        self.mono.start();self.addCleanup(self.mono.stop)

    def odom(self,frame='map',child='base_link',stamp=100.):
        return types.SimpleNamespace(header=types.SimpleNamespace(frame_id=frame,stamp=Stamp(stamp)),
                                     child_frame_id=child,pose=types.SimpleNamespace(pose=Pose().pose))

    def test_valid_tf_uses_original_stamp_and_nonblocking_query(self):
        carrier=self.module.Lite3Carrier();self.assertTrue(carrier.step())
        self.assertEqual(self.lookup_args[-1],0.)
        self.assertEqual(carrier.pose_pub.messages[-1].header.stamp.to_sec(),100.)
        self.assertTrue(carrier.loc_pub.messages[-1].data)

    def test_lookup_after_clock_sampling_has_no_artificial_future(self):
        def arrive():
            self.clock['ros']=100.02;self.transform.header.stamp=Stamp(100.01)
        self.lookup_hook=arrive
        self.assertTrue(self.module.Lite3Carrier().step())

    def test_old_future_zero_and_wrong_tf_frames_never_publish_pose(self):
        for stamp,parent,child in ((1.,'map','base_link'),(101.,'map','base_link'),
                (0.,'map','base_link'),(100.,'odom','base_link'),(100.,'map','lidar')):
            with self.subTest(stamp=stamp,parent=parent,child=child):
                self.transform.header.stamp=Stamp(stamp);self.transform.header.frame_id=parent
                self.transform.child_frame_id=child
                carrier=self.module.Lite3Carrier();self.assertFalse(carrier.step())
                self.assertFalse(carrier.pose_pub.messages);self.assertFalse(carrier.loc_pub.messages[-1].data)

    def test_duplicate_tf_cannot_renew_during_paused_ros_clock(self):
        carrier=self.module.Lite3Carrier();self.assertTrue(carrier.step())
        self.clock['wall']=10.49;self.assertTrue(carrier.step())
        count=len(carrier.pose_pub.messages)
        self.clock['wall']=10.5;self.assertFalse(carrier.step())
        self.assertEqual(len(carrier.pose_pub.messages),count)
        self.assertEqual(json.loads(carrier.status_pub.messages[-1].data)['reason'],'E_POSE_NO_PROGRESS')
        self.clock.update(ros=100.1,wall=10.6);self.transform.header.stamp=Stamp(100.1)
        self.assertTrue(carrier.step())

    def test_rollback_invalidates_then_new_epoch_sample_recovers(self):
        carrier=self.module.Lite3Carrier();self.assertTrue(carrier.step())
        self.clock.update(ros=50.,wall=10.1);self.transform.header.stamp=Stamp(50.)
        self.assertFalse(carrier.step());self.assertEqual(carrier.guard.epoch,1)
        self.clock.update(ros=50.1,wall=10.2);self.transform.header.stamp=Stamp(50.1)
        self.assertTrue(carrier.step())

    def test_odom_relabel_zero_child_and_invalid_geometry_rejected(self):
        self.params['~pose_source']='topic'
        for message in (self.odom(frame='odom'),self.odom(child=''),self.odom(stamp=0.)):
            carrier=self.module.Lite3Carrier();carrier.on_odom(message)
            self.assertFalse(carrier.step());self.assertFalse(carrier.pose_pub.messages)
        for bad in (float('nan'),float('inf')):
            carrier=self.module.Lite3Carrier();message=self.odom();message.pose.pose.position.x=bad
            carrier.on_odom(message);self.assertFalse(carrier.step())
        carrier=self.module.Lite3Carrier();message=self.odom();message.pose.pose.orientation.w=0.
        carrier.on_odom(message);self.assertFalse(carrier.step())

    def test_lookup_failure_and_source_regression_need_new_measurement(self):
        carrier=self.module.Lite3Carrier();self.assertTrue(carrier.step())
        original=self.transform;self.transform=None
        self.assertFalse(carrier.step())
        self.transform=original;self.assertFalse(carrier.step())
        self.clock.update(ros=100.1,wall=10.1);self.transform.header.stamp=Stamp(100.1)
        self.assertTrue(carrier.step())
        self.transform.header.stamp=Stamp(100.)
        self.assertFalse(carrier.step())
        self.transform.header.stamp=Stamp(100.1);self.assertFalse(carrier.step())
        self.clock.update(ros=100.2,wall=10.2);self.transform.header.stamp=Stamp(100.2)
        self.assertTrue(carrier.step())

    def test_valid_topic_preserves_frame_stamp_and_normalizes_without_mutation(self):
        self.params['~pose_source']='topic';carrier=self.module.Lite3Carrier();message=self.odom()
        message.pose.pose.orientation.w=3.
        carrier.on_odom(message);self.assertTrue(carrier.step())
        self.assertEqual(message.pose.pose.orientation.w,3.)
        published=carrier.pose_pub.messages[-1]
        self.assertEqual(published.pose.orientation.w,1.)
        self.assertEqual(published.header.frame_id,'map');self.assertEqual(published.header.stamp.to_sec(),100.)

    def test_topic_fault_needs_new_source_progress_not_duplicate(self):
        self.params['~pose_source']='topic';carrier=self.module.Lite3Carrier()
        carrier.on_odom(self.odom());self.assertTrue(carrier.step())
        carrier.on_odom(self.odom(frame='odom'));self.assertFalse(carrier.step())
        carrier.on_odom(self.odom());self.assertFalse(carrier.step())
        self.clock.update(ros=100.1,wall=10.1);carrier.on_odom(self.odom(stamp=100.1))
        self.assertTrue(carrier.step())

    def test_future_tf_requery_does_not_heal_when_ros_clock_catches_up(self):
        carrier=self.module.Lite3Carrier();self.assertTrue(carrier.step())
        self.transform.header.stamp=Stamp(101.);self.clock.update(ros=100.1,wall=10.1)
        self.assertFalse(carrier.step());count=len(carrier.pose_pub.messages)
        self.clock.update(ros=101.1,wall=11.1)
        self.assertFalse(carrier.step());self.assertEqual(len(carrier.pose_pub.messages),count)
        self.assertEqual(carrier.guard.progress_at,10.)
        self.transform.header.stamp=Stamp(101.2);self.clock.update(ros=101.2,wall=11.2)
        self.assertTrue(carrier.step())

    def test_tf_new_epoch_can_cross_previous_stamp_without_blacklist(self):
        carrier=self.module.Lite3Carrier();self.assertTrue(carrier.step())
        self.clock.update(ros=20.,wall=10.1);self.assertFalse(carrier.step())
        for stamp,wall in ((20.1,10.2),(99.9,10.3),(100.,10.4)):
            self.transform.header.stamp=Stamp(stamp);self.clock.update(ros=stamp,wall=wall)
            self.assertTrue(carrier.step())

    def test_old_tf_cache_cannot_restore_after_clock_reset_and_catchup(self):
        carrier=self.module.Lite3Carrier();self.assertTrue(carrier.step())
        with carrier.lock:carrier.guard.check(20.,10.1)
        self.clock.update(ros=100.,wall=90.)
        self.assertFalse(carrier.step())
        self.assertEqual(carrier.guard.reason,'E_POSE_EPOCH_REPLAY')
        self.clock.update(ros=100.1,wall=90.1);self.transform.header.stamp=Stamp(100.1)
        self.assertTrue(carrier.step())

    def test_bad_frame_source_cannot_be_reused_after_merely_fixing_label(self):
        carrier=self.module.Lite3Carrier();self.assertTrue(carrier.step())
        self.clock.update(ros=100.1,wall=10.1);self.transform.header.stamp=Stamp(100.1)
        self.transform.header.frame_id='odom';self.assertFalse(carrier.step())
        self.transform.header.frame_id='map';self.assertFalse(carrier.step())
        self.clock.update(ros=100.2,wall=10.2);self.transform.header.stamp=Stamp(100.2)
        self.assertTrue(carrier.step())

    def test_spin_evaluates_wall_timeout_while_ros_time_stays_paused(self):
        carrier=self.module.Lite3Carrier();iterations=[]
        def sleep(period):
            self.clock['wall']+=.25;iterations.append(period)
        with mock.patch.object(self.module.time,'sleep',side_effect=sleep), \
                mock.patch.object(self.rospy,'is_shutdown',side_effect=lambda:len(iterations)>=4):
            carrier.spin()
        self.assertEqual([m.data for m in carrier.loc_pub.messages],[True,True,False,False,False])
        self.assertTrue(all(0.<=p<=.05 for p in iterations))
        self.rospy.Rate.assert_not_called();self.rospy.sleep.assert_not_called()

    def test_publish_and_invalidation_are_serialized(self):
        self.params['~pose_source']='topic';carrier=self.module.Lite3Carrier();carrier.on_odom(self.odom())
        inside=threading.Event();release=threading.Event();callback_started=threading.Event();invalid_done=threading.Event()
        events=[]
        def held_publish(message):
            events.append('pose_publish');inside.set();self.assertTrue(release.wait(2.))
        carrier.pose_pub.hook=held_publish
        def invalid():
            callback_started.set();carrier.on_odom(self.odom(frame='odom'));events.append('invalid_done');invalid_done.set()
        step=threading.Thread(target=carrier.step);callback=threading.Thread(target=invalid)
        step.start();self.assertTrue(inside.wait(2.));callback.start();self.assertTrue(callback_started.wait(2.))
        try:self.assertFalse(invalid_done.wait(.05))
        finally:
            release.set();step.join(2.);callback.join(2.)
        self.assertFalse(step.is_alive());self.assertFalse(callback.is_alive());self.assertTrue(invalid_done.is_set())
        count=len(carrier.pose_pub.messages);self.assertFalse(carrier.step())
        self.assertEqual(len(carrier.pose_pub.messages),count);self.assertFalse(carrier.loc_pub.messages[-1].data)
        self.assertEqual(events,['pose_publish','invalid_done'])


if __name__=='__main__':unittest.main()
