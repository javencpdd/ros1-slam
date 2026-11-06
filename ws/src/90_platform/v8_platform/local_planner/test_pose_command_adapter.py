"""Real local input/control/publish paths with injected ROS and clocks only."""
import copy
import json
import math
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest import mock

sys.path.insert(0,str(Path(__file__).resolve().parent))
import ros_node as node


class Stamp:
    def __init__(self,value=0.):self.value=value
    def to_sec(self):return self.value


class Twist:
    def __init__(self):
        self.linear=types.SimpleNamespace(x=0.,y=0.,z=0.)
        self.angular=types.SimpleNamespace(x=0.,y=0.,z=0.)


class Stamped:
    def __init__(self):
        self.header=types.SimpleNamespace(frame_id='',stamp=Stamp(),seq=0)
        self.twist=Twist()


class Data:
    def __init__(self,data=None):self.data=data


class Publisher:
    def __init__(self,*args,**kwargs):self.messages=[];self.hook=None
    def publish(self,message):
        if self.hook:self.hook(message)
        self.messages.append(copy.deepcopy(message))


def pose(stamp=100.,yaw=0.,frame='map',q=None):
    values=q if q is not None else (0.,0.,math.sin(yaw/2.),math.cos(yaw/2.))
    return types.SimpleNamespace(header=types.SimpleNamespace(frame_id=frame,stamp=Stamp(stamp)),
        pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.,y=0.,z=0.),
                                  orientation=types.SimpleNamespace(**dict(zip(('x','y','z','w'),values)))))


def moving(velocity=(1.,0.,0.)):
    return dict(velocity=list(velocity),reason='track',stop=False,complete=False)


class LocalAdapterTests(unittest.TestCase):
    def fixture(self,body=True,yaw=0.):
        self.clock={'ros':100.,'wall':10.}
        self.shared=dict(pose=None,goal=(1.,0.,0.),pose_at=-1e100,pose_ros_stamp=0.,
            obstacles=[],obstacles_at=10.,localization_at=10.,localization_ok=True,
            result={'valid_until':101.},result_at=9.9,map={},generation=0,result_generation=0,
            last_error='ok',stop_latched=False,stop_reason=None,plan_ms=None,plan_ms_max=0.,map_version=1)
        self.lock=threading.Lock();self.input=node.LocalPoseInput(self.shared,1.,body)
        self.assertIsNone(self.input.accept(pose(yaw=yaw),100.,10.))
        self.command,self.twist,self.status=Publisher(),Publisher() if body else None,Publisher()
        self.control=node.LocalControl(self.shared,self.input,self.lock,node.native.Config(),1.,7.5,
            self.command,self.twist,self.status,(Stamped,Twist,Data))

    def tick(self,**kwargs):
        return self.control.tick(lambda:self.clock['ros'],lambda:self.clock['wall'],Stamp,**kwargs)

    def assert_zero(self):
        for message in (self.command.messages[-1].twist,self.twist.messages[-1] if self.twist else Twist()):
            self.assertEqual((message.linear.x,message.linear.y,message.linear.z,
                              message.angular.x,message.angular.y,message.angular.z),(0.,)*6)
        self.assertEqual(self.control.history[-1][1],(0.,0.,0.))

    def test_real_publish_rotates_body_and_keeps_stamped_map(self):
        self.fixture(yaw=math.pi/2.)
        with mock.patch.object(node.native,'track_segment',return_value=moving()):out=self.tick()
        self.assertEqual(out['velocity'],[1.,0.,0.]);self.assertEqual(self.command.messages[-1].header.frame_id,'map')
        self.assertEqual(self.command.messages[-1].twist.linear.x,1.)
        self.assertAlmostEqual(self.twist.messages[-1].linear.x,0.)
        self.assertAlmostEqual(self.twist.messages[-1].linear.y,-1.)
        self.assertEqual(self.control.history[-1][1],(1.,0.,0.))

    def test_mirror_off_preserves_map_three_dimensional_motion(self):
        self.fixture(body=False)
        self.input.accept(pose(stamp=100.1,q=(0.,0.,0.,0.)),100.1,10.1)
        self.clock.update(ros=100.1,wall=10.1)
        with mock.patch.object(node.native,'track_segment',return_value=moving((.2,.3,.1))):out=self.tick()
        self.assertEqual(out['velocity'],[.2,.3,.1]);self.assertEqual(self.command.messages[-1].twist.linear.z,.1)

    def test_invalid_pose_or_source_produces_real_zero_messages(self):
        for message in (pose(q=(0.,0.,0.,0.)),pose(q=(0.,math.nan,0.,1.)),pose(q=(False,0.,0.,1.)),
                        pose(stamp=0.),pose(stamp=99.),pose(stamp=101.),pose(frame='odom')):
            with self.subTest(message=message):
                self.fixture();self.input.accept(message,100.,10.)
                with mock.patch.object(node.native,'track_segment') as tracking:self.tick()
                tracking.assert_not_called();self.assert_zero()

    def test_vertical_nonfinite_and_overflow_never_escape_dual_publish(self):
        for velocity,yaw in (((1.,0.,.01),0.),((math.inf,0.,0.),0.),((True,0.,0.),0.),
                             ((1.7e308,1.7e308,0.),math.pi/4.),((10**1000,0.,0.),0.)):
            with self.subTest(velocity=velocity):
                self.fixture(yaw=yaw)
                with mock.patch.object(node.native,'track_segment',return_value=moving(velocity)):self.tick()
                self.assert_zero()
        self.fixture()
        with mock.patch.object(node.native,'track_segment',return_value=moving((1.,0.,1e-6))):self.tick()
        self.assertEqual(self.command.messages[-1].twist.linear.z,0.)
        self.assertEqual(self.control.history[-1][1],(1.,0.,0.))

    def test_bad_pose_during_tracking_rejects_snapshot_before_publish(self):
        self.fixture()
        def race(*args,**kwargs):
            with self.lock:self.input.accept(pose(frame='odom'),100.,10.)
            return moving()
        with mock.patch.object(node.native,'track_segment',side_effect=race):out=self.tick()
        self.assert_zero();self.assertEqual(out['reason'],'E_LOCAL_FRAME')
        self.assertIsNone(self.shared['result'])

    def test_epoch_during_tracking_clears_history_and_pending_generation(self):
        self.fixture();self.control.history[:]=[(99.9,(1.,0.,0.))]
        generation=self.shared['generation']
        def race(*args,**kwargs):
            self.clock.update(ros=50.,wall=10.1)
            with self.lock:self.input.accept(pose(stamp=50.),50.,10.1)
            return moving()
        with mock.patch.object(node.native,'track_segment',side_effect=race):self.tick()
        self.assert_zero();self.assertEqual(self.control.history,[(50.,(0.,0.,0.))])
        self.assertFalse(node.accept_worker_result(self.shared,dict(generation=generation,ok=True,result={'valid_until':999.}),10.2))
        self.assertIsNone(self.shared['result'])

    def test_invalid_frame_clears_nonzero_history_before_next_prediction(self):
        self.fixture()
        with mock.patch.object(node.native,'track_segment',return_value=moving()):self.tick()
        self.input.accept(pose(frame='odom'),100.1,10.1)
        self.clock.update(ros=100.1,wall=10.1)
        self.tick();self.assert_zero()
        self.assertEqual(self.control.history,[(100.1,(0.,0.,0.))])
        self.assertEqual(self.control.previous_command,(0.,0.,0.))

    def test_invalid_and_recovered_pose_between_ticks_cannot_reuse_old_history(self):
        self.fixture()
        with mock.patch.object(node.native,'track_segment',return_value=moving()):self.tick()
        self.input.accept(pose(stamp=100.1,frame='odom'),100.1,10.1)
        self.assertIsNone(self.input.accept(pose(stamp=100.2),100.2,10.2))
        self.clock.update(ros=100.2,wall=10.2)
        payload=dict(generation=self.shared['generation'],ok=True,result={'valid_until':101.})
        self.assertTrue(node.accept_worker_result(self.shared,payload,10.2))
        def tracking(*args,**kwargs):
            self.assertEqual(self.control.history,[])
            self.assertEqual(kwargs['previous_velocity'],(0.,0.,0.))
            return moving()
        with mock.patch.object(node.native,'track_segment',side_effect=tracking):self.tick()
        self.assertEqual(self.control.history,[(100.2,(1.,0.,0.))])

    def test_invalid_planner_output_always_reaches_dual_zero_publication(self):
        for output in (None,[],{},dict(velocity=(1.,0.,0.),reason=None),
                       dict(velocity=(10**1000,0.,0.),reason='track',stop=False)):
            self.fixture()
            with mock.patch.object(node.native,'track_segment',return_value=output):self.tick()
            self.assert_zero()
            self.assertEqual(len(self.command.messages),1)
            self.assertEqual(len(self.twist.messages),1)

    def test_source_fault_needs_new_sample_and_new_epoch_cache_does_not_recover(self):
        self.fixture()
        self.assertIsNotNone(self.input.accept(pose(stamp=101.),100.1,10.1))
        self.assertIsNotNone(self.input.accept(pose(stamp=101.),101.1,11.1))
        self.assertIsNone(self.input.accept(pose(stamp=101.2),101.2,11.2))
        self.input.check(20.,11.3)
        self.assertIsNotNone(self.input.accept(pose(stamp=101.2),101.2,91.2))
        self.assertIsNone(self.input.accept(pose(stamp=101.3),101.3,91.3))

    def test_worker_serialization_race_cannot_publish_revoked_result(self):
        self.fixture();path=Publisher()
        payload=dict(generation=self.shared['generation'],ok=True,result={'valid_until':101.})
        dumps=node.json.dumps
        def invalidate_during_serialization(*args,**kwargs):
            with self.lock:self.input.accept(pose(frame='odom'),100.,10.)
            return dumps(*args,**kwargs)
        with mock.patch.object(node.json,'dumps',side_effect=invalidate_during_serialization):
            accepted=node.publish_worker_result(self.shared,payload,self.input,self.lock,
                lambda:100.,lambda:10.,path,Data)
        self.assertFalse(accepted);self.assertEqual(path.messages,[])
        self.assertIsNone(self.shared['result'])

    def test_worker_publication_excludes_pose_revocation_callback(self):
        self.fixture();path=Publisher();entered=threading.Event();release=threading.Event()
        callback_started=threading.Event();invalid_done=threading.Event();errors=[]
        payload=dict(generation=self.shared['generation'],ok=True,result={'valid_until':101.})
        def held_publish(message):entered.set();self.assertTrue(release.wait(2.))
        path.hook=held_publish
        def publish():
            try:node.publish_worker_result(self.shared,payload,self.input,self.lock,
                lambda:100.,lambda:10.,path,Data)
            except BaseException as error:errors.append(error)
        def invalidate():
            callback_started.set()
            with self.lock:self.input.accept(pose(frame='odom'),100.,10.)
            invalid_done.set()
        publisher=threading.Thread(target=publish);publisher.start();self.assertTrue(entered.wait(2.))
        writer=threading.Thread(target=invalidate);writer.start();self.assertTrue(callback_started.wait(2.))
        try:self.assertFalse(invalid_done.wait(.05))
        finally:release.set();publisher.join(2.);writer.join(2.)
        self.assertFalse(publisher.is_alive());self.assertFalse(writer.is_alive())
        self.assertEqual(errors,[]);self.assertTrue(invalid_done.is_set())
        self.assertEqual(len(path.messages),1);self.assertIsNone(self.shared['result'])

    def test_latest_valid_yaw_is_used_without_invalidating_worker(self):
        self.fixture();generation=self.shared['generation']
        def race(*args,**kwargs):
            self.clock.update(ros=100.1,wall=10.1)
            with self.lock:self.input.accept(pose(stamp=100.1,yaw=math.pi/2.),100.1,10.1)
            return moving()
        with mock.patch.object(node.native,'track_segment',side_effect=race):self.tick()
        self.assertEqual(self.shared['generation'],generation)
        self.assertAlmostEqual(self.twist.messages[-1].linear.x,0.)
        self.assertAlmostEqual(self.twist.messages[-1].linear.y,-1.)

    def test_expiry_during_tracking_is_rechecked_at_publication(self):
        self.fixture()
        def expire(*args,**kwargs):self.clock['wall']=11.;return moving()
        with mock.patch.object(node.native,'track_segment',side_effect=expire):self.tick()
        self.assert_zero()

    def test_new_source_recovers_same_goal_but_old_worker_does_not(self):
        self.fixture();goal=self.shared['goal'];old_generation=self.shared['generation']
        self.input.accept(pose(frame='odom'),100.,10.)
        self.assertIsNotNone(self.input.accept(pose(),100.,10.1))
        self.clock.update(ros=100.2,wall=10.2)
        self.assertIsNone(self.input.accept(pose(stamp=100.2),100.2,10.2))
        payload=dict(generation=old_generation,ok=True,result={'valid_until':101.})
        self.assertFalse(node.accept_worker_result(self.shared,payload,10.2))
        payload['generation']=self.shared['generation']
        self.assertTrue(node.accept_worker_result(self.shared,payload,10.2))
        self.assertIs(self.shared['goal'],goal)
        with mock.patch.object(node.native,'track_segment',return_value=moving()):out=self.tick()
        self.assertFalse(out['stop']);self.assertEqual(self.twist.messages[-1].linear.x,1.)

    def test_lockstep_timeout_publishes_zero_instead_of_waiting_for_clock(self):
        self.fixture()
        with mock.patch.object(node.native,'track_segment') as track:out=self.tick(clock_ready=False,clock_stamp=100.)
        track.assert_not_called();self.assert_zero();self.assertEqual(out['reason'],'E_LOCAL_CLOCK')

    def test_invalid_result_lease_cannot_kill_controller_or_publish_motion(self):
        for result in ([],{}, {'valid_until':float('nan')},{'valid_until':float('inf')}):
            with self.subTest(result=result):
                self.fixture();self.shared['result']=result
                with mock.patch.object(node.native,'track_segment') as track:out=self.tick()
                track.assert_not_called();self.assert_zero();self.assertEqual(out['reason'],'E_LOCAL_INPUT')
                self.assertFalse(node.accept_worker_result(self.shared,dict(
                    ok=True,generation=self.shared['generation'],result=result),10.))

    def test_result_lease_is_rechecked_after_tracking_with_fresh_pose(self):
        for kind in ('ros','wall'):
            with self.subTest(kind=kind):
                self.fixture()
                if kind=='ros':self.shared['result']['valid_until']=100.1
                def expire(*args,**kwargs):
                    self.clock.update(ros=100.2,wall=18. if kind=='wall' else 10.2)
                    with self.lock:
                        self.input.accept(pose(stamp=100.19),100.2,self.clock['wall'])
                        self.shared.update(obstacles_at=self.clock['wall'],localization_at=self.clock['wall'])
                    return moving()
                with mock.patch.object(node.native,'track_segment',side_effect=expire):out=self.tick()
                self.assert_zero()
                self.assertEqual(out['reason'],'E_LOCAL_WALL_EXPIRED' if kind=='wall' else 'E_LOCAL_EXPIRED')
                self.assertEqual(self.command.messages[-1].header.stamp.to_sec(),100.2)

    def test_history_and_header_use_publication_time_after_calculation(self):
        self.fixture()
        def advance(*args,**kwargs):
            self.clock.update(ros=100.2,wall=10.2)
            with self.lock:self.input.accept(pose(stamp=100.19),100.2,10.2)
            return moving()
        with mock.patch.object(node.native,'track_segment',side_effect=advance):out=self.tick()
        self.assertFalse(out['stop'])
        self.assertEqual(self.control.history,[(100.2,(1.,0.,0.))])
        self.assertEqual(self.control.previous_stamp,100.2)
        self.assertEqual(self.command.messages[-1].header.stamp.to_sec(),100.2)

    def test_existing_terminal_latch_survives_pose_fault_and_needs_actual_new_goal(self):
        self.fixture();self.shared.update(stop_latched=True,stop_reason='E_LOCAL_COLLISION')
        self.input.accept(pose(frame='odom'),100.,10.)
        self.tick();self.assert_zero();self.assertTrue(self.shared['stop_latched'])
        self.assertEqual(self.shared['stop_reason'],'E_LOCAL_COLLISION')
        self.clock.update(ros=100.1,wall=10.1)
        self.input.accept(pose(stamp=100.1),100.1,10.1)
        payload=dict(generation=self.shared['generation'],ok=True,result={'valid_until':101.})
        self.assertFalse(node.accept_worker_result(self.shared,payload,10.1))
        self.tick();self.assert_zero();self.assertTrue(self.shared['stop_latched'])
        self.assertTrue(node.accept_goal(self.shared,pose(stamp=100.1)))
        self.assertFalse(self.shared['stop_latched'])
        self.assertFalse(node.accept_worker_result(self.shared,payload,10.1))
        payload['generation']=self.shared['generation']
        self.assertTrue(node.accept_worker_result(self.shared,payload,10.1))
        with mock.patch.object(node.native,'track_segment',return_value=moving()):out=self.tick()
        self.assertFalse(out['stop']);self.assertEqual(self.twist.messages[-1].linear.x,1.)

    def test_inconsistent_clock_event_cannot_track_before_pose_or_in_future(self):
        for stamp in (99.9,100.1):
            self.fixture()
            with mock.patch.object(node.native,'track_segment') as track:self.tick(clock_stamp=stamp)
            track.assert_not_called();self.assert_zero()

    def test_local_duplicate_pose_delivery_expires_without_source_progress(self):
        self.fixture();self.input.accept(pose(),100.,10.9)
        self.assertEqual(self.shared['pose_at'],10.)
        self.clock['wall']=11.
        with mock.patch.object(node.native,'track_segment') as track:self.tick()
        track.assert_not_called();self.assert_zero()

    def test_callback_cannot_complete_inside_publish_history_boundary(self):
        self.fixture();inside=threading.Event();release=threading.Event();started=threading.Event();done=threading.Event()
        def publisher_hook(message):inside.set();self.assertTrue(release.wait(2.))
        self.command.hook=publisher_hook
        def callback():
            started.set()
            with self.lock:self.input.accept(pose(frame='odom'),100.,10.)
            done.set()
        with mock.patch.object(node.native,'track_segment',return_value=moving()):
            control=threading.Thread(target=self.tick);control.start();self.assertTrue(inside.wait(2.))
            writer=threading.Thread(target=callback);writer.start();self.assertTrue(started.wait(2.))
            try:self.assertFalse(done.wait(.05))
            finally:release.set();control.join(2.);writer.join(2.)
        self.assertFalse(control.is_alive());self.assertFalse(writer.is_alive());self.assertTrue(done.is_set())
        self.command.hook=None;self.tick();self.assert_zero()


class MainLoopTests(unittest.TestCase):
    def test_real_main_event_timeout_reaches_a_well_formed_zero_publish(self):
        done={'value':False};pubs={};waits=[]
        class Event:
            def wait(self,timeout):
                waits.append(timeout)
                if len(waits)>1:done['value']=True
                return False
            def set(self):pass
            def clear(self):pass
        class Thread:
            def __init__(self,target,**kwargs):self.target=target
            def start(self):self.target()
            def join(self,**kwargs):pass
        class Worker:
            def __init__(self,*args):self.closed=False;self.pending=None
            def close(self):self.closed=True
        def publisher(topic,*args,**kwargs):pubs[topic]=Publisher();return pubs[topic]
        rospy=types.ModuleType('rospy');rospy.init_node=mock.Mock();rospy.Publisher=publisher
        params={'~map_topic':'/map','~lockstep_command':True,'~twist_topic':'/cmd_vel_raw'}
        rospy.get_param=lambda name,default=None:params.get(name,default)
        rospy.Time=types.SimpleNamespace(now=lambda:Stamp(100.),from_sec=Stamp)
        rospy.Subscriber=lambda *a,**kw:None;rospy.is_shutdown=lambda:done['value']
        geometry=types.ModuleType('geometry_msgs.msg');geometry.PoseStamped=object;geometry.Twist=Twist;geometry.TwistStamped=Stamped
        std=types.ModuleType('std_msgs.msg');std.Bool=Data;std.String=Data
        clock=types.ModuleType('rosgraph_msgs.msg');clock.Clock=object
        modules={'rospy':rospy,'geometry_msgs':types.ModuleType('geometry_msgs'),'geometry_msgs.msg':geometry,
                 'std_msgs':types.ModuleType('std_msgs'),'std_msgs.msg':std,
                 'rosgraph_msgs':types.ModuleType('rosgraph_msgs'),'rosgraph_msgs.msg':clock}
        with mock.patch.dict(sys.modules,modules),mock.patch.object(node.threading,'Event',Event), \
                mock.patch.object(node.threading,'Thread',Thread),mock.patch.object(node,'PlannerWorker',Worker), \
                mock.patch.object(node.time,'sleep',side_effect=lambda period:done.update(value=True)):
            node.main()
        self.assertEqual(waits,[1.])
        first=pubs['/v8/local/cmd_vel_3d'].messages[0]
        self.assertEqual(first.header.frame_id,'map')
        self.assertEqual((first.twist.linear.x,first.twist.linear.y,first.twist.linear.z),(0.,0.,0.))
        self.assertEqual(pubs['/cmd_vel_raw'].messages[0].linear.x,0.)
        self.assertEqual(json.loads(pubs['/v8/local/status'].messages[0].data)['reason'],'E_LOCAL_CLOCK')


if __name__=='__main__':unittest.main()
