import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import xmlrpc.client

ROOT = Path('/opt/ros1-slam/r625')
RUN = ROOT / ('runtime-' + os.environ['R625_RUN_TOKEN'])
RUN.mkdir()
processes = []
result = {'scope': 'stationary synthetic geometry; real Gazebo ray and IMU; plugin build/load only, not SLAM or navigation acceptance',
          'run': str(RUN), 'errors': [], 'checks': {}, 'environment': dict(os.environ)}


def start(name, args):
    stream = (RUN / (name + '.log')).open('x')
    process = subprocess.Popen(args, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append((name, process, stream))
    return process


def wait_until(predicate, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return True
        threading.Event().wait(0.1)
    return False


def fixtures():
    world = '''<sdf version="1.6"><world name="r625">
<gravity>0 0 -9.8</gravity><physics type="ode"><max_step_size>0.002</max_step_size><real_time_update_rate>500</real_time_update_rate></physics>
<model name="wall"><static>true</static><pose>3 0 1 0 0 0</pose><link name="wall"><collision name="collision"><geometry><box><size>1 2 2</size></box></geometry></collision></link></model>
<model name="side"><static>true</static><pose>1 3 1 0 0 0</pose><link name="side"><collision name="collision"><geometry><box><size>1 1 2</size></box></geometry></collision></link></model>
<model name="sensor_rig"><static>true</static><pose>0 0 0.5 0 0 0</pose><link name="base">
<inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
<sensor name="laser" type="ray"><always_on>true</always_on><update_rate>10</update_rate><visualize>false</visualize>
<ray><scan><horizontal><samples>181</samples><resolution>1</resolution><min_angle>-1.57079632679</min_angle><max_angle>1.57079632679</max_angle></horizontal></scan><range><min>0.1</min><max>10</max><resolution>0.01</resolution></range></ray>
<plugin name="laser_ros" filename="libgazebo_ros_laser.so"><robotNamespace>/r625</robotNamespace><topicName>scan</topicName><frameName>lidar</frameName></plugin></sensor>
<sensor name="imu" type="imu"><always_on>true</always_on><update_rate>100</update_rate>
<plugin name="imu_ros" filename="libgazebo_ros_imu_sensor.so"><robotNamespace>/r625</robotNamespace><topicName>imu</topicName><frameName>imu</frameName><bodyName>base</bodyName><gaussianNoise>0</gaussianNoise><updateRateHZ>100</updateRateHZ><xyzOffset>0 0 0</xyzOffset><rpyOffset>0 0 0</rpyOffset><initialOrientationAsReference>false</initialOrientationAsReference></plugin></sensor>
</link></model></world></sdf>'''
    (RUN / 'scene.world').write_text(world)
    pixels = bytearray()
    for row in range(100):
        y = 5 - (row + 0.5) * 0.1
        for col in range(100):
            x = -5 + (col + 0.5) * 0.1
            blocked = (2.5 <= x <= 3.5 and -1 <= y <= 1) or (0.5 <= x <= 1.5 and 2.5 <= y <= 3.5)
            pixels.append(0 if blocked else 254)
    (RUN / 'map.pgm').write_bytes(b'P5\n100 100\n255\n' + pixels)
    (RUN / 'map.yaml').write_text('image: map.pgm\nresolution: 0.1\norigin: [-5.0, -5.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    result['fixture_hashes'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in RUN.iterdir() if p.is_file()}


def execute():
    fixtures()
    start('roscore', ['roscore', '-p', '11311'])
    def master_ready():
        try:
            return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/r625_wait')[0] == 1
        except OSError:
            return False
    assert wait_until(master_ready, 20), 'ROS master timeout'
    import rospy
    import tf2_ros
    from sensor_msgs.msg import LaserScan, Imu
    from nav_msgs.msg import OccupancyGrid
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    rospy.init_node('r625_observer', disable_signals=True)
    rospy.set_param('/use_sim_time', True)
    config = {'global_frame': 'map', 'robot_base_frame': 'base_link', 'update_frequency': 5.0,
              'publish_frequency': 5.0, 'transform_tolerance': 1.0, 'width': 10.0, 'height': 10.0,
              'resolution': 0.1, 'origin_x': -5.0, 'origin_y': -5.0, 'robot_radius': 0.1,
              'rolling_window': False, 'track_unknown_space': False, 'always_send_full_costmap': True,
              'plugins': [{'name': 'static', 'type': 'costmap_2d::StaticLayer'},
                          {'name': 'elevation', 'type': 'lite3_costmap_layers::ElevationMergedLayer'}],
              'static': {'map_topic': '/map', 'use_maximum': True},
              'elevation': {'enabled': True, 'use_maximum': True, 'input_topic': '/r625/elevation',
                            'layer_name': 'elevation', 'height_threshold': 0.15, 'ground_z': 0.0, 'obstacle_cost': 254}}
    rospy.set_param('/r625_costmap/costmap', config)
    (RUN / 'costmap-params.json').write_text(json.dumps(config, indent=2))
    samples = {'scan': [], 'imu': [], 'clock': [], 'map': [], 'costmap': []}
    tf_edges = {}
    def observe_tf(msg):
        for tr in msg.transforms:
            key = tr.header.frame_id + '->' + tr.child_frame_id
            tf_edges.setdefault(key, set()).add(msg._connection_header.get('callerid', ''))
    subscriptions = [rospy.Subscriber('/tf_static', TFMessage, observe_tf), rospy.Subscriber('/tf', TFMessage, observe_tf)]
    for topic, msgtype, key in (('/r625/scan', LaserScan, 'scan'), ('/r625/imu', Imu, 'imu'),
                                ('/clock', Clock, 'clock'), ('/map', OccupancyGrid, 'map'),
                                ('/r625_costmap/costmap/costmap', OccupancyGrid, 'costmap')):
        subscriptions.append(rospy.Subscriber(topic, msgtype, lambda msg, k=key: samples[k].append(msg), queue_size=1000))
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    for name, xyz, parent, child in [('map_odom', ['0','0','0'], 'map', 'odom'),
                                   ('odom_base', ['0','0','0.5'], 'odom', 'base_link'),
                                   ('base_lidar', ['0','0','0'], 'base_link', 'lidar'),
                                   ('base_imu', ['0','0','0'], 'base_link', 'imu')]:
        start(name, ['/opt/ros/noetic/lib/tf2_ros/static_transform_publisher', *xyz, '0','0','0', parent, child, '__name:=r625_' + name])
    start('map_server', ['/opt/ros/noetic/lib/map_server/map_server', str(RUN / 'map.yaml'), '__name:=r625_map'])
    start('xvfb', ['Xvfb', ':95', '-screen', '0', '640x480x24', '-nolisten', 'tcp'])
    assert wait_until(lambda: Path('/tmp/.X11-unix/X95').exists(), 10), 'Xvfb timeout'
    os.environ['DISPLAY'] = ':95'
    gz = start('gazebo', ['gzserver', '--verbose', '-s', 'libgazebo_ros_api_plugin.so', str(RUN / 'scene.world')])
    costmap = start('costmap', ['/opt/ros/noetic/lib/costmap_2d/costmap_2d_node', '__name:=r625_costmap'])
    begin = time.monotonic()
    ready = wait_until(lambda: len(samples['scan']) >= 50 and len(samples['imu']) >= 300
                      and len(samples['costmap']) >= 10 and bool(samples['map']), 60)
    result['wall_observation_seconds'] = time.monotonic() - begin
    result['counts'] = {k: len(v) for k, v in samples.items()}
    result['checks']['samples_ready'] = ready
    result['topics'] = rospy.get_published_topics()
    result['graph'] = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getSystemState('/r625_observer')
    result['tf_writers'] = {k: sorted(v) for k, v in tf_edges.items()}
    result['checks']['tf_single_writer'] = (set(tf_edges) == {'map->odom','odom->base_link','base_link->lidar','base_link->imu'}
                                           and all(len(v) == 1 for v in tf_edges.values()))
    result['transforms'] = {}
    for frame in ('lidar', 'imu'):
        tr = buffer.lookup_transform('map', frame, rospy.Time(0), rospy.Duration(2))
        result['transforms'][frame] = str(tr)
    result['checks']['tf_connected'] = True
    if samples['scan']:
        scan = samples['scan'][-1]
        finite = [x for x in scan.ranges if math.isfinite(x) and scan.range_min < x < scan.range_max]
        stamps = [s.header.stamp.to_sec() for s in samples['scan']]
        result['scan'] = {'stamp': stamps[-1], 'frame': scan.header.frame_id, 'angle_min': scan.angle_min,
                          'angle_increment': scan.angle_increment, 'ranges': [x if math.isfinite(x) else None for x in scan.ranges],
                          'publisher': scan._connection_header.get('callerid'), 'stamps': stamps}
        result['checks']['ray_geometry'] = (len(finite) > 10 and max(finite)-min(finite) > 0.1
                                            and abs(scan.ranges[90] - 2.5) < 0.05 and scan.header.frame_id == 'lidar')
        result['checks']['scan_time'] = len(stamps) >= 50 and all(b > a for a,b in zip(stamps, stamps[1:]))
    if samples['imu']:
        msg = samples['imu'][-1]
        acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        stamps = [s.header.stamp.to_sec() for s in samples['imu']]
        result['imu'] = {'frame': msg.header.frame_id, 'acceleration': acc, 'publisher': msg._connection_header.get('callerid'), 'stamps': stamps}
        result['checks']['imu_gravity'] = all(math.isfinite(x) for x in acc) and abs(math.sqrt(sum(x*x for x in acc))-9.8) < 0.2 and msg.header.frame_id == 'imu'
        result['checks']['imu_time'] = len(stamps) >= 300 and all(b > a for a,b in zip(stamps, stamps[1:]))
    clocks = [s.clock.to_sec() for s in samples['clock']]
    result['clock'] = {'first': clocks[0] if clocks else None, 'last': clocks[-1] if clocks else None, 'count': len(clocks)}
    result['checks']['clock_monotonic'] = len(clocks)>10 and clocks[-1]-clocks[0]>4 and all(b>=a for a,b in zip(clocks,clocks[1:]))
    for key in ('map', 'costmap'):
        if samples[key]:
            m = samples[key][-1]
            result[key] = {'width': m.info.width, 'height': m.info.height, 'resolution': m.info.resolution,
                           'frame': m.header.frame_id, 'occupied': sum(x==100 for x in m.data), 'free': sum(x==0 for x in m.data),
                           'sha256': hashlib.sha256(bytes(x+1 for x in m.data)).hexdigest()}
            result['checks'][key + '_nonempty'] = m.info.width == 100 and m.info.height == 100 and 0 in m.data and 100 in m.data
    result['loaded_files'] = {}
    for name, proc in (('gazebo', gz), ('costmap', costmap)):
        paths = sorted({line.split()[-1] for line in Path('/proc/' + str(proc.pid) + '/maps').read_text().splitlines()
                        if len(line.split()) >= 6 and line.split()[-1].startswith('/')})
        result['loaded_files'][name] = {p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in paths if Path(p).is_file()}
    lib = str(ROOT / 'ws/devel/lib/liblite3_costmap_layers.so')
    expected = json.loads((ROOT / 'build-result.json').read_text())['sha256']
    result['checks']['built_plugin_loaded'] = result['loaded_files']['costmap'].get(lib) == expected
    result['checks']['no_old_workspace_libraries'] = not any('/opt/ros1-slam/ws/devel/' in p for group in result['loaded_files'].values() for p in group)
    result['checks']['system_sensors_loaded'] = all('/opt/ros/noetic/lib/' + lib in result['loaded_files']['gazebo'] for lib in ('libgazebo_ros_laser.so','libgazebo_ros_imu_sensor.so'))
    result['checks']['processes_alive'] = all(p.poll() is None for _, p, _ in processes)
    result['resource'] = {'load': os.getloadavg(), 'cpu_count': os.cpu_count(),
                          'statvfs_free': os.statvfs(str(ROOT)).f_bavail * os.statvfs(str(ROOT)).f_frsize,
                          'memory': Path('/proc/meminfo').read_text()}
    result['checks']['resources'] = os.getloadavg()[0] <= os.cpu_count()*0.5 and result['resource']['statvfs_free'] >= 15*1024**3
    rospy.signal_shutdown('evidence complete')


try:
    execute()
except Exception as error:
    result['errors'].append(repr(error))
finally:
    stopped = []
    for name, proc, stream in reversed(processes):
        forced = False
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                forced = True
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
        stopped.append({'name': name, 'pid': proc.pid, 'rc': proc.returncode, 'forced': forced})
        stream.close()
    result['stopped'] = stopped
    result['checks']['basic_stop'] = bool(stopped) and all(not s['forced'] and s['rc'] in (0, -signal.SIGINT) for s in stopped)
    required = {'samples_ready','tf_single_writer','tf_connected','ray_geometry','scan_time','imu_gravity','imu_time',
                'clock_monotonic','map_nonempty','costmap_nonempty','built_plugin_loaded','no_old_workspace_libraries',
                'system_sensors_loaded','processes_alive','resources','basic_stop'}
    result['required_checks'] = sorted(required)
    result['accepted'] = not result['errors'] and all(result['checks'].get(k) is True for k in required)
    (RUN / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'run': str(RUN), 'accepted': result['accepted'], 'errors': result['errors'], 'checks': result['checks'], 'counts': result.get('counts')}, ensure_ascii=False))
raise SystemExit(0 if result['accepted'] else 5)
