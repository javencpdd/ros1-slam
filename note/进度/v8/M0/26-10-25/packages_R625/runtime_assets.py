import json
from pathlib import Path
import subprocess
import sys
import capture

PACKAGES = ['ros-noetic-' + p for p in ('gazebo-ros', 'gazebo-plugins', 'costmap-2d',
    'map-server', 'grid-map-core', 'grid-map-ros', 'grid-map-msgs', 'pluginlib', 'roscpp', 'tf2-ros')]
HEADERS = ('gazebo_ros_laser.h', 'gazebo_ros_imu_sensor.h', 'costmap_2d.h',
           'layer.h', 'GridMap.hpp', 'GridMapRosConverter.hpp', 'class_loader.hpp')
original_copy = capture.copy_file
extras = []


def copy_file(cid, path):
    row = original_copy(cid, path)
    if path.endswith('.list') and row['rc'] == 0:
        for name in row['text'].splitlines():
            if Path(name).name in HEADERS:
                extras.append(original_copy(cid, name))
        if path.endswith('ros-noetic-gazebo-ros.list'):
            for name in ('/usr/share/doc/libgazebo11/copyright',
                         '/usr/share/doc/gazebo11/copyright',
                         '/opt/ros/noetic/include/gazebo_plugins/gazebo_ros_laser.h',
                         '/opt/ros/noetic/include/gazebo_plugins/gazebo_ros_imu_sensor.h'):
                extras.append(original_copy(cid, name))
    return row


if __name__ == '__main__':
    capture.PACKAGES = PACKAGES
    capture.copy_file = copy_file
    capture.LABEL = 'org.ros1-slam.r625-assets'
    with Path(sys.argv[1]).open('x') as stream:
        data = capture.collect()
        data['driver_sha256'] = capture.digest(Path(__file__).read_bytes())
        data['headers_and_copyright'] = extras
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    subprocess.run(['setfacl', '-m', 'u:ros1-slam:rwx', sys.argv[1]], check=True)
    print(json.dumps({'complete': data['capture_complete'], 'errors': data['errors'],
                      'extra_paths': [(p['path'], p['rc']) for p in extras]}, ensure_ascii=False))
    sys.exit(0 if data['capture_complete'] else 5)
