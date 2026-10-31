"""ROS1 multi-map provider: route /map_delta by map_id.

Each map gets its own latched topic and Trigger query service. State is persisted
through MapRouter/MapStore; the node does not change map or delta contracts.
"""
import json
import math
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import map_router  # noqa: E402
import occupancy_adapter  # noqa: E402


class MultiMapCore:
    def __init__(self, state_dir, maps, exclusive=False):
        self.router = map_router.MapRouter(state_dir, maps, exclusive=exclusive)
        self.errors = []
        self.lock = threading.Lock()

    def apply_text(self, text):
        with self.lock:
            try:
                version = self.router.apply_text(text)
            except Exception as error:  # map_service errors are fail-closed
                code = getattr(error, 'code', 'E_DELTA_JSON')
                self.errors.append(code)
                self.errors = self.errors[-10:]
                return {'ok': False, 'reason': code, 'detail': str(error)}
            return {'ok': True, 'map_version': version}

    def snapshot(self, map_id):
        with self.lock:
            return self.router.snapshot(map_id)

    def status(self):
        with self.lock:
            return {'maps': self.router.status(), 'errors': list(self.errors)}

    def close(self):
        self.router.close()


def _load_files(value):
    if isinstance(value, str):
        paths = [item.strip() for item in value.split(',') if item.strip()]
    else:
        paths = list(value)
    return [(Path(path), json.loads(Path(path).read_text(encoding='utf-8')))
            for path in paths]


def main():
    import rospy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger, TriggerResponse

    rospy.init_node('v8_multi_map')
    state_dir = rospy.get_param('~state_dir')
    loaded = _load_files(rospy.get_param('~map_files', []))
    if not loaded:
        raise ValueError('map_files must contain at least one map')
    maps = [value for _path, value in loaded]
    map_paths = {value['map_id']: path for path, value in loaded}
    core = MultiMapCore(state_dir, maps, exclusive=True)
    prefix = rospy.get_param('~topic_prefix', '/v8/maps')
    status_topic = rospy.get_param('~status_topic', prefix + '/status')
    delta_topic = rospy.get_param('~delta_topic', '/map_delta')
    publishers = {map_id: rospy.Publisher('%s/%s' % (prefix, map_id), String,
                                         queue_size=1, latch=True)
                  for map_id in core.router.ids()}
    status_pub = rospy.Publisher(status_topic, String, queue_size=10, latch=True)
    from nav_msgs.msg import OccupancyGrid
    grid_publishers, grid_status = {}, {}
    for map_id in core.router.ids():
        value = next(item for item in maps if item['map_id'] == map_id)
        if value.get('kind') != 'occupancy_grid':
            continue
        try:
            layer = value.get('layers', [{}])[0]
            path = Path(layer.get('data_uri', ''))
            if not path.is_absolute():
                path = map_paths[map_id].parent / path
            grid = occupancy_adapter.occupancy_grid(value, path)
            publisher = rospy.Publisher('%s/%s/grid' % (prefix, map_id),
                                              OccupancyGrid, queue_size=1, latch=True)
            message = OccupancyGrid()
            message.header.frame_id, message.header.stamp = grid['frame_id'], rospy.Time.now()
            message.info.resolution = grid['resolution']
            message.info.width, message.info.height = grid['width'], grid['height']
            message.info.origin.position.x, message.info.origin.position.y = grid['origin'][:2]
            message.info.origin.orientation.z = math.sin(grid['origin'][2] / 2.)
            message.info.origin.orientation.w = math.cos(grid['origin'][2] / 2.)
            message.data = grid['data']
            grid_publishers[map_id] = (publisher, message)
            grid_status[map_id] = {'ok': True, 'width': grid['width'],
                                   'height': grid['height'], 'kind': value.get('kind'),
                                   'frame_id': grid['frame_id'],
                                   'map_version': core.router.service(map_id).current['map_version']}
        except Exception as error:
            grid_status[map_id] = {'ok': False, 'error': str(error)}

    def publish_all():
        for map_id, publisher in publishers.items():
            snapshot = core.snapshot(map_id)
            if map_id in grid_status and grid_status[map_id].get('ok'):
                grid_status[map_id]['map_version'] = snapshot['map_version']
            publisher.publish(String(data=json.dumps(snapshot, ensure_ascii=False)))
        for publisher, message in grid_publishers.values():
            publisher.publish(message)
        status_pub.publish(String(data=json.dumps(dict(core.status(), grids=grid_status),
                                                 ensure_ascii=False)))

    def on_delta(message):
        result = core.apply_text(message.data)
        if result['ok']:
            publish_all()
        else:
            status_pub.publish(String(data=json.dumps(
                dict(core.status(), last_error=result), ensure_ascii=False)))

    rospy.Subscriber(delta_topic, String, on_delta, queue_size=20)

    for map_id in core.router.ids():
        def query(_request, map_id=map_id):
            try:
                return TriggerResponse(success=True,
                                       message=json.dumps(core.snapshot(map_id),
                                                          ensure_ascii=False))
            except Exception as error:
                return TriggerResponse(success=False, message=str(error))
        rospy.Service('%s/%s/query' % (prefix, map_id), Trigger, query)

    publish_all()
    keep_alive_s = float(rospy.get_param('~keep_alive_s', 0.0))
    deadline = time.monotonic() + keep_alive_s
    try:
        while not rospy.is_shutdown():
            if keep_alive_s > 0 and time.monotonic() >= deadline:
                break
            time.sleep(.05)
    finally:
        core.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
