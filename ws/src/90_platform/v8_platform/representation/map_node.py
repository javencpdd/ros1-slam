"""ROS1 地图查询适配层：Trigger 返回 JSON 快照，空闲证据与支撑语义分别上报。

输入均为本节点参数：map_json、evidence_json（可选，[{xyz:[x,y,z],state:...}]）、
query_points。每次调用重新载入，版本身份随响应返回；不修改 map.v1 契约。
"""
import json
import math
import time

from map_service import MapService, MapServiceError, content_digest


def snapshot(map_path, evidence_path, queries):
    with open(map_path, encoding='utf-8') as stream:
        value = json.load(stream)
    evidence = {}
    if evidence_path:
        with open(evidence_path, encoding='utf-8') as stream:
            rows = json.load(stream)
        for row in rows:
            xyz = tuple(row['xyz'])
            if len(xyz) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float))
                                    or not math.isfinite(v) for v in xyz):
                raise ValueError('evidence xyz requires three finite numbers')
            if xyz in evidence or row['state'] not in ('free', 'occupied', 'unknown'):
                raise ValueError('duplicate evidence or invalid state')
            evidence[xyz] = row['state']
    service = MapService(value['map_id'])
    service.publish(value, evidence=evidence)
    for point in queries:
        if len(point) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float))
                                  or not math.isfinite(v) for v in point):
            raise ValueError('query requires three finite numbers')
    return {'ok': True, 'map_id': value['map_id'], 'map_version': value['map_version'],
            'digest': content_digest(value, evidence),
            'queries': [service.query(*point) for point in queries]}


def main():
    import rospy
    from std_srvs.srv import Trigger, TriggerResponse

    rospy.init_node('v8_support_map')

    def query(_request):
        try:
            result = snapshot(rospy.get_param('~map_json'),
                              rospy.get_param('~evidence_json', ''),
                              rospy.get_param('~query_points', []))
        except (ValueError, KeyError, OSError, TypeError, MapServiceError) as error:
            return TriggerResponse(success=False, message=json.dumps({
                'ok': False, 'reason': getattr(error, 'code', 'E_MAP_QUERY_INPUT'),
                'detail': str(error)}, ensure_ascii=False))
        return TriggerResponse(success=True, message=json.dumps(result, ensure_ascii=False))

    rospy.Service(rospy.get_param('~query_service', rospy.get_name() + '/query'), Trigger, query)
    deadline = time.monotonic() + float(rospy.get_param('~keep_alive_s', 60.0))
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        time.sleep(0.05)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
