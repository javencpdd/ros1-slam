"""V8-08 次切片测试：版本语义（升版/拒膨胀/拒回退）、漂移检查、证据集成、净空 API。"""
import copy
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import map_service as svc  # noqa: E402
import support_map as rep  # noqa: E402

SOURCE = {'uri': 'design://two_floors', 'sha256': 'a' * 64}


def two_floors():
    return rep.build_support_map(
        'svc_map', 0.1, [-5.0, -5.0, 0.0],
        [{'surface_id': 'ground', 'z_reference': 0.0, 'data_uri': 'g.json',
          'footprint': {'min_x': -5, 'min_y': -5, 'max_x': 5, 'max_y': 5},
          'edges': [{'to': 'mezz', 'kind': 'slope', 'max_slope_deg': 12.0, 'bidirectional': True}]},
         {'surface_id': 'mezz', 'z_reference': 0.6, 'data_uri': 'm.json',
          'footprint': {'min_x': 2, 'min_y': 1, 'max_x': 4, 'max_y': 3},
          'edges': [{'to': 'ground', 'kind': 'slope', 'max_slope_deg': 12.0, 'bidirectional': True}]}],
        None, SOURCE)


def changed_floors(value):
    """内容变化：mezz 抬高到 0.8（step 语义同步调整以保合法）。"""
    bumped = copy.deepcopy(value)
    for s in bumped['surfaces']:
        if s['surface_id'] == 'mezz':
            s['z_reference'] = 0.8
            s['edges'] = [{'to': 'ground', 'kind': 'slope', 'max_slope_deg': 12.0,
                           'bidirectional': True}]
    for s in bumped['surfaces']:
        if s['surface_id'] == 'ground':
            s['edges'] = [{'to': 'mezz', 'kind': 'slope', 'max_slope_deg': 12.0,
                           'bidirectional': True}]
    return bumped


class MapServiceTests(unittest.TestCase):
    def test_publish_and_update_semantics(self):
        service = svc.MapService('svc_map')
        self.assertEqual(service.publish(two_floors()), 1)
        # 同内容重发＝拒（版本不膨胀）
        with self.assertRaises(svc.MapServiceError) as caught:
            service.update(two_floors())
        self.assertEqual(caught.exception.code, 'E_MAPSERVICE_NO_CHANGE')
        # 内容变化→升版＋source.sha256＝内容身份
        version = service.update(changed_floors(two_floors()))
        self.assertEqual(version, 2)
        self.assertEqual(len(service.current['source']['sha256']), 64)
        # 显式低/高版本传入＝拒（版本由服务管理）
        bad = changed_floors(two_floors())
        bad['map_version'] = 1
        with self.assertRaises(svc.MapServiceError) as caught:
            service.update(bad)
        self.assertEqual(caught.exception.code, 'E_MAPSERVICE_VERSION_REGRESSION')
        self.assertEqual(service.history, [(1, service.history[0][1]), (2, service.history[1][1])])

    def test_map_id_and_contract_guards(self):
        service = svc.MapService('svc_map')
        wrong = two_floors()
        wrong['map_id'] = 'other'
        with self.assertRaises(svc.MapServiceError) as caught:
            service.publish(wrong)
        self.assertEqual(caught.exception.code, 'E_MAPSERVICE_MAP_ID')
        with self.assertRaises(svc.MapServiceError) as caught:
            service.update(two_floors())
        self.assertEqual(caught.exception.code, 'E_MAPSERVICE_NOT_PUBLISHED')
        # 契约不合法的更新拒
        service.publish(two_floors())
        bad = changed_floors(two_floors())
        bad['surfaces'][0]['edges'] = []  # 断开连通（双向边两边都要清）
        bad['surfaces'][1]['edges'] = []
        with self.assertRaises(svc.MapServiceError) as caught:
            service.update(bad)
        self.assertEqual(caught.exception.code, 'E_MAPSERVICE_CONTRACT')

    def test_version_drift_check(self):
        service = svc.MapService('svc_map')
        first = two_floors()
        service.publish(first)
        self.assertTrue(service.check_version(first))  # 持同版
        service.update(changed_floors(two_floors()))
        with self.assertRaises(svc.MapServiceError) as caught:
            service.check_version(first)  # 消费者持旧版
        self.assertEqual(caught.exception.code, 'E_MAPSERVICE_STALE')

    def test_evidence_integration(self):
        """证据层随版本管理：证据变化即内容变化（升版）；摘要按足迹统计。"""
        service = svc.MapService('svc_map')
        base = two_floors()
        ev1 = {(0, 0, 0.0): 'free', (1, 0, 0.0): 'free'}
        service.publish(base, evidence=ev1)
        # 仅地图变（证据同）→升版；仅证据变（地图同）→也升版
        ev2 = dict(ev1)
        ev2[(2, 0, 0.0)] = 'free'
        self.assertEqual(service.update(base, evidence=ev2), 2)  # 地图同、证据变
        self.assertEqual(service.update(changed_floors(service.current), evidence=ev2), 3)
        summary = service.evidence_summary()
        self.assertEqual(summary['state'], 'ok')
        ground = next(s for s in summary['surfaces'] if s['surface'] == 'ground')
        self.assertEqual(ground['free'], 3)
        # 无证据层如实报
        bare = svc.MapService('svc_map')
        bare.publish(two_floors())
        self.assertEqual(bare.evidence_summary(), {'state': 'no_evidence_layer'})

    def test_clearance_api(self):
        service = svc.MapService('svc_map')
        service.publish(two_floors())
        self.assertAlmostEqual(service.clearance_at(3, 2, 0.0), 0.6)  # 重叠列
        self.assertIsNone(service.clearance_at(3, 2, 0.6))           # 开顶
        self.assertIsNone(service.clearance_at(50, 50, 0.0))         # 列外
        self.assertEqual(service.clearance_profile(3, 2), [(0.0, 0.6), (0.6, None)])
        self.assertEqual(service.clearance_profile(-4, -4), [(0.0, None)])

    def test_fractional_query_does_not_extend_roof_past_footprint(self):
        service = svc.MapService('svc_map')
        service.publish(two_floors(), evidence={(3.5, 2.5, 0.0): 'free'})
        self.assertAlmostEqual(service.clearance_at(3.5, 2.5, 0.0), 0.6)
        self.assertIsNone(service.clearance_at(4.01, 2.5, 0.0))
        query = service.query(3.5, 2.5, 0.6)
        self.assertTrue(query['supported'])
        self.assertEqual(query['state'], 'unknown')
        self.assertEqual([row['z'] for row in query['layers']], [0.0, 0.6])
        self.assertFalse(service.query(3.5, 2.5, 0.3)['supported'])
        self.assertEqual(service.query(3.5, 2.5, 0.0)['state'], 'free')

    def test_negative_fractional_query_does_not_truncate_toward_zero(self):
        value = two_floors()
        value['surfaces'][1]['footprint'] = {'min_x': -0.5, 'max_x': 0.5,
                                            'min_y': -0.5, 'max_y': 0.5}
        service = svc.MapService('svc_map')
        service.publish(value)
        self.assertEqual(service.clearance_profile(-0.25, 0.25), [(0.0, 0.6), (0.6, None)])
        self.assertEqual(service.clearance_profile(-0.75, 0.25), [(0.0, None)])

    def test_existing_unsupported_surface_is_not_reported_as_support(self):
        value = two_floors()
        value['multiple_surfaces_per_xy'] = False
        value['surfaces'] = [value['surfaces'][0]]
        value['surfaces'][0]['traversability'] = 'unsupported'
        value['surfaces'][0]['edges'] = []
        service = svc.MapService('svc_map')
        service.publish(value, evidence={(0.5, 0.5, 0): 'free'})
        query = service.query(0.5, 0.5, 0)
        self.assertEqual(query['surface_id'], 'ground')
        self.assertEqual(query['state'], 'free')
        self.assertFalse(query['supported'])
        self.assertEqual(query['layers'][0]['traversability'], 'unsupported')
        self.assertIsNone(query['clearance_m'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
