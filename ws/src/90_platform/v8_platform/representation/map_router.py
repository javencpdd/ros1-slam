"""Multi-map routing core: map_delta by map_id -> per-map MapService + MapStore.

No ROS import. Each map keeps its own version, history and applied-delta table;
the router only dispatches by map_id, so 2D/3D/semantic maps can coexist in one
state directory without sharing versions.
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import map_service as svc  # noqa: E402


class MapRouter:
    def __init__(self, root, maps, exclusive=False):
        self.store = svc.MapStore(root)
        self.exclusive = bool(exclusive)
        self.services = {}
        if self.exclusive:
            self.store.acquire()
        try:
            for value in maps:
                self.add(value)
        except Exception:
            self.close()
            raise

    def close(self):
        self.store.release()

    def add(self, value):
        map_id = value.get('map_id')
        if not isinstance(map_id, str) or not map_id:
            raise svc.MapServiceError('E_MAPSERVICE_MAP_ID', 'map_id missing')
        if map_id in self.services:
            raise svc.MapServiceError('E_MAPSERVICE_DUPLICATE', map_id)
        path = self.store.path_for(map_id)
        if path.exists():
            self.services[map_id] = self.store.load(map_id)
        else:
            service = svc.MapService(map_id)
            service.publish(value)
            self.store.save(service)
            self.services[map_id] = service

    def service(self, map_id):
        try:
            return self.services[map_id]
        except KeyError:
            raise svc.MapServiceError('E_MAPSERVICE_MAP_ID', 'unknown map %s' % map_id)

    def ids(self):
        return sorted(self.services)

    def select(self, kind):
        return [map_id for map_id, service in sorted(self.services.items())
                if service.current.get('kind') == kind]

    def apply(self, delta):
        if not isinstance(delta, dict):
            raise svc.MapServiceError('E_DELTA_INVALID', 'delta must be an object')
        service = self.service(delta.get('map_id'))
        version = service.propose_delta(delta)
        self.store.save(service)
        return version

    def apply_text(self, text):
        try:
            delta = json.loads(text)
        except ValueError as error:
            raise svc.MapServiceError('E_DELTA_JSON', str(error))
        return self.apply(delta)

    def snapshot(self, map_id):
        service = self.service(map_id)
        return {'ok': True, 'map_id': map_id, 'kind': service.current.get('kind'),
                'map_version': service.current['map_version'],
                'digest': svc.content_digest(service.current, service.evidence),
                'map': service.current,
                'evidence': service.evidence_summary()}

    def status(self):
        return {map_id: {'kind': service.current.get('kind'),
                         'map_version': service.current['map_version'],
                         'history': [list(row) for row in service.history],
                         'applied_deltas': len(service.applied_deltas)}
                for map_id, service in sorted(self.services.items())}
