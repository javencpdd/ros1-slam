"""V8-08 次切片：地图版本与更新服务＋空闲证据集成＋净空查询 API（R650）。

版本语义：
- content_digest＝对 map 内容（除 map_version 外）的规范序列化 sha256——版本无关的内容身份；
- 只有内容变化才升版（map_version+1）；同内容重复提交＝E_MAPSERVICE_NO_CHANGE（拒绝版本膨胀）；
- map_id 不一致＝E_MAPSERVICE_MAP_ID；显式回退版本＝E_MAPSERVICE_VERSION_REGRESSION。
- 消费者漂移检查：同 map_id 下 version 或 digest 不一致＝E_MAPSERVICE_STALE（消费者可校验，
  不靠口头同步）。

空闲证据集成：证据网格作为服务侧数据层随版本一起管理——证据变化即内容变化（升版）；
证据摘要按支撑面足迹统计 free/unknown（缺证＝unknown，无默认 free）。
分层：消费契约层（map.v1）与生产者（support_map.py），不 import ROS。
"""
import copy
import fcntl
import hashlib
import json
import os
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import support_map as rep  # noqa: E402
import validate as contract  # noqa: E402


SNAPSHOT_SCHEMA = 'map_service_snapshot/v1'
MAP_ID = re.compile(r'^[A-Za-z0-9_.-]+$')


class MapServiceError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


def content_digest(value, evidence=None):
    """版本无关的内容身份：map（除 map_version 与 source.sha256——后者是本身份的自戳印，
    纳入即自指不一致，R650 对抗面实证）＋证据（若有）的规范序列化 sha256。"""
    payload = {k: v for k, v in value.items() if k != 'map_version'}
    source = dict(payload.get('source', {}))
    source.pop('sha256', None)
    if source:
        payload['source'] = source
    if evidence is not None:
        payload['#evidence'] = sorted(evidence.items())
    canonical = json.dumps(payload, sort_keys=True, ensure_ascii=False, separators=(',', ':'))
    return hashlib.sha256(canonical.encode()).hexdigest()


class MapStore:
    """Directory of per-map snapshots; map_id is the filename key, not a path."""

    def __init__(self, root):
        self.root = Path(root)
        self._lock_handle = None

    def acquire(self, blocking=False):
        """Single-writer lock for a state directory (provider process)."""
        self.root.mkdir(parents=True, exist_ok=True)
        handle = open(self.root / '.map_store.lock', 'a+')
        try:
            flags = fcntl.LOCK_EX | (0 if blocking else fcntl.LOCK_NB)
            fcntl.flock(handle.fileno(), flags)
        except OSError as error:
            handle.close()
            raise MapServiceError('E_MAPSERVICE_LOCKED', str(error))
        self._lock_handle = handle
        return self

    def release(self):
        if self._lock_handle is not None:
            fcntl.flock(self._lock_handle.fileno(), fcntl.LOCK_UN)
            self._lock_handle.close()
            self._lock_handle = None

    def path_for(self, map_id):
        if not isinstance(map_id, str) or not MAP_ID.match(map_id):
            raise MapServiceError('E_MAPSERVICE_MAP_ID', 'unsafe map_id %r' % (map_id,))
        return self.root / (map_id + '.json')

    def save(self, service):
        saved = service.save(self.path_for(service.map_id))
        return saved

    def load(self, map_id):
        return MapService.load(self.path_for(map_id), map_id=map_id)

    def ids(self):
        if not self.root.is_dir():
            return []
        return sorted(path.stem for path in self.root.glob('*.json'))

    def load_all(self):
        return {map_id: self.load(map_id) for map_id in self.ids()}


class MapService:
    """单 map_id 的版本化地图服务：发布、更新、漂移检查、证据摘要、净空查询。"""

    def __init__(self, map_id):
        self.map_id = map_id
        self.current = None
        self.evidence = None
        self.history = []  # (map_version, digest)
        self.applied_deltas = {}  # delta_id -> delta digest (idempotent replay)
        self.listeners = []
        self.listener_errors = []

    def publish(self, value, evidence=None):
        """首发（map_version 以传入值为准，≥1）；再发走 update。"""
        if self.current is not None:
            raise MapServiceError('E_MAPSERVICE_ALREADY_PUBLISHED', '已发布，走 update')
        if value.get('map_id') != self.map_id:
            raise MapServiceError('E_MAPSERVICE_MAP_ID',
                                  '%s ≠ %s' % (value.get('map_id'), self.map_id))
        self._validate(value)
        self.current = value
        self.evidence = dict(evidence) if evidence else None
        self.history.append((value['map_version'], content_digest(value, self.evidence)))
        self._notify(value['map_version'])
        return value['map_version']

    def update(self, value, evidence=None):
        """更新：内容变化→升版；同内容→拒（不膨胀）；显式低版本→拒（不回退）。"""
        self._require_current()
        if value.get('map_id') != self.map_id:
            raise MapServiceError('E_MAPSERVICE_MAP_ID',
                                  '%s ≠ %s' % (value.get('map_id'), self.map_id))
        self._validate(value)
        if value['map_version'] != self.current['map_version']:
            raise MapServiceError('E_MAPSERVICE_VERSION_REGRESSION',
                                  '版本由服务管理（现 %s，传入 %s）' % (self.current['map_version'],
                                                                           value['map_version']))
        digest = content_digest(value, evidence)
        if digest == self.history[-1][1]:
            raise MapServiceError('E_MAPSERVICE_NO_CHANGE', '内容未变，不升版')
        bumped = dict(value)
        bumped['map_version'] = self.current['map_version'] + 1
        bumped['source'] = dict(bumped.get('source', {}))
        bumped['source']['sha256'] = digest  # 服务自产图：source.sha256＝内容身份
        self._validate(bumped)
        self.current = bumped
        self.evidence = dict(evidence) if evidence else None
        self.history.append((bumped['map_version'], content_digest(bumped, self.evidence)))
        self._notify(bumped['map_version'])
        return bumped['map_version']

    def check_version(self, consumer_map):
        """消费者漂移检查：同 map_id 的 version 与内容 digest 必须一致。"""
        self._require_current()
        if consumer_map.get('map_id') != self.map_id:
            raise MapServiceError('E_MAPSERVICE_MAP_ID', '消费者持不同地图')
        if consumer_map['map_version'] != self.current['map_version']:
            raise MapServiceError('E_MAPSERVICE_STALE',
                                  '消费者 v%s，服务 v%s' % (consumer_map['map_version'],
                                                             self.current['map_version']))
        return True

    def evidence_summary(self):
        """按支撑面足迹统计证据（缺证＝unknown；服务无证据层时如实报无）。"""
        self._require_current()
        if 'surfaces' not in self.current:
            return {'state': 'not_applicable', 'kind': self.current.get('kind')}
        if self.evidence is None:
            return {'state': 'no_evidence_layer'}
        summary = []
        for surface in self.current['surfaces']:
            fp = surface['footprint']
            free = unknown = 0
            for (x, y, z), state in self.evidence.items():
                if (fp['min_x'] <= x <= fp['max_x'] and fp['min_y'] <= y <= fp['max_y']
                        and abs(z - surface['z_reference']) < 1e-9):
                    if state == 'free':
                        free += 1
                    else:
                        unknown += 1
            summary.append({'surface': surface['surface_id'], 'free': free, 'unknown': unknown})
        return {'state': 'ok', 'surfaces': summary}


    # -- 增量提交（局部缓存 → 全局地图）--
    def subscribe(self, callback):
        self.listeners.append(callback)

    def _notify(self, version):
        for callback in list(self.listeners):
            try:
                callback(version)
            except Exception as error:  # a bad listener must not undo a commit
                self.listener_errors.append(repr(error))

    def propose_delta(self, delta):
        """Apply a base-version delta atomically; replay returns the current version."""
        self._require_current()
        if self.current.get('kind') != 'support_map':
            raise MapServiceError('E_MAPSERVICE_UPDATE_SCHEMA',
                                  'delta only applies to support_map, got %r'
                                  % self.current.get('kind'))
        try:
            contract.validate('map_delta', delta)
        except contract.ContractError as error:
            raise MapServiceError('E_DELTA_INVALID', '%s: %s' % (error.code, error.detail))
        if delta['map_id'] != self.map_id:
            raise MapServiceError('E_MAPSERVICE_MAP_ID',
                                  '%s ≠ %s' % (delta['map_id'], self.map_id))
        delta_digest = hashlib.sha256(
            json.dumps(delta, sort_keys=True, ensure_ascii=False,
                       separators=(',', ':')).encode()).hexdigest()
        if delta['delta_id'] in self.applied_deltas:
            if self.applied_deltas[delta['delta_id']] != delta_digest:
                raise MapServiceError('E_DELTA_REPLAY_CONFLICT',
                                      'delta_id %s reused with different content'
                                      % delta['delta_id'])
            return self.current['map_version']
        if delta['base_version'] != self.current['map_version']:
            raise MapServiceError('E_MAPSERVICE_STALE',
                                  'delta base v%s, current v%s'
                                  % (delta['base_version'], self.current['map_version']))
        candidate = self._apply_delta(delta)
        digest = content_digest(candidate, self.evidence)
        if digest == self.history[-1][1]:
            raise MapServiceError('E_DELTA_NO_CHANGE', 'delta changed nothing')
        bumped = copy.deepcopy(candidate)
        bumped['map_version'] = self.current['map_version'] + 1
        bumped['source'] = dict(bumped.get('source', {}))
        bumped['source']['uri'] = delta['source']['uri']
        bumped['source']['sha256'] = digest
        self._validate(bumped)
        self.current = bumped
        self.history.append((bumped['map_version'], content_digest(bumped, self.evidence)))
        self.applied_deltas[delta['delta_id']] = delta_digest
        self._notify(bumped['map_version'])
        return bumped['map_version']

    def _apply_delta(self, delta):
        candidate = copy.deepcopy(self.current)
        surfaces = candidate['surfaces']
        index = {surface['surface_id']: position for position, surface in enumerate(surfaces)}
        for op in delta['ops']:
            if op['op'] == 'upsert_surface':
                surface = copy.deepcopy(op['surface'])
                surface_id = surface.get('surface_id')
                if not isinstance(surface_id, str) or not surface_id:
                    raise MapServiceError('E_DELTA_OP', 'surface_id missing')
                if surface_id in index:
                    surfaces[index[surface_id]] = surface
                else:
                    index[surface_id] = len(surfaces)
                    surfaces.append(surface)
            else:
                surface_id = op['surface_id']
                if surface_id not in index:
                    raise MapServiceError('E_DELTA_OP', 'remove unknown %s' % surface_id)
                surfaces.pop(index[surface_id])
                index = {surface['surface_id']: position
                         for position, surface in enumerate(surfaces)}
        self._validate(candidate)
        return candidate

    # -- 持久化：原子快照与恢复（跨进程重启保持版本/幂等表）--
    def snapshot(self):
        self._require_current()
        evidence = None
        if self.evidence is not None:
            evidence = [[x, y, z, state]
                        for (x, y, z), state in sorted(self.evidence.items())]
        return {
            'schema': SNAPSHOT_SCHEMA,
            'map_id': self.map_id,
            'current': copy.deepcopy(self.current),
            'evidence': evidence,
            'history': [[version, digest] for version, digest in self.history],
            'applied_deltas': dict(sorted(self.applied_deltas.items())),
        }

    def save(self, path):
        """Atomically persist the service state; readers see old or new, never partial."""
        payload = self.snapshot()
        data = json.dumps(payload, ensure_ascii=False, sort_keys=True,
                          separators=(',', ':')).encode() + b'\n'
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_name(path.name + '.tmp.%d' % os.getpid())
        with open(tmp, 'wb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, path)
        try:
            directory = os.open(str(path.parent), os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        except OSError:
            pass
        return {'path': str(path), 'sha256': hashlib.sha256(data).hexdigest(),
                'map_version': self.current['map_version'],
                'digest': self.history[-1][1], 'bytes': len(data)}

    @classmethod
    def load(cls, path, map_id=None):
        """Restore and verify a snapshot; corrupt/foreign/rewound files fail closed."""
        path = Path(path)
        try:
            raw = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError) as error:
            raise MapServiceError('E_MAPSERVICE_SNAPSHOT',
                                  '%s: %s' % (path, error))
        if not isinstance(raw, dict) or raw.get('schema') != SNAPSHOT_SCHEMA:
            raise MapServiceError('E_MAPSERVICE_SNAPSHOT', 'schema mismatch')
        raw_id = raw.get('map_id')
        if not isinstance(raw_id, str) or not raw_id:
            raise MapServiceError('E_MAPSERVICE_SNAPSHOT', 'map_id missing')
        if map_id is not None and raw_id != map_id:
            raise MapServiceError('E_MAPSERVICE_MAP_ID', '%s ≠ %s' % (raw_id, map_id))
        current = raw.get('current')
        if not isinstance(current, dict) or current.get('map_id') != raw_id:
            raise MapServiceError('E_MAPSERVICE_SNAPSHOT', 'current map_id mismatch')
        cls._validate(current)
        evidence = None
        raw_evidence = raw.get('evidence')
        if raw_evidence is not None:
            if not isinstance(raw_evidence, list):
                raise MapServiceError('E_MAPSERVICE_SNAPSHOT', 'evidence must be a list')
            evidence = {}
            for row in raw_evidence:
                if (not isinstance(row, list) or len(row) != 4
                        or row[3] not in ('free', 'occupied', 'unknown')):
                    raise MapServiceError('E_MAPSERVICE_SNAPSHOT', 'bad evidence row')
                try:
                    key = (row[0], row[1], row[2])
                    float(key[0]); float(key[1]); float(key[2])
                except (TypeError, ValueError):
                    raise MapServiceError('E_MAPSERVICE_SNAPSHOT', 'bad evidence key')
                evidence[key] = row[3]
        history = raw.get('history')
        if not isinstance(history, list) or not history:
            raise MapServiceError('E_MAPSERVICE_SNAPSHOT', 'history missing')
        clean_history = []
        for row in history:
            if (not isinstance(row, list) or len(row) != 2
                    or not isinstance(row[0], int) or not isinstance(row[1], str)
                    or len(row[1]) != 64):
                raise MapServiceError('E_MAPSERVICE_SNAPSHOT', 'bad history row')
            clean_history.append((row[0], row[1]))
        if any(b[0] <= a[0] for a, b in zip(clean_history, clean_history[1:])):
            raise MapServiceError('E_MAPSERVICE_SNAPSHOT', 'history versions not increasing')
        digest = content_digest(current, evidence)
        if clean_history[-1][0] != current['map_version'] or clean_history[-1][1] != digest:
            raise MapServiceError('E_MAPSERVICE_SNAPSHOT', 'snapshot digest/version mismatch')
        applied = raw.get('applied_deltas', {})
        if not isinstance(applied, dict) or any(
                not isinstance(key, str) or not isinstance(value, str) or len(value) != 64
                for key, value in applied.items()):
            raise MapServiceError('E_MAPSERVICE_SNAPSHOT', 'bad applied_deltas')
        service = cls(raw_id)
        service.current = current
        service.evidence = evidence
        service.history = clean_history
        service.applied_deltas = dict(applied)
        return service

    # -- 净空查询 API（对当前版本）--
    def clearance_at(self, x, y, z):
        self._require_current()
        self._require_support()
        column = rep.column_at(self.current, x, y)
        if not column.heights():
            return None
        return column.clearance(z)

    def clearance_profile(self, x, y):
        """该 XY 列的全部 (高度, 上方净距)；开顶层净距＝None。"""
        self._require_current()
        self._require_support()
        column = rep.column_at(self.current, x, y)
        if not column.heights():
            return []
        return [(z, column.clearance(z)) for z in column.heights()]

    def query(self, x, y, z):
        """支撑、净空和观测证据是独立字段；支撑面存在不等于观测为 free。"""
        self._require_current()
        self._require_support()
        column = rep.column_at(self.current, x, y)
        sid = column.surface_at(z)
        surfaces = {surface['surface_id']: surface for surface in self.current['surfaces']}
        supported = sid is not None and surfaces[sid]['traversability'] == 'supported'
        state = (self.evidence or {}).get((x, y, z), 'unknown')
        if state not in ('free', 'occupied', 'unknown'):
            raise MapServiceError('E_MAPSERVICE_EVIDENCE', 'invalid evidence state %r' % state)
        return {'x': x, 'y': y, 'z': z, 'surface_id': sid,
                'supported': supported, 'state': state,
                'layers': [{'surface_id': column.surface_at(height), 'z': height,
                            'traversability': surfaces[column.surface_at(height)]['traversability'],
                            'clearance_m': column.clearance(height)}
                           for height in column.heights()],
                'clearance_m': column.clearance(z) if supported else None}

    def _require_current(self):
        if self.current is None:
            raise MapServiceError('E_MAPSERVICE_NOT_PUBLISHED', self.map_id)

    def _require_support(self):
        if 'surfaces' not in self.current:
            raise MapServiceError('E_MAPSERVICE_UNSUPPORTED_KIND',
                                  'expected support_map, got %r'
                                  % self.current.get('kind'))

    @staticmethod
    def _validate(value):
        try:
            contract.validate('map', value)
        except contract.ContractError as error:
            raise MapServiceError('E_MAPSERVICE_CONTRACT', '%s: %s' % (error.code, error.detail))
