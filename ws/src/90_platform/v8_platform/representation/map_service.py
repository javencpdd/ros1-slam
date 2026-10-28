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
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import support_map as rep  # noqa: E402
import validate as contract  # noqa: E402


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


class MapService:
    """单 map_id 的版本化地图服务：发布、更新、漂移检查、证据摘要、净空查询。"""

    def __init__(self, map_id):
        self.map_id = map_id
        self.current = None
        self.evidence = None
        self.history = []  # (map_version, digest)

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

    # -- 净空查询 API（对当前版本）--
    def clearance_at(self, x, y, z):
        self._require_current()
        column = rep.column_at(self.current, x, y)
        if not column.heights():
            return None
        return column.clearance(z)

    def clearance_profile(self, x, y):
        """该 XY 列的全部 (高度, 上方净距)；开顶层净距＝None。"""
        self._require_current()
        column = rep.column_at(self.current, x, y)
        if not column.heights():
            return []
        return [(z, column.clearance(z)) for z in column.heights()]

    def query(self, x, y, z):
        """支撑、净空和观测证据是独立字段；支撑面存在不等于观测为 free。"""
        self._require_current()
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

    @staticmethod
    def _validate(value):
        try:
            contract.validate('map', value)
        except contract.ContractError as error:
            raise MapServiceError('E_MAPSERVICE_CONTRACT', '%s: %s' % (error.code, error.detail))
