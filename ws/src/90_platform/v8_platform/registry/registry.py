"""V8-04 组件注册表：加载 manifest、透传契约层校验、检索与证据状态分离。

分层（22 号 §代码分层）：本件属领域层，**只消费契约层**（../contracts/validate.py），
不 import ROS、不读写运行目录。跨组件冲突判定、依赖图与锁文件归 V8-05/06，本件不做。
原则（22 号 §2）：标签用于检索、契约用于组合、证据用于宣称——三者分开保存与展示。

失败语义＝fail-closed：目录加载中任一 manifest 无效即整体拒绝（组合层不得静默跳件），
理由码透传契约层（E_REG_INVALID 的 detail 携带契约码）。
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'contracts'))
import validate as contract  # noqa: E402  契约层（领域层只消费契约层）

MANIFEST_GLOB = 'manifest_*.json'
TESTED_LEVELS = ('contract_tested', 'simulation_verified', 'robot_verified')


class RegistryError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


class Registry:
    """已加载组件的内存索引；条目按 id 字典序确定性排序。

    provenance 边车＝注册层溯源数据（id→来源/许可/构建入口），与 manifest 分开保存：
    manifest.v1 已签契约不含这些字段（22 号 §3 差额见 README），在注册层补足而不改 schema。
    """

    PROVENANCE_FIELDS = ('source', 'license', 'build_entry')

    def __init__(self):
        self.entries = {}
        self.provenance = {}

    def load_dir(self, directory):
        base = Path(directory)
        if not base.is_dir():
            raise RegistryError('E_INPUT_MISSING', str(base))
        files = sorted(base.glob(MANIFEST_GLOB))
        if not files:
            raise RegistryError('E_REG_NO_MANIFESTS', 'no %s under %s' % (MANIFEST_GLOB, base))
        for path in files:
            self.load_file(path)
        return self

    def load_file(self, path):
        path = Path(path)
        if not path.is_file():
            raise RegistryError('E_INPUT_MISSING', str(path))
        raw = path.read_bytes()
        try:
            value = json.loads(raw)
        except json.JSONDecodeError as error:
            raise RegistryError('E_INPUT_JSON', '%s: %s' % (path.name, error))
        entry = self._register(value, path.name, hashlib.sha256(raw).hexdigest())
        self.entries[entry['id']] = entry
        return entry

    def _register(self, value, filename, sha256):
        try:
            contract.validate('manifest', value)
            normalized = contract.normalize_manifest(value)
        except contract.ContractError as error:
            raise RegistryError('E_REG_INVALID', '%s: %s: %s' % (filename, error.code, error.detail))
        if value['id'] in self.entries:
            raise RegistryError('E_REG_DUP_ID',
                                '%s duplicates %s' % (filename, self.entries[value['id']]['file']))
        return {
            'id': value['id'],
            'version': value['version'],
            'roles': sorted(value['roles']),
            'tags': sorted(value.get('tags', [])),
            'io': normalized['io'],
            'evidence': {
                'level': value['evidence']['level'],
                'reports': list(value['evidence']['reports']),
                'claimed_only': value['evidence']['level'] == 'declared',
            },
            'file': filename,
            'sha256': sha256,
        }

    # -- 检索（标签只用于检索，不推组合能力）--
    def get(self, component_id):
        return self.entries.get(component_id)

    def all(self):
        return [self.entries[key] for key in sorted(self.entries)]

    def by_role(self, role):
        return [e for e in self.all() if role in e['roles']]

    def by_tag(self, tag):
        return [e for e in self.all() if tag in e['tags']]

    # -- 溯源边车（来源/许可/构建入口；与 manifest 分开保存）--
    def load_provenance(self, path):
        path = Path(path)
        if not path.is_file():
            raise RegistryError('E_INPUT_MISSING', str(path))
        try:
            data = json.loads(path.read_bytes())
        except json.JSONDecodeError as error:
            raise RegistryError('E_INPUT_JSON', '%s: %s' % (path.name, error))
        if not isinstance(data, dict):
            raise RegistryError('E_PROV_INVALID_FIELD', '%s: provenance 顶层必须是对象' % path.name)
        for component_id, record in data.items():
            if component_id not in self.entries:
                raise RegistryError('E_PROV_UNKNOWN_ID',
                                    '%s 引用未注册组件 %s（先加载 manifest）' % (path.name, component_id))
            if not isinstance(record, dict):
                raise RegistryError('E_PROV_INVALID_FIELD', '%s: %s 记录必须是对象' % (path.name, component_id))
            for field in self.PROVENANCE_FIELDS:
                value = record.get(field)
                if not isinstance(value, str) or not value.strip():
                    raise RegistryError('E_PROV_INVALID_FIELD',
                                        '%s: %s.%s 缺失或空白' % (path.name, component_id, field))
            self.provenance[component_id] = {field: record[field] for field in self.PROVENANCE_FIELDS}
        return self

    def unprovenanced(self):
        """已注册但无边车溯源记录的组件 id（如实暴露覆盖缺口，不静默）。"""
        return sorted(set(self.entries) - set(self.provenance))

    # -- 证据状态：声明与实测分开（证据决定能否宣称支持）--
    def evidence_separation(self):
        declared = sorted(e['id'] for e in self.entries.values()
                          if e['evidence']['claimed_only'])
        tested = sorted(e['id'] for e in self.entries.values()
                        if e['evidence']['level'] in TESTED_LEVELS)
        return {'declared': declared, 'tested': tested}


def main(argv):
    parser = argparse.ArgumentParser(prog='registry.py', add_help=False)
    parser.add_argument('directory')
    parser.add_argument('--tag')
    parser.add_argument('--role')
    parser.add_argument('--id')
    parser.add_argument('--provenance')
    try:
        args = parser.parse_args(argv)
    except SystemExit:
        print('usage: registry.py DIRECTORY [--tag T] [--role R] [--id I] [--provenance P]',
              file=sys.stderr)
        return 9
    try:
        registry = Registry().load_dir(args.directory)
        if args.provenance:
            registry.load_provenance(args.provenance)
    except RegistryError as error:
        print(json.dumps({'ok': False, 'reason': error.code, 'detail': error.detail}, ensure_ascii=False))
        return 5
    if args.id:
        matches = [e] if (e := registry.get(args.id)) else []
    elif args.tag:
        matches = registry.by_tag(args.tag)
    elif args.role:
        matches = registry.by_role(args.role)
    else:
        matches = None
    result = {'ok': True, 'count': len(registry.entries),
              'evidence': registry.evidence_separation()}
    if args.provenance:
        result['provenance_count'] = len(registry.provenance)
        result['unprovenanced'] = registry.unprovenanced()
    if matches is None:
        result['components'] = [{'id': e['id'], 'version': e['version'], 'roles': e['roles'],
                                 'evidence': e['evidence']['level']} for e in registry.all()]
    else:
        result['matches'] = [e['id'] for e in matches]
    print(json.dumps(result, ensure_ascii=False, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
