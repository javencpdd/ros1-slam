"""Bounded, offline single-task handoff. Packages never carry executable files."""
import argparse
import base64
import copy
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess
import sys
import tempfile
import uuid
from datetime import datetime, timedelta, timezone

PROJECT = 'ros1-slam'
PLATFORMS = ('x86_sim', 'lite3', 'm20')
AGENTS = ('zcode', 'qoder', 'workbuddy', 'codex', 'kimi', 'human')
MAX_PACKAGE = 1024 * 1024
MAX_LOCAL = 32 * 1024 * 1024
TASK_ID = re.compile(r'T-\d{8}-\d{2,6}\Z')
SHA = re.compile(r'[0-9a-f]{64}\Z')
COMMIT = re.compile(r'[0-9a-f]{40}\Z')
POINTER = '# AGENTS.md · ros1-slam 沙箱 · 指针\n\n唯一公共面＝`note/09_跨工具协作契约.md`（五家工具进场必读：zcode／qoder／WorkBuddy／Codex／Kimi Code）；本文件不入仓、只是指针，正文一律以该件为准。\n'
SHIM = '''#!/usr/bin/env python3
"""Generated ros1-slam bridge shim; canonical code is tracked in scripts/agent."""
import runpy
from pathlib import Path
runpy.run_path(str(Path(__file__).resolve().parents[1] / 'scripts' / 'agent' / 'bridge.py'), run_name='__main__')
'''
KNOWN_LEGACY = '3001ccfed42ab8d8f82fb109a7fb0436176b54cfee0e9e284d98eac429a0bf0a'


class TransferError(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise TransferError(message)


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'),
                      allow_nan=False).encode('utf-8')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def utc_now():
    return datetime.now(timezone.utc).replace(microsecond=0)


def timestamp(value=None):
    return (value or utc_now()).isoformat()


def parse_time(value):
    require(isinstance(value, str) and len(value) == 25 and value.endswith('+00:00'), '时间必须为UTC ISO8601')
    try:
        return datetime.fromisoformat(value)
    except ValueError as exc:
        raise TransferError('非法时间') from exc


def keys(value, expected, label):
    require(isinstance(value, dict) and set(value) == set(expected), label + '字段缺失/未知')


def identifier(value, label):
    require(isinstance(value, str), label + '必须是UUID')
    try:
        require(str(uuid.UUID(value)) == value, label + 'UUID不规范')
    except (ValueError, AttributeError) as exc:
        raise TransferError(label + '非法UUID') from exc
    return value


def integer(value, low, high, label):
    require(type(value) is int and low <= value <= high, label + '整数越界')


def text_value(value, limit, label, single=False):
    require(isinstance(value, str) and 0 < len(value.encode('utf-8')) <= limit and '\x00' not in value,
            label + '空值/类型/长度非法')
    if single:
        require('\n' not in value and '\r' not in value, label + '不得含换行')


def no_symlink(path):
    path = Path(path).absolute()
    require(not any(p.is_symlink() for p in (path, *path.parents)), '路径不允许软链: ' + str(path))
    return path


def read_bytes(path, limit=MAX_LOCAL):
    path = no_symlink(path)
    descriptor = os.open(str(path), os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0))
    with os.fdopen(descriptor, 'rb') as stream:
        require(stat.S_ISREG(os.fstat(stream.fileno()).st_mode), '输入必须为普通文件')
        data = stream.read(limit + 1)
    require(len(data) <= limit, '输入超过资源限额')
    return data


def unique_object(items):
    result = {}
    for key, value in items:
        require(key not in result, 'JSON重复键: ' + key)
        result[key] = value
    return result


def read_json(path, limit=MAX_PACKAGE):
    try:
        return json.loads(read_bytes(path, limit).decode('utf-8'), object_pairs_hook=unique_object,
                          parse_constant=lambda value: (_ for _ in ()).throw(TransferError('JSON非法常数')))
    except (UnicodeError, json.JSONDecodeError, RecursionError) as exc:
        raise TransferError('输入不是合法UTF-8 JSON') from exc


def envelope(schema, payload):
    return {'schema': schema, 'payload': payload, 'sha256': digest(canonical(payload))}


def unwrap(value, schema):
    keys(value, ('schema', 'payload', 'sha256'), 'envelope')
    require(value['schema'] == schema and isinstance(value['sha256'], str), '不支持的包版本')
    require(value['sha256'] == digest(canonical(value['payload'])), '包摘要不符')
    return value['payload']


def git(root, *args):
    env = {key: value for key, value in os.environ.items() if not key.startswith('GIT_')}
    env['GIT_CONFIG_NOSYSTEM'] = '1'
    command = ['git', '--no-optional-locks', '-c', 'core.fsmonitor=false', '-c',
               'core.hooksPath=/dev/null', '-C', str(root), *args]
    result = subprocess.run(command, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    require(result.returncode == 0, 'Git检查失败: ' + result.stderr.decode('utf-8', errors='replace')[:500])
    return result.stdout


def repository(bridge):
    root = no_symlink(bridge.ROOT)
    require(root.is_dir(), '项目根不存在')
    top = git(root, 'rev-parse', '--show-toplevel').decode().strip()
    require(Path(top) == root, '--root必须是精确Git仓根')
    head = git(root, 'rev-parse', 'HEAD').decode().strip()
    require(COMMIT.fullmatch(head) is not None, 'Git提交格式不支持')
    return root, head


def clean_tools(bridge, root):
    """The executing protocol must be exactly the code represented by HEAD."""
    actual = {'bridge.py': Path(bridge.__file__), 'crosshost.py': Path(__file__)}
    for basename, running in actual.items():
        name = 'scripts/agent/' + basename
        committed = git(root, 'show', 'HEAD:' + name)
        require(read_bytes(root / name) == committed and read_bytes(running) == committed,
                '协作工具未提交或执行版本与HEAD不符: ' + name)


def ai_path(bridge, name):
    require(isinstance(name, str) and name and '\\' not in name, '非法本机状态路径')
    pure = PurePosixPath(name)
    require(not pure.is_absolute() and all(p not in ('', '.', '..') for p in name.split('/')), '状态路径越界')
    return no_symlink(Path(bridge.AI_DIR) / name)


def validate_scope(scope):
    require(isinstance(scope, list) and 1 <= len(scope) <= 128, 'scope必须为1..128个明确相对路径')
    require(all(isinstance(item, str) for item in scope), 'scope必须为字符串路径')
    require(len(set(scope)) == len(scope), 'scope重复')
    for item in scope:
        text_value(item, 512, 'scope', True)
        path = item.rstrip('/')
        require(path and '\\' not in path and not path.startswith(('/', ':')) and
                all(part not in ('', '.', '..') for part in path.split('/')) and
                path.split('/')[0] != '.git', 'scope绝对/逃逸/特殊路径')


def dirty_scope(root, scope):
    records = git(root, 'status', '--porcelain=v1', '-z', '--untracked-files=all').split(b'\0')
    names = []
    index = 0
    while index < len(records):
        record = records[index]
        index += 1
        if not record:
            continue
        require(len(record) >= 4 and record[2:3] == b' ', 'Git状态输出不可解析')
        names.append(os.fsdecode(record[3:]))
        if b'R' in record[:2] or b'C' in record[:2]:
            require(index < len(records), 'Git重命名记录不完整')
            names.append(os.fsdecode(records[index])); index += 1
    return [name for name in names if any(name == p.rstrip('/') or name.startswith(p.rstrip('/') + '/') for p in scope)]


def local_host(bridge):
    value = read_json(ai_path(bridge, 'host.json'))
    keys(value, ('schema', 'project', 'host_id', 'platform', 'created_at'), '本机host')
    require(value['schema'] == 'agent_local_host/v1' and value['project'] == PROJECT, '本机host版本错误')
    identifier(value['host_id'], 'host_id'); require(value['platform'] in PLATFORMS, '未知platform')
    parse_time(value['created_at'])
    return value


def ticket(bridge):
    root, head = repository(bridge); clean_tools(bridge, root)
    host = local_host(bridge)
    return envelope('agent_host/v1', {'project': PROJECT, 'host_id': host['host_id'],
                                    'platform': host['platform'], 'code_commit': head})


def validate_ticket(value):
    data = unwrap(value, 'agent_host/v1')
    keys(data, ('project', 'host_id', 'platform', 'code_commit'), 'host票据')
    require(data['project'] == PROJECT and data['platform'] in PLATFORMS, '项目/platform不匹配')
    identifier(data['host_id'], 'host_id'); require(isinstance(data['code_commit'], str) and COMMIT.fullmatch(data['code_commit']), '提交格式非法')
    return data


def validate_package(value):
    payload = unwrap(value, 'agent_handoff/v1')
    keys(payload, ('core', 'core_sha256', 'lease'), 'handoff payload')
    core, lease = payload['core'], payload['lease']
    require(payload['core_sha256'] == digest(canonical(core)), '不可变core摘要错误')
    keys(core, ('package_id', 'task_uuid', 'hop', 'source', 'destination', 'recipient_agent',
                'code_commit', 'created_at', 'task', 'card', 'summary', 'evidence'), 'core')
    identifier(core['package_id'], 'package_id'); identifier(core['task_uuid'], 'task_uuid')
    integer(core['hop'], 1, 1000000, 'hop')
    for side in ('source', 'destination'):
        keys(core[side], ('host_id', 'platform'), side)
        identifier(core[side]['host_id'], side + '.host_id')
        require(core[side]['platform'] in PLATFORMS, '非法platform')
    require(core['source']['host_id'] != core['destination']['host_id'], '同主机应使用handoff命令')
    require(core['recipient_agent'] in AGENTS, '非法接收agent')
    require(isinstance(core['code_commit'], str) and COMMIT.fullmatch(core['code_commit']), '非法代码提交')
    parse_time(core['created_at'])
    task = core['task']
    keys(task, ('id', 'title', 'scope', 'created_at', 'created_date', 'latest_handoff', 'origin'), 'task')
    require(isinstance(task['id'], str) and TASK_ID.fullmatch(task['id']), '非法源task ID')
    text_value(task['title'], 1024, 'title', True); validate_scope(task['scope'])
    require(isinstance(task['created_at'], str) and len(task['created_at']) <= 64 and
            isinstance(task['created_date'], str) and len(task['created_date']) <= 32 and
            isinstance(task['latest_handoff'], str) and len(task['latest_handoff'].encode()) <= 65536,
            '任务日期/交接字段非法')
    keys(task['origin'], ('host_id', 'task_id'), 'origin')
    identifier(task['origin']['host_id'], 'origin.host_id')
    require(isinstance(task['origin']['task_id'], str) and TASK_ID.fullmatch(task['origin']['task_id']), '非法origin task ID')
    text_value(core['card'], 256 * 1024, 'card'); text_value(core['summary'], 65536, 'summary')
    require(isinstance(core['evidence'], list) and len(core['evidence']) <= 128, '证据索引列表越界')
    for item in core['evidence']:
        text_value(item, 4096, 'evidence', True)
    keys(lease, ('revision', 'issued_at', 'expires_at'), 'lease')
    integer(lease['revision'], 0, 1000000, 'lease revision')
    issued, expires = parse_time(lease['issued_at']), parse_time(lease['expires_at'])
    require(timedelta(hours=1) <= expires - issued <= timedelta(hours=168), 'lease时长超出1..168小时')
    require(issued >= parse_time(core['created_at']), 'lease早于core')
    require(len(canonical(value)) <= MAX_PACKAGE, '包超过1MiB')
    return payload


def lease_valid(payload):
    now = utc_now(); lease = payload['lease']
    require(parse_time(lease['issued_at']) <= now < parse_time(lease['expires_at']), '包未生效/已过期；核本机UTC或请源端--renew')


def ensure_local_paths(bridge):
    no_symlink(bridge.ROOT); no_symlink(bridge.AI_DIR)
    for name in ('state.json', 'TASKS.md', 'HANDOFF.md', 'JOURNAL.md', '.state.lock', 'tasks',
                 'host.json', 'outbox', 'inbox', 'legacy', 'knowledge.md', 'tasks/_template.md',
                 'transfer.pending.json'):
        path = ai_path(bridge, name)
        if path.exists():
            require(path.is_dir() if name in ('tasks', 'outbox', 'inbox', 'legacy') else path.is_file(), '状态路径类型错误: ' + name)


def state(bridge):
    ensure_local_paths(bridge)
    require(not ai_path(bridge, 'transfer.pending.json').exists(), '移交事务未完整结束；先运行recover检查恢复')
    value = read_json(bridge.STATE, MAX_LOCAL)
    require(isinstance(value, dict) and isinstance(value.get('tasks'), dict), '本机state结构无效')
    require(value.get('root') == bridge.ROOT, '本机state.root不符；禁止静默跨机覆盖')
    require(type(value.get('epoch')) is int and value['epoch'] >= 1, '非法epoch')
    require(bridge._handoff_file_epoch() == value['epoch'], '本机epoch脱节，拒绝移交')
    for tid, task in value['tasks'].items():
        require(isinstance(tid, str) and TASK_ID.fullmatch(tid) and isinstance(task, dict) and
                task.get('id') == tid and task.get('file') == 'tasks/' + tid + '.md', '本机任务元信息路径无效')
        require(task.get('status') in bridge.VALID_STATUS, '本机任务状态无效')
        read_bytes(ai_path(bridge, task['file']))
    cur = value.get('current_task')
    require(cur is None or cur in value['tasks'], 'current缺失')
    require(isinstance(value.get('transfers', {}), dict), '本机移交ledger无效')
    return value


def sync_dir(path):
    descriptor = os.open(str(path), os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def durable_write(bridge, path, data, exclusive=False):
    path = no_symlink(path)
    flags = os.O_WRONLY | os.O_CREAT | (os.O_EXCL if exclusive else os.O_TRUNC) | getattr(os, 'O_NOFOLLOW', 0)
    descriptor = os.open(str(path), flags, 0o600)
    with os.fdopen(descriptor, 'wb') as stream:
        stream.write(data); stream.flush(); os.fsync(stream.fileno())
    bridge._setfacl(str(path)); sync_dir(path.parent)


def directory(bridge, name):
    path = ai_path(bridge, name)
    if not path.exists():
        path.mkdir(); bridge._setfacl(str(path)); bridge._setfacl(str(path), default=True)
    require(path.is_dir(), '状态目录错误')
    return path


def publish(bridge, path, data):
    """No overwrite, no partially visible output. Never called before source fence."""
    path = no_symlink(path)
    require(path.parent.is_dir(), '输出父目录必须先存在')
    if path.exists():
        require(read_bytes(path, MAX_PACKAGE) == data, '输出已存在且内容不同')
        return
    descriptor, temporary = tempfile.mkstemp(prefix='.' + path.name + '.', dir=str(path.parent))
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(data); stream.flush(); os.fsync(stream.fileno())
        bridge._setfacl(temporary)
        os.link(temporary, str(path)); sync_dir(path.parent)
    finally:
        os.unlink(temporary)


def transaction_paths(bridge, task_id, extra=()):
    return ['state.json', 'TASKS.md', 'HANDOFF.md', 'JOURNAL.md', 'tasks/' + task_id + '.md', *extra]


def valid_snapshot_name(name):
    return name in ('state.json', 'TASKS.md', 'HANDOFF.md', 'JOURNAL.md') or bool(
        re.fullmatch(r'tasks/T-\d{8}-\d{2,6}\.md|(?:inbox|outbox)/[0-9a-f-]{36}\.json', name))


def restore(bridge, snapshots):
    require(isinstance(snapshots, dict) and 4 <= len(snapshots) <= 8, '恢复快照名册错误')
    require({'state.json', 'TASKS.md', 'HANDOFF.md', 'JOURNAL.md'}.issubset(snapshots) and
            sum(name.startswith('tasks/') for name in snapshots) == 1, '恢复快照缺少完整生成文件或唯一任务卡')
    decoded = {}
    for name, item in snapshots.items():
        require(valid_snapshot_name(name), '恢复路径不在私有白名单')
        if item is None:
            decoded[name] = None
        else:
            keys(item, ('base64', 'sha256'), 'snapshot')
            try:
                data = base64.b64decode(item['base64'], validate=True)
            except (ValueError, TypeError) as exc:
                raise TransferError('恢复快照编码非法') from exc
            require(len(data) <= MAX_LOCAL and digest(data) == item['sha256'], '恢复快照摘要不符')
            decoded[name] = data
        ai_path(bridge, name)
    for name, data in decoded.items():
        path = ai_path(bridge, name)
        if data is None:
            if path.exists(): path.unlink(); sync_dir(path.parent)
        else:
            durable_write(bridge, path, data)


class Transaction:
    """Ordinary failures roll back; a hard crash leaves a fail-closed intent."""
    def __init__(self, bridge, names):
        self.bridge, self.names = bridge, names

    def __enter__(self):
        self.snapshots = {}
        for name in self.names:
            require(valid_snapshot_name(name), '事务路径越界')
            path = ai_path(self.bridge, name)
            data = read_bytes(path) if path.exists() else None
            self.snapshots[name] = None if data is None else {'base64': base64.b64encode(data).decode(), 'sha256': digest(data)}
        self.marker = ai_path(self.bridge, 'transfer.pending.json')
        durable_write(self.bridge, self.marker, canonical(envelope('agent_recovery/v1', self.snapshots)), exclusive=True)
        return self

    def __exit__(self, kind, value, trace):
        if kind is not None:
            restore(self.bridge, self.snapshots)
        # Legacy writers close files but do not fsync. Complete every changed
        # file before dropping the crash marker or publishing any handoff.
        for name in self.names:
            path = ai_path(self.bridge, name)
            if path.exists():
                descriptor = os.open(str(path), os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0))
                try: os.fsync(descriptor)
                finally: os.close(descriptor)
                sync_dir(path.parent)
        self.marker.unlink(); sync_dir(self.marker.parent)
        return False


def guard_legacy(bridge, args):
    ensure_local_paths(bridge)
    if ai_path(bridge, 'transfer.pending.json').exists():
        raise TransferError('移交事务未完整结束；先运行recover，禁止覆盖部分状态')
    if ai_path(bridge, 'host.json').exists():
        state(bridge)  # A bootstrapped host never silently adopts a copied state.root.


def cmd_bootstrap(bridge, args):
    root, _ = repository(bridge); clean_tools(bridge, root); ensure_local_paths(bridge)
    require(not ai_path(bridge, 'transfer.pending.json').exists(), '先recover未完成事务')
    require((root / 'note/09_跨工具协作契约.md').is_file(), '缺项目协作契约')
    runner, pointer = ai_path(bridge, 'bridge.py'), no_symlink(root / 'AGENTS.md')
    previous = read_bytes(runner) if runner.exists() else None
    if previous is not None and previous != SHIM.encode():
        require(digest(previous) == KNOWN_LEGACY or args.replace_legacy, '未知本机runner；审查后用--replace-legacy保留备份再迁移')
    if pointer.exists():
        require('note/09_跨工具协作契约.md' in read_bytes(pointer).decode('utf-8'), '已有AGENTS不含共同指针，不覆盖')
    if ai_path(bridge, 'host.json').exists():
        host = local_host(bridge); require(host['platform'] == args.platform, '本机platform已登记，不隐式重绑')
    else:
        host = {'schema': 'agent_local_host/v1', 'project': PROJECT, 'host_id': str(uuid.uuid4()),
                'platform': args.platform, 'created_at': timestamp()}
    if Path(bridge.STATE).exists():
        state(bridge)
    with bridge.Lock():
        # Repeat mutable inputs after acquiring the lock: two bootstrap
        # processes must report the same persisted identity and keep backups.
        previous = read_bytes(runner) if runner.exists() else None
        if previous is not None and previous != SHIM.encode():
            require(digest(previous) == KNOWN_LEGACY or args.replace_legacy, '未知本机runner；审查后用--replace-legacy迁移')
        if pointer.exists():
            require('note/09_跨工具协作契约.md' in read_bytes(pointer).decode('utf-8'), '已有AGENTS不含共同指针，不覆盖')
        if ai_path(bridge, 'host.json').exists():
            host = local_host(bridge); require(host['platform'] == args.platform, '本机platform已登记，不隐式重绑')
        if Path(bridge.STATE).exists(): state(bridge)
        if previous is not None and previous != SHIM.encode():
            backup = directory(bridge, 'legacy') / ('bridge-' + digest(previous) + '.py')
            if not backup.exists(): durable_write(bridge, backup, previous, exclusive=True)
        if not Path(bridge.STATE).exists(): bridge._init_state(argparse.Namespace(force=False))
        if not runner.exists() or read_bytes(runner) != SHIM.encode(): durable_write(bridge, runner, SHIM.encode())
        if not pointer.exists(): durable_write(bridge, pointer, POINTER.encode(), exclusive=True)
        if not ai_path(bridge, 'host.json').exists(): durable_write(bridge, ai_path(bridge, 'host.json'), canonical(host), exclusive=True)
        if not Path(bridge.JOURNAL).exists(): durable_write(bridge, Path(bridge.JOURNAL), '# 协作日志\n'.encode())
        for name in ('inbox', 'outbox', 'legacy'): directory(bridge, name)
    print(json.dumps({'bootstrapped': str(root), 'host_id': host['host_id'], 'platform': host['platform'],
                      'state_preserved': True}, ensure_ascii=False))


def cmd_host(bridge, args):
    value = ticket(bridge)
    if args.out: publish(bridge, args.out, canonical(value))
    print(json.dumps(value, ensure_ascii=False, indent=2))


def cmd_inspect(bridge, args):
    payload = validate_package(read_json(args.package)); core = payload['core']
    result = {'package_id': core['package_id'], 'task_id': core['task']['id'], 'task_uuid': core['task_uuid'],
              'hop': core['hop'], 'code_commit': core['code_commit'], 'source': core['source'],
              'destination': core['destination'], 'recipient_agent': core['recipient_agent'],
              'scope': core['task']['scope'], 'summary': core['summary'], 'evidence': core['evidence'],
              'lease': payload['lease'], 'lease_current': parse_time(payload['lease']['issued_at']) <= utc_now() < parse_time(payload['lease']['expires_at'])}
    if args.show_card: result['card'] = core['card']
    print(json.dumps(result, ensure_ascii=False, indent=2))


def lease(hours, revision):
    integer(hours, 1, 168, 'ttl-hours'); now = utc_now()
    return {'revision': revision, 'issued_at': timestamp(now), 'expires_at': timestamp(now + timedelta(hours=hours))}


def cmd_export(bridge, args):
    root, head = repository(bridge); clean_tools(bridge, root); host = local_host(bridge)
    require(TASK_ID.fullmatch(args.id), '非法任务ID')
    with bridge.Lock():
        st = state(bridge); task = st['tasks'].get(args.id); require(task is not None, '任务不存在')
        require(task.get('owner') == args.as_agent, 'export须由任务现任agent执行')
        outgoing = task.get('outgoing_transfer')
        if outgoing:
            require(task['status'] == 'transferred', 'outgoing状态不一致')
            private = ai_path(bridge, 'outbox/' + outgoing['package_id'] + '.json')
            value = read_json(private); payload = validate_package(value); core = payload['core']
            require(payload['core_sha256'] == outgoing['core_sha256'] and core['source']['host_id'] == host['host_id'] and
                    core['task_uuid'] == task.get('task_uuid') and core['hop'] == task.get('transfer_hop'), '私有outbox与fence不一致')
            if args.destination:
                dest = validate_ticket(read_json(args.destination)); require(core['destination'] == {k: dest[k] for k in ('host_id', 'platform')}, '不能改变接收主机')
            if args.to: require(args.to == core['recipient_agent'], '不能改变接收agent')
            if args.summary: require(args.summary == core['summary'], '不能改变交接core')
            if args.evidence: require(args.evidence == core['evidence'], '不能改变证据core')
            if args.renew:
                payload['lease'] = lease(args.ttl_hours, payload['lease']['revision'] + 1)
                value = envelope('agent_handoff/v1', payload)
                with Transaction(bridge, transaction_paths(bridge, args.id, ('outbox/' + core['package_id'] + '.json',))):
                    durable_write(bridge, private, canonical(value))
            data = canonical(value)
        else:
            require(not args.renew, '新移交不能renew')
            require(st.get('current_task') == args.id and task['status'] in bridge.OPEN_STATUS, '只能移交当前未完成任务')
            require(task.get('owner') == args.as_agent, 'export须由任务现任agent执行')
            require(not task.get('blocked_by'), '任务仍有blocked_by依赖；先在源端解决依赖，不跨主机丢弃引用')
            require(args.destination and args.summary and args.to, '首次export需要--destination/--to/--summary')
            require(git(root, 'rev-parse', 'HEAD').decode().strip() == head, '等待锁期间HEAD已变化')
            dest = validate_ticket(read_json(args.destination)); require(dest['code_commit'] == head, '接收票据与源HEAD不同，先对齐代码')
            require(dest['host_id'] != host['host_id'], '同主机使用handoff')
            validate_scope(task['scope'])
            for item in task['scope']: no_symlink(root / item.rstrip('/'))
            require(not dirty_scope(root, task['scope']), '任务scope有未提交改动，先完成本地阶段提交')
            stable = task.get('task_uuid') or str(uuid.uuid4()); identifier(stable, 'task_uuid')
            origin = task.get('origin') or {'host_id': host['host_id'], 'task_id': args.id}
            core = {'package_id': str(uuid.uuid4()), 'task_uuid': stable,
                    'hop': task.get('transfer_hop', 0) + 1,
                    'source': {k: host[k] for k in ('host_id', 'platform')},
                    'destination': {k: dest[k] for k in ('host_id', 'platform')},
                    'recipient_agent': args.to, 'code_commit': head, 'created_at': timestamp(),
                    'task': {**{k: copy.deepcopy(task.get(k, '')) for k in ('id', 'title', 'scope', 'created_at', 'created_date', 'latest_handoff')}, 'origin': origin},
                    'card': read_bytes(ai_path(bridge, task['file']), 256 * 1024).decode('utf-8'),
                    'summary': args.summary, 'evidence': args.evidence}
            payload = {'core': core, 'core_sha256': digest(canonical(core)), 'lease': lease(args.ttl_hours, 0)}
            value = envelope('agent_handoff/v1', payload); validate_package(value); data = canonical(value)
            private = directory(bridge, 'outbox') / (core['package_id'] + '.json')
            durable_write(bridge, private, data, exclusive=True)
            try:
                with Transaction(bridge, transaction_paths(bridge, args.id)):
                    task.update(status='transferred', task_uuid=stable, origin=origin, transfer_hop=core['hop'],
                                updated_at=bridge.ts(), latest_handoff=args.summary,
                                outgoing_transfer={'package_id': core['package_id'], 'core_sha256': payload['core_sha256'],
                                                   'destination': core['destination'], 'hop': core['hop']})
                    st.setdefault('transfers', {}).setdefault('outgoing', {})[core['package_id']] = copy.deepcopy(task['outgoing_transfer'])
                    st['current_task'] = None; bridge.bump(st)
                    bridge.append_to_card(args.id, bridge.SECTION_HANDOFF, bridge.ts() + ' · 跨主机移交',
                                          'package=' + core['package_id'] + '\n源端已隔离，目标只接续任务，不获得额外设备许可。\n' + args.summary)
                    bridge.refresh_card_meta(task); bridge.commit(st, '跨主机导出 ' + args.id + ' -> ' + core['destination']['host_id'])
            except BaseException:
                if not ai_path(bridge, 'transfer.pending.json').exists():
                    private.unlink(); sync_dir(private.parent)
                raise
        # The source is durably fenced before output publication. Any failure
        # here leaves it fenced; retry publishes this SAME immutable core.
        publish(bridge, args.out, data)
    print(json.dumps({'exported': args.out, 'package_id': core['package_id'], 'task_uuid': core['task_uuid'],
                      'hop': core['hop'], 'source_status': 'transferred', 'lease': payload['lease']}, ensure_ascii=False))


def receipt_state(bridge, st, core, payload, receipt):
    require(receipt['core_sha256'] == payload['core_sha256'], '同package_id的core被改写')
    task = st['tasks'].get(receipt['local_id'])
    require(task is not None and task.get('task_uuid') == core['task_uuid'], '收据对应任务缺失/身份变化')
    body = read_bytes(ai_path(bridge, task['file'])).decode('utf-8')
    require(body.startswith('# `' + task['id'] + '`'), '本机任务卡头缺失/不一致')
    saved = validate_package(read_json(ai_path(bridge, 'inbox/' + core['package_id'] + '.json')))
    require(saved['core_sha256'] == receipt['core_sha256'], '私有接收core损坏')
    return receipt


def cmd_import(bridge, args):
    payload = validate_package(read_json(args.package)); core = payload['core']
    root, head = repository(bridge); clean_tools(bridge, root); host = local_host(bridge)
    require(core['destination'] == {k: host[k] for k in ('host_id', 'platform')}, '目标主机UUID/platform不符')
    require(args.as_agent == core['recipient_agent'], '接收agent不符')
    with bridge.Lock():
        st = state(bridge); ledger = st.get('transfers', {}).get('received', {})
        if core['package_id'] in ledger:
            receipt = receipt_state(bridge, st, core, payload, ledger[core['package_id']])
            print(json.dumps({'already_imported': True, **receipt}, ensure_ascii=False)); return
        lease_valid(payload); require(head == core['code_commit'], '目标HEAD与包绑定提交不同')
        require(git(root, 'rev-parse', 'HEAD').decode().strip() == head, '等待锁期间HEAD已变化')
        require(st.get('current_task') is None, '目标已有当前任务，不能覆盖')
        require(not any(t['status'] == 'in_progress' for t in st['tasks'].values()), '目标已有执行中任务')
        for item in core['task']['scope']: no_symlink(root / item.rstrip('/'))
        require(not dirty_scope(Path(bridge.ROOT), core['task']['scope']), '目标scope有未提交改动，不能接续')
        existing = [t for t in st['tasks'].values() if t.get('task_uuid') == core['task_uuid']]
        require(len(existing) <= 1, '本机重复task_uuid')
        if existing:
            task = existing[0]; outgoing = task.get('outgoing_transfer')
            require(task['status'] == 'transferred' and outgoing and
                    outgoing['destination'] == core['source'] and core['hop'] == outgoing['hop'] + 1,
                    '旧包/非直接回流不能解除本机fence')
            local_id = task['id']
        else:
            local_id = core['task']['id']
            if local_id in st['tasks'] or ai_path(bridge, 'tasks/' + local_id + '.md').exists():
                local_id = bridge.next_id(st)
                while local_id in st['tasks'] or ai_path(bridge, 'tasks/' + local_id + '.md').exists():
                    local_id = bridge.next_id(st)
            prefix, number = local_id.rsplit('-', 1)
            st.setdefault('counters', {})[prefix] = max(st['counters'].get(prefix, 0), int(number))
        task = {'id': local_id, 'title': core['task']['title'], 'scope': copy.deepcopy(core['task']['scope']),
                'status': 'review', 'owner': args.as_agent, 'created_at': core['task']['created_at'],
                'created_date': core['task']['created_date'], 'updated_at': bridge.ts(), 'blocked_by': [],
                'file': 'tasks/' + local_id + '.md', 'latest_handoff': core['summary'],
                'task_uuid': core['task_uuid'], 'origin': core['task']['origin'], 'transfer_hop': core['hop']}
        receipt = {'package_id': core['package_id'], 'core_sha256': payload['core_sha256'], 'local_id': local_id,
                   'source_id': core['task']['id'], 'task_uuid': core['task_uuid'], 'hop': core['hop']}
        private_name = 'inbox/' + core['package_id'] + '.json'; directory(bridge, 'inbox')
        with Transaction(bridge, transaction_paths(bridge, local_id, (private_name,))):
            durable_write(bridge, ai_path(bridge, private_name), canonical(envelope('agent_handoff/v1', payload)), exclusive=True)
            body = bridge.render_task_card(task) + '\n## 跨主机接收来源\n\n'
            body += '源task=' + core['task']['id'] + '；package=' + core['package_id'] + '；hop=' + str(core['hop']) + '\n\n'
            body += '原卡仅作上下文；接续先核本机环境、当前授权与作用域，不获得设备动作许可。\n\n'
            body += '\n'.join('    ' + line for line in core['card'].splitlines()) + '\n'
            durable_write(bridge, ai_path(bridge, task['file']), body.encode('utf-8'), exclusive=not existing)
            st['tasks'][local_id] = task; st['current_task'] = local_id
            st.setdefault('transfers', {}).setdefault('received', {})[core['package_id']] = receipt
            bridge.bump(st); bridge.commit(st, '跨主机接收 ' + core['package_id'] + ' -> ' + local_id)
    print(json.dumps({'already_imported': False, **receipt}, ensure_ascii=False))


def cmd_recover(bridge, args):
    repository(bridge); ensure_local_paths(bridge)
    require(ai_path(bridge, 'transfer.pending.json').exists(), '没有待恢复事务；已公开的移交不能撤销')
    with bridge.Lock(allow_pending=True):
        marker = ai_path(bridge, 'transfer.pending.json')
        snapshots = unwrap(read_json(marker, MAX_LOCAL * 8), 'agent_recovery/v1')
        restore(bridge, snapshots); marker.unlink(); sync_dir(marker.parent)
    print('未公开事务已恢复；已公开的移交fence不会被recover撤销。')


def register_cli(sub):
    def add(name, function):
        parser = sub.add_parser(name)
        parser.add_argument('--as', dest='as_agent', choices=AGENTS, default=os.environ.get('AI_BRIDGE_AGENT', 'human'))
        parser.set_defaults(crosshost=True, crosshost_function=function)
        return parser
    parser = add('bootstrap', cmd_bootstrap)
    parser.add_argument('--platform', required=True, choices=PLATFORMS)
    parser.add_argument('--replace-legacy', action='store_true')
    parser = add('host', cmd_host); parser.add_argument('--out')
    parser = add('export', cmd_export); parser.add_argument('id')
    parser.add_argument('--destination'); parser.add_argument('--to', choices=AGENTS)
    parser.add_argument('--summary'); parser.add_argument('--evidence', action='append', default=[])
    parser.add_argument('--ttl-hours', type=int, default=24); parser.add_argument('--renew', action='store_true')
    parser.add_argument('--out', required=True)
    parser = add('inspect', cmd_inspect); parser.add_argument('package'); parser.add_argument('--show-card', action='store_true')
    parser = add('import', cmd_import); parser.add_argument('package')
    add('recover', cmd_recover)


def dispatch(bridge, args):
    try:
        args.crosshost_function(bridge, args)
        return 0
    except (TransferError, OSError, UnicodeError, ValueError, KeyError, TypeError, RecursionError) as exc:
        sys.stderr.write('ERROR: ' + str(exc) + '\n')
        return 2
