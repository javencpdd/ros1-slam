#!/usr/bin/env python3
"""Read-only checks for the explicitly covered engineering-rule documents.

0: covered text/pointers agree; 5: text conflicts; 9: input/coverage invalid.
This is not a runtime, build-isolation, algorithm or commit-completion gate.
"""
import argparse
import datetime
import json
from pathlib import Path, PurePosixPath
import posixpath
import re
import stat
import sys
import unittest

sys.dont_write_bytecode = True  # --selftest must not create workspace __pycache__.

from urllib.parse import unquote

STANDARD = 'note/架构/工程标准.md'
CATALOG = 'tooling/standards/rules.json'
SKILL_ROOT = 'tooling/skill'
SKILLS = {
    'tooling/skill/software-company/SKILL.md',
    'tooling/skill/senior-developer/SKILL.md',
    'tooling/skill/senior-developer/agents/senior-developer.md',
    'tooling/skill/window-scheduler/SKILL.md',
    'tooling/skill/window-scheduler/references/queue-format.md',
    'tooling/skill/note/software-company-使用说明.md',
    'tooling/skill/note/senior-developer-使用说明.md',
} | {'tooling/skill/software-company/agents/%s.md' % name for name in (
    '主理人-佳文', '计划与风险管家-小王', '部署架构师-小张', '施工工程师-小明', '验收工程师-小美')}
ENTRIES = {p: ('skill', 'full') for p in SKILLS}
ENTRIES.update({p: ('entry', 'full') for p in (
    'README.md', 'ws/README.md', 'docker/README.md', 'tooling/README.md',
    'note/架构/项目架构与目录.md', 'note/计划书v8/README.md')})
ENTRIES.update({p: ('reference', 'full') for p in (
    'note/09_跨工具协作契约.md', 'note/计划书v8/28_工程标准统一补丁.md')})
ENTRIES.update({p: ('scoped_rules', 'markers') for p in (
    'note/计划书v8/23_v8任务队列.md', 'note/计划书v8/25_v8自动调度作业单.md')})
ENTRIES['note/00_README.md'] = ('pointer_index', 'current_pointer')
ENTRIES[STANDARD] = ('standard', 'full')
BEGIN = '<!-- standards-scope:start -->'
END = '<!-- standards-scope:end -->'
LINK = re.compile(r'\[[^\]\n]+\]\(([^)\n]+)\)')
# A literal pointer is a complete path token, not a substring of another path.
# Markdown links are checked through their destination, not their label.
LITERAL_STANDARD = re.compile(
    r"(?<![^\s`'\"“”‘’<>()\[\]{}，。；、：！？])" + re.escape(STANDARD) +
    r"(?![^\s`'\"“”‘’<>()\[\]{}，。；、：！？])")
UPDATED = re.compile(r'最新更新时间[：:]\s*(\d{4}-\d{2}-\d{2}(?:[ T]\d{2}:\d{2}(?::\d{2})?)?)')
# These are known directive shapes, not a general natural-language interpreter.
# A denial may suppress only the immediately adjacent directive, never a whole
# line merely because it says "历史", "现读" or "指针".
DENIAL = re.compile(r'(?:不|禁止|不得|不能|不应|无需|严禁|避免)(?:再|去|将|默认)?\s*$')
CONFLICTS = (
    ('legacy_stage', r'S0\s*[—–~\-]\s*S4|(?:Lite3|M20)[^。；\n]{0,16}冻结[^。；\n]{0,12}S4'),
    ('legacy_plan', r'(?:现行(?:中心)?计划书|现读|现行阶段|阶段表)\s*[＝=:：为]*\s*v[3-7](?:\.\d+)?'),
    ('background_watch', r'(?:启动|起|运行|挂|开启|启用)[^。；\n]{0,12}?(?:后台看守|后台\s*watch|window_watch\.sh)|起看守'),
    ('skip_commit', r'(?:不(?:需要|必|用)?|无需|免于|跳过)\s*(?:git\s+)?(?:commit|提交)'),
    ('missing_state_default', r'(?:缺(?:少)?状态|未填状态|状态(?:为空|缺失|不明))[^。；\n]{0,18}?(?P<action>默认(?:为|成|按)?|视为|当作)[^。；\n]{0,8}?(?:未开工|待办|可开工|可执行)'),
    ('revive_canceled', r'(?:取消|废弃|被替代)(?:的)?(?:任务|施工|排程)\s*(?<!不)(?:自动恢复|重启|重新执行|照常执行)|(?:必须|应当|自动)\s*(?:恢复|重启)(?:已)?取消(?:的)?(?:任务|施工|排程)'),
    ('state_not_accepted', r'(?:取|选取|选择)[^。；\n]{0,12}状态\s*(?:≠|不等于)\s*已验收|按状态\s*(?:≠|不等于)\s*已验收[^。；\n]{0,8}(?:选择|选取|执行|开工)'),
    ('automatic_push', r'(?:自动|必须|直接)\s*(?:git\s+)?push'),
    ('robot_sim_build', r'(?:Lite3|M20)(?:端|真机|机器人)?\s*(?:也)?(?:编译|安装)\s*(?:Gazebo|MuJoCo|仿真)'),
)
PATTERNS = tuple((name, re.compile(pattern, re.I)) for name, pattern in CONFLICTS)
LIMITATION = ('仅检查声明的文案模式、指针、章节及作用域；不解释任意同义句，'
              '不证明构建/运行隔离、算法可替换、训练或真机验收，也不验证阶段已提交。')


class InvalidInput(ValueError):
    pass


def relative_name(name):
    if not isinstance(name, str) or not name or '\\' in name or '\x00' in name:
        raise InvalidInput('路径必须是非空的仓内 POSIX 相对路径')
    pure = PurePosixPath(name)
    if pure.is_absolute() or any(p in ('.', '..') for p in name.split('/')) or str(pure) != name:
        raise InvalidInput('路径不规范或越出仓根: %r' % name)
    return pure


def safe_path(root, name, require_file=True):
    relative_name(name)
    path = root
    for part in PurePosixPath(name).parts:
        path = path / part
        if path.is_symlink():
            raise InvalidInput('覆盖路径不允许软链: %s' % name)
    try:
        path.resolve(strict=True).relative_to(root)
        mode = path.stat().st_mode
    except (OSError, ValueError) as exc:
        raise InvalidInput('路径缺失或越界: %s (%s)' % (name, exc)) from exc
    if require_file and not stat.S_ISREG(mode):
        raise InvalidInput('覆盖路径不是普通文件: %s' % name)
    return path


def read_doc(root, name):
    try:
        text = safe_path(root, name).read_text(encoding='utf-8')
    except (OSError, UnicodeError) as exc:
        raise InvalidInput('无法读取 UTF-8 文件: %s (%s)' % (name, exc)) from exc
    if not text.strip():
        raise InvalidInput('文件为空: %s' % name)
    return text


def unique_object(items):
    value = {}
    for key, item in items:
        if key in value:
            raise InvalidInput('JSON 重复键: %s' % key)
        value[key] = item
    return value


def load_catalog(root, rules_path=None):
    # --rules may select an external test/config file explicitly. Its entries
    # are still constrained to ordinary files inside --root.
    if isinstance(rules_path, str) and rules_path == '':
        raise InvalidInput('显式规则路径不可为空')
    path = Path(rules_path).absolute() if rules_path is not None else safe_path(root, CATALOG)
    if any(part.is_symlink() for part in (path, *path.parents)) or not path.is_file():
        raise InvalidInput('规则配置不是普通文件或路径含软链: %s' % path)
    try:
        config = json.loads(path.read_text(encoding='utf-8'), object_pairs_hook=unique_object)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise InvalidInput('规则 JSON 不可读: %s' % exc) from exc
    if not isinstance(config, dict) or set(config) != {'schema_version', 'standard', 'entries'}:
        raise InvalidInput('规则配置顶层字段不合规格')
    if type(config['schema_version']) is not int or config['schema_version'] != 1:
        raise InvalidInput('不支持的规则 schema_version')
    if config['standard'] != STANDARD:
        raise InvalidInput('工程标准真源不得在配置中另设')
    rows = config['entries']
    if not isinstance(rows, list) or not rows:
        raise InvalidInput('覆盖面为空或不是列表')
    catalog = {}
    for row in rows:
        if not isinstance(row, dict) or set(row) != {'path', 'profile', 'scope'}:
            raise InvalidInput('覆盖条目字段不合规格')
        name = row['path']
        relative_name(name)
        if name in catalog:
            raise InvalidInput('覆盖路径重复: %s' % name)
        if name not in ENTRIES or (row['profile'], row['scope']) != ENTRIES[name]:
            raise InvalidInput('未知覆盖路径/profile/scope: %s' % name)
        catalog[name] = row
    if set(catalog) != set(ENTRIES):
        raise InvalidInput('固定入口覆盖不全: 缺 %s' % sorted(set(ENTRIES) - set(catalog)))
    return catalog


def discover_skills(root):
    base = safe_path(root, SKILL_ROOT, require_file=False)
    found = set()
    for package in sorted(base.iterdir()):
        if package.is_symlink():
            raise InvalidInput('技能根含软链，覆盖不可信: %s' % package.name)
        if not package.is_dir():
            continue
        entry = package / 'SKILL.md'
        if entry.exists() or entry.is_symlink():
            found.add(entry.relative_to(root).as_posix())
            for role in (package / 'agents').rglob('*'):
                if role.is_symlink():
                    raise InvalidInput('角色目录含软链: %s' % role.relative_to(root))
                if role.is_file() and role.suffix.lower() == '.md':
                    found.add(role.relative_to(root).as_posix())
    for name in ('software-company-使用说明.md', 'senior-developer-使用说明.md'):
        found.add('%s/note/%s' % (SKILL_ROOT, name))
    found.add('%s/window-scheduler/references/queue-format.md' % SKILL_ROOT)
    if found != SKILLS:
        raise InvalidInput('活技能发现集与名册不一致: 新增=%s 缺失=%s' %
                           (sorted(found - SKILLS), sorted(SKILLS - found)))
    return found


def scope_lines(text, kind, path):
    lines = text.splitlines()
    if kind == 'full':
        return list(enumerate(lines, 1))
    if kind == 'markers':
        starts = [i for i, line in enumerate(lines) if line.strip() == BEGIN]
        ends = [i for i, line in enumerate(lines) if line.strip() == END]
    else:
        starts = [i for i, line in enumerate(lines) if line.startswith('## ★现行指针（')]
        ends = [i for i, line in enumerate(lines) if line.startswith('## 规则一')]
    if len(starts) != 1 or len(ends) != 1 or starts[0] >= ends[0]:
        raise InvalidInput('作用域边界缺失、重复或反序: %s' % path)
    selected = [(i + 1, lines[i]) for i in range(starts[0] + 1, ends[0])]
    if not any(line.strip() for _, line in selected):
        raise InvalidInput('当前规则作用域为空: %s' % path)
    return selected


def has_standard_pointer(root, name, text):
    if LITERAL_STANDARD.search(LINK.sub('', text)):
        return True
    for target in LINK.findall(text):
        target = unquote(target.strip('<>').split('#', 1)[0])
        if not target or re.match(r'^[a-z]+:', target, re.I):
            continue
        candidate = root / name
        try:
            if (candidate.parent / target).resolve() == root / STANDARD:
                return True
        except (OSError, ValueError):
            continue
    return False


def issue(issues, rule, path, line, message):
    issues.append({'rule': rule, 'path': path, 'line': line, 'message': message})


def check_policy_source(text, issues):
    ids = re.findall(r'^## (STD-\d\d)\b', text, re.M)
    if ids != ['STD-%02d' % n for n in range(1, 7)]:
        raise InvalidInput('工程标准 STD-01..06 章节必须各出现一次且按序')
    anchors = {
        'STD-01': ('git commit', 'push', '用户'),
        'STD-02': ('Lite3', 'M20', 'x86', '不编译仿真'),
        'STD-03': ('manifest', '适配器', '业务', '版本'),
        'STD-04': ('TF', '单调', '停车'),
        'STD-05': ('取消', '缺状态', '后台', 'FORCE'),
        'STD-06': ('PASS', 'FAIL', 'INVALID', '训练', '最新更新时间'),
    }
    for key, words in anchors.items():
        part = text.split('## ' + key, 1)[1].split('\n## ', 1)[0]
        plain = part.replace('`', '').replace('*', '')
        for word in words:
            if word not in plain:
                issue(issues, 'policy_anchor', STANDARD, 0, '%s 缺政策锚: %s' % (key, word))


def check_pointer_index(root, name, text, issues):
    for label in ('现行中心计划书指针', '当前执行队列指针', '现行调度作业单指针'):
        lines = [line for line in text.splitlines() if '**' + label in line]
        if len(lines) != 1 or not LINK.search(lines[0]):
            raise InvalidInput('现行入口标签/链接缺失或重复: %s' % label)
        target = LINK.search(lines[0]).group(1)
        target = unquote(target.split('#', 1)[0].strip('<>'))
        name_target = posixpath.normpath(str(PurePosixPath(name).parent / target))
        try:
            safe_path(root, name_target)
        except (OSError, ValueError) as exc:
            raise InvalidInput('现行入口悬空/越界: %s' % label) from exc
        expected = {'现行中心计划书指针': 'note/计划书v8/21_ros1-slam中心计划书-v8.md',
                    '当前执行队列指针': 'note/计划书v8/23_v8任务队列.md',
                    '现行调度作业单指针': 'note/计划书v8/25_v8自动调度作业单.md'}[label]
        if name_target != expected:
            issue(issues, 'current_pointer', name, 0, '%s 与覆盖目录不一致' % label)


def inspect(root, rules_path=None):
    result = {'rc': 9, 'coverage': {}, 'issues': [], 'limitation': LIMITATION}
    try:
        if isinstance(root, str) and root == '':
            raise InvalidInput('显式仓根不可为空')
        root = Path(root).absolute()
        if any(part.is_symlink() for part in (root, *root.parents)) or not root.is_dir():
            raise InvalidInput('仓根不存在、不是目录或是软链')
        root = root.resolve()
        catalog = load_catalog(root, rules_path)
        discover_skills(root)
        # Read all inputs before drawing conclusions from partial coverage.
        docs = {name: read_doc(root, name) for name in catalog}
        selected = {name: scope_lines(docs[name], row['scope'], name) for name, row in catalog.items()}
        check_policy_source(docs[STANDARD], result['issues'])
        for name, row in catalog.items():
            text = docs[name]
            scoped = '\n'.join(line for _, line in selected[name])
            profile = row['profile']
            result['coverage'][profile] = result['coverage'].get(profile, 0) + 1
            stamp = UPDATED.search(text)
            try:
                if stamp is None:
                    raise ValueError('missing')
                datetime.datetime.fromisoformat(stamp.group(1))
            except ValueError:
                issue(result['issues'], 'updated', name, 0, '缺有效最新更新时间')
            if not has_standard_pointer(root, name, scoped):
                issue(result['issues'], 'standard_pointer', name, 0, '当前范围缺工程标准真源路径/有效链接')
            if profile == 'pointer_index':
                check_pointer_index(root, name, scoped, result['issues'])
            if profile not in ('skill', 'scoped_rules'):
                continue
            plain = scoped.replace('`', '').replace('*', '')
            required = ('本地', 'git commit', 'push', '用户')
            if profile == 'skill':
                required += ('note/09_跨工具协作契约.md', 'note/00_README.md')
            for word in required:
                if word not in plain:
                    issue(result['issues'], 'required_anchor', name, 0, '缺当前规则锚: %s' % word)
            for line_no, line in selected[name]:
                line = line.replace('`', '').replace('*', '')
                for clause in re.split(r'[。；;]', line):
                    for rule, pattern in PATTERNS:
                        for match in pattern.finditer(clause):
                            action = match.start('action') if match.groupdict().get('action') else match.start()
                            if (DENIAL.search(clause[:match.start()].rstrip()) or
                                    DENIAL.search(clause[:action].rstrip())):
                                continue
                            issue(result['issues'], rule, name, line_no, match.group(0))
        result['rc'] = 5 if result['issues'] else 0
    except (InvalidInput, OSError, UnicodeError) as exc:
        result['error'] = str(exc)
    return result


def run_selftest(directory=None):
    directory = Path(directory) if directory is not None else Path(__file__).parent
    module = directory / 'test_project_standards.py'
    if not module.is_file() or module.is_symlink():
        print('PROJECT_STANDARDS SELFTEST rc=9: 相邻测试模块缺失或不是普通文件')
        return 9
    try:
        suite = unittest.defaultTestLoader.discover(str(directory), pattern=module.name)
    except (ImportError, OSError) as exc:
        print('PROJECT_STANDARDS SELFTEST rc=9: 无法发现测试: %s' % exc)
        return 9
    if suite.countTestCases() == 0:
        print('PROJECT_STANDARDS SELFTEST rc=9: 测试集为空')
        return 9
    return 0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 9


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', default=str(Path(__file__).resolve().parents[2]))
    parser.add_argument('--rules', help='显式读取另一份覆盖配置，仍严格核对覆盖名册')
    parser.add_argument('--json', action='store_true', dest='json_output')
    parser.add_argument('--selftest', action='store_true', help='运行相邻测试文件中的临时夹具正反控')
    try:
        args = parser.parse_args(argv)
    except SystemExit as exc:
        return 0 if exc.code == 0 else 9
    if args.selftest and args.root != '' and args.rules != '':
        return run_selftest()
    result = inspect(args.root, args.rules)
    if args.json_output:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print('PROJECT_STANDARDS rc=%s coverage=%s' % (result['rc'], result['coverage']))
        if result.get('error'):
            print('[前置无效] ' + result['error'])
        for row in result['issues']:
            print('[冲突] %s:%s %s: %s' % (row['path'], row['line'], row['rule'], row['message']))
        print(result['limitation'])
    return result['rc']


if __name__ == '__main__':
    sys.exit(main())
