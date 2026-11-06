#!/usr/bin/env python3
"""Behavioral positive/negative controls for the current-document checker.

Fixtures copy real covered documents into TemporaryDirectory, so historical
contrast tables and actual prohibitions are exercised without touching source.
"""
import copy
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import v_project_standards as checker

ROOT = Path(__file__).resolve().parents[2]
SKILL = 'tooling/skill/window-scheduler/SKILL.md'
QUEUE = 'tooling/skill/window-scheduler/references/queue-format.md'
SCHEDULE = 'note/计划书v8.1/25_v8.1执行与迁移规则.md'
CURRENT_QUEUE = 'note/计划书v8.1/23_v8.1任务队列.md'


def copy_fixture(source_root, fixture_root):
    """Validate the complete read set before writing into a fresh owned tree."""
    source_root, fixture_root = Path(source_root).absolute(), Path(fixture_root).absolute()
    for base in (source_root, fixture_root):
        if (not base.is_dir() or
                any(part.is_symlink() for part in (base, *base.parents))):
            raise checker.InvalidInput('夹具源/目标必须是普通目录，路径不含软链')
    catalog = checker.load_catalog(source_root)
    names = list(catalog) + ['note/计划书v8.1/21_ros1-slam中心计划书-v8.1.md', checker.CATALOG]
    # No unvalidated JSON entry is ever joined to a writable destination.
    # Read and validate every source first, so a bad late source writes nothing.
    contents = {name: checker.safe_path(source_root, name).read_bytes() for name in names}
    if any(fixture_root.iterdir()):
        raise checker.InvalidInput('夹具目标必须是新的空目录')
    for name, data in contents.items():
        dest = fixture_root / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(data)


class ProjectStandardsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='ros1-standard-controls-')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        copy_fixture(ROOT, self.root)
        self.config = json.loads((self.root / checker.CATALOG).read_text())

    def scan(self):
        return checker.inspect(self.root)

    def append(self, name, text):
        p = self.root / name
        p.write_text(p.read_text() + '\n' + text + '\n')

    def write_config(self):
        (self.root / checker.CATALOG).write_text(json.dumps(self.config, ensure_ascii=False))

    def expect_conflict(self, text, rule):
        self.append(SKILL, text)
        result = self.scan()
        self.assertEqual(5, result['rc'], result)
        self.assertIn(rule, [row['rule'] for row in result['issues']], result)

    def test_real_documents_and_historical_contrast_are_valid(self):
        self.assertIn('原分散/冲突口径', (self.root / 'note/计划书v8.1/21_ros1-slam中心计划书-v8.1.md').read_text())
        self.assertIn('不能用“状态不等于已验收”', (self.root / QUEUE).read_text())
        result = self.scan()
        self.assertEqual(0, result['rc'], result)
        self.assertEqual(25, sum(result['coverage'].values()))
        self.assertEqual(12, result['coverage']['skill'])

    def write_release_notes(self, headings):
        text = '# 版本成果总览\n\n> 最新更新时间：2026-11-05 12:00:00 CST\n'
        text += '工程标准真源：`%s`。\n\n' % checker.STANDARD
        for heading in headings:
            text += '## %s\n\n- 累计成果与适用范围。\n\n' % heading
        (self.root / checker.OVERVIEW).write_text(text)

    def test_release_numeric_order_accepts_future_major_versions(self):
        self.write_release_notes(['v10 · 当前', 'v9 · 上一版', 'v8', 'v7', 'v3—v6 · 早期'])
        result = self.scan()
        self.assertEqual(0, result['rc'], result)
        self.assertEqual(1, result['coverage']['release_notes'])

    def test_release_lexicographic_order_is_not_numeric_order(self):
        self.write_release_notes(['v9', 'v10'])
        result = self.scan()
        self.assertEqual(5, result['rc'], result)
        self.assertIn('release_order', [x['rule'] for x in result['issues']])

    def test_release_oldest_first_is_rejected(self):
        self.write_release_notes(['v7', 'v8'])
        self.assertEqual(5, self.scan()['rc'])

    def test_release_duplicate_and_overlapping_versions_are_rejected(self):
        for headings in (['v8', 'v8'], ['v8', 'v6—v8'], ['v8', 'v3—v7', 'v5']):
            with self.subTest(headings=headings):
                self.write_release_notes(headings)
                result = self.scan()
                self.assertEqual(5, result['rc'], result)
                self.assertIn('release_order', [x['rule'] for x in result['issues']])

    def test_release_requires_two_version_sections(self):
        for headings in ([], ['v8'], ['v3—v6']):
            with self.subTest(headings=headings):
                self.write_release_notes(headings)
                self.assertEqual(5, self.scan()['rc'])

    def test_release_rejects_r_batch_action_lines(self):
        self.write_release_notes(['v8', 'v7'])
        self.append(checker.OVERVIEW, '- 2026-11-05：**R1127** 执行一轮命令并登记。')
        result = self.scan()
        self.assertEqual(5, result['rc'], result)
        self.assertIn('release_batch_action', [x['rule'] for x in result['issues']])

    def test_release_rejects_all_commonmark_r_action_markers(self):
        for marker in ('-', '*', '+', '1.', '2)', '123456789.'):
            with self.subTest(marker=marker):
                self.write_release_notes(['v8', 'v7'])
                self.append(checker.OVERVIEW, marker + ' 2026-11-05：**R1127** 执行一轮命令并登记。')
                result = self.scan()
                self.assertEqual(5, result['rc'], result)
                self.assertIn('release_batch_action', [x['rule'] for x in result['issues']])

    def test_release_rejects_indented_r_batch_list_actions(self):
        for prefix in ('   - ', '  * ', '    + ', '\t1. ', '    2) ', ' \t42.\t'):
            with self.subTest(prefix=repr(prefix)):
                self.write_release_notes(['v8', 'v7'])
                self.append(checker.OVERVIEW, prefix + '2026-11-05: R1127 增加逐批执行动作。')
                self.assertEqual(5, self.scan()['rc'])

    def test_release_keeps_plain_dated_notes_and_feature_lists_valid(self):
        self.write_release_notes(['v8', 'v7'])
        for prefix in ('- ', '* ', '+ ', '1. ', '2) ', '    12) ', '\t+ '):
            self.append(checker.OVERVIEW, prefix + '2026-11-05：合并本版地图能力说明。')
            self.append(checker.OVERVIEW, prefix + '新增地图与组件功能。')
        self.assertEqual(0, self.scan()['rc'])

    def test_release_accepts_merged_version_results_without_action_ledger(self):
        self.write_release_notes(['v8 · 当前', 'v7 · 上一版'])
        self.append(checker.OVERVIEW, '### 补充成果\n\n- 地图与接口成果合并到本版说明。\n- 2026-11-05：补充本版能力边界。')
        self.assertEqual(0, self.scan()['rc'])

    def test_release_cannot_be_dropped_or_renamed_from_coverage(self):
        original = copy.deepcopy(self.config)
        self.config['entries'] = [x for x in self.config['entries'] if x['path'] != checker.OVERVIEW]
        self.write_config()
        self.assertEqual(9, self.scan()['rc'])
        self.config = original
        self.write_config()
        p = self.root / checker.OVERVIEW
        p.rename(p.with_name('renamed-overview.md'))
        self.assertEqual(9, self.scan()['rc'])

    def test_release_keeps_common_pointer_and_timestamp_requirements(self):
        self.write_release_notes(['v8', 'v7'])
        p = self.root / checker.OVERVIEW
        p.write_text(p.read_text().replace(checker.STANDARD, 'retired/' + checker.STANDARD))
        self.assertEqual(5, self.scan()['rc'])
        self.write_release_notes(['v8', 'v7'])
        p.write_text(p.read_text().replace('最新更新时间', '旧时点'))
        self.assertEqual(5, self.scan()['rc'])

    def test_adjacent_denials_and_unsubmitted_stage_rule_are_valid(self):
        self.append(SKILL, '不挂后台看守；不使用 FORCE；不得自动恢复取消的任务；禁止自动 push。未提交的阶段不算收口。')
        self.assertEqual(0, self.scan()['rc'])

    def test_history_and_pointer_words_do_not_hide_old_stage(self):
        self.expect_conflict('历史指针仍按现行阶段 S0—S4 施工。', 'legacy_stage')

    def test_history_word_does_not_hide_current_old_version(self):
        self.expect_conflict('这是历史指针，现读＝v7，继续执行。', 'legacy_plan')

    def test_background_watch_directive_is_rejected(self):
        self.expect_conflict('无旗时启动后台看守，等窗后运行。', 'background_watch')

    def test_legacy_start_watcher_shape_is_rejected(self):
        self.expect_conflict('无旗 ⇒ 起看守。', 'background_watch')

    def test_correct_denial_does_not_hide_later_conflict_on_same_line(self):
        self.expect_conflict('不启动后台看守；但本任务必须启动 window_watch.sh 后台看守。', 'background_watch')

    def test_local_denial_without_semicolon_does_not_hide_later_directive(self):
        self.expect_conflict('不启动后台看守，但本任务必须启动 window_watch.sh。', 'background_watch')

    def test_short_denial_does_not_swallow_later_watcher_directive(self):
        self.expect_conflict('禁止启动后台看守，但必须运行 window_watch.sh。', 'background_watch')

    def test_two_short_denied_watcher_actions_are_valid(self):
        self.append(SKILL, '禁止启动后台看守，但禁止运行 window_watch.sh。')
        self.assertEqual(0, self.scan()['rc'])

    def test_skipping_commit_is_rejected(self):
        self.expect_conflict('阶段完成无需 git commit。', 'skip_commit')

    def test_explicit_prohibitions_against_skipping_commit_are_valid(self):
        self.append(SKILL, '禁止跳过 git commit；不能不提交；不得不提交；禁止不需要 git commit。')
        self.assertEqual(0, self.scan()['rc'])

    def test_correct_commit_prohibition_does_not_hide_later_skip(self):
        self.expect_conflict('禁止跳过 git commit，但本阶段无需 git commit。', 'skip_commit')

    def test_adjacent_denials_of_default_state_are_valid(self):
        self.append(SKILL, '缺状态不得视为未开工；缺状态禁止默认待办；缺状态不能当作可执行；不得将缺状态视为未开工。')
        self.assertEqual(0, self.scan()['rc'])

    def test_denied_default_does_not_hide_later_state_default(self):
        self.expect_conflict('缺状态不得视为未开工，但缺状态默认待办。', 'missing_state_default')

    def test_missing_state_default_is_rejected(self):
        self.expect_conflict('缺状态默认未开工，直接选择。', 'missing_state_default')

    def test_state_default_violation_before_denial_stays_visible(self):
        self.expect_conflict('缺状态默认待办，但缺状态不得视为待办。', 'missing_state_default')

    def test_state_default_violation_after_denial_stays_visible(self):
        self.expect_conflict('缺状态不得视为待办，但缺状态默认待办。', 'missing_state_default')

    def test_short_denied_state_does_not_swallow_next_default(self):
        self.expect_conflict('缺状态不得视为待办，缺状态默认待办。', 'missing_state_default')

    def test_two_local_state_default_denials_are_valid(self):
        self.append(SKILL, '缺状态不得视为待办，但缺状态禁止默认待办。')
        self.assertEqual(0, self.scan()['rc'])

    def test_canceled_task_revival_is_rejected(self):
        self.expect_conflict('取消的任务自动恢复。', 'revive_canceled')

    def test_nonaccepted_state_is_not_an_execution_allowlist(self):
        self.expect_conflict('选取所有状态≠已验收的任务。', 'state_not_accepted')

    def test_automatic_push_is_rejected(self):
        self.expect_conflict('阶段完成后自动 git push。', 'automatic_push')

    def test_robot_simulation_build_directive_is_rejected(self):
        self.expect_conflict('Lite3真机编译 Gazebo。', 'robot_sim_build')

    def test_required_commit_anchor_cannot_be_removed(self):
        p = self.root / SKILL
        p.write_text(p.read_text().replace('git commit', '归档'))
        result = self.scan()
        self.assertEqual(5, result['rc'])
        self.assertTrue(any(row['rule'] == 'required_anchor' for row in result['issues']))

    def test_missing_pointer_is_a_text_conflict(self):
        p = self.root / 'tooling/README.md'
        p.write_text('# 工具\n\n> 最新更新时间：2026-11-05 12:00:00 CST\n')
        result = self.scan()
        self.assertEqual(5, result['rc'])
        self.assertTrue(any(row['rule'] == 'standard_pointer' for row in result['issues']))

    def set_tooling_pointer(self, pointer):
        p = self.root / 'tooling/README.md'
        p.write_text('# 工具\n\n> 最新更新时间：2026-11-05 12:00:00 CST\n' + pointer + '\n')

    def test_backup_suffix_is_not_a_standard_pointer(self):
        self.set_tooling_pointer('`note/架构/工程标准.md.bak`')
        result = self.scan()
        self.assertEqual(5, result['rc'])
        self.assertTrue(any(row['rule'] == 'standard_pointer' for row in result['issues']))

    def test_retired_directory_prefix_is_not_a_standard_pointer(self):
        self.set_tooling_pointer('`retired/note/架构/工程标准.md`')
        self.assertEqual(5, self.scan()['rc'])

    def test_markdown_backup_destination_is_not_a_standard_pointer(self):
        self.set_tooling_pointer('[工程标准](../note/架构/工程标准.md.bak)')
        self.assertEqual(5, self.scan()['rc'])

    def test_exact_bare_and_quoted_standard_tokens_are_accepted(self):
        for pointer in ('note/架构/工程标准.md', '`note/架构/工程标准.md`'):
            self.set_tooling_pointer(pointer)
            self.assertEqual(0, self.scan()['rc'])

    def test_correct_path_label_does_not_hide_wrong_markdown_destination(self):
        self.set_tooling_pointer('[note/架构/工程标准.md](../note/架构/工程标准.md.bak)')
        self.assertEqual(5, self.scan()['rc'])

    def test_exact_path_label_does_not_hide_obsolete_link(self):
        self.set_tooling_pointer('[note/架构/工程标准.md](obsolete.md)')
        self.assertEqual(5, self.scan()['rc'])

    def test_percent_escaped_standard_link_is_accepted(self):
        self.set_tooling_pointer('[工程标准](../note/%E6%9E%B6%E6%9E%84/%E5%B7%A5%E7%A8%8B%E6%A0%87%E5%87%86.md#std-01)')
        self.assertEqual(0, self.scan()['rc'])

    def test_explicit_empty_catalog_path_is_invalid(self):
        self.assertEqual(9, checker.inspect(self.root, '')['rc'])

    def test_explicit_empty_root_is_invalid(self):
        self.assertEqual(9, checker.inspect('')['rc'])

    def test_relative_standard_link_is_accepted(self):
        p = self.root / 'tooling/README.md'
        p.write_text('# 工具\n\n> 最新更新时间：2026-11-05 12:00:00 CST\n[工程标准](../note/架构/工程标准.md)\n')
        self.assertEqual(0, self.scan()['rc'])

    def test_invalid_updated_date_is_a_text_conflict(self):
        p = self.root / 'tooling/README.md'
        p.write_text('# 工具\n> 最新更新时间：2026-19-45\n' + checker.STANDARD)
        self.assertEqual(5, self.scan()['rc'])

    def test_invalid_updated_clock_is_a_text_conflict(self):
        p = self.root / 'tooling/README.md'
        p.write_text('# 工具\n> 最新更新时间：2026-11-05 39:99:00\n' + checker.STANDARD)
        self.assertEqual(5, self.scan()['rc'])

    def test_current_scope_conflict_is_not_hidden_by_historical_body(self):
        p = self.root / SCHEDULE
        p.write_text(p.read_text().replace(checker.BEGIN, checker.BEGIN + '\n历史指针：现行阶段 S0—S4。', 1))
        result = self.scan()
        self.assertEqual(5, result['rc'])
        self.assertTrue(any(row['path'] == SCHEDULE and row['rule'] == 'legacy_stage' for row in result['issues']))

    def test_historical_body_is_not_current_directive_scope(self):
        self.append(SCHEDULE, '历史原件：现行阶段 S0—S4；无旗 ⇒ 起看守；不提交。')
        self.assertEqual(0, self.scan()['rc'])

    def test_missing_duplicate_and_reversed_scope_markers_are_invalid(self):
        p = self.root / CURRENT_QUEUE
        original = p.read_text()
        variants = [original.replace(checker.BEGIN, ''), original + '\n' + checker.END,
                    original.replace(checker.BEGIN, 'SWAP').replace(checker.END, checker.BEGIN).replace('SWAP', checker.END)]
        for text in variants:
            with self.subTest(text=text[-80:]):
                p.write_text(text)
                self.assertEqual(9, self.scan()['rc'])

    def test_empty_current_scope_is_invalid(self):
        p = self.root / SCHEDULE
        p.write_text('# doc\n' + checker.BEGIN + '\n\n' + checker.END)
        self.assertEqual(9, self.scan()['rc'])

    def test_current_index_boundary_is_required(self):
        p = self.root / 'note/00_README.md'
        p.write_text(p.read_text().replace('## 规则一', '## 改名规则一', 1))
        self.assertEqual(9, self.scan()['rc'])

    def test_actual_pointer_target_must_exist(self):
        (self.root / 'note/计划书v8.1/21_ros1-slam中心计划书-v8.1.md').unlink()
        self.assertEqual(9, self.scan()['rc'])

    def test_current_pointer_target_symlink_is_invalid_even_inside_root(self):
        p = self.root / 'note/计划书v8.1/21_ros1-slam中心计划书-v8.1.md'
        data = p.read_bytes()
        p.unlink()
        other = p.with_name('other.md')
        other.write_bytes(data)
        p.symlink_to(other)
        self.assertEqual(9, self.scan()['rc'])

    def test_standard_chapters_are_not_optional_or_duplicated(self):
        p = self.root / checker.STANDARD
        original = p.read_text()
        for text in [original.replace('## STD-03 ', '## REMOVED-03 '), original + '\n## STD-01 重复\n']:
            p.write_text(text)
            self.assertEqual(9, self.scan()['rc'])

    def test_standard_policy_anchor_is_not_only_a_section_title(self):
        p = self.root / checker.STANDARD
        p.write_text(p.read_text().replace('git commit', '提交'))
        result = self.scan()
        self.assertEqual(5, result['rc'])
        self.assertTrue(any(row['rule'] == 'policy_anchor' for row in result['issues']))

    def test_empty_face_missing_entry_and_wrong_types_are_invalid(self):
        base = copy.deepcopy(self.config)
        variants = [[], base['entries'][1:], False, 'all']
        for entries in variants:
            with self.subTest(entries=type(entries).__name__):
                self.config = copy.deepcopy(base)
                self.config['entries'] = entries
                self.write_config()
                self.assertEqual(9, self.scan()['rc'])

    def test_duplicate_paths_unknown_profile_and_scope_are_invalid(self):
        base = copy.deepcopy(self.config)
        variants = [base['entries'] + [base['entries'][0]],
                    [dict(base['entries'][0], profile='history')] + base['entries'][1:],
                    [dict(base['entries'][0], scope='none')] + base['entries'][1:]]
        for entries in variants:
            self.config = copy.deepcopy(base)
            self.config['entries'] = entries
            self.write_config()
            self.assertEqual(9, self.scan()['rc'])

    def test_bad_json_duplicate_key_and_unknown_schema_are_invalid(self):
        p = self.root / checker.CATALOG
        variants = ['{', '{"schema_version":1,"schema_version":1}',
                    json.dumps(dict(self.config, schema_version=2)),
                    json.dumps(dict(self.config, schema_version=True)),
                    json.dumps(dict(self.config, unknown='ignored'))]
        for text in variants:
            p.write_text(text)
            self.assertEqual(9, self.scan()['rc'])

    def test_missing_empty_or_directory_input_is_invalid(self):
        p = self.root / SKILL
        p.unlink()
        self.assertEqual(9, self.scan()['rc'])
        p.write_text('')
        self.assertEqual(9, self.scan()['rc'])
        p.unlink()
        p.mkdir()
        self.assertEqual(9, self.scan()['rc'])

    def test_path_escape_and_aliases_are_invalid(self):
        base = copy.deepcopy(self.config)
        for name in ('../escape.md', '/escape.md', 'README.md/../README.md', './README.md', 'README.md//x', 'note\\README.md'):
            self.config = copy.deepcopy(base)
            self.config['entries'][0]['path'] = name
            self.write_config()
            self.assertEqual(9, self.scan()['rc'])

    def test_symlink_input_or_parent_is_invalid(self):
        p = self.root / SKILL
        p.unlink()
        p.symlink_to(ROOT / SKILL)
        self.assertEqual(9, self.scan()['rc'])
        p.unlink()
        p.parent.rename(p.parent.with_name('old-window'))
        p.parent.symlink_to(p.parent.with_name('old-window'), target_is_directory=True)
        self.assertEqual(9, self.scan()['rc'])

    def test_root_through_symlink_parent_is_invalid(self):
        alias = self.root / 'root-alias'
        alias.symlink_to(self.root, target_is_directory=True)
        self.assertEqual(9, checker.inspect(alias)['rc'])
        self.assertEqual(9, checker.inspect(alias / 'tooling/..')['rc'])

    def test_external_catalog_symlink_parent_is_invalid(self):
        other = self.root / 'linked-rules'
        other.symlink_to(self.root / 'tooling/standards', target_is_directory=True)
        result = checker.inspect(self.root, other / 'rules.json')
        self.assertEqual(9, result['rc'])

    def test_unknown_role_subdirectory_cannot_silently_drop_from_coverage(self):
        p = self.root / 'tooling/skill/software-company/agents/new-role/extra.md'
        p.parent.mkdir()
        p.write_text('# 新活角色\n')
        self.assertEqual(9, self.scan()['rc'])

    def test_unknown_top_level_skill_cannot_silently_drop_from_coverage(self):
        p = self.root / 'tooling/skill/extra-active/SKILL.md'
        p.parent.mkdir()
        p.write_text('# 新技能\n')
        self.assertEqual(9, self.scan()['rc'])

    def test_empty_cli_paths_are_invalid_before_selftest(self):
        for option in ('--root', '--rules'):
            with mock.patch.object(checker, 'run_selftest') as run:
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(9, checker.main(['--selftest', option, '']))
                run.assert_not_called()

    def test_selftest_missing_module_is_invalid(self):
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(9, checker.run_selftest(self.root))

    def test_selftest_empty_discovery_is_invalid(self):
        module = self.root / 'test_project_standards.py'
        module.write_text('# empty module\n')
        with mock.patch.object(checker.unittest.defaultTestLoader, 'discover', return_value=unittest.TestSuite()):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(9, checker.run_selftest(self.root))

    def test_bad_real_catalog_cannot_copy_outside_fixture(self):
        with tempfile.TemporaryDirectory(prefix='ros1-copy-boundary-') as directory:
            outer = Path(directory)
            source, dest = outer / 'source/tree', outer / 'fixture'
            source.mkdir(parents=True)
            dest.mkdir()
            copy_fixture(ROOT, source)
            source_probe, outside_probe = source.parent / 'probe.txt', outer / 'probe.txt'
            source_probe.write_bytes(b'source')
            outside_probe.write_bytes(b'keep')
            config = json.loads((source / checker.CATALOG).read_text())
            config['entries'][0]['path'] = '../probe.txt'
            (source / checker.CATALOG).write_text(json.dumps(config, ensure_ascii=False))
            with self.assertRaises(checker.InvalidInput):
                copy_fixture(source, dest)
            self.assertEqual(b'keep', outside_probe.read_bytes())
            self.assertEqual(b'source', source_probe.read_bytes())
            self.assertEqual([], list(dest.iterdir()))

    def test_source_symlink_prevents_all_fixture_writes(self):
        with tempfile.TemporaryDirectory(prefix='ros1-copy-source-') as directory:
            outer = Path(directory)
            source, dest = outer / 'source', outer / 'fixture'
            source.mkdir()
            dest.mkdir()
            copy_fixture(ROOT, source)
            probe = outer / 'probe.txt'
            probe.write_bytes(b'keep')
            source_readme = source / 'README.md'
            source_readme.unlink()
            source_readme.symlink_to(probe)
            with self.assertRaises(checker.InvalidInput):
                copy_fixture(source, dest)
            self.assertEqual(b'keep', probe.read_bytes())
            self.assertEqual([], list(dest.iterdir()))

    def test_destination_symlink_cannot_redirect_fixture_writes(self):
        with tempfile.TemporaryDirectory(prefix='ros1-copy-destination-') as directory:
            outer = Path(directory)
            dest, outside = outer / 'fixture', outer / 'outside'
            dest.mkdir()
            outside.mkdir()
            probe = outside / 'README.md'
            probe.write_bytes(b'keep')
            (dest / 'tooling').symlink_to(outside, target_is_directory=True)
            with self.assertRaises(checker.InvalidInput):
                copy_fixture(ROOT, dest)
            self.assertEqual(b'keep', probe.read_bytes())
            self.assertEqual(['README.md'], [p.name for p in outside.iterdir()])

    def test_inspection_is_read_only(self):
        before = {p.relative_to(self.root): p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        self.assertEqual(0, self.scan()['rc'])
        after = {p.relative_to(self.root): p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        self.assertEqual(before, after)


if __name__ == '__main__':
    unittest.main(verbosity=2)
