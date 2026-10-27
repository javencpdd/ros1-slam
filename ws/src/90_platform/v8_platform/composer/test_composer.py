"""V8-05 composer 首切片测试：结构检测四族＋正例，负例理由码逐例对账。

正例组合取自 12 份真实 manifest（faster_lio＋lite3＋costmap_layers＋scan_planner＋dwa＋safety）；
负例优先用真实组合（dwa+teb 天然重复角色），TF/命令冲突用临时目录注入。
"""
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import composer as comp  # noqa: E402

MDIR = HERE.parent / 'registry' / 'data' / 'manifests'
STACK = ['v8.odometry.faster_lio', 'v8.global_localization.lite3',
         'v8.representation.costmap_layers', 'v8.local_planner.scan_planner',
         'v8.controller.dwa_adapter', 'v8.safety_arbiter.lite3']


def reason(store, ids):
    try:
        comp.compose(store, ids)
        return 'OK', None
    except comp.ComposerError as error:
        return error.code, error.detail


class ComposerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.store = comp.load(MDIR)

    def test_positive_real_stack(self):
        result = comp.compose(self.store, STACK)
        self.assertTrue(result['ok'])
        self.assertEqual(len(result['components']), 6)
        self.assertIn('v8.local_planner.scan_planner', result['tested'])
        self.assertIn('v8.controller.dwa_adapter', result['declared_only'])
        self.assertEqual(result['command_topics']['/cmd_vel'], 'v8.safety_arbiter.lite3')
        self.assertIn('/cmd_vel_advice/dwa', result['command_topics'])

    def test_unknown_id(self):
        code, detail = reason(self.store, ['v8.no.such'])
        self.assertEqual(code, 'E_COMP_UNKNOWN_ID')

    def test_empty_selection(self):
        code, _ = reason(self.store, [])
        self.assertEqual(code, 'E_COMP_EMPTY')

    def test_duplicate_role_real(self):
        code, detail = reason(self.store, STACK + ['v8.controller.teb_adapter'])
        self.assertEqual(code, 'E_COMP_DUP_ROLE')
        self.assertIn('controller', detail)

    def test_missing_role(self):
        code, detail = reason(self.store, ['v8.controller.dwa_adapter'])
        self.assertEqual(code, 'E_COMP_MISSING_ROLE')
        self.assertIn('local_planner', detail)
        code, _ = reason(self.store, ['v8.local_planner.scan_planner',
                                      'v8.representation.costmap_layers'])
        self.assertEqual(code, 'E_COMP_MISSING_ROLE')  # representation→odometry

    def test_role_cycle_defensive(self):
        original = dict(comp.ROLE_DEPS)
        try:
            comp.ROLE_DEPS['odometry'] = {'controller'}
            comp.ROLE_DEPS['controller'] = {'odometry'}  # 闭环：odometry→controller→odometry
            code, detail = reason(self.store, STACK)
            self.assertEqual(code, 'E_COMP_TF_CYCLE')
            self.assertIn('->', detail)
        finally:
            comp.ROLE_DEPS = original

    def _mutated(self, mutate):
        """复制 12 manifest 到临时目录逐件变异，返回新 store。"""
        tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, tmp)
        for f in MDIR.glob('manifest_*.json'):
            data = json.loads(f.read_text())
            mutate(data)
            (Path(tmp) / f.name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
        return comp.load(tmp)

    def test_tf_dup_child(self):
        def mutate(d):
            if d['id'] == 'v8.representation.costmap_layers':
                d['ownership']['tf_edges'] = [{'parent': 'map', 'child': 'odom'}]
            if d['id'] == 'v8.local_planner.scan_planner':
                d['ownership']['tf_edges'] = [{'parent': 'world', 'child': 'odom'}]
        code, detail = reason(self._mutated(mutate), STACK)
        self.assertEqual(code, 'E_COMP_TF_DUP')
        self.assertIn('odom', detail)

    def test_tf_cycle(self):
        def mutate(d):
            if d['id'] == 'v8.representation.costmap_layers':
                d['ownership']['tf_edges'] = [{'parent': 'odom', 'child': 'map'}]
            if d['id'] == 'v8.local_planner.scan_planner':
                d['ownership']['tf_edges'] = [{'parent': 'map', 'child': 'odom'}]
        code, detail = reason(self._mutated(mutate), STACK)
        self.assertEqual(code, 'E_COMP_TF_CYCLE')

    def test_command_topic_dup(self):
        def mutate(d):
            if d['id'] == 'v8.controller.teb_adapter':
                d['roles'] = ['trajectory_validator']  # 避开重复角色，专测命令双写者
                d['ownership']['command_topics'] = [d['ownership']['command_topics'][0]
                                                    .replace('/cmd_vel_advice/teb', '/cmd_vel_advice/dwa')]
        code, detail = reason(self._mutated(mutate), STACK + ['v8.controller.teb_adapter'])
        self.assertEqual(code, 'E_COMP_CMD_DUP')
        self.assertIn('/cmd_vel_advice/dwa', detail)

    def test_advice_and_arbiter_distinct(self):
        """dwa 建议话题与 safety 仲裁 /cmd_vel 分层——22 号差异表的组合层落点。"""
        result = comp.compose(self.store, STACK)
        writers = set(result['command_topics'])
        self.assertIn('/cmd_vel', writers)
        self.assertIn('/cmd_vel_advice/dwa', writers)

    # -- 流程3 兼容比对 --

    def test_compat_report_positive(self):
        result = comp.compose(self.store, STACK)
        checks = {item['check'] for item in result['compatibility']}
        self.assertIn('map_supply', checks)
        self.assertIn('kinematics_pair', checks)
        supply = next(i for i in result['compatibility'] if i['check'] == 'map_supply')
        self.assertEqual(supply['component'], 'v8.controller.dwa_adapter')  # occupancy 供需
        self.assertIn('v8.representation.costmap_layers', supply['providers'])
        pair = next(i for i in result['compatibility'] if i['check'] == 'kinematics_pair')
        self.assertEqual(pair['pair'], ['quadruped_kinematic', 'differential'])  # 实栈配对
        self.assertEqual(result['adapters'], [])

    def test_map_supply_negative(self):
        """occupancy×support 供需错配：scan 改要 support_map/v1（dims 3）后无生产者。"""
        def mutate(d):
            if d['id'] == 'v8.local_planner.scan_planner':
                d['io']['map_schema'] = 'support_map/v1'
                d['io']['dimensions'] = 3
        code, detail = reason(self._mutated(mutate), STACK)
        self.assertEqual(code, 'E_COMP_MAP_SUPPLY')
        self.assertIn('support_map/v1', detail)

    def test_kinematics_not_applicable_motion_role(self):
        def mutate(d):
            if d['id'] == 'v8.controller.dwa_adapter':
                d['capability']['kinematics'] = 'not_applicable'
        code, detail = reason(self._mutated(mutate), STACK)
        self.assertEqual(code, 'E_COMP_KINEMATICS')

    def test_kinematics_pairing_negative(self):
        """holonomic 规划 × differential 控制＝错配（不在允许配对表）。"""
        def mutate(d):
            if d['id'] == 'v8.local_planner.scan_planner':
                d['capability']['kinematics'] = 'holonomic'
        code, detail = reason(self._mutated(mutate), STACK)
        self.assertEqual(code, 'E_COMP_KINEMATICS')
        self.assertIn('错配', detail)

    # -- 流程6 锁文件 --

    def test_lock_deterministic_and_annotated(self):
        import hashlib
        first = comp.lock(self.store, STACK)
        second = comp.lock(self.store, list(reversed(STACK)))  # 选择序不得影响锁
        self.assertEqual(first, second)
        noisy = comp.lock(self.store, STACK + ['v8.controller.dwa_adapter'])  # 重复选择同理
        self.assertEqual(first['digest'], noisy['digest'])
        self.assertEqual(len(first['digest']), 64)
        payload = {k: v for k, v in first.items() if k != 'digest'}
        canonical = json.dumps(payload, sort_keys=True, ensure_ascii=False, separators=(',', ':'))
        self.assertEqual(first['digest'], hashlib.sha256(canonical.encode()).hexdigest())
        manifests = first['manifests']
        self.assertTrue(manifests['v8.local_planner.scan_planner']['declared_only'] is False)
        self.assertEqual(manifests['v8.local_planner.scan_planner']['evidence'], 'contract_tested')
        self.assertTrue(manifests['v8.controller.dwa_adapter']['declared_only'] is True)
        self.assertEqual(manifests['v8.controller.dwa_adapter']['sha256'],
                         self.store.get('v8.controller.dwa_adapter')['sha256'])

    def test_cli_lock_and_reject(self):
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'lock.json'
            result = subprocess.run([sys.executable, '-B', str(HERE / 'composer.py'),
                                     str(MDIR), '--select', ','.join(STACK), '--lock', str(out)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0)
            verdict = json.loads(result.stdout)
            self.assertEqual(verdict['ok'], True)
            self.assertIn('v8.controller.dwa_adapter', verdict['declared_only'])
            saved = json.loads(out.read_text())
            self.assertEqual(saved['digest'], verdict['digest'])
            self.assertEqual(saved['lock_version'], 1)

            result = subprocess.run([sys.executable, '-B', str(HERE / 'composer.py'),
                                     str(MDIR), '--select',
                                     'v8.controller.dwa_adapter,v8.controller.teb_adapter',
                                     '--lock', str(Path(tmp) / 'no.json')],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 5)  # 拒绝报告（重复角色）
            self.assertFalse((Path(tmp) / 'no.json').exists())

    def test_cli(self):
        def run(*args):
            result = subprocess.run([sys.executable, '-B', str(HERE / 'composer.py'), *args],
                                    capture_output=True, text=True)
            return result.returncode, json.loads(result.stdout) if result.stdout.strip() else None

        rc, out = run(str(MDIR), '--select', ','.join(STACK))
        self.assertEqual(rc, 0)
        self.assertEqual(out['ok'], True)
        self.assertEqual(out['command_topics']['/cmd_vel'], 'v8.safety_arbiter.lite3')

        rc, out = run(str(MDIR), '--select', 'v8.controller.dwa_adapter')
        self.assertEqual(rc, 5)
        self.assertEqual(out['reason'], 'E_COMP_MISSING_ROLE')

        rc, _ = run(str(MDIR))
        self.assertEqual(rc, 9)


if __name__ == '__main__':
    unittest.main(verbosity=2)
