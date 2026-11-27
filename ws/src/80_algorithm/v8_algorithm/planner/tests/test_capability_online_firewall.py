#!/usr/bin/env python3
"""CAP-03/04/05 在线接入防火墙（静态）：离线能力/路线模块不得被在线规划/节点导入。"""
import unittest
from pathlib import Path

V8 = Path(__file__).resolve().parents[2]
SRC = V8.parent.parent
FORBIDDEN = ('offline_capability_cost', 'route_contract',
             'query_capabilities', 'authorize_capabilities')

ONLINE = [
    V8 / 'local_planner' / 'local_native.py',
    V8 / 'planner' / 'support_planner.py',
]
NODES = SRC / '90_platform' / 'v8_platform' / 'framework' / 'nodes'


class OnlineCapabilityFirewallTest(unittest.TestCase):
    def test_online_modules_do_not_import_offline_capability(self):
        files = list(ONLINE) + sorted(NODES.glob('*.py'))
        self.assertTrue(files)
        for path in files:
            text = path.read_text(encoding='utf-8', errors='replace')
            found = [token for token in FORBIDDEN if token in text]
            self.assertEqual(found, [], '%s 出现在线能力接入标记 %s' % (path, found))

    def test_offline_module_really_carries_the_query_token(self):
        text = (V8 / 'planner' / 'offline_capability_cost.py').read_text()
        self.assertIn('query_capabilities', text)


if __name__ == '__main__':
    unittest.main()
