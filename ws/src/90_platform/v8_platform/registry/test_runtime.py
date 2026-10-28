"""Exercise discovery through a separate live process and network requests."""
import json
import subprocess
import sys
import unittest
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

HERE = Path(__file__).resolve().parent


class RuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.proc = subprocess.Popen([sys.executable, '-B', str(HERE / 'runtime.py')],
                                    stdout=subprocess.PIPE, text=True)
        ready = json.loads(cls.proc.stdout.readline())
        cls.base = 'http://127.0.0.1:%d' % ready['port']

    @classmethod
    def tearDownClass(cls):
        cls.proc.terminate()
        cls.proc.wait(timeout=5)
        cls.proc.stdout.close()

    def request(self, path, value=None):
        data = None if value is None else json.dumps(value).encode()
        request = Request(self.base + path, data=data,
                          headers={'Content-Type': 'application/json'})
        try:
            with urlopen(request, timeout=5) as response:
                return response.status, json.load(response)
        except HTTPError as error:
            with error:
                return error.code, json.load(error)

    def test_query_tags_roles_and_evidence(self):
        status, result = self.request('/components?tag=indoor&role=controller')
        self.assertEqual(status, 200)
        self.assertEqual({e['id'] for e in result['components']},
                         {'v8.controller.dwa_adapter', 'v8.controller.teb_adapter'})
        self.assertTrue(all(e['evidence']['claimed_only'] for e in result['components']))
        self.assertEqual(len(result['registry_digest']), 64)

    def test_incompatibility_is_explained(self):
        status, result = self.request('/compose', {'components': [
            'v8.controller.dwa_adapter', 'v8.controller.teb_adapter']})
        self.assertEqual(status, 409)
        self.assertEqual(result['reason'], 'E_COMP_DUP_ROLE')
        self.assertIn('controller', result['detail'])

    def test_valid_composition_keeps_declared_evidence(self):
        status, result = self.request('/compose', {'components': ['v8.map_server.apt']})
        self.assertEqual(status, 200)
        self.assertTrue(result['ok'])
        self.assertEqual(result['declared_only'], ['v8.map_server.apt'])

    def test_unknown_and_invalid_requests(self):
        self.assertEqual(self.request('/components?id=no.such')[1]['count'], 0)
        self.assertEqual(self.request('/compose', {'components': ['no.such']})[1]
                         ['reason'], 'E_COMP_UNKNOWN_ID')
        self.assertEqual(self.request('/compose', {'components': 'not-a-list'})[0], 400)
        self.assertEqual(self.request('/components?evidence=tested')[0], 400)


if __name__ == '__main__':
    unittest.main()
