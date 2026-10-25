import copy
import io
from pathlib import Path
import tarfile
import unittest
from unittest.mock import patch
import capture


def archive(name='copyright', text=b'License: BSD\n', kind=tarfile.REGTYPE, extra=False):
    stream = io.BytesIO()
    with tarfile.open(fileobj=stream, mode='w') as tar:
        item = tarfile.TarInfo(name)
        item.type = kind
        item.size = len(text) if kind == tarfile.REGTYPE else 0
        item.linkname = '/etc/passwd' if kind != tarfile.REGTYPE else ''
        tar.addfile(item, io.BytesIO(text) if kind == tarfile.REGTYPE else None)
        if extra:
            tar.addfile(tarfile.TarInfo('other'))
    return stream.getvalue()


class CaptureTests(unittest.TestCase):
    def test_exact_file_and_hash(self):
        row = capture.parse_file(archive(), '/usr/share/doc/pkg/copyright')
        self.assertEqual(row['text'], 'License: BSD\n')
        self.assertEqual(row['sha256'], capture.digest(b'License: BSD\n'))

    def test_wrong_or_traversal_name_rejected(self):
        for name in ('other', '../copyright', '/copyright', 'dir/copyright'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                capture.parse_file(archive(name=name), '/x/copyright')

    def test_links_and_multiple_rejected(self):
        for raw in (archive(kind=tarfile.SYMTYPE), archive(kind=tarfile.LNKTYPE), archive(extra=True)):
            with self.subTest(raw=raw[:20]), self.assertRaises(ValueError):
                capture.parse_file(raw, '/x/copyright')

    def test_empty_invalid_archive_rejected(self):
        for raw in (b'', b'bad tar'):
            with self.assertRaises(tarfile.TarError):
                capture.parse_file(raw, '/x/copyright')

    def test_identity_and_never_started(self):
        info = {'Name': '/mine', 'Config': {'Labels': {capture.LABEL: 'token'}},
                'Image': capture.IMAGE, 'State': {'Status': 'created', 'Running': False}}
        self.assertTrue(capture.owned(info, 'mine', 'token'))
        for key, val in (('Name', '/foreign'), ('Image', 'other'), ('Config', {}),
                         ('State', {'Status': 'running', 'Running': True}),
                         ('State', {'Status': 'exited', 'Running': False})):
            changed = copy.deepcopy(info)
            changed[key] = val
            self.assertFalse(capture.owned(changed, 'mine', 'token'))

    def test_package_scope(self):
        self.assertEqual(len(capture.PACKAGES), 12)
        self.assertEqual(len(set(capture.PACKAGES)), 12)

    def test_copy_failure_not_empty_success(self):
        result = type('Result', (), {'returncode': 1, 'stderr': b'not found', 'stdout': b''})()
        with patch.object(capture, 'run', return_value=result):
            row = capture.copy_file('container', '/x/copyright')
        self.assertEqual(row['rc'], 1)
        self.assertNotIn('text', row)
        self.assertNotIn('sha256', row)

    def test_no_start_exec_or_mount_commands(self):
        text = Path(capture.__file__).read_text()
        for command in ("'start'", "'exec'", "'stop'", "'--mount'", "'--privileged'"):
            self.assertNotIn(command, text)


if __name__ == '__main__':
    unittest.main()
