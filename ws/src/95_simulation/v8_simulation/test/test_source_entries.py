"""Offline delegation checks: no ROS, simulator or external process is started."""
import importlib.util
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
SPEC = importlib.util.spec_from_file_location('_source_entry', SCRIPTS / '_source_entry.py')
ENTRY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ENTRY)


class SourceEntryTests(unittest.TestCase):
    def test_verification_launches_preserve_legacy_configuration(self):
        platform = SCRIPTS.parents[2] / '90_platform' / 'v8_platform'

        def normalized(element):
            return (element.tag, sorted(element.attrib.items()),
                    (element.text or '').strip(),
                    [normalized(child) for child in element])

        for filename in ('verification_gazebo.launch', 'verification_mujoco.launch'):
            with self.subTest(launch=filename):
                legacy = ET.parse(platform / 'launch' / filename).getroot()
                current = ET.parse(SCRIPTS.parent / 'launch' / filename).getroot()
                for node in current.findall('node'):
                    self.assertEqual(node.get('pkg'), 'v8_simulation')
                    node.set('pkg', 'v8_platform')
                self.assertEqual(normalized(current), normalized(legacy))

    def test_source_sibling_import_and_ros_arguments_are_preserved(self):
        before = sys.path[:]
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            (root / 'fixture_sibling.py').write_text('VALUE = 42\n')
            (root / 'probe.py').write_text(
                'import sys\nimport fixture_sibling\n'
                'result = (fixture_sibling.VALUE, sys.argv[1:])\n')
            try:
                with mock.patch.object(ENTRY, '_platform_root', return_value=root), \
                        mock.patch.object(sys, 'argv', ['wrapper', '__name:=truth', '_rate:=10']):
                    result = ENTRY.run_source('probe.py')
                self.assertEqual(result['result'], (42, ['__name:=truth', '_rate:=10']))
                self.assertEqual(sys.path, before)
            finally:
                sys.modules.pop('fixture_sibling', None)

    def test_missing_source_reports_install_only_limitation(self):
        with tempfile.TemporaryDirectory() as raw:
            with mock.patch.object(ENTRY, '_platform_root', return_value=Path(raw)):
                with self.assertRaisesRegex(RuntimeError, 'source/devel workspace'):
                    ENTRY.run_source('gazebo_truth.py')

    def test_source_exit_is_propagated_and_import_path_is_restored(self):
        before = sys.path[:]
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            (root / 'probe.py').write_text('raise SystemExit(7)\n')
            with mock.patch.object(ENTRY, '_platform_root', return_value=root):
                with self.assertRaises(SystemExit) as caught:
                    ENTRY.run_source('probe.py')
            self.assertEqual(caught.exception.code, 7)
            self.assertEqual(sys.path, before)

    def test_catkin_style_execution_finds_sibling_helper(self):
        # catkin's devel relay execs the source with __file__ set to the source
        # path; it need not add that directory to sys.path for sibling imports.
        before = sys.path[:]
        entries = [
            ('gazebo_carrier3d_node.py', 'framework/gazebo_carrier3d_node.py'),
            ('gazebo_truth.py', 'gazebo_truth.py'),
            ('mujoco_truth.py', 'mujoco_truth.py')]
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            entered = root / 'entered.txt'
            fake = SimpleNamespace(RosPack=lambda: SimpleNamespace(get_path=lambda _: raw))
            with mock.patch.dict(sys.modules, {'rospkg': fake}):
                sys.modules.pop('_source_entry', None)
                for wrapper, source in entries:
                    path = SCRIPTS / wrapper
                    implementation = root / source
                    implementation.parent.mkdir(parents=True, exist_ok=True)
                    implementation.write_text(
                        'with open(%r, "a") as stream:\n    stream.write(%r)\n'
                        % (str(entered), source + '\n'))
                    try:
                        exec(compile(path.read_text(), str(path), 'exec'),
                             {'__name__': '__main__', '__file__': str(path)})
                    finally:
                        sys.path[:] = before
            self.assertEqual(entered.read_text().splitlines(), [item[1] for item in entries])


if __name__ == '__main__':
    unittest.main()
