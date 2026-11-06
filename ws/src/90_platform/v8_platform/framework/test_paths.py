"""Root discovery keeps source/worktree/image imports free of git processes."""
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import paths


class RootPathTests(unittest.TestCase):
    def test_environment_override_precedes_markers_and_git(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {'ROS1_SLAM_ROOT': directory}):
            with patch.object(paths.subprocess, 'run') as run:
                self.assertEqual(paths.repo_root('/unknown/file.py'), Path(directory))
            run.assert_not_called()

    def test_source_and_worktree_file_markers_need_no_git_process(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ):
            os.environ.pop('ROS1_SLAM_ROOT', None)
            root = Path(directory)
            (root / 'ws/src/framework').mkdir(parents=True)
            (root / 'note').mkdir()
            (root / '.git').write_text('gitdir: /not/a/real/worktree\n')
            with patch.object(paths.subprocess, 'run') as run:
                self.assertEqual(paths.repo_root(root / 'ws/src/framework/app.py'), root)
            run.assert_not_called()

    def test_runtime_image_structure_without_git_note_or_readme_needs_no_git(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ):
            os.environ.pop('ROS1_SLAM_ROOT', None)
            root = Path(directory)
            for name in ('ws/src/framework', 'scripts/build', 'tooling/build'):
                (root / name).mkdir(parents=True)
            with patch.object(paths.subprocess, 'run') as run:
                self.assertEqual(paths.repo_root(root / 'ws/src/framework/app.py'), root)
            run.assert_not_called()
            self.assertFalse((root / '.git').exists())
            self.assertFalse((root / 'note').exists())

    def test_git_fallback_remains_available_when_explicitly_enabled(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ):
            os.environ.pop('ROS1_SLAM_ROOT', None)
            with patch.object(paths.subprocess, 'run', return_value=SimpleNamespace(returncode=0, stdout=directory + '\n')) as run:
                self.assertEqual(paths.repo_root(Path(directory) / 'tool.py'), Path(directory))
            self.assertEqual(run.call_args[0][0][:3], ['git', '-C', directory])

    def test_failed_or_unavailable_git_uses_documented_fallback(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ):
            os.environ.pop('ROS1_SLAM_ROOT', None)
            for result in (SimpleNamespace(returncode=1, stdout=''), OSError('no git')):
                with self.subTest(result=result):
                    with patch.object(paths.subprocess, 'run') as run:
                        if isinstance(result, Exception):
                            run.side_effect = result
                        else:
                            run.return_value = result
                        self.assertEqual(paths.repo_root(Path(directory) / 'tool.py'), paths.FALLBACK)

    def test_app_safe_root_discovery_never_falls_back_to_a_process(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ):
            os.environ.pop('ROS1_SLAM_ROOT', None)
            with patch.object(paths.subprocess, 'run') as run:
                self.assertEqual(paths.repo_root(Path(directory) / 'app.py', allow_git=False), paths.FALLBACK)
            run.assert_not_called()


if __name__ == '__main__':
    unittest.main()
