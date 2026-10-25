#!/usr/bin/env python3
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import v8_isolation as iso


class IsolationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='v8_iso_test_')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.nav = self.root / 'ws/src/40_navigation'
        shutil.copytree(iso.HERE.parent.parent / 'ws/src/40_navigation', self.nav)
        self.plugin = self.nav / 'lite3_costmap_layers'

    def check(self):
        iso.check_navigation(self.root)

    def reject(self):
        with self.assertRaises((ValueError, OSError, iso.ET.ParseError)):
            self.check()

    def test_current_snapshot(self):
        self.check()

    def test_shell_injection_and_recovery(self):
        p = self.nav / 'lite3_navigation/injected.cpp'
        p.write_text('int x;')
        self.reject()
        p.unlink()
        self.check()

    def test_extra_source_suffixes(self):
        for suffix in sorted(iso.SUFFIXES):
            with self.subTest(suffix=suffix):
                p = self.plugin / ('extra' + suffix)
                p.write_text('int x;')
                self.reject()
                p.unlink()

    def test_unapproved_package(self):
        p = self.nav / 'extra'
        p.mkdir()
        (p / 'package.xml').write_text('<package><name>extra</name></package>')
        self.reject()

    def test_missing_source(self):
        (self.plugin / 'src/elevation_merged_layer.cpp').unlink()
        self.reject()

    def test_empty_source(self):
        (self.plugin / 'src/elevation_merged_layer.cpp').write_text('')
        self.reject()

    def test_missing_root(self):
        shutil.rmtree(self.nav)
        self.reject()

    def test_package_rename(self):
        p = self.plugin / 'package.xml'
        p.write_text(p.read_text().replace('<name>lite3_costmap_layers</name>', '<name>other</name>'))
        self.reject()

    def test_comment_not_cmake_declaration(self):
        p = self.plugin / 'CMakeLists.txt'
        p.write_text(p.read_text().replace('add_library(', '# add_library('))
        self.reject()

    def test_dead_cmake_declaration(self):
        p = self.plugin / 'CMakeLists.txt'
        p.write_text('if(FALSE)\n' + p.read_text() + '\nendif()\n')
        self.reject()

    def test_missing_export(self):
        p = self.plugin / 'package.xml'
        p.write_text(p.read_text().replace('<costmap_2d plugin="${prefix}/plugins.xml"/>', ''))
        self.reject()

    def test_plugin_wrong_class(self):
        p = self.plugin / 'plugins.xml'
        p.write_text(p.read_text().replace('ElevationMergedLayer', 'OtherLayer'))
        self.reject()

    def test_plugin_extra_class(self):
        p = self.plugin / 'plugins.xml'
        p.write_text(p.read_text().replace('</library>', '<class type="Other"/></library>'))
        self.reject()

    def test_malformed_xml(self):
        (self.plugin / 'plugins.xml').write_text('<library>')
        self.reject()

    def test_source_symlink(self):
        p = self.plugin / 'src/elevation_merged_layer.cpp'
        p.unlink()
        p.symlink_to(iso.HERE / 'v8_isolation.py')
        self.reject()

    def test_directory_symlink(self):
        (self.nav / 'hidden').symlink_to(iso.HERE, target_is_directory=True)
        self.reject()

    def test_root_symlink(self):
        dest = self.root / 'outside'
        self.nav.rename(dest)
        self.nav.symlink_to(dest, target_is_directory=True)
        self.reject()

    def test_legacy_mutation_rejected(self):
        raw = (iso.HERE / 'v3_isolation.sh').read_bytes()
        with self.assertRaises(ValueError):
            iso.derive_legacy(raw + b'\n')

    def test_only_section_nine_changes(self):
        raw = (iso.HERE / 'v3_isolation.sh').read_bytes()
        old = raw.decode()
        new = iso.derive_legacy(raw)
        self.assertEqual(old[:old.index('say "[9] ')], new[:new.index('say "[9] ')])
        self.assertEqual(old[old.index('say "[10] '):], new[new.index('say "[10] '):])

    def full_fixture(self, extra_env=None, wrong_alias=False):
        home = self.root / 'home'
        home.mkdir(exist_ok=True)
        ws_alias = home / 'ws'
        if ws_alias.is_symlink():
            ws_alias.unlink()
        ws_alias.symlink_to(self.root / ('other_ws' if wrong_alias else 'ws'), target_is_directory=True)
        bindir = self.root / 'bin'
        bindir.mkdir(exist_ok=True)
        rospack = bindir / 'rospack'
        rospack.write_text('#!/bin/bash\nif [ "$1" = list ]; then printf "pkg %s\\n" "' + str(home)
                           + '"; else printf "%s\\n" "' + str(home) + '"; fi\n')
        rospack.chmod(0o700)
        df = bindir / 'df'
        df.write_text('#!/bin/bash\nprintf "Filesystem 1G-blocks Used Available Use%% Mounted\\nfixture 100G 1G 99G 1%% /\\n"\n')
        df.chmod(0o700)
        (self.root / 'iso_fake.py').write_text('')
        (home / '.bashrc').write_text('')
        script = self.root / 'derived.sh'
        script.write_text(iso.derive_legacy((iso.HERE / 'v3_isolation.sh').read_bytes())
                          .replace('/home/ros1-slam', str(home))
                          .replace('/opt/ros1-slam', str(self.root)))
        env = dict(os.environ, PATH=str(bindir) + ':' + os.environ['PATH'],
                   ISO_ROOT=str(self.root), ISO_ALLOW=str(self.root) + ':/usr:/opt/ros/noetic',
                   ROS_PACKAGE_PATH=str(self.root), PYTHONPATH=str(self.root),
                   LD_LIBRARY_PATH='', CMAKE_PREFIX_PATH='', ISO10_MODULES='iso_fake')
        env.update(extra_env or {})
        return subprocess.run(['bash', str(script)], env=env, capture_output=True, text=True, timeout=30)

    def test_full_derived_baseline_and_environment_negative(self):
        baseline = self.full_fixture()
        self.assertEqual(baseline.returncode, 0, baseline.stdout + baseline.stderr)
        bad = self.full_fixture({'PYTHONPATH': str(self.root) + ':/outside-v8-test'})
        self.assertEqual(bad.returncode, 1, bad.stdout + bad.stderr)
        self.assertIn('首条越界=/outside-v8-test', bad.stdout)
        self.assertIn('[9-v8] approved', bad.stdout)
        restored = self.full_fixture()
        self.assertEqual(restored.returncode, 0, restored.stdout + restored.stderr)

    def test_full_derived_alias_negative_and_recovery(self):
        baseline = self.full_fixture()
        self.assertEqual(baseline.returncode, 0, baseline.stdout + baseline.stderr)
        shutil.copytree(self.root / 'ws', self.root / 'other_ws')
        bad = self.full_fixture(wrong_alias=True)
        self.assertEqual(bad.returncode, 1, bad.stdout + bad.stderr)
        self.assertIn('[9-v8] unexpected x86 navigation alias', bad.stdout)
        self.assertNotIn('[9-v8] approved', bad.stdout)
        restored = self.full_fixture()
        self.assertEqual(restored.returncode, 0, restored.stdout + restored.stderr)

    def test_full_derived_source_negative(self):
        baseline = self.full_fixture()
        self.assertEqual(baseline.returncode, 0, baseline.stdout + baseline.stderr)
        (self.nav / 'lite3_scan/injected.cpp').write_text('int x;')
        bad = self.full_fixture()
        self.assertEqual(bad.returncode, 1)
        self.assertIn('source set mismatch', bad.stderr)

    def test_entrypoint_fixed_target_and_exit_propagation(self):
        verify = self.root / 'scripts/verify'
        verify.mkdir(parents=True)
        for name in ('v8_window_preflight.sh', 'v_window_preflight_x86_variant.sh', '_iso_alt_wrapper.sh'):
            shutil.copy2(iso.HERE / name, verify / name)
        (verify / 'v_window_preflight.sh').write_text('bash "$VPF_ISO"\n')
        (verify / 'v8_isolation.sh').write_text('printf "v8-target\\n"; exit 5\n')
        p = subprocess.run(['bash', str(verify / 'v8_window_preflight.sh')],
                           env=dict(os.environ, ISO_TARGET='/not-authorized', VPF_ISO='/not-authorized'),
                           capture_output=True, text=True, timeout=10)
        self.assertEqual(p.returncode, 5, p.stdout + p.stderr)
        self.assertIn('v8-target', p.stdout)

    def test_preflight_argument_cannot_skip_full_check(self):
        p = subprocess.run(['bash', str(iso.HERE / 'v8_window_preflight.sh'), '--selftest-hg'],
                           capture_output=True, timeout=10)
        self.assertEqual(p.returncode, 9)

    def test_independent_cmake_suffix_cases(self):
        for suffix in ('.c++', '.CPP'):
            p = self.nav / ('lite3_navigation/injected' + suffix)
            p.write_text('int x;')
            self.reject()
            p.unlink()
        self.check()

    def test_public_iso_rejects_partial_check(self):
        p = subprocess.run(['bash', str(iso.HERE / 'v8_isolation.sh'), '--navigation', str(self.root)],
                           env=dict(os.environ, PYTHONPATH='/outside-v8-test'),
                           capture_output=True, timeout=10)
        self.assertEqual(p.returncode, 9)

    def test_full_main_lifecycle_and_rc(self):
        from unittest.mock import patch
        scripts = []
        def execute(argv, env):
            self.assertEqual(argv[0], 'bash')
            script = Path(argv[1])
            scripts.append(script)
            self.assertEqual(script.read_text(), iso.derive_legacy((iso.HERE / 'v3_isolation.sh').read_bytes()))
            self.assertEqual(env['ISO_ROOT'], str(iso.HERE.parent.parent))
            return subprocess.CompletedProcess(argv, 5)
        with patch.dict(os.environ, {}, clear=True), patch.object(iso.subprocess, 'run', side_effect=execute):
            self.assertEqual(iso.main([]), 5)
        self.assertEqual(len(scripts), 1)
        self.assertFalse(scripts[0].exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)
