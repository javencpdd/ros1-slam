#!/usr/bin/env python3
import hashlib
import os
from pathlib import Path
import shlex
import stat
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
LEGACY_SHA = '6b39e0d8a5819963ccf2238fb2170d349abadaf2a2a9c8b45a4cc102b70c4dde'
CMAKE_SHA = '64a1cd98637364e4690d5b6f3eb20c7e328b29711cb55782a0aa2dcfbb8c698c'
# 2026-11-21（真人勾选⑥）：costmap 插件包移出 40_navigation、移入 30_perception；
# 政策层恢复「零源码」，插件契约改在新层校验（包名/导出/CMake 快照不变）。
PACKAGES = {'lite3_navigation', 'lite3_scan'}
SOURCES = set()
SUFFIXES = {'.c', '.cc', '.cpp', '.cxx', '.c++', '.CPP', '.h', '.hh', '.hpp', '.hxx', '.C', '.H'}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def check_navigation(root):
    root = Path(root).resolve(strict=True)
    nav = root / 'ws/src/40_navigation'
    if not nav.is_dir() or nav.resolve() != nav:
        raise ValueError('navigation root missing or symlinked')
    found_sources, found_packages = set(), set()
    for parent, dirs, files in os.walk(nav, onerror=lambda error: (_ for _ in ()).throw(error)):
        for name in dirs + files:
            p = Path(parent) / name
            mode = p.lstat().st_mode
            if not (stat.S_ISDIR(mode) or stat.S_ISREG(mode)):
                raise ValueError('nonregular navigation entry: ' + str(p))
            if p.suffix in SUFFIXES:
                if not stat.S_ISREG(mode) or p.stat().st_size == 0:
                    raise ValueError('empty or nonregular source: ' + str(p))
                found_sources.add(p.relative_to(nav).as_posix())
            if name == 'package.xml':
                package = ET.parse(p).getroot()
                expected = p.parent.name
                names = package.findall('name')
                if (p.parent.parent != nav or expected not in PACKAGES or package.tag != 'package'
                        or len(names) != 1 or names[0].text != expected):
                    raise ValueError('unapproved package identity: ' + str(p))
                found_packages.add(expected)
    if found_packages != PACKAGES:
        raise ValueError('package set mismatch: ' + repr(sorted(found_packages)))
    if found_sources != SOURCES:
        raise ValueError('source set mismatch: ' + repr(sorted(found_sources)))
    plugin = root / 'ws/src/30_perception/lite3_costmap_layers'
    if not plugin.is_dir() or plugin.is_symlink():
        raise ValueError('moved costmap plugin package missing')
    # CMake is executable: pin the inspected declaration instead of accepting a regex match in dead code.
    if sha((plugin / 'CMakeLists.txt').read_bytes()) != CMAKE_SHA:
        raise ValueError('plugin CMake declaration differs from inspected snapshot')
    package = ET.parse(plugin / 'package.xml').getroot()
    if package.tag != 'package' or package.findtext('name') != plugin.name:
        raise ValueError('plugin package identity mismatch')
    exports = package.findall('export')
    if (len(exports) != 1 or len(exports[0]) != 1 or exports[0][0].tag != 'costmap_2d'
            or exports[0][0].attrib != {'plugin': '${prefix}/plugins.xml'}):
        raise ValueError('plugin export mismatch')
    library = ET.parse(plugin / 'plugins.xml').getroot()
    if (library.tag != 'library' or library.attrib != {'path': 'lib/liblite3_costmap_layers'}
            or len(library) != 1 or library[0].tag != 'class'
            or library[0].attrib != {'type': 'lite3_costmap_layers::ElevationMergedLayer',
                                     'base_class_type': 'costmap_2d::Layer'}
            or any(child.tag != 'description' for child in library[0])):
        raise ValueError('plugin class or library mismatch')
    print('[9-v8] approved packages=2 sources=0; plugin moved to 30_perception', flush=True)


def derive_legacy(raw):
    if sha(raw) != LEGACY_SHA:
        raise ValueError('legacy ISO hash mismatch; refusing a different base')
    text = raw.decode('utf-8')
    begin = text.index('say "[9] ')
    end = text.index('say "[10] ', begin)
    replacement = ('say "[9] v8 shell/source/plugin boundary"\n'
                   + 'if [ "$(readlink -f /home/ros1-slam/ws/src/40_navigation)" != '
                   + '"/opt/ros1-slam/ws/src/40_navigation" ]; then\n'
                   + '  echo "[9-v8] unexpected x86 navigation alias"; FAIL=1\nelse\n'
                   + shlex.quote(sys.executable) + ' -B ' + shlex.quote(str(HERE / 'v8_isolation.py'))
                   + ' --navigation /opt/ros1-slam || FAIL=1\nfi\n')
    return text[:begin] + replacement + text[end:]


def main(args):
    try:
        if args[:1] == ['--navigation'] and len(args) == 2:
            check_navigation(args[1])
            return 0
        if args:
            print('usage: v8_isolation.sh (full check); tests: python3 -B test_v8_isolation.py')
            return 9
        raw = (HERE / 'v3_isolation.sh').read_bytes()
        derived = derive_legacy(raw)
        env = os.environ.copy()
        env.setdefault('ISO_ROOT', str(HERE.parent.parent))
        with tempfile.TemporaryDirectory(prefix='v8_iso_') as temporary:
            script = Path(temporary) / 'isolation.sh'
            script.write_text(derived)
            print('v8-isolation legacy_sha=' + sha(raw) + ' derived_sha=' + sha(derived.encode())
                  + ' checker_sha=' + sha(Path(__file__).read_bytes()), flush=True)
            return subprocess.run(['bash', str(script)], env=env).returncode
    except (OSError, ValueError, ET.ParseError) as error:
        print('[v8-isolation] rejected: ' + str(error), file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
