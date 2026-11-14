#!/usr/bin/env python3
"""Static checks for the BT configuration profiles.

No ROS, no third-party packages: the profiles are flat `key: value` files, so a
tiny parser is enough and the check runs on any host python.

What it enforces:
1. every key the C++ node reads (pnh.param) is supplied either by the profile
   file or by a launch <param> -- no silent fallback to a default;
2. demo and real profiles expose the same key set (so a real run cannot be
   missing a wiring key);
3. the real profile carries no numeric physical value (physics needs a work
   order, note/10:4), queue_max being the only structural exception;
4. the demo profile file referenced by the launch actually exists.
"""
from __future__ import annotations

import os
import re
import sys

ROOT = os.environ.get('V8_REPO_ROOT') or os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', 'ros1-slam'))
PKG = os.path.join(ROOT, 'ws/src/90_platform/lite3_bt')
CPP = os.path.join(PKG, 'src/bt_console.cpp')
LAUNCH = os.path.join(PKG, 'launch/bt_console.launch')
DEMO = os.path.join(PKG, 'config/bt_console_demo.yaml')
REAL = os.path.join(PKG, 'config/bt_console_real.yaml.template')

KEY_RE = re.compile(r'pnh\.param<[^>]+>\("([A-Za-z0-9_]+)"')
FLAT_RE = re.compile(r'^([A-Za-z0-9_]+):\s*(.*)$')
STRUCTURAL = {'queue_max', 'cancel_ack_timeout', 'retry_max', 'retry_delay_s'}

failures = []


def check(cond, what):
    if not cond:
        failures.append(what)
        print('FAIL  ' + what)
    else:
        print('PASS  ' + what)


def read(path):
    with open(path, 'r', encoding='utf-8') as fh:
        return fh.read()


def flat_keys(text):
    keys, values = [], {}
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith('#'):
            continue
        match = FLAT_RE.match(stripped)
        if match:
            keys.append(match.group(1))
            values[match.group(1)] = match.group(2).strip()
    return keys, values


def launch_keys(text):
    start = text.find('type="bt_console"')
    end = text.find('</node>', start)
    block = text[start:end] if start >= 0 and end > start else ''
    keys = re.findall(r'<param\s+name="([A-Za-z0-9_]+)"', block)
    return keys, block


def main():
    if not os.path.isdir(PKG):
        print('SKIP  P1 lite3_bt not found at %s' % PKG)
        return 0
    code = set(KEY_RE.findall(read(CPP)))
    demo_keys, demo_values = flat_keys(read(DEMO))
    real_keys, real_values = flat_keys(read(REAL))
    l_keys, block = launch_keys(read(LAUNCH))
    launch_set = set(l_keys)

    check(len(code) >= 8, 'code reads %d param keys' % len(code))
    check(set(demo_keys) <= code, 'demo profile only uses keys the code reads')
    check(launch_set <= code, 'launch params only use keys the code reads')
    missing = code - (set(demo_keys) | launch_set)
    check(not missing, 'every code key is supplied by profile or launch: %s'
          % (sorted(missing) if missing else 'none'))
    check(set(demo_keys) == set(real_keys),
          'demo and real profiles expose the same key set')
    check(len(set(demo_keys)) == len(demo_keys), 'demo profile has no duplicate keys')

    numeric = []
    for key, value in real_values.items():
        if key in STRUCTURAL:
            continue
        if re.fullmatch(r'-?\d+(\.\d+)?', value):
            numeric.append(key)
    check(not numeric, 'real profile carries no numeric physical value: %s'
          % (numeric if numeric else 'none'))

    check('$(arg profile)' in block, 'launch loads the profile file')
    check(os.path.isfile(DEMO), 'demo profile file exists')
    check(not os.path.isfile(REAL[:-len('.template')]),
          'no bt_console_real.yaml yet (real profile must be copied on purpose)')

    if failures:
        print('FAILED (%d)' % len(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
