#!/usr/bin/env python3
"""Canonical world registry for the console testbed.

The registry is the single source of truth for a Gazebo world's file path,
SHA256, scene id, map bindings and stack bindings.  Generated artifacts
(worlds.json, maps.json) and stacks.json are checked against it; nothing here
starts ROS or Gazebo.

Usage:
  python3 scripts/ops/world_registry.py --check
  python3 scripts/ops/world_registry.py --update-shas
"""
from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = Path(os.environ.get('V8_SIM_ROOT') or (ROOT.parent / 'ros1-slam-simulation')).resolve()
REGISTRY = Path(__file__).resolve().parent.parent / 'console/world_registry.json'


def load_registry(path=None):
    return json.loads(Path(path or REGISTRY).read_text(encoding='utf-8'))


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def by_id(registry):
    return {item['id']: item for item in registry.get('worlds', [])}


def map_bindings(registry):
    bindings = {}
    for world in registry.get('worlds', []):
        for item in world.get('maps', []):
            bindings[item['id']] = {'world_id': world['id'], 'label': item['label']}
    return bindings


def stack_bindings(registry):
    bindings = {}
    for world in registry.get('worlds', []):
        for item in world.get('stacks', []):
            bindings[(item['stack'], item['file'])] = world['id']
    return bindings


def check_registry(registry, root=None):
    root = Path(root or SIM_ROOT)
    errors = []
    if registry.get('schema') != 'v8-world-registry/v1':
        errors.append('schema must be v8-world-registry/v1')
    worlds = registry.get('worlds')
    if not isinstance(worlds, list) or not worlds:
        return errors + ['worlds must be a non-empty list']
    ids = set()
    paths = set()
    map_ids = set()
    for world in worlds:
        wid = world.get('id')
        if not wid:
            errors.append('world without id')
            continue
        if wid in ids:
            errors.append('duplicate world id: ' + wid)
        ids.add(wid)
        if not world.get('label'):
            errors.append('%s: missing label' % wid)
        rel = world.get('path')
        if not rel:
            errors.append('%s: missing path' % wid)
            continue
        if rel in paths:
            errors.append('duplicate world path: ' + rel)
        paths.add(rel)
        path = root / rel
        if not path.exists():
            errors.append('%s: path missing: %s' % (wid, rel))
            continue
        actual = sha256_file(path)
        if world.get('sha256') != actual:
            errors.append('%s: sha256 mismatch (registry %s, actual %s)' % (
                wid, str(world.get('sha256'))[:12], actual[:12]))
        if world.get('scene') != wid:
            errors.append('%s: scene must equal id' % wid)
        for item in world.get('maps', []):
            mid = item.get('id')
            if not mid or not item.get('label'):
                errors.append('%s: malformed map binding' % wid)
                continue
            if mid in map_ids:
                errors.append('map %s bound to multiple worlds' % mid)
            map_ids.add(mid)
        for item in world.get('stacks', []):
            if not item.get('stack') or not item.get('file'):
                errors.append('%s: malformed stack binding' % wid)
    return errors


def update_shas(registry, root=None):
    root = Path(root or SIM_ROOT)
    for world in registry.get('worlds', []):
        path = root / world['path']
        world['sha256'] = sha256_file(path)
    registry['updated'] = datetime.datetime.now().astimezone().isoformat(timespec='seconds')
    REGISTRY.write_text(json.dumps(registry, ensure_ascii=False, indent=2) + '\n',
                        encoding='utf-8')
    return registry


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--registry', default=str(REGISTRY))
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--update-shas', action='store_true')
    args = parser.parse_args()
    registry = load_registry(args.registry)
    if args.update_shas:
        registry = update_shas(registry)
        print('updated %d world SHA256 entries' % len(registry.get('worlds', [])))
        return
    errors = check_registry(registry)
    if errors:
        for error in errors:
            print('FAIL  ' + error)
        raise SystemExit(1)
    print('PASS  world registry: %d worlds, %d maps, %d stack bindings' % (
        len(registry['worlds']),
        len(map_bindings(registry)),
        len(stack_bindings(registry))))


if __name__ == '__main__':
    main()
