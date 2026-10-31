#!/usr/bin/env python3
"""Preload ROS parameters, then exec the component command.

roslaunch cannot resolve nested node paths such as ``local_planner/ros_node.py``
and rosrun does not load YAML parameter files. This shim sets the target node's
private parameters directly on the master, then execs the real rosrun command so
the component still owns its process group.
"""
import json
import os
import subprocess
import sys

import rosgraph
import yaml


def _merge(path, params):
    with open(path, encoding='utf-8') as stream:
        value = yaml.safe_load(stream)
    if value is None:
        return
    if not isinstance(value, dict):
        raise ValueError('%s: parameter file must contain a mapping' % path)
    params.update(value)


def main(argv):
    if len(argv) < 2:
        raise SystemExit('usage: param_exec.py <spec.json>')
    with open(argv[1], encoding='utf-8') as stream:
        spec = json.load(stream)
    params = {}
    if spec.get('config_path'):
        _merge(spec['config_path'], params)
    for item in spec.get('param_files', []):
        package, _, relative = item.partition(':')
        root = subprocess.check_output(['rospack', 'find', package],
                                       text=True).strip()
        _merge(os.path.join(root, relative), params)
    params.update(spec.get('params', {}))
    namespace = (spec.get('namespace') or '').strip('/')
    base = '/' + (namespace + '/' if namespace else '') + spec['name']
    master = rosgraph.Master('/v8_param_exec')
    for key in sorted(params):
        master.setParam(base + '/' + key, params[key])
    command = list(spec['argv'])
    os.execvp(command[0], command)


if __name__ == '__main__':
    main(sys.argv)
