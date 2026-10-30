#!/usr/bin/env python3
"""Framework CLI (ROS1 v0): check a YAML graph and emit plan/launch skeletons.

Usage:
  framework.py check MANIFEST_DIR GRAPH
  framework.py plan MANIFEST_DIR GRAPH --out-plan PLAN.json [--out-launch FILE.launch]
"""
import argparse
import json
import sys
from pathlib import Path

from config import FrameworkError, load_components, load_graph
import graph as graph_mod
import launch as launch_mod
import render


def _run(args):
    catalog = load_components(args.manifests)
    launch_summary = None
    if args.package_index:
        launch_summary = launch_mod.check_components(
            catalog, launch_mod.load_index(args.package_index))
    graph = load_graph(args.graph)
    report = graph_mod.check(graph, catalog)
    if args.command == 'check':
        summary = {key: report[key] for key in
                   ('ok', 'name', 'digest', 'startup_order', 'feedback',
                    'command_writers')}
        if launch_summary is not None:
            summary['launch'] = launch_summary
        print(json.dumps(summary, ensure_ascii=False, indent=1))
        return 0
    plan = render.render_plan(graph, catalog, report)
    Path(args.out_plan).write_text(json.dumps(plan, ensure_ascii=False, indent=1) + '\n',
                                   encoding='utf-8')
    launch = render.render_launch(graph, catalog, report)
    Path(args.out_launch).write_text(launch, encoding='utf-8')
    result = {'ok': True, 'digest': report['digest'],
              'plan': args.out_plan, 'launch': args.out_launch}
    if launch_summary is not None:
        result['launch_check'] = launch_summary
    print(json.dumps(result, ensure_ascii=False))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(prog='framework.py')
    sub = parser.add_subparsers(dest='command', required=True)
    check = sub.add_parser('check')
    check.add_argument('manifests')
    check.add_argument('graph')
    check.add_argument('--package-index')
    plan = sub.add_parser('plan')
    plan.add_argument('manifests')
    plan.add_argument('graph')
    plan.add_argument('--package-index')
    plan.add_argument('--out-plan', required=True)
    plan.add_argument('--out-launch', required=True)
    try:
        args = parser.parse_args(argv)
    except SystemExit:
        return 9
    try:
        return _run(args)
    except FrameworkError as error:
        print(json.dumps({'ok': False, 'reason': error.code, 'detail': error.detail},
                         ensure_ascii=False))
        return 5
    except OSError as error:
        print(json.dumps({'ok': False, 'reason': 'E_IO', 'detail': str(error)},
                         ensure_ascii=False))
        return 5


if __name__ == '__main__':
    sys.exit(main())
