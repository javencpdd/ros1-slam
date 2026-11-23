#!/usr/bin/env python3
"""统一的开发入口：把「四个入口各有环境前提」收成一处。

此前要干一件事，得先知道去哪、以什么姿势：
    构建   bash docker/build.sh [--plan|--clean]        （必须在容器内、且已 source ROS）
    单测   cd <包目录> && python3 -m unittest test_xxx  （sys.path 依赖 cwd）
    门禁   GATE_ONLY=SYN,PYST bash scripts/verify/run_gate_chain.sh（只报不判，rc 恒 0）
    冒烟   docker compose exec sim bash /opt/ros1-slam/docker/sim.sh smoke stageA
本脚本把这些收成子命令，并按当前所处环境自动切换「容器内直跑」与「宿主调 docker exec」。

子命令：
    status              环境自检（在哪、容器在不在、ROS/devel 就绪否）
    build [--plan|--clean]
    test [路径...]       不给路径则扫 v8_platform 下全部 test_*.py
    gate [成员,...]      跑门禁串并**汇总退出码**（见下）
    smoke [stage]        仿真冒烟，默认 stageA

退出码约定：0 成功｜1 有失败项｜2 用法错误｜9 环境前置缺失。
★gate 的判定语义★：run_gate_chain.sh 本身只报不判（件内明写 exit 0，这是设计不是缺陷）。
本脚本解析它输出里的「rc=N」，任一成员非零即整体返回 1——CI 要的是这个，而不是恒 0。
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
CONT_DIR = '/opt/ros1-slam'
ROS_SETUP = '/opt/ros/noetic/setup.bash'
DEVEL_SETUP = f'{CONT_DIR}/ws/devel/setup.bash'
COMPOSE = ['docker', 'compose', '-f', str(REPO / 'docker/docker-compose.yml'),
           '--env-file', str(REPO / 'docker/.env')]
GATE_MARK = re.compile(r'\brc=(\d+)\b')


def in_container() -> bool:
    """判据落在这两层同时存在：容器把仓挂到 /opt/ros1-slam，且带 ROS 发行版。"""
    return Path(CONT_DIR, 'ws').is_dir() and Path(ROS_SETUP).is_file()


def say(msg: str = '') -> None:
    print(msg, flush=True)


def fail(msg: str, code: int = 9) -> int:
    print(f'[dev][FATAL] {msg}', file=sys.stderr)
    return code


def compose_running(service: str) -> bool:
    if shutil.which('docker') is None:
        return False
    out = subprocess.run([*COMPOSE, 'ps', '--services', '--status', 'running'],
                         capture_output=True, text=True)
    return service in out.stdout.split()


def exec_in(service: str, argv: list[str]) -> int:
    """在指定服务容器里跑命令；容器没起就先起（sim-viz 走 viz profile）。"""
    if not compose_running(service):
        profile = 'viz' if service == 'sim-viz' else 'sim'
        say(f'[dev] {service} 未运行，按 {profile} profile 拉起…')
        up = subprocess.run([*COMPOSE, '--profile', profile, 'up', '-d', service])
        if up.returncode != 0:
            return fail(f'拉起 {service} 失败（先看 docker compose ps / 日志）')
    cmd = [*COMPOSE, 'exec', '-T', '-w', CONT_DIR, service, 'bash', '-lc', ' && '.join(argv)]
    return subprocess.run(cmd).returncode


def with_env(inner: str) -> str:
    """容器内的统一起手式：双 source + PYTHONNOUSERSITE + git 信任根。

    git 那句是账构门禁必需的：.git 以只读方式挂进容器，属主是宿主用户而进程是 root，
    git 会以「dubious ownership」拒绝读史 ⇒ 件按前置缺失 rc=9 拒判。
    ★ 不能用 GIT_CONFIG_COUNT/KEY/VALUE 那套 ★ 容器里 git 是 2.25.1，该机制要 2.35+，
    实测设了也照样报 dubious ownership；只能用 `git config --global --add safe.directory`。
    写在容器内 /root/.gitconfig，容器重建即丢——每次进容器补一遍，比改镜像轻。
    """
    return (f'source {ROS_SETUP} && '
            f'[ -f {DEVEL_SETUP} ] && source {DEVEL_SETUP} || true; '
            f'export PYTHONNOUSERSITE=1 GIT_OPTIONAL_LOCKS=0; '
            f'git config --global --add safe.directory {CONT_DIR} 2>/dev/null || true; '
            f'{inner}')


# --------------------------------------------------------------------------- 子命令

def cmd_status(_args) -> int:
    where = '容器内' if in_container() else '宿主机'
    say(f'环境      : {where}')
    say(f'仓根      : {REPO}')
    if shutil.which('docker') is None:
        say('docker    : 不可用（宿主机上 build/gate/smoke 都跑不了）')
    else:
        for svc in ('roscore', 'sim', 'sim-viz', 'rosbridge', 'web'):
            say(f'  {svc:<10}: {"up" if compose_running(svc) else "down"}')
    if in_container():
        say(f'ROS setup : {"有" if Path(ROS_SETUP).is_file() else "缺"}')
        say(f'devel     : {"已编译" if Path(DEVEL_SETUP).is_file() else "未编译（先跑 build）"}')
    else:
        say('提示      : 宿主机上 build/test/gate/smoke 会自动进容器；改代码用 bind mount 即时生效')
    return 0


def cmd_build(args) -> int:
    extra = ' '.join(args.flags)
    inner = with_env(f'bash {CONT_DIR}/docker/build.sh {extra}'.strip())
    if in_container():
        return subprocess.run(['bash', '-lc', inner]).returncode
    return exec_in('sim-viz', [inner])


def cmd_test(args) -> int:
    targets = args.paths or ['ws/src/90_platform/v8_platform']
    inner = with_env('; '.join(_test_cmd(t) for t in targets))
    if in_container():
        return subprocess.run(['bash', '-lc', inner]).returncode
    return exec_in('sim-viz', [inner])


def _test_cmd(target: str) -> str:
    """单测的调用姿势取决于目标是目录还是文件（sys.path 依赖 cwd，不能凭空 python3 -m unittest）。"""
    if target.endswith('.py'):
        p = Path(target)
        return f'( cd {p.parent} && python3 -m unittest {p.stem} )'
    return f'( cd {target} && python3 -m unittest discover -p "test_*.py" )'


def cmd_gate(args) -> int:
    env_prefix = f'GATE_ONLY={args.members} ' if args.members else ''
    inner = with_env(f'{env_prefix}bash {CONT_DIR}/scripts/verify/run_gate_chain.sh')
    if in_container():
        cmd = ['bash', '-lc', inner]
    else:
        cmd = [*COMPOSE, 'exec', '-T', '-w', CONT_DIR, 'sim-viz', 'bash', '-lc',
               with_env(f'bash {CONT_DIR}/scripts/verify/run_gate_chain.sh')]

    proc = subprocess.run(cmd, capture_output=True, text=True)
    out = proc.stdout + proc.stderr
    say(out.rstrip())
    # run_gate_chain.sh 自己恒 exit 0（设计如此）；判定在这里做：
    rcs = [(m.group(1), int(m.group(2))) for m in
           (re.match(r'\s*(\S+)\s+rc=(\d+)', ln) for ln in out.splitlines()) if m]
    bad = [(tag, rc) for tag, rc in rcs if rc != 0]
    say()
    say(f'[dev] 汇总：{len(rcs)} 件，非零 {len(bad)} 件' +
        (f' → {", ".join(f"{t}(rc={rc})" for t, rc in bad)}' if bad else ' → 全绿'))
    if not rcs:
        return fail('没解析到任何「rc=N」行 —— 门禁串可能没跑起来，别把空输出当绿')
    return 1 if bad else 0


def cmd_docs(args) -> int:
    """重建 note 索引层；--check 只读核对（不一致返回 5，前置缺失 9）。"""
    flag = ' --check' if getattr(args, 'check', False) else ''
    cmd = f'python3 -B {REPO}/scripts/ops/compile_notes.py{flag}'
    return subprocess.run(['bash', '-lc', cmd], cwd=str(REPO)).returncode


def cmd_lint(args) -> int:
    """note 文档健康巡检（只读）：断链/孤儿/同名/戳失真/空件。有问题返回 5。"""
    flag = ' --all' if getattr(args, 'all', False) else ''
    cmd = f'python3 -B {REPO}/scripts/ops/lint_notes.py{flag}'
    return subprocess.run(['bash', '-lc', cmd], cwd=str(REPO)).returncode


def cmd_smoke(args) -> int:
    stage = args.stage
    inner = with_env(f'bash {CONT_DIR}/docker/sim.sh smoke {stage}')
    if in_container():
        return subprocess.run(['bash', '-lc', inner]).returncode
    return exec_in('sim', [inner])


# --------------------------------------------------------------------------- 入口

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog='dev', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    sub.add_parser('status', help='环境自检').set_defaults(fn=cmd_status)

    b = sub.add_parser('build', help='编译工作区')
    b.add_argument('flags', nargs='*', help='--plan / --clean')
    b.set_defaults(fn=cmd_build)

    t = sub.add_parser('test', help='跑单测')
    t.add_argument('paths', nargs='*', help='目录或 test_*.py；缺省扫 v8_platform')
    t.set_defaults(fn=cmd_test)

    g = sub.add_parser('gate', help='跑门禁串并汇总退出码')
    g.add_argument('members', nargs='?', default='', help='逗号分隔成员，如 SYN,PYST,DC')
    g.set_defaults(fn=cmd_gate)

    s = sub.add_parser('smoke', help='仿真冒烟')
    s.add_argument('stage', nargs='?', default='stageA')
    s.set_defaults(fn=cmd_smoke)

    d = sub.add_parser('docs', help='重建 note 索引层（--check 只读核对）')
    d.add_argument('--check', action='store_true', help='只读：不一致返回 5，前置缺失返回 9')
    d.set_defaults(fn=cmd_docs)

    l = sub.add_parser('lint', help='note 文档健康巡检（只读，有问题返回 5）')
    l.add_argument('--all', action='store_true', help='连历史区一起查')
    l.set_defaults(fn=cmd_lint)

    args = ap.parse_args(argv)
    try:
        return args.fn(args)
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    sys.exit(main())
