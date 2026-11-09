#!/usr/bin/env python3
"""Host-side whitelist agent for the Lite3 console (P3 stack lifecycle).

The web page can only name a stack listed in stacks.json; it can never pass a
command line. Each stack runs in the ros1-slam-sim-viz container in its own
session/process group, with a per-stack log and PID file under /tmp/v8-console.
Stop sends SIGINT -> SIGTERM -> SIGKILL to the whole group.

This is a demo/visualization whitelist. Formal/EVT runs, real-robot launch and
work-order windows stay in the CLI; the agent does not accept arbitrary
commands and binds 127.0.0.1 by default.

Usage:
  python3 scripts/console/agent.py [--bind 127.0.0.1] [--port 8780]
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = Path(__file__).resolve().parents[2]
STACKS_FILE = Path(__file__).resolve().parent / 'stacks.json'
DIR = '/tmp/v8-console'
LOCK = threading.Lock()


def load_stacks():
    data = json.loads(STACKS_FILE.read_text(encoding='utf-8'))
    return {item['name']: item for item in data.get('stacks', [])}


STACKS = load_stacks()


def docker(args, timeout=20):
    return subprocess.run(['docker'] + args, capture_output=True, text=True,
                          timeout=timeout)


def stack_paths(name):
    return '%s/%s.pid' % (DIR, name), '%s/%s.log' % (DIR, name)


def stack_state(stack):
    name = stack['name']
    container = stack['container']
    pidfile, _log = stack_paths(name)
    script = (
        'if [ -f %s ]; then pid=$(cat %s); '
        'if kill -0 -- -$pid 2>/dev/null; then echo "running $pid"; '
        'else echo "stale $pid"; fi; else echo stopped; fi'
        % (shlex.quote(pidfile), shlex.quote(pidfile)))
    try:
        result = docker(['exec', container, 'bash', '-lc', script], timeout=8)
    except (OSError, subprocess.SubprocessError) as error:
        return 'error', None, str(error)
    out = (result.stdout or '').strip()
    if out.startswith('running'):
        return 'running', out.split()[1], None
    if out.startswith('stale'):
        return 'stale', out.split()[1], None
    return 'stopped', None, None


def start_stack(stack):
    name = stack['name']
    state, pid, error = stack_state(stack)
    if error:
        return False, error
    if state == 'running':
        return True, 'already running (pid %s)' % pid
    requires = stack.get('requires') or []
    for required in requires:
        other = STACKS.get(required)
        if not other:
            return False, 'requires unknown stack %s' % required
        rstate, _rpid, rerr = stack_state(other)
        if rerr or rstate != 'running':
            return False, '先启动 %s（requires %s）' % (other.get('label', required), required)
    pidfile, logfile = stack_paths(name)
    master = stack.get('master') or 'http://roscore:11312'
    setup = (
        'source /opt/ros/noetic/setup.bash; '
        'source /opt/ros1-slam/ws/devel/setup.bash; '
        'export ROS_MASTER_URI=%s; export ROS_HOSTNAME=$(hostname); '
        'cd /opt/ros1-slam; exec %s'
        % (shlex.quote(master), stack['cmd']))
    outer = (
        'mkdir -p %s; setsid nohup bash -lc %s > %s 2>&1 < /dev/null & '
        'echo $! > %s'
        % (shlex.quote(DIR), shlex.quote(setup), shlex.quote(logfile),
           shlex.quote(pidfile)))
    try:
        result = docker(['exec', '-d', stack['container'], 'bash', '-lc', outer],
                        timeout=10)
    except (OSError, subprocess.SubprocessError) as error:
        return False, str(error)
    if result.returncode != 0:
        return False, (result.stderr or result.stdout or 'docker exec failed').strip()
    time.sleep(1.5)
    state, pid, error = stack_state(stack)
    if error:
        return False, error
    if state != 'running':
        return False, '启动后未保持运行；看日志：%s' % logfile
    return True, 'started (pid %s)' % pid


def stop_stack(stack):
    name = stack['name']
    container = stack['container']
    pidfile, _log = stack_paths(name)
    state, pid, error = stack_state(stack)
    if error:
        return False, error
    if state != 'running':
        try:
            docker(['exec', container, 'bash', '-lc', 'rm -f %s' % shlex.quote(pidfile)],
                   timeout=5)
        except (OSError, subprocess.SubprocessError):
            pass
        return True, 'not running'
    timeout = float(stack.get('stop_timeout', 10))
    stages = [('INT', timeout), ('TERM', 5), ('KILL', 2)]
    for sig, wait in stages:
        try:
            docker(['exec', container, 'bash', '-lc',
                    'kill -%s -- -%s 2>/dev/null || true' % (sig, pid)], timeout=5)
        except (OSError, subprocess.SubprocessError):
            pass
        deadline = time.time() + wait
        while time.time() < deadline:
            state, _pid, error = stack_state(stack)
            if error or state != 'running':
                break
            time.sleep(0.3)
        if state != 'running':
            break
    try:
        docker(['exec', container, 'bash', '-lc', 'rm -f %s' % shlex.quote(pidfile)],
               timeout=5)
    except (OSError, subprocess.SubprocessError):
        pass
    state, _pid, _error = stack_state(stack)
    if state == 'running':
        return False, '仍在运行；请用作业单手工清场'
    return True, 'stopped'


def stack_log(stack, tail=80):
    _pidfile, logfile = stack_paths(stack['name'])
    tail = max(1, min(500, int(tail)))
    try:
        result = docker(['exec', stack['container'], 'bash', '-lc',
                         'tail -n %d %s' % (tail, shlex.quote(logfile))], timeout=10)
        return (result.stdout or '') + (result.stderr or '')
    except (OSError, subprocess.SubprocessError) as error:
        return 'log unavailable: %s' % error


def public_stack(stack):
    state, pid, error = stack_state(stack)
    return {
        'name': stack['name'],
        'label': stack.get('label', stack['name']),
        'description': stack.get('description', ''),
        'category': stack.get('category', 'demo'),
        'container': stack.get('container'),
        'master': stack.get('master'),
        'requires': stack.get('requires') or [],
        'notes': stack.get('notes', ''),
        'state': state if not error else 'error',
        'pid': pid,
        'error': error,
    }


class Handler(BaseHTTPRequestHandler):
    server_version = 'Lite3ConsoleAgent/1.0'

    def _json(self, code, payload):
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self._json(200, {'ok': True})

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        parts = [p for p in parsed.path.split('/') if p]
        if parsed.path == '/api/health':
            self._json(200, {'ok': True, 'stacks': len(STACKS)})
            return
        if parts[:2] == ['api', 'stacks'] and len(parts) == 2:
            with LOCK:
                items = [public_stack(s) for s in STACKS.values()]
            self._json(200, {'ok': True, 'stacks': items})
            return
        if parts[:2] == ['api', 'stacks'] and len(parts) == 4 and parts[3] == 'log':
            stack = STACKS.get(parts[2])
            if not stack:
                self._json(404, {'ok': False, 'error': 'unknown stack'})
                return
            query = urllib.parse.parse_qs(parsed.query)
            tail = query.get('tail', ['80'])[0]
            self._json(200, {'ok': True, 'log': stack_log(stack, tail)})
            return
        self._json(404, {'ok': False, 'error': 'not found'})

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        parts = [p for p in parsed.path.split('/') if p]
        if len(parts) != 4 or parts[:2] != ['api', 'stacks'] or parts[3] not in ('start', 'stop'):
            self._json(404, {'ok': False, 'error': 'not found'})
            return
        stack = STACKS.get(parts[2])
        if not stack:
            self._json(404, {'ok': False, 'error': 'unknown stack'})
            return
        with LOCK:
            if parts[3] == 'start':
                ok, message = start_stack(stack)
            else:
                ok, message = stop_stack(stack)
        self._json(200 if ok else 500, {
            'ok': ok, 'message': message, 'stack': public_stack(stack),
        })

    def log_message(self, fmt, *args):
        print('[agent] ' + (fmt % args), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8780)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    print('Lite3 console agent on http://%s:%d ｜ stacks=%d' % (
        args.bind, args.port, len(STACKS)), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
