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
import ipaddress
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import socket
import subprocess
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = Path(__file__).resolve().parents[2]
STACKS_FILE = Path(__file__).resolve().parent / 'stacks.json'
VOXEL_DIR = ROOT / 'data/console/voxels'
DIR = '/tmp/v8-console'
LOCK = threading.Lock()

DEFAULT_LOCAL_CIDRS = '127.0.0.0/8,::1/128'


def normalize_ip(value):
    raw = (value or '').split('%')[0]
    try:
        address = ipaddress.ip_address(raw)
    except ValueError:
        return raw
    if isinstance(address, ipaddress.IPv6Address) and address.ipv4_mapped:
        address = address.ipv4_mapped
    return str(address)


def host_ips():
    found = {'127.0.0.1', '::1'}
    try:
        result = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=2)
        found.update(result.stdout.split())
    except (OSError, subprocess.SubprocessError):
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            found.add(info[4][0])
    except OSError:
        pass
    return {normalize_ip(value) for value in found if value}


def local_cidrs():
    raw = os.environ.get('LITE3_LOCAL_CIDRS', DEFAULT_LOCAL_CIDRS)
    networks = []
    for part in raw.split(','):
        part = part.strip()
        if not part:
            continue
        try:
            networks.append(ipaddress.ip_network(part, strict=False))
        except ValueError:
            continue
    return networks


def classify_client(client_ip, known_host_ips=None, cidrs=None):
    address_text = normalize_ip(client_ip)
    try:
        address = ipaddress.ip_address(address_text)
    except ValueError:
        return False, 'invalid'
    if address.is_loopback:
        return True, 'loopback'
    known = known_host_ips if known_host_ips is not None else host_ips()
    if address_text in known:
        return True, 'host-ip'
    for network in (cidrs if cidrs is not None else local_cidrs()):
        if address.version == network.version and address in network:
            return True, 'local-cidr'
    return False, 'lan'


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
    save_rel = stack.get('save_map')
    if save_rel:
        sample = ROOT / save_rel
        backup = Path('/tmp/v8-console') / (name + '.sample.pcd')
        if sample.exists():
            backup.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(sample, backup)
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


def finalize_saved_map(stack):
    """Move the faster-lio output to imported/ and restore the GAP-5 sample."""
    save_rel = stack.get('save_map')
    if not save_rel:
        return None
    sample = ROOT / save_rel
    imported = ROOT / 'data/maps/imported'
    imported.mkdir(parents=True, exist_ok=True)
    message = None
    if sample.exists() and sample.stat().st_size > 0:
        name = 'map_' + time.strftime('%Y%m%d_%H%M%S') + '.pcd'
        shutil.move(str(sample), str(imported / name))
        message = 'saved data/maps/imported/' + name
    backup = Path('/tmp/v8-console') / (stack['name'] + '.sample.pcd')
    if backup.exists():
        shutil.copy2(backup, sample)
    try:
        run_script(['python3', 'scripts/ops/build_maps_index.py'], 120)
    except (OSError, subprocess.SubprocessError):
        pass
    return message


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
        saved = finalize_saved_map(stack)
        return True, 'not running' + ('；' + saved if saved else '')
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
    saved = finalize_saved_map(stack)
    return True, 'stopped' + ('；' + saved if saved else '')


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
        'world': stack.get('world'),
        'requires': stack.get('requires') or [],
        'local_only': bool(stack.get('local_only')),
        'notes': stack.get('notes', ''),
        'state': state if not error else 'error',
        'pid': pid,
        'error': error,
    }


def run_script(args, timeout=120):
    return subprocess.run(args, cwd=str(ROOT), capture_output=True, text=True,
                          timeout=timeout)


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

    def _binary(self, code, body):
        self.send_response(code)
        self.send_header('Content-Type', 'application/octet-stream')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('Cache-Control', 'no-store')
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
        if parts == ['api', 'whoami']:
            client = self.client_address[0]
            local, reason = classify_client(client)
            self._json(200, {
                'ok': True,
                'local': local,
                'reason': reason,
                'client': normalize_ip(client),
                'host': socket.gethostname(),
            })
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
        if parts == ['api', 'voxels', 'meta']:
            path = VOXEL_DIR / 'meta.json'
            if not path.exists():
                self._json(404, {'ok': False,
                                 'error': 'no voxel snapshot; start the voxel_stream stack'})
                return
            try:
                payload = json.loads(path.read_text(encoding='utf-8'))
            except (OSError, ValueError) as error:
                self._json(500, {'ok': False, 'error': str(error)})
                return
            self._json(200, payload)
            return
        if parts == ['api', 'voxels', 'latest']:
            path = VOXEL_DIR / 'latest.bin'
            if not path.exists():
                self._json(404, {'ok': False,
                                 'error': 'no voxel snapshot; start the voxel_stream stack'})
                return
            try:
                body = path.read_bytes()
            except OSError as error:
                self._json(500, {'ok': False, 'error': str(error)})
                return
            self._binary(200, body)
            return
        self._json(404, {'ok': False, 'error': 'not found'})

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        parts = [p for p in parsed.path.split('/') if p]
        if parts == ['api', 'maps', 'refresh']:
            try:
                result = run_script(['python3', 'scripts/ops/build_maps_index.py'], 120)
            except (OSError, subprocess.SubprocessError) as error:
                self._json(500, {'ok': False, 'error': str(error)})
                return
            self._json(200 if result.returncode == 0 else 500, {
                'ok': result.returncode == 0,
                'output': (result.stdout or '')[-2000:] + (result.stderr or '')[-2000:],
            })
            return
        if parts == ['api', 'maps', 'edit']:
            length = int(self.headers.get('Content-Length') or 0)
            try:
                body = json.loads(self.rfile.read(length).decode('utf-8') or '{}')
            except ValueError:
                self._json(400, {'ok': False, 'error': 'bad json'})
                return
            map_id = str(body.get('id') or '')
            if not map_id.startswith('console_'):
                self._json(400, {'ok': False, 'error': 'only console_ maps are editable'})
                return
            name = map_id[len('console_'):]
            if not re.fullmatch(r'[A-Za-z0-9_.-]{1,64}', name):
                self._json(400, {'ok': False, 'error': 'bad map id'})
                return
            folder = ROOT / 'data/maps/console' / name
            pgm_path = folder / 'map.pgm'
            if not pgm_path.exists():
                self._json(404, {'ok': False, 'error': 'map.pgm not found'})
                return
            raw = pgm_path.read_bytes()
            # P5 header: magic, "width height", maxval, then binary data
            lines = []
            index = 0
            for _ in range(3):
                end = raw.index(b'\n', index)
                lines.append(raw[index:end].strip())
                index = end + 1
            if lines[0] != b'P5':
                self._json(400, {'ok': False, 'error': 'not a P5 pgm'})
                return
            width, height = [int(v) for v in lines[1].split()]
            data = bytearray(raw[index:])
            changes = body.get('changes') or []
            if len(changes) > 20000:
                self._json(400, {'ok': False, 'error': 'too many changes'})
                return
            applied = 0
            for change in changes:
                try:
                    col, row, value = int(change[0]), int(change[1]), int(change[2])
                except (TypeError, ValueError, IndexError):
                    continue
                if value not in (0, 254):
                    continue
                if 0 <= col < width and 0 <= row < height:
                    data[(height - 1 - row) * width + col] = value
                    applied += 1
            pgm_path.write_bytes(b'P5\n%d %d\n255\n' % (width, height) + bytes(data))
            try:
                refresh = run_script(['python3', 'scripts/ops/build_maps_index.py'], 120)
            except (OSError, subprocess.SubprocessError) as error:
                self._json(500, {'ok': False, 'error': str(error)})
                return
            self._json(200, {'ok': refresh.returncode == 0, 'applied': applied,
                             'output': (refresh.stdout or '')[-1000:]})
            return
        if parts == ['api', 'maps', 'regrid']:
            length = int(self.headers.get('Content-Length') or 0)
            try:
                body = json.loads(self.rfile.read(length).decode('utf-8') or '{}')
            except ValueError:
                self._json(400, {'ok': False, 'error': 'bad json'})
                return
            source = str(body.get('source') or '')
            name = str(body.get('name') or '')
            if not source.startswith('data/maps/') or '..' in source:
                self._json(400, {'ok': False, 'error': 'source must be under data/maps'})
                return
            if not re.fullmatch(r'[A-Za-z0-9_.-]{1,64}', name):
                self._json(400, {'ok': False, 'error': 'bad name'})
                return
            source_path = (ROOT / source).resolve()
            if not str(source_path).startswith(str((ROOT / 'data/maps').resolve())) or not source_path.exists():
                self._json(400, {'ok': False, 'error': 'source not found'})
                return
            try:
                resolution = float(body.get('resolution') or 0.05)
                min_z = float(body.get('min_z', -1.0))
                max_z = float(body.get('max_z', 2.0))
            except (TypeError, ValueError):
                self._json(400, {'ok': False, 'error': 'bad numeric params'})
                return
            try:
                result = run_script(['python3', 'scripts/ops/regrid_map.py',
                                     '--pcd', str(source_path), '--name', name,
                                     '--resolution', str(resolution),
                                     '--min-z', str(min_z), '--max-z', str(max_z)], 180)
            except (OSError, subprocess.SubprocessError) as error:
                self._json(500, {'ok': False, 'error': str(error)})
                return
            if result.returncode != 0:
                self._json(500, {'ok': False, 'error': (result.stdout + result.stderr)[-2000:]})
                return
            try:
                refresh = run_script(['python3', 'scripts/ops/build_maps_index.py'], 120)
            except (OSError, subprocess.SubprocessError) as error:
                self._json(500, {'ok': False, 'error': str(error)})
                return
            self._json(200, {'ok': refresh.returncode == 0, 'map_id': 'console_' + name,
                             'output': (result.stdout + result.stderr)[-2000:] +
                                       (refresh.stdout or '')[-1000:]})
            return
        if len(parts) != 4 or parts[:2] != ['api', 'stacks'] or parts[3] not in ('start', 'stop'):
            self._json(404, {'ok': False, 'error': 'not found'})
            return
        stack = STACKS.get(parts[2])
        if not stack:
            self._json(404, {'ok': False, 'error': 'unknown stack'})
            return
        if stack.get('local_only'):
            local, reason = classify_client(self.client_address[0])
            if not local:
                self._json(403, {'ok': False,
                                 'error': 'local-only stack; local client required (%s)' % reason})
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
