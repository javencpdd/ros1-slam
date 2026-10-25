import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[6]
IMAGE = 'sha256:59f759dd5001f3b40da2c2678644ab9f578e7d45a62e757c9a7ba44195caf7d7'
LABEL = 'org.ros1-slam.r624'


def run(args, timeout=120):
    result = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, timeout=timeout)
    return {'argv': args, 'rc': result.returncode,
            'stdout': result.stdout, 'stderr': result.stderr}


def checked(args):
    result = run(args)
    if result['rc']:
        raise RuntimeError(result)
    return result['stdout']


def state():
    ids = checked(['docker', 'ps', '-aq', '--no-trunc']).split()
    containers = json.loads(checked(['docker', 'inspect', *ids])) if ids else []
    volumes = checked(['docker', 'volume', 'ls', '-q']).split()
    return {
        'containers': {c['Id']: {k: c[k] for k in ('Name', 'Image', 'State', 'Mounts')}
                       for c in containers},
        'volumes': json.loads(checked(['docker', 'volume', 'inspect', *volumes])) if volumes else [],
    }


def fingerprint():
    result = {}
    for rel in ('ws/src', 'opt/3rdparty', 'docker', 'scripts/verify'):
        base = ROOT / rel
        if not base.is_dir():
            raise ValueError(f'missing input: {base}')
        for folder, dirs, files in os.walk(base, followlinks=False):
            for name in sorted(dirs + files):
                path = Path(folder) / name
                key = str(path.relative_to(ROOT))
                if path.is_symlink():
                    result[key] = {'link': os.readlink(path)}
                elif path.is_file():
                    result[key] = {'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    return result


def create_args(token):
    return ['docker', 'create', '--name', 'ros1-slam-r624-' + token,
            '--label', LABEL + '=' + token, '--network', 'none', '--read-only',
            '--cap-drop', 'ALL', '--security-opt', 'no-new-privileges',
            '--cpus', '4', '--memory', '8g', '--memory-swap', '8g', '--pids-limit', '512',
            '--user', '1000:1000', '--tmpfs', '/tmp:rw,nosuid,nodev,size=268435456,mode=1777',
            '--env', 'ROS_HOME=/tmp/ros', '--env', 'ROS_LOG_DIR=/tmp/ros/log',
            '--env', 'PYTHONDONTWRITEBYTECODE=1', '--env', 'PYTHONNOUSERSITE=1',
            '--mount', f'type=bind,src={ROOT}/ws/src,dst=/opt/ros1-slam/ws/src,readonly',
            '--mount', f'type=bind,src={ROOT}/scripts,dst=/opt/ros1-slam/scripts,readonly',
            '--entrypoint', '/bin/bash', IMAGE, '-c',
            'mkdir -p /tmp/ros; '
            'bash /opt/ros1-slam/scripts/verify/v_window_preflight_x86_variant.sh; rc=$?; '
            'printf "\\nR624_PREFLIGHT_RC=%s\\n" "$rc"; '
            'if [ "$rc" != 0 ]; then '
            'source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash; '
            'bash /opt/ros1-slam/scripts/verify/_iso_alt_wrapper.sh; '
            'printf "R624_ISO_DIAGNOSTIC_RC=%s\\n" "$?"; fi; exit "$rc"']


def main(output):
    with output.open('x') as stream:
        checked(['setfacl', '-m', 'u:ros1-slam:rwx', str(output)])
        data = {'started_at': datetime.now(timezone.utc).isoformat(),
                'scope': 'bootstrap_preflight_with_current_readonly_source_and_old_image_devel_not_build_acceptance',
                'image': IMAGE, 'run_token': uuid.uuid4().hex[:12],
                'host': {key: run(cmd) for key, cmd in {
                    'load': ['uptime'], 'cpu': ['nproc'], 'memory': ['free', '-m'],
                    'disk': ['df', '-h', str(ROOT), '/tmp'],
                    'processes': ['ps', '-eo', 'pid,user,pcpu,comm', '--sort=-pcpu'],
                    'git': ['git', 'status', '--short']}.items()}}
        cid = None
        try:
            data['before'] = state()
            data['input_before'] = fingerprint()
            data['create_argv'] = create_args(data['run_token'])
            cid = checked(data['create_argv']).strip()
            data['container_id'] = cid
            info = json.loads(checked(['docker', 'inspect', cid]))[0]
            data['isolation'] = {key: info[key] for key in ('Id', 'Image', 'HostConfig', 'Mounts')}
            if info['Image'] != IMAGE or info['HostConfig']['NetworkMode'] != 'none':
                raise ValueError('image/network mismatch')
            if not info['HostConfig']['ReadonlyRootfs'] or len(info['Mounts']) != 2:
                raise ValueError('root/mount mismatch')
            if any(m['Type'] != 'bind' or m['RW'] for m in info['Mounts']):
                raise ValueError('unexpected writable/shared mount')
            data['preflight'] = run(['docker', 'start', '-a', cid], timeout=180)
            data['exit_state'] = json.loads(checked(['docker', 'inspect', cid]))[0]['State']
        except Exception as exc:
            data['error'] = str(exc)
        finally:
            if cid:
                info = json.loads(checked(['docker', 'inspect', cid]))[0]
                if info['Config']['Labels'].get(LABEL) != data['run_token']:
                    raise ValueError('refuse cleanup: ownership mismatch')
                if info['State']['Running']:
                    data['stop'] = run(['docker', 'stop', '-t', '10', cid])
                data['remove'] = run(['docker', 'rm', cid])
            data['after'] = state()
            data['input_after'] = fingerprint()
            data['existing_state_unchanged'] = data.get('before') == data['after']
            data['inputs_unchanged'] = data.get('input_before') == data['input_after']
            data['remaining'] = checked(['docker', 'ps', '-aq', '--filter',
                                         'label=' + LABEL + '=' + data['run_token']]).split()
            data['finished_at'] = datetime.now(timezone.utc).isoformat()
            json.dump(data, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
    print(json.dumps({k: data.get(k) for k in ('error', 'container_id', 'exit_state',
                     'existing_state_unchanged', 'inputs_unchanged', 'remaining')}, ensure_ascii=False))
    print(data.get('preflight', {}).get('stdout', ''))
    return 0 if (not data.get('error') and data['existing_state_unchanged']
                 and data['inputs_unchanged'] and not data['remaining']
                 and data['exit_state']['ExitCode'] == 0) else 5


if __name__ == '__main__':
    sys.exit(main(Path(sys.argv[1])))
