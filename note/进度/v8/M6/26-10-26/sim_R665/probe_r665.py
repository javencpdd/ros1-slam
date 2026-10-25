import hashlib
import json
import os
from pathlib import Path
import subprocess
import uuid

ROOT = Path('/home/jack/ysc-code/ros1-slam')
PRIVATE = Path('/home/jack/temp/ros1-slam-r665')
IMAGE = 'sha256:59f759dd5001f3b40da2c2678644ab9f578e7d45a62e757c9a7ba44195caf7d7'
LABEL = 'org.ros1-slam.r665-stage'


def run(args, timeout=120):
    result = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, timeout=timeout)
    return {'argv': args, 'rc': result.returncode, 'stdout': result.stdout, 'stderr': result.stderr}


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
    """仓库侧输入指纹（ws/src 头两包对账用 scripts/verify）；私有目录另用 source_hashes。"""
    result = {}
    for rel in ('scripts/verify',):
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


def source_hashes():
    """R660 无源码副本（不构建）：指纹面由驱动另核（scripts/verify 前后一致）。"""
    return {}


def create_args(token):
    return ['docker', 'create', '--name', 'ros1-slam-r665-stage-' + token,
            '--label', LABEL + '=' + token, '--network', 'none', '--read-only',
            '--cap-drop', 'ALL', '--security-opt', 'no-new-privileges',
            '--cpus', '4', '--memory', '8g', '--memory-swap', '8g', '--pids-limit', '512',
            '--user', '1000:1000',
            '--tmpfs', '/tmp:rw,nosuid,nodev,size=536870912,mode=1777',
            '--env', 'ROS_HOME=/tmp/ros', '--env', 'PYTHONDONTWRITEBYTECODE=1',
            '--env', 'PYTHONNOUSERSITE=1',
            '--mount', f'type=bind,src={ROOT}/ws/src,dst=/opt/ros1-slam/ws/src,readonly',
            '--mount', f'type=bind,src={ROOT}/scripts,dst=/opt/ros1-slam/scripts,readonly',
            '--entrypoint', '/bin/bash', IMAGE, '-c', 'true']
