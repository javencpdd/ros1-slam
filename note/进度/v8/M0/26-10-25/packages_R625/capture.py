import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import subprocess
import sys
import tarfile
import uuid
from datetime import datetime, timezone

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
IMAGE = 'sha256:59f759dd5001f3b40da2c2678644ab9f578e7d45a62e757c9a7ba44195caf7d7'
LABEL = 'org.ros1-slam.r625'
PACKAGES = ['ros-noetic-' + n for n in (
    'move-base', 'move-base-flex', 'mbf-costmap-nav', 'global-planner', 'navfn',
    'base-local-planner', 'dwa-local-planner', 'teb-local-planner', 'map-server',
    'costmap-2d', 'spatio-temporal-voxel-layer', 'sob-layer')]


def run(args):
    p = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120)
    return p


def checked(args):
    p = run(args)
    if p.returncode:
        raise RuntimeError((args, p.returncode, p.stderr.decode(errors='replace')))
    return p.stdout.decode()


def digest(data):
    return hashlib.sha256(data).hexdigest()


def parse_file(raw, path):
    expected = PurePosixPath(path).name
    with tarfile.open(fileobj=io.BytesIO(raw)) as archive:
        members = archive.getmembers()
        if len(members) != 1:
            raise ValueError('expected exactly one archive member')
        item = members[0]
        if item.name != expected or not item.isfile():
            raise ValueError('wrong name or nonregular member')
        data = archive.extractfile(item).read()
    return {'sha256': digest(data), 'bytes': len(data), 'text': data.decode('utf-8')}


def copy_file(cid, path):
    args = ['docker', 'cp', cid + ':' + path, '-']
    p = run(args)
    row = {'path': path, 'command': args, 'rc': p.returncode,
           'stderr': p.stderr.decode(errors='replace')}
    if not p.returncode:
        row.update(parse_file(p.stdout, path))
    return row


def snapshot():
    ids = checked(['docker', 'ps', '-aq']).split()
    info = json.loads(checked(['docker', 'inspect', *ids])) if ids else []
    return {r['Id']: {k: r[k] for k in ('Name', 'Image', 'State', 'Mounts')} for r in info}


def owned(info, name, token):
    return (info.get('Name') == '/' + name and info.get('Config', {}).get('Labels', {}).get(LABEL) == token
            and info.get('Image') == IMAGE and info.get('State', {}).get('Status') == 'created'
            and info['State'].get('Running') is False)


def collect():
    token = uuid.uuid4().hex
    name = 'ros1-slam-r625-' + token
    data = {'started_at': datetime.now(timezone.utc).isoformat(), 'image': IMAGE,
            'collector_sha256': digest(Path(__file__).read_bytes()), 'errors': [], 'packages': [],
            'meaning': 'static package evidence; not upstream grant, retrieval, build or runtime acceptance'}
    attempted = False
    try:
        data['before'] = snapshot()
        data['volumes_before'] = checked(['docker', 'volume', 'ls', '--format', '{{.Name}}']).splitlines()
        args = ['docker', 'create', '--name', name, '--label', LABEL + '=' + token,
                '--network', 'none', '--read-only', '--cap-drop', 'ALL',
                '--security-opt', 'no-new-privileges', '--entrypoint', '/bin/true', IMAGE]
        data['create_command'] = args
        attempted = True
        cid = checked(args).strip()
        info = json.loads(checked(['docker', 'inspect', cid]))[0]
        if not owned(info, name, token):
            raise ValueError('container identity/state mismatch')
        if info['Mounts'] or info['HostConfig']['NetworkMode'] != 'none':
            raise ValueError('unexpected mounts/network')
        data['container'] = {k: info[k] for k in ('Id', 'Name', 'Image', 'State', 'Mounts', 'HostConfig')}
        data['dpkg_status'] = copy_file(cid, '/var/lib/dpkg/status')
        for package in PACKAGES:
            ros_name = package.removeprefix('ros-noetic-').replace('-', '_')
            file_list = copy_file(cid, '/var/lib/dpkg/info/' + package + '.list')
            listed = file_list.get('text', '').splitlines()
            paths = sorted({p for p in listed if p.endswith('/copyright') or p.endswith('/package.xml')})
            expected_xml = '/opt/ros/noetic/share/' + ros_name + '/package.xml'
            if expected_xml not in paths:
                paths.append(expected_xml)
            data['packages'].append({'package': package, 'file_list': file_list,
                                     'documents': [copy_file(cid, p) for p in paths]})
    except Exception as exc:
        data['errors'].append('capture: ' + repr(exc))
    finally:
        if attempted:
            try:
                info = json.loads(checked(['docker', 'inspect', name]))[0]
                if not owned(info, name, token):
                    raise ValueError('refusing cleanup of foreign or started container')
                data['final_state'] = info['State']
                args = ['docker', 'rm', info['Id']]
                p = run(args)
                data['cleanup'] = {'command': args, 'rc': p.returncode, 'stdout': p.stdout.decode(),
                                   'stderr': p.stderr.decode(errors='replace')}
                if p.returncode:
                    raise RuntimeError('remove failed')
            except Exception as exc:
                data['errors'].append('cleanup: ' + repr(exc))
        try:
            data['after'] = snapshot()
            data['existing_unchanged'] = 'before' in data and data['before'] == data['after']
            data['volumes_after'] = checked(['docker', 'volume', 'ls', '--format', '{{.Name}}']).splitlines()
            data['volume_names_unchanged'] = data.get('volumes_before') == data['volumes_after']
            data['remaining'] = checked(['docker', 'ps', '-aq', '--filter', 'label=' + LABEL + '=' + token]).split()
        except Exception as exc:
            data['errors'].append('resample: ' + repr(exc))
    data['completed_at'] = datetime.now(timezone.utc).isoformat()
    data['capture_complete'] = (not data['errors'] and len(data['packages']) == len(PACKAGES)
        and data.get('existing_unchanged') is True and data.get('volume_names_unchanged') is True
        and data.get('remaining') == [] and data.get('cleanup', {}).get('rc') == 0
        and data.get('dpkg_status', {}).get('rc') == 0
        and all(p['file_list']['rc'] == 0 for p in data['packages']))
    return data


if __name__ == '__main__':
    output = Path(sys.argv[1])
    with output.open('x') as stream:
        result = collect()
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    subprocess.run(['setfacl', '-m', 'u:ros1-slam:rwx', str(output)], check=True)
    print(json.dumps({k: result.get(k) for k in ('capture_complete', 'errors', 'existing_unchanged',
          'volume_names_unchanged', 'remaining')}, ensure_ascii=False))
    raise SystemExit(0 if result['capture_complete'] else 5)
