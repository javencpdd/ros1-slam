import hashlib
import json
from pathlib import Path
import subprocess
import sys
import uuid
from datetime import datetime, timezone

import probe_r633 as probe

HERE = Path(__file__).resolve().parent
PRIVATE = probe.PRIVATE
DEST = '/opt/ros1-slam/r633'
LABEL = probe.LABEL
TIMEOUT = {'build': 1500, 'smoke': 300}


def main(stage, output):
    if stage not in TIMEOUT:
        raise ValueError('unknown stage')
    with output.open('x') as stream:
        token = uuid.uuid4().hex[:12]
        name = 'ros1-slam-r633-' + stage + '-' + token
        data = {'stage': stage, 'started_at': datetime.now(timezone.utc).isoformat(),
                'name': name, 'token': token, 'errors': [],
                'driver_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
        cid = None
        attempted = False
        result_path = PRIVATE / ('build-result.json' if stage == 'build' else 'smoke-result.json')
        try:
            data['before'] = probe.state()
            data['input_before'] = probe.fingerprint()
            data['source_before'] = probe.source_hashes()
            assert data['source_before'] == json.loads((PRIVATE / 'source-manifest.json').read_text())['files']
            assert not result_path.exists(), 'refusing an existing stage result'
            snapshot = HERE / ('execution-' + token)
            snapshot.mkdir()
            names = ['mode3_stage.py', 'probe_r633.py', stage + '.sh']
            if stage == 'smoke':
                names += ['smoke.launch', 'smoke_driver.py']
            for filename in names:
                (snapshot / filename).write_bytes((HERE / filename).read_bytes())
            data['execution_snapshot'] = snapshot.name
            data['execution_inputs_before'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                               for p in sorted(snapshot.iterdir())}
            args = probe.create_args(token)
            args[args.index('--name') + 1] = name
            args[args.index('--label') + 1] = LABEL + '=' + token
            args[args.index('--tmpfs') + 1] = '/tmp:rw,nosuid,nodev,size=536870912,mode=1777'
            idx = args.index('--entrypoint')
            args[idx:idx] = ['--mount', f'type=bind,src={PRIVATE},dst={DEST}',
                             '--mount', f'type=bind,src={snapshot},dst={DEST}/inputs,readonly']
            args[-1] = ('mkdir -p /tmp/ros; bash /opt/ros1-slam/scripts/verify/v8_window_preflight.sh '
                        '|| exit $?; exec env -i HOME=/tmp/ros USER=ros1-slam '
                        'PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin '
                        'ROS_HOME=/tmp/ros ROS_LOG_DIR=' + DEST + '/logs '
                        'PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 R633_RUN_TOKEN=' + token + ' '
                        '/bin/bash ' + DEST + '/inputs/' + stage + '.sh')
            data['create_argv'] = args
            attempted = True
            cid = probe.checked(args).strip()
            data['id'] = cid
            info = json.loads(probe.checked(['docker', 'inspect', cid]))[0]
            data['isolation'] = {k: info[k] for k in ('Id', 'Image', 'HostConfig', 'Mounts')}
            mounts = {m['Destination']: m for m in info['Mounts']}
            assert info['Image'] == probe.IMAGE and info['HostConfig']['NetworkMode'] == 'none'
            assert info['HostConfig']['ReadonlyRootfs'] and len(mounts) == 4
            assert mounts[DEST]['Source'] == str(PRIVATE) and mounts[DEST]['RW']
            assert all(m['Type'] == 'bind' and (not m['RW'] or p == DEST) for p, m in mounts.items())
            data['execution'] = probe.run(['docker', 'start', '-a', cid], timeout=TIMEOUT[stage])
            data['exit_state'] = json.loads(probe.checked(['docker', 'inspect', cid]))[0]['State']
        except Exception as error:
            data['errors'].append(repr(error))
        finally:
            if attempted:
                try:
                    info = json.loads(probe.checked(['docker', 'inspect', cid or name]))[0]
                    assert info['Name'] == '/' + name and info['Config']['Labels'].get(LABEL) == token
                    if info['State']['Running']:
                        data['forced_stop'] = probe.run(['docker', 'stop', '-t', '10', info['Id']])
                        assert data['forced_stop']['rc'] == 0
                    data['remove'] = probe.run(['docker', 'rm', info['Id']])
                    assert data['remove']['rc'] == 0
                except Exception as error:
                    data['errors'].append('cleanup: ' + repr(error))
            try:
                data['after'] = probe.state()
                data['existing_state_unchanged'] = data.get('before') == data['after']
            except Exception as error:
                data['errors'].append('after: ' + repr(error))
            try:
                data['input_after'] = probe.fingerprint()
                data['inputs_unchanged'] = data.get('input_before') == data['input_after']
            except Exception as error:
                data['errors'].append('input_after: ' + repr(error))
            data['source_after'] = probe.source_hashes()
            data['source_unchanged'] = data.get('source_before') == data['source_after']
            snap = HERE / data.get('execution_snapshot', '')
            if snap.is_dir():
                data['execution_inputs_after'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                                  for p in sorted(snap.iterdir())}
                data['execution_inputs_unchanged'] = data.get('execution_inputs_before') == data['execution_inputs_after']
            if result_path.exists():
                data['result_sha256'] = hashlib.sha256(result_path.read_bytes()).hexdigest()
            status = data.get('exit_state', {})
            data['accepted'] = (not data['errors']
                                and data['execution']['rc'] == 0
                                and status.get('Status') == 'exited' and status.get('ExitCode') == 0
                                and status.get('OOMKilled') is False
                                and data.get('existing_state_unchanged') is True
                                and data.get('inputs_unchanged') is True
                                and data.get('source_unchanged') is True
                                and data.get('execution_inputs_unchanged') is True
                                and 'result_sha256' in data)
            json.dump(data, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
    tail = data.get('execution', {}).get('stdout', '')
    print(json.dumps({k: data.get(k) for k in ('errors', 'id', 'exit_state', 'source_unchanged',
                                               'existing_state_unchanged', 'accepted')}, ensure_ascii=False))
    print(tail[-3000:])
    return 0 if data['accepted'] else 5


if __name__ == '__main__':
    sys.exit(main(sys.argv[1], Path(sys.argv[2])))
