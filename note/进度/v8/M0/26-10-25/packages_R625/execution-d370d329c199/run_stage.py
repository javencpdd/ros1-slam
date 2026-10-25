import hashlib
import json
from pathlib import Path
import subprocess
import sys
import uuid
from datetime import datetime, timezone
import probe

HERE = Path(__file__).resolve().parent
PRIVATE = Path('/home/jack/temp/ros1-slam-r625')
DEST = '/opt/ros1-slam/r625'
LABEL = 'org.ros1-slam.r625-stage'


def source_hashes():
    return {str(p.relative_to(PRIVATE / 'ws/src/lite3_costmap_layers')):
            hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted((PRIVATE / 'ws/src/lite3_costmap_layers').rglob('*')) if p.is_file()}


def main(stage, output):
    if stage not in ('build', 'runtime'):
        raise ValueError('unknown stage')
    with output.open('x') as stream:
        subprocess.run(['setfacl', '-m', 'u:ros1-slam:rwx', str(output)], check=True)
        token = uuid.uuid4().hex[:12]
        name = 'ros1-slam-r625-' + stage + '-' + token
        data = {'stage': stage, 'started_at': datetime.now(timezone.utc).isoformat(),
                'name': name, 'token': token, 'errors': [],
                'driver_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
        cid = None
        attempted = False
        try:
            data['before'] = probe.state()
            data['input_before'] = probe.fingerprint()
            data['source_before'] = source_hashes()
            assert data['source_before'] == json.loads((PRIVATE / 'source-manifest.json').read_text())['files']
            snapshot = HERE / ('execution-' + token)
            snapshot.mkdir()
            subprocess.run(['setfacl', '-m', 'u:ros1-slam:rwx,d:u:ros1-slam:rwx', str(snapshot)], check=True)
            names = ['run_stage.py', 'probe.py', stage + '.sh']
            if stage == 'runtime':
                names.append('runtime.py')
            for filename in names:
                (snapshot / filename).write_bytes((HERE / filename).read_bytes())
            data['execution_snapshot'] = snapshot.name
            data['execution_inputs_before'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                               for p in sorted(snapshot.iterdir())}
            data['stage_sha256'] = data['execution_inputs_before'][stage + '.sh']
            result_path = PRIVATE / ('runtime-' + token) / 'result.json' if stage == 'runtime' else PRIVATE / 'build-result.json'
            assert not result_path.exists(), 'refusing an existing stage result'
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
                        'PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 R625_RUN_TOKEN=' + token + ' '
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
            data['execution'] = probe.run(['docker', 'start', '-a', cid], timeout=600)
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
                data['input_after'] = probe.fingerprint()
                data['source_after'] = source_hashes()
                data['execution_inputs_after'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                                  for p in sorted(snapshot.iterdir())}
                data['execution_inputs_unchanged'] = data['execution_inputs_before'] == data['execution_inputs_after']
                raw_result = result_path.read_bytes()
                data['result'] = {'path': str(result_path.relative_to(PRIVATE)),
                                  'sha256': hashlib.sha256(raw_result).hexdigest()}
                if stage == 'runtime':
                    runtime_result = json.loads(raw_result)
                    assert runtime_result['run'] == DEST + '/runtime-' + token
                    assert runtime_result['accepted'] is True and runtime_result['errors'] == []
                data['existing_unchanged'] = data.get('before') == data['after']
                data['inputs_unchanged'] = data.get('input_before') == data['input_after']
                data['source_unchanged'] = data.get('source_before') == data['source_after']
                data['remaining'] = probe.checked(['docker', 'ps', '-aq', '--filter', 'label=' + LABEL + '=' + token]).split()
            except Exception as error:
                data['errors'].append('resample: ' + repr(error))
            data['finished_at'] = datetime.now(timezone.utc).isoformat()
            state = data.get('exit_state', {})
            data['stage_ok'] = (not data['errors'] and data.get('execution', {}).get('rc') == 0
                and state.get('ExitCode') == 0 and state.get('Status') == 'exited'
                and state.get('OOMKilled') is False and not state.get('Error')
                and data.get('existing_unchanged') is True and data.get('inputs_unchanged') is True
                and data.get('source_unchanged') is True and data.get('remaining') == []
                and data.get('execution_inputs_unchanged') is True
                and data.get('remove', {}).get('rc') == 0 and 'forced_stop' not in data)
            json.dump(data, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
    print(json.dumps({k: data.get(k) for k in ('stage', 'stage_ok', 'errors', 'exit_state', 'existing_unchanged', 'inputs_unchanged', 'source_unchanged', 'remaining')}, ensure_ascii=False))
    print(data.get('execution', {}).get('stdout', ''))
    print(data.get('execution', {}).get('stderr', ''))
    return 0 if data['stage_ok'] else 5


if __name__ == '__main__':
    sys.exit(main(sys.argv[1], Path(sys.argv[2])))
