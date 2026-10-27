"""flat2d 定标集预登记生成器：运行前写出每个 run 的 manifest。

用法：python3 pre_register.py   （在 flat2d 目录下）
生成 runs/<run_id>.json 与 runs/index.sha256；已存在则拒绝覆盖（预登记不可事后改）。
"""
import datetime
import hashlib
import json
import pathlib
import sys

BASE = pathlib.Path(__file__).resolve().parent
SET_PATH = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else BASE / 'scenarios.json'
SET = json.loads(SET_PATH.read_text())
NOW = datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds')
RUNS = pathlib.Path(sys.argv[2]) if len(sys.argv) > 2 else BASE / 'runs'
RUNS.mkdir(exist_ok=True)

created = []
for scenario in SET['scenarios']:
    for group, goal in enumerate(scenario['goals']):
        for rep in range(1, SET['repetitions'] + 1):
            run_id = '%s_%s_g%d_r%d' % (SET['set_id'], scenario['id'], group, rep)
            path = RUNS / ('%s.json' % run_id)
            if path.exists():
                print('SKIP existing', run_id)
                continue
            manifest = {
                'run_id': run_id,
                'run_class': 'calibration',
                'calibration_only': True,
                'set_id': SET['set_id'],
                'scenario_id': scenario['id'],
                'scenario_name': scenario['name'],
                'scenario_semantics': '24§3',
                'expected_outcome': scenario['expected_outcome'],
                'seed': group,
                'repetition': rep,
                'track': 'truth',
                'combo': SET['combo'],
                'start': SET['start'],
                'goal': goal,
                'world': SET['world'],
                'map': SET['map'],
                'config_lock': SET['config_lock'],
                'criteria': SET.get('criteria', {'goal_position_m': 0.30, 'min_clearance_m': 0.20}),
                'evaluator': 'R665 a1_driver.py, goal parameterized (GT trajectory vs known boxes)',
                'pre_registered_at': NOW,
                'status': 'pre_registered',
                'counts_planned': {'N_total': 1, 'N_valid': 1, 'N_invalid': 0,
                                   'N_pass': None, 'N_fail': None},
            }
            path.write_text(json.dumps(manifest, ensure_ascii=False, indent=1) + '\n')
            created.append(path.name)

index = []
for path in sorted(RUNS.glob('*.json')):
    index.append('%s  %s' % (hashlib.sha256(path.read_bytes()).hexdigest(), path.name))
if index:
    (RUNS / 'index.sha256').write_text('\n'.join(index) + '\n')
print('created', len(created), 'manifests; total', len(index))
