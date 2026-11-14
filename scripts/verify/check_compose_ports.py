#!/usr/bin/env python3
"""One owner for the simulation host ports; sim is headless."""
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
compose = yaml.safe_load((ROOT / 'docker' / 'docker-compose.yml').read_text(encoding='utf-8'))
owners = {}
for name, service in compose.get('services', {}).items():
    for port in service.get('ports', []) or []:
        owners.setdefault(str(port), []).append(name)
failures = []
for port in ('11313:11313', '11345:11345'):
    if owners.get(port) != ['sim-viz']:
        failures.append('%s owners=%r' % (port, owners.get(port)))
if failures:
    print('FAIL  ' + '; '.join(failures))
    sys.exit(1)
print('PASS  11313/11345 owned by sim-viz only; sim headless')
