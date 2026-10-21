import sys
import tempfile

import yaml

sys.path.insert(0, '/home/ros1-slam/ws/src/00_drivers/lite3_drivers/scripts')
import sim_adapter as SA

d = {'waypoints': [[0, 0], [1, 0]], 'speed': 0.5}     # 无三键 yaml（R1 形态）→ 缺省值须=现行为
f = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
yaml.safe_dump(d, f)
f.close()
t = SA.load_traj(f.name)
print('缺省 turn_omega=%.1f v_ramp_s=%.1f align_gate=%.2f（=现行为兼容）' % (t['turn_omega'], t['v_ramp_s'], t['align_gate']))
