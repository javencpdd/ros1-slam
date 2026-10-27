"""V8-10 次切片：运动学上限核验＋失效撤销（R654）。

运动学核验（消费 contracts robot.v1，22 号「运动学上限」核验位）：
  台阶 E_TRAJ_STEP——相邻点 |dz| 对 capability.max_step_height_m；
  坡度 E_TRAJ_SLOPE——相邻点坡角对 capability.max_slope_deg；
  制动 E_TRAJ_DECEL——带时戳轨迹的减速度对 brake.max_decel_mps2。
  ★铁则面：所需能力 level=declared 且 composition_use=prohibited 时**拒绝判定**
  （E_TRAJ_CAPABILITY_UNVERIFIED）——未验证值不得用于组合/判定（22 号），检查器
  不得拿声明值冒充已核验（fail-closed，宁可拒判不可假绿）。

失效撤销（A5「持续被拒轨迹不算规划成功」的编排闭环）：
  on_rejection(verdict)→停止链动作（联动 V8-06 Orchestrator）；
  RejectionTracker——连续被拒 N 次后强制停（不得无限重规划）。
"""
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'contracts'))
import validate as contract  # noqa: E402


class EnforcementError(Exception):
    def __init__(self, code, detail):
        super().__init__(code + ': ' + detail)
        self.code = code
        self.detail = detail


def _validated_robot(profile):
    try:
        contract.validate('robot', profile)
    except contract.ContractError as error:
        raise EnforcementError('E_ENF_ROBOT_INVALID', '%s: %s' % (error.code, error.detail))
    return profile


def _usable(capability_block, name):
    """铁则：declared+prohibited 的能力值不得用于判定；tested（有 report）可用。"""
    if capability_block['level'] == 'declared' and capability_block['composition_use'] == 'prohibited':
        raise EnforcementError('E_TRAJ_CAPABILITY_UNVERIFIED',
                               '%s 为 declared+prohibited，未验证值不得用于判定（22 号铁则）' % name)
    return capability_block.get('value', capability_block.get('max_decel_mps2'))


def check_kinematics(trajectory, robot_profile, slope_tol_deg=1e-6):
    """时戳轨迹 [(x,y,z[,t]),...] 对 robot.v1 能力/制动上限核验；返回违例列表。"""
    _validated_robot(robot_profile)
    if len(trajectory) < 2:
        raise EnforcementError('E_ENF_TRAJ_TOO_SHORT', '运动学核验至少两个点')
    if any(len(p) < 4 for p in trajectory):
        raise EnforcementError('E_ENF_NEED_TIMESTAMPS', '运动学核验需带时戳轨迹')
    max_step = _usable(robot_profile['capability']['max_step_height_m'], 'max_step_height_m')
    max_slope = _usable(robot_profile['capability']['max_slope_deg'], 'max_slope_deg')
    max_decel = _usable(robot_profile['brake'], 'brake.max_decel_mps2')

    violations = []
    prev_v = None
    for index in range(1, len(trajectory)):
        a, b = trajectory[index - 1], trajectory[index]
        dz = b[2] - a[2]
        dxy = math.hypot(b[0] - a[0], b[1] - a[1])
        dt = b[3] - a[3]
        if dt <= 0:
            violations.append(('E_TRAJ_TIME', index, '时戳非单调'))
            continue
        if abs(dz) > max_step + 1e-9:
            violations.append(('E_TRAJ_STEP', index,
                               '|dz|=%.3f > max_step_height_m=%.3f' % (abs(dz), max_step)))
        if dxy > 1e-9:
            slope = math.degrees(math.atan2(abs(dz), dxy))
            if slope > max_slope + slope_tol_deg:
                violations.append(('E_TRAJ_SLOPE', index,
                                   '坡角 %.1f° > max_slope_deg=%.1f°' % (slope, max_slope)))
        v = math.hypot(dxy, dz) / dt
        if prev_v is not None:
            decel = (prev_v - v) / dt
            if decel > max_decel + 1e-9:
                violations.append(('E_TRAJ_DECEL', index,
                                   '减速度 %.2f > brake.max_decel_mps2=%.2f' % (decel, max_decel)))
        prev_v = v
    return violations


# -- 失效撤销（A5 编排闭环） --

MAX_REJECTIONS = 3  # 连续被拒上限：达限强制停，不无限重规划


def on_rejection(verdict):
    """检查器否决→编排动作（V8-06 Orchestrator 消费）。verdict＝check() 结果。"""
    if verdict.get('ok'):
        raise EnforcementError('E_ENF_NOT_REJECTED', '通过轨迹无撤销动作')
    codes = sorted({v[0] for v in verdict['violations']})
    return {'action': 'stop_chain', 'reason': 'rejected:%s' % ','.join(codes),
            'task_outcome': 'FAILED'}


class RejectionTracker:
    """连续被拒计数：未达限→replan（重规划）；达限→stop（强制停车，A5）。"""

    def __init__(self, limit=MAX_REJECTIONS):
        self.limit = limit
        self.consecutive = 0

    def record(self, verdict):
        if verdict.get('ok'):
            self.consecutive = 0
            return 'accept'
        self.consecutive += 1
        return 'stop' if self.consecutive >= self.limit else 'replan'

    def reset(self):
        self.consecutive = 0
