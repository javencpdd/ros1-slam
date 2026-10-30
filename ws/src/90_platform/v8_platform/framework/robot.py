"""Robot profile loading and command/telemetry port contract (F5)."""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'contracts'))
import validate as contract  # noqa: E402

from config import FrameworkError  # noqa: E402

COMMAND_TYPE = 'geometry_msgs/TwistStamped'
TELEMETRY_TYPES = {'odom': 'nav_msgs/Odometry', 'imu': 'sensor_msgs/Imu'}
CAPABILITY = ('max_slope_deg', 'max_step_height_m', 'min_clearance_m')


def load_profile(path):
    path = Path(path)
    if not path.is_file():
        raise FrameworkError('E_ROBOT_MISSING', str(path))
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
        contract.validate('robot', value)
    except contract.ContractError as error:
        raise FrameworkError('E_ROBOT_CONTRACT', '%s: %s: %s'
                             % (path.name, error.code, error.detail))
    return value


def limits(profile):
    """Only composition_use=allowed values may drive planning; declared stays out."""
    result = {}
    for name in CAPABILITY:
        item = profile['capability'][name]
        if item['composition_use'] != 'allowed':
            raise FrameworkError('E_ROBOT_UNVERIFIED',
                                 '%s %s is %s/%s' % (profile['robot_id'], name,
                                                     item['level'], item['composition_use']))
        result[name] = item['value']
    brake = profile['brake']
    if brake['composition_use'] != 'allowed':
        raise FrameworkError('E_ROBOT_UNVERIFIED',
                             '%s brake is %s/%s' % (profile['robot_id'],
                                                    brake['level'], brake['composition_use']))
    result['max_decel_mps2'] = brake['max_decel_mps2']
    result['latency_s'] = brake['latency_s']
    return result


def adapter_ports(component):
    """Check a robot_adapter component has one command input and telemetry outputs."""
    if 'robot_adapter' not in component['roles']:
        raise FrameworkError('E_ROBOT_ROLE', '%s is not robot_adapter' % component['id'])
    inputs = {port['name']: port for port in component['inputs']}
    command = inputs.get('cmd')
    if command is None or command['type'] != COMMAND_TYPE:
        raise FrameworkError('E_ROBOT_PORTS', '%s needs cmd %s' % (component['id'], COMMAND_TYPE))
    outputs = {port['name']: port for port in component['outputs']}
    telemetry = {}
    for name, msg_type in TELEMETRY_TYPES.items():
        if name in outputs:
            if outputs[name]['type'] != msg_type:
                raise FrameworkError('E_ROBOT_PORTS', '%s.%s must be %s'
                                     % (component['id'], name, msg_type))
            telemetry[name] = msg_type
    if not telemetry:
        raise FrameworkError('E_ROBOT_PORTS', '%s needs odom or imu telemetry' % component['id'])
    return {'command': {'name': 'cmd', 'type': COMMAND_TYPE}, 'telemetry': telemetry}
