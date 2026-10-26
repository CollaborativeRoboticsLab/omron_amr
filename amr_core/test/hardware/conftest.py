from datetime import datetime, timezone
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import subprocess

import pytest
import yaml


HARDWARE = Path(__file__).resolve().parent
PACKAGE = HARDWARE.parents[1]
WORKSPACE = PACKAGE.parents[2]


def ratio_mapping_assessment(snapshot, driver=None):
    assessment = {
        'ready_for_motion': False,
        'firmware_contract_confirmed': False,
        'candidate_model': 'bundled_aria_ratio_v1',
        'blockers': [],
    }
    if snapshot.get('complete') is not True or snapshot.get('read_only') is not True:
        assessment['blockers'].append('A complete read-only configuration capture is required.')
        return assessment
    values = {}
    for parameter in snapshot.get('parameters', []):
        key = (parameter.get('section'), parameter.get('name'))
        if key in values:
            assessment['blockers'].append('Duplicate configuration parameter identity.')
            return assessment
        values[key] = parameter.get('value')

    def numeric(section, name):
        value = values[(section, name)]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError()
        return value

    try:
        forward_limit = numeric('Robot config', 'TransVelMax')
        reverse_limit = numeric('Robot config', 'TransNegVelMax')
        rotation_limit = numeric('Robot config', 'RotVelMax')
        forward = numeric('Teleop Settings', 'FullThrottleForwards')
        reverse = numeric('Teleop Settings', 'FullThrottleBackwards')
        stopped = numeric('Teleop Settings', 'RotAtStopped')
        moving_forward = numeric('Teleop Settings', 'RotAtFullForwards')
        moving_reverse = numeric('Teleop Settings', 'RotAtFullBackwards')
        translation_dead_zone = numeric('Teleop Settings', 'TransDeadZone')
        rotation_dead_zone = numeric('Teleop Settings', 'RotDeadZone')
        if (forward_limit <= 0 or reverse_limit >= 0 or rotation_limit <= 0 or
                any(value < 0 for value in (forward, reverse, stopped, moving_forward, moving_reverse)) or
                not 0 <= translation_dead_zone < 100 or not 0 <= rotation_dead_zone < 100):
            raise ValueError()
        candidate = {
            'full_throttle_forward_mm_s': forward if forward >= 1 else forward_limit,
            'full_throttle_reverse_mm_s': reverse if reverse >= 1 else forward_limit / 4,
            'rotation_stopped_deg_s': stopped if stopped >= 1 else rotation_limit,
            'rotation_full_forward_deg_s': moving_forward if moving_forward >= 1 else rotation_limit / 2,
            'rotation_full_reverse_deg_s': moving_reverse if moving_reverse >= 1 else rotation_limit / 2,
            'translation_dead_zone_pct': translation_dead_zone,
            'rotation_dead_zone_pct': rotation_dead_zone,
        }
        assessment['candidate_calibration'] = candidate
        assessment['reported_platform_limits'] = {
            'forward_mm_s': forward_limit, 'reverse_mm_s': reverse_limit, 'rotation_deg_s': rotation_limit,
        }
        assessment['candidate_translation_dead_zone_mps'] = {
            'forward': candidate['full_throttle_forward_mm_s'] * translation_dead_zone / 100000,
            'reverse': candidate['full_throttle_reverse_mm_s'] * translation_dead_zone / 100000,
        }
        teleop = ('TeleopTransVelMax', 'TeleopTransNegVelMax', 'TeleopTransAccel', 'TeleopTransDecel',
                  'TeleopRotVelMax', 'TeleopRotAccel', 'TeleopRotDecel')
        assessment['teleop_zero_sentinels'] = [name for name in teleop
                                              if numeric('Teleop Settings', name) == 0]
    except (KeyError, TypeError, ValueError, OverflowError):
        assessment.pop('candidate_calibration', None)
        assessment['blockers'].append('Required numeric drive settings are missing or invalid.')
        return assessment

    if driver is not None:
        try:
            minimum = driver['min_linear_speed']
            maximum = driver['max_linear_speed']
            if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                   for value in (minimum, maximum)) or minimum > 0 or maximum <= 0:
                raise ValueError()
            if minimum < reverse_limit:
                assessment['blockers'].append('Local reverse limit exceeds the reported robot reverse limit.')
            if maximum > forward_limit:
                assessment['blockers'].append('Local forward limit exceeds the reported robot forward limit.')
            if driver.get('ratio_mapping') == 'legacy':
                cutoff = max(abs(minimum), maximum) * translation_dead_zone / 100000
                assessment['legacy_translation_dead_zone_mps'] = cutoff
                assessment['legacy_reverse_representable'] = abs(minimum) / 1000 > cutoff
                if minimum < 0 and not assessment['legacy_reverse_representable']:
                    assessment['blockers'].append('Every permitted legacy reverse command is inside the dead zone.')
                assessment['blockers'].append('Legacy mapping is not calibrated SI velocity control.')
        except (KeyError, TypeError, ValueError):
            assessment['blockers'].append('Local driver limits are missing or invalid.')

    assessment['blockers'].extend([
        'Candidate fallback rules and teleop sentinel inheritance require installed-firmware confirmation.',
        'Nonzero firmware dead zones cannot represent arbitrary low-speed Nav2 targets without changing the target.'
        if translation_dead_zone > 0 or rotation_dead_zone > 0 else
        'Zero dead zones do not independently confirm the firmware command contract.',
        'Control ownership, watchdog behavior, protective functions, and physical tracking remain unverified.',
    ])
    return assessment


def pytest_addoption(parser):
    group = parser.getgroup('robot hardware')
    group.addoption('--robot', action='store_true', help='Allow explicit live robot connectivity/config tests.')
    group.addoption('--robot-params', default=str(PACKAGE.parent / 'amr_ros/config/ld250_parameters.yaml'))
    group.addoption('--robot-probe', default=str(WORKSPACE / 'build/amr_core/amr_hardware_probe'))
    group.addoption('--robot-capture', default=str(WORKSPACE / 'build/amr_core/hardware-config.json'))


def pytest_configure(config):
    for marker, description in (
        ('hardware', 'Explicitly opted-in live robot checks; never part of default colcon tests'),
        ('connectivity', 'TCP reachability and ARNetworking authentication'),
        ('config', 'Read-only advertised services, drive mode, and stationary telemetry'),
        ('movement', 'Supervised physical motion acceptance; not implemented yet'),
    ):
        config.addinivalue_line('markers', f'{marker}: {description}')


def pytest_collection_modifyitems(config, items):
    if not config.getoption('--robot'):
        for item in items:
            if Path(item.path).is_relative_to(HARDWARE):
                item.add_marker(pytest.mark.skip(reason='Live robot tests require --robot'))


@pytest.fixture(scope='session')
def robot_settings(pytestconfig):
    path = Path(pytestconfig.getoption('--robot-params'))
    try:
        parameters = yaml.safe_load(path.read_text())['amr_core']['ros__parameters']
        robot = parameters['robot']
        host = robot['ip']
        port = int(robot['port'])
        user = robot['user']
        password = robot['password']
        protocol = robot.get('protocol', '6MTX')
        if not 1 <= port <= 65535 or not isinstance(host, str) or not host:
            raise ValueError()
        if not all(isinstance(value, str) and '\n' not in value and '\r' not in value
                   for value in (host, user, password, protocol)):
            raise ValueError()
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError):
        pytest.fail('Cannot load robot connection settings; check --robot-params (values withheld).', pytrace=False)
    return {'host': host, 'port': port, 'user': user, 'password': password, 'protocol': protocol,
            'driver': parameters.get('driver')}


@pytest.fixture(scope='session')
def robot_probe(pytestconfig, robot_settings):
    executable = Path(pytestconfig.getoption('--robot-probe')).resolve()
    if not executable.is_file():
        pytest.fail('Build amr_core with BUILD_TESTING=ON, or specify --robot-probe.', pytrace=False)

    @lru_cache(maxsize=3)
    def run(category):
        if category not in ('connectivity', 'config', 'config-capture'):
            pytest.fail('Read-only probe does not permit movement.', pytrace=False)
        connection = '\n'.join(str(robot_settings[key]) for key in ('host', 'port', 'user', 'password', 'protocol')) + '\n'
        try:
            result = subprocess.run([str(executable), category], input=connection, text=True,
                                    capture_output=True, timeout=15, check=False)
        except subprocess.TimeoutExpired:
            pytest.fail(f'{category}: probe exceeded the 15-second deadline.', pytrace=False)
        except OSError:
            pytest.fail('Unable to execute hardware probe; check build and shared-library environment.', pytrace=False)
        print(result.stdout, end='')
        return result

    return run


@pytest.fixture(scope='session')
def authenticated_robot(robot_probe):
    if robot_probe('connectivity').returncode != 0:
        pytest.fail('Config checks blocked by ARNetworking authentication failure.', pytrace=False)
    return robot_probe


@pytest.fixture(scope='session')
def robot_capture(authenticated_robot, robot_settings, pytestconfig):
    result = authenticated_robot('config-capture')
    records = [line.removeprefix('CAPTURE ') for line in result.stdout.splitlines() if line.startswith('CAPTURE ')]
    if len(records) != 1:
        pytest.fail('Probe did not return exactly one sanitized configuration snapshot.', pytrace=False)
    try:
        snapshot = json.loads(records[0])
    except json.JSONDecodeError:
        pytest.fail('Probe returned invalid snapshot JSON.', pytrace=False)
    canonical = json.dumps(snapshot, sort_keys=True, separators=(',', ':')).encode()
    document = {
        'captured_at_utc': datetime.now(timezone.utc).isoformat(),
        'endpoint': f"{robot_settings['host']}:{robot_settings['port']}",
        'protocol': robot_settings['protocol'],
        'probe_returncode': result.returncode,
        'snapshot_sha256': hashlib.sha256(canonical).hexdigest(),
        'snapshot': snapshot,
        'ratio_mapping_assessment': ratio_mapping_assessment(snapshot, robot_settings.get('driver')),
    }
    path = Path(pytestconfig.getoption('--robot-capture'))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2, sort_keys=True) + '\n')
    print(f'Sanitized configuration snapshot: {path}')
    return document