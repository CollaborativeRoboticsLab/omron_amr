import hashlib
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
from types import SimpleNamespace

import pytest
import yaml


HARDWARE = Path(__file__).resolve().parent


def load_module(name, filename):
    spec = importlib.util.spec_from_file_location(name, HARDWARE / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('port', [7272, 0])
def test_robot_settings_returns_valid_connection_and_driver_or_fails_closed(tmp_path, port):
    fixtures = load_module('hardware_settings_fixtures', 'test_configuration.py')
    robot = {'ip': 'offline', 'port': port, 'user': 'test-user', 'password': 'test-only'}
    driver = {'cmd_vel': {'min_linear': -100, 'max_linear': 1000}, 'drive': {'ratio_mapping': 'legacy'}}
    parameters = tmp_path / 'parameters.yaml'
    parameters.write_text(fixtures.yaml.safe_dump({
        'amr_core': {'ros__parameters': {'robot': robot, 'driver': driver}},
    }))
    configuration = SimpleNamespace(getoption=lambda option: str(parameters))
    if port == 0:
        with pytest.raises(pytest.fail.Exception, match='values withheld'):
            fixtures.robot_settings.__wrapped__(configuration)
    else:
        settings = fixtures.robot_settings.__wrapped__(configuration)
        assert settings == {'host': 'offline', 'port': port, 'user': 'test-user',
                            'password': 'test-only', 'protocol': '6MTX', 'driver': driver}


@pytest.mark.parametrize('returncode', [0, 1])
def test_capture_artifact_preserves_result_and_enforces_acceptance(tmp_path, returncode, pytestconfig):
    fixtures = load_module('hardware_capture_fixtures', 'test_configuration.py')
    fixtures.pytest_configure(pytestconfig)
    checks = load_module('hardware_capture_checks', 'test_config.py')
    snapshot = {
        'schema_version': 1, 'read_only': True, 'complete': returncode == 0,
        'parameters': [{'section': 'Teleop', 'name': 'FullThrottleForwards', 'value': 1200}],
        'config_service': 'getConfigValues', 'failure_reason': 'Unsupported configuration export',
    }
    destination = tmp_path / 'capture.json'
    configuration = SimpleNamespace(getoption=lambda option: str(destination))
    settings = {'host': 'offline', 'port': 1, 'protocol': 'test', 'password': 'not-for-the-report'}
    probe = lambda category: SimpleNamespace(returncode=returncode, stdout='CAPTURE ' + json.dumps(snapshot) + '\n')
    artifact = fixtures.robot_capture.__wrapped__(probe, settings, configuration)
    saved = destination.read_text()
    assert json.loads(saved) == artifact
    assert 'not-for-the-report' not in saved
    assert artifact['probe_returncode'] == returncode
    canonical = json.dumps(snapshot, sort_keys=True, separators=(',', ':')).encode()
    assert artifact['snapshot_sha256'] == hashlib.sha256(canonical).hexdigest()
    if returncode == 0:
        checks.test_live_configuration_snapshot_is_complete(artifact)
    else:
        with pytest.raises(AssertionError, match='Unsupported configuration export'):
            checks.test_live_configuration_snapshot_is_complete(artifact)


@pytest.mark.parametrize('returncode', [0, 1])
def test_config_authentication_gate_fails_closed(returncode):
    fixtures = load_module('hardware_auth_fixtures', 'test_configuration.py')
    probe = lambda category: SimpleNamespace(returncode=returncode)
    if returncode == 0:
        assert fixtures.authenticated_robot.__wrapped__(probe) is probe
    else:
        with pytest.raises(pytest.fail.Exception, match='authentication failure'):
            fixtures.authenticated_robot.__wrapped__(probe)


def test_readonly_probe_rejects_movement_before_subprocess(tmp_path, monkeypatch):
    fixtures = load_module('hardware_readonly_fixtures', 'test_configuration.py')
    executable = tmp_path / 'probe'
    executable.touch()
    configuration = SimpleNamespace(getoption=lambda option: str(executable))

    def unexpected_execution(*args, **kwargs):
        pytest.fail('Movement category must not execute a subprocess.')

    monkeypatch.setattr(fixtures.subprocess, 'run', unexpected_execution)
    probe = fixtures.robot_probe.__wrapped__(configuration, {})
    with pytest.raises(pytest.fail.Exception, match='does not permit movement'):
        probe('movement')


def mapping_snapshot():
    robot = {'TransVelMax': 1000, 'TransNegVelMax': -100, 'RotVelMax': 60}
    teleop = {
        'FullThrottleForwards': 0, 'FullThrottleBackwards': 0, 'RotAtStopped': 50,
        'RotAtFullForwards': 25, 'RotAtFullBackwards': 25, 'TransDeadZone': 10, 'RotDeadZone': 5,
        'TeleopTransVelMax': 0, 'TeleopTransNegVelMax': 0, 'TeleopTransAccel': 0, 'TeleopTransDecel': 0,
        'TeleopRotVelMax': 0, 'TeleopRotAccel': 0, 'TeleopRotDecel': 0,
    }
    return {'complete': True, 'read_only': True, 'parameters': [
        {'section': section, 'name': name, 'value': value}
        for section, parameters in (('Robot config', robot), ('Teleop Settings', teleop))
        for name, value in parameters.items()
    ]}


def test_mapping_assessment_keeps_reverse_gain_separate_from_speed_limit():
    fixtures = load_module('mapping_assessment_fixtures', 'test_configuration.py')
    snapshot = mapping_snapshot()
    original = json.dumps(snapshot, sort_keys=True)
    assessment = fixtures.ratio_mapping_assessment(
        snapshot, {'cmd_vel': {'min_linear': -100, 'max_linear': 1200}, 'drive': {'ratio_mapping': 'legacy'}})
    assert assessment['candidate_calibration']['full_throttle_forward_mm_s'] == 1000
    assert assessment['candidate_calibration']['full_throttle_reverse_mm_s'] == 250
    assert assessment['reported_platform_limits']['reverse_mm_s'] == -100
    assert assessment['candidate_translation_dead_zone_mps'] == {'forward': 0.1, 'reverse': 0.025}
    assert assessment['legacy_translation_dead_zone_mps'] == 0.12
    assert assessment['legacy_reverse_representable'] is False
    assert assessment['ready_for_motion'] is False
    assert assessment['firmware_contract_confirmed'] is False
    assert len(assessment['teleop_zero_sentinels']) == 7
    assert json.dumps(snapshot, sort_keys=True) == original


@pytest.mark.parametrize('invalid', [None, True, float('nan'), float('inf'), -1])
def test_mapping_assessment_rejects_invalid_values_without_enabling_motion(invalid):
    fixtures = load_module('invalid_mapping_assessment_fixtures', 'test_configuration.py')
    snapshot = mapping_snapshot()
    snapshot['parameters'][0]['value'] = invalid
    assessment = fixtures.ratio_mapping_assessment(snapshot)
    assert assessment['ready_for_motion'] is False
    assert 'candidate_calibration' not in assessment
    assert assessment['blockers']


def test_mapping_assessment_does_not_use_failed_capture():
    fixtures = load_module('failed_mapping_assessment_fixtures', 'test_configuration.py')
    snapshot = mapping_snapshot()
    snapshot['complete'] = False
    assessment = fixtures.ratio_mapping_assessment(snapshot)
    assert assessment['ready_for_motion'] is False
    assert 'candidate_calibration' not in assessment


@pytest.mark.parametrize('scenario', [
    'no-options', 'no-environment', 'missing-input', 'bad-token', 'bad-target',
    'same-target', 'bad-rotation', 'bad-port', 'bad-integer', 'carriage-return', 'nul',
])
def test_production_writer_rejects_unconfirmed_or_invalid_requests_before_connecting(scenario):
    binary = Path(os.environ.get('AMR_DEADZONE_WRITER',
                  Path(__file__).resolve().parents[4] / 'build/amr_core/amr_deadzone_writer'))
    if not binary.is_file():
        pytest.skip('Build amr_deadzone_writer to exercise its offline command gates.')
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        listener.listen()
        listener.setblocking(False)
        fields = ['127.0.0.1', str(listener.getsockname()[1]), 'test-user', 'test-only-private-value',
                  '6MTX', '10', '14', '5', 'APPLY_TRANSLATION_DEADZONE']
        options = ['--apply-translation', '--confirm-write']
        environment = dict(os.environ, AMR_APPLY_TRANSLATION_DEADZONE='CONFIRMED')
        if scenario == 'no-options':
            options = []
        elif scenario == 'no-environment':
            environment.pop('AMR_APPLY_TRANSLATION_DEADZONE')
        elif scenario == 'missing-input':
            fields = fields[:5]
        else:
            index, value = {
                'bad-token': (8, 'UNCONFIRMED'), 'bad-target': (6, '0'),
                'same-target': (6, '10'), 'bad-rotation': (7, '100'), 'bad-port': (1, '0'),
                'bad-integer': (6, '14extra'), 'carriage-return': (0, '127.0.0.1\r'),
                'nul': (0, '127.0.0.1\0'),
            }[scenario]
            fields[index] = value
        result = subprocess.run([str(binary), *options], input='\n'.join(fields) + '\n',
                                text=True, capture_output=True, timeout=3, env=environment)
        assert result.returncode == 2
        assert 'no connection opened' in result.stderr
        assert 'test-only-private-value' not in result.stdout + result.stderr
        with pytest.raises(BlockingIOError):
            listener.accept()


@pytest.mark.parametrize('operation,token,expected,target,unchanged', [
    ('--apply-rotation', 'APPLY_ROTATION_DEADZONE', '5', '6', '14'),
    ('--apply-timeout', 'APPLY_TELEOP_TIMEOUT', '2', '1', '5'),
])
@pytest.mark.parametrize('scenario', ['no-environment', 'bad-token', 'same-target', 'zero-target',
                                      'nonfinite', 'bad-unchanged', 'missing-confirmation'])
def test_teleop_writer_rejects_invalid_requests_before_connecting(operation, token, expected, target,
                                                                   unchanged, scenario):
    binary = Path(os.environ.get('AMR_DEADZONE_WRITER',
                  Path(__file__).resolve().parents[4] / 'build/amr_core/amr_deadzone_writer'))
    if not binary.is_file():
        pytest.skip('Build amr_deadzone_writer to exercise its offline command gates.')
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        listener.listen()
        listener.setblocking(False)
        fields = ['127.0.0.1', str(listener.getsockname()[1]), 'test-user', 'test-only-private-value',
                  '6MTX', expected, target, unchanged, token]
        options = [operation, '--confirm-write']
        environment = dict(os.environ, AMR_APPLY_TELEOP_SETTING='CONFIRMED')
        if scenario == 'no-environment':
            environment.pop('AMR_APPLY_TELEOP_SETTING')
        elif scenario == 'missing-confirmation':
            options.pop()
        else:
            index, value = {'bad-token': (8, 'UNCONFIRMED'), 'same-target': (6, expected),
                            'zero-target': (6, '0'), 'nonfinite': (6, 'nan'),
                            'bad-unchanged': (7, '100')}[scenario]
            fields[index] = value
        result = subprocess.run([str(binary), *options], input='\n'.join(fields) + '\n', text=True,
                                capture_output=True, timeout=3, env=environment)
        assert result.returncode == 2
        assert 'no connection opened' in result.stderr
        assert 'test-only-private-value' not in result.stdout + result.stderr
        with pytest.raises(BlockingIOError):
            listener.accept()


def calibration_binary():
    binary = Path(os.environ.get('AMR_DEADZONE_CALIBRATION',
                  Path(__file__).resolve().parents[4] / 'build/amr_core/amr_deadzone_calibration'))
    if not binary.is_file():
        pytest.skip('Build amr_deadzone_calibration to exercise its offline command gates.')
    return binary


def calibration_settings(path, port):
    path.write_text(yaml.safe_dump({'amr_core': {'ros__parameters': {'robot': {
        'ip': '127.0.0.1', 'port': port, 'user': 'test-user', 'password': 'test-only-private-value',
    }}}}))


@pytest.mark.parametrize('scenario', [
    'no-robot', 'unconfirmed-write', 'confirmation-without-target', 'same-target', 'fractional-write',
    'invalid-rotation', 'nan', 'existing-output', 'duplicate-option', 'unknown-option', 'bad-settings',
])
def test_cpp_calibration_rejects_invalid_requests_before_connecting(tmp_path, scenario):
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        listener.listen()
        listener.setblocking(False)
        parameters = tmp_path / 'settings.yaml'
        calibration_settings(parameters, listener.getsockname()[1])
        destination = tmp_path / 'report.json'
        command = [str(calibration_binary()), '--robot', '--params', str(parameters),
                   '--expected-translation', '14', '--expected-rotation', '5', '--output', str(destination)]
        if scenario == 'no-robot':
            command.remove('--robot')
        elif scenario == 'unconfirmed-write':
            command += ['--apply-translation', '5']
        elif scenario == 'confirmation-without-target':
            command += ['--confirm-write']
        elif scenario in ('same-target', 'fractional-write'):
            command += ['--apply-translation', '14' if scenario == 'same-target' else '3.5', '--confirm-write']
        elif scenario in ('invalid-rotation', 'nan'):
            command[command.index('--expected-rotation') + 1] = '100' if scenario == 'invalid-rotation' else 'nan'
        elif scenario == 'existing-output':
            destination.write_text('preserve-existing-report')
        elif scenario == 'duplicate-option':
            command += ['--robot']
        elif scenario == 'unknown-option':
            command += ['--writer', 'removed-option']
        else:
            parameters.write_text('robot: [invalid yaml')
        result = subprocess.run(command, text=True, capture_output=True, timeout=3)
        assert result.returncode == 2
        assert 'no connection opened' in result.stderr
        assert 'test-only-private-value' not in result.stdout + result.stderr
        if scenario == 'existing-output':
            assert destination.read_text() == 'preserve-existing-report'
        else:
            assert not destination.exists()
        with pytest.raises(BlockingIOError):
            listener.accept()


@pytest.mark.parametrize('write', [False, True])
def test_cpp_calibration_audits_connection_failure_without_leaking_credentials(tmp_path, write):
    with socket.socket() as reserved:
        reserved.bind(('127.0.0.1', 0))
        parameters = tmp_path / 'settings.yaml'
        calibration_settings(parameters, reserved.getsockname()[1])
        destination = tmp_path / 'report.json'
        command = [str(calibration_binary()), '--robot', '--params', str(parameters),
                   '--expected-translation', '14', '--expected-rotation', '5', '--output', str(destination)]
        if write:
            command += ['--apply-translation', '5', '--confirm-write']
        result = subprocess.run(command, text=True, capture_output=True, timeout=10)
    assert result.returncode == 1
    saved = destination.read_text()
    report = json.loads(saved)
    assert report['complete'] is False
    assert report['ready_for_motion'] is False
    assert 'finished_at_utc' in report
    assert destination.stat().st_mode & 0o777 == 0o600
    assert 'test-only-private-value' not in saved + result.stdout + result.stderr
    if write:
        assert report['state'] == 'failed_or_uncertain'
        assert report['write_attempted'] is None
        assert report['persistence_verified'] is False
        assert 'readback_failure' in report
    else:
        assert report['read_only'] is True
        assert report['samples'] == []