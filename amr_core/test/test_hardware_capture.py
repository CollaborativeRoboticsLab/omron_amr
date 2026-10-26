import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


HARDWARE = Path(__file__).resolve().parent / 'hardware'


def load_module(name, filename):
    spec = importlib.util.spec_from_file_location(name, HARDWARE / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('port', [7272, 0])
def test_robot_settings_returns_valid_connection_and_driver_or_fails_closed(tmp_path, port):
    fixtures = load_module('hardware_settings_fixtures', 'conftest.py')
    robot = {'ip': 'offline', 'port': port, 'user': 'test-user', 'password': 'test-only'}
    driver = {'min_linear_speed': -100, 'max_linear_speed': 1000, 'ratio_mapping': 'legacy'}
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
    fixtures = load_module('hardware_capture_fixtures', 'conftest.py')
    fixtures.pytest_configure(pytestconfig)
    checks = load_module('hardware_capture_checks', 'test_config.py')
    snapshot = {
        'schema_version': 1, 'read_only': True, 'complete': returncode == 0,
        'parameters': [{'section': 'Teleop', 'name': 'FullThrottleForwards', 'value': 1200}],
        'config_service': 'getConfig', 'failure_reason': 'Unsupported configuration export',
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
    fixtures = load_module('hardware_auth_fixtures', 'conftest.py')
    probe = lambda category: SimpleNamespace(returncode=returncode)
    if returncode == 0:
        assert fixtures.authenticated_robot.__wrapped__(probe) is probe
    else:
        with pytest.raises(pytest.fail.Exception, match='authentication failure'):
            fixtures.authenticated_robot.__wrapped__(probe)


def test_readonly_probe_rejects_movement_before_subprocess(tmp_path, monkeypatch):
    fixtures = load_module('hardware_readonly_fixtures', 'conftest.py')
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
    fixtures = load_module('mapping_assessment_fixtures', 'conftest.py')
    snapshot = mapping_snapshot()
    original = json.dumps(snapshot, sort_keys=True)
    assessment = fixtures.ratio_mapping_assessment(
        snapshot, {'min_linear_speed': -100, 'max_linear_speed': 1200, 'ratio_mapping': 'legacy'})
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
    fixtures = load_module('invalid_mapping_assessment_fixtures', 'conftest.py')
    snapshot = mapping_snapshot()
    snapshot['parameters'][0]['value'] = invalid
    assessment = fixtures.ratio_mapping_assessment(snapshot)
    assert assessment['ready_for_motion'] is False
    assert 'candidate_calibration' not in assessment
    assert assessment['blockers']


def test_mapping_assessment_does_not_use_failed_capture():
    fixtures = load_module('failed_mapping_assessment_fixtures', 'conftest.py')
    snapshot = mapping_snapshot()
    snapshot['complete'] = False
    assessment = fixtures.ratio_mapping_assessment(snapshot)
    assert assessment['ready_for_motion'] is False
    assert 'candidate_calibration' not in assessment


def calibration_module():
    path = HARDWARE.parents[1] / 'calibration/deadzones.py'
    spec = importlib.util.spec_from_file_location('deadzone_calibration', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('scenario', ['success', 'failed', 'timeout', 'bad-result', 'wrong-readback'])
def test_calibration_write_is_once_audited_and_independently_verified(tmp_path, monkeypatch, capsys, scenario):
    calibration = calibration_module()
    destination = tmp_path / 'write.json'
    calls = []
    secret = 'test-only-private-value'

    def execute(command, **kwargs):
        calls.append(command)
        assert destination.is_file()
        assert json.loads(destination.read_text())['state'] == 'pending'
        assert secret not in ' '.join(command)
        assert secret in kwargs['input']
        assert kwargs['env']['AMR_APPLY_TRANSLATION_DEADZONE'] == 'CONFIRMED'
        assert kwargs['input'].endswith('10\n14\n5\nAPPLY_TRANSLATION_DEADZONE\n')
        if scenario == 'timeout':
            raise calibration.subprocess.TimeoutExpired(command, 45)
        evidence = {'complete': scenario != 'failed', 'write_attempted': True}
        payload = 'invalid' if scenario == 'bad-result' else json.dumps(evidence)
        return SimpleNamespace(returncode=1 if scenario == 'failed' else 0,
                               stdout='CALIBRATION_RESULT ' + payload + '\n')

    monkeypatch.setattr(calibration.subprocess, 'run', execute)
    monkeypatch.setattr(calibration, 'capture', lambda *args: {
        'TransDeadZone': 10 if scenario == 'wrong-readback' else 14, 'RotDeadZone': 5,
    })
    result = calibration.apply_translation(Path('offline-binary'), Path('offline-probe'),
                                           'offline\n7272\nuser\n' + secret + '\n6MTX\n',
                                           10, 14, 5, destination)
    assert result == (0 if scenario == 'success' else 1)
    assert len(calls) == 1
    saved = destination.read_text()
    report = json.loads(saved)
    assert report['complete'] is (scenario == 'success')
    assert report['ready_for_motion'] is False
    assert report['persistence_verified'] is False
    assert 'independent_readback' in report
    assert secret not in saved + capsys.readouterr().out


def test_calibration_cli_rejects_unconfirmed_write_before_connection(tmp_path, monkeypatch):
    calibration = calibration_module()
    monkeypatch.setattr('sys.argv', ['deadzones.py', '--robot', '--expected-translation', '10',
                                    '--expected-rotation', '5', '--apply-translation', '14',
                                    '--output', str(tmp_path / 'unused.json')])
    with pytest.raises(SystemExit) as error:
        calibration.main()
    assert error.value.code == 2
    assert not (tmp_path / 'unused.json').exists()


def test_deadzone_observer_detects_small_motion_without_claiming_no_creep():
    calibration = calibration_module()
    first = {'x_mm': 0, 'y_mm': 0, 'heading_deg': 179,
             'forward_mm_s': 0, 'angular_deg_s': 0, 'lateral_mm_s': 0}
    stationary = calibration.summarize([first, dict(first)])
    assert stationary['motion_evidence'] is False
    assert stationary['no_creep_proven'] is False
    moved = dict(first, x_mm=1, heading_deg=-179, forward_mm_s=1)
    summary = calibration.summarize([first, moved])
    assert summary['motion_evidence'] is True
    assert summary['max_position_change_mm'] == 1
    assert summary['max_heading_change_deg'] == 2
    assert summary['peak_reported_speeds']['forward_mm_s'] == 1


def test_deadzone_observer_checks_complete_readback_and_rejects_writes():
    calibration = calibration_module()
    snapshot = mapping_snapshot()
    assert calibration.deadzones(snapshot) == {'TransDeadZone': 10, 'RotDeadZone': 5}
    snapshot['complete'] = False
    with pytest.raises(ValueError, match='complete read-only'):
        calibration.deadzones(snapshot)
    with pytest.raises(ValueError, match='read-only'):
        calibration.run_probe('/not-executed', 'setConfigValues', 'not-a-connection')
    with pytest.raises(ValueError, match='Insufficient'):
        calibration.summarize([])


@pytest.mark.parametrize('mismatch', [None, 'before', 'after'])
def test_deadzone_trial_checks_both_readbacks_and_saves_result(tmp_path, monkeypatch, capsys, mismatch):
    calibration = calibration_module()
    parameters = tmp_path / 'parameters.yaml'
    parameters.write_text(calibration.yaml.safe_dump({'amr_core': {'ros__parameters': {'robot': {
        'ip': 'offline', 'port': 7272, 'user': 'test-user', 'password': 'test-only-secret',
    }}}}))
    destination = tmp_path / 'trial.json'
    monkeypatch.setattr('sys.argv', ['deadzones.py', '--robot', '--params', str(parameters),
                                    '--expected-translation', '5', '--expected-rotation', '5',
                                    '--output', str(destination)])
    calls = []

    def capture(executable, connection):
        stage = 'before' if not calls else 'after'
        calls.append(stage)
        return {'TransDeadZone': 3 if stage == mismatch else 5, 'RotDeadZone': 5}

    def observe(executable, category, connection):
        assert category == 'observe'
        calls.append(category)
        sample = {'x_mm': 0, 'y_mm': 0, 'heading_deg': 0,
                  'forward_mm_s': 0, 'angular_deg_s': 0, 'lateral_mm_s': 0}
        return ('ODOMETRY ' + json.dumps(sample) + '\n') * 2

    monkeypatch.setattr(calibration, 'capture', capture)
    monkeypatch.setattr(calibration, 'run_probe', observe)
    assert calibration.main() == (0 if mismatch is None else 1)
    saved = destination.read_text()
    report = json.loads(saved)
    assert report['complete'] is (mismatch is None)
    assert report['ready_for_motion'] is False
    assert 'test-only-secret' not in saved + capsys.readouterr().out
    assert calls == (['before'] if mismatch == 'before' else ['before', 'observe', 'after'])