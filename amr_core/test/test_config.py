import pytest


pytestmark = [pytest.mark.hardware, pytest.mark.config]


@pytest.mark.parametrize('service', ['ratioDrive', 'stop', 'setSafeDrive', 'getSafeDrive', 'updateNumbers'])
def test_required_drive_service_is_advertised(authenticated_robot, service):
    result = authenticated_robot('connectivity')
    assert result.returncode == 0, 'Cannot inspect services without authentication.'
    services = {line.removeprefix('SERVICE ') for line in result.stdout.splitlines() if line.startswith('SERVICE ')}
    assert service in services, f'Required service is not advertised: {service}'


def test_drive_mode_and_stationary_telemetry(authenticated_robot):
    result = authenticated_robot('config')
    assert result.returncode == 0, 'Read-only mode/telemetry checks failed; inspect sanitized captured output.'
    assert any(line in ('SAFE_DRIVE 0', 'SAFE_DRIVE 1') for line in result.stdout.splitlines())
    assert 'TELEMETRY ' in result.stdout
    assert 'motion_observed=0 malformed=0 transport_fault=0' in result.stdout


def test_live_configuration_snapshot_is_complete(robot_capture):
    snapshot = robot_capture['snapshot']
    assert robot_capture['probe_returncode'] == 0, snapshot.get('failure_reason', 'Read-only configuration retrieval failed.')
    assert snapshot['read_only'] is True
    assert snapshot['complete'] is True
    assert snapshot['config_service'].startswith('getConfig')
    assert snapshot['parameters'], 'No allowlisted drive parameters returned; do not infer archived defaults.'