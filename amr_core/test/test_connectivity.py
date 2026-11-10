import socket

import pytest


pytestmark = [pytest.mark.hardware, pytest.mark.connectivity]


def test_tcp_endpoint_is_reachable(robot_settings):
    try:
        with socket.create_connection((robot_settings['host'], robot_settings['port']), timeout=5.0):
            pass
    except OSError as error:
        pytest.fail(f'Robot TCP endpoint is not reachable: {type(error).__name__}', pytrace=False)


def test_arnetworking_authentication(robot_probe):
    result = robot_probe('connectivity')
    assert result.returncode == 0, 'ARNetworking authentication failed; inspect sanitized captured output.'
    assert 'AUTHENTICATION_OK' in result.stdout