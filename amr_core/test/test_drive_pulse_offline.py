import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
from types import SimpleNamespace

import pytest
import yaml


spec = importlib.util.spec_from_file_location('drive_pulse', Path(__file__).with_name('drive_pulse.py'))
drive_pulse = importlib.util.module_from_spec(spec)
spec.loader.exec_module(drive_pulse)


@pytest.mark.parametrize(('scenario', 'linear', 'angular'), [
    ('forward', 0.5, 0.0),
    ('reverse', -0.2, 0.0),
    ('left', 0.0, 0.5),
    ('right', 0.0, -0.5),
    ('forward-left', 0.5, 0.5),
    ('forward-right', 0.5, -0.5),
])
def test_scenario_streams_at_20_hz_then_stops(scenario, linear, angular):
    now = [0.0]
    events = []

    def spin(duration):
        now[0] += duration

    drive_pulse.stream_command(lambda: events.append(('command', now[0], *drive_pulse.SCENARIOS[scenario])),
                               lambda: events.append(('stop', now[0])), spin, lambda: now[0])
    commands = [event for event in events if event[0] == 'command']
    assert len(commands) == 10
    assert all(event[2:] == (linear, angular) for event in commands)
    assert all(commands[index + 1][1] - commands[index][1] == pytest.approx(0.05) for index in range(9))
    assert events[-1][0] == 'stop'
    assert events[-1][1] == pytest.approx(0.5)


def test_stream_stops_on_callback_failure():
    events = []

    def fail(_duration):
        raise RuntimeError('test failure')

    with pytest.raises(RuntimeError):
        drive_pulse.stream_command(lambda: events.append('command'), lambda: events.append('stop'), fail, lambda: 0.0)
    assert events == ['command', 'stop']


def test_two_second_stream_is_bounded():
    now = [0.0]
    events = []

    def spin(duration):
        now[0] += duration

    drive_pulse.stream_command(lambda: events.append(('command', now[0])),
                               lambda: events.append(('stop', now[0])), spin, lambda: now[0], duration=2.0)
    assert len([event for event in events if event[0] == 'command']) == 40
    assert events[-1][0] == 'stop'
    assert events[-1][1] == pytest.approx(2.0)


def test_six_second_stream_is_bounded():
    now = [0.0]
    events = []

    def spin(duration):
        now[0] += duration

    drive_pulse.stream_command(lambda: events.append(('command', now[0])),
                               lambda: events.append(('stop', now[0])), spin, lambda: now[0], duration=6.0)
    assert len([event for event in events if event[0] == 'command']) == 120
    assert events[-1][0] == 'stop'
    assert events[-1][1] == pytest.approx(6.0)


@pytest.mark.parametrize('duration', [0.0, -1.0, 6.01, float('inf'), float('nan')])
def test_invalid_duration_sends_nothing(duration):
    events = []
    with pytest.raises(ValueError):
        drive_pulse.stream_command(lambda: events.append('command'), lambda: events.append('stop'),
                                   lambda _duration: None, duration=duration)
    assert not events


def test_stalled_stream_stops_without_resuming():
    now = [0.0]
    events = []

    def stall(_duration):
        now[0] += 0.21

    with pytest.raises(drive_pulse.GuardFailure, match='command_stream_stalled'):
        drive_pulse.stream_command(lambda: events.append('command'), lambda: events.append('stop'),
                                   stall, lambda: now[0])
    assert events == ['command', 'stop']


def test_contact_dock_must_be_explicitly_confirmed():
    with pytest.raises(RuntimeError):
        drive_pulse.require_contact_dock(None)
    with pytest.raises(RuntimeError):
        drive_pulse.require_contact_dock(False)
    drive_pulse.require_contact_dock(True)


def test_command_traffic_tracks_unexpected_values():
    traffic = {'expected': None, 'last_external': 0.0}
    drive_pulse.record_command_traffic(traffic, 'forward', 1.0)
    assert traffic['last_external'] == 1.0
    traffic['expected'] = 'forward'
    drive_pulse.record_command_traffic(traffic, 'forward', 2.0)
    assert traffic['last_external'] == 1.0
    drive_pulse.record_command_traffic(traffic, 'stop', 3.0)
    assert traffic['last_external'] == 3.0


def test_target_turn_tracks_heading_across_wrap():
    turn = {'previous_heading': math.radians(175), 'angle': 0.0}
    assert not drive_pulse.track_turn(turn, math.radians(-175), 20.0)
    assert drive_pulse.track_turn(turn, math.radians(-164), 20.0)
    assert math.degrees(turn['angle']) == pytest.approx(21.0)


def test_clockwise_target_tracks_wrap_and_rejects_wrong_direction():
    turn = {'previous_heading': math.radians(-175), 'angle': 0.0}
    assert not drive_pulse.track_turn(turn, math.radians(175), 20.0, -1.0)
    assert drive_pulse.track_turn(turn, math.radians(164), 20.0, -1.0)
    assert math.degrees(turn['angle']) == pytest.approx(21.0)
    with pytest.raises(drive_pulse.GuardFailure, match='wrong_turn_direction'):
        drive_pulse.track_turn({'previous_heading': 0.0, 'angle': 0.0}, math.radians(11), 20.0, -1.0)


@pytest.mark.parametrize(('heading', 'reason'), [(math.radians(-11), 'wrong_turn_direction'),
                                                 (math.radians(21), 'heading_jump')])
def test_target_turn_rejects_bad_heading(heading, reason):
    with pytest.raises(drive_pulse.GuardFailure, match=reason):
        drive_pulse.track_turn({'previous_heading': 0.0, 'angle': 0.0}, heading, 100.0)


def test_target_turn_timeout_requests_stop():
    now = [0.0]
    events = []

    def spin(duration):
        now[0] += duration
        return False

    with pytest.raises(drive_pulse.GuardFailure, match='turn_timeout'):
        drive_pulse.stream_turn(lambda: events.append('command'), lambda: events.append('stop'),
                                spin, lambda: now[0])
    assert events[-1] == 'stop'
    assert len(events) <= 241


@pytest.mark.parametrize(('scenario', 'duration'), [('reverse', '0.5'), ('forward', '1.0')])
def test_existing_driver_rejects_other_motion(scenario, duration):
    result = subprocess.run([sys.executable, str(Path(__file__).with_name('drive_pulse.py')),
                             '--execute', '--scenario', scenario, '--duration-sec', duration,
                             '--confirm-contact-dock-no-cable', '--existing-driver'],
                            capture_output=True, text=True, check=False)
    assert result.returncode == 2
    assert 'only supports a 0.5 second forward or in-place turn test' in result.stderr


@pytest.mark.parametrize(('scenario', 'target'), [('forward', '20'), ('right', '171'),
                                                  ('left', '0'), ('left', '171'), ('left', 'nan')])
def test_target_turn_rejects_invalid_request(scenario, target):
    result = subprocess.run([sys.executable, str(Path(__file__).with_name('drive_pulse.py')),
                             '--execute', '--scenario', scenario, '--target-turn-deg', target,
                             '--confirm-contact-dock-no-cable', '--existing-driver'],
                            capture_output=True, text=True, check=False)
    assert result.returncode == 2
    assert 'Target turn requires' in result.stderr


@pytest.mark.parametrize(('scenario', 'competing_command', 'target', 'bad_heading'), [
    ('forward', False, None, False), ('forward', True, None, False), ('left', False, None, False),
    ('left', True, None, False), ('right', False, None, False), ('right', True, None, False),
    ('left', False, 20, False), ('left', True, 20, False), ('left', False, 20, True),
    ('right', False, 20, False), ('right', True, 20, False)])
def test_existing_driver_with_isolated_ros(monkeypatch, tmp_path, competing_command, scenario, target, bad_heading):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Bool, Empty

    monkeypatch.setenv('ROS_DOMAIN_ID', '211')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    fake_file = tmp_path / 'src/omron_amr/amr_core/test/drive_pulse.py'
    fake_file.parent.mkdir(parents=True)
    monkeypatch.setattr(drive_pulse, '__file__', str(fake_file))
    parameters_path = tmp_path / 'src/omron_amr/amr_ros/config/ld250_parameters.yaml'
    parameters_path.parent.mkdir(parents=True)
    parameters_path.write_text(yaml.safe_dump({'amr_core': {'ros__parameters': {
        'robot': {}, 'driver': {'drive': {'ratio_mapping': 'legacy', 'unsafe_drive': True,
                                         'throttle_pct': 100.0},
                                'calibration': {'confirmed': False},
                                'cmd_vel': {'timeout_sec': 0.2, 'min_linear': -100.0,
                                            'max_linear': 1000.0, 'min_angular': -90.0,
                                            'max_angular': 90.0}}}}}))
    capture = {'complete': True, 'parameters': [
        {'section': 'Teleop Settings', 'name': name, 'value': value}
        for name, value in [('Timeout', 1.0), ('TransDeadZone', 14.0), ('RotDeadZone', 5.0)]]}
    replies = {'docking-status': 'DOCK_INFO ' + json.dumps({'state': 0, 'forced': False,
                                                            'shutdown_countdown': 0}),
               'config': 'motion_observed=0 malformed=0 transport_fault=0',
               'config-capture': 'CAPTURE ' + json.dumps(capture)}
    monkeypatch.setattr(drive_pulse.subprocess, 'run',
                        lambda command, **_kwargs: SimpleNamespace(returncode=0, stdout=replies[command[1]]))
    monkeypatch.setattr(drive_pulse.subprocess, 'Popen',
                        lambda *_args, **_kwargs: pytest.fail('must not launch another driver'))
    arguments = ['drive_pulse.py', '--execute', '--scenario', scenario,
                 '--confirm-contact-dock-no-cable', '--existing-driver']
    if target is not None:
        heading = math.radians(160 if bad_heading else 175)
        arguments.extend(['--target-turn-deg', str(target), '--expected-heading-rad', str(heading)])
    monkeypatch.setattr(sys, 'argv', arguments)

    context = Context()
    rclpy.init(context=context)
    driver = Node('amr_core', context=context)
    smoother = Node('velocity_smoother', context=context)
    odometry_publisher = driver.create_publisher(Odometry, '/odom', 10)
    ready_publisher = driver.create_publisher(Bool, '/amr/drive_ready', 10)
    competing_publisher = smoother.create_publisher(Twist, '/cmd_vel', 1)
    commands = []
    stops = []
    expected = (0.2, 0.0) if scenario == 'forward' else drive_pulse.SCENARIOS[scenario]
    conflict_sent = []
    simulated = {'heading': math.radians(175), 'turning': False}

    def on_command(message):
        commands.append((message.linear.x, message.angular.z))
        if target is not None and commands[-1] == expected:
            simulated['turning'] = True
        if competing_command and commands[-1] == expected and not conflict_sent:
            conflict = Twist()
            conflict.linear.x = 0.3
            competing_publisher.publish(conflict)
            conflict_sent.append(True)

    def on_stop(_message):
        simulated['turning'] = False
        stops.append(True)

    def publish_state():
        if simulated['turning']:
            simulated['heading'] += 0.05 if scenario == 'left' else -0.05
        odometry = Odometry()
        odometry.pose.pose.orientation.z = math.sin(simulated['heading'] / 2.0)
        odometry.pose.pose.orientation.w = math.cos(simulated['heading'] / 2.0)
        odometry_publisher.publish(odometry)
        ready_publisher.publish(Bool(data=True))

    driver.create_subscription(Twist, '/cmd_vel', on_command, 10)
    driver.create_subscription(Empty, '/stop', on_stop, 10)
    driver.create_timer(0.05, publish_state)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(driver)
    executor.add_node(smoother)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        result = drive_pulse.main()
        audit = json.loads(next((tmp_path / 'build/amr_core/drive-pulse').glob('*.json')).read_text())
        if bad_heading:
            assert result == 1
            assert audit['command_attempts'] == 0
            assert not commands
            assert not stops
            return
        assert stops
        assert expected in commands
        assert audit['linear_x_m_s'] == expected[0]
        assert audit['angular_z_rad_s'] == expected[1]
        if competing_command:
            assert result == 1
            assert audit['guard_failure'] == 'competing_velocity_command'
        else:
            assert result == 0
            assert audit['state'] == 'completed_unverified'
            if target is None:
                assert commands.count(expected) == 10
            else:
                assert audit['target_reported_angle_deg'] >= target
    finally:
        executor.shutdown()
        thread.join(timeout=5)
        driver.destroy_node()
        smoother.destroy_node()
        rclpy.shutdown(context=context)


def test_removed_departure_option_is_rejected():
    result = subprocess.run([sys.executable, str(Path(__file__).with_name('drive_pulse.py')),
                             '--execute', '--scenario', 'forward', '--confirm-contact-dock-no-cable',
                             '--forward-departure-bypass'], capture_output=True, text=True, check=False)
    assert result.returncode == 2
    assert 'unrecognized arguments: --forward-departure-bypass' in result.stderr

