import importlib.util
from pathlib import Path

import pytest


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


@pytest.mark.parametrize(('scenario', 'duration', 'confirmed'), [
    ('reverse', 0.5, True), ('forward-left', 0.5, True), ('forward', 0.51, True),
    ('forward', 0.0, True), ('forward', float('nan'), True), ('forward', 0.5, False),
])
def test_departure_rejects_unapproved_request(scenario, duration, confirmed):
    with pytest.raises(ValueError):
        drive_pulse.departure_command(scenario, duration, confirmed)


def test_departure_uses_bounded_forward_command():
    assert drive_pulse.departure_command('forward', 0.1, True) == (0.2, 0.0)

