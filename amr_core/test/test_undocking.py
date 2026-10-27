from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import pytest
import yaml


pytestmark = [pytest.mark.hardware, pytest.mark.movement]


def scoped_robot_parameters(parameters):
    values = yaml.safe_load(Path(parameters).read_text())['amr_core']['ros__parameters']
    temporary = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml')
    try:
        yaml.safe_dump({'/undock_acceptance/amr_core': {'ros__parameters': values}}, temporary)
        temporary.flush()
    except Exception:
        temporary.close()
        raise
    return temporary


def test_one_supervised_contact_dock_departure(pytestconfig, robot_settings):
    if not pytestconfig.getoption('--robot') or not pytestconfig.getoption('--undock-once'):
        pytest.skip('Supervised undocking requires --robot and --undock-once.')
    import rclpy
    from rclpy.action import ActionClient
    from nav2_msgs.action import UndockRobot

    workspace = Path(__file__).resolve().parents[4]
    executable = workspace / 'install/amr_core/lib/amr_core/amr_core'
    probe = executable.with_name('amr_hardware_probe')
    parameters = str(Path(pytestconfig.getoption('--robot-params')).resolve())
    connection = '\n'.join(str(robot_settings[key]) for key in ('host', 'port', 'user', 'password', 'protocol')) + '\n'
    directory = workspace / 'build/amr_core/undocking'
    directory.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    path = directory / f'undock-once-{timestamp}.json'
    descriptor = os.open(path, os.O_CREAT | os.O_EXCL | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    report = {'started_at_utc': timestamp, 'state': 'pending', 'goal_attempts': 0,
              'physical_departure_confirmed': False, 'feedback_count': 0}

    def persist():
        content = (json.dumps(report, indent=2, sort_keys=True) + '\n').encode()
        os.lseek(descriptor, 0, os.SEEK_SET)
        offset = 0
        while offset < len(content):
            written = os.write(descriptor, content[offset:])
            if written <= 0:
                raise OSError('Audit persistence failed')
            offset += written
        os.ftruncate(descriptor, len(content))
        os.fsync(descriptor)

    def query(mode):
        result = subprocess.run([str(probe), mode], input=connection, text=True, capture_output=True,
                                timeout=15, check=False)
        if result.returncode:
            raise RuntimeError('Read-only probe failed; values withheld')
        if mode == 'docking-status':
            records = [json.loads(line.split(' ', 1)[1]) for line in result.stdout.splitlines()
                       if line.startswith('DOCK_INFO ')]
            if len(records) != 1:
                raise RuntimeError('Invalid docking status evidence')
            return records[0]
        return {'stationary': 'motion_observed=0 malformed=0 transport_fault=0' in result.stdout}

    process = None
    node = None
    client = None
    initialized = False
    goal = None
    scoped_parameters = None
    try:
        persist()
        report['before'] = query('docking-status')
        if report['before'] != {'state': 2, 'forced': False, 'shutdown_countdown': 0}:
            raise RuntimeError('Robot is not in the approved unforced Docked state')
        if not query('config')['stationary']:
            raise RuntimeError('Stationary preflight failed')
        scoped_parameters = scoped_robot_parameters(parameters)
        process = subprocess.Popen([str(executable), '--ros-args', '--params-file', scoped_parameters.name,
            '-r', '__node:=amr_core', '-r', '__ns:=/undock_acceptance',
            '-p', 'status.publish:=false', '-p', 'laser.main_laser.enabled:=false',
            '-p', 'laser.low_laser.enabled:=false', '-p', 'driver.publish_robot_tf:=false',
            '-p', 'docking.contact_dock_confirmed:=true', '-p', 'docking.exclusive_control_confirmed:=true',
            '-p', 'driver.cmd_vel_topic:=unused_cmd_vel'], stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, start_new_session=True)
        rclpy.init()
        initialized = True
        node = rclpy.create_node('undock_acceptance_client')
        client = ActionClient(node, UndockRobot, '/undock_acceptance/undock_robot')
        if not client.wait_for_server(timeout_sec=15):
            raise RuntimeError('Undock action server unavailable; no goal sent')
        settled = time.monotonic() + 2.0
        while time.monotonic() < settled:
            rclpy.spin_once(node, timeout_sec=0.1)

        def wait(future, seconds):
            deadline = time.monotonic() + seconds
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not future.done():
                raise TimeoutError('ROS action deadline exceeded; outcome uncertain')
            return future.result()

        def feedback(_message):
            report['feedback_count'] += 1

        report['state'] = 'goal_pending'
        report['goal_attempts'] = 1
        persist()
        goal = wait(client.send_goal_async(UndockRobot.Goal(dock_type='omron', max_undocking_time=30.0),
                          feedback_callback=feedback), 5)
        if goal is None or not goal.accepted:
            report['state'] = 'rejected'
            raise RuntimeError('Undock goal rejected; do not retry automatically')
        try:
            result = wait(goal.get_result_async(), 40)
        except TimeoutError:
            wait(goal.cancel_goal_async(), 5)
            raise
        report['result'] = {'status': result.status, 'success': result.result.success,
                            'error_code': result.result.error_code, 'error_msg': result.result.error_msg}
        report['state'] = ('reported_success' if result.status == 4 and result.result.success and
                           result.result.error_code == UndockRobot.Result.NONE else 'failed_or_uncertain')
    except Exception:
        report['state'] = 'failed_or_uncertain' if report['state'] != 'rejected' else 'rejected'
    finally:
        if client is not None:
            client.destroy()
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        if scoped_parameters is not None:
            scoped_parameters.close()
        try:
            report['after'] = query('docking-status')
            report['after_telemetry'] = query('config')
        except Exception:
            report['readback_failed'] = True
        try:
            persist()
        finally:
            os.close(descriptor)
    print(f'Undocking audit: {path}')
    assert report['state'] == 'reported_success', 'Departure failed or uncertain; inspect audit; no automatic retry.'
    assert report.get('after') == {'state': 0, 'forced': False, 'shutdown_countdown': 0}
    assert report.get('after_telemetry', {}).get('stationary') is True