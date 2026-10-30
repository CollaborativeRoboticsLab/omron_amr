"""Opt-in supervised cmd_vel-style drive acceptance tests."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import yaml


class GuardFailure(RuntimeError):
    pass


SCENARIOS = {
    'forward': (0.5, 0.0),
    'reverse': (-0.2, 0.0),
    'left': (0.0, 0.5),
    'right': (0.0, -0.5),
    'forward-left': (0.5, 0.5),
    'forward-right': (0.5, -0.5),
}


def stream_command(send_command, send_stop, spin, clock=time.monotonic, duration=0.5):
    if not math.isfinite(duration) or duration <= 0.0 or duration > 6.0:
        raise ValueError('Stream duration must be greater than zero and at most six seconds')
    deadline = clock() + duration
    next_send = clock()
    sent = 0
    maximum_commands = math.ceil(duration * 20.0 - 1e-9)
    last_send = None
    try:
        while clock() < deadline:
            now = clock()
            if last_send is not None and now - last_send >= 0.2:
                raise GuardFailure('command_stream_stalled')
            if now >= next_send and sent < maximum_commands:
                send_command()
                sent += 1
                last_send = clock()
                next_send = last_send + 0.05
            else:
                spin(min(0.01, max(0.0, next_send - now) if sent < maximum_commands else 0.01, deadline - now))
    finally:
        send_stop()


def require_contact_dock(confirmed):
    if confirmed is not True:
        raise RuntimeError('Contact-dock and disconnected-cable attestation required; no command sent')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--execute', action='store_true', required=True)
    parser.add_argument('--scenario', choices=SCENARIOS, required=True)
    parser.add_argument('--duration-sec', type=float, default=0.5)
    parser.add_argument('--confirm-contact-dock-no-cable', action='store_true')
    args = parser.parse_args()
    max_duration = 6.0 if args.scenario == 'reverse' else 2.0
    if not math.isfinite(args.duration_sec) or args.duration_sec <= 0.0 or args.duration_sec > max_duration:
        parser.error(f'Stream duration must be greater than zero and at most {max_duration:g} seconds')
    if not args.confirm_contact_dock_no_cable:
        parser.error('Movement requires explicit confirmation of a contact dock and no connected cable')

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, Empty

    workspace = Path(__file__).resolve().parents[4]
    parameters_path = workspace / 'src/omron_amr/amr_ros/config/ld250_parameters.yaml'
    parameters = yaml.safe_load(parameters_path.read_text())['amr_core']['ros__parameters']
    robot = parameters['robot']
    require_contact_dock(args.confirm_contact_dock_no_cable)
    driver = parameters['driver']
    if (driver.get('ratio_mapping') != 'legacy' or driver.get('calibration', {}).get('confirmed') is not False
            or driver.get('unsafe_drive') is not True or driver.get('cmd_vel_timeout_sec') != 0.2
            or driver.get('max_linear_speed') != 1000.0 or driver.get('drive_throttle_pct') != 100.0):
        raise RuntimeError('Unexpected driver configuration; no command sent')
    linear, angular = SCENARIOS[args.scenario]
    if not (driver['min_linear_speed'] / 1000.0 <= linear <= driver['max_linear_speed'] / 1000.0
            and math.radians(driver['min_angular_speed']) <= angular <= math.radians(driver['max_angular_speed'])):
        raise RuntimeError('Scenario exceeds configured speed limits; no command sent')
    connection = '\n'.join(str(robot.get(key, '6MTX')) for key in ('ip', 'port', 'user', 'password', 'protocol')) + '\n'
    executable = workspace / 'install/amr_core/lib/amr_core/amr_core'
    probe = executable.with_name('amr_hardware_probe')
    directory = workspace / 'build/amr_core/drive-pulse'
    directory.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    path = directory / f'{args.scenario}-stream-{timestamp}.json'
    descriptor = os.open(path, os.O_CREAT | os.O_EXCL | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    report = {'state': 'pending', 'scenario': args.scenario, 'command_attempts': 0, 'command_times': [],
              'contact_dock_no_cable_confirmed': True,
              'linear_x_m_s': linear, 'angular_z_rad_s': angular,
              'command_duration_s': args.duration_sec, 'command_frequency_hz': 20.0,
              'stop_acknowledged': False, 'odometry': []}

    def persist():
        data = (json.dumps(report, indent=2, sort_keys=True) + '\n').encode()
        os.lseek(descriptor, 0, os.SEEK_SET)
        offset = 0
        while offset < len(data):
            written = os.write(descriptor, data[offset:])
            if written <= 0:
                raise OSError('Audit persistence failed')
            offset += written
        os.ftruncate(descriptor, len(data))
        os.fsync(descriptor)

    def query(mode):
        result = subprocess.run([str(probe), mode], input=connection, text=True, capture_output=True,
                                timeout=20, check=False)
        if result.returncode:
            raise RuntimeError('Read-only preflight failed')
        if mode == 'docking-status':
            records = [json.loads(line.split(' ', 1)[1]) for line in result.stdout.splitlines()
                       if line.startswith('DOCK_INFO ')]
            if len(records) != 1:
                raise RuntimeError('Invalid docking response')
            return records[0]
        if mode == 'config-capture':
            records = [json.loads(line.split(' ', 1)[1]) for line in result.stdout.splitlines()
                       if line.startswith('CAPTURE ')]
            if len(records) != 1 or records[0].get('complete') is not True:
                raise RuntimeError('Invalid configuration snapshot')
            values = {item['name']: item['value'] for item in records[0]['parameters']
                      if item.get('section') == 'Teleop Settings'}
            if values.get('Timeout') != 1.0 or values.get('TransDeadZone') != 14.0 or values.get('RotDeadZone') != 5.0:
                raise RuntimeError('Teleop configuration changed')
            return {key: values[key] for key in ('Timeout', 'TransDeadZone', 'RotDeadZone')}
        if 'motion_observed=0 malformed=0 transport_fault=0' not in result.stdout:
            raise RuntimeError('Stationary telemetry preflight failed')
        return {'stationary': True}

    process = None
    node = None
    initialized = False
    stop_publisher = None
    samples = []
    try:
        persist()
        report['before_docking'] = query('docking-status')
        if report['before_docking'] != {'state': 0, 'forced': False, 'shutdown_countdown': 0}:
            raise RuntimeError('Robot docking state does not match the selected test')
        report['before_telemetry'] = query('config')
        report['before_settings'] = query('config-capture')
        driver_command = [str(executable), '--ros-args', '--params-file', str(parameters_path),
            '-r', '__node:=amr_core', '-r', 'undock_robot:=/drive_acceptance/undock_robot',
            '-p', 'status.publish:=false', '-p', 'laser.main_laser.enabled:=false',
            '-p', 'laser.low_laser.enabled:=false', '-p', 'driver.publish_robot_tf:=false',
            '-p', 'docking.contact_dock_confirmed:=true',
            '-p', 'driver.cmd_vel_topic:=/drive_acceptance/cmd_vel',
            '-p', 'driver.stop_topic:=/drive_acceptance/stop', '-p', 'driver.odom_topic:=/drive_acceptance/odom',
            '-p', 'driver.drive_ready_topic:=/drive_acceptance/drive_ready']
        process = subprocess.Popen(driver_command,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        rclpy.init()
        initialized = True
        node = rclpy.create_node('drive_acceptance_client')
        command_publisher = node.create_publisher(Twist, '/drive_acceptance/cmd_vel', 1)
        stop_publisher = node.create_publisher(Empty, '/drive_acceptance/stop', 10)
        readiness = {'ready': False, 'received_at': 0.0}

        def on_readiness(message):
            readiness['ready'] = message.data
            readiness['received_at'] = time.monotonic()

        def on_odometry(message):
            orientation = message.pose.pose.orientation
            sample = {'monotonic': time.monotonic(), 'x': message.pose.pose.position.x,
                      'y': message.pose.pose.position.y, 'linear_x': message.twist.twist.linear.x,
                      'linear_y': message.twist.twist.linear.y, 'angular_z': message.twist.twist.angular.z,
                      'heading': math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                                            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))}
            samples.append(sample)

        subscription = node.create_subscription(Odometry, '/drive_acceptance/odom', on_odometry, 10)
        ready_subscription = node.create_subscription(Bool, '/drive_acceptance/drive_ready', on_readiness, 10)
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('Driver exited before stream')
            rclpy.spin_once(node, timeout_sec=0.05)
            if (command_publisher.get_subscription_count() == 1 and stop_publisher.get_subscription_count() == 1
                    and len(samples) >= 3 and samples[-1]['monotonic'] - samples[0]['monotonic'] >= 1.0
                    and time.monotonic() - samples[-1]['monotonic'] < 0.3 and readiness['ready']
                    and time.monotonic() - readiness['received_at'] < 0.2):
                break
        else:
            raise RuntimeError('Driver or fresh odometry unavailable')
        if any(abs(sample[key]) > 0.01 for sample in samples[-3:] for key in ('linear_x', 'linear_y', 'angular_z')):
            raise RuntimeError('Robot telemetry is not stationary')
        if any(not math.isfinite(value) for sample in samples[-3:] for value in sample.values()):
            raise RuntimeError('Robot telemetry is invalid')
        if node.count_publishers('/drive_acceptance/cmd_vel') != 1:
            raise RuntimeError('Unexpected command publisher')

        command = Twist()
        command.linear.x = linear
        command.angular.z = angular

        origin = samples[-1]

        def guarded_spin(timeout):
            rclpy.spin_once(node, timeout_sec=timeout)
            sample = samples[-1]
            if process.poll() is not None:
                raise GuardFailure('driver_exited')
            if time.monotonic() - sample['monotonic'] > 0.5:
                raise GuardFailure('odometry_stale')
            if not readiness['ready'] or time.monotonic() - readiness['received_at'] > 0.2:
                raise GuardFailure('driver_not_ready')
            if any(not math.isfinite(value) for value in sample.values()):
                raise GuardFailure('odometry_invalid')
            if math.hypot(sample['x'] - origin['x'], sample['y'] - origin['y']) > 0.4:
                raise GuardFailure('displacement_limit')
            heading_change = sample['heading'] - origin['heading']
            if abs(math.atan2(math.sin(heading_change), math.cos(heading_change))) > 0.6:
                raise GuardFailure('rotation_limit')

        def send_command():
            if not readiness['ready'] or time.monotonic() - readiness['received_at'] > 0.2:
                raise GuardFailure('driver_not_ready')
            report['command_times'].append(time.monotonic())
            command_publisher.publish(command)

        def send_stop():
            report['stop_requested_at'] = time.monotonic()
            stop_publisher.publish(Empty())

        report['state'] = 'attempting'
        report['command_attempts'] = 1
        persist()
        stream_command(send_command, send_stop, guarded_spin, duration=args.duration_sec)
        report['state'] = 'stop_requested'
        persist()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('Driver exited after stream')
            rclpy.spin_once(node, timeout_sec=0.05)
        report['state'] = 'completed_unverified'
        del subscription
        del ready_subscription
    except Exception as error:
        report['state'] = 'failed_or_uncertain'
        report['failure_type'] = type(error).__name__
        if isinstance(error, GuardFailure):
            report['guard_failure'] = str(error)
    finally:
        report['odometry'] = samples
        if node is not None and stop_publisher is not None and report['command_attempts']:
            try:
                stop_publisher.publish(Empty())
                rclpy.spin_once(node, timeout_sec=0.1)
            except Exception:
                report['cleanup_stop_failed'] = True
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
        try:
            report['after_docking'] = query('docking-status')
            report['after_telemetry'] = query('config')
        except Exception:
            report['readback_failed'] = True
        persist()
        os.close(descriptor)
    print(f'Audit: {path}\nState: {report["state"]}')
    return 0 if report['state'] == 'completed_unverified' and not report.get('readback_failed') else 1


if __name__ == '__main__':
    raise SystemExit(main())