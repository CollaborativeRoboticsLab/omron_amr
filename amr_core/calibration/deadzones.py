import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess

import yaml


WORKSPACE = Path(__file__).resolve().parents[4]


def deadzones(snapshot):
    if snapshot.get('complete') is not True or snapshot.get('read_only') is not True:
        raise ValueError('A complete read-only snapshot is required.')
    result = {}
    for parameter in snapshot.get('parameters', []):
        name = parameter.get('name')
        if parameter.get('section') != 'Teleop Settings' or name not in ('TransDeadZone', 'RotDeadZone'):
            continue
        value = parameter.get('value')
        if (name in result or isinstance(value, bool) or not isinstance(value, (float, int)) or
                not math.isfinite(value) or not 0 <= value < 100):
            raise ValueError('Invalid or duplicate dead-zone value.')
        result[name] = value
    if len(result) != 2:
        raise ValueError('Both dead-zone values are required.')
    return result


def summarize(samples):
    fields = ('x_mm', 'y_mm', 'heading_deg', 'forward_mm_s', 'angular_deg_s', 'lateral_mm_s')
    if len(samples) < 2:
        raise ValueError('Insufficient odometry samples.')
    for sample in samples:
        if any(isinstance(sample.get(field), bool) or not isinstance(sample.get(field), (int, float)) or
               not math.isfinite(sample[field]) for field in fields):
            raise ValueError('Malformed odometry sample.')
    first = samples[0]
    position_change = max(math.hypot(sample['x_mm'] - first['x_mm'], sample['y_mm'] - first['y_mm'])
                          for sample in samples)
    heading_change = max(abs((sample['heading_deg'] - first['heading_deg'] + 180) % 360 - 180)
                         for sample in samples)
    peaks = {field: max(abs(sample[field]) for sample in samples) for field in fields[3:]}
    return {
        'samples': len(samples), 'max_position_change_mm': position_change,
        'max_heading_change_deg': heading_change, 'peak_reported_speeds': peaks,
        'motion_evidence': position_change > 0 or heading_change > 0 or any(peaks.values()),
        'no_creep_proven': False,
    }


def run_probe(executable, category, connection):
    if category not in ('config-capture', 'observe'):
        raise ValueError('Only read-only calibration operations are permitted.')
    result = subprocess.run([str(executable), category], input=connection, text=True,
                            capture_output=True, timeout=15, check=False)
    if result.returncode != 0:
        raise ValueError('Read-only probe failed; output withheld. Check connection and robot state.')
    return result.stdout


def capture(executable, connection):
    records = [line.removeprefix('CAPTURE ') for line in
               run_probe(executable, 'config-capture', connection).splitlines() if line.startswith('CAPTURE ')]
    if len(records) != 1:
        raise ValueError('Expected one sanitized configuration snapshot.')
    return deadzones(json.loads(records[0]))


def apply_translation(binary, probe, connection, expected, target, rotation, output):
    report = {'started_at_utc': datetime.now(timezone.utc).isoformat(), 'operation': 'translation_deadzone_write',
              'expected_translation': expected, 'target_translation': target, 'expected_rotation': rotation,
              'complete': False, 'write_attempted': None, 'ready_for_motion': False,
              'persistence_verified': False, 'state': 'pending'}
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as destination:
        def save():
            destination.seek(0)
            json.dump(report, destination, indent=2)
            destination.write('\n')
            destination.truncate()
            destination.flush()
            os.fsync(destination.fileno())

        save()
        settings = connection + f'{expected}\n{target}\n{rotation}\nAPPLY_TRANSLATION_DEADZONE\n'
        environment = dict(os.environ, AMR_APPLY_TRANSLATION_DEADZONE='CONFIRMED')
        try:
            result = subprocess.run([str(binary), '--gtest_filter=HybridCalibration.DISABLED_ApplyTranslationDeadzone',
                                     '--gtest_also_run_disabled_tests'], input=settings, text=True,
                                    capture_output=True, timeout=45, check=False, env=environment)
            records = [line.removeprefix('CALIBRATION_RESULT ') for line in result.stdout.splitlines()
                       if line.startswith('CALIBRATION_RESULT ')]
            if len(records) != 1:
                raise ValueError('Expected one sanitized calibration result.')
            evidence = json.loads(records[0])
            if not isinstance(evidence, dict):
                raise ValueError('Invalid calibration result.')
            report['evidence'] = evidence
            report['write_attempted'] = evidence.get('write_attempted')
            report['probe_returncode'] = result.returncode
            report['complete'] = result.returncode == 0 and evidence.get('complete') is True
            report['state'] = 'verified' if report['complete'] else 'failed_or_uncertain'
        except (OSError, ValueError, subprocess.TimeoutExpired, KeyboardInterrupt):
            report['state'] = 'failed_or_uncertain'
            report['failure_reason'] = 'Calibration invocation failed; no retry. Independent read-back required.'
        save()
        try:
            report['independent_readback'] = capture(probe, connection)
            if report['independent_readback'] != {'TransDeadZone': target, 'RotDeadZone': rotation}:
                report['complete'] = False
                report['state'] = 'failed_or_uncertain'
        except (OSError, ValueError, subprocess.TimeoutExpired, KeyboardInterrupt):
            report['complete'] = False
            report['state'] = 'failed_or_uncertain'
            report['readback_failure'] = 'Independent read-back unavailable; current robot state is unverified.'
        report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        save()
    print(json.dumps({key: value for key, value in report.items() if key != 'evidence'}, indent=2))
    return 0 if report['complete'] else 1


def main():
    parser = argparse.ArgumentParser(description='Dead-zone observer; writes require separate explicit confirmation. Never drives.')
    parser.add_argument('--robot', action='store_true', help='Allow robot access; writes additionally require --confirm-write.')
    parser.add_argument('--params', type=Path, default=WORKSPACE / 'src/omron_amr/amr_ros/config/ld250_parameters.yaml')
    parser.add_argument('--probe', type=Path, default=WORKSPACE / 'build/amr_core/amr_hardware_probe')
    parser.add_argument('--expected-translation', type=float, required=True)
    parser.add_argument('--expected-rotation', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--apply-translation', type=int, help='Explicitly request one numeric translation-dead-zone change.')
    parser.add_argument('--confirm-write', action='store_true', help='Confirm the configuration write; not a motion authorization.')
    parser.add_argument('--test-binary', type=Path, default=WORKSPACE / 'build/amr_core/test_hybrid_config')
    args = parser.parse_args()
    if not args.robot:
        parser.error('--robot is required; no connection was opened.')
    expected = {'TransDeadZone': args.expected_translation, 'RotDeadZone': args.expected_rotation}
    if any(not math.isfinite(value) or not 0 <= value < 100 for value in expected.values()):
        parser.error('Expected percentages must be finite and between 0 inclusive and 100 exclusive.')
    if args.output.exists():
        parser.error('Output already exists; choose a new trial filename.')
    if args.confirm_write != (args.apply_translation is not None):
        parser.error('--apply-translation and --confirm-write must be provided together.')
    if args.apply_translation is not None and (
            not 1 <= args.apply_translation <= 99 or args.apply_translation == args.expected_translation or
            not args.expected_translation.is_integer() or not 1 <= args.expected_translation <= 99 or
            not args.expected_rotation.is_integer() or not 1 <= args.expected_rotation <= 99):
        parser.error('Write settings require distinct translation integer percentages and rotation in [1, 99].')
    try:
        robot = yaml.safe_load(args.params.read_text())['amr_core']['ros__parameters']['robot']
        port = int(robot['port'])
        values = (robot['ip'], str(port), robot['user'], robot['password'], robot.get('protocol', '6MTX'))
        if (not 1 <= port <= 65535 or not values[0] or
                any(not isinstance(value, str) or '\n' in value or '\r' in value for value in values)):
            raise ValueError()
        connection = '\n'.join(values) + '\n'
    except (OSError, KeyError, ValueError, TypeError, yaml.YAMLError):
        parser.error('Invalid connection file; values withheld.')
    if args.apply_translation is not None:
        return apply_translation(args.test_binary, args.probe, connection, int(args.expected_translation),
                                 args.apply_translation, int(args.expected_rotation), args.output)
    report = {'started_at_utc': datetime.now(timezone.utc).isoformat(), 'read_only': True,
              'expected_deadzones': expected, 'complete': False, 'samples': [], 'ready_for_motion': False}
    status = 1
    try:
        report['before'] = capture(args.probe, connection)
        if report['before'] != expected:
            raise ValueError('Read-back differs from expected settings; trial not started.')
        output = run_probe(args.probe, 'observe', connection)
        report['samples'] = [json.loads(line.removeprefix('ODOMETRY ')) for line in output.splitlines()
                             if line.startswith('ODOMETRY ')]
        report['odometry'] = summarize(report['samples'])
        report['after'] = capture(args.probe, connection)
        if report['after'] != expected:
            raise ValueError('Settings changed during observation; trial invalid.')
        report['complete'] = True
        status = 0
    except (ValueError, OSError, subprocess.TimeoutExpired):
        report['failure_reason'] = 'Observation failed or read-back mismatched; trial is not accepted.'
    report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as destination:
        json.dump(report, destination, indent=2)
        destination.write('\n')
    print(json.dumps({key: value for key, value in report.items() if key != 'samples'}, indent=2))
    return status


if __name__ == '__main__':
    raise SystemExit(main())