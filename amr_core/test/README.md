# AMR Core Testing

LD-250 test results and commands. See the [experiment log](../../docs/experiement_log.md)
for current findings. Run commands from the colcon workspace root; past approvals
do not authorize new robot access or movement.

## Automated Offline Tests

The core suite tests command expiry, stop priority, ratio mapping, transport,
docking, configuration and calibration workflows. Localhost tests exercise the
driver and docking action against a fake server, including all six primitive
streams. Python tests cover captured data and pulse-runner guards. All four
registered core test groups passed on 2026-10-30 after the loopback tests were
split by interface. The aggregate target was then renamed to
`test_robot_interface`, rebuilt and passed separately. The test executable
contains 117 cases, including 69 loopback cases. Offline tests do not establish
physical motion, firmware behavior or safety performance.

The aggregate target includes drive, configuration, map, debug and docking
tests. Loopback cases share [robot_interface_test_fixture.hpp](robot_interface_test_fixture.hpp);
the drive-specific cases remain in [test_drive_interface.cpp](test_drive_interface.cpp).

Run the registered offline tests from the workspace root:

```bash
source install/setup.bash
ctest --test-dir build/amr_core --output-on-failure
```

To rebuild and run the aggregate GoogleTest target:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
cmake --build build/amr_core --target test_robot_interface -j2
ctest --test-dir build/amr_core --output-on-failure -R '^test_robot_interface$'
```

## Manual Static Inspection

- SDK ratio-input logic was inspected, but is not verification of the installed
	firmware or physical calibration.
- A MobilePlanner capture showed movement-like and zero commands. The inferred
	`ratioDrive` service mapping and safe-drive startup sequence were not proven.
- A supplied server binary's ARCL and binary undock handlers use the same
	backend; its correspondence to the installed firmware is unverified.

Raw captures and configuration snapshots may contain credentials; keep them
private and out of source control. The read-only test passes credentials to the
probe on stdin; never put them in command arguments or reports.

## Stationary Robot Tests

### Configuration Interface

Read-only discovery and configuration snapshots exercised the connection and the 22-parameter allowlist. An authorized translation dead-zone change from 10% to 14% was independently read back and remained present after one reboot. An authorized teleoperation timeout change from 2.0 s to 1.0 s was confirmed by a write receipt and independent readback; persistence across reboot and physical watchdog stopping were not verified. The rotation dead zone was read as 5%; a live rotation dead-zone write was not verified.

Motor status reported enabled, but a valid fault response was unavailable. A powered motor does not establish permission to move. The driver has been used
with `unsafe_drive: true` and `SAFE_DRIVE` reported as 0 under explicit prior authorization; this is not a protective safety guarantee.

With separate approval for read-only access, run the connectivity and configuration checks from the workspace root:

```bash
source install/setup.bash
PYTHONPATH=src/omron_amr/amr_core/test python3 -m pytest -p test_configuration \
	src/omron_amr/amr_core/test/test_connectivity.py \
	src/omron_amr/amr_core/test/test_config.py \
	--robot -m 'connectivity or config' -v \
	--junitxml=build/amr_core/hardware-readonly-results.xml
```

### Charging, Odometry and Battery

Docking state and motion telemetry were read during the tests. The operator reported the charging situation, but no separate end-to-end acceptance of the charging and battery interfaces is documented here. Odometry supplied the motion guards and reported displacement and velocity; its accuracy and units have not been independently verified against a physical reference.

Two separately authorized firmware-backed undock actions timed out without observed movement. After the charger was unplugged and the robot rebooted, later preflights reported Undocked. The failure remains unresolved; see [known issues](../../known_issues.md).

The following read-only observer records odometry and checks the expected dead zones; it is not a charging or battery acceptance test. Use a new output filename and obtain separate approval for robot access:

```bash
source install/setup.bash
ros2 run amr_core amr_deadzone_calibration --robot \
	--expected-translation 14 --expected-rotation 5 \
	--output build/amr_core/calibration/observation-14-5.json
```

## Supervised Motion Tests

These are evidence that basic command paths can produce motion, not acceptance of distance tracking, trajectory following or protective stopping. The available audits do not independently establish a completed standalone physical test of each in-place turn.

| Trial | Recorded result |
| --- | --- |
| Initial 0.1 s forward pulse | Inhibited by the docking readiness gate; no motion observed. |
| Forward pulse and ten-pulse retry | Operator observed movement and stopping; ten-pulse retry reported about 0.031 m total odometric displacement. An earlier ten-pulse attempt stopped after one pulse with a guard failure. |
| 0.5 s reverse | Operator observed reverse movement and stopping; about 0.023 m odometric displacement. |
| 2 s and 6 s reverse | Completed the command stream with stationary post-test telemetry; about 0.094 m and 0.286 m odometric displacement, respectively. |
| Up to 2 s forward | Displacement guard stopped the stream after about 1.42 s and 0.401 m odometry. Immediate readback failed; the operator later confirmed the robot stopped and a later read-only probe reported stationary. |
| Up to 2 s forward-left | Displacement guard stopped the stream after about 1.38 s and 0.400 m odometry. Immediate readback failed; the operator later confirmed the robot stopped and a later probe reported stationary. |
| Up to 2 s forward-right | Displacement guard stopped the stream after about 1.50 s and 0.448 m odometry. Immediate readback failed; no separate post-test stationary readback is recorded in that audit. |

Recorded displacement is odometry, not ground truth. The guards are not safety-rated, and a stop request being queued does not prove the robot received or acted on it. Audit outcomes with a completed stream are still marked `completed_unverified`, because stop delivery is not acknowledged.

The test-only [pulse runner](drive_pulse.py) sends one selected stream per invocation. Announce each test and obtain fresh explicit authorization; confirm an accessible emergency stop, exclusive control, a clear area, and contact docking with no attached cable. If the E-stop is pressed, keep it engaged until the runner and driver exit. Investigate uncertain results before another test.

These are individual examples of the supervised stream commands used from the workspace root. Each command can move the robot and requires its own fresh authorization; do not run them as an unattended sequence.

```bash
source install/setup.bash
python3 src/omron_amr/amr_core/test/drive_pulse.py --execute \
	--scenario reverse --duration-sec 6 --confirm-contact-dock-no-cable
```

```bash
python3 src/omron_amr/amr_core/test/drive_pulse.py --execute \
	--scenario forward-left --duration-sec 2 --confirm-contact-dock-no-cable
```

```bash
python3 src/omron_amr/amr_core/test/drive_pulse.py --execute \
	--scenario forward-right --duration-sec 2 --confirm-contact-dock-no-cable
```

Scenarios are `forward`, `reverse`, `left`, `right`, `forward-left`, and
`forward-right`. The default duration is 0.5 s; the maximum is 2 s, or 6 s for
reverse. The runner records a private audit under `build/amr_core/drive-pulse/`.
Its odometry guards are not safety-rated. A stop request is not an acknowledged
physical stop; investigate uncertain outcomes before any further test.

The driver requests its configured `unsafe_drive` mode at startup. Docking
readiness and the `drive_ready` topic are not protective safety functions.
Firmware undocking remains unresolved; see [known issues](../../known_issues.md).

The experimental dock-departure bypass is default-off and has been tested only
with a fake server. It requires fresh Docked feedback, stationary telemetry,
and explicit contact-dock and exclusive-control confirmations. It permits one
straight-forward request of 0.2 m/s for at most 0.5 s, then latches off until
restart; the undock action is disabled in this mode. Direct departure support by
the installed firmware, actual speed, and stop delivery are unverified. Only
after separate approval for this specific motion, a single 0.1 s test is:

```bash
source install/setup.bash
python3 src/omron_amr/amr_core/test/drive_pulse.py --execute \
	--scenario forward --duration-sec 0.1 --confirm-contact-dock-no-cable \
	--forward-departure-bypass --confirm-exclusive-control
```

## Distance and Velocity Calibration

Odometry alone cannot validate its own accuracy: compare it with a measured
course or independent localization, in both directions and on representative
floors and loads. Separately verify actual speed against commanded speed and
any closed-loop distance controller. The recorded -0.2 m/s reverse request
produced about -0.048 m/s reported steady velocity.

A 0.1 m/s command for 10 s implies 1 m only at constant actual speed;
acceleration and stopping affect distance. In the bundled SDK model, the
legacy 10% ratio for this request is below the observed 14% translation dead
zone and may not move the robot. Installed firmware behavior is unverified.
Trajectory following, obstacle avoidance and physical watchdog stopping remain
unaccepted. Do not enable calibration from these estimates.

## Configuration Changes

Changes require separate approval and independent readback. The guarded native
`amr_deadzone_calibration` workflow supports translation dead-zone changes;
the low-level writer supports the rotation dead zone and teleop timeout. A
write receipt alone does not establish application or persistence. Never retry
an uncertain write blindly or enable calibration from guessed settings.