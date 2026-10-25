# AMR Navigation Interface Remediation Plan

## 1. Scope and Baseline

Reviewed baseline: `omron_amr` branch `develop`, commit `a7a9e5a8`.
This document tracks the plan and initial implementation. Checked tasks indicate
completed software work only unless explicitly stated; no hardware validation
has been performed.

### Robot Work Deferred

The robot is not connected to this machine. Live configuration capture,
firmware/service verification, velocity calibration, joystick arbitration,
physical stopping tests, and Nav2 hardware acceptance are deferred until the
robot and a supervised test session are available. Keep their checklist items
open; do not substitute archive defaults for live evidence or retry connections
as part of offline work.

Keep `ratio_mapping: legacy`, `calibration.confirmed: false`, and
`unsafe_drive: true` unchanged. Resume with read-only configuration capture
before enabling calibrated mapping or running supervised motion tests.

Offline odometry heading/wrap/TF checks and the initial Nav2 topic/QoS/limit
review are now complete. Remaining offline follow-up includes failure/recovery
coverage and explicit deployment/arbitration design. These do not complete
hardware gates.

### Initial Implementation Status

Implemented and verified by 64 passing GoogleTest cases (11 command-policy tests,
26 loopback ARNetworking tests, 11 mapping tests, 10 local TCP queue tests, and 6 client-adapter
tests): a mutex-protected latest-command mailbox,
steady-clock expiry in the networking callback, 20 Hz default packet generation,
ordinary zero streaming, explicit stop precedence, joined networking teardown,
input validation, local asymmetric limit enforcement, and body-frame odometry
twist. The loopback tests use only an in-process fake server bound to localhost,
not a mobile robot. `unsafe_drive: true` remains unchanged.

Six additional Nav2 pytest checks cover both launch descriptions, the parameter
file default, nominal limits/rates, odometry topics, explicit Twist types, and
actual installed smoother traffic/QoS in an isolated localhost ROS domain. Both
controller and behavior output now enter the smoother; its reverse limit matches
the driver's -0.2 m/s clamp. OPEN_LOOP feedback and existing acceleration/timeouts
remain unchanged. Deployment namespaces, publisher arbitration, physical limits,
and the smoother-to-driver end-to-end timeout budget remain open.

The next software tranche adds confirmed startup mode gating and periodic
`getSafeDrive` monitoring. Startup rejects missing, malformed, or mismatched
confirmations. Runtime mismatch or lost confirmation invalidates movement,
requests a best-effort stop, and latches the interface off until restart. It
never repeatedly forces the mode back after an external change. The expanded
tests cover rejected/silent/invalid startup confirmations and runtime mode changes
or lost replies, including without ROS executor progress. This confirms only the
software flag in loopback, not real robot control ownership or protective state.

Queue bounds, absolute queue-age expiry, partial-write bookkeeping, and latched
failure cleanup now live in the driver-local
[DriveTcpSender helper](amr_core/include/amr_core/utils/drive_tcp_sender.hpp),
with an injectable writer, abort callback, and clock. Tests no longer require
additional methods or members in libaria. The helper is single-owner and expects
finalized packets; its caller must serialize all access and close a failed
connection through the abort callback. It is now wired into the driver through
the local [DriveClient adapter](amr_core/include/amr_core/utils/drive_client.hpp).

The SDK embeds its concrete sender and its send methods are non-virtual. The
adapter directs that sender into a local nonblocking socket pair, starting with
the connection handshake. Each SDK batch is drained within byte/time limits;
all control and motion packets then share the bounded downstream queue. Partial
frames cannot interleave with a separate SDK writer. Defaults are 32 packets,
65536 bytes, and 0.1 seconds; queue age cannot exceed command timeout. Overflow,
expiry, or transport failure closes the socket with zero linger, discards local
backlog, and latches movement off until restart. The adapter checks age before
writes and retains the original staging timestamp. No libaria edits are needed.

The local sender no longer uses `ArClientRatioDrive`; this relocation makes no
additional vendor changes. Existing include-only edits in the libaria worktree
were left untouched.
An opt-in `aria_ratio_v1` inverse now separates permitted velocity limits from
explicit full-throttle calibration, inverts throttle, and models direction- and
measured-speed-dependent turning. Invalid/unresolved calibration fails startup;
unrepresentable targets or stale required feedback invalidate the target and
request stop. Dead zones are not bypassed by boosting small commands. Hardware
YAML remains in warning-emitting `legacy` mode with unconfirmed calibration until
an operator supplies and confirms the installed firmware contract and settings.
The 4.1.12 archive contains LD250 simulation defaults but no verified live export;
those defaults were not installed. Configuration capture and supervised tracking,
manual override, and stopping remain open. See the capture/acceptance procedure
in [parameters.md](docs/parameters.md#calibration-capture-and-acceptance).
A stop accepted by the SDK is only locally queued, not confirmed
by the robot. Already-delivered kernel/robot bytes cannot be recalled, and a TCP
reset is not a cancellation acknowledgement. Do not deploy this tranche as a
fully validated navigation or stopping solution without hardware acceptance.

Still open: Phase A's live robot contract/baseline; hardware transport/deadman
verification; real firmware mode-query semantics and manual arbitration; live
calibration/firmware verification; additional failure/recovery tests; and all physical
acceptance checks. Automatic reconnect is not implemented. Fresh post-stop input
can resume after local stop submission, subject to server mode restrictions.

Software verification completed in this workspace:

```bash
colcon build --packages-select amr_core amr_nav2 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select amr_core amr_nav2
colcon test-result --verbose
```

Result: both affected packages build and all 64 GoogleTest cases plus six Nav2
pytest checks pass. Colcon reports 72 tests including wrappers, with zero errors,
failures, or skipped tests. The editor retains ROS include-path diagnostics,
but the actual ROS compiler succeeds.
Both hardware YAML configurations parse with `send_frequency: 20.0` and
`unsafe_drive: true`. No commands were sent to a physical robot.

Goal: provide continuous, predictable Nav2 velocity control without introducing
stop-mode transitions during normal driving, replaying stale commands, or
weakening robot safety and physical joystick override behavior.

Keep the existing ROS velocity input and the firmware's `ratioDrive` interface
initially. Do not replace the firmware, introduce raw motor commands, or assume
that the older SDK's `driveJoystick` service exists on the installed robot.

### Intentional Control Ownership

Keep `unsafe_drive: true` for autonomous Nav2 operation. This is an intentional
integration requirement: the robot's internal software lidar/sensor-based motion
limiting previously interfered with Nav2 commands. Nav2 owns normal navigation,
obstacle avoidance, and planned stopping; the remediation must not re-enable a
competing robot-side software avoidance controller as a default or fallback.

This setting is not evidence that independent protective functions remain
unaffected. Verify its exact firmware scope. Command-loss stopping, emergency
stops, and independent protective functions are separate from normal obstacle
avoidance and must not be disabled to achieve exclusive Nav2 control.

### Implementation Surfaces

- [Driver interface](amr_core/include/amr_core/interfaces/drive_interface.hpp):
	command intake, connection lifecycle, watchdog, conversion, and odometry.
- [Ratio-drive helper](libaria/ArNetworking/src/ArClientRatioDrive.cpp): current
	background resend behavior and implicit stop-mode requests.
- [Networking loop](libaria/ArNetworking/src/ArClientBase.cpp): callback execution
	and transport queuing behavior to account for when replacing the helper.
- [Server ratio mapping](libaria/src/ArActionRatioInput.cpp): SDK reference for
	asymmetric limits, dead zones, and speed-dependent rotation scaling.
- [LD250 hardware parameters](amr_ros/config/ld250_parameters.yaml) and
	[default hardware parameters](amr_ros/config/parameters.yaml).
- [Nav2 configuration](amr_nav2/config/ld250_nav2.yaml) and
	[Nav2 launch](amr_nav2/launch/nav2.launch.py).
- [Build configuration](amr_core/CMakeLists.txt) and
	[package dependencies](amr_core/package.xml): focused test registration.
- [Parameter documentation](docs/parameters.md): new and corrected semantics.

## 2. Required Behavior

1. A fresh command replaces the previous target; no completion acknowledgement
	 is required before accepting another velocity command.
2. A normal zero `Twist` sends zero ratios while remaining in ratio-drive mode.
3. Explicit stop, stale input, shutdown, disconnect, and physical override are
	 distinct events with documented transitions and recovery rules.
4. Only one execution context owns drive packet construction and transmission.
5. No nonzero packet is generated from an expired command. ROS time pauses or
	 executor stalls must not keep an old command alive indefinitely.
6. Transmission uses a bounded, configured rate and the newest complete command,
	 not a FIFO of historical commands.
7. Command conversion matches verified robot settings or reports that the
	 requested velocity cannot be represented. It must not silently assume a
	 symmetric joystick mapping.
8. Odometry pose is relative to its configured origin; odometry twist remains in
	 the child/body frame.
9. Independent robot-side timeouts, emergency stops, protective functions, and
	 physical joystick arbitration remain in effect. A software watchdog is not a
	 substitute for those functions.
10. Preserve `unsafe_drive: true` during autonomous operation. Do not restore
	robot-side software obstacle limiting on connection, recovery, or timeout as
	a workaround for jitter. Loss of Nav2 commands must cause a stop, not a
	transfer to another navigation or avoidance controller.

## 3. Phase A: Capture the Robot Contract

Do this before selecting conversion constants or changing robot configuration.

- [ ] A1. Record the source revision, deployed build revision, launch arguments,
	ROS distribution, firmware version, and active parameter overrides.
- [ ] A2. Inspect the robot's advertised services and access permissions using a
	read-only connection. Confirm `ratioDrive`, `stop`, `updateNumbers`, and any
	safety-state/configuration queries actually available at runtime.
- [ ] A3. Export the effective teleop configuration through supported tooling.
	Capture forward/reverse full-throttle speeds, rotation scaling at rest and in
	motion, dead zones, acceleration/deceleration limits, and teleop timeout.
- [ ] A4. Establish whether physical joystick and server teleop use separate
	configurations or share them. Record deadman, mode-lock, and override rules.
- [ ] A5. Capture a baseline ROS bag with command topics, odometry, TF, and robot
	status. Collect timestamped outbound-command events and robot mode changes
	without logging credentials. Record command gaps and actual packet rate.
- [ ] A6. Confirm the deployed executable uses the reviewed source rather than
	an older installed package or a different workspace overlay.

Exit gate: a saved robot contract and baseline trace. Binary strings and SDK
defaults are evidence of possible behavior, not proof of active settings.

## 4. Phase B: Command Ownership, Expiry, and Lifecycle

### Design

Use a small latest-command mailbox protected by a mutex. Each accepted sample
contains the entire linear/angular target, a steady-clock receipt timestamp,
and a sequence identifier. ROS callbacks update the mailbox; they do not mutate
the SDK helper's shared fields or transmit drive packets.

Prefer a driver-owned callback on the existing networking cycle to a second
independent sender thread. That callback snapshots the mailbox, checks command
age, and applies a steady-clock send-rate gate before constructing a packet.
Keep callback execution short and never hold the mailbox lock during network IO.

Replace the current helper in this driver rather than adding a second sender
alongside it. Inspect callback removal, thread joining, packet buffering, and
destructor ordering before choosing the exact shutdown implementation.

- [ ] B1. Define the command mailbox and explicit state transitions for waiting,
	fresh input, stale input, explicit stop, connection loss, and shutdown.
- [x] B2. Validate finite inputs before acceptance. Reject NaN/infinity without
	refreshing freshness, and define handling of unsupported `Twist` components.
- [ ] B3. Use latest-value input history, normally depth 1, with QoS verified
	against Nav2 publishers. Do not treat DDS delivery as a command-age guarantee.
- [ ] B4. Replace `ArClientRatioDrive` ownership in the driver with one sender
	that packs the verified protocol schema directly through `ArClientBase`.
- [x] B5. Add an independent send-rate parameter, initially 20 Hz. Preserve
	`expected_cmd_vel_freq` compatibility or explicitly document its deprecation;
	do not silently reuse it with changed semantics.
- [x] B6. Enforce expiry in the sending context using `std::chrono::steady_clock`.
	Do not depend on `node_->now()` or a ROS executor timer for command freshness.
- [x] B7. Prevent catch-up bursts after delays: send at most the newest sample
	for a cycle, and advance the schedule without replaying missed intervals.
- [x] B8. Inspect transport buffering and send failures. Prevent unbounded
	command accumulation, stop generation on transport failure, and document
	handling of already-queued TCP data. Do not assume queue cancellation exists.
	Implemented for the drive connection with a bounded local adapter; kernel/robot
	buffers and physical deadman behavior remain hardware acceptance requirements.
- [ ] B9. Require fresh post-connect input before movement. Do not automatically
	replay a pre-disconnect or pre-stop sample after recovery.
- [x] B10. Stop command production before removing callbacks and disconnecting.
	Join/stop networking with supported APIs so no callback accesses a destroyed
	driver. Treat shutdown stop transmission as best effort, not guaranteed.
- [ ] B11. Verify the robot-side timeout remains effective when the sender or
	process freezes. A stalled networking thread cannot run a local watchdog.

Exit gate: deterministic tests show coherent snapshots, bounded sends, no stale
nonzero packet generation, and safe callback teardown without robot hardware.

## 5. Phase C: Separate Zero Velocity from Stop Mode

- [x] C1. Remove the ordinary near-zero-to-`stop()` branch. With fresh input,
	zero ratios remain valid streaming commands with the configured throttle.
- [x] C2. Keep explicit stop requests distinct. Ensure the sender processes a
	pending stop before any subsequently considered movement command.
- [x] C3. Invalidate all samples received before an explicit stop. Define whether
	recovery requires a newer command, an explicit rearm, or verified mode release;
	do not add an arbitrary sleep or wait for each velocity command to complete.
- [ ] C4. On expiry, stop nonzero streaming and issue the supported stop request.
	Define bounded retry behavior for stop-send failure; do not flood requests or
	presume receipt without confirmation.
- [x] C5. Remove duplicate stop requests and hidden helper-destructor stop sends.
- [ ] C6. Verify continuous nonzero -> zero -> nonzero updates do not enter
	`Stopping`, while explicit stop and command expiry still halt the robot.

Exit gate: normal zero samples never generate a server `stop` request; explicit
stop and timeout events have tested, documented precedence and recovery.

## 6. Phase D: Match Velocity Conversion to Firmware

- [x] D1. Separate local permitted velocity limits from server full-throttle
	calibration. Enforce forward and reverse limits independently before mapping.
- [x] D2. Account for throttle explicitly, including zero throttle. Do not claim
	that a requested SI velocity is achieved when throttle changes its magnitude.
- [ ] D3. Model forward/reverse server scaling and speed-dependent angular
	scaling from the verified firmware contract. Check the units used by measured
	speed feedback before using it in an inverse mapping.
	The bundled SDK model is implemented and software-tested behind explicit
	opt-in confirmation. Installed firmware semantics and telemetry units remain
	unverified, so this hardware-contract task stays open.
- [ ] D4. Decide how dead zones are handled. Prefer verified server-side settings
	suitable for autonomous control when independently configurable and approved.
	Do not merely offset small requests past a dead zone: that introduces a minimum
	speed discontinuity and does not reproduce arbitrary small Nav2 velocities.
	Software rejects unrepresentable targets and handles SDK wire rounding;
	approved server settings and shared joystick behavior still require inspection.
- [ ] D5. If robot joystick settings are shared, assess the effect on human
	control before changing them. Do not silently overwrite robot configuration
	during startup. Detect or document configuration drift.
- [x] D6. Define behavior for saturation, stale feedback, invalid calibration,
	zero denominators, and unrepresentable commands. Surface actionable diagnostics.
- [ ] D7. If accurate continuous velocity mapping cannot be achieved with the
	available ratio interface, assess a vendor-supported absolute-velocity service
	as a separate decision, including arbitration and watchdog guarantees. Do not
	fall back to an unadvertised SDK service or raw controller commands.

Exit gate: measured forward/reverse and turning responses match the agreed
mapping across a low-speed validation grid, within recorded tolerances. Set those
tolerances from robot capabilities and baseline measurements, not assumptions.

## 7. Phase E: Correct Odometry Feedback

- [x] E1. Preserve the relative initial-frame transform for pose and heading.
- [x] E2. Publish decoded body-frame linear velocities directly in the odometry
	child frame; remove rotation by the initial heading from twist.
- [ ] E3. Verify the running firmware's `updateNumbers` schema, units, and frame
	semantics before relying on the older SDK's packet layout.
- [x] E4. Test origins at 0, +90, and -90 degrees: forward motion must remain
	positive body `linear.x` with approximately zero body `linear.y`.
- [x] E5. Verify TF direction, heading wrap, and consistent pose/twist timestamps.
	Packet-level loopback checks only; E3 and hardware heading acceptance remain open.

Exit gate: packet-level tests and a recorded heading experiment confirm correct
pose and twist conventions without changing topic or frame names unnecessarily.

## 8. Phase F: Nav2 Integration and Diagnostics

- [ ] F1. Keep the intended controller -> velocity smoother -> driver topic path.
	Check composed and standalone launches, namespaces, command message types, and
	publisher/subscriber QoS on the deployed ROS distribution.
	Offline checks now cover both launch descriptions and installed Jazzy smoother
	Twist/QoS. Both controller and behaviors pass through smoothing. Namespaced
	hardware overlays and the live graph remain unverified.
- [ ] F2. Ensure only the intended publisher owns the final command topic during
	autonomous operation. Define manual/automatic arbitration rather than allowing
	independent publishers to interleave commands accidentally.
- [ ] F3. Align Nav2 forward/reverse, angular, and acceleration limits with
	measured robot capability and the corrected mapping.
	Nominal smoother reverse now matches -0.2 m/s; configured forward/angular
	limits fit both hardware YAMLs. Real capability and acceleration remain open.
- [x] F4. Keep OPEN_LOOP smoothing initially to isolate driver changes. Evaluate
	CLOSED_LOOP separately only after odometry quality and latency are verified.
- [ ] F5. Choose command expiry from measured publishing jitter and an explicit
	stopping-latency budget. Review the current driver 0.2 s and smoother 1.0 s
	timeouts as different layers; do not increase either just to hide interruptions.
- [ ] F6. Add restrained diagnostics for command age, sender rate, expiry count,
	rejected commands, clipping, connection state, and stop reason. Prefer existing
	logging/diagnostic facilities over introducing an unrelated telemetry system.
- [ ] F7. Preserve the intentional `unsafe_drive: true` configuration so Nav2
	remains the sole normal navigation/obstacle-avoidance controller. Document
	exactly which robot-side lidar/sensor limiting it disables, verify independent
	protective functions separately, and verify reconnect/mode changes do not
	silently restore competing software control. Treat inability to establish the
	required mode as a startup/recovery failure, not permission to drive with a
	different controller. Do not equate either setting with certified safety.
- [x] F7a. Require a matching `getSafeDrive` startup confirmation, monitor it
	periodically with steady-clock deadlines, and latch movement off on mode loss
	without repeatedly forcing unsafe mode. Verify software behavior in loopback;
	retain F7's real-firmware and independent-protection checks as open gates.
- [ ] F8. Document parameters, units, robot-side prerequisites, stop/rearm rules,
	compatibility changes, and rollback steps.

## 9. Test Work and Required Gates

Add one focused `ament_cmake_gtest` test target for the command policy and mapping.
Use a fake clock and recording transport rather than a real robot for unit tests.
Add a separate integration test only where callback/transport behavior cannot be
covered by that target; avoid creating a broad test framework.

- [x] T1. Register the focused tests and required test dependencies under
	`BUILD_TESTING`; keep production dependencies unchanged unless required.
- [ ] T2. Test complete snapshots, newest-value replacement, configured rate,
	delayed cycles without bursts, startup idle, and reconnection without replay.
- [ ] T3. Test zero transitions, explicit-stop precedence, old-sample invalidation,
	expiry boundaries, stop-send failures, and documented rearm behavior.
- [ ] T4. Test ROS-clock pause/jump independence and simulated ROS executor stalls
	while networking continues. Expired movement must not continue being generated.
- [ ] T5. Test NaN/infinity rejection, asymmetric limits, throttle, saturation,
	dead-zone policy, angular mapping, and stale/invalid feedback.
- [ ] T6. Test odometry decoding and child-frame twist with nonzero initial yaw.
- [ ] T7. Exercise teardown and concurrent mailbox access; use ThreadSanitizer
	where supported, accounting for third-party-library limitations.
- [x] T8. Build affected packages and run the focused test target first, then
	package tests. Check test results explicitly before hardware validation.
- [x] T9. Cover rejected, silent, empty, and invalid startup mode replies plus
	runtime mode change, missing confirmations without ROS executor progress, and
	fault latching without automatic mode forcing in loopback tests.
- [x] T10. Exercise real localhost TCP with a non-reading peer, bounded SDK
	staging, queue expiry before resumed writes, latched failure, and ordered
	SDK control/motion frames. Inject steady time for deterministic expiry checks.
- [x] T11. Exercise calibrated directional/throttle/feedback grids, dead zones
	and wire rounding, malformed/stale feedback, invalid calibration, and
	unrepresentable-target stop behavior with pure and localhost tests.
- [x] T12. Cover nonzero odometry origins, 0/+90/-90 headings, both wrap
	directions, frame IDs, and identical TF/odometry pose and timestamps.
- [x] T13. Check Nav2 standalone/composed remappings, default YAML existence,
	Twist configuration, nominal limits/rates, and odometry topics; validate the
	installed smoother's Twist/QoS on localhost without launching a robot driver.

Example workspace commands after test registration and sourcing the appropriate
ROS environment:

```bash
colcon build --packages-up-to amr_core
colcon test --packages-select amr_core --event-handlers console_direct+
colcon test-result --verbose
```

## 10. Staged Hardware Acceptance

These are supervised tests, not commands to run automatically. Use a cleared
test area, conservative speed limits, an available emergency stop, and the
site's approved procedure. Agree on maximum travel and stopping distances before
testing interruptions or faults.

- [ ] H1. Complete a read-only connection/configuration check and archive settings.
- [ ] H2. At low speed, step between fresh targets before the previous target is
	reached. Verify the target is updated without waiting for motion completion.
- [ ] H3. Send a short zero interval followed by a new target. Confirm ordinary
	zeros remain in drive mode and no unexpected stop-mode transition occurs.
- [ ] H4. Test small corrections, reverse motion, in-place rotation, and curved
	motion against the recorded mapping tolerances.
- [ ] H5. Separately test input loss, explicit stop, process failure, network loss,
	and recovery under the approved procedure. Record command age, mode, velocity,
	and stopping time/distance; verify no old movement resumes after recovery.
- [ ] H6. Verify physical joystick takeover and deadman release behavior with the
	robot operator. Confirm autonomous commands do not defeat manual ownership or
	mode locks. If they do, block deployment pending an arbitration solution.
- [ ] H7. Repeat odometry tests with different starting headings.
- [ ] H8. Run a short Nav2 route with straight segments, curves, rotation, and a
	final approach. Compare velocity tracking, unexpected stops, packet rate, and
	command gaps with the baseline using recorded, agreed acceptance thresholds.
- [ ] H9. Verify Nav2-controlled obstacle avoidance and stopping without competing
	robot-side software limiting. Separately verify emergency stops, independent
	protective functions, and command-loss stopping under the approved procedure;
	do not disable those functions to complete this test.

## 11. Delivery Order and Rollback

1. Capture the contract and baseline: Phase A.
2. Implement sender ownership/expiry and zero/stop semantics together: Phases B
	 and C, with their focused tests. Do not deploy a partial sender replacement.
3. Correct mapping and calibration handling: Phase D, with conversion tests.
4. Correct odometry independently: Phase E, with packet-level tests.
5. Align configuration, diagnostics, and documentation: Phase F.
6. Complete software gates, supervised hardware checks, and Nav2 comparison:
	 Sections 9 and 10.

- [ ] R1. Deliver small reviewable changes with tests; do not modify vendored
	libaria. Keep additional transport behavior in driver-local helpers.
- [ ] R2. Preserve the previous deployable build and robot configuration export.
- [ ] R3. Define rollback as stopping the robot, restoring the prior software and
	any approved robot settings, and repeating a low-speed connection/stop check.
- [ ] R4. Block release on unresolved stale-command replay, stop precedence,
	manual-override failures, or unexplained velocity mapping errors.
- [ ] R5. Record final settings, build identity, test results, measured acceptance
	thresholds, and remaining limitations in the deployment handoff.
