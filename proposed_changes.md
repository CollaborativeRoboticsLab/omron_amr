# AMR Navigation Remaining Work

Status as of 2026-11-11. The source baseline inspected for this update is
`728acc20`; the deployed revision, active overrides, and present robot state
still need independent confirmation. This plan is not authorization for robot
access, configuration changes, or movement.

Maintain `ratio_mapping: legacy`, `calibration.confirmed: false`, and
`unsafe_drive: true` until their behavior is verified and changes are approved.
Do not modify or reset vendor `libaria`. A locally queued stop is not proof of
delivery or physical stopping; software checks are not protective functions.
See [known issues](known_issues.md) and the
[experiment log](docs/experiement_log.md) for unresolved observations.

## Immediate Navigation Gates

Before movement, confirm the robot is stationary, inspect docking state and
faults, and resolve any uncertain operation using the supported recovery
procedure. Do not automatically retry an uncertain undock or assume a stop
canceled it.

- [ ] When manually parking, verify that the robot display shows charging;
  the dock's yellow light alone may not mean the robot registered docking.
  Investigate intermittent disagreement between the display and dock light.
- [ ] For a namespaced robot, configure and verify both costmap observation
  sources against its own scan topic.
- [ ] Verify map coverage and localization and TF during separately authorized
  movement across the intended operating area. Do not assume the current SLAM
  map is complete or suitable for autonomous movement.
- [ ] Establish exclusive command ownership, physical joystick takeover,
  independent protective functions, measured stopping behavior and conservative
  motion limits before separately authorized Nav2 movement trials.
- [ ] Verify the integrated MOMA hardware, Nav2 and joystick command path on
  hardware under a separately approved procedure, including the MOMA-specific
  costmap configuration and combined robot footprint.

## Robot Contract and Calibration

- [ ] Record source and deployed build revisions, ROS and firmware versions,
  launch arguments, active overrides, advertised services and permissions.
  Confirm the deployed executable and configuration match the reviewed source.
- [ ] Verify the firmware contract for the captured teleop settings, including
  directional full-throttle speeds, rotation scaling, dead zones, acceleration,
  deceleration and timeout behavior. Verify persistence and the physical
  watchdog effect of the changed teleop timeout.
- [ ] Determine whether the physical joystick and server teleop share settings;
  verify deadman, mode locking, takeover and override behavior before changes.
- [ ] Complete the approved baseline of outbound-command timing and mode
  changes without recording credentials.
- [ ] Validate velocity conversion against the installed firmware and measured
  forward, reverse and turning response. Decide the dead-zone policy, assess
  shared joystick effects, and independently measure odometry accuracy and
  `updateNumbers` schema, units and body-frame semantics. Do not mark calibration
  confirmed from SDK defaults or odometry alone.
- [ ] Align Nav2 velocity and acceleration limits with measured capability.
  The running driver still uses a -0.2 m/s reverse clamp; the source clamp is
  now -0.1 m/s and has not been installed or deployed. Assess the estimated
  legacy dead-zone boundary and approve any deployment separately. If the ratio
  interface cannot meet agreed tolerances, assess a vendor-supported
  absolute-velocity interface separately.
- [ ] Verify firmware mode-query semantics, the exact scope of `unsafe_drive`,
  independent protective functions, and nonempty fault-reply formats. Do not
  infer fault status from a rejected response or silently force a lost mode.

## Control, Recovery and Diagnostics

- [ ] Verify complete latest-command snapshots, depth/QoS and the deployed
  controller-to-smoother-to-driver path, including composed and standalone
  launches, namespaces, message types and publisher ownership.
- [ ] Implement or verify enforced exclusive-source selection and break-before-
  make handover, including retained-target clearing and physical joystick
  takeover. Current operator confirmations do not enforce arbitration.
- [ ] Verify the sender's startup, fresh post-connect input, no stale replay,
  expiry and explicit-stop precedence. Define bounded handling of stop-send
  failure; do not presume robot receipt or physical stopping.
- [ ] Establish a measured command-loss and stopping-latency budget before
  changing driver or smoother timeouts. Verify robot-side timeout behavior when
  the sender, process or network stops making progress.
- [ ] Characterize the real docking feedback gap and validate the physical
  effect of the configured five-second active-undock limit. A longer limit
  delays detection of lost feedback; query/reply association remains uncertain.
  Keep the stop request, failure latch and other admission and telemetry guards.
- [ ] Add or complete actionable diagnostics for command age, sender rate,
  expiry, rejected and clipped commands, connection state and stop reason.
  Complete parameter, prerequisites, stop/rearm and rollback documentation.
- [ ] Complete remaining localhost coverage for additional sender rates and
  delayed network cycles, startup/reconnection, ROS clock jumps outside command
  expiry, executor stalls, teardown and concurrency; use ThreadSanitizer where
  supported.

## Supervised Hardware Acceptance

Obtain separate, fresh approval for each test. Use the site's approved
procedure, a clear area, verified E-stop and protective functions, exclusive
control, and agreed travel and stopping limits. Stop and investigate unexpected
or uncertain outcomes; never automatically retry.

- [ ] Resolve the empty and malformed fault-response formats against the
  installed firmware contract; archive the remaining firmware contract and
  deployment baseline privately. Do not treat an empty reply as no faults.
- [ ] Verify low-speed fresh-target changes and nonzero-to-zero-to-nonzero
  behavior without unexpected stop-mode transitions.
- [ ] Measure forward, reverse, in-place turns and curves against agreed
  tolerances, and verify odometry at multiple headings with an independent
  physical reference. Investigate the observed left target-turn overshoot and
  stopping behavior before repeating a target turn.
- [ ] Separately test input loss, explicit stop, process and network failure,
  recovery, watchdog behavior and stale-command replay. Record command timing,
  mode, velocity and physical stopping time and distance.
- [ ] Verify joystick takeover and deadman release without autonomous commands
  defeating manual ownership or mode locks.
- [ ] After the preceding gates, test a separately authorized short Nav2 route
  and obstacle avoidance with agreed acceptance thresholds. Independently test
  protective functions and command-loss stopping.

## Release and Rollback

- [ ] Preserve the previous deployable build and robot configuration export.
  Define rollback as stopping the robot, restoring approved software and
  settings, and repeating an approved low-speed connection/stop check.
- [ ] Block release on unresolved stale-command replay, stop precedence,
  manual-override failures, unexplained velocity mapping, or incomplete
  protective and stopping acceptance.
- [ ] Record final settings, build identity, test results, measured acceptance
  thresholds, remaining limitations and operator handoff. Keep raw captures,
  audits and connection credentials private and out of source control.