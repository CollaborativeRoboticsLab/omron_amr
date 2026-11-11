# Experiment Log

Current findings from the LD-250 work. See [testing](../amr_core/test/README.md) for test
results and commands. These records are not authorization for new robot access,
configuration changes, or movement.

## Outstanding Findings

- When manually parked, the dock's yellow light has sometimes turned on while
  the robot display did not show charging and docking feedback remained
  Undocked. Confirm the robot's displayed charging state and investigate
  inconsistent dock recognition.
- One supervised undock on 2026-11-11 returned success with error code 0;
  read-only feedback reported Docked and stationary before, then Undocked and
  stationary afterward. The operator separately confirmed physical departure
  and stopping. The active feedback timeout was 5.0 seconds; feedback-gap
  timing and the physical effect of lost feedback were not measured. The audit
  retains `physical_departure_confirmed: false` because the operator confirmed
  departure afterward. Post-undock `drive_ready` was true, not a safety guarantee.
- On 2026-11-11, nine read-only hardware checks passed, docking feedback reported
  Docked, and telemetry observed no movement. A separate fault response was
  malformed; the operator reported no faults shown on the robot display.
  Physical charging remains unconfirmed.
- A prior action failed on stale feedback; the full feedback gap, its cause and
  the physical effect of the configured five-second limit remain unverified.
  See [known issues](../known_issues.md). A queued stop is not an acknowledged
  physical stop; do not automatically retry uncertain movement.
- On 2026-11-11, after an authorized stack start, both costmaps subscribed to
  `/scan`, and scan data and transforms were available. With a stationary test
  object, the local voxel layer marked 13 cells and its combined costmap showed
  747 inflated cells; both returned to zero after removal. In a separate test,
  16 newly marked global obstacle-layer cells near the scan returns cleared
  after removal. The combined global costmap had 1,039 cost increases after
  placement and 1,036 decreases after removal. A further stationary test marked
  13 global obstacle cells, all cleared after removal; 336 nearby nonlethal
  combined-cost cells rose and fell while their static and obstacle-layer
  values remained unchanged, consistent with global inflation. `drive_ready`
  was false while docked; this is not a protective function.
- The active SLAM mapping-mode map contained 86 occupied cells at 0.05 m
  resolution. Map-to-odom and odom-to-base transforms were available, and
  odometry reported zero linear and angular velocity. The operator confirmed
  that the stationary map and robot pose matched the surroundings. Route-wide
  map coverage and localization during movement remain unverified. RViz logged
  a shader error during map display initialization; other display effects have
  not been assessed.
- MOMA's AMR hardware and Nav2 launch wiring and joystick topic alignment were
  build- and launch-checked offline in a separate session. Integrated hardware
  operation and the combined robot's footprint and costmaps remain unverified.
- During a six-second stationary observation, no messages arrived on either
  velocity command topic, odometry reported zero, and `drive_ready` stayed
  false. The graph showed one smoother publisher to the driver, six Nav2
  controller/behavior publisher endpoints upstream, and no active teleop
  publisher. This does not establish exclusive control or physical stopping.
- On 2026-11-11, one authorized 0.5-second forward pulse at a requested 0.2 m/s
  used the existing driver. The audit recorded ten commands, approximately
  0.058 m of odometric displacement and 0.129 m/s peak reported velocity;
  readback reported Undocked and stationary. The operator confirmed forward
  movement and physical stopping. The requested stop was not acknowledged and
  odometry is not independent ground truth. The existing-driver runner checks
  for quiet preflight traffic and unexpected command values, but cannot detect
  identical commands from another publisher or enforce exclusive control.
- On 2026-11-11, an authorized 0.5-second left pulse at a requested 0.5 rad/s
  produced about 12 degrees of odometric rotation. A separately authorized left
  turn then targeted the remaining 168 degrees with a 12-second command limit.
  The target was reported at 169 degrees after about 7.46 seconds; final
  odometry showed 178 degrees for that turn, or 190 degrees across both turns.
  Reported angular velocity remained nonzero for about 0.56 seconds after the
  stop request. Readback reported Undocked and stationary, and the operator
  confirmed physical turning and stopping, but did not measure the angle.
  Stop delivery was not acknowledged. Do not retry the target turn until its
  overshoot and stopping behavior are investigated.
- Isolated fake-server checks passed: all 122 core GoogleTests, the remaining
  three registered core tests, Nav2 integration, and SLAM map padding. The core
  test target's 60-second timeout was raised to 180 seconds after a timeout; it
  then passed under CTest. After the LD250 source reverse clamp was changed to
  -0.1 m/s, all 21 Nav2 contract checks passed; that source change is not
  installed or active.
- Firmware ratio semantics, effective limits, odometry accuracy, controller
  ownership, joystick takeover, watchdog and physical stopping, and independent
  protective functions still need validation. A -0.2 m/s reverse request
  produced approximately -0.048 m/s reported velocity; this is not physical
  ground truth. The active driver reverse clamp is -0.2 m/s, while the source
  clamp and Nav2 smoother use -0.1 m/s. Legacy mapping remains uncalibrated and
  `unsafe_drive` remains true.
- The 2026-11-11 read-only configuration reported a -0.1 m/s robot reverse limit
  and 14% translation dead zone. The existing offline assessment estimates a
  0.14 m/s legacy command dead-zone boundary, placing all Nav2 reverse requests
  inside it under that candidate model. Seven assessment tests passed; the
  installed-firmware semantics and physical response remain unverified.
- The changed teleop timeout's reboot persistence and physical watchdog effect
  remain unverified. Nonempty fault replies may not match the advertised schema;
  do not infer fault status from a rejected reply.
- A later 2026-11-11 read-only `getFaults` query returned a zero-byte payload,
  although the advertised schema begins with a two-byte count. No valid reply
  followed during a four-second observation. The active fault monitor was
  enabled; automatic and service-based acknowledgement were disabled. The
  decoder and fake-server tests reject empty replies without acknowledgement.
  Fault status and the installed firmware's reply semantics remain unknown.

Private audits are retained under the workspace's `build/amr_core/calibration/`,
`build/amr_core/undocking/`, and `build/amr_core/drive-pulse/` directories.
Raw captures and full configuration exports can contain credentials and must
remain private and out of source control.