# Known Issues

## Undocking Feedback

On 2026-11-10, one supervised `undock_robot` attempt physically departed and
the operator reported the robot stationary afterward. The action returned
`FAILED_TO_CONTROL` (905) after stale docking feedback at the previous one-second
limit. Its result recorded a maximum observed feedback gap of 1,092 ms, then
fresh Undocked and stationary feedback. A stop request was queued; this does not
prove its delivery or that it caused the robot to stop. Error 905 is an action
result, not a confirmed firmware fault. The private audit is
`build/amr_core/undocking/undock-once-20261110T065607Z.json`; its physical
departure flag was not updated, so the operator's report is separate evidence.

The cause and full duration of robot feedback gaps remain unknown. The LD250
configuration now sets `driver.undocking.feedback_timeout_sec` to 5.0 seconds;
the code default remains 1.0 second. The physical behavior of this limit remains
uncharacterized.
A longer limit delays detection of lost feedback and may allow
undocking to continue longer without a fresh state. Reply receipt does not prove
the age of the robot state or identify which poll produced it. Feedback timing
and the physical effect of lost feedback remain unmeasured.

When manually parking, the operator observed that the dock's yellow charging
light can turn on without the robot display changing to charging; `dockInfo`
remained Undocked in that condition. Verify proper parking and the robot display,
not the dock light alone. Do not automatically retry an uncertain departure or
assume a queued stop canceled firmware undocking. Inspect physical state and
faults and follow supported recovery procedures before further movement.

## Navigation and Safety

A namespaced robot must configure both costmap observation sources to its own
scan topic. Do not rely on obstacle avoidance until the full setup and physical
behavior are verified.

Localization during movement and map coverage across the intended route remain
unverified. RViz logged a shader error during map display initialization; its
effect on other displays has not been assessed.

Velocity mapping remains `legacy` and calibration is unconfirmed. Command
arbitration, joystick takeover, watchdog and physical stopping behavior,
independent protective functions, and odometry accuracy remain unverified.
On 2026-11-11, a supervised left turn toward 180 degrees used a short pulse
followed by a separately authorized target turn. Odometry reported about 190
degrees total, including about 9 degrees after the second stop request; reported
angular velocity remained nonzero for about 0.56 seconds afterward. The
operator confirmed the robot turned and stopped, but did not independently
measure the angle. Stop delivery was not acknowledged. Investigate the observed
overshoot and stopping behavior before another target turn.
The active driver reverse clamp is -0.2 m/s; the Nav2 smoother and the
2026-11-11 read-only robot configuration report -0.1 m/s. The source LD250
driver clamp was changed to -0.1 m/s, but is not installed or active; the
captured 14% translation dead zone gives an
estimated legacy command boundary of 0.14 m/s, so all permitted Nav2 reverse
requests may fall within the dead zone. Firmware ratio semantics and physical
response are unverified; do not treat this estimate as measured behavior.
`unsafe_drive: true` is not a safety guarantee. Nonempty firmware fault replies
have included undocumented bytes and may be rejected; fault status cannot be
inferred from an invalid query. On 2026-11-11, a read-only query returned an
empty payload despite the advertised two-byte count, with no valid follow-up
within four seconds. The active fault monitor was enabled, but automatic and
service-based acknowledgement were disabled. The operator previously reported
that the robot display showed no faults; the API fault status remains unknown.
See the [remaining work](proposed_changes.md)
and [experiment log](docs/experiement_log.md).