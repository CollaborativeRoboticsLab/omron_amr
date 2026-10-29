# Known Issues

## Undocking issue

Two supervised firmware-backed `undock_robot` action attempts on 2026-10-27
timed out without the robot leaving the dock. Both returned `FAILED_TO_CONTROL`
(905), with stop unconfirmed. After each attempt, `dockInfo` reported Undocking
(state 3), while odometry reported zero velocity and an unchanged pose. Zero
velocity does not prove that firmware undocking was canceled.

The operator confirmed the robot did not move and subsequently unplugged the
charger and rebooted the robot. Later preflights reported Undocked and
supervised drive tests produced movement. The cause of the failed undocking
remains unknown; motor power was reported enabled before the second attempt,
but fault status was unavailable. Motor power alone does not establish that
motion is permitted.

On 2026-10-28, a separately authorized 0.1-second forward pulse was attempted
from the dock using the experimental bypass. The operator observed no movement
and reported a similar "decoupling" message. The audit recorded zero odometric
displacement and unacknowledged stop delivery. Post-test and later read-only
checks reported Undocking (state 3) and stationary telemetry; the cause of the
state change and firmware cancellation remain unknown. The private audit is
`build/amr_core/drive-pulse/forward-stream-20261028T050327Z.json`.

A subsequent read-only check after reconnection still reported Undocking
(state 3) and stationary telemetry. The motor-state query returned a value,
but no usable fault response was obtained. The active map was `uc_room1.map`;
no map transfer or activation was attempted during this check.

After the operator rebooted the robot, a read-only check reported Docked
(state 2), stationary telemetry, and `uc_room1.map` still active. The fault
response reported one fault and provisionally decoded as `IrregularShutdown`,
but included undocumented trailing bytes and was rejected as malformed. No map
was changed.

On 2026-10-29, the operator confirmed `IrregularShutdown` in MobilePlanner and
authorized one named acknowledgement through the ROS service. The request was
locally queued; a subsequent valid fault query reported zero faults. Docking
status remained Docked and telemetry remained stationary. This does not resolve
or explain the earlier undocking failures.

Do not automatically retry or assume that a stop request canceled firmware
undocking. Inspect the current robot state and faults and use supported recovery
procedures before further movement. See the [experiment log](docs/experiement_log.md)
and [test results](amr_core/test/README.md) for safeguards and recorded results.