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

Do not automatically retry or assume that a stop request canceled firmware
undocking. Inspect the current robot state and faults and use supported recovery
procedures before further movement. See the [experiment log](docs/experiement_log.md)
and [test results](amr_core/test/README.md) for safeguards and recorded results.