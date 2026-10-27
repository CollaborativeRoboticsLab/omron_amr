# `amr_core` Parameters

The default hardware parameters live in [amr_ros/config/parameters.yaml](../amr_ros/config/parameters.yaml).
The LD250 wrapper launch uses [amr_ros/config/ld250_parameters.yaml](../amr_ros/config/ld250_parameters.yaml) as its base file.

[amr_ros/launch/amr_core.launch.py](../amr_ros/launch/amr_core.launch.py) accepts:

- `params_file`: the base parameter file passed to `amr_core`
- `extra_params_file`: an optional overlay file applied after `params_file`

## `robot.*`

- `robot.ip`: target robot IP address.
- `robot.port`: controller port used by the libaria client.
- `robot.user`: login user for the robot controller.
- `robot.password`: login password for the robot controller.
- `robot.protocol`: robot protocol profile, for example `6MTX`.

## `status.*`

- `status.publish`: enables the status interface.
- `status.topic`: topic for the published AMR status message.
- `status.battery_topic`: topic for the battery state publisher.
- `status.publish_period_ms`: status polling and publish period in milliseconds.

## `laser.main_laser.*`

- `laser.main_laser.enabled`: enables the primary front laser publisher.
- `laser.main_laser.topic`: output topic, normally `/scan`.
- `laser.main_laser.frame_id`: frame used in the published scan.
- `laser.main_laser.request`: libaria request name used to subscribe to the laser packet.
- `laser.main_laser.request_period_ms`: packet request period in milliseconds.
- `laser.main_laser.angle_min`: minimum scan angle in radians.
- `laser.main_laser.angle_max`: maximum scan angle in radians.
- `laser.main_laser.angle_increment`: fallback angle increment in radians.
- `laser.main_laser.range_min`: minimum valid range in meters.
- `laser.main_laser.range_max`: maximum valid range in meters.

## `laser.low_laser.*`

- `laser.low_laser.enabled`: enables the low front laser publisher.
- `laser.low_laser.topic`: output topic, normally `/scan_low`.
- `laser.low_laser.frame_id`: frame used in the published scan.
- `laser.low_laser.request`: libaria request name used to subscribe to the laser packet.
- `laser.low_laser.request_period_ms`: packet request period in milliseconds.
- `laser.low_laser.angle_min`: minimum scan angle in radians.
- `laser.low_laser.angle_max`: maximum scan angle in radians.
- `laser.low_laser.angle_increment`: fallback angle increment in radians.
- `laser.low_laser.range_min`: minimum valid range in meters.
- `laser.low_laser.range_max`: maximum valid range in meters.

The published angle increment is inferred from the packet when enough points are available. The configured increment is only used as a fallback.

## `driver.*`

- `driver.odom_topic`: odometry topic name.
- `driver.cmd_vel_topic`: velocity command topic consumed by `amr_core`.
- `driver.stop_topic`: stop command topic.
- `driver.publish_odom`: enables odometry publication from `amr_core`.
- `driver.publish_robot_tf`: enables the `odom -> base_link` TF publisher.
- `driver.odom_frame`: frame id assigned to published odometry.
- `driver.base_frame`: child frame id assigned to published odometry and TF.
- `driver.expected_cmd_vel_freq`: deprecated compatibility parameter, still accepted but no longer controls a watchdog timer or transmission rate.
- `driver.send_frequency`: maximum normal drive packet generation rate in Hz, default `20.0`. Must be finite, positive, and representable by the steady clock. Delayed networking cycles do not produce catch-up bursts. A newly requested stop bypasses this rate gate; failed stop submissions retry at this rate.
- `driver.tcp_queue_max_packets`: maximum pending finalized TCP packets, default `32`. Must be positive. Applies to control and motion packets together.
- `driver.tcp_queue_max_bytes`: maximum retained packet bytes and maximum SDK staging batch size, default `65536`. Must be positive. Partially written packets retain their full byte budget until complete.
- `driver.tcp_queue_max_age_sec`: absolute maximum pending TCP packet age, default `0.1` seconds. Must be finite, positive, and no greater than `cmd_vel_timeout_sec`. Staging time counts toward this budget; partial-write progress does not refresh it. This is a separate transport budget, not an end-to-end command-age guarantee.
- `driver.min_linear_speed`: minimum linear speed clamp in mm/s.
- `driver.max_linear_speed`: maximum linear speed clamp in mm/s.
- `driver.min_angular_speed`: minimum angular speed clamp in deg/s.
- `driver.max_angular_speed`: maximum angular speed clamp in deg/s.
- `driver.drive_throttle_pct`: throttle scaling applied to outgoing drive commands.
- `driver.ratio_mapping`: `legacy` (default, warns that it is uncalibrated) or opt-in `aria_ratio_v1`. The latter implements the bundled SDK model, not an automatically verified contract with the installed firmware.
- `driver.calibration.confirmed`: must be `true` to use `aria_ratio_v1`. Operator attestation that the installed firmware formula, units, packet schema, and effective settings have been checked; this is not physical tracking certification.
- `driver.calibration.source`: required nonblank export/revision identifier in calibrated mode. Include robot identity, firmware revision, capture date, and configuration hash. Logged at startup; never include credentials.
- `driver.calibration.full_throttle_forward_mm_s`: resolved firmware forward speed at full ratio and 100% throttle, positive mm/s. Separate from the permitted `max_linear_speed`.
- `driver.calibration.full_throttle_reverse_mm_s`: resolved firmware reverse speed magnitude at full ratio and 100% throttle, positive mm/s. Separate from the permitted negative `min_linear_speed`.
- `driver.calibration.rotation_stopped_deg_s`: resolved positive turning gain for zero translation ratio, in deg/s.
- `driver.calibration.rotation_full_forward_deg_s`: resolved positive full-forward turning gain, in deg/s.
- `driver.calibration.rotation_full_reverse_deg_s`: resolved positive full-reverse turning gain, in deg/s.
- `driver.calibration.translation_dead_zone_pct`: actual firmware translation dead zone, finite in `[0, 100)`. Default `-1.0` is deliberately invalid until supplied.
- `driver.calibration.rotation_dead_zone_pct`: actual firmware rotation dead zone, finite in `[0, 100)`. Default `-1.0` is deliberately invalid until supplied.
- `driver.calibration.feedback_timeout_sec`: maximum accepted body-speed feedback age, default `0.2` seconds, finite and positive. Steady-clock age at or beyond the limit rejects a speed-dependent turn.
- `driver.unsafe_drive`: requests libaria unsafe drive mode. Kept `true` intentionally so Nav2 owns normal software obstacle avoidance without competing robot-side lidar/sensor motion limiting. This is separate from emergency stops and independent protective functions, whose firmware behavior must be verified.
- `driver.mode_check_period_sec`: minimum interval between `getSafeDrive` queries in seconds, default `0.5`. Must be finite and positive. Only one query is outstanding at a time.
- `driver.mode_confirmation_timeout_sec`: maximum age of a matching mode confirmation in seconds, default `2.0`. Also bounds the startup mode handshake. Must be finite and greater than `mode_check_period_sec`. Uses steady time, independently of ROS executor progress.
- `driver.cmd_vel_timeout_sec`: maximum accepted-command age in seconds, default `0.2`. Must be finite, positive, and representable by the steady clock. Checked in the networking callback independently of ROS time and executor timers. At or beyond this age, the sender invalidates movement and requests stop mode.

### Command and Stop Behavior

#### Docking Readiness Gate

[docking_interface.hpp](../amr_core/include/amr_core/interfaces/docking_interface.hpp)
provides a header-only `amr_core::DockingInterface` shared with `DriverInterface`.
It starts `Unknown`. Ordinary velocity is admitted only in `Ready` with fresh
verified `Undocked` feedback. Docked, undocking, cable-connected, unknown, stale,
and failed states inhibit velocity. Receiving `cmd_vel` never requests undocking.
Commands received while inhibited are discarded, not buffered for departure.
A readiness transition invalidates an accepted target even if readiness returns
before the next network cycle; fresh velocity input is required after the
resulting stop submission. Existing explicit stops remain available.

`CoreNode` owns the relative `undock_robot` ROS action server using the installed
Jazzy `nav2_msgs/action/UndockRobot` contract, goal handles, feedback timer,
and results. `DockingInterface` owns the ROS-independent operation and protocol
handling; there is no separate docking action interface header. `DriverInterface`
attaches it to the existing drive connection and calls its polling/submission
cycle from the network owner. ROS callbacks only reserve, begin, or cancel an
operation; they never send robot packets directly.

Read-only discovery on 2026-10-27 confirmed `undock` takes no arguments and
`dockInfo` returns state (0 Undocked, 1 Docking, 2 Docked, 3 Undocking), a forced
docking flag, and a 16-bit shutdown countdown. The live read reported Docked,
not forced, and countdown zero. The first live attempt failed as recorded below. The conflicting
state description in `dockInfoChanged` is not used.

- `docking.contact_dock_confirmed`: default `false`. Operator attestation that
	no manually attached charging cable is present. Required to enable ordinary
	velocity from Undocked feedback after startup. The four-byte protocol cannot
	establish cable absence; do not infer it from docking state.
- `docking.undock_timeout_sec`: default `30.0`, positive finite steady-clock
	maximum deadline. A goal's `max_undocking_time` must be positive and finite;
	the smaller of the goal duration and this configured limit is enforced.
	This is a policy default, not a firmware-certified duration.
- `docking.exclusive_control_confirmed`: default `false`. Operator attestation
	that other motion sources are inactive; required separately from the action
	goal. This is not automatic command-source arbitration or a protective interlock.
- `docking.dock_type`: default `omron`. A goal may leave `dock_type` empty for
	this sole firmware adapter, or specify the configured type. Other types are rejected.

An undock goal requires both `docking.contact_dock_confirmed` and
`docking.exclusive_control_confirmed` to be true, unforced Docked feedback, no shutdown
countdown, confirmed drive mode, and at least 500 ms of fresh zero-velocity
telemetry. Operator confirmation is not automatic arbitration of other clients.
Only one goal can reserve the interface. Network preflight checks repeat before
the single request. Submission alone is not success: fresh Undocked status and
settled post-request zero-velocity telemetry are required. No retry is made.
The Jazzy action feedback is empty. Success returns `success: true` and error
code `NONE`; failed or canceled operations return `success: false`,
`FAILED_TO_CONTROL`, and an explanatory `error_msg`. Invalid or unsafe goals
are rejected without sending a firmware undock request. Independent read-back
and physical operator observation remain separate acceptance evidence.

This is standard Nav2 action API compatibility backed by the robot's firmware,
not an instance of the Nav2 docking server or its velocity controller. Do not
run both servers at the same resolved action name or let another docking
controller command motion concurrently. The custom `amr_msgs/Undock` contract
has been removed; there is no legacy endpoint or message compatibility shim.

Cancellation suppresses a pending departure, latches velocity inhibition, and
queues `stop`. Its result distinguishes observed stationary feedback from an
unconfirmed stop. Firmware cancellation semantics and physical stopping remain
unverified; cancellation is not a protective stop. Failure, feedback loss, or
deadline expiry also inhibits control. Recovery requires operator review rather
than automatic rearming. Without supported services or cable attestation, the
default driver remains inhibited. Never bypass it with synthetic feedback.

The feedback timeout is one second. `withReadiness()` serializes short,
nonblocking velocity admission/submission with feedback updates; its callback
must not reenter the interface. This software gate cannot recall already queued
or transmitted packets and is not an independent protective stop.

**Live undocking checkpoint, 2026-10-27 06:15:29 UTC:** one operator-authorized
goal was attempted with contact docking/no cable, clear path, accessible E-stop,
and other motion sources inactive confirmed. The action timed out and returned
ABORTED / `FAILED_TO_CONTROL` (905): stop requested but not confirmed. Independent
read-back and a subsequent final check reported state 3 (Undocking), not forced,
no shutdown countdown, unchanged pose (1732, -1145 mm; -95 degrees), and zero
reported velocities. The operator confirmed the robot stayed on the dock and
was safely stopped. This is a FAILED/UNCERTAIN maneuver, not successful departure
or verified firmware cancellation. Do not retry until the active docking state
and motion inhibition are diagnosed and cleared through supported procedures.

The audit is `build/amr_core/undocking/undock-once-20261027T061529Z.json` in the
workspace. The test core process exited; no background controller remains.
Startup requested the intentionally configured unsafe drive: live read-back
changed from `SAFE_DRIVE 1` before the test to `SAFE_DRIVE 0` afterward. No mode
restoration was attempted. Calibration settings were not changed. The test
uses a private, namespaced YAML copy so the isolated node loads the correct
connection parameters without putting credentials in command arguments.

The driver subscribes to `geometry_msgs/msg/Twist` using reliable, depth-one
history. Only `linear.x` and `angular.z` are supported. Non-finite fields or
unsupported components larger than `1e-6` are rejected without refreshing command
freshness. The newest accepted command replaces the previous command atomically.

A fresh zero command sends zero ratios; it does not request server stop mode.
Zeros expire like other commands if the publisher stops. An explicit stop or
command expiry invalidates the stored target. Commands arriving while a stop is
awaiting local submission are rejected. Once the stop is queued, a newly received
valid command can resume streaming; no pre-stop target is automatically replayed.
The server can still enforce its own stopping/mode-lock restrictions. This stop
topic is not a latched emergency stop or a substitute for a physical emergency
stop. A packet already selected for sending can precede the stop request.

### Drive-Mode Confirmation

The drive interface requires advertised `ratioDrive`, `stop`, `setSafeDrive`, and
`getSafeDrive` services. Initialization queues the requested mode, queries its
state, and processes network responses within a bounded handshake. It starts
the background sender only after a valid one-byte reply confirms the requested
safe-drive flag. A missing service, malformed reply, rejected mode, disconnect,
or handshake timeout fails drive-interface initialization without sending drive
commands. Commands received before confirmation are not accepted for later replay.

During operation the networking callback periodically queries the same state.
A mismatched or malformed reply, late confirmation, query-submission failure,
or confirmation timeout invalidates movement and requests a best-effort stop.
The fault is latched: later matching replies or fresh velocity commands cannot
resume movement. Restart the driver after investigating the fault. Unlike an
ordinary command timeout or explicit stop, this is not automatically rearmed.
The driver does not repeatedly send `setSafeDrive` to override an external mode
change, nor silently switch to another avoidance controller.

`getSafeDrive` confirms a software mode flag, not active control ownership,
certified safety, or the robot's physical stopping state. Changes are observable
only when responses arrive; packets already queued cannot be withdrawn by this
gate. The actual firmware query semantics, permissions, physical joystick
arbitration, and independent protective functions still need hardware validation.

The driver does not automatically reconnect. After connection loss, restore the
connection by restarting the driver and supplying fresh commands. Shutdown stops
and joins the networking thread before removing its callback and queues one
best-effort stop before disconnecting. Stop delivery is not guaranteed.

### Calibration and Transport Limitations

Local forward/reverse and angular clamps are enforced before either mapping.
`legacy` preserves the previous conversion using the larger absolute permitted
limit per axis; throttle remains a firmware multiplier. It is not calibrated SI
velocity control. At zero throttle it streams zero ratios rather than a nonzero
target. The hardware YAML files retain this mode with an unconfirmed, unresolved
calibration so guessed archive defaults cannot silently enable a new controller.

`aria_ratio_v1` instead inverts the directional full-throttle speed and throttle
fraction. For curved motion its turning gain uses the magnitude of measured body
speed divided by the full-forward calibration, including during reverse, exactly
as the bundled `ArActionRatioInput::fire()` does. Zero translation uses the
stopped turning gain, not a predicted gain based on the requested speed. Fresh
feedback is required only when the requested curved motion actually uses a
speed-dependent gain. Invalid/truncated telemetry invalidates feedback even when
odometry publication is disabled. This packet layout and formula must be checked
against the installed firmware before opting in.

The inverse does not cancel a dead zone by boosting the requested velocity.
Nonzero translation ratios at or below the configured dead zone and rotation
ratios below their dead zone are unrepresentable. This includes six-significant-
digit SDK wire rounding. Ratios beyond 100%, zero-throttle nonzero targets,
missing/stale/out-of-range required feedback, and invalid turning gains are also
rejected. Rejection invalidates the target and requests a best-effort stop on the
next networking cycle; no nonzero ratio for that target is generated. Fresh input
after stop submission may resume, subject to the same checks. This is different
from a latched mode/transport fault, which requires restart. Warnings identify the
reason. Exact zeros remain zero drive packets and do not require feedback.

These conversions produce nominal ratios, not a closed-loop velocity controller
or a guarantee of tracking through acceleration limits, localization errors,
payload effects, or other firmware constraints. Saturation at the calibrated
ratio envelope rejects the target instead of silently claiming the requested
velocity. No robot-side settings are read back or changed automatically; restart
with an updated export after any configuration change.

The local `DriveClient` adapter routes the SDK sender through a nonblocking local
socket pair from connection setup onward. The networking owner drains each SDK
batch through one bounded `DriveTcpSender` before writing to the robot socket.
Control and motion packets share this path; no independent writer can interleave
partial frames. A staging batch that exceeds its byte/time budget or cannot end
at a packet boundary faults without forwarding that batch. No libaria changes
are required. This adapter uses Linux/POSIX sockets and is specific to the drive
interface; it does not change other clients in the process.

Queue overflow, expiry, write failure, or connection loss discards the local
backlog, closes TCP with zero linger (requesting a reset), stops networking, and
inhibits movement until restart. Fresh commands cannot rearm a failed connection.
A stop cannot be delivered reliably over a failed transport, so fault handling
does not enqueue another stop behind blocked motion. A successful SDK submission
is still only local admission, not a delivery acknowledgement.

Data already accepted by the operating system or robot cannot be recalled by
the queue guard; a reset is not a remote cancellation acknowledgement. The
robot-side teleop timeout and independent protective functions remain necessary,
especially if networking or the whole process freezes. Actual firmware behavior
and physical stopping distances still require supervised hardware validation.

Odometry pose remains relative to the initial position and heading. Linear twist
is published directly in the configured child/body frame rather than rotated by
the initial heading. The installed firmware's packet schema still needs live
verification.

### Captured LD-250 Mapping Blockers

The 2026-10-26 read-only capture reports `TransVelMax=1000` and
`TransNegVelMax=-100` mm/s. Default driver translation caps now match those
values, and the Nav2 smoother spans -0.10 to 1.00 m/s. This does not make legacy
ratio mapping calibrated: its symmetric normalization leaves the whole permitted
reverse range inside the original captured 10% translation dead zone. The live
translation dead zone was subsequently changed to 14% for a persistence test;
the current state and restart result are recorded below.

The matching server's ratio-action descriptions and bundled SDK resolve zero
`FullThrottleForwards` to `TransVelMax` and zero `FullThrottleBackwards` to one
quarter of `TransVelMax`. This gives candidate full-ratio gains of 1000/250 mm/s,
separate from the forward/reverse speed caps. Captured turn gains are 50/25/25
degrees/s. The seven zero-valued teleop limit/acceleration settings still need
their own installed-firmware inheritance confirmation. None of these observations
automatically sets calibration values or confirms the installed control contract.

At the original 10% translation dead zone, even under that candidate model,
nonzero forward targets at or below 0.10 m/s
and reverse targets at or below 0.025 m/s in magnitude fall in the dead zone at
100% throttle. Boosting them would change the requested velocity. Keep calibration
disabled until a vendor-confirmed configuration or absolute-velocity interface
supports the required Nav2 envelope and supervised physical acceptance passes.
The permanent hardware capture artifact includes a conservative
`ratio_mapping_assessment` listing these gates without enabling motion.

### Calibration Handoff: 2026-10-26

Work is paused with the robot at **14% translation / 5% rotation dead zone**.
The original 10% / 5% values have not been restored. At 05:19:52Z, the
operator-approved header-only writer changed only numeric `TransDeadZone`
from 10 to 14. Stationary preflight, configuration comparison, receipt, and
independent read-back passed; no other captured value or property metadata changed.
After the operator restarted the robot, `getConfigValues` at 05:24:47Z again
reported 14% / 5%. Numeric persistence passed for that single restart without
property, save, reload, or restart commands from the tooling.

This is not a calibrated motion profile or a low-speed improvement. At 100%
throttle under the still-unconfirmed candidate gains above, a 14% translation
dead zone gives forward/reverse boundaries of 0.14/0.035 m/s in magnitude.
The whole locally permitted legacy reverse range remains unrepresentable.
`ratio_mapping: legacy`, `calibration.confirmed: false`, and `unsafe_drive: true`
remain unchanged. No motion commands or 5%/3%/1% trials were run.

On resumption, check Ethernet selection, capture fresh read-only settings, and
get fresh operator approval for any restoration, dead-zone reduction, or motion
test. Do not repeat the completed 10-to-14 write automatically. Remaining gates
include firmware inheritance/control semantics, watchdogs, ownership, protective
functions, physical drift/tracking, and exact restoration of property state.
The [hardware calibration procedure](../amr_core/test/README.md) records
the commands and audit artifacts. The native workflow now passes 23 offline
C++ configuration/workflow tests and 38 offline Python regression cases. Raw captures may
contain credentials and stay private.

Configuration and calibration operations are now production headers under
`amr_core/include/amr_core`: a configuration interface for allowlisted reads and
the validated translation write, a protocol utility, and a header-only guarded
calibration utility. A header-only client workflow is called directly by a thin
C++ `main()`; Python is used only for tests. The standalone workflow is native C++:
yaml-cpp loads settings, isolated bounded workers perform network operations,
and exclusive fsynced JSON reports preserve audit evidence. No production Python
script or GTest invocation remains. The workflow, low-level writer adapter,
and separate read-only probe build and
install with `BUILD_TESTING=OFF`; `ros2 run amr_core amr_deadzone_calibration
--help` describes the workflow without connecting. The refactor adds no ROS
node, automatic configuration writes, or unrestricted setter, and changes no
robot settings. Operator writes still require the calibration guards and fresh
approval, not just a successful low-level receipt.

Configuration retrieval now requires `getConfigValues`; older SDK configuration
services and their separate parameter allowlist have been removed. Shared
telemetry decoding and maintenance connection/session helpers live in utility
headers, while consumer-specific stationary and observation acceptance remain
separate. This cleanup does not change `ratio_mapping: legacy` or enable a
calibrated motion profile. All 69 offline driver tests pass with the shared decoder.

### Calibration Capture and Acceptance

**2026-10-27 charging checkpoint:** the operator reported a depleted battery,
connected charging, and powered the robot on. A complete read-only snapshot at
00:46:40Z confirmed translation/rotation dead zones of **14% / 5%** again;
the original 10% / 5% settings have not been restored. Evidence is in
`build/amr_core/calibration/charging-readback-20261027T004640Z.json`.
Charging status was not independently measured. No configuration writes or
motion commands were sent. Before the production refactor, offline builds
passed, and registered core/Nav2
results were 118 passed plus one intentionally disabled live-write case skipped.

All dead-zone changes and physical trials are on hold while charging. After
normal disconnection from charging, refresh the read-only snapshot and obtain
fresh operator readiness/approval before restoring/lowering settings or starting
movement-related tests. Nothing is queued to run automatically; the 5%/3%/1%
trials and physical/safety acceptance remain pending.

1. Keep the ROS driver stopped and motion inhibited under the site's procedure.
	Use vendor-supported read-only configuration export tooling with an operator
	present. Do not launch the full driver to inspect settings: startup requests
	the configured safe-drive mode. Archive firmware/build identity, advertised
	service permissions, and a sanitized effective configuration export with a
	hash. Credentials do not belong in the calibration source field.
2. Capture `FullThrottleForwards`, `FullThrottleBackwards`, `RotAtStopped`,
	`RotAtFullForwards`, `RotAtFullBackwards`, `TransDeadZone`, `RotDeadZone`,
	platform `TransVelMax`/`RotVelMax`, teleop speed/acceleration limits, and the
	teleop loss-of-command timeout. Resolve inherited/zero sentinel settings using
	the installed firmware's documented rules, not guessed SDK defaults. Confirm
	that `updateNumbers` carries body-forward mm/s in the expected field.
3. Confirm with firmware documentation/vendor support that `ratioDrive` uses the
	modeled formula, throttle fraction, dead-zone inequalities, reverse turning
	denominator, control ownership, and watchdog. Determine whether these settings
	are shared with the physical joystick. Do not change them automatically or
	reduce protective functions. If independent, approved autonomous settings
	cannot represent the required low velocities, leave calibrated mode disabled
	and assess a vendor-supported absolute-velocity service separately. Do not
	assume the old SDK's `driveJoystick` service exists.
4. Fill the calibration fields from that export, set a traceable `source`, and
	set `confirmed: true` only after the contract checks above. Choose local/Nav2
	limits within the reachable throttle-dependent envelope and a feedback timeout
	justified by measured telemetry cadence. Then opt in with
	`ratio_mapping: "aria_ratio_v1"`. No numeric profile is auto-selected.
5. In a cleared area with an operator, available emergency stop, conservative
	speed limits, and agreed tolerances, record command/odometry/TF and robot mode.
	Test zero, forward, reverse, stationary turns, and curved targets in both
	directions across approved throttle levels. Compare steady body velocities
	after acceleration settles, including small commands around each dead zone.
	Test several initial headings and record tracking error and packet gaps.
6. Separately supervise input loss, stale/malformed telemetry, explicit stop,
	process freeze, network loss, and restart with fresh input. Record stopping
	latency and distance against limits agreed before testing. Verify joystick
	takeover/deadman release, emergency protection, and Nav2-owned obstacle
	avoidance with `unsafe_drive: true`. Keep release blocked on replay,
	unexpected mode changes, failed override, or unexplained tracking errors.

The supplied 4.1.12 archive contains an LD250 **simulation default**, not a live
export: `DefaultConfig/SimBaseConfig/aramConfig_LD-250.txt`. Its teleop values are
1200/200 mm/s forward/reverse, 50 deg/s stopped turning, 25/25 deg/s moving turning,
and 10%/5% translation/rotation dead zones. They are evidence of configuration
names only and are deliberately not installed into the calibration YAML. At
100% throttle under that model, forward targets at or below 0.12 m/s and reverse
targets at or below 0.02 m/s cannot be reproduced continuously. Live numeric
settings and the later persistence experiment are recorded above; firmware
semantics, joystick sharing, physical tracking, and stopping remain unverified.
Offline software tests send no robot commands. The separately approved live
configuration write sent no motion commands and did not enable calibration.

### Offline Nav2 Integration Checks

The LD250 launch configuration routes both controller and behavior commands into
`cmd_vel_nav`, then the velocity smoother publishes `cmd_vel`. The standalone
and composed node descriptions use the same relative remappings. This removes
the behavior server's direct bypass of smoothing; it does not provide arbitration
between controller and recovery publishers. Nav2 lifecycle/task management and
the actual deployed publisher graph still need verification.

Controller, behavior server, and smoother explicitly use unstamped
`geometry_msgs/msg/Twist`, matching the driver. A localhost-only integration test
configures the installed Jazzy smoother and verifies real Twist delivery and
reliable QoS on its input/output. The driver uses reliable, depth-one input.
This does not certify all deployed publishers, overrides, or other ROS releases.

The smoother stays `OPEN_LOOP` at 20 Hz, matching controller and driver rates.
Its nominal forward/angular limits remain 1.0 m/s and 1.0 rad/s, while reverse is
bounded to -0.2 m/s to match the driver's configured limit. Controller/smoother
acceleration settings agree but remain unvalidated against actual robot
capabilities. Agreement with configured clamps is not evidence of calibrated
physical velocity tracking.

Both Nav2 odometry consumers now default to relative `odom`, matching `/odom`
in the unnamespaced hardware configuration. Hardware command, odometry, stop,
and Nav2 scan topics are relative, so they follow the node namespace. Prefer
the LD250 wrapper for bringup; composed mode expects its component container.

The smoother's 1.0-second input timeout and driver's 0.2-second receive timeout
are separate. While the smoother republishes a retained target, the driver sees
fresh input and its receive timeout will not detect upstream command loss.
Establish an end-to-end stopping budget and measured jitter before changing
either value. Do not increase timeouts to conceal gaps. Joystick/teleop must not
publish independently onto the final command topic during autonomy without an
explicit arbitration policy. These deployment decisions remain open.

Loopback odometry tests cover nonzero position origins, headings 0/+90/-90,
bidirectional wrap across +/-180 degrees, body-frame twist, frame IDs, and
identical TF/odometry timestamps and poses. They test the assumed packet schema,
not the installed firmware or physical heading response.

### End-to-End Command Loss

Two tests run the installed Jazzy smoother and real driver against a fake
localhost ARNetworking server. With 20 Hz smoothing/sending, 1.0 s upstream
timeout, 2.5 m/s2 deceleration, 0.6 m/s input, and 0.2 s driver expiry, one
focused run observed:

| Event | Time From Loss |
| --- | --- |
| First zero ratio after upstream silence | 1.265 s |
| Stop request after upstream silence | 1.447 s |
| Stop request after smoother deactivation | 0.181 s |

These are scheduler-dependent fake-server packet arrival times, not physical
stopping times or guaranteed bounds. During upstream silence, nonzero ratios
continue past the driver's receive timeout because the smoother keeps publishing
its retained target. After its own timeout, it decelerates to zero; when smoother
output then ceases, the driver expires that last input and requests stop. The
test checks no autonomous restart during silence and resumption on fresh input.
The separate smoother-failure case blocks ROS executor progress and confirms
the independent networking watchdog still requests stop. GoogleTest XML stores
these timing properties for later comparisons. Runtime timeouts are unchanged.

### Command Ownership Contract

The current contract is exclusive ownership, not priority-based automatic
arbitration. During autonomy, only the smoother may publish the final driver
command topic; controller and behavior tasks share its input under Nav2 task
management. ROS joystick input must remain disconnected from that input and the
final topic unless manual mode has been explicitly selected.

Handover must be break-before-make: cancel the outgoing task, quiesce its command
sources, clear the smoother's retained target, let output cease, and require
fresh input from the incoming source before re-enabling motion. A driver stop
alone is insufficient while a smoother can immediately republish an old target.
In manual mode, keep autonomous command producers inactive and route the selected
deadman-gated joystick through the smoother, never alongside it on the final
topic. Reverse the sequence to restore autonomy; never replay a saved target.
Physical joystick priority and independent emergency protection remain separate
firmware/hardware acceptance requirements.

This is a deployment requirement, not an implemented mux or mode manager. The
current launch files do not enforce it. Before enabling concurrent manual/Nav2
bringup, implement and test source selection, zero/deadman behavior, retained-
target clearing, and failure handling. Checking publisher counts alone does not
provide arbitration or prevent in-flight commands during handover.

### Namespace Contract

For a namespace such as `/amr1`, controller/behavior output must resolve to
`/amr1/cmd_vel_nav`, smoother output and driver input to `/amr1/cmd_vel`, odometry
to `/amr1/odom`, and the explicit stop input to `/amr1/stop`. Manual inputs must
be separately named until explicitly selected. Nav2 already uses relative
remappings internally, and the hardware YAML uses relative topics too.

The LD250 wrapper passes its `namespace` launch argument to hardware, Nav2, and
optional RViz. For a connected robot, use:

```bash
ros2 launch amr_ros ld250.launch.py namespace:=amr1
```

Omit `namespace` to preserve the unnamespaced behavior. The core launch also
accepts it directly: `ros2 launch amr_ros amr_core.launch.py namespace:=amr1`.
Nodes and components receive the standard ROS namespace; no custom driver
parameter or topic-prefixing code is needed. Parameter files and
`extra_params_file` overlays retain their ordinary node keys (`amr_core`, for
example); launch scopes these files under the selected namespace. Explicit
absolute topics in site overlays remain absolute and bypass namespace isolation.

`/tf` and `/tf_static` are remapped to relative `tf` and `tf_static`, giving each
robot its own TF topics. Frame IDs remain `map`, `odom`, and `base_link`; they
are not automatically prefixed. Consumers must use the same namespaced TF
topics. A shared global TF tree with prefixed frames requires separate coordinated
configuration. Offline tests cover parameter overlays and command/TF topic
isolation using dummy nodes; the live multi-robot graph remains unverified.

## Digital twin overlays

For simulation-authoritative setups, keep the hardware defaults in [../amr_ros/config/parameters.yaml](../amr_ros/config/parameters.yaml) or [../amr_ros/config/ld250_parameters.yaml](../amr_ros/config/ld250_parameters.yaml), then layer a package-specific override file with `extra_params_file`.

The override usually disables:

- `status.publish`
- `laser.main_laser.enabled`
- `laser.low_laser.enabled`
- `driver.publish_odom`
- `driver.publish_robot_tf`

That leaves `amr_core` acting as the command bridge while simulation owns TF, odometry, and sensors.
