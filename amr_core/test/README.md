# Robot Hardware Tests

These are opt-in hardware acceptance tests, separate from the fake-server and
unit tests registered with `colcon test`. Running pytest without `--robot` skips
all hardware cases without reading credentials or connecting. Results describe
the connected robot at the time of the run, not general firmware certification.
Robot-free regression cases in `test/test_hardware_capture.py` verify
capture persistence, credential exclusion, failure/authentication handling, and
rejection of movement requests, plus conservative mapping preflight assessment.
Those cases are registered with `colcon test`.
The robot-free `test_hybrid_config` target checks request bytes, bounded chunk
reassembly, malformed records, source slots, and captured subnormal doubles.

## Offline Docking Foundation

The header-only docking state machine and localhost driver gating tests are
registered in `test_drive_interface`. They verify explicit-only departure,
fresh-feedback readiness, cable/unknown inhibition, one-attempt submission,
failure/timeout latching, default startup inhibition, and no stale-command replay.
The injected test backend never contacts the robot. Transport loopback tests also
verify confirmed reservation, settled departure, and cancellation before begin
without sending an undock request. See the
[docking readiness gate](../../docs/parameters.md#docking-readiness-gate)
for the protocol adapter and CoreNode-owned ROS action. The first supervised
hardware attempt on 2026-10-27 failed: the robot stayed on the dock, the action
timed out, and firmware state remained Undocking despite zero reported velocity.
Physical departure and firmware cancellation are not accepted. The production driver remains inhibited
without fresh docking readiness and operator cable attestation.

The real-core localhost action test uses `nav2_msgs/action/UndockRobot` at
`undock_robot`. It covers false-by-default authorization, dock-type and timeout
validation, standard success/error results, and enforcement of a short goal
deadline. ROS action ownership remains in CoreNode, not the docking header.
The separate `test_undocking.py` movement test requires both `--robot` and
`--undock-once`; never enable these flags for offline validation. Its audit
uses the standard result fields and independent post-test status/telemetry.

## Production Configuration and Calibration

Functional operations no longer live in the test tree or execute through GTest:

- [configuration_protocol.hpp](../include/amr_core/utils/configuration_protocol.hpp)
   owns the allowlisted protocol encoding, decoding, and bounded chunk assembly.
- [drive_telemetry.hpp](../include/amr_core/utils/drive_telemetry.hpp)
   owns checked `updateNumbers` and safe-drive reply decoding. Driver, status,
   probe, and calibration consumers share it but retain their own policies.
- [robot_session.hpp](../include/amr_core/utils/robot_session.hpp)
   owns maintenance connection validation, protocol enforcement, transport
   defaults, and exception-safe client lifetime through the shared SDK runtime.
- [configuration_interface.hpp](../include/amr_core/interfaces/configuration_interface.hpp)
   provides `amr_core::ConfigurationInterface`, constructed with a caller-owned,
   connected `DriveClient`. `getValues()` returns the allowlisted drive snapshot;
   `setTranslationDeadzone(expected, target)` performs only the validated numeric
   translation write and receipt check. There is no unrestricted setter.
- [deadzone_calibration.hpp](../include/amr_core/utils/deadzone_calibration.hpp)
   retains header-only calibration policy: stationary preflight, expected values,
   complete before/pre-write/after comparisons, and at most one write attempt.
   Operator workflows must use this guarded layer, not the low-level setter alone.
- [deadzone_calibration_client.hpp](../include/amr_core/utils/deadzone_calibration_client.hpp)
   implements CLI validation, YAML loading, bounded network workers, and durable
   audit reporting. The C++ entry point only calls `runCalibrationClient()`.

The caller must serialize access to the client and keep it alive throughout each
operation. These synchronous operations pump its network loop; do not invoke
them concurrently with another owner. The low-level setter does not establish
stationarity, command-source exclusivity, atomic compare-and-set semantics, or
effective application from a receipt alone. Read-back remains mandatory.

The native C++ `amr_deadzone_calibration` workflow loads connection settings
with yaml-cpp, calls the header-only calibration operations directly, and owns
exclusive audit files and independent read-back. There is no Python production
workflow or executable chaining. `amr_hardware_probe` remains a separate read-only
production executable; `amr_deadzone_writer` remains a low-level stdin adapter.
All three install without `BUILD_TESTING`; no new ROS node or driver-startup
configuration write was added. Header dependencies are installed and exported.

```bash
colcon build --packages-select amr_core --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
ros2 run amr_core amr_deadzone_calibration --help
```

The workflow uses the installed `amr_ros` configuration by default. Use `--params`
for an explicit file, including when invoking the build executable directly.
The former Python-only `--probe`, `--writer`, and `--test-binary` options are no
longer accepted. Help opens no robot connection. Credentials are loaded from
the YAML file and never placed in argv or reports.

Each network operation runs in an isolated child process: read/observe workers
have a 15-second deadline and the single write worker has a 45-second deadline.
Timed-out/interrupted workers are terminated and reaped. The write is never
retried; a new read-only connection attempts independent read-back even after
an uncertain failure. Audit files use exclusive creation, owner-only permissions,
and fsync, with a pending record saved before any network access. SIGINT/SIGTERM
marks the operation unsuccessful; read-back and final audit still run where
possible. Forced process termination or audit-storage failure can leave an
unfinished artifact: never interpret it as proof no write occurred.

Native workflow validation passed 23 C++ configuration/workflow tests and 38
Python regression cases. Python remains test tooling only; the executable tests
use localhost, never the physical robot. The former disabled live-write GTest
was removed; these results are not live configuration or physical-motion acceptance.
All 69 driver tests also pass with the shared telemetry decoder. Legacy
configuration-service fallback has been removed; the distinct uncalibrated
`ratio_mapping: legacy` motion setting is unchanged, not enabled calibration.

## Charging Checkpoint: 2026-10-27

The operator reported that the battery had depleted, connected charging, and
powered the robot on. No motion trials or dead-zone changes are to run while
the robot is charging. Charging/battery state is operator-reported, not measured
or independently certified by this configuration check.

At 00:46:40Z, a complete read-only `getConfigValues` snapshot again confirmed
**translation dead zone 14% / rotation dead zone 5%**, matching the last handoff.
Artifact: `build/amr_core/calibration/charging-readback-20261027T004640Z.json`.
The initial connection attempt timed out before the operator powered the robot
on; the explicitly requested retry passed. No settings, mode, or motion commands
were sent, and the original 10% / 5% settings remain unrestored.

Before the production refactor, offline rebuilds of `amr_core`, `amr_nav2`, and
`amr_ros` passed. Registered
results were 107 core cases (zero failures, one disabled live-write case skipped)
and 12 Nav2 cases (zero failures): 118 passed, one skipped. The skip is intentional
and does not count as configuration-write or physical-motion acceptance.

When charging is complete and the robot is disconnected using the site's normal
procedure, obtain fresh operator readiness/approval and read back settings again
before any trial. The first lower setting must be approved explicitly; the
proposed 5%, 3%, 1% trials remain pending. Announce movement-related testing before
starting it. No writes, restorations, motion trials, or restarts are queued to
run automatically. Calibration remains disabled and the firmware/safety gates
below remain open.

### Offline Candidate Trial Envelope

While charging, candidate mapping tests cover the current 14% setting and the
proposed 5%, 3%, and 1% translation trials using the SDK's real in-memory packet
serializer. No robot connection or configuration write is involved. At 100%
throttle with the still-unconfirmed 1000/250 mm/s forward/reverse ratio gains:

| Translation Dead Zone | Forward Boundary Magnitude | Reverse Boundary Magnitude |
| --- | --- | --- |
| 14% (last verified setting) | 140 mm/s | 35 mm/s |
| 5% (pending trial) | 50 mm/s | 12.5 mm/s |
| 3% (pending trial) | 30 mm/s | 7.5 mm/s |
| 1% (pending trial) | 10 mm/s | 2.5 mm/s |

Nonzero translation targets exactly on these boundaries are rejected. Targets
just above them can also be rejected when six-significant-digit wire formatting
rounds their ratios back onto a boundary. Tests verify both directions, rejection
without boosting, and acceptance above the rounded boundary. Rotation stays at
5%; under the stopped-turn candidate gain of 50 degrees/s, its boundary remains
2.5 degrees/s in either direction, with equality accepted by the modeled rule.

These are model boundaries, not measured minimum velocities or automatically
applied settings. A no-input creep observation is separate from commanded
low-speed tracking. Passing offline tests proves neither the installed firmware
contract nor safe motion. No physical trial or dead-zone reduction has occurred.

## Session Handoff: 2026-10-26

At the end of that session, work was paused at **translation dead zone 14%
and rotation dead zone 5%**; the original 10% / 5% settings have not been restored.
Do not assume that the historical exports or baseline examples describe the
current robot. This higher translation dead zone is a persistence experiment,
not an accepted Nav2 operating configuration or an improvement to low-speed control.

- At 05:19:52Z, one operator-approved numeric `TransDeadZone` change from 10 to
   14 passed stationary preflight, repeated configuration checks, receipt checking,
   full 22-parameter post-write comparison, and independent read-back. Rotation
   and all other captured values/metadata were unchanged. The header-only writer
   sent no property, mode, motion, save, reload, or restart commands.
- The operator restarted the robot. At 05:24:47Z, read-only `getConfigValues`
   confirmed 14% / 5%, demonstrating persistence across that restart. The first
   connection timed out because the host Ethernet profile was not selected;
   verification passed after the operator corrected it.
- Evidence: `build/amr_core/calibration/write-translation-10-to-14-20261026T051952Z.json`
   and `build/amr_core/calibration/post-restart-20261026T052446Z.json`.
   Artifacts belong in the build directory; raw captures remain private/out of Git.
- Validation: 14 offline configuration GTests and 25 offline Python safeguards
   passed. The disabled live adapter was also verified to skip before connecting
   without its separate approval token. No motion commands or low-dead-zone trials
   were run; absence of creep at 14% has not been physically accepted.
- Runtime configuration remains `ratio_mapping: legacy`,
   `calibration.confirmed: false`, and `unsafe_drive: true`. No calibrated profile
   or motion-readiness claim follows from the configuration/persistence checks.

To resume, first confirm the host Ethernet profile and obtain a fresh read-only
configuration snapshot. Review the current settings with the operator and get
fresh approval before restoring 10%, lowering a dead zone, or running any motion
test. The proposed 5%, 3%, 1% trials remain pending, one setting at a time, with
physical observation and odometry recording. Firmware inheritance/ratio semantics,
control ownership, watchdog behavior, protective functions, and physical tracking
remain acceptance gates. Do not replay the completed 10-to-14 write automatically.

## Categories

| Marker | Current Coverage | Robot Effects |
| --- | --- | --- |
| `connectivity` | TCP connection and authenticated ARNetworking handshake | Opens and closes connections |
| `config` | Advertised services, safe-drive flag, stationary telemetry, allowlisted configuration capture | Reads mode/telemetry and supported configuration services only |
| `movement` | Acceptance plan below; automated cases not implemented yet | No movement test is currently runnable |

The read-only probe never requests `ratioDrive`, `stop`, or `setSafeDrive`.
It does not start ROS bringup, acquire control, change the safe-drive flag,
enable calibration, or modify robot configuration. The stationary test expects
at least two valid samples over four seconds, with reported forward/lateral
speed no greater than 5 mm/s and angular speed no greater than 1 degree/s.
The decoder currently checks the driver's assumed packet layout; passing does
not independently prove its physical units or calibration.

## Run

From the colcon workspace root:

```bash
colcon build --packages-select amr_core --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
python3 -m pytest src/omron_amr/amr_core/test/hardware --robot \
  -m 'connectivity or config' -v \
  --junitxml=build/amr_core/hardware-readonly-results.xml
```

Select just one category with `-m connectivity` or `-m config`. Use
`--robot-params /path/to/parameters.yaml` to select another connection file,
or `--robot-probe /path/to/amr_hardware_probe` for another build location.
Parameter files use ordinary `amr_core.ros__parameters.robot` YAML keys, not a
namespace-rooted launch-generated file. Credentials travel to the probe over
stdin, not command-line arguments. SDK logging is disabled; reports include
only allowlisted diagnostic fields. Do not enable pytest `--showlocals`, which
can expose credentials in fixture locals. Reports belong in the build directory,
not source control. Authentication has a 15-second process deadline; TCP has a
5-second deadline. No failed test retries the robot automatically.

## Configuration Capture

The config category also saves a sanitized snapshot to
`build/amr_core/hardware-config.json`; override it with `--robot-capture`.
It includes a UTC timestamp, endpoint/protocol, probe result, canonical snapshot
SHA-256 and allowlisted numeric drive settings. Full configuration values and
credentials are not written to the snapshot.

The probe exclusively uses `getConfigValues` through `ConfigurationInterface`
and the local `aram_hybrid_v8` reader described below. Older `getConfig` and
`getConfigBySections*` services are unsupported, even if advertised. Missing
modern service support produces an incomplete capture, not a fallback. The
probe never calls configuration save, defaults, reload, set, or update APIs.
Reads have a four-second deadline per section, within the 15-second process deadline. A completed
transfer is not proof that all effective/inherited settings were supplied.
Missing settings and unresolved zero sentinels must not be replaced with archived
defaults. Firmware identity and configuration semantics need independent evidence.

On 2026-10-26 the connected robot passed the eight basic read-only checks but
did not advertise the older SDK configuration-export services. The hybrid reader
was derived from static inspection of the supplied Fleet Operations Workspace
Core 4.1.12 server and checked against a MobilePlanner packet capture. Reported
versions are ARAM 8.1.15 and MobilePlanner 8.1.9; these versions were supplied by
the operator, not independently queried by the probe. Live capture then passed
at 2026-10-26T03:57:10Z: all 22 allowlisted numeric settings matched both supplied
model and calibration XML exports. The general config XML contained none of
these settings. This verifies selective retrieval, not motion calibration or
the meaning of inherited zero values. Calibration and movement remain deferred.

Run only the capture acceptance test with:

```bash
python3 -m pytest src/omron_amr/amr_core/test/hardware --robot \
   -m config -k snapshot -v \
   --junitxml=build/amr_core/hardware-config-capture-results.xml
```

### Hybrid Read Profile

The service ID comes from authenticated ARNetworking discovery; it must not be
hardcoded from one capture. The application payload is:

| Field | Encoding | Allowed Value |
| --- | --- | --- |
| Configuration selector | One byte | `1`, ARAMConfig |
| Read operation | One byte | `4`, parameters for named section |
| Section | NUL-terminated string | `Robot config` or `Teleop Settings` only |
| Metadata selector | One byte | `0`; other selector semantics are unsupported |

Each response repeats that prefix, followed by little-endian chunk count and
zero-based index (two bytes each), total body length and chunk body length
(eight bytes each), then chunk data. The assembled body starts with a four-byte
record count; each parameter record starts with `P`. The reader is pinned to the
version-8 metadata layout, not a generic decoder for every configuration API.
It rejects changed identities, duplicate/out-of-order chunks, inconsistent
lengths, unsupported records, missing drive fields, and trailing data. Limits
are 128 chunks, 1 MiB per section, 2,048 records, and 16 KiB per string.

Primitive values include signed little-endian integers and NUL-terminated
decimal doubles. Valid doubles can be subnormal: the capture contains
`4.67261e-310` in source metadata. `std::stod` rejected that representable value;
the reader now uses `std::from_chars` and tests it explicitly. Source slots carry
either a typed value or an empty-string placeholder. Metadata codes and source
slots remain uninterpreted and must not be treated as calibrated motion limits.
Only finite numeric primary values from the drive allowlist are accepted.

Keep raw packet captures and full XML exports private and out of Git: they can
contain authentication data and plaintext configuration credentials. The supplied
capture SHA-256 is
`2137dd37c997b09d260625ebd488f6507f688a308531723bc3d05b81be7a27aa`.
The capture included both TCP directions, connection establishment, and 8,288
packets with no detected sequence gaps; only drive-section data was inspected
for the decoder correction. Neither the capture nor vendor binaries are required
by the registered offline tests. No changes to `libaria` were needed.

### Read-Only Mapping Preflight

Capture artifacts also include `ratio_mapping_assessment`, derived from the
allowlisted snapshot and the driver settings in `--robot-params`. It never
changes parameters, enables calibration, or declares readiness for motion.
Retrieval success and motion readiness are deliberately separate outcomes.

The supplied server binary's `ArActionRatioInput` descriptions and the bundled
SDK agree that zero `FullThrottleForwards` falls back to robot `TransVelMax`,
while zero `FullThrottleBackwards` falls back to one quarter of `TransVelMax`.
For the current capture the candidate gains are therefore 1000 and 250 mm/s,
not 1000 and 100 mm/s. The separate reported reverse speed cap is -100 mm/s.
The candidate stopped/forward/reverse turn gains are 50/25/25 degrees/s.
These are model candidates, not independently confirmed effective firmware gains.
The seven zero-valued teleop speed/acceleration settings still require their own
installed-firmware inheritance rules; they must not be treated as zero gains.

Local driver limits now match the captured translation caps: -100 to 1000 mm/s.
The Nav2 smoother uses -0.10 to 1.00 m/s. With the existing 10% translation dead
zone, legacy normalization by 1.00 m/s leaves every permitted reverse command
at or inside the dead zone, including exactly -0.10 m/s. The candidate calibrated
model instead has a forward dead-zone boundary of 0.10 m/s and a reverse boundary
of 0.025 m/s in magnitude; translation exactly at a boundary is rejected. A
100%-throttle -0.10 m/s target would require -40% ratio in that candidate model,
not -100%. Unit tests exercise this distinction without enabling the model.

Do not fix this by raising minimum Nav2 speeds, boosting commands, guessing
calibration, or changing firmware dead zones automatically. Those changes can
alter stopping/goal behavior or physical joystick operation. Arbitrary low-speed
tracking needs a vendor-confirmed compatible ratio configuration or a supported
absolute-velocity interface, followed by independently supervised acceptance.
Control ownership, watchdog behavior, protective functions, and physical tracking
remain gates. `unsafe_drive: true`, `ratio_mapping: legacy`, and
`calibration.confirmed: false` are unchanged.

## Dead-Zone Calibration Observer

Reusable tooling is the native C++ `amr_deadzone_calibration` executable.
[deadzones.cpp](../src/deadzones.cpp) is a thin `main()` calling the client
header directly. It observes directly
through a local `DriveClient` by default, not the ROS driver. Observation
sends no motion, stop, safe-drive, or configuration-write commands. It requires explicit `--robot`,
checks both dead zones before and after a ten-second observation, and saves
every received numeric odometry sample in a new JSON artifact. Observation
success means valid data with matching settings, not safe motion acceptance.

With the operator present, clear test area, accessible E-stop, and Nav2/other
command sources inactive, a read-only observation of the last verified settings is:

```bash
source install/setup.bash
ros2 run amr_core amr_deadzone_calibration --robot \
   --expected-translation 14 --expected-rotation 5 \
   --output build/amr_core/calibration/observation-14-5.json
```

Use new filenames for subsequent trials; existing artifacts are never replaced.
The tool detects any nonzero reported velocity or change in position/heading,
including values below the ordinary stationary acceptance thresholds. Odometry
quantization, estimator noise, or wheel slip mean it cannot prove absence of
creep and must be paired with the operator's physical observation. It is not an
independent protective stop or a speed/displacement limiter. The ten-second
window is bounded, but the tool has no authority to stop another command source.

For future operator-approved lower-setting trials, start at 5%, then 3%, then 1%,
one setting at a time, with a physical check at each step and read-back of both
settings. Preserve the original 10% translation / 5% rotation configuration;
agree on and verify the final setting explicitly. This session stopped at the
verified 14% / 5% settings, without restoration. Do not change settings during
an observation window. A no-input trial
checks drift only; it does not measure a commanded low-speed response or confirm
the ratio mapping. Do not enable calibration from these trials alone.

### Explicit Configuration Writes

The calibration implementation is header-only in
[deadzone_calibration.hpp](../include/amr_core/utils/deadzone_calibration.hpp).
Its
`changeTranslationDeadzone` overload accepts an already-connected local
`DriveClient`, checks stationary telemetry and complete configuration snapshots,
and attempts at most one numeric translation-dead-zone write. It sends no
property, rotation, motion, mode, save, reload, or restart commands. The C++
workflow calls this guarded header directly, not GTest or the low-level writer
adapter. It rejects requests before connecting unless `--robot`,
`--apply-translation`, and `--confirm-write` are supplied with valid distinct
integer percentages. The separate low-level `amr_deadzone_writer` still requires
its command options, environment approval, and stdin token; the workflow no
longer invokes it. Normal tests cannot perform a live write. Credentials remain
in the settings file or low-level adapter stdin, not argv or logs.

The following is the historical command for the completed operator-approved
translation-only change. Do not rerun it: the current translation value is 14,
not the expected starting value of 10. New writes require fresh approval,
stationary robot, inactive other command sources, and accessible E-stop.

```bash
ros2 run amr_core amr_deadzone_calibration --robot \
   --expected-translation 10 --expected-rotation 5 \
   --apply-translation 14 --confirm-write \
   --output build/amr_core/calibration/write-translation-10-to-14.json
```

Both write options are mandatory together. The workflow creates an exclusive
audit file before invocation, allows one attempt only, and performs a separate
`getConfigValues` read afterward. A timeout or failed receipt is an uncertain
outcome, not proof that no write occurred. Do not retry blindly; inspect the
artifact and obtain read-back first. No automatic restoration or restart occurs.

On 2026-10-26 at 05:19:52Z the operator-approved numeric 10-to-14 change succeeded:
stationary/configuration preflight, single-change receipt, full 22-parameter
post-write comparison, and independent read-back passed. Translation was 14 and
rotation remained 5. No captured property metadata or other parameter changed.
Evidence is under `build/amr_core/calibration/write-translation-10-to-14-*.json`.
The original 10/5 baseline was historical; expected values must match the current
robot before any later trial. After the operator's normal restart, the read-only
check at 05:24:47Z confirmed translation 14 and rotation 5. This establishes
persistence across that single observed restart only, not general persistence
semantics, physical low-speed tracking, or reversible property changes.
Calibration remains disabled; no further restart or write is pending automatically.

Static inspection identifies a chunked transaction handler. The private
2026-10-26 `deadzonechange.pcapng`
capture verifies one MobilePlanner transaction changing `TransDeadZone` from
10 to 5, with two records: a double-value change (operation 4) and a separate
property change (operation 6, property 2, `false` to `true`). The request has
configuration selector 1, a zero byte, one chunk at index zero, and two 64-bit
lengths of 122 bytes. The body starts with a 32-bit record count of two; each
record addresses `Robot Operation` / `Teleop Settings` / empty parent /
`TransDeadZone`. The six-byte reply is `01 00 02 00 00 00`. This reply is not
independent proof of effective application or persistence. Captured numeric
values use NUL-terminated text, not binary doubles.

Both TCP directions and connection establishment were captured: 8,324 packets,
one connection, no detected sequence gaps or conflicting retransmissions, and
valid ARNetworking frame checksums. Capture SHA-256:
`50e1a59f8e14455a0b02d24f316b1d382cc87155cad137677a175826d71ec944`.
The server advertised configuration reads/writes as IDs 288/289 in this session;
future clients must resolve service names rather than hardcode those IDs.
Only one write appears: no 5-to-10 restoration is present. A subsequent live
read-only snapshot confirmed translation 5 / rotation 5; the observer must not
keep assuming the original 10 / 5 settings after this manual change.

The sanitized captured request and reply are covered by a robot-free GTest;
the capture and vendor binary are not test dependencies. Property-code semantics
and exact restoration of the original inherited/override state remain unverified,
although the numeric value-only change survived the single restart above. Further
write captures or vendor protocol documentation are needed before promising exact
restoration of property/inheritance state. Never send `getConfigValues` payloads
to the write service. Treat captures as credential-bearing private data and keep
them out of Git. Capture analysis itself sent no writes or motion commands;
the later operator-approved numeric write is recorded separately above.

## Movement Acceptance Plan

Before implementing or running each physical-motion phase, announce it to the
operator and confirm the clear area, stationary robot, exclusive command source,
and accessible emergency stop. The current operator-approved area is a 1.5 m
radius around the initial robot position; account separately for the robot
footprint and stopping margin. Do not treat odometry as a safety-rated boundary.

The following remain pending and must not be reported as passing skipped tests:

1. Capture live effective speed/dead-zone/watchdog settings and firmware identity
   using read-only vendor-supported tooling. Retain legacy mapping and unconfirmed
   calibration until the modeled firmware contract is verified.
2. Establish conservative motion, elapsed-time, displacement, and stopping limits;
   verify that zeros and explicit stops work before nonzero trials.
3. Test short forward/reverse segments, stationary turns, then curved motion,
   recording commands, body velocities, pose, mode, and packet timing.
4. Supervise command loss, smoother loss, explicit stop, and fresh-input recovery.
   Network/process-loss tests require separately verified robot-side stopping.
5. Run Nav2 motion only after those gates pass. Record physical tracking,
   smoothness, stopping distance, protective behavior, and acceptance tolerances.

Passing connectivity/config tests does not authorize movement automatically.