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
- `driver.drive_ready_topic`: mode and docking readiness status topic, default `amr/drive_ready` (`std_msgs/Bool`). A fresh `true` indicates those gates are open; it does not acknowledge command delivery, indicate that no stop is pending, or certify physical safety. Treat missing or stale updates as not ready.
- `shutdown.enabled`: default `false`. When enabled, exposes the relative `shutdown_robot` `std_srvs/Trigger` service and requires an advertised `shutdownRobot` command, managed docking, `docking.contact_dock_confirmed: true`, and `shutdown.exclusive_control_confirmed: true`.
- `shutdown.exclusive_control_confirmed`: default `false`; an operator confirmation that no other controller can command the robot during shutdown. This setting does not enforce exclusive control or provide a safety function.
- `fault.ros_service.enabled`: default `false`. Exposes the named `amr_msgs/srv/AcknowledgeFault` service. Requires advertised `acknowledgeFault`, managed docking, `docking.contact_dock_confirmed` and `fault.ack.exclusive_control_confirmed` set to true.
- `fault.ros_service.name`: service name, default `acknowledge_fault` (relative to the node namespace).
- `fault.ack.exclusive_control_confirmed`: default `false`; an operator confirmation that no other controller can command the robot during acknowledgement. This setting does not enforce exclusive control or provide a safety function.
- `fault.monitor.enabled`: default `true`. Queries `getFaults` once per second and reports newly observed faults at ROS error level. If the service is unavailable, the query cannot be queued, or a reply times out, an error is logged and monitoring stops until the driver restarts.
- `fault.monitor.auto_acknowledge_enabled`: default `false`. Requires fault monitoring and the same docking, advertised command, and exclusive-control confirmations as the ROS service, but does not require the service to be enabled. Only faults in `fault.auto_acknowledge_fault_list` can be acknowledged automatically.
- `fault.auto_acknowledge_fault_list`: string array, empty by default. The LD250 configuration lists `IrregularShutdown`, but leaves automatic acknowledgement disabled. Names must be unique, nonblank, at most 128 bytes, and contain no control characters.
- `map.ros_service.enabled`: default `false`. Enables a read-only active-map query over a separate robot connection.
- `map.ros_service.name`: default `map/get_active_map`. The `std_srvs/Trigger` response contains the active map name in `message` when successful. It does not upload, activate, or validate a map.

For a separately authorized upload, `amr_hardware_probe map-upload` reads the robot host, port, user, password, protocol, and local map path as six newline-separated stdin fields. It validates the local map with ARIA, uploads it under a random `staged_*.map` filename, and compares a full readback before reporting success. `map-upload-named` additionally reads the desired filename as a seventh field and checks the remote directory for an existing name. Neither activates the file. The server protocol can overwrite existing names and does not provide exclusive creation; the directory check is not atomic, and a random 128-bit suffix makes a collision unlikely but cannot guarantee it. A failure after transfer begins can leave a staged or temporary file on the robot. Keep credentials off command lines and out of logs.

`amr_hardware_probe map-download` accepts the same seven stdin fields as `map-upload-named`, with the sixth field specifying a new local output path and the seventh the remote filename. It saves the downloaded bytes without overwriting an existing file and reports whether ARIA can parse them; an unparseable file is still saved for inspection.

`amr_hardware_probe map-activate` reads connection fields, the expected local map path, the remote target name, and the expected currently active name on stdin. It compares the remote file, checks active and configured names and fresh docked/stationary feedback, sends one map-selection transaction, and verifies both names afterward. A failed or missing receipt leaves the outcome unverified and must not be blindly retried. Activation can disrupt localization; docked status and this guard do not establish movement safety.
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

The shutdown service accepts one request only when fresh dock feedback reports Docked, no undocking operation is reserved, and fresh telemetry has reported zero velocity continuously for at least 0.5 seconds. These checks are not safety-rated. The service response confirms local acceptance only; the networking loop checks the conditions again and logs whether the command was locally queued. Delivery and power-off are not acknowledged. After acceptance, velocity commands and further shutdown or undock requests are inhibited until the driver restarts, including when submission fails. Explicit stop requests remain available. Test coverage uses a localhost fake server; shutdown on the physical robot is unverified. Restrict ROS service access to authorized operators.

The fault acknowledgement service accepts one `fault_name` (1 to 128 bytes, not whitespace-only, with no control characters); it never requests acknowledgement of all faults. Acceptance requires fresh Docked feedback and at least 0.5 seconds of fresh stationary telemetry, with no undock or shutdown reservation and no request already pending. The networking loop checks the conditions again before submission and logs whether the request was locally queued. An accepted request can be discarded if conditions change, shutdown is requested, or the connection is lost; there is no automatic retry. A successful response means local acceptance only, not queueing, delivery, acknowledgement, or fault clearance. Independently verify the fault and robot state before further operation. Submission does not inhibit a later undock, and the checks are not safety-rated. Restrict service access to authorized operators; acknowledgement must not substitute for diagnosing or resolving the fault. In one supervised physical test, an `IrregularShutdown` request was locally queued and a subsequent fault query reported zero faults; other fault types and conditions remain unverified.

Automatic acknowledgement uses the same eligibility checks and attempts each continuously observed allowlisted fault at most once, even if submission fails. A fault becomes eligible for another attempt only after a valid reply no longer contains it and a later valid reply reports it again. The monitor rejects truncated, ambiguous, or trailing data rather than naming or acknowledging faults from an unverified packet. The connected robot's nonempty fault reply previously contained undocumented trailing bytes, so it will be reported as unsupported and cannot currently trigger automatic acknowledgement or identify individual faults. Automatic acknowledgement has only been tested with a localhost fake server; confirm the installed firmware's fault format and use supported fault diagnosis before enabling it.

No robot reboot command was identified in the advertised shutdown/reboot/restart names; restart notifications are not evidence of a reboot interface.


## Digital twin overlays

For simulation-authoritative setups, keep the hardware defaults in [../amr_ros/config/parameters.yaml](../amr_ros/config/parameters.yaml) or [../amr_ros/config/ld250_parameters.yaml](../amr_ros/config/ld250_parameters.yaml), then layer a package-specific override file with `extra_params_file`.

The override usually disables:

- `status.publish`
- `laser.main_laser.enabled`
- `laser.low_laser.enabled`
- `driver.publish_odom`
- `driver.publish_robot_tf`

That leaves `amr_core` acting as the command bridge while simulation owns TF, odometry, and sensors.
