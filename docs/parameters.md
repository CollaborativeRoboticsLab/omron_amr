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
- `docking.forward_departure_bypass`: experimental, default `false`. Requires explicit contact-dock/no-cable and exclusive-control confirmations. Allows one straight forward command up to 0.2 m/s for at most 0.5 s with fresh Docked feedback and stationary telemetry at startup; normal undocking action is disabled. Readiness in this mode means eligibility for that one attempt only. The limits are command limits, not verified physical speed or a protective stop. Direct departure support by the installed firmware is unverified.
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


## Digital twin overlays

For simulation-authoritative setups, keep the hardware defaults in [../amr_ros/config/parameters.yaml](../amr_ros/config/parameters.yaml) or [../amr_ros/config/ld250_parameters.yaml](../amr_ros/config/ld250_parameters.yaml), then layer a package-specific override file with `extra_params_file`.

The override usually disables:

- `status.publish`
- `laser.main_laser.enabled`
- `laser.low_laser.enabled`
- `driver.publish_odom`
- `driver.publish_robot_tf`

That leaves `amr_core` acting as the command bridge while simulation owns TF, odometry, and sensors.
