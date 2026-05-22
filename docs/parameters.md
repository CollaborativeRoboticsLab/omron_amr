# `amr_core` Parameters

The default parameters for `amr_core` live in [amr_ros/config/parameters.yaml](../amr_ros/config/parameters.yaml).
Launch files can layer an additional parameter file on top of those defaults with the `extra_params_file` argument in [amr_ros/launch/amr_core.launch.py](../amr_ros/launch/amr_core.launch.py).

## Common groups

### `robot.*`

- `robot.ip`: target AMR IP address.
- `robot.port`: ARCL port used by the driver connection.
- `robot.password`: ARCL password.

### `host.*`

- `host.ip`: local host IP for the AMR listener socket.
- `host.port`: local host port for the AMR listener socket.

### `source_data.*`

- `source_data.publish`: publishes raw listener-derived status, odom, laser, and fault topics under `amr/source/*`.
- `source_data.frequency`: polling and publish rate for listener-driven source data.

### `map_data.*`

- `map_data.publish`: enables map file publication.
- `map_data.topic`: output topic for map data.
- `map_data.frequency`: publication rate.
- `map_data.file`: map file name.

### `laser_scans.*`

- `laser_scans.publish`: enables ROS laser scan publication from AMR listener data.
- `laser_scans.topic`: laser scan topic name.
- `laser_scans.frame_id`: frame used for published scans.

### `arcl.*`

- `arcl.enable`: enables the ARCL ROS interface.
- `arcl.timeout_ms`: timeout used by ARCL service/action calls.

## `driver.*`

- `driver.publish_odom`: publishes the AMR odometry topic.
- `driver.publish_robot_tf`: broadcasts the `odom_frame -> base_frame` transform from `amr_core`.
- `driver.subscribe_cmd_vel`: enables velocity command subscription.
- `driver.subscribe_goal_pose`: enables goal pose subscription.
- `driver.subscribe_initial_pose`: enables initial pose subscription.
- `driver.subscribe_localplan`: subscribes to `/local_plan` instead of `cmd_vel` when enabled.
- `driver.odom_topic`: odometry topic name used by `amr_core`.
- `driver.cmd_vel_topic`: command velocity topic name consumed by `amr_core`.
- `driver.odom_reset_topic`: topic that triggers odometer reset.
- `driver.goal_pose_topic`: goal pose topic.
- `driver.initial_pose_topic`: initial pose topic.
- `driver.odom_frame`: frame id assigned to published odometry.
- `driver.base_frame`: child frame id assigned to published odometry / TF.
- `driver.reset_odometer_on_startup`: resets the AMR odometer during node startup.
- `driver.expected_cmd_vel_freq`: expected command rate used by the driver logic.
- `driver.command_timeout_ms`: timeout for command/response interactions with the AMR.
- `driver.min_linear_speed`: minimum linear speed clamp in mm/s.
- `driver.max_linear_speed`: maximum linear speed clamp in mm/s.
- `driver.min_angular_speed`: minimum angular speed clamp in deg/s.
- `driver.max_angular_speed`: maximum angular speed clamp in deg/s.
- `driver.unit_move_distance`: linear move step sent per command in mm.
- `driver.unit_turn_angle`: angular turn step sent per command in deg.

## Digital twin overrides

For a simulation-authoritative setup, keep the normal robot defaults in [amr_ros/config/parameters.yaml](../amr_ros/config/parameters.yaml) and place override values in a separate package-specific file.

The Handsolo twin configuration does this in [omron_handsolo/handsolo_ros/config/handsolo-amr.yaml](../../omron_handsolo/handsolo_ros/config/handsolo-amr.yaml), where it disables:

- raw source-data publication
- laser scan publication from `amr_core`
- AMR odometry publication
- AMR TF publication
- odometer reset on startup

That leaves `amr_core` acting as the command bridge while simulation owns TF, odometry, and sensors.
