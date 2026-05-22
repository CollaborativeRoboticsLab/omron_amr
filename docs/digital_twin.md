# Digital twin overrides

The `main` branch keeps the default LD250 hardware configuration in [../amr_ros/config/ld250_parameters.yaml](../amr_ros/config/ld250_parameters.yaml) and supports an integration-specific override file layered on top.

## Launch layering

[../amr_ros/launch/amr_core.launch.py](../amr_ros/launch/amr_core.launch.py) accepts:

- `params_file`: the base parameter file
- `extra_params_file`: an optional overlay applied after `params_file`

The LD250 wrapper launch [../amr_ros/launch/ld250.launch.py](../amr_ros/launch/ld250.launch.py) forwards the same `extra_params_file` argument.

## Twin-safe controls

For simulation-authoritative setups, the main branch uses these controls:

- `status.publish`
- `laser.main_laser.enabled`
- `laser.low_laser.enabled`
- `driver.publish_odom`
- `driver.publish_robot_tf`

These let the hardware interface remain command-capable without duplicating robot state already published by simulation.

## Odom origin note

Unlike the Humble branch, `main` does not explicitly reset the robot odometer on startup. The libaria-backed driver captures the first received pose as the ROS odometry origin and publishes relative motion from that point onward.