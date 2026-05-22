# Digital Twin Override Pattern

For mixed real-hardware and simulation workflows, keep the generic AMR defaults in [../amr_ros/config/parameters.yaml](../amr_ros/config/parameters.yaml) and layer package-specific overrides on top.

## Why use an override file

The base AMR package should preserve normal robot behavior:

- publishes odometry
- publishes `odom -> base_link` TF
- optionally publishes listener-derived laser and source topics
- resets the odometer on startup

That is the right default for a standalone AMR integration, but it conflicts with a digital twin that already publishes the authoritative robot state.

## Recommended pattern

1. Keep hardware defaults in [../amr_ros/config/parameters.yaml](../amr_ros/config/parameters.yaml).
2. Add a package-specific override file in the integration package.
3. Pass that file into [../amr_ros/launch/amr_core.launch.py](../amr_ros/launch/amr_core.launch.py) with the `extra_params_file` launch argument.

## An example

Omron Handsolo uses [handsolo_ros/config/handsolo-amr.yaml](https://github.com/CollaborativeRoboticsLab/omron_handsolo/tree/humble/handsolo_ros/config/handsolo-amr.yaml) to disable ROS-side state publication from `amr_core` while keeping the command path active.

The relevant override values are:

```yaml
amr_core:
  ros__parameters:
    source_data:
      publish: false

    laser_scans:
      publish: false

    driver:
      publish_odom: false
      publish_robot_tf: false
      reset_odometer_on_startup: false
```

With that arrangement:

- simulation owns TF and odometry
- simulation owns scan publication
- `amr_core` still receives filtered velocity commands and forwards them to the AMR

This keeps the authority over robot state in one place instead of splitting it across simulation and hardware interfaces.