# Core startup

This document covers the launch entry points in `amr_ros` and how they relate to the hardware interface, robot description, and optional RViz sessions.

## Packages involved

- `amr_ros`: top-level launch files for LD90 and LD250
- `amr_core`: hardware interface node that talks to the robot
- `amr_description`: robot description and xacro files
- `amr_nav2`: optional Nav2, SLAM, localization, maps, and Nav2 RViz workflow

## Common startup pattern

Source the workspace before launching anything:

```sh
source install/setup.bash
```

## Start the hardware interface only

This launches the `amr_core` node with `amr_ros/config/parameters.yaml`.

```sh
ros2 launch amr_ros amr_core.launch.py
```

## Start LD90

This launches:

- `amr_core`
- `robot_state_publisher` with the LD90 description

```sh
ros2 launch amr_ros ld90.launch.py
```

## Start LD250

This launches:

- `amr_core`
- `robot_state_publisher` with the LD250 description

```sh
ros2 launch amr_ros ld250.launch.py
```

## Open the legacy robot RViz view

These views are the existing robot-centric RViz sessions from `amr_ros/rviz`.

```sh
ros2 launch amr_ros ld90.launch.py rviz:=true
```

```sh
ros2 launch amr_ros ld250.launch.py rviz:=true
```

## Top-level launch arguments

The LD90 and LD250 top-level launches expose these common arguments:

- `rviz:=true|false`: open the legacy robot RViz config from `amr_ros`
- `robot_description_override:=true|false`: skip the default robot description if a parent launch provides one
- `use_nav2:=true|false`: include the Nav2 child launch from `amr_nav2`
- `use_slam:=true|false`: enable SLAM Toolbox in the Nav2 child launch
- `use_localization:=true|false`: enable Nav2 localization in the child launch
- `nav2_params_file:=...`: override the Nav2 YAML file
- `map:=filename.yaml`: map YAML filename resolved under `amr_nav2/maps`

## Startup examples

Start LD250 with Nav2 localization:

```sh
ros2 launch amr_ros ld250.launch.py use_nav2:=true use_localization:=true map:=warehouse.yaml
```

Start LD90 with SLAM:

```sh
ros2 launch amr_ros ld90.launch.py use_nav2:=true use_slam:=true
```

## Notes

- `use_localization` only starts localization when `map` is non-empty.
- If both `use_slam:=true` and `use_localization:=true` are set, the launch flow suppresses localization so only one map-to-odom source is active.
- The Nav2 RViz session is launched separately from `amr_nav2`; see [nav2_usage.md](./nav2_usage.md).
