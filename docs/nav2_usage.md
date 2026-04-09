# Nav2 usage

This document covers the new Nav2 integration in `amr_nav2`, including SLAM, localization, the map directory, and the Nav2 RViz workflow.

## Files and directories

- `amr_nav2/launch/ld90.launch.py`: LD90 Nav2 child launch
- `amr_nav2/launch/ld250.launch.py`: LD250 Nav2 child launch
- `amr_nav2/launch/nav2.launch.py`: Nav2 server bringup used by both robots
- `amr_nav2/launch/slam.launch.py`: SLAM Toolbox integration
- `amr_nav2/launch/nav2_rviz.launch.py`: RViz launcher for Nav2 workflows
- `amr_nav2/config/ld90_nav2.yaml`: LD90 Nav2 parameters
- `amr_nav2/config/ld250_nav2.yaml`: LD250 Nav2 parameters
- `amr_nav2/maps`: map YAML files and referenced occupancy images
- `amr_nav2/rviz/nav2.rviz`: Nav2 RViz config with Nav2 plugins and tools

## Map directory convention

Map files now live under `amr_nav2/maps`.

The launch argument is a filename, not a full path.

Example layout:

```text
amr_nav2/maps/
	warehouse.yaml
	warehouse.pgm
```

Example launch argument:

```sh
map:=warehouse.yaml
```

## Start Nav2 localization

LD250 example:

```sh
source install/setup.bash
ros2 launch amr_ros ld250.launch.py use_nav2:=true use_localization:=true map:=warehouse.yaml
```

LD90 example:

```sh
source install/setup.bash
ros2 launch amr_ros ld90.launch.py use_nav2:=true use_localization:=true map:=warehouse.yaml
```

Behavior:

- localization only starts if `use_localization:=true`
- localization only starts if `map` is non-empty
- the map filename is resolved relative to `amr_nav2/maps`

## Start Nav2 SLAM

LD250 example:

```sh
source install/setup.bash
ros2 launch amr_ros ld250.launch.py use_nav2:=true use_slam:=true
```

LD90 example:

```sh
source install/setup.bash
ros2 launch amr_ros ld90.launch.py use_nav2:=true use_slam:=true
```

If both `use_slam:=true` and `use_localization:=true` are set, the launch flow suppresses localization to avoid conflicting map sources.

## Launch the Nav2 RViz session

The Nav2 RViz view is separate from the legacy `amr_ros` RViz views.

```sh
source install/setup.bash
ros2 launch amr_nav2 nav2_rviz.launch.py
```

This RViz config includes:

- Nav2 RViz panel plugin
- map display
- local and global costmaps
- laser scan
- robot model
- TF tree
- global and local paths
- AMCL pose display
- `2D Pose Estimate` and `2D Goal Pose` tools

## Recommended workflows

### Mapping workflow

1. Launch the base and Nav2 with `use_slam:=true`.
2. Launch `amr_nav2 nav2_rviz.launch.py`.
3. Drive the robot and build the map in RViz.
4. Save the map into `amr_nav2/maps`.

### Localization workflow

1. Copy the saved map YAML and image into `amr_nav2/maps`.
2. Launch the base and Nav2 with `use_localization:=true map:=your_map.yaml`.
3. Launch `amr_nav2 nav2_rviz.launch.py`.
4. Use `2D Pose Estimate` in RViz to initialize localization.
5. Use `2D Goal Pose` to send navigation goals.

## Useful commands

Start LD250 localization and open Nav2 RViz:

```sh
ros2 launch amr_ros ld250.launch.py use_nav2:=true use_localization:=true map:=warehouse.yaml
ros2 launch amr_nav2 nav2_rviz.launch.py
```

Start LD90 SLAM and open Nav2 RViz:

```sh
ros2 launch amr_ros ld90.launch.py use_nav2:=true use_slam:=true
ros2 launch amr_nav2 nav2_rviz.launch.py
```
