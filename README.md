# Omron AMR package

To view the original Readme.md [click here](./docs/old_docs/original_readme.md)

This package is a restructuring of [OmronAPAC/Omron_AMR_ROS2](https://github.com/OmronAPAC/Omron_AMR_ROS2) along with integration of [libaria](https://github.com/CollaborativeRoboticsLab/libaria) following [KalanaRatnayake/ros_omron_agv](https://github.com/KalanaRatnayake/ros_omron_agv)

| Branch | ROS2 Version | Compile |
|--------|--------------|---------|
| main | Jazzy | [![main](https://github.com/CollaborativeRoboticsLab/omron_amr/actions/workflows/compile.yml/badge.svg?branch=main)](https://github.com/CollaborativeRoboticsLab/omron_amr/actions/workflows/compile.yml?query=branch%3Amain) |
| develop | Jazzy | [![develop](https://github.com/CollaborativeRoboticsLab/omron_amr/actions/workflows/compile.yml/badge.svg?branch=develop)](https://github.com/CollaborativeRoboticsLab/omron_amr/actions/workflows/compile.yml?query=branch%3Adevelop) |
| humble | Humble | [![humble](https://github.com/CollaborativeRoboticsLab/omron_amr/actions/workflows/compile.yml/badge.svg?branch=humble)](https://github.com/CollaborativeRoboticsLab/omron_amr/actions/workflows/compile.yml?query=branch%3Ahumble) |

## Documentation

- [Core startup](./docs/core_startup.md)
- [Core parameters](./docs/core_parameters.md)
- [Digital twin overrides](./docs/digital_twin.md)
- [Laser interface](./docs/laser_interface.md)
- [Nav2 usage](./docs/nav2_usage.md)

## Setup

Create a workspace

```sh
mkdir -p omron_ws/src
cd omron_ws/src
```

Install dependencies
```sh
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-joy ros-jazzy-joy
```

Clone the repositories into the `src` folder by

```sh
git clone --recurse-submodules https://github.com/CollaborativeRoboticsLab/omron_amr.git
```

Build by

```sh
cd ..
colcon build
```

## Start only the hardware interface

```sh
source install/setup.bash
ros2 launch amr_ros amr_core.launch.py
```

## LD250

### Start LD250 base only (Hardware interface + robot description)

```sh
source install/setup.bash
ros2 launch amr_ros ld250.launch.py use_nav2:=false use_rviz:=false
```

### Start Nav2 SLAM and open the Nav2 RViz view for mapping

RViz starts by default; pass `use_rviz:=false` to disable it.

The SLAM launch publishes a map with a 1 m unknown border on `map`; the unmodified SLAM grid is available on `slam_map_raw`. The default Nav2 planner does not plan through unknown space in SLAM mode. Padding does not add obstacle observations, and does not establish that navigation is safe. Custom SLAM parameters must use the default `/map` topic for this padding pipeline.


```sh
source install/setup.bash
ros2 launch amr_ros ld250.launch.py
```

### Start Nav2 localization with a map from amr_nav2/maps and open the Nav2 RViz view

```sh
source install/setup.bash
ros2 launch amr_ros ld250.launch.py use_nav2:=true use_localization:=true map:=my_map.yaml
```

### Undock separately

Navigation and velocity commands do not initiate undocking. With the hardware interface running, undocking is a separate `nav2_msgs/action/UndockRobot` action. It requires verified contact docking, exclusive control, fresh docking feedback, and stationary telemetry. `driver.docking.contact_dock_confirmed` is read at startup; `driver.drive.exclusive_control_confirmed` also defaults to false. Set these confirmations only after verifying the corresponding conditions.

For an explicitly authorized, supervised departure, send one goal:

```sh
source install/setup.bash
ros2 action send_goal /undock_robot nav2_msgs/action/UndockRobot "{dock_type: 'omron', max_undocking_time: 30.0}" --feedback
```

Proceed with navigation only after successful action completion and independent confirmation that the robot has stopped and is undocked. Do not retry a failed or uncertain departure automatically; investigate and verify the physical state first. The previously observed stale-feedback failure remains unresolved.

If a goal is rejected, the driver logs the reason in the launch terminal; the action client reports only that the goal was rejected.


## LD90

### Start LD90 base only (Hardware interface + robot description)

```sh
source install/setup.bash
ros2 launch amr_ros ld90.launch.py
```

### Start Nav2 SLAM and open the Nav2 RViz view for mapping

```sh
source install/setup.bash
ros2 launch amr_ros ld90.launch.py use_nav2:=true use_slam:=true
```

```sh
source install/setup.bash
ros2 launch amr_nav2 nav2_rviz.launch.py
```

### Start Nav2 localization with a map from amr_nav2/maps and open the Nav2 RViz view

```sh
source install/setup.bash
ros2 launch amr_ros ld90.launch.py use_nav2:=true use_localization:=true map:=my_map.yaml
```

```sh
source install/setup.bash
ros2 launch amr_nav2 nav2_rviz.launch.py
```

## Start teleoperation

```sh
source install/setup.bash
ros2 launch amr_teleop amr_joyop.launch.py
```
 or 

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```