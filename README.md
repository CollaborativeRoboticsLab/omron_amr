# Omron Base package

To view the original Readme.md [click here](./docs/original_readme.md)

This package is a restructuring of [OmronAPAC/Omron_AMR_ROS2](https://github.com/OmronAPAC/Omron_AMR_ROS2)

View [New Developer's Guide](https://github.com/CollaborativeRoboticsLab/omron_base/blob/master/docs/DeveloperGuide_updated.adoc).
View [Old Developer's Guide](https://github.com/CollaborativeRoboticsLab/omron_base/blob/master/docs/DeveloperGuide.adoc).

## Setup

Create a workspace

```sh
mkdir -p omron_ws/src
cd omron_ws/src
```

Install dependencies
```sh
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-teleop-twist-joy ros-humble-joy
```

Clone the repositories into the `src` folder by

```sh
git clone https://github.com/CollaborativeRoboticsLab/omron_base.git
```

Build by

```sh
cd ..
colcon build
```

## Usage 

### Initialization

1. [Establish Remote connection to AMR Robot](https://github.com/CollaborativeRoboticsLab/omron_base/blob/main/docs/DeveloperGuide.adoc#231-set-up-user-ethernet)

2. [Configure AMR robot via Mobile Planner](https://github.com/CollaborativeRoboticsLab/omron_base/blob/main/docs/DeveloperGuide_updated.adoc#332-set-up-arcl)


### Connect with the robot base

Run the following command to connect to the robot.

```sh
source install/setup.bash
ros2 launch amr_ros amr_core.launch.py
```
### Start visualization

Run the following command to visualize robot. Swap `ld250` with `ld90` or `amr_platform` for other robot models.

```sh
source install/setup.bash
ros2 launch amr_ros ld250_visualize.launch.py
```

### Start Teleoperation

```sh
source install/setup.bash
ros2 launch amr_teleop amr_joyop.launch.py
```

## To Do List

- [x] Replace AMR_Core with a cpp package with support for standard ros2 interface (/cmd_vel, /tf and /odom)
- [ ] Add support for NAV2
- [ ] Create cascadeing launch files for AMR core and RVIz

