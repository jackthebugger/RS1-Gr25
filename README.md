# RS1-Gr25

RS1 Forest Management Project using ROS 2 Humble and Ignition Gazebo Fortress.

## Final code access

The final implementation is maintained on the `main` branch:

<https://github.com/jackthebugger/RS1-Gr25/tree/main>

The repository contains the simulation worlds, Husky and Parrot models, sensor
configuration, SLAM/Nav2 setup, B.E.E.R. thermal fire detection, fire-hazard
localisation, obstacle memory, tests and runbooks.

## Requirements

- Ubuntu Linux
- ROS 2 Humble
- Ignition Gazebo Fortress
- `ros-dev-tools`
- ROS 2 Navigation2
- SLAM Toolbox
- `robot_localization`
- Python 3 and NumPy

Install the main ROS dependencies with:

```bash
sudo apt update
sudo apt install -y \
  ros-dev-tools \
  ros-humble-robot-localization \
  ros-humble-ros-ign \
  ros-humble-ros-ign-interfaces \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-teleop-twist-keyboard \
  python3-numpy
```

Install Ignition Gazebo Fortress separately if it is not already installed.

## Build

```bash
git clone https://github.com/jackthebugger/RS1-Gr25.git
cd RS1-Gr25
git checkout main

export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select \
  rs1_interfaces \
  beer_fire_detection \
  41068_ignition_bringup

source install/setup.bash
```

`build/`, `install/` and `log/` are local build outputs and are intentionally
not part of the repository. Rebuild after pulling changes that affect the ROS
packages.

## Run the final Husky demonstration

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch 41068_ignition_bringup \
  41068_ignition_husky.launch.py \
  slam:=true \
  nav2:=true \
  rviz:=true \
  gui:=true \
  world:=bush_trail_world \
  husky_x:=-18 \
  husky_y:=3 \
  husky_z:=0.50 \
  husky_yaw:=0.0
```

Use RViz with `husky1_map` as the Fixed Frame and send a goal using the
**Nav2 Goal** or **2D Goal Pose** tool.

For the controlled thermal fire-detection scenario:

```bash
ros2 launch 41068_ignition_bringup beer_sensor_demo.launch.py
```

## Documentation

The complete installation and operation runbook is in
[`TERMINAL_WORKFLOW.md`](src/41068_ignition_bringup_v1/TERMINAL_WORKFLOW.md).

The implementation overview is in
[`simulation_workflow_and_implementation_overview.md`](src/41068_ignition_bringup_v1/41068_ignition_bringup/simulation_workflow_and_implementation_overview.md).
