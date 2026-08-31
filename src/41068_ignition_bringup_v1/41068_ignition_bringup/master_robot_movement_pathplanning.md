# Master Robot Movement & Path Planning Knowledge Base

> **Package:** `41068_ignition_bringup` (UTS 41068 Robotics Studio 1)
> **Evidence date:** 2026-08-28
> **Companion reference:** `MASTER_CONTEXT.md` (course concepts — not runtime truth)
> **Codebase root:** `/home/jordan/RS1-Gr25/src/41068_ignition_bringup_v1/41068_ignition_bringup/`

---

## 1. Document Purpose

This document reverse-engineers **what the repository actually implements** for robot movement, sensing, mapping, localization, obstacle representation, path planning, path following, and replanning in Ignition Gazebo Fortress + ROS 2 Humble.

It is written for future AI coding agents and human developers who need to:

- Understand how the Husky/Parrot move in simulation today
- Trace topics, nodes, frames, and parameters end-to-end
- Distinguish **IMPLEMENTED** vs **PROPOSED** functionality
- Implement or debug autonomous navigation without re-investigating the whole repo

---

## 2. Source of Truth / Evidence Rules

| Source | Role |
|--------|------|
| **This repository** | Authoritative for runtime behavior, file paths, parameters, topics |
| **`MASTER_CONTEXT.md`** | Authoritative for course concepts, seminar theory, intended autonomy stack |
| **External packages** (`nav2_bringup`, `slam_toolbox`, etc.) | Behavior defined by installed distro packages + this repo's YAML overrides |

**Evidence labels used throughout:**

- **Confirmed** — verified in repo source/config
- **Likely** — strongly indicated by standard Nav2/SLAM behavior + local config, not traced inside external package source here
- **Proposed** — recommended architecture, not implemented in this repo
- **Unknown** — cannot be determined from this repository alone

**Discrepancy protocol:** When `MASTER_CONTEXT.md` differs from code, discrepancies are listed in §33 and inline with `DISCREPANCY` tags.

---

## 3. Repository Overview

### 3.1 High-level summary (**Confirmed**)

Single ROS 2 package providing:

- Ignition Gazebo Fortress simulation (forest worlds)
- Namespaced Husky UGV (`/husky1`) and simplified Parrot UAV (`/parrot1`)
- `ros_ign_bridge` sensor/actuator bridges
- `robot_localization` EKF for `odom → base_link`
- Optional **SLAM Toolbox** online mapping
- Optional **Nav2** full navigation stack (global planner, local planner, BT navigator, recovery behaviors)
- Two example Python nodes (autonomy demo, dynamic world demo)

**No C++ nodes.** No `ros2_control`. No custom path planner implementation in this package.

### 3.2 Directory layout

```
41068_ignition_bringup/
├── package.xml, CMakeLists.txt, README.md, MASTER_CONTEXT.md
├── config/           # Nav2, SLAM, EKF, bridges, RViz, Gazebo server plugins
├── launch/           # 6 launch files
├── scripts/          # basic_autonomy_demo.py, dynamic_world_demo.py
├── urdf_husky/       # Husky URDF + Gazebo plugins + meshes
├── urdf_parrot/      # Parrot URDF + Gazebo plugins + meshes
├── worlds/           # simple_trees.sdf, large_demo.sdf
├── models/           # grass_plane, forest_plane, forest_wall
└── knowldge/         # Course seminar extraction (not runtime)
```

---

## 4. Complete Package Inventory

| Package | Path | Build type | Purpose |
|---------|------|------------|---------|
| `41068_ignition_bringup` | repo root | `ament_cmake` | Simulation bringup, config, examples |

**Executables installed** (`CMakeLists.txt`):

| Executable | Source | Type |
|------------|--------|------|
| `basic_autonomy_demo.py` | `scripts/basic_autonomy_demo.py` | ROS 2 node |
| `dynamic_world_demo.py` | `scripts/dynamic_world_demo.py` | ROS 2 node |

**External runtime dependencies** (`package.xml`, `README.md`):

- `ros_ign_gazebo`, `ros_ign_bridge`, `ros_ign_interfaces`
- `robot_localization`
- `nav2_bringup`, `navigation2`, `slam_toolbox` (exec deps via README install, not in package.xml)
- `rclpy`, `nav2_msgs`, `nav_msgs`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `numpy`

---

## 5. System Architecture

### 5.1 Implemented architecture (Husky with `nav2:=true`)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    Ignition Gazebo Fortress (world SDF)                 │
│  Physics │ Sensors (ogre) │ Contact │ SceneBroadcaster │ UserCommands   │
│  World geometry (trees, ground, walls) + spawned robot models           │
└───────────────────────────────┬─────────────────────────────────────────┘
                                │ Gazebo Transport topics
                                ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  Gazebo robot plugins (per URDF gazebo.xacro)                           │
│  VelocityControl │ OdometryPublisher │ JointStatePublisher │ IMU        │
│  gpu_lidar │ rgbd_camera                                                │
└───────────────────────────────┬─────────────────────────────────────────┘
                                │ /model/husky1/*
                                ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  ros_ign_bridge (namespace /husky1)  + global /clock bridge             │
│  → scan, odometry, imu, camera/*, joint_states                          │
│  ← cmd_vel                                                              │
└───────────────────────────────┬─────────────────────────────────────────┘
                                │
        ┌───────────────────────┼───────────────────────┐
        ▼                       ▼                       ▼
 robot_state_publisher    robot_localization EKF    SLAM Toolbox (if slam/nav2)
 (URDF → tf_static,       (odometry+imu → odom,     (scan+TF → map, map→odom TF)
  joint TF)                odom→base_link TF)
        │                       │                       │
        └───────────────────────┴───────────────────────┘
                                │ TF tree + /husky1/map
                                ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  Nav2 (if nav2:=true) — nav2_bringup/navigation_launch.py              │
│  bt_navigator → planner_server → smoother_server                        │
│  controller_server → velocity_smoother → cmd_vel                        │
│  local_costmap + global_costmap (scan → obstacle layer)                 │
│  behavior_server (recovery) │ AMCL (lifecycle node, see §16)            │
└───────────────────────────────┬─────────────────────────────────────────┘
                                │ /husky1/cmd_vel
                                ▼
                         Gazebo VelocityControl → robot motion
```

### 5.2 Course target architecture (`MASTER_CONTEXT.md`)

Five-layer stack: Map Processor → Behavior Executive → Global Planner → Local Planner → Controller.

**Status:** **PARTIALLY IMPLEMENTED** via Nav2 + SLAM Toolbox + example autonomy node. No dedicated Map Processor or Behavior Executive nodes exist in this package.

---

## 6. Gazebo Simulation

### 6.1 Simulator (**Confirmed**)

| Item | Value | Evidence |
|------|-------|----------|
| Simulator | Ignition Gazebo Fortress | `README.md` |
| Launch | `ros_ign_gazebo/ign_gazebo.launch.py` | `launch/41068_ignition.launch.py` |
| Render engine | `ogre` (server + GUI) | launch file `ign_args` |
| Sim clock | `/clock` bridged to ROS | `config/gazebo_bridge_clock.yaml` |
| `use_sim_time` | Default `True` | launch args |

### 6.2 World-level server plugins (**Confirmed**)

File: `config/ignition_server.config`

| Plugin | Purpose |
|--------|---------|
| `ignition-gazebo-physics-system` | Physics simulation |
| `ignition-gazebo-user-commands-system` | User/world commands |
| `ignition-gazebo-scene-broadcaster-system` | Scene state |
| `ignition-gazebo-sensors-system` | **Required** for lidar/camera rendering (loaded once at world level) |
| `ignition-gazebo-contact-system` | Contact events |

Loaded via env vars `IGN_GAZEBO_SERVER_CONFIG_PATH` / `GZ_SIM_SERVER_CONFIG_PATH` in `41068_ignition.launch.py`.

### 6.3 Worlds (**Confirmed**)

| World | File | Description |
|-------|------|-------------|
| `simple_trees` | `worlds/simple_trees.sdf` | 15×15 m grass plane, 1 oak + 1 pine (Gazebo Fuel) |
| `large_demo` | `worlds/large_demo.sdf` | Forest floor, boundary walls, many trees, demo dynamic models |

**Physics** (`simple_trees.sdf`): gravity −9.81 m/s², `max_step_size=0.01`, `real_time_update_rate=100`.

**Robot spawn** (`41068_ignition.launch.py`):

| Robot | Namespace | Gazebo name | Spawn pose (x,y,z) | Delay |
|-------|-----------|-------------|-------------------|-------|
| Husky | `husky1` | `husky1` | (0, 0, 0.4) | 3.0 s |
| Parrot | `parrot1` | `parrot1` | (2, 0, 0.8) | 6.0 s |

Spawn via `ros_ign_gazebo/create` subscribed to `/{namespace}/robot_description`.

### 6.4 Simulation vs real robot

| Aspect | Simulation | Real robot implication |
|--------|--------------|------------------------|
| Motion | `VelocityControl` directly moves base | Real Husky uses motor controllers / diff-drive |
| Odometry | Gazebo `OdometryPublisher` (near ground truth) | Wheel slip, calibration errors |
| Lidar/camera | GPU simulated sensors, 3 Hz default | Higher rates, different noise |
| Parrot | Gravity off, collisions disabled | Not representative of real flight |

---

## 7. Robot Model

### 7.1 Husky UGV (**Confirmed**)

**Files:** `urdf_husky/husky.urdf.xacro`, `husky.gazebo.xacro`, `wheel.urdf.xacro`

| Property | Value | Evidence |
|----------|-------|----------|
| Base frame | `{prefix}base_link` → `husky1_base_link` | URDF |
| Base mass | 46.064 kg | URDF inertial |
| Wheel separation | 0.5708 m | `wheel.urdf.xacro` |
| Wheel radius | 0.1651 m | `wheel.urdf.xacro` |
| Wheel joints | 4 revolute (`front_left`, `front_right`, `rear_left`, `rear_right`) | URDF |
| Drive type (URDF) | 4 wheels with collision cylinders | URDF |
| **Actual sim drive** | **VelocityControl on base (not diff-drive plugin)** | `husky.gazebo.xacro` comment + plugin |

**Collision geometry (base):** Multiple boxes approximating chassis (~0.99 × 0.57 m footprint region).

**Sensor mounts (Husky):**

| Sensor | Frame | Position (base_link) | Orientation |
|--------|-------|----------------------|-------------|
| IMU | `husky1_imu_link` | (0, 0, 0.068) | identity |
| Lidar | `husky1_base_scan` | (0, 0, 0.68) | identity |
| Camera | `husky1_camera_link` | (0.45, 0, 0.25) | identity |
| Camera optical frames | `camera_rgb_optical_frame`, `camera_depth_optical_frame` | via fixed joints | standard camera optical rotation |

### 7.2 Parrot UAV (**Confirmed**)

**Files:** `urdf_parrot/parrot.urdf.xacro`, `parrot.gazebo.xacro`

| Property | Value |
|----------|-------|
| Base frame | `parrot1_base_link` |
| Collisions | **Commented out** in URDF — flies through trees |
| Gravity | `turnGravityOff=true` on links | `parrot.gazebo.xacro` |
| Motion | Same `VelocityControl` as Husky (planar) |
| Lidar mount | z=0.2 m on base (lower than Husky) |
| Camera | Pitched ~45° down (`rpy="0 0.785398 0"`) |

### 7.3 Gazebo plugins per robot (**Confirmed**)

From `husky.gazebo.xacro` / `parrot.gazebo.xacro`:

| Plugin | System | Topic (Gazebo) | Purpose |
|--------|--------|----------------|---------|
| `ignition-gazebo-velocity-control-system` | VelocityControl | `model/{name}/cmd_vel` | Apply linear/angular velocity to model |
| `ignition-gazebo-odometry-publisher-system` | OdometryPublisher | `model/{name}/odometry` | Publish odometry 20 Hz |
| `ignition-gazebo-joint-state-publisher-system` | JointStatePublisher | `model/{name}/joint_states` | Wheel joint states |
| `ignition-gazebo-imu-system` | IMU | `model/{name}/imu` | IMU 100 Hz |
| `gpu_lidar` sensor | (Sensors system) | `model/{name}/scan` | LaserScan + point cloud |
| `rgbd_camera` sensor | (Sensors system) | `model/{name}/camera/*` | RGB, depth, points |

---

## 8. Robot Movement Pipeline

### 8.1 End-to-end chain (**Confirmed**)

```
Goal source (RViz / basic_autonomy_demo / teleop)
    ↓
Nav2 controller_server (DWB)  OR  teleop_twist_keyboard  OR  other node
    ↓
velocity_smoother (Nav2, when nav2 active)  [Likely outputs smoothed cmd_vel]
    ↓
geometry_msgs/Twist on /husky1/cmd_vel
    ↓
ros_ign_bridge (ROS_TO_GZ)  config/gazebo_bridge_husky1.yaml
    ↓
/model/husky1/cmd_vel (Gazebo Transport)
    ↓
ignition::gazebo::systems::VelocityControl
    ↓
Gazebo physics updates model pose (base moved directly; wheels are visual/collision only)
    ↓
OdometryPublisher → /model/husky1/odometry → /husky1/odometry
    ↓
EKF → /husky1/odom + TF husky1_odom → husky1_base_link
```

### 8.2 Critical implementation detail

**DISCREPANCY:** `MASTER_CONTEXT.md` and README describe Husky as "differential drive." The URDF has four wheel joints, but **no differential-drive or skid-steer Gazebo plugin is used**. Movement is **kinematic velocity control of the whole model**:

```80:87:urdf_husky/husky.gazebo.xacro
  <!-- Move base directly without faffing around with diff drive controllers -->
  <gazebo>
    <plugin
      filename="ignition-gazebo-velocity-control-system"
      name="ignition::gazebo::systems::VelocityControl">
      <topic>model/${gz_model_name}/cmd_vel</topic>
    </plugin>
  </gazebo>
```

Wheel joints rotate visually via physics contact but are **not** the primary drive interface.

### 8.3 cmd_vel semantics (**Confirmed**)

| Field | Husky usage |
|-------|-------------|
| `linear.x` | Forward velocity |
| `linear.y` | Not used by Nav2 config (`max_vel_y: 0`) |
| `angular.z` | Yaw rate |

Nav2 DWB limits (`nav2_params_husky1.yaml`): `max_vel_x=1.0`, `max_vel_theta=1.5`, `acc_lim_x=2.5`.

### 8.4 When no command is received

**Likely:** VelocityControl holds zero velocity; robot stops. Nav2 `velocity_smoother` has `velocity_timeout: 1.0` s — stops sending commands if controller stalls.

### 8.5 Collision behavior (**Confirmed**)

Husky has collision geometry on base and wheels; Gazebo physics prevents interpenetration with world obstacles. Parrot collisions disabled in URDF.

### 8.6 Safety controller

**NOT IMPLEMENTED** in this package. Nav2 `BaseObstacle` critic and costmap inflation provide soft avoidance, not emergency braking hardware interface.

---

## 9. Controllers

| Controller | Status | Location | Notes |
|------------|--------|----------|-------|
| `ros2_control` | **NOT IMPLEMENTED** | — | No controller YAML |
| Gazebo VelocityControl | **IMPLEMENTED** | `husky.gazebo.xacro` | Primary motion |
| Nav2 DWB local planner | **IMPLEMENTED** (external) | `nav2_params_husky1.yaml` | Outputs cmd_vel to controller_server |
| Nav2 velocity_smoother | **IMPLEMENTED** (external) | same YAML | Rate/accel limits |
| Recovery behaviors | **IMPLEMENTED** (external) | spin, backup, drive_on_heading, wait, assisted_teleop |

---

## 10. Sensors

### 10.1 Sensor inventory (**Confirmed**)

| Sensor | Gazebo type | Update rate | Range / FOV | Noise | ROS topic | Message | Frame |
|--------|-------------|-------------|-------------|-------|-----------|---------|-------|
| Lidar | `gpu_lidar` | 3 Hz | 0.2–40 m, 360° (360 samples) | Gaussian σ=0.01 | `/husky1/scan` | `sensor_msgs/LaserScan` | `husky1_base_scan` |
| Lidar PC | same | 3 Hz | — | — | `/husky1/scan/points` | `PointCloud2` | `husky1_base_scan` |
| RGB camera | `rgbd_camera` | 3 Hz | HFOV 2.094 rad, 720×480 | — | `/husky1/camera/image` | `Image` | `husky1_camera_link` |
| Depth image | `rgbd_camera` | 3 Hz | near 0.4 / far 10 (depth) | — | `/husky1/camera/depth/image` | `Image` | `husky1_camera_link` |
| Depth cloud | `rgbd_camera` | 3 Hz | — | — | `/husky1/camera/depth/points` | `PointCloud2` | `husky1_camera_link` |
| IMU | `imu` | 100 Hz | — | per-axis Gaussian | `/husky1/imu` | `sensor_msgs/Imu` | `husky1_imu_link` |
| Wheel encoders | — | via joint_states | — | — | `/husky1/joint_states` | `JointState` | wheel links |
| Odometry | OdometryPublisher | 20 Hz | — | — | `/husky1/odometry` (raw), `/husky1/odom` (filtered) | `nav_msgs/Odometry` | `husky1_odom` |

Parrot uses identical sensor stack with `parrot1_*` names (`gazebo_bridge_parrot1.yaml`).

### 10.2 Sensors NOT present

GPS, ultrasonic, contact/bumper ROS topics, dedicated wheel encoder odometry node.

### 10.3 Consumers (**Confirmed**)

| Sensor topic | Primary consumers |
|--------------|-------------------|
| `scan` | SLAM Toolbox, Nav2 obstacle layers (local + global costmap), RViz |
| `map` | Nav2 static layer, `basic_autonomy_demo.py`, RViz |
| `odom` / `odometry` | EKF, Nav2 controller, BT navigator |
| `imu` | EKF |
| `camera/image` | `basic_autonomy_demo.py` (brightness), RViz |
| `camera/depth/*` | RViz only (no navigation consumer in this package) |

---

## 11. Sensor Data Flow

### 11.1 Lidar pipeline (**Confirmed**)

```
Gazebo gpu_lidar (husky1_base_scan)
  → /model/husky1/scan
  → ros_ign_bridge (gazebo_bridge_husky1.yaml)
  → /husky1/scan (LaserScan, frame_id=husky1_base_scan)
  → SLAM Toolbox (mapping, scan matching)
  → Nav2 ObstacleLayer (marking + clearing, max range 10 m obstacle, 20 m raytrace)
  → RViz LaserScan display
```

### 11.2 Odometry pipeline (**Confirmed**)

```
Gazebo OdometryPublisher (20 Hz, odom_frame=husky1_odom, base=husky1_base_link)
  → /model/husky1/odometry
  → bridge → /husky1/odometry
  → robot_localization EKF (fuses vx, vy, vyaw from odom + yaw rate from IMU)
  → /husky1/odom + TF husky1_odom → husky1_base_link (30 Hz)
```

EKF config (`robot_localization_husky1.yaml`): `two_d_mode: true`, `world_frame: husky1_odom`, does **not** fuse position from IMU.

### 11.3 Topic table (Husky, navigation-relevant)

| Topic | Message | Publisher | Subscriber(s) | Purpose | Frame |
|-------|---------|-----------|---------------|---------|-------|
| `/clock` | `rosgraph_msgs/Clock` | bridge | all `use_sim_time` nodes | Sim time | — |
| `/husky1/cmd_vel` | `Twist` | Nav2 / teleop | bridge → Gazebo | Motion command | — |
| `/husky1/scan` | `LaserScan` | bridge | SLAM, costmaps | Obstacle detection | `husky1_base_scan` |
| `/husky1/odometry` | `Odometry` | bridge | EKF | Raw sim odom | `husky1_odom` |
| `/husky1/odom` | `Odometry` | EKF | Nav2 | Filtered odom | `husky1_odom` |
| `/husky1/imu` | `Imu` | bridge | EKF | Angular rate fusion | `husky1_imu_link` |
| `/husky1/map` | `OccupancyGrid` | SLAM Toolbox | Nav2 static layer, autonomy demo | SLAM map | `husky1_map` |
| `/husky1/map_updates` | `OccupancyGridUpdate` | SLAM | RViz, static layer | Map updates | `husky1_map` |
| `/husky1/plan` | `Path` | Nav2 planner | RViz | Global plan | `husky1_map` |
| `/husky1/local_plan` | `Path` | Nav2 controller | RViz | Local plan | `husky1_odom` |
| `/husky1/global_costmap/costmap` | `OccupancyGrid` | Nav2 | RViz | Global costmap | `husky1_map` |
| `/husky1/local_costmap/costmap` | `OccupancyGrid` | Nav2 | RViz, behaviors | Local costmap | `husky1_odom` |
| `/husky1/tf` | `tf2_msgs/TFMessage` | EKF, SLAM, robot_state_publisher | all TF users | Dynamic transforms | — |
| `/husky1/tf_static` | `tf2_msgs/TFMessage` | robot_state_publisher | all TF users | URDF static transforms | — |
| `/husky1/navigate_to_pose` | Action `NavigateToPose` | — | autonomy demo, RViz Nav2 panel | Navigation goals | — |

---

## 12. ROS 2 Nodes

### 12.1 Nodes started by canonical launch (**Confirmed**)

**Always (per enabled robot):**

| Node | Package | Namespace | Source |
|------|---------|-----------|--------|
| `robot_state_publisher` | `robot_state_publisher` | `/husky1` | `41068_ignition.launch.py` |
| `robot_localization` (EKF) | `robot_localization` | `/husky1` | same |
| `spawn_husky1` | `ros_ign_gazebo` | — | same |
| `gazebo_bridge` | `ros_ign_bridge` | `/husky1` | same |

**Global:**

| Node | Package | Name |
|------|---------|------|
| Gazebo server | `ros_ign_gazebo` | via `ign_gazebo.launch.py` |
| `ros_gz_bridge_clock` | `ros_ign_bridge` | clock bridge |

**If `slam:=true` OR `nav2:=true`:**

| Node(s) | Package | Launch |
|---------|---------|--------|
| SLAM Toolbox async | `slam_toolbox` | `41068_navigation.launch.py` → `online_async_launch.py` |

**If `nav2:=true`:**

| Node(s) | Package | Launch |
|---------|---------|--------|
| Full Nav2 stack | `nav2_bringup` | `navigation_launch.py` |

Includes (**Likely** from standard Nav2 Humble bringup): `controller_server`, `planner_server`, `smoother_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`, `velocity_smoother`, `lifecycle_manager_navigation`, `map_server`, `amcl`.

**If `rviz:=true`:**

| Node | Namespace |
|------|-----------|
| `rviz2` | `/husky1` |

**Optional add-ons:**

| Node | Launch file |
|------|-------------|
| `basic_autonomy_demo` | `41068_autonomy_demo.launch.py` |
| `dynamic_world_demo` | `41068_dynamic_world_demo.launch.py` |

### 12.2 Custom node details

#### `basic_autonomy_demo` (**Confirmed**)

- **File:** `scripts/basic_autonomy_demo.py`
- **Class:** `BasicAutonomyDemo`
- **Subscriptions:** `map` (`OccupancyGrid`), `camera/image` (`Image`)
- **Action client:** `navigate_to_pose` (`NavigateToPose`)
- **TF:** lookup `husky1_map` → `husky1_base_link`
- **Algorithm:** Random free-space goal sampling with camera brightness biasing distance; **not** a path planner

#### `dynamic_world_demo` (**Confirmed**)

- **File:** `scripts/dynamic_world_demo.py`
- **Purpose:** Move `demo_animal`, cycle tree visual states via `ign service` `set_pose`
- **ROS sensor/planning integration:** **None** — does not publish ROS topics

---

## 13. ROS 2 Topics / Services / Actions

### 13.1 Actions (**Confirmed**)

| Action | Type | Server (when Nav2 active) | Clients |
|--------|------|---------------------------|---------|
| `/husky1/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Nav2 `bt_navigator` | RViz, `basic_autonomy_demo.py` |
| `/husky1/compute_path_to_pose` | `nav2_msgs/ComputePathToPose` | Nav2 | BT nodes |
| `/husky1/follow_path` | `nav2_msgs/FollowPath` | Nav2 | BT nodes |
| Recovery actions | various | `behavior_server` | BT |

### 13.2 Services (**Likely** — standard Nav2)

Lifecycle services under `/husky1/` for each Nav2 server; `map_saver/save_map` when configured.

### 13.3 Gazebo services (dynamic demo)

`/world/large_demo/set_pose` — used by `dynamic_world_demo.py` (not ROS).

---

## 14. TF Tree

### 14.1 Husky TF tree (**Confirmed**)

```
husky1_map
 └── husky1_odom          [SLAM Toolbox: map→odom, AND possibly AMCL — see §16]
      └── husky1_base_link [EKF: odom→base_link]
           ├── husky1_imu_link          [static URDF]
           ├── husky1_base_scan         [static URDF]
           ├── husky1_camera_link       [static URDF]
           │    ├── husky1_camera_rgb_frame
           │    │    └── husky1_camera_rgb_optical_frame
           │    ├── husky1_camera_depth_frame
           │    │    └── husky1_camera_depth_optical_frame
           ├── husky1_front_left_wheel_link   [revolute joint]
           ├── husky1_front_right_wheel_link
           ├── husky1_rear_left_wheel_link
           └── husky1_rear_right_wheel_link
```

### 14.2 TF publishers

| Transform | Publisher | Rate | Notes |
|-----------|-----------|------|-------|
| `odom → base_link` | `robot_localization` EKF | 30 Hz | Fused odom+IMU |
| `map → odom` | SLAM Toolbox | ~50 Hz (`transform_publish_period: 0.02`) | When SLAM active |
| `map → odom` | AMCL | on update | **Configured** `tf_broadcast: true` — potential conflict |
| URDF fixed joints | `robot_state_publisher` | static | Sensor/wheel frames |

### 14.3 Frame semantics

| Frame | Meaning |
|-------|---------|
| `husky1_map` | Global map frame — SLAM map origin; planning global frame |
| `husky1_odom` | Odometry origin — continuous but drifts; local costmap frame |
| `husky1_base_link` | Robot base — control and footprint frame |
| `husky1_base_scan` | Lidar — perception ray origin |

**Navigation rule:** Global plans in `husky1_map`; local control in `husky1_odom`; sensor data in sensor frames transformed via TF.

---

## 15. Coordinate Frames

Planning frame: `husky1_map` (global), `husky1_odom` (local rolling window).

Sensor data frame: `husky1_base_scan` for lidar; camera data in `husky1_camera_link` / optical frames.

Gazebo world frame: implicit world origin where robots are spawned.

---

## 16. Localisation

### 16.1 What tells the system where the robot is? (**Confirmed**)

| Layer | Source | Type | Frame |
|-------|--------|------|-------|
| Short-term pose | Gazebo OdometryPublisher → EKF | **Sim ground truth + light fusion** | `husky1_odom` → `husky1_base_link` |
| Map-relative pose | SLAM Toolbox scan matching | **Estimated** | `husky1_map` → `husky1_odom` |
| AMCL | Configured in Nav2 YAML | **Estimated** (particle filter) | Would publish `husky1_map` → `husky1_odom` |

### 16.2 Ground truth vs estimated

| Data | Ground truth? |
|------|---------------|
| Gazebo model pose | Yes (physics engine) |
| `/husky1/odometry` | Derived from sim physics — **near ground truth** |
| `/husky1/odom` (EKF) | Filtered sim odom — **near ground truth** |
| `/husky1/map` + map→odom TF | **Estimated** by SLAM |
| AMCL pose | **Estimated** — intended for static maps |

### 16.3 SLAM vs AMCL (**DISCREPANCY / RISK**)

This package runs **SLAM Toolbox in `mapping` mode** when `nav2:=true`, which publishes `map→odom`.

AMCL is also configured with `tf_broadcast: true` in `nav2_params_husky1.yaml`.

**Likely conflict:** Two nodes may compete to publish `husky1_map → husky1_odom`. Standard practice with online SLAM: disable AMCL TF or use localization-only SLAM mode.

**Evidence:**
- SLAM: `config/slam_params_husky1.yaml` `mode: mapping`, `transform_publish_period: 0.02`
- AMCL: `config/nav2_params_husky1.yaml` `tf_broadcast: true`

### 16.4 Drift

Sim odometry: minimal drift (VelocityControl + OdometryPublisher). SLAM map→odom corrects drift relative to built map. Loop closure enabled in SLAM config.

---

## 17. Mapping

### 17.1 Implementation (**IMPLEMENTED** when `slam:=true` or `nav2:=true`)

| Item | Value | Evidence |
|------|-------|----------|
| Node | SLAM Toolbox `async_slam_toolbox_node` | `41068_navigation.launch.py` |
| Mode | `mapping` (online SLAM) | `slam_params_husky1.yaml` |
| Input | `/husky1/scan` | `scan_topic: scan` |
| Output | `/husky1/map`, `/husky1/map_updates` | launch remaps |
| Resolution | 0.05 m | `slam_params_husky1.yaml` |
| Map frame | `husky1_map` | `map_frame: husky1_odom` wait - map_frame: husky1_map |
| Max laser range | 20.0 m | slam params |
| Update interval | 2.0 s (`map_update_interval`) | slam params |
| Loop closure | enabled | `do_loop_closing: true` |

### 17.2 Occupancy grid semantics (**Confirmed**)

Standard `nav_msgs/OccupancyGrid`:

| Value | Meaning |
|-------|---------|
| `-1` | Unknown |
| `0` | Free |
| `1–99` | Partial occupancy (SLAM/probability) |
| `100` | Occupied |

`basic_autonomy_demo.py` treats `-1` as unknown, `0` as free, `> free_cell_threshold (20)` as occupied.

### 17.3 Static vs dynamic map

| Aspect | Status |
|--------|--------|
| SLAM map | **Dynamic** — grows/updates as robot explores |
| Nav2 static layer | Subscribes to SLAM map — updates with SLAM |
| Dynamic obstacles in map | **PARTIAL** — laser marks obstacles in costmap obstacle layer; SLAM map updates slowly (2 s); fast-moving objects may not appear in static map |

### 17.4 No pre-built map

`map_server` `yaml_filename: ''` — no static map file loaded by default.

---

## 18. Perception

**NOT IMPLEMENTED** as dedicated perception nodes in this package.

Raw sensor data flows directly to:

- SLAM Toolbox (lidar)
- Nav2 costmap obstacle layers (lidar)
- `basic_autonomy_demo.py` (camera brightness heuristic)

No object detection, no point-cloud segmentation, no custom fusion nodes.

---

## 19. Obstacle Detection

### 19.1 Implemented pipeline (**Confirmed**)

```
LaserScan (/husky1/scan)
  → Nav2 ObstacleLayer (local + global costmaps)
      marking: obstacle_max_range 10 m
      clearing: raytrace_max_range 20 m
  → InflationLayer (inflation_radius 2.0 m, cost_scaling_factor 3.0)
  → Planner collision checking / DWB BaseObstacle critic
```

Camera depth/point cloud: **not** used for obstacle detection in Nav2 config.

### 19.2 Gazebo ground truth

Physics collisions affect robot motion (Husky). No ROS topic exposes Gazebo contact data to Nav2.

### 19.3 Dynamic obstacles

`dynamic_world_demo.py` moves `demo_animal` in Gazebo. If visible to lidar, **Likely** detected by obstacle layer at next costmap update (5 Hz). Not added to SLAM map instantly.

---

## 20. Robot Footprint & Collision Safety

### 20.1 Physical robot (**Confirmed**)

Approximate chassis: ~0.99 m × 0.57 m (collision boxes in URDF). Wheel separation 0.5708 m.

### 20.2 Nav2 footprint (**Confirmed**)

```yaml
footprint: '[ [0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5] ]'
```

**1.0 m × 1.0 m square** in `nav2_params_husky1.yaml` — **larger than physical Husky** (conservative/safe).

### 20.3 Inflation (**Confirmed**)

| Parameter | Value |
|-----------|-------|
| `inflation_radius` | 2.0 m |
| `cost_scaling_factor` | 3.0 |

### 20.4 Collision checking

| Layer | Method |
|-------|--------|
| Global planner | NavFn grid search on inflated costmap |
| Local planner | DWB trajectory sampling + `BaseObstacle` critic (scale 0.02) |
| Physical sim | Gazebo collision geometry |

**Gap:** Footprint is 2D polygon; no explicit 3D obstacle handling for camera/lidar vertical structure beyond `max_obstacle_height: 2.0`.

---

## 21. Start / Goal Handling

### 21.1 Start position (**Confirmed**)

| Source | Representation |
|--------|----------------|
| Gazebo spawn | `41068_ignition.launch.py` `x,y,z,yaw` args (Husky: 0,0,0.4) |
| Runtime pose | TF `husky1_map` → `husky1_base_link` or `husky1_odom` → `husky1_base_link` |

Nav2 uses current localized pose as planning start when goal received.

### 21.2 Goal position (**Confirmed**)

| Source | Interface | Frame |
|--------|-----------|-------|
| RViz "Nav2 Goal" / "2D Goal Pose" | `NavigateToPose` action | `husky1_map` |
| `basic_autonomy_demo.py` | `NavigateToPose` action | `husky1_map` |
| Parameters | — | not used for goals in stock config |

Goal message: `geometry_msgs/PoseStamped` with position (x,y) and quaternion orientation.

Nav2 goal tolerance: `xy_goal_tolerance: 1.0` m, `yaw_goal_tolerance: 1.57` rad (~90°).

---

## 22. Current Path Planning

### 22.1 Custom planners in this repo

**NOT IMPLEMENTED.** No A*, RRT, PRM, or custom graph search in package source.

### 22.2 Nav2 global planner (**IMPLEMENTED** via external package)

| Item | Value |
|------|-------|
| Plugin | `nav2_navfn_planner/NavfnPlanner` |
| Algorithm | Navigation function (Dijkstra-like grid propagation on costmap) |
| `use_astar` | `false` |
| `allow_unknown` | `true` |
| Input | global costmap, start, goal |
| Output | `nav_msgs/Path` on `/husky1/plan` |
| Frequency | `expected_planner_frequency: 20` Hz |

**Beginner:** Finds a route across the map grid favoring low-cost cells.

**Technical:** Builds a potential field from goal on 2D cost grid, extracts gradient descent path. Not A* despite course seminar coverage of A*.

### 22.3 Path smoothing (**IMPLEMENTED**)

`nav2_smoother::SimpleSmoother` via `smoother_server`.

---

## 23. Current Path Following

### 23.1 Local planner (**IMPLEMENTED**)

| Item | Value |
|------|-------|
| Plugin | `dwb_core::DWBLocalPlanner` |
| Type | Dynamic Window Approach — samples (vx, vtheta) trajectories |
| Frequency | `controller_frequency: 5.0` Hz |
| Critics | RotateToGoal, Oscillation, BaseObstacle, GoalAlign, PathAlign, PathDist, GoalDist |
| Output | `Twist` → controller_server → velocity_smoother → `cmd_vel` |

### 23.2 Behavior tree navigator (**IMPLEMENTED**)

`bt_navigator` orchestrates: compute path → smooth → follow path → recoveries.

Plugin list includes `nav2_is_path_valid_condition_bt_node` — supports path validity checking (**Likely** triggers replan).

---

## 24. Global Planning

**IMPLEMENTED** via Nav2 `planner_server` + NavFn on `global_costmap`.

| Property | Value |
|----------|-------|
| Map frame | `husky1_map` |
| Resolution | 0.1 m (global costmap) |
| Static layer | SLAM map |
| Obstacle layer | Live laser |
| Unknown space | Tracked (`track_unknown_space: true`); NavFn `allow_unknown: true` |

---

## 25. Local Planning

**IMPLEMENTED** via DWB on `local_costmap`.

| Property | Value |
|----------|-------|
| Frame | `husky1_odom` |
| Rolling window | 10 m × 10 m |
| Resolution | 0.05 m |
| Plugins | obstacle_layer + inflation_layer (no static layer) |

Local planner reacts to nearby obstacles not yet in global map.

---

## 26. Real-Time Replanning

### 26.1 What exists (**PARTIALLY IMPLEMENTED**)

| Mechanism | Status | Rate |
|-----------|--------|------|
| Local obstacle avoidance | **IMPLEMENTED** (DWB + local costmap) | 5 Hz |
| Costmap obstacle updates | **IMPLEMENTED** | 5 Hz |
| Global path recomputation | **Likely** via BT on goal update / invalid path | on event |
| SLAM map update | **IMPLEMENTED** | 2 s interval + scan matching |
| `is_path_valid` BT node | **Configured** in plugin list | — |

### 26.2 Scenario: obstacle appears 2 m ahead (**Confirmed analysis**)

**With `nav2:=true` active:**

1. Lidar detects obstacle at next scan (~3 Hz sensor, 5 Hz costmap).
2. Local costmap obstacle layer marks cells within 10 m.
3. DWB `BaseObstacle` critic penalizes trajectories intersecting high-cost cells.
4. Robot **Likely slows/stops/steers** around obstacle if local planner finds feasible trajectory.
5. Global path may still route through obstacle until BT triggers replan (**Likely** if `is_path_valid` fails).
6. SLAM static map may **not** update for 2+ seconds.

**Without `nav2:=true`:** Only teleop or direct `cmd_vel` — **no avoidance**. Robot **will hit obstacle** (Husky).

**`dynamic_world_demo` animal:** If moved into lidar FOV, same Nav2 behavior applies.

### 26.3 Gaps

- No dedicated emergency stop node
- Camera/depth not used for reactive avoidance
- Slow lidar (3 Hz) limits reaction latency
- No explicit dynamic obstacle layer (e.g. velocity obstacles)

---

## 27. Nav2

### 27.1 Status (**IMPLEMENTED** when `nav2:=true`)

| Aspect | Status |
|--------|--------|
| Installed | Expected via `ros-humble-navigation2` (README) |
| Launched | `41068_navigation.launch.py` → `nav2_bringup/navigation_launch.py` |
| Configured | `config/nav2_params_husky1.yaml`, `nav2_params_parrot1.yaml` |
| Namespaced | Yes — `/husky1/*` |

### 27.2 Key plugins (**Confirmed** from YAML)

| Server | Plugin |
|--------|--------|
| Global planner | NavfnPlanner |
| Local controller | DWBLocalPlanner |
| Smoother | SimpleSmoother |
| Localisation | AMCL (configured) |
| Recovery | Spin, BackUp, DriveOnHeading, Wait, AssistedTeleop |

### 27.3 Legacy/unused config

| File | Status |
|------|--------|
| `config/nav2_params.yaml` | Unprefixed frames — **not used** by launch |
| `config/slam_params.yaml` | **not used** |
| `config/robot_localization.yaml` | **not used** |
| `config/gazebo_bridge_husky.yaml` | **not used** (uses `husky1` suffix) |

**DISCREPANCY:** `MASTER_CONTEXT.md` references `config/nav_params.yaml` — **file does not exist**; actual files are `nav2_params_*.yaml`.

---

## 28. RViz

**Config:** `config/41068_husky1.rviz`

| Display | Topic | Represents |
|---------|-------|------------|
| Fixed Frame | `husky1_map` | Global visualization frame |
| RobotModel | `/husky1/robot_description` | URDF geometry |
| LaserScan | `/husky1/scan` | Live lidar — **real sensor data** |
| Map | `/husky1/map` | SLAM occupancy grid — **real map** |
| Path | `/husky1/plan` | Global plan — **only when Nav2 planning** |
| Path (local) | `/husky1/local_plan` | Local plan |
| Costmap | `/husky1/global_costmap/costmap` | Planner obstacle costs |
| Camera | `/husky1/camera/image` | RGB |
| PointCloud2 | depth points | Camera depth cloud |
| Navigation 2 panel | — | Sends goals, recovery buttons |

**Warning:** Displaying a path in RViz does not mean custom code planned it — Nav2 generates paths when active.

---

## 29. Complete Data Flow

| Stage | Source | Interface | Destination | Data |
|-------|--------|-----------|-------------|------|
| Sim motion | Gazebo VelocityControl | `/model/husky1/cmd_vel` | Physics | Twist |
| Bridge in | `ros_ign_bridge` | `/husky1/cmd_vel` | Gazebo | Twist |
| Bridge out | Gazebo lidar | `/husky1/scan` | SLAM, costmaps | LaserScan |
| Odometry | Gazebo | `/husky1/odometry` | EKF | Odometry |
| Filtered odom | EKF | `/husky1/odom`, TF | Nav2, SLAM | Odometry, TF |
| Mapping | SLAM | `/husky1/map` | Nav2 static, demo | OccupancyGrid |
| Map TF | SLAM | TF `map→odom` | TF tree | Transform |
| Goal | RViz/demo | `NavigateToPose` | BT navigator | PoseStamped |
| Global plan | planner_server | `/husky1/plan` | controller via BT | Path |
| Local cmd | controller_server | `cmd_vel` (internal) | velocity_smoother | Twist |
| Actuation | velocity_smoother | `/husky1/cmd_vel` | bridge | Twist |

---

## 30. Code-Level Execution Flow

### 30.1 Launch to motion (**Confirmed**)

```
main: 41068_ignition.launch.py::generate_launch_description()
  → IncludeLaunchDescription(ign_gazebo.launch.py)
  → add_robot() for husky1
      → Node(robot_state_publisher)
      → Node(robot_localization/ekf_node)  params: robot_localization_husky1.yaml
      → TimerAction → Node(ros_ign_gazebo/create) spawn
      → Node(ros_ign_bridge/parameter_bridge)  config: gazebo_bridge_husky1.yaml
  → add_navigation_instance() if nav2/slam
      → IncludeLaunchDescription(41068_navigation.launch.py)
          → slam_toolbox online_async_launch.py
          → nav2_bringup navigation_launch.py
```

### 30.2 Autonomy demo goal send (**Confirmed**)

```
main() → BasicAutonomyDemo.__init__()
  → create_subscription(map), create_subscription(image)
  → ActionClient(NavigateToPose, 'navigate_to_pose')
  → create_timer(1.0, _tick)

_tick()
  → _lookup_robot_pose()  # tf_buffer.lookup_transform(map, base_link)
  → _choose_goal()        # _sample_free_goal() on OccupancyGrid
  → _send_goal()
      → nav_client.send_goal_async(NavigateToPose.Goal)
```

**File:** `scripts/basic_autonomy_demo.py`
**Symbols:** `BasicAutonomyDemo._send_goal`, `_sample_free_goal`, `_is_free_with_margin`

### 30.3 Gazebo motion (**Confirmed**)

URDF processed by xacro → spawned into Gazebo → `VelocityControl` plugin subscribes `model/husky1/cmd_vel`.

No package C++ callback — motion is entirely Gazebo plugin + physics.

---

## 31. Parameters & Configuration

### 31.1 Launch arguments

| Parameter | Default | File | Effect |
|-----------|---------|------|--------|
| `husky` | `true` | `41068_ignition.launch.py` | Spawn Husky |
| `parrot` | `false` | same | Spawn Parrot |
| `slam` | `false` | same | SLAM Toolbox per robot |
| `nav2` | `false` | same | Nav2 (+ SLAM via nav launch) |
| `rviz` | `false` | same | RViz per robot |
| `world` | `simple_trees` | same | World file |
| `gui` | `true` | same | Gazebo GUI |
| `use_sim_time` | `True` | same | Sim clock |

### 31.2 Navigation parameters (Husky)

| Parameter | Value | File | Used by | Effect |
|-----------|-------|------|---------|--------|
| `controller_frequency` | 5.0 Hz | nav2_params_husky1.yaml | controller_server | Local control rate |
| `max_vel_x` | 1.0 m/s | same | DWB | Max forward speed |
| `max_vel_theta` | 1.5 rad/s | same | DWB | Max turn rate |
| `inflation_radius` | 2.0 m | same | costmaps | Safety margin |
| `footprint` | 1.0×1.0 m box | same | costmaps | Collision checking |
| `obstacle_max_range` | 10.0 m | same | obstacle layer | Lidar marking range |
| `global_costmap.resolution` | 0.1 m | same | global costmap | Grid size |
| `local_costmap.width/height` | 10 m | same | local costmap | Local window |
| `resolution` (SLAM) | 0.05 m | slam_params_husky1.yaml | SLAM | Map resolution |
| `map_update_interval` | 2.0 s | same | SLAM | Map publish throttle |
| `max_laser_range` | 20.0 m | same | SLAM | Scan range cap |
| `frequency` (EKF) | 30 Hz | robot_localization_husky1.yaml | EKF | TF publish rate |
| lidar `update_rate` | 3 Hz | husky.gazebo.xacro | Gazebo | Sensor rate |
| odom publish | 20 Hz | husky.gazebo.xacro | Gazebo | Raw odometry |

### 31.3 Autonomy demo parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `close_goal_min_distance` | 4.0 m | Dark image goal band |
| `far_goal_max_distance` | 15.0 m | Bright image goal band |
| `bright_image_threshold` | 0.25 | Image brightness threshold |
| `free_cell_threshold` | 20 | Max occupancy for "free" |
| `occupied_margin_cells` | 4 | Clearance from obstacles in grid |

---

## 32. Current System Behaviour

### Summary

| Mode | Command | Behavior |
|------|---------|----------|
| Default launch | none | Robot spawned, sensors active, **no autonomous motion** |
| Teleop | `cmd_vel` | Direct motion via VelocityControl |
| `slam:=true` | — | Map builds as robot moves; no auto motion |
| `nav2:=true` | RViz goal | Nav2 plans + follows path with avoidance |
| Autonomy demo | internal | Random Nav2 goals based on map + brightness |

---

## 33. Missing Components

| Component | Status | Notes |
|-----------|--------|-------|
| Custom global planner | **NOT IMPLEMENTED** | Nav2 only |
| Custom local planner | **NOT IMPLEMENTED** | Nav2 only |
| Perception pipeline | **NOT IMPLEMENTED** | Direct sensor → SLAM/Nav2 |
| Behavior executive | **NOT IMPLEMENTED** | Only demo random walk |
| Map processor / multi-robot fusion | **NOT IMPLEMENTED** | |
| ros2_control / real diff-drive | **NOT IMPLEMENTED** | VelocityControl |
| Camera-based avoidance | **NOT IMPLEMENTED** | |
| Emergency stop | **NOT IMPLEMENTED** | |
| Static map localization workflow | **PARTIAL** | AMCL configured but SLAM used |
| User interface | **PARTIAL** | RViz only |

---

## 34. Risks / Limitations

1. **AMCL + SLAM TF conflict** — both may publish `map→odom`
2. **VelocityControl ≠ diff-drive** — wheel dynamics, slip not modeled realistically
3. **Slow sensors** — 3 Hz lidar/camera limits reaction time
4. **Large Nav2 footprint** — may block narrow passages that physical robot fits
5. **Large goal tolerance** — 1 m xy, ~90° yaw
6. **Parrot** — no collisions; Nav2 not meaningful for 3D flight
7. **WSL rendering** — software GL mitigations required (`LIBGL_ALWAYS_SOFTWARE`, etc.)
8. **Classroom ROS discovery** — need `ROS_LOCALHOST_ONLY=1` (README)
9. **Dynamic obstacles** — demo animal not integrated with planning logic
10. **MASTER_CONTEXT nav_params.yaml** — wrong filename in course doc

---

## 35. Recommended Target Architecture

**PROPOSED** — minimum additions for robust start→goal with live replanning, integrating with existing stack:

```
[Gazebo + existing bridges + EKF]  (keep)
        ↓
[SLAM Toolbox] → map + map→odom     (keep; fix AMCL conflict)
        ↓
[Nav2 global costmap + NavFn]       (keep or swap planner plugin)
        ↓
[Nav2 local costmap + DWB]          (keep; tune footprint/inflation)
        ↓
[Optional: student Behavior Executive node]
  - subscribes: map, stuck signals, mission state
  - publishes: NavigateToPose goals
        ↓
[velocity_smoother → cmd_vel]       (keep)
        ↓
[Optional: emergency stop watchdog on scan/min range]
```

**Live replanning loop (already largely in Nav2):**

```
scan → local costmap (5 Hz) → DWB avoids
scan → global costmap → is_path_valid? → replan (BT)
```

**Recommended fixes before custom algorithms:**

1. Set AMCL `tf_broadcast: false` OR remove AMCL from lifecycle when using SLAM mapping
2. Tune footprint to match URDF collision (~0.55 m half-width)
3. Increase lidar `update_rate` if CPU allows
4. Add depth camera to obstacle layer for close-range obstacles

---

## 36. Recommended Implementation Roadmap

| Step | Action | Files | Test |
|------|--------|-------|------|
| 1 | Verify teleop motion | — | `teleop_twist_keyboard` → `/husky1/cmd_vel` |
| 2 | Verify sensors | `gazebo_bridge_husky1.yaml` | `ros2 topic hz /husky1/scan` |
| 3 | Verify TF | EKF + robot_state_publisher | `ros2 run tf2_tools view_frames` |
| 4 | Fix AMCL/SLAM TF | `nav2_params_husky1.yaml` | `ros2 run tf2_ros tf2_echo husky1_map husky1_odom` |
| 5 | SLAM mapping | `slam:=true` | Map grows in RViz |
| 6 | Nav2 goal | `nav2:=true rviz:=true` | 2D Goal Pose |
| 7 | Tune footprint/inflation | `nav2_params_husky1.yaml` | Narrow passage test |
| 8 | Extend autonomy demo | `basic_autonomy_demo.py` | Custom goal logic |
| 9 | Add perception | new package/node | Object markers |
| 10 | Custom planner (if needed) | Nav2 plugin OR replace planner_server plugin | Compare to NavFn |

---

## 37. Testing Strategy

### Unit tests (PROPOSED — none exist in repo)

| Test | Target |
|------|--------|
| `_is_free_with_margin` | `basic_autonomy_demo.py` |
| `_map_cell_to_world` | same |
| Footprint collision | costmap with known grid |

### Integration tests

| Test | Initial conditions | Expected | Pass criteria |
|------|-------------------|----------|---------------|
| T1 Empty env | `simple_trees`, nav2, goal 5 m ahead | Plan + drive | Goal succeeded |
| T2 Static obstacle | Tree between start/goal | Path around tree | No collision, success |
| T3 Multiple obstacles | `large_demo` | Valid path | Success |
| T4 Dynamic obstacle | `dynamic_world_demo` + nav2 | Local avoidance | Robot does not collide; may abort/replan |
| T5 Path blocked mid-route | Obstacle placed in path | Replan or recovery | Recovery or new plan |
| T6 Narrow passage | Between trees | Pass or fail safe | No stuck infinite spin |
| T7 Impossible goal | Goal inside wall | Planner fails | ABORTED, no cmd_vel crash |

---

## 38. Debugging Guide

### Robot does not move

```bash
ros2 topic list | grep cmd_vel
ros2 topic echo /husky1/cmd_vel
ros2 node list | grep -E "controller|nav2|bridge"
ros2 topic info /husky1/cmd_vel -v
```

- No publisher on `cmd_vel` → Nav2 not running or no goal sent
- Publisher exists, no bridge → check `gazebo_bridge` node
- Bridge ok, no Gazebo motion → check Gazebo model `husky1` exists

### Sensor data missing

```bash
ros2 topic hz /husky1/scan
ros2 topic echo /husky1/scan --once
```

- 0 Hz → Gazebo Sensors plugin / rendering (check `ignition_server.config`, GUI/headless flags)
- Bridge issue → check `gazebo_bridge` logs

### TF broken

```bash
ros2 run tf2_ros tf2_echo husky1_map husky1_base_link
ros2 run tf2_tools view_frames
```

- Missing `map→odom` → SLAM not running
- Multiple publishers → AMCL+SLAM conflict
- "jump back in time" → multiple Gazebo instances (README)

### Planner produces no path

- Check goal in `husky1_map`
- Check `/husky1/map` populated
- Check global costmap in RViz for obstacle blocking
- `allow_unknown: true` — unknown space allowed but high cost

### Robot drives into obstacles

- Verify `/husky1/scan` in costmap (RViz layer)
- Check `inflation_radius`, `footprint` size
- Remember VelocityControl may slide along collisions
- Lidar 3 Hz — latency

---

## 39. Complete Start-to-Goal Example

**Scenario:** Husky at (0, 0) must reach (8, 0) in `simple_trees` with `nav2:=true rviz:=true`.

### CURRENT implementation walkthrough

| Step | Node | Interface | Frame | Algorithm | Code |
|------|------|-----------|-------|-----------|------|
| 1. Start pose | EKF + SLAM | TF `map→odom→base_link` | `husky1_map` | EKF fusion | `robot_localization_husky1.yaml` |
| 2. Goal received | `bt_navigator` | `NavigateToPose` action | `husky1_map` | BT | Nav2 external |
| 3. Sensor data | bridge | `/husky1/scan` | `base_scan` | — | `gazebo_bridge_husky1.yaml` |
| 4. Map | SLAM Toolbox | `/husky1/map` | `husky1_map` | Scan matching | `slam_params_husky1.yaml` |
| 5. Obstacles | costmap | obstacle layer | map/odom | Ray marking | `nav2_params_husky1.yaml` |
| 6. Global path | `planner_server` | `/husky1/plan` | `husky1_map` | NavFn | Nav2 |
| 7. Footprint check | costmap | inflation layer | — | Grid cost | Nav2 |
| 8. Follow path | `controller_server` | DWB | `husky1_odom` | DWB sampling | Nav2 |
| 9. cmd_vel | `velocity_smoother` | `/husky1/cmd_vel` | — | Accel limit | Nav2 |
| 10. Motion | Gazebo | VelocityControl | world | Kinematic | `husky.gazebo.xacro` |
| 11. New obstacle (pine) | lidar → costmap | scan | — | Obstacle layer | 5 Hz update |
| 12. Local update | local costmap | — | `husky1_odom` | Rolling window | Nav2 |
| 13. Path unsafe | BT | `is_path_valid` | — | **Likely** replan | Nav2 BT |
| 14. New path | `planner_server` | `/husky1/plan` | `husky1_map` | NavFn | Nav2 |
| 15. Continue | DWB | cmd_vel | — | Avoid + track | Nav2 |
| 16. Goal reached | BT | `goal_reached` condition | — | tolerance 1 m | Nav2 |

### PROPOSED enhancements (not current)

- Dedicated replanning watchdog node
- Camera depth for obstacles below lidar plane
- Student behavior executive replacing random demo

---

## 40. AI Agent Implementation Context

### Repository structure

| Path | Responsibility |
|------|----------------|
| `launch/41068_ignition.launch.py` | Canonical sim bringup |
| `launch/41068_navigation.launch.py` | SLAM + Nav2 wrapper with namespace remaps |
| `config/nav2_params_husky1.yaml` | All Nav2 tuning for Husky |
| `config/slam_params_husky1.yaml` | SLAM tuning |
| `config/gazebo_bridge_husky1.yaml` | Topic bridge definitions |
| `urdf_husky/*.xacro` | Robot + sensors + motion plugins |
| `scripts/basic_autonomy_demo.py` | Example goal sender |

### Reuse (do not duplicate)

- Namespace convention `/husky1`, frames `husky1_*`
- Bridge config pattern for new sensors
- `41068_navigation.launch.py` for Nav2/SLAM
- Nav2 action `navigate_to_pose` for goals

### Do not break

- Namespaced TF (`/husky1/tf` not global `/tf`)
- `SetRemap` for `/map`, `/tf` in navigation launch
- World-level Sensors plugin (do not add per-robot Sensors plugin)
- `gz_model_name` xacro arg matching spawn name

### Missing (build here)

- Custom decision making / behavior executive
- Custom perception beyond brightness demo
- Custom planners (if not using Nav2 plugins)
- UI beyond RViz
- AMCL/SLAM coexistence fix

### Recommended sequence for agents

1. Read `launch/41068_ignition.launch.py` launch graph
2. Confirm topic names in `gazebo_bridge_husky1.yaml`
3. Trace TF in `robot_localization_husky1.yaml` + `slam_params_husky1.yaml`
4. Read `nav2_params_husky1.yaml` for planner/controller/costmap
5. Extend `basic_autonomy_demo.py` OR add new namespaced node
6. Never invent global `/cmd_vel` — use `/husky1/cmd_vel`

---

## 41. AI Agent Modification Rules

1. Inspect existing launch/YAML before adding nodes
2. Do not duplicate Nav2 or SLAM launch logic — extend `41068_navigation.launch.py` or add sibling launch
3. Use namespaced relative topics inside robot nodes (`map`, `scan`, `navigate_to_pose`)
4. Do not invent TF frames — use `husky1_map`, `husky1_odom`, `husky1_base_link`, sensor frames from URDF
5. Add new sensors to URDF gazebo block + `gazebo_bridge_husky1.yaml`
6. Verify `package.xml` exec_depend for new message packages
7. Label IMPLEMENTED vs PROPOSED in docs and comments
8. RViz display ≠ functionality — verify with `ros2 topic info`
9. Distinguish Gazebo ground truth odometry from SLAM estimates
10. Test teleop before Nav2 before custom autonomy
11. Smallest change that works — this is a student starter package
12. Do not add Sensors plugin to robot URDF (world-level only)

---

## 42. Beginner Explanation

### How does a robot know where it is?

The simulator tracks the robot pose. That estimate is published as odometry on `/husky1/odom`. While SLAM runs, scan matching also estimates how the map aligns with the robot (`husky1_map` → `husky1_odom`).

### How does it see obstacles?

A spinning lidar on top of the robot measures distances. In ROS, this appears as `/husky1/scan`. Nav2 marks obstacles on a costmap from these ranges.

### How does it build a map?

SLAM Toolbox watches lidar scans and odometry, and builds `/husky1/map` — a grid where cells are free, occupied, or unknown.

### How does it know where the goal is?

You click a goal in RViz, or a script sends a `NavigateToPose` action with coordinates in the `husky1_map` frame.

### How does it find a route?

Nav2's global planner (NavFn) searches the costmap grid for a low-cost path from start to goal.

### How does it know the route is safe?

The costmap inflates obstacles, and the planner avoids high-cost cells. The local planner (DWB) simulates short trajectories and rejects ones that hit obstacles. The footprint is a 1 m × 1 m square — larger than the physical robot for safety.

### How does it physically move?

Nav2 sends velocity commands on `/husky1/cmd_vel` (forward speed + turn rate). The bridge forwards them to Gazebo, which moves the whole robot model directly.

### What happens when a new obstacle appears?

The lidar sees it within ~⅓ second (3 Hz). The local costmap updates (~5 Hz). The local planner steers around it. If the global path is blocked, Nav2 may replan or run recovery behaviors (spin, backup).

### Why replan?

The first global path assumed an old map. New obstacles invalidate it. Replanning finds a new safe route.

### What is Gazebo doing?

Simulating physics, collisions, sensors, and the forest world.

### What is RViz doing?

Visualizing maps, laser, robot model, plans — it does not move the robot by itself except when you send goals.

### What is ROS 2 doing?

Connecting nodes via topics, actions, TF, and parameters.

### What is TF doing?

Storing coordinate frame relationships so lidar data, map, and goals share a common spatial understanding.

### Map vs sensor data vs path vs velocity command

| Concept | What it is |
|---------|------------|
| Sensor data | Instant measurements (ranges, images) |
| Map | Remembered environment grid |
| Path | Planned sequence of poses |
| Velocity command | Immediate "drive forward 0.5 m/s, turn 0.1 rad/s" |

---

## 43. Master Architecture Diagram

```
                    ┌──────────────────────────────────────┐
                    │         Ignition Gazebo Fortress       │
                    │  World SDF + Physics + Contact         │
                    │  Server Sensors (ogre) — once/world    │
                    │                                        │
                    │  Model husky1:                         │
                    │    VelocityControl ← cmd_vel           │
                    │    OdometryPublisher → odometry        │
                    │    gpu_lidar → scan                    │
                    │    rgbd_camera → camera/*              │
                    │    imu                                 │
                    └───────────────┬────────────────────────┘
                                    │ Gazebo Transport
                                    ▼
                    ┌──────────────────────────────────────┐
                    │   ros_ign_bridge (/husky1)             │
                    │   + global /clock bridge               │
                    └───────────────┬────────────────────────┘
                                    │
         ┌──────────────────────────┼──────────────────────────┐
         ▼                          ▼                          ▼
┌─────────────────┐    ┌──────────────────────┐    ┌─────────────────────┐
│ robot_state_    │    │ robot_localization   │    │ SLAM Toolbox        │
│ publisher       │    │ EKF                  │    │ (slam/nav2 true)    │
│ URDF → tf_static│    │ odom+imu → odom TF   │    │ scan → map          │
└────────┬────────┘    └──────────┬───────────┘    │ map → odom TF       │
         │                        │                └──────────┬──────────┘
         └────────────────────────┴───────────────────────────┘
                                    │ TF + /husky1/map
                                    ▼
         ┌──────────────────────────────────────────────────────────────┐
         │ Nav2 (nav2:=true)                                            │
         │  global_costmap ← map (static) + scan (obstacle)            │
         │  local_costmap  ← scan (obstacle, rolling 10m)               │
         │  planner_server (NavFn) → /plan                              │
         │  controller_server (DWB) → velocity_smoother → cmd_vel       │
         │  bt_navigator ← NavigateToPose (RViz / autonomy demo)         │
         │  behavior_server (recovery)                                    │
         │  amcl (configured — see TF conflict note)                      │
         └──────────────────────────┬───────────────────────────────────┘
                                    │ /husky1/cmd_vel
                                    ▼
                         Gazebo VelocityControl
                                    │
                                    ▼
                              Robot motion
                                    │
                                    ▼
                         New scan → costmap → DWB / replan
```

---

## 44. File / Symbol Reference

| File | Symbol / Config | Responsibility |
|------|-----------------|----------------|
| `launch/41068_ignition.launch.py` | `add_robot()`, `add_navigation_instance()` | Full bringup orchestration |
| `launch/41068_navigation.launch.py` | `generate_launch_description()` | Namespaced SLAM + Nav2 |
| `config/gazebo_bridge_husky1.yaml` | bridge entries | All Husky ROS↔GZ topics |
| `config/robot_localization_husky1.yaml` | EKF params | odom fusion |
| `config/slam_params_husky1.yaml` | SLAM params | Online mapping |
| `config/nav2_params_husky1.yaml` | Nav2 stack | Planning, control, costmaps |
| `urdf_husky/husky.gazebo.xacro` | VelocityControl, sensors | Sim plugins |
| `urdf_husky/husky.urdf.xacro` | links, joints | Robot geometry |
| `urdf_husky/wheel.urdf.xacro` | `wheel_separation`, `wheel_radius` | Wheel geometry |
| `scripts/basic_autonomy_demo.py` | `BasicAutonomyDemo` | Example NavigateToPose client |
| `scripts/dynamic_world_demo.py` | `DynamicWorldDemo` | Gazebo set_pose demo |
| `config/ignition_server.config` | world plugins | Physics, Sensors, Contact |
| `worlds/simple_trees.sdf` | world `simple_trees` | Basic forest |
| `worlds/large_demo.sdf` | world `large_demo` | Large forest + demo models |
| `config/41068_husky1.rviz` | RViz displays | Visualization |

---

## 45. Final Findings

### Critical investigation answers

| Question | Answer |
|----------|--------|
| How does Husky move? | `VelocityControl` Gazebo plugin — kinematic cmd_vel |
| What command? | `geometry_msgs/Twist` on `/husky1/cmd_vel` |
| Who publishes? | Nav2 `velocity_smoother` (when nav2 active) or teleop |
| Who consumes? | `ros_ign_bridge` → Gazebo |
| Wheel motion? | Visual/physics wheels; not primary drive interface |
| Sensors? | Lidar, RGB-D, IMU, odometry — see §10 |
| Where is robot? | EKF odom (sim); SLAM map→odom (estimated) |
| Environment representation? | SLAM OccupancyGrid + Nav2 costmaps |
| Obstacle detection? | Lidar → Nav2 obstacle layer |
| Planner implemented? | **Yes** — Nav2 NavFn (external), not custom code |
| Footprint/safety? | 1 m box footprint, 2 m inflation |
| Path following? | Nav2 DWB at 5 Hz |
| Replanning? | **Partial** — local avoidance + BT path valid (**Likely**) |
| What's missing? | Custom perception, behavior executive, real diff-drive, emergency stop |
| What to implement first? | Fix AMCL/SLAM TF, tune footprint, then extend autonomy demo |

### Current vs target (summary table)

| Capability | Current State | Evidence | Missing Work |
|------------|---------------|----------|--------------|
| Robot simulation | **IMPLEMENTED** | `41068_ignition.launch.py` | — |
| Sensors | **IMPLEMENTED** | `husky.gazebo.xacro`, bridges | Higher rates, more sensors |
| TF | **IMPLEMENTED** | EKF, SLAM, URDF | AMCL conflict resolution |
| Localisation | **PARTIAL** | EKF + SLAM; AMCL redundant | Choose SLAM vs AMCL mode |
| Mapping | **IMPLEMENTED** | SLAM Toolbox | Semantic/custom maps |
| Obstacle detection | **IMPLEMENTED** | Nav2 obstacle layer | Camera depth, faster lidar |
| Global planner | **IMPLEMENTED** (Nav2) | `nav2_params_husky1.yaml` | Custom plugin if needed |
| Local planner | **IMPLEMENTED** (Nav2 DWB) | same | Tuning / alternatives |
| Controller | **IMPLEMENTED** | VelocityControl + Nav2 | ros2_control if real robot |
| Goal handling | **IMPLEMENTED** | RViz, autonomy demo | Mission-level goals |
| Collision checking | **IMPLEMENTED** | costmap + DWB | Emergency stop |
| Replanning | **PARTIAL** | BT + local planner | Explicit dynamic obstacle layer |
| Safety stop | **NOT IMPLEMENTED** | — | Watchdog node |

---

## 46. MASTER_CONTEXT Discrepancies

| MASTER_CONTEXT claim | Code reality | Severity |
|---------------------|--------------|----------|
| Nav2 config in `nav_params.yaml` | Files are `nav2_params_husky1.yaml` | Doc only |
| Husky "differential drive" | `VelocityControl` on base, not diff-drive plugin | Behavioral |
| Five-layer autonomy stack | Only Nav2 + demo node; no Map Processor / Behavior Executive | Architectural |
| AMCL as localisation | SLAM mapping mode also publishes map→odom | TF conflict risk |
| Course algorithms (A*, RRT, etc.) | Not in repo; NavFn + DWB only | Expected — student work |

---

*End of knowledge base. Verify runtime behavior with `nav2:=true rviz:=true` and `ros2 topic list -t` on your machine.*
