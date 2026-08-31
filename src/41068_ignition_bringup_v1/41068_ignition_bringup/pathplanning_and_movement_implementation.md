# Path Planning & Robot Movement Implementation Plan

> **Package:** `41068_ignition_bringup` (UTS 41068 Robotics Studio 1)  
> **Evidence date:** 2026-08-28  
> **ROS distribution:** ROS 2 Humble  
> **Simulator:** Ignition Gazebo Fortress  
> **Purpose:** Authoritative implementation contract for a future AI coding agent  
> **Companion references:** `MASTER_CONTEXT.md` (course concepts), `master_robot_movement_pathplanning.md` (prior audit)

> **Implementation status (2026-08-28):** The autonomous loop is implemented and tested in `simple_trees`. Start at **§51 How to Run**, **§52 How to Change Start and Goal**, and **§65 Implementation Log**. Sections 1–50 keep the original plan for context; where they disagree with the log or the repository, the repository wins.

---

## 1. Executive Summary

This repository **already contains a working simulation stack** with namespaced Husky UGV (`/husky1`), Gazebo sensor bridges, EKF odometry fusion, optional **SLAM Toolbox** online mapping, and optional **Nav2** global/local planning with DWB control. Autonomous navigation is **not active by default** (`nav2:=false`); it must be enabled at launch.

**What works today (verified in code):**
- Gazebo world + Husky spawn + sensor publishing
- `cmd_vel` → bridge → Gazebo `VelocityControl` → robot motion
- SLAM map building when `slam:=true` or `nav2:=true`
- Full Nav2 stack when `nav2:=true` (NavFn global planner, DWB local planner, costmaps, BT navigator, recovery behaviors)
- RViz goal sending via `NavigateToPose` action
- Example random-walk autonomy via `basic_autonomy_demo.py` (Nav2 action client, **not** configurable start→goal demo)

**What is missing or broken for the target objective:**
- Configurable start position via launch arguments (spawn is hard-coded)
- Configurable goal in a single demo command (start→goal→replan demo)
- Resolved **AMCL + SLAM TF conflict** (`tf_broadcast: true` on both)
- Tuned footprint/inflation matching physical Husky (~0.99 × 0.57 m, not 1.0 × 1.0 m)
- Dedicated mission/behavior node for structured start→goal navigation
- Optional safety watchdog (collision monitor / min-range stop)
- Camera depth not used in costmaps
- Slow lidar rate (3 Hz) limits reactive replanning latency

**Recommended approach:** **Extend and fix the existing Nav2 + SLAM stack** rather than replace it. The course starter package already wires the correct architecture; implementation work should focus on configuration fixes, launch ergonomics, mission/demo layer, tuning, and testing — not a greenfield navigation stack.

---

## 2. Project Objective

Build a simulated Husky that:

1. Starts from a **configurable pose**
2. Receives a **configurable goal pose**
3. Uses **live lidar** (and optionally IMU/odom/camera) to map and detect obstacles
4. Plans a **global path**, follows it with **local obstacle avoidance**
5. **Replans** when the path becomes blocked or unsafe
6. Reaches the goal safely
7. Demonstrates via **RViz** and a **Python entry point**

Target behaviour loop:

```text
START → initialise sensors/TF/map → GOAL → global plan → follow path
  → continuous sensor updates → costmap updates → path valid?
      → YES: continue | NO: replan → continue → GOAL reached
```

---

## 3. Source-of-Truth Rules

| Priority | Source | Use for |
|----------|--------|---------|
| 1 | Repository code, YAML, launch files | Runtime truth |
| 2 | `master_robot_movement_pathplanning.md` | Prior reverse-engineering |
| 3 | `MASTER_CONTEXT.md` | Course concepts, intended stack |
| 4 | External Nav2/SLAM docs | Package behaviour defaults |

**Verified discrepancies:**

| Claim (MASTER_CONTEXT / README) | Code reality | Severity |
|--------------------------------|--------------|----------|
| Nav2 config in `nav_params.yaml` | Files are `config/nav2_params_husky1.yaml` | Documentation only |
| Husky is "differential drive" | `VelocityControl` on base — kinematic, not diff-drive plugin | Behavioural |
| Full five-layer autonomy stack | Nav2 + SLAM + demo node only | Architectural gap |
| `basic_autonomy_demo.py` demonstrates navigation | Demonstrates **random Nav2 goals**, not start→goal mission | Functional gap |
| Nav2 always available | Only when `nav2:=true` at launch | Operational |

---

## 4. Existing Repository Overview

### 4.1 Package structure

```text
41068_ignition_bringup/
├── config/              # Nav2, SLAM, EKF, bridges, RViz, Gazebo server
├── launch/              # 6 launch files
├── scripts/             # basic_autonomy_demo.py, dynamic_world_demo.py
├── urdf_husky/          # Husky URDF + Gazebo plugins
├── urdf_parrot/         # Parrot URDF + Gazebo plugins
├── worlds/              # simple_trees.sdf, large_demo.sdf
├── models/              # grass_plane, forest_plane, forest_wall
├── knowldge/            # Course seminar extractions (not runtime)
├── package.xml          # v1.0.5
├── CMakeLists.txt
├── README.md
├── MASTER_CONTEXT.md
└── master_robot_movement_pathplanning.md
```

### 4.2 Single ROS 2 package

Only one package: `41068_ignition_bringup` (`ament_cmake`). No C++ nodes. Two Python executables installed via `CMakeLists.txt`.

---

## 5. Current Architecture

### 5.1 Default launch (`nav2:=false`, `slam:=false`)

```text
Ignition Gazebo → ros_ign_bridge → robot_state_publisher + EKF
  → sensors publish (/husky1/scan, /odom, /imu, /camera/*)
  → NO map, NO planner, NO autonomous motion (teleop only)
```

### 5.2 Full navigation launch (`nav2:=true`)

```text
Gazebo (physics + gpu_lidar + rgbd_camera + IMU + VelocityControl)
  ↕ ros_ign_bridge (/husky1/*)
robot_state_publisher + robot_localization EKF (odom→base_link)
SLAM Toolbox (scan → /husky1/map, map→odom TF)
Nav2 (global/local costmaps, NavFn, DWB, BT navigator, behaviors)
  → /husky1/cmd_vel → bridge → Gazebo VelocityControl
```

Evidence: `launch/41068_ignition.launch.py`, `launch/41068_navigation.launch.py`, `config/nav2_params_husky1.yaml`.

---

## 6. Current Package Inventory

| Component | Package | Status | Evidence |
|-----------|---------|--------|----------|
| Simulation bringup | `41068_ignition_bringup` | IMPLEMENTED | `launch/41068_ignition.launch.py` |
| Gazebo integration | `ros_ign_gazebo`, `ros_ign_bridge` | IMPLEMENTED | launch + bridges |
| Robot model | `41068_ignition_bringup/urdf_husky` | IMPLEMENTED | `husky.urdf.xacro` |
| Sensor bridges | `ros_ign_bridge` | IMPLEMENTED | `gazebo_bridge_husky1.yaml` |
| EKF localisation | `robot_localization` | IMPLEMENTED | `robot_localization_husky1.yaml` |
| SLAM mapping | `slam_toolbox` | IMPLEMENTED (optional) | `41068_navigation.launch.py` |
| Nav2 navigation | `nav2_bringup` / `navigation2` | IMPLEMENTED (optional) | `41068_navigation.launch.py` |
| Custom planners | — | MISSING | No package source |
| Custom perception | — | MISSING | Only demo brightness |
| Behavior executive | — | MISSING | Only random demo |
| ros2_control | — | NOT IMPLEMENTED | No controller YAML |
| Tests | — | MISSING | No navigation tests |

---

## 7. Complete Implementation Inventory

### 7.1 Simulation

| Component | Package | File | Status | Purpose |
|-----------|---------|------|--------|---------|
| World loader | `ros_ign_gazebo` | `launch/41068_ignition.launch.py:344` | IMPLEMENTED | Loads `worlds/{world}.sdf` |
| Server plugins | `41068_ignition_bringup` | `config/ignition_server.config` | IMPLEMENTED | Physics, Sensors, Contact |
| Robot spawn | `ros_ign_gazebo` | `launch/41068_ignition.launch.py:118` | IMPLEMENTED | `create` node at hard-coded pose |
| Clock bridge | `ros_ign_bridge` | `config/gazebo_bridge_clock.yaml` | IMPLEMENTED | `/clock` for `use_sim_time` |
| Dynamic world demo | `41068_ignition_bringup` | `scripts/dynamic_world_demo.py` | IMPLEMENTED | Gazebo `set_pose` only; no ROS planning |

### 7.2 Robot model

| Component | File | Status | Notes |
|-----------|------|--------|-------|
| Husky URDF | `urdf_husky/husky.urdf.xacro` | IMPLEMENTED | 46 kg, 4 wheels |
| Gazebo plugins | `urdf_husky/husky.gazebo.xacro` | IMPLEMENTED | VelocityControl, sensors |
| Wheel geometry | `urdf_husky/wheel.urdf.xacro` | IMPLEMENTED | sep=0.5708 m, r=0.1651 m |

### 7.3 Sensors

| Sensor | Plugin | ROS topic | Msg | Frame | Rate | Status |
|--------|--------|-----------|-----|-------|------|--------|
| Lidar | `gpu_lidar` | `/husky1/scan` | `LaserScan` | `husky1_base_scan` | 3 Hz | IMPLEMENTED |
| Lidar PC | same | `/husky1/scan/points` | `PointCloud2` | `husky1_base_scan` | 3 Hz | IMPLEMENTED |
| RGB-D | `rgbd_camera` | `/husky1/camera/image` | `Image` | `husky1_camera_link` | 3 Hz | IMPLEMENTED |
| Depth | same | `/husky1/camera/depth/image` | `Image` | `husky1_camera_link` | 3 Hz | IMPLEMENTED |
| Depth cloud | same | `/husky1/camera/depth/points` | `PointCloud2` | `husky1_camera_link` | 3 Hz | IMPLEMENTED |
| IMU | `imu` | `/husky1/imu` | `Imu` | `husky1_imu_link` | 100 Hz | IMPLEMENTED |
| Raw odom | OdometryPublisher | `/husky1/odometry` | `Odometry` | `husky1_odom` | 20 Hz | IMPLEMENTED |
| Joint states | JointStatePublisher | `/husky1/joint_states` | `JointState` | wheel links | — | IMPLEMENTED |

Bridge evidence: `config/gazebo_bridge_husky1.yaml`. Sensor config: `urdf_husky/husky.gazebo.xacro:109-176`.

### 7.4 Sensor processing / fusion

| Component | Status | Details |
|-----------|--------|---------|
| Dedicated perception nodes | MISSING | |
| EKF sensor fusion | IMPLEMENTED | Odom velocities + IMU yaw rate → `/husky1/odom` |
| Multi-sensor fusion for mapping | PARTIAL | SLAM uses scan + TF from odom |

EKF evidence: `config/robot_localization_husky1.yaml` — fuses vx, vy, vyaw from odometry; yaw rate from IMU; **does not** fuse position.

### 7.5 TF / Localisation

| Transform | Publisher | Status | Rate |
|-----------|-----------|--------|------|
| URDF static | `robot_state_publisher` | IMPLEMENTED | static |
| `odom → base_link` | EKF | IMPLEMENTED | 30 Hz |
| `map → odom` | SLAM Toolbox | IMPLEMENTED when SLAM active | ~50 Hz |
| `map → odom` | AMCL | CONFIGURED | **CONFLICT RISK** |

AMCL: `config/nav2_params_husky1.yaml:32` `tf_broadcast: true`  
SLAM: `config/slam_params_husky1.yaml:17` `transform_publish_period: 0.02`

### 7.6 Mapping

| Component | Status | Input | Output |
|-----------|--------|-------|--------|
| SLAM Toolbox async | IMPLEMENTED (optional) | `/husky1/scan`, TF | `/husky1/map`, map→odom |
| Static map server | CONFIGURED empty | — | `yaml_filename: ''` |
| Pre-built map | MISSING | — | — |

SLAM mode: `mapping` (`slam_params_husky1.yaml:14`). Resolution: 0.05 m. Update interval: 2.0 s.

### 7.7 Costmaps / obstacle representation

| Layer | Costmap | Status | Source |
|-------|---------|--------|--------|
| Static (SLAM map) | Global | IMPLEMENTED | `/husky1/map` |
| Obstacle (lidar) | Global + Local | IMPLEMENTED | `/husky1/scan` |
| Inflation | Global + Local | IMPLEMENTED | radius 2.0 m |
| Voxel / depth | — | MISSING | Camera not in costmap |
| Dynamic obstacle layer | — | MISSING | Only rolling lidar marks |

Evidence: `config/nav2_params_husky1.yaml:174-248`.

### 7.8 Global planning

| Component | Status | Algorithm | Plugin |
|-----------|--------|-----------|--------|
| `planner_server` | IMPLEMENTED (nav2) | NavFn (Dijkstra-like) | `nav2_navfn_planner/NavfnPlanner` |
| Custom A*/RRT/PRM | MISSING | — | — |

`use_astar: false`, `allow_unknown: true` — `nav2_params_husky1.yaml:267-270`.

### 7.9 Local planning / control

| Component | Status | Algorithm | Plugin |
|-----------|--------|-----------|--------|
| `controller_server` | IMPLEMENTED | DWB | `dwb_core::DWBLocalPlanner` |
| `velocity_smoother` | IMPLEMENTED | Accel limits | Nav2 |
| `behavior_server` | IMPLEMENTED | Recovery | spin, backup, wait, etc. |
| Gazebo actuation | IMPLEMENTED | VelocityControl | Not ros2_control |

Controller frequency: 5 Hz. Max vel: 1.0 m/s, 1.5 rad/s.

### 7.10 Replanning

| Mechanism | Status | Evidence |
|-----------|--------|----------|
| Local avoidance (DWB) | IMPLEMENTED | BaseObstacle critic |
| Costmap updates | IMPLEMENTED | 5 Hz |
| BT `is_path_valid` | CONFIGURED | `nav2_params_husky1.yaml:65` |
| Global replan on invalid path | LIKELY (Nav2 default BT) | Not custom code |
| Custom replan watchdog | MISSING | — |

### 7.11 Goal handling

| Source | Interface | Status |
|--------|-----------|--------|
| RViz Nav2 Goal | `NavigateToPose` action | IMPLEMENTED |
| `basic_autonomy_demo.py` | `NavigateToPose` action | IMPLEMENTED (random goals) |
| Launch-arg goals | — | MISSING |
| Parameter goals | — | MISSING |

### 7.12 Demo

| Script | Status | Behaviour |
|--------|--------|-----------|
| `basic_autonomy_demo.py` | IMPLEMENTED | Random free-space goals via Nav2 |
| `dynamic_world_demo.py` | IMPLEMENTED | Moves Gazebo models; no Nav2 integration |

---

## 8. Current Sensor Packages

| Sensor | Providing package | Bridge file | Actually used by |
|--------|-------------------|-------------|------------------|
| Lidar | Gazebo `gpu_lidar` + `ros_ign_bridge` | `gazebo_bridge_husky1.yaml:29-34` | SLAM, Nav2 costmaps, RViz |
| Camera RGB | Gazebo `rgbd_camera` + bridge | lines 43-48 | `basic_autonomy_demo.py`, RViz |
| Camera depth | same | lines 50-61 | RViz only |
| IMU | Gazebo IMU + bridge | lines 15-20 | EKF |
| Odometry | Gazebo OdometryPublisher + bridge | lines 8-13 | EKF, Nav2 |
| Encoders | JointStatePublisher | lines 1-6 | RViz/URDF only |

**Why installed:** Course simulation starter; bridges defined in package config.  
**Redundant:** Unprefixed legacy configs (`gazebo_bridge_husky.yaml`, etc.) — **PRESENT BUT UNUSED**.

---

## 9. Current Movement Packages

### 9.1 Movement chain (verified)

```text
Nav2 velocity_smoother (when nav2:=true)
  OR teleop_twist_keyboard
  OR any node publishing Twist
    ↓
/husky1/cmd_vel  (geometry_msgs/Twist)
    ↓
ros_ign_bridge  (ROS_TO_GZ)  config/gazebo_bridge_husky1.yaml:22-27
    ↓
/model/husky1/cmd_vel  (Gazebo Transport)
    ↓
ignition::gazebo::systems::VelocityControl  urdf_husky/husky.gazebo.xacro:80-87
    ↓
Gazebo physics moves entire model (kinematic base velocity)
    ↓
OdometryPublisher → /husky1/odometry → EKF → /husky1/odom
```

**No** `ros2_control`, **no** diff-drive controller, **no** custom movement C++ node.

### 9.2 cmd_vel semantics

| Field | Used | Limit (Nav2) |
|-------|------|--------------|
| `linear.x` | Yes | max 1.0 m/s |
| `linear.y` | No (max_vel_y: 0) | — |
| `angular.z` | Yes | max 1.5 rad/s |

---

## 10. Current Localisation Packages

| Layer | Package | Node | Type | Frame |
|-------|---------|------|------|-------|
| Short-term pose | `robot_localization` | `ekf_node` | Filtered sim odom | `husky1_odom`→`husky1_base_link` |
| Map-relative | `slam_toolbox` | `async_slam_toolbox_node` | Scan matching | `husky1_map`→`husky1_odom` |
| Particle filter | `nav2_amcl` | lifecycle node | **Configured but conflicts with SLAM** | would publish map→odom |

**Ground truth:** Gazebo physics pose (not exposed as ROS topic). `/husky1/odometry` is near ground truth.

**Drift:** Minimal in sim odometry; SLAM corrects map alignment.

---

## 11. Current Mapping Packages

| Package | Mode | Output | Dynamic updates |
|---------|------|--------|-----------------|
| `slam_toolbox` | `mapping` | `/husky1/map` OccupancyGrid | Yes, 2 s throttle + scan matching |

Supports: static obstacles (as mapped), unknown space expansion, slow update of new static obstacles.  
Does **not** reliably track fast dynamic obstacles in the static map (costmap handles reactive layer).

---

## 12. Current Perception Packages

**NOT IMPLEMENTED** as dedicated nodes. Raw sensors feed SLAM and Nav2 directly.

`basic_autonomy_demo.py` computes image brightness only — placeholder perception.

---

## 13. Current Path Planning Packages

When `nav2:=true`:

| Server | Plugin | Algorithm |
|--------|--------|-----------|
| `planner_server` | NavfnPlanner | Grid navigation function |
| `smoother_server` | SimpleSmoother | Path smoothing |
| `bt_navigator` | Default Nav2 BT | Orchestration + replan triggers |

No custom planner code in repository.

---

## 14. Current Local Planning / Controller Packages

| Server | Plugin | Frequency |
|--------|--------|-----------|
| `controller_server` | DWBLocalPlanner | 5 Hz |
| `velocity_smoother` | Nav2 smoother | 20 Hz |

Critics: RotateToGoal, Oscillation, BaseObstacle, GoalAlign, PathAlign, PathDist, GoalDist.

---

## 15. Current Replanning Functionality

### 15.1 What happens if obstacle appears in front of moving Husky?

**With `nav2:=true` (LIKELY behaviour from Nav2 defaults + config):**

1. Lidar detects obstacle (~333 ms at 3 Hz)
2. Local costmap updates at 5 Hz — obstacle marked within 10 m
3. DWB BaseObstacle penalizes trajectories through high cost — robot slows/steers
4. If global path blocked, BT `nav2_is_path_valid_condition_bt_node` may trigger replan
5. Recovery behaviors (spin, backup) if stuck

**Without `nav2:=true`:** Robot drives into obstacle (physics collision stops interpenetration only).

**Evidence gap:** Default BT XML not overridden in repo — replan timing is Nav2 Humble default (Likely, not traced in local source).

### 15.2 Dynamic vs static obstacles

| Type | Detection | Representation | Persistence |
|------|-----------|----------------|-------------|
| Static (trees) | Lidar | SLAM map + costmap | Persistent in map |
| New static | Lidar | Costmap immediate; SLAM delayed ~2 s | Partial |
| Moving (demo_animal) | Lidar if in FOV | Local costmap only | Cleared when gone |

No obstacle tracking, velocity prediction, or dedicated dynamic layer.

---

## 16. Current TF Architecture

```text
husky1_map
 └── husky1_odom          [SLAM (+ AMCL conflict risk)]
      └── husky1_base_link [EKF]
           ├── husky1_imu_link
           ├── husky1_base_scan
           ├── husky1_camera_link (+ optical frames)
           └── husky1_*_wheel_link (×4)
```

TF topics: `/husky1/tf`, `/husky1/tf_static` (namespaced remaps in launch).

Planning frames: global=`husky1_map`, local=`husky1_odom`, control base=`husky1_base_link`.

---

## 17. Current Gazebo Architecture

| Item | Value | File |
|------|-------|------|
| Simulator | Ignition Fortress | README |
| Render | ogre (+ software GL env vars on WSL) | `41068_ignition.launch.py:331-334` |
| Worlds | `simple_trees`, `large_demo` | `worlds/` |
| Husky spawn | (0, 0, 0.4), yaw=0, delay 3 s | `41068_ignition.launch.py:390-393` |
| Parrot spawn | (2, 0, 0.8), delay 6 s | lines 408-411 |
| Sensors plugin | World-level only | `ignition_server.config` — **do not add per-robot** |

---

## 18. Current RViz Architecture

Config: `config/41068_husky1.rviz`

| Display | Topic | Real data? |
|---------|-------|------------|
| Fixed frame | `husky1_map` | Yes when SLAM active |
| RobotModel | robot_description | Yes |
| LaserScan | `/husky1/scan` | Yes |
| Map | `/husky1/map` | Yes when SLAM active |
| Path | `/husky1/plan` | Only when Nav2 planning |
| Costmap | global/local costmap | When Nav2 active |
| Nav2 panel | actions/services | When Nav2 active |

---

## 19. Current basic_autonomy_demo.py

**File:** `scripts/basic_autonomy_demo.py`  
**Class:** `BasicAutonomyDemo`

| Aspect | Detail |
|--------|--------|
| Imports | rclpy, nav2_msgs, tf2_ros, numpy |
| Subscriptions | `map` (OccupancyGrid), `camera/image` |
| Action client | `navigate_to_pose` (NavigateToPose) |
| TF | `{robot}_map` → `{robot}_base_link` |
| Algorithm | Random free cell sampling; brightness biases goal distance |
| Launches sim? | **No** — requires main launch running |
| Publishes cmd_vel? | **No** — uses Nav2 action |
| Start/goal config? | **No** — random goals only |

Launch: `41068_autonomy_demo.launch.py` — namespace `husky1` or `parrot1`.

---

## 20. Complete Current Data Flow

```text
[Gazebo gpu_lidar] → /model/husky1/scan → bridge → /husky1/scan
  ├→ SLAM Toolbox → /husky1/map + TF map→odom
  └→ Nav2 obstacle layers → local/global costmaps

[Gazebo OdometryPublisher] → /husky1/odometry → EKF → /husky1/odom + TF odom→base

[Goal: RViz or demo] → NavigateToPose → bt_navigator
  → planner_server → /husky1/plan
  → controller_server (DWB) → velocity_smoother → /husky1/cmd_vel
  → bridge → VelocityControl → motion → new scans → loop
```

---

## 21. Current Start Configuration

| Method | Location | Values | Configurable at launch? |
|--------|----------|--------|-------------------------|
| Gazebo spawn | `41068_ignition.launch.py` `add_robot()` | Husky: x=0, y=0, z=0.4, yaw=0 | **No** — hard-coded strings |
| Runtime pose | TF | From spawn + motion | N/A |

Parrot spawn z documented as editable in README (line 166).

---

## 22. Current Goal Configuration

| Method | Interface | Frame | Status |
|--------|-----------|-------|--------|
| RViz "Nav2 Goal" | `/husky1/navigate_to_pose` action | `husky1_map` | IMPLEMENTED |
| Autonomy demo | Same action, random coords | `husky1_map` | IMPLEMENTED |
| Launch args | — | — | MISSING |
| Python script params | — | — | MISSING |

Goal tolerance: xy=1.0 m, yaw=1.57 rad (~90°) — `nav2_params_husky1.yaml:124-125`.

---

## 23. Current System Limitations

1. Nav2/SLAM off by default — no autonomous navigation without launch args
2. AMCL + SLAM both publish map→odom TF
3. Spawn pose not launch-configurable
4. No single-command start→goal demo
5. Footprint (1.0×1.0 m) oversized vs URDF (~0.99×0.57 m chassis)
6. Lidar 3 Hz — ~333 ms detection latency
7. Camera depth unused for navigation
8. Large goal tolerance (1 m)
9. `basic_autonomy_demo.py` is random walk, not mission navigation
10. No automated tests
11. Nav2/slam not in `package.xml` exec_depend (install via README apt only)
12. No emergency stop / collision monitor

---

## 24. Package / Technology Research

Evaluated for **ROS 2 Humble + Ignition Fortress + existing Husky sim**:

| Technology | Humble compatible | Fit for this repo |
|------------|-------------------|-------------------|
| **Nav2** | Yes | **Already integrated** — keep |
| **SLAM Toolbox** | Yes | **Already integrated** — keep for online mapping |
| **robot_localization EKF** | Yes | **Already integrated** — keep |
| **AMCL** | Yes | **Disable TF** when using SLAM mapping |
| **NavFn planner** | Yes | Default — adequate for demo |
| **Smac Planner** | Yes (Humble) | Optional upgrade for smoother paths |
| **DWB controller** | Yes | Default — adequate |
| **Regulated Pure Pursuit** | Yes | Optional simpler local controller |
| **MPPI controller** | Yes | Higher CPU; optional P2 |
| **nav2_collision_monitor** | Yes | Recommended P1 safety add-on |
| **Cartographer** | Yes | Redundant with SLAM Toolbox |
| **ros2_control + diff_drive** | Possible | **Not needed for sim** — VelocityControl works |
| **Depth costmap layer** | Nav2 supported | Optional P2 for close obstacles |

---

## 25. Package Comparison / Evaluation Matrix

| Package / Technology | Purpose | Current? | Recommended? | Why | Integration Difficulty | Benefit |
|---------------------|---------|:--------:|:------------:|-----|------------------------|---------|
| `ros_ign_gazebo/bridge` | Sim↔ROS | Yes | **Keep** | Core sim I/O | — | Essential |
| `robot_localization` EKF | odom fusion | Yes | **Keep** | Stable odom TF | — | Essential |
| `slam_toolbox` | Online mapping | Yes | **Keep** | Course standard | — | Essential |
| `nav2_bringup` | Full stack | Yes (optional) | **Keep + enable** | Already wired | Low | Essential |
| AMCL | Particle localisation | Configured | **Disable TF** | Conflicts with SLAM | Low | Stability |
| NavFn | Global plan | Yes | **Keep** (P0) | Works out of box | — | Adequate |
| Smac Planner 2D | Global plan | No | **Optional P2** | Better paths in clutter | Medium | Quality |
| DWB | Local control | Yes | **Keep** (P0) | Proven in stack | — | Essential |
| Regulated Pure Pursuit | Local control | No | **Optional P2** | Simpler tuning | Medium | Maintainability |
| MPPI | Local control | No | **Optional P3** | Advanced avoidance | High | Diminishing returns |
| `nav2_collision_monitor` | Safety stop | No | **Add P1** | Emergency braking | Medium | Safety demo |
| Depth in costmap | Close obstacles | No | **Optional P2** | Low obstacles | Medium | Robustness |
| Custom A*/RRT | Global plan | No | **Do not add P0** | Nav2 covers requirement | High | Unnecessary |
| `ros2_control` | Wheel control | No | **Defer** | Sim uses VelocityControl | High | Real robot only |
| `basic_autonomy_demo.py` | Demo | Yes | **Extend** | Good ROS wiring template | Low | User-facing |
| Legacy unprefixed YAML | Templates | Unused | **Remove or document** | Confusion risk | Low | Clarity |

---

## 26. Recommended Technology Stack

### RECOMMENDED IMPLEMENTATION ARCHITECTURE

**Stack:** Existing Ignition Fortress sim + ros_ign_bridge + EKF + SLAM Toolbox (mapping) + Nav2 (NavFn + DWB + BT) + extended mission demo node + optional collision monitor.

**Do not replace Nav2.** Fix, tune, and extend.

| Layer | Technology | Node(s) |
|-------|------------|---------|
| Simulation | Ignition Fortress | Gazebo server |
| Actuation | VelocityControl plugin | (Gazebo system) |
| Bridge | ros_ign_bridge | `gazebo_bridge` per robot |
| State | robot_state_publisher | URDF TF |
| Localisation (odom) | robot_localization EKF | `robot_localization` |
| Localisation (map) | SLAM Toolbox | `async_slam_toolbox_node` |
| Mapping | SLAM Toolbox | `/husky1/map` |
| Global planning | Nav2 NavFn | `planner_server` |
| Local planning | Nav2 DWB | `controller_server` |
| Orchestration | Nav2 BT navigator | `bt_navigator` |
| Safety | nav2_collision_monitor (new P1) | optional |
| Mission / demo | Extended `basic_autonomy_demo.py` or `mission_nav_demo.py` | Python node |
| Visualisation | RViz2 | `41068_husky1.rviz` |

---

## 27. Recommended Target Architecture

```text
                         ┌─────────────────────┐
                         │  Ignition Gazebo    │
                         │  World + Husky1     │
                         │  VelocityControl    │
                         │  gpu_lidar/IMU/cam  │
                         └─────────┬───────────┘
                                   │ Gazebo Transport
                                   ▼
                         ┌─────────────────────┐
                         │   ros_ign_bridge    │
                         │   /husky1/* topics  │
                         └─────────┬───────────┘
                                   │
         ┌─────────────────────────┼─────────────────────────┐
         ▼                         ▼                         ▼
┌─────────────────┐    ┌──────────────────────┐    ┌─────────────────────┐
│ robot_state_    │    │ robot_localization   │    │ SLAM Toolbox        │
│ publisher       │    │ EKF: odom→base_link  │    │ scan→map, map→odom  │
└────────┬────────┘    └──────────┬───────────┘    └──────────┬──────────┘
         │                        │                           │
         └────────────────────────┴───────────────────────────┘
                                   │ TF + /husky1/map
                                   ▼
         ┌─────────────────────────────────────────────────────────────┐
         │ Nav2 (nav2:=true)                                           │
         │  global_costmap ← SLAM map + live scan                       │
         │  local_costmap  ← live scan (10 m rolling)                  │
         │  planner_server (NavFn) → /husky1/plan                      │
         │  controller_server (DWB) → velocity_smoother → cmd_vel      │
         │  bt_navigator ← NavigateToPose                              │
         │  [collision_monitor] ← scan (optional safety)               │
         │  AMCL: tf_broadcast=false OR lifecycle deactivated          │
         └─────────────────────────┬───────────────────────────────────┘
                                   │ /husky1/cmd_vel
                                   ▼
                              Husky moves
                                   │
                                   ▼
              Mission demo / RViz goal → replan loop
```

---

## 28. Sensor Architecture

### 28.1 Primary navigation sensor: Lidar

- Topic: `/husky1/scan`
- Used by: SLAM, global/local costmap obstacle layers
- **P1 tuning:** Increase `update_rate` in `husky.gazebo.xacro` from 3→10 Hz if CPU allows

### 28.2 Supporting sensors

| Sensor | Role | Fusion |
|--------|------|--------|
| Odometry | EKF velocity fusion | EKF |
| IMU | EKF yaw rate | EKF |
| Camera RGB | Demo perception placeholder | None for Nav2 |
| Camera depth | **Optional** close-range costmap | P2 |

### 28.3 Sensor fusion recommendation

**Keep existing EKF** — sufficient for sim where odom is high quality.  
**Do not add** full `robot_localization` map→odom fusion or custom UKF unless moving to real hardware.

**Optional P2:** Add depth point cloud to local costmap obstacle layer for obstacles below lidar plane or very close range.

---

## 29. Localisation Architecture

### 29.1 Recommended mode: SLAM mapping (online)

| Transform | Publisher | Notes |
|-----------|-----------|-------|
| `odom→base_link` | EKF | Continuous pose |
| `map→odom` | SLAM Toolbox **only** | Disable AMCL TF |

### 29.2 AMCL fix (P0)

In `config/nav2_params_husky1.yaml`:

```yaml
amcl:
  ros__parameters:
    tf_broadcast: false
```

Alternative: remove AMCL from Nav2 lifecycle via custom params (more invasive).

### 29.3 Future static-map mode (P3)

For repeated runs on saved map: switch SLAM to `localization` mode + AMCL OR map_server with saved YAML — not required for initial demo.

---

## 30. Mapping Architecture

- **Keep SLAM Toolbox** `mode: mapping` for live exploration demo
- Map topic: `/husky1/map` (OccupancyGrid, 0.05 m resolution)
- Nav2 global costmap static layer subscribes to same map
- **P1:** Reduce `map_update_interval` from 2.0→1.0 s if performance OK

Static vs dynamic:
- **Static obstacles:** SLAM map + costmap
- **Dynamic/new obstacles:** Local costmap (immediate), global costmap obstacle layer, SLAM map (delayed)

---

## 31. Costmap Architecture

### 31.1 Global costmap

| Property | Current | Recommended |
|----------|---------|-------------|
| Frame | `husky1_map` | Keep |
| Resolution | 0.1 m | Keep |
| Layers | static + obstacle + inflation | Keep |
| Unknown space | tracked | Keep |

### 31.2 Local costmap

| Property | Current | Recommended |
|----------|---------|-------------|
| Frame | `husky1_odom` | Keep |
| Size | 10×10 m | Keep |
| Resolution | 0.05 m | Keep |
| Layers | obstacle + inflation | Keep (+ optional depth P2) |

### 31.3 Footprint & inflation (P0 tune)

Current footprint: 1.0×1.0 m square.

Recommended based on URDF (`wheel_separation=0.5708`, wheel x_pos ±0.256):

```yaml
footprint: '[[0.55, 0.35], [0.55, -0.35], [-0.55, -0.35], [-0.55, 0.35]]'
```

Inflation: reduce from 2.0→1.0 m for tighter paths in forest; keep ≥0.5 m safety margin.

---

## 32. Global Path Planning

### 32.1 Keep NavFn (P0)

- Plugin: `nav2_navfn_planner/NavfnPlanner`
- Input: global costmap, start pose, goal pose
- Output: `/husky1/plan` (`nav_msgs/Path` in `husky1_map`)
- Frequency: up to 20 Hz on replan events

### 32.2 Why global planner is needed

Computes coarse route through entire map — necessary when local planner cannot see distant obstacles or goal is far away.

### 32.3 Optional Smac 2D (P2)

Replace plugin with `nav2_smac_planner/SmacPlanner2D` for better handling of narrow passages after footprint tuning.

---

## 33. Local Planning / Control

### 33.1 Keep DWB (P0)

- Samples (vx, vtheta) trajectories at 5 Hz
- Tracks global path while avoiding local costmap obstacles
- Outputs to velocity_smoother → cmd_vel

### 33.2 Why local planner is needed

Reacts to **new** obstacles not in global map; provides smooth velocity commands; handles dynamic blockage between replans.

### 33.3 Tuning recommendations (P1)

| Parameter | Current | Recommended |
|-----------|---------|-------------|
| `controller_frequency` | 5.0 | 10.0 |
| `xy_goal_tolerance` | 1.0 | 0.25 |
| `yaw_goal_tolerance` | 1.57 | 0.35 |
| `max_vel_x` | 1.0 | 0.5 (safer demo) |

---

## 34. Dynamic Obstacle Handling

### 34.1 Current capability (PARTIAL)

- Lidar marks obstacles in rolling local costmap at 5 Hz
- DWB avoids high-cost cells
- SLAM map updates slowly — dynamic objects not reliably in static map
- `dynamic_world_demo.py` moves Gazebo models but does not integrate with Nav2

### 34.2 Recommended (P1)

No custom dynamic obstacle tracker required for course demo. Rely on:
1. Local costmap reactive avoidance
2. BT global replan when path invalid
3. Recovery behaviors if stuck

### 34.3 Optional P3

Velocity obstacle layer or prediction — **not required** for initial acceptance criteria.

---

## 35. Real-Time Replanning

### 35.1 Recommended logic (mostly Nav2 built-in)

```text
1. Receive NavigateToPose goal (RViz or mission node)
2. BT: compute path → smooth → follow path
3. Each control cycle (5-10 Hz):
   a. Update local costmap from /husky1/scan
   b. DWB selects feasible cmd_vel avoiding obstacles
4. Parallel (BT conditions):
   a. is_path_valid? → if false: compute new path
   b. is_stuck? → recovery (spin, backup)
   c. goal_reached? → success
5. Global costmap updates at 5 Hz with live scan + SLAM map
6. Repeat until goal or abort
```

### 35.2 Replan triggers (use Nav2 defaults — do not over-implement)

| Trigger | Source | Priority |
|---------|--------|----------|
| Path invalid (obstacle on path) | BT `is_path_valid` | P0 |
| Controller cannot progress | `is_stuck` + recovery | P0 |
| New goal received | BT `goal_updated` | P0 |
| Path cost increase substantially | Not custom — rely on is_path_valid | — |
| Robot deviates from path | DWB PathAlign critic | Built-in |

---

## 36. Safety Architecture

| Mechanism | Current | Recommended |
|-----------|---------|-------------|
| Footprint collision check | 1.0 m box in costmap | Tune to URDF (~1.1×0.7 m) P0 |
| Inflation layer | 2.0 m | 1.0 m P1 |
| DWB BaseObstacle | scale 0.02 | Keep; tune if too timid |
| Velocity limits | 1.0 m/s | 0.5 m/s for demo P1 |
| Acceleration limits | 2.5 m/s² | Keep |
| Emergency stop | None | `nav2_collision_monitor` P1 |
| velocity_timeout | 1.0 s | Keep |

### 36.1 Failure responses

| Condition | Detection | Response | Recovery |
|-----------|-----------|----------|----------|
| No sensor data | scan hz=0, costmap stale | Nav2 pause/fail | Restart sim/bridge |
| No localisation | TF lookup fail | Mission node wait | Ensure SLAM+EKF running |
| No map | /map empty | Planner fail | Drive to explore or wait for SLAM |
| No valid path | planner returns failure | ABORTED action | Relaxed goal or clear costmap |
| Goal unreachable | BT abort after recovery | Stop, report | User moves goal |
| Obstacle blocks route | is_path_valid false | Replan | NavFn new path |
| Dynamic obstacle ahead | local costmap | DWB avoid + replan | Built-in |
| Robot stuck | progress checker 10 s | Recovery behaviors | spin, backup |
| Controller fails | action ABORTED | Log, stop | Resend goal |
| TF unavailable | tf2 exceptions | Node waits | Fix AMCL/SLAM conflict |
| Sensor disappears mid-run | collision_monitor / scan timeout | Stop cmd_vel | Halt mission |

---

## 37. Start / Goal Configuration

### 37.1 Current method

**Start:** Hard-coded in `launch/41068_ignition.launch.py` lines 390-393:
```python
x='0.0', y='0.0', z='0.4', yaw='0.0'  # implicit default in add_robot spawn
```

**Goal:** RViz Nav2 Goal tool OR random in `basic_autonomy_demo.py`.

### 37.2 Recommended method (P0)

Add launch arguments to `41068_ignition.launch.py`:

```bash
ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py \
  nav2:=true slam:=true rviz:=true \
  husky_x:=0.0 husky_y:=0.0 husky_z:=0.4 husky_yaw:=0.0
```

Modify `add_robot()` to accept `LaunchConfiguration` for x, y, z, yaw.

**Goal via mission demo (P0):**

```bash
# Terminal 1: simulation + nav
ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py nav2:=true rviz:=true

# Terminal 2: mission
ros2 run 41068_ignition_bringup basic_autonomy_demo.py --ros-args \
  -p mission_mode:=single_goal \
  -p goal_x:=8.0 -p goal_y:=0.0 -p goal_yaw:=0.0
```

Or unified wrapper script (P1):

```bash
python3 scripts/basic_autonomy_demo.py --start 0 0 0 --goal 8 0 0
```

(Requires extending script to optionally launch sim via subprocess — see Phase 11.)

### 37.3 Coordinate conventions

| Item | Value |
|------|-------|
| Frame | `husky1_map` for goals; spawn in Gazebo world frame (aligned at origin) |
| Units | metres, radians (yaw) |
| Z | 0.4 m for Husky spawn height |

---

## 38. basic_autonomy_demo.py Design

### 38.1 Current behaviour

Random free-space goals; brightness biases distance; uses Nav2 action; 1 Hz state machine.

### 38.2 Target behaviour

Support modes via parameter `mission_mode`:

| Mode | Behaviour |
|------|-----------|
| `random_walk` | Current behaviour (preserve) |
| `single_goal` | Wait for map+TF, send one goal, wait for result, exit |
| `replann_demo` | Send goal, on success or if blocked log replan events |

New parameters:
- `goal_x`, `goal_y`, `goal_yaw` (float)
- `start_x`, `start_y`, `start_yaw` (optional — only if script triggers respawn)
- `mission_mode` (string)
- `wait_for_map_seconds` (float, default 30)

**Important:** Start pose is set at Gazebo spawn (launch args), not by demo node — unless demo calls `set_entity_state` (not recommended P0).

### 38.3 Recommended demo command (after implementation)

```bash
# Full workflow — two terminals (preserves ROS launch architecture)

# Terminal 1
ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py \
  nav2:=true rviz:=true world:=simple_trees \
  husky_x:=0.0 husky_y:=0.0 husky_yaw:=0.0

# Terminal 2 (after map builds ~10-20 s)
ros2 launch 41068_ignition_bringup 41068_autonomy_demo.launch.py robot:=husky1 \
  --ros-args -p mission_mode:=single_goal -p goal_x:=8.0 -p goal_y:=0.0 -p goal_yaw:=0.0
```

For replan demo: use `world:=large_demo` + `41068_dynamic_world_demo.launch.py` in Terminal 3.

---

## 39. Launch Architecture

### 39.1 Files

| File | Role |
|------|------|
| `41068_ignition.launch.py` | Canonical — add spawn args P0 |
| `41068_ignition_husky.launch.py` | Husky wrapper |
| `41068_navigation.launch.py` | SLAM + Nav2 — keep |
| `41068_autonomy_demo.launch.py` | Mission node — extend params P0 |
| **NEW** `41068_mission.launch.py` | Optional: nav2+rviz+mission one-shot P2 |

### 39.2 Recommended default for autonomous demo

Always launch with `nav2:=true` (implies SLAM via navigation launch condition).

Evidence: `41068_navigation.launch.py:29-32` — nav2 OR slam triggers SLAM.

---

## 40. ROS 2 Interface Contract

| Interface | Type | Publisher | Subscriber | Purpose | Status |
|-----------|------|-----------|------------|---------|--------|
| `/clock` | topic | clock bridge | all sim nodes | Sim time | IMPLEMENTED |
| `/husky1/cmd_vel` | topic | velocity_smoother / teleop | gazebo_bridge | Motion | IMPLEMENTED |
| `/husky1/scan` | topic | bridge | SLAM, costmaps | Obstacles | IMPLEMENTED |
| `/husky1/odometry` | topic | bridge | EKF | Raw odom | IMPLEMENTED |
| `/husky1/odom` | topic | EKF | Nav2 | Filtered odom | IMPLEMENTED |
| `/husky1/imu` | topic | bridge | EKF | Yaw rate | IMPLEMENTED |
| `/husky1/map` | topic | SLAM | Nav2, demo, RViz | Occupancy grid | IMPLEMENTED |
| `/husky1/plan` | topic | planner_server | RViz | Global path | IMPLEMENTED |
| `/husky1/local_plan` | topic | controller_server | RViz | Local path | IMPLEMENTED |
| `/husky1/global_costmap/costmap` | topic | Nav2 | RViz | Global costs | IMPLEMENTED |
| `/husky1/local_costmap/costmap` | topic | Nav2 | RViz | Local costs | IMPLEMENTED |
| `/husky1/navigate_to_pose` | action | bt_navigator | demo, RViz | Go to pose | IMPLEMENTED |
| `/husky1/compute_path_to_pose` | action | planner_server | BT | Plan only | IMPLEMENTED |
| `/husky1/follow_path` | action | controller_server | BT | Track path | IMPLEMENTED |
| `/husky1/tf` | topic | EKF, SLAM, rsp | all | Dynamic TF | IMPLEMENTED |
| `/husky1/tf_static` | topic | rsp | all | Static TF | IMPLEMENTED |
| `/husky1/camera/image` | topic | bridge | demo | Perception demo | IMPLEMENTED |
| `/husky1/camera/depth/points` | topic | bridge | — (future costmap) | Close obstacles | IMPLEMENTED unused |

All interfaces namespaced — **never use global `/cmd_vel` or `/map`**.

---

## 41. Path Planning Data Model

```text
LaserScan (/husky1/scan)
  → ObstacleLayer → Costmap2D (0-254 costs, 255 lethal)
  → StaticLayer ← OccupancyGrid (/husky1/map): -1 unknown, 0 free, 100 occupied

TF: husky1_map → husky1_base_link (start pose)
Goal: PoseStamped in husky1_map

Planner input: costmap + start + goal
Planner output: nav_msgs/Path (poses in husky1_map)

Controller input: Path + odom + local costmap
Controller output: geometry_msgs/Twist

Actuation: Twist.linear.x, Twist.angular.z → Gazebo VelocityControl
```

---

## 42. Replanning Data Model

```text
New LaserScan
  → obstacle_layer.markObstacle / raytrace clear
  → costmap updated (5 Hz)
  → DWB trajectory evaluation (immediate avoidance)

Parallel BT branch:
  is_path_valid(path, costmap)?
    NO → ComputePathToPose → new Path → FollowPath
    YES → continue FollowPath

SLAM (parallel, slower):
  scan + odom → map update (2 s) → static layer refresh
```

Steps occur in **Nav2 nodes** (external) — no custom replan node required for P0.

---

## 43. Performance Considerations

| Component | Current | Recommended | Notes |
|-----------|---------|-------------|-------|
| Lidar rate | 3 Hz | 10 Hz P1 | CPU dependent |
| Camera rate | 3 Hz | Keep 3 Hz | Not critical for nav |
| IMU | 100 Hz | Keep | EKF uses subset |
| EKF | 30 Hz | Keep | |
| SLAM map update | 2 s | 1 s P1 | |
| Costmap update | 5 Hz | 5-10 Hz | Match controller |
| Controller | 5 Hz | 10 Hz P1 | |
| Planner | 20 Hz max | On-demand | Event driven |
| Velocity smoother | 20 Hz | Keep | |

Avoid running Husky + Parrot + dual RViz on weak hardware.

---

## 44. File-by-File Implementation Plan

| File | Current Purpose | Required Change | Reason | Priority |
|------|-----------------|-----------------|--------|----------|
| `launch/41068_ignition.launch.py` | Sim bringup | Add `husky_x/y/z/yaw` launch args | Configurable start | P0 |
| `config/nav2_params_husky1.yaml` | Nav2 tuning | `amcl.tf_broadcast: false`; footprint; tolerances | TF fix + tuning | P0 |
| `config/nav2_params_parrot1.yaml` | Parrot Nav2 | Same AMCL fix | Consistency | P1 |
| `scripts/basic_autonomy_demo.py` | Random walk | Add `mission_mode`, goal params | Start→goal demo | P0 |
| `launch/41068_autonomy_demo.launch.py` | Launch demo | Declare new parameters | Pass goal config | P0 |
| `urdf_husky/husky.gazebo.xacro` | Sensors | Increase lidar `update_rate` | Faster replan | P1 |
| `package.xml` | Dependencies | Add exec_depend nav2_bringup, slam_toolbox | Correct deps | P1 |
| **NEW** `config/collision_monitor_husky1.yaml` | — | Collision monitor zones | Safety stop | P1 |
| **NEW** `launch/41068_collision_monitor.launch.py` | — | Optional safety node | Emergency stop | P1 |
| `config/41068_husky1.rviz` | RViz | Verify displays; add costmap if missing | Demo clarity | P1 |
| `README.md` | User docs | Document start/goal args, mission mode | Usability | P1 |
| `config/nav2_params.yaml` | Legacy | Add comment "UNUSED" or remove | Avoid confusion | P2 |
| **NEW** `test/test_goal_sampling.py` | — | Unit test `_is_free_with_margin` | Regression | P2 |
| **NEW** `41068_mission.launch.py` | — | All-in-one demo launch | Convenience | P2 |

---

## 45. Package Dependency Plan

| Package | Distro | Purpose | Existing/New | Install |
|---------|--------|---------|--------------|---------|
| `ros-humble-ros-ign` | Humble | Gazebo bridge | Existing | apt |
| `ros-humble-robot-localization` | Humble | EKF | Existing | apt |
| `ros-humble-navigation2` | Humble | Nav2 stack | Existing | apt |
| `ros-humble-nav2-bringup` | Humble | Nav2 launch | Existing | apt |
| `ros-humble-slam-toolbox` | Humble | SLAM | Existing | apt |
| `ros-humble-nav2-collision-monitor` | Humble | Safety | **New P1** | apt |
| `ros-humble-teleop-twist-keyboard` | Humble | Manual drive | Existing | apt |
| `python3-numpy` | system | Demo script | Existing | apt |
| `41068_ignition_bringup` | workspace | This package | Existing | colcon build |

Add to `package.xml` exec_depend: `nav2_bringup`, `slam_toolbox`, `nav2_collision_monitor` (P1).

---

## 46. Package Installation Plan

### Already installed (expected per README)

- ROS 2 Humble, Ignition Fortress
- ros_ign, robot_localization, navigation2, nav2_bringup, slam_toolbox

### Required (implementation agent)

```bash
sudo apt install ros-humble-nav2-collision-monitor
```

### Optional

```bash
# Only if implementing Smac planner P2 — already in navigation2 metapackage
# No extra install typically needed on Humble
```

### Remove/replace

- Do **not** remove Nav2 or SLAM
- Disable AMCL TF (config change only)
- Mark legacy YAML as unused (do not delete without team approval)

---

## 47. Implementation Phases

### PHASE 0 — Baseline verification

**Task 0.1:** Build and source workspace  
**Purpose:** Confirm clean baseline  
**Command:** `colcon build --symlink-install && source install/setup.bash`  
**Test:** `ros2 pkg list | grep 41068`

**Task 0.2:** Launch sim without nav  
**Command:** `ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py`  
**Expected:** Gazebo + Husky, `/husky1/scan` publishing  
**Test:** `ros2 topic hz /husky1/scan`

**Task 0.3:** Teleop motion  
**Command:** `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/husky1/cmd_vel`  
**Expected:** Robot moves  
**Failure modes:** No bridge, wrong topic

**Task 0.4:** Launch with Nav2  
**Command:** `... nav2:=true rviz:=true`  
**Expected:** Map builds, Nav2 action server available  
**Test:** `ros2 action list | grep navigate_to_pose`

---

### PHASE 1 — Package/dependency cleanup

**Task 1.1:** Fix AMCL/SLAM TF conflict  
**Purpose:** Stable map→odom  
**Package:** `41068_ignition_bringup`  
**Files to modify:** `config/nav2_params_husky1.yaml`, `config/nav2_params_parrot1.yaml`  
**Change:** `amcl.ros__parameters.tf_broadcast: false`  
**Test:** `ros2 run tf2_ros tf2_echo husky1_map husky1_odom` — single consistent transform  
**Failure modes:** Multiple publishers warning in tf2

**Task 1.2:** Update package.xml exec_depend  
**Files to modify:** `package.xml`  
**Dependencies:** nav2_bringup, slam_toolbox  

---

### PHASE 2 — Robot movement foundation

**Task 2.1:** Verify cmd_vel chain documented  
**No code change** — regression teleop test each phase  
**Expected behaviour:** Twist on `/husky1/cmd_vel` moves robot in Gazebo  

---

### PHASE 3 — Sensor pipeline

**Task 3.1:** Increase lidar rate (optional P1)  
**Files to modify:** `urdf_husky/husky.gazebo.xacro` — `<update_rate>10</update_rate>`  
**Expected:** `ros2 topic hz /husky1/scan` ≈ 10 Hz  
**Failure modes:** CPU overload, rendering failures on WSL

---

### PHASE 4 — TF/localisation

**Task 4.1:** Verify TF tree after AMCL fix  
**Test:** `ros2 run tf2_tools view_frames`  
**Expected:** map→odom (SLAM only), odom→base_link (EKF)  

---

### PHASE 5 — Mapping

**Task 5.1:** Verify SLAM with nav2 launch  
**Expected:** `/husky1/map` grows when robot moves  
**Test:** Drive teleop, watch RViz map  

---

### PHASE 6 — Costmaps/perception

**Task 6.1:** Tune footprint and inflation  
**Files to modify:** `config/nav2_params_husky1.yaml` (local + global costmap sections)  
**Parameters:** footprint polygon, inflation_radius: 1.0  
**Test:** RViz global costmap shows reasonable robot size  

**Task 6.2 (P2):** Add depth to local costmap  
**Files to modify:** `nav2_params_husky1.yaml` — add observation source for `camera/depth/points`  

---

### PHASE 7 — Global planning

**Task 7.1:** Verify NavFn planning  
**Test:** RViz goal → `/husky1/plan` appears  
**Expected:** Path avoids known obstacles  

---

### PHASE 8 — Local planning/control

**Task 8.1:** Tune DWB + goal tolerances  
**Files to modify:** `config/nav2_params_husky1.yaml`  
**Parameters:** controller_frequency, xy/yaw tolerances, max_vel_x  
**Test:** Robot reaches goal within 0.25 m  

---

### PHASE 9 — Dynamic replanning

**Task 9.1:** Verify local avoidance  
**Setup:** `world:=simple_trees`, nav2 active, send goal past tree  
**Expected:** Robot routes around tree  

**Task 9.2:** Dynamic obstacle test  
**Setup:** `large_demo` + `dynamic_world_demo.launch.py`  
**Expected:** Robot slows/avoids animal if in lidar FOV  

**Task 9.3 (P1):** Add collision monitor  
**Files to create:** `config/collision_monitor_husky1.yaml`, launch include  
**Expected:** Stop when obstacle within stop zone  

---

### PHASE 10 — Start/goal configuration

**Task 10.1:** Add spawn launch arguments  
**Files to modify:** `launch/41068_ignition.launch.py`  
**Parameters:** `husky_x`, `husky_y`, `husky_z`, `husky_yaw`  
**Expected:** `husky_x:=2.0` spawns at x=2  

**Task 10.2:** Extend autonomy demo with mission_mode  
**Files to modify:** `scripts/basic_autonomy_demo.py`, `41068_autonomy_demo.launch.py`  
**Parameters:** goal_x, goal_y, goal_yaw, mission_mode  
**ROS interfaces:** NavigateToPose action  
**Expected:** Single goal sent and awaited  

---

### PHASE 11 — basic_autonomy_demo.py / mission wrapper

**Task 11.1:** Implement `single_goal` mode in demo  
**Purpose:** One-shot start→goal for grading/demo  
**Expected behaviour:** Logs success/failure, exits or waits  

**Task 11.2 (P2):** Optional unified Python entry  
**NEW FILE:** `scripts/run_autonomy_demo.py` — subprocess launch sim+wait+mission  
**Note:** Two-terminal approach is acceptable and more idiomatic ROS  

---

### PHASE 12 — RViz integration

**Task 12.1:** Confirm RViz config shows map, scan, plan, costmap  
**Files:** `config/41068_husky1.rviz`  
**Test:** Visual inspection during nav  

---

### PHASE 13 — Testing

See Section 48 Test Plan. Add `test/test_goal_sampling.py` for demo helpers (P2).

---

### PHASE 14 — Demonstration

Document workflow in README. Prepare 30s/2min presentation scripts (Section 51).

---

## 48. Testing Strategy

| Test | Setup | Command | Expected | Success | Failure |
|------|-------|---------|----------|---------|---------|
| T1 Basic movement | sim only | teleop cmd_vel | Robot moves | Motion in Gazebo | No cmd_vel |
| T2 Empty env | simple_trees, nav2 | Goal 5 m ahead | Straight-ish path | SUCCESS | No plan |
| T3 Static obstacle | tree in path | Goal beyond tree | Path around | No collision | Hit tree |
| T4 Multiple obstacles | large_demo | Goal across forest | Valid path | SUCCESS or safe abort | Stuck |
| T5 Dynamic obstacle | + dynamic demo | Goal while animal moves | Avoid/replan | No collision | Drive through |
| T6 Path blockage | Place obstacle mid-route | Continue mission | Replan or recovery | New plan visible | Infinite spin |
| T7 Narrow passage | between trees | Through gap | Pass or safe fail | No wedging | Stuck |
| T8 Impossible goal | Goal inside obstacle | Send goal | ABORTED | No crash | Hang |
| T9 Sensor failure | Stop bridge | Navigating | Stop safely | No runaway | Crash |
| T10 Different start/goal | husky_x/y, goal params | Mission demo | Reaches goal | SUCCESS | Wrong frame |

---

## 49. Failure Handling

(See Section 36.1 table — implement detection in mission node via action result codes: SUCCEEDED, ABORTED, CANCELED.)

Mission node should:
- Wait for map non-empty before sending goal
- Log Nav2 feedback distance_remaining
- On ABORTED: optionally retry once with cleared local costmap (call `/husky1/local_costmap/clear_entirely_local_costmap` service if available)

---

## 50. Acceptance Criteria

- [ ] Simulation launches with `nav2:=true rviz:=true`
- [ ] Robot spawns at configurable x, y, yaw
- [ ] Robot moves via Nav2 to configured goal
- [ ] Sensors publish (`/husky1/scan`, `/husky1/odom`)
- [ ] TF valid (no AMCL/SLAM conflict)
- [ ] Map builds during motion
- [ ] Global path generated and visible in RViz
- [ ] Path avoids known static obstacles
- [ ] Live scan updates costmap during motion
- [ ] Local planner avoids sudden obstacles
- [ ] Global replan occurs when path blocked (visible plan change)
- [ ] Robot reaches goal within tuned tolerance
- [ ] RViz shows map, scan, plan, costmap
- [ ] Mission demo sends single goal (not random only)
- [ ] Dynamic world demo + navigation coexist without crash

---

## 51. How to Run the Final System

### Quick Start

Workspace in these commands is `~/RS1-Gr25`. Adjust if yours is different.

```bash
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/RS1-Gr25/install/setup.bash
cd ~/RS1-Gr25/src/41068_ignition_bringup_v1/41068_ignition_bringup
```

1. **One-command autonomous demo** (starts Gazebo headless, Nav2, sends a goal, stops on success or timeout):

```bash
python3 scripts/basic_autonomy_demo.py
```

2. **Watch it** — add RViz and/or the Gazebo GUI:

```bash
python3 scripts/basic_autonomy_demo.py --rviz --gui --start 0 0 0 --goal 0 -5 0
```

3. **Dynamic replanning demo** (a real Gazebo wall is dropped on the path mid-run):

```bash
python3 scripts/basic_autonomy_demo.py --replan
```

4. **Manual launch + RViz goal** — click "Nav2 Goal" in frame `husky1_map`:

```bash
ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py \
  nav2:=true rviz:=true gui:=true world:=simple_trees \
  husky_x:=0.0 husky_y:=0.0 husky_yaw:=0.0
```

5. **Stop** — Ctrl+C. The Python demo also tears down Gazebo/Nav2 when the mission ends. If a previous run crashed:

```bash
pkill -f 'ign gazebo' ; pkill -f 'ros2 launch 41068'
```

Default world is `simple_trees` (15 × 15 m, two trees). `large_demo` works for visualisation but Nav2 lifecycle can time out on WSL2 software GL.

### Tests

```bash
python3 test/run_nav_tests.py
python3 test/run_nav_tests.py --only fast movement tf_slam navigation replan
```

---

## 52. How to Change Start and Goal

Goals are in `husky1_map`. SLAM initialises that frame on the robot, so a goal of `(0, -5)` means "5 m south of where the Husky spawned", not a fixed Gazebo-world coordinate. `simple_trees` grass is only 15 × 15 m; keep spawn and goal combinations on that plane.

### START X / Y / YAW

| Parameter | Meaning | Default | Where |
|-----------|---------|---------|--------|
| `husky_x` | Spawn X (m) | 0.0 | launch argument |
| `husky_y` | Spawn Y (m) | 0.0 | launch argument |
| `husky_z` | Spawn Z (m) | 0.4 | launch argument (leave default) |
| `husky_yaw` | Spawn yaw (rad) | 0.0 | launch argument |

```bash
ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py \
  nav2:=true rviz:=true \
  husky_x:=1.0 husky_y:=0.5 husky_yaw:=0.0

python3 scripts/basic_autonomy_demo.py --start 1.0 0.5 0.0 --goal 0 -5 0
```

### GOAL X / Y / YAW

**Python demo**

```bash
python3 scripts/basic_autonomy_demo.py --goal 0 -5 0
# yaw in radians, e.g. 1.57 ≈ 90°
python3 scripts/basic_autonomy_demo.py --goal -4.5 1.5 1.57
```

**Already-running simulation** (second terminal):

```bash
ros2 launch 41068_ignition_bringup 41068_autonomy_demo.launch.py \
  robot:=husky1 mission_mode:=single_goal \
  goal_x:=0.0 goal_y:=-5.0 goal_yaw:=0.0
```

**RViz:** "Nav2 Goal" / "2D Goal Pose" in frame `husky1_map`.

**CLI:**

```bash
ros2 action send_goal /husky1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'husky1_map'}, pose: {position: {x: 0.0, y: -5.0}, orientation: {w: 1.0}}}}"
```

---

## 53. What Code Does What?

| Function | Package | File | Node/Class | Key function | Purpose |
|----------|---------|------|------------|--------------|---------|
| Robot movement | 41068_ignition_bringup | `urdf_husky/husky.gazebo.xacro` | Gazebo DiffDrive | `cmd_vel` → wheel joints | Drive the Husky. VelocityControl remains selectable (`drive_plugin:=velocity_control`) but cannot yaw. |
| LiDAR | 41068_ignition_bringup | `husky.gazebo.xacro`, `config/gazebo_bridge_husky1.yaml` | `gpu_lidar` | `/husky1/scan` @ 10 Hz | Mapping and obstacle detection |
| IMU | same | same | imu sensor | `/husky1/imu` | EKF orientation |
| Odometry | robot_localization + Gazebo | `config/robot_localization_husky1.yaml` | `ekf_node` | `/husky1/odom` | Fused odom |
| TF | slam_toolbox, robot_localization, robot_state_publisher | slam / ekf / URDF | three publishers | `map→odom→base_link→sensors` | Single-owner tree; AMCL is not launched |
| Mapping | slam_toolbox | `config/slam_params_husky1.yaml` | `async_slam_toolbox_node` | `/husky1/map` | Live occupancy grid |
| Localisation | slam_toolbox + EKF | same | SLAM owns `map→odom` | pose in `husky1_map` | AMCL `tf_broadcast: false` as a safety net |
| Costmap | Nav2 | `config/nav2_params_husky1.yaml` | obstacle + inflation layers | global 40×40 m rolling, local 10×10 m | Sensor-driven occupancy for planning |
| Global planning | Nav2 | same | `NavfnPlanner` (A*) | `/husky1/plan` | Route around lethal cells; `allow_unknown: true` |
| Local control | Nav2 | same | Regulated Pure Pursuit | `/husky1/cmd_vel_nav` | Follow the global path |
| Replanning | Nav2 BT | same | `is_path_valid` @ 1 Hz | new `/husky1/plan` | Sensor-driven reroute when the path is blocked |
| Goal handling | nav2_bt_navigator | `/husky1/navigate_to_pose` | `bt_navigator` | accept pose goals | RViz, demo, and tests share this action |
| Autonomous demo | 41068_ignition_bringup | `scripts/basic_autonomy_demo.py`, `rs1_nav/mission.py` | `MissionRunner` | bounded start→goal | Launch, wait, send goal, optional barrier, report |
| Obstacle injection | 41068_ignition_bringup | `rs1_nav/gazebo_world.py` | `PathBlocker` | Ignition `create` service | Real Gazebo box on the current path |
| Simulation supervision | 41068_ignition_bringup | `rs1_nav/sim.py` | `SimSupervisor` | timeouts + process sweep | No indefinite waits, no orphan Gazebo |

### Presentation code references

**Movement** — `urdf_husky/husky.gazebo.xacro` DiffDrive: *"Turns Nav2 speed commands into wheel motion in Gazebo."*

**LiDAR** — `/husky1/scan`: *"The laser that builds the map and sees new obstacles."*

**Mapping** — `config/slam_params_husky1.yaml`: *"Builds the occupancy grid from lidar while the robot moves."*

**Path planning** — NavFn in `config/nav2_params_husky1.yaml`: *"Searches the costmap for a route from here to the goal."*

**Local control** — Regulated Pure Pursuit: *"Follows that route and slows near obstacles."*

**Demo** — `scripts/basic_autonomy_demo.py`: *"One command: start the sim, send a goal, show whether the robot arrived."*

---

## 54. Presentation-Ready Explanation

### What the sensors do
The Husky's lidar is a spinning laser that measures distance in every direction, 10 times a second. That is how the robot "sees" trees and anything that appears in front of it. An IMU senses turning; wheel odometry senses driving. The camera exists on the model but is off by default because it can freeze the simulator on some machines, and navigation does not use it.

### What mapping does
SLAM Toolbox sketches a bird's-eye grid as the robot moves: free space, occupied cells, and unknown cells. That sketch is the map you see in RViz.

### What path planning does
When you give a goal, Nav2's global planner (NavFn) searches that grid for a safe route around lethal cells, the same idea as a GPS route around a blocked road.

### What dynamic replanning does
The robot keeps looking. If a wall appears on the route, the lidar shortens, the costmap paints those cells occupied, Nav2 drops the old path and draws a new one around the wall, and the robot follows the new path. Nobody types a second goal.

### What Nav2 does
Nav2 is the standard ROS 2 navigation stack: costmaps, global planner, path follower, recoveries, and the "go to this pose" action. This project configures it; it does not replace it with a custom planner.

### What Gazebo does
Gazebo is the virtual forest: physics, collisions, and fake sensors. The Husky in the window is the same robot Nav2 is driving.

### What RViz does
RViz is the dashboard: map, laser rays, planned path, costmaps, and a button to click a goal.

### What makes the system autonomous
After you set a start and a goal, the loop runs by itself: sense → map → plan → drive → sense again → replan if needed → arrive.

---

## 55. 30-Second Project Explanation

"We simulate a Husky in Gazebo. Its lidar builds a live map. You pick a start and a goal; Nav2 plans a path and drives there. If we drop a wall on that path, the laser sees it, the map updates, and the robot goes around — no one is joysticking it."

---

## 56. 2-Minute Project Explanation

"This is the university ROS 2 / Ignition Gazebo Husky package, extended into a working autonomous navigator.

The robot is driven by Gazebo's DiffDrive plugin from Nav2 `cmd_vel` commands. A 10 Hz lidar, IMU, and fused odometry feed SLAM Toolbox, which owns the map→odom transform, and robot_localization, which owns odom→base_link. There is no AMCL in the launch, so those two transforms do not fight.

Nav2 plans on a rolling global costmap with NavFn (A*) and follows the path with Regulated Pure Pursuit. Costmaps are filled from live lidar, not from a pre-drawn map. When a real Gazebo obstacle is inserted on the current path, the scan shortens, occupied cells appear, the behaviour tree invalidates the path, and a new route is published. Tests require that chain: they do not send a second goal or teleport the robot.

You change the start with `husky_x`, `husky_y`, `husky_yaw` (or `--start` on the demo). You change the goal with `--goal`, RViz, or the Nav2 action. One command, `python3 scripts/basic_autonomy_demo.py`, runs the whole demonstration with timeouts so the simulator cannot hang forever."

---

## 57. Technical Architecture Diagram

```text
Ignition Gazebo Fortress
  ├─ DiffDrive ← /model/husky1/cmd_vel
  ├─ gpu_lidar (10 Hz) → /model/husky1/scan
  ├─ OdometryPublisher → /model/husky1/odometry
  └─ IMU  (RGB-D camera off by default)
        ↓ ros_ign_bridge (/husky1/*)
robot_state_publisher + robot_localization (EKF)
        husky1_odom → husky1_base_link
        ↓
SLAM Toolbox → /husky1/map + husky1_map → husky1_odom
        ↓
Nav2
  ├─ global_costmap (rolling 40 m, scan + SLAM map)
  ├─ local_costmap (rolling 10 m, scan)
  ├─ planner_server (NavFn A*, allow_unknown)
  ├─ controller_server (Regulated Pure Pursuit)
  ├─ velocity_smoother → /husky1/cmd_vel
  └─ bt_navigator ← NavigateToPose  (is_path_valid replan)
        ↓
MissionRunner / RViz / tests
```

---

## 58. Implementation Priority

### P0 — Required for basic autonomous navigation
- Enable and verify Nav2 launch path
- Fix AMCL `tf_broadcast: false`
- Add spawn launch args (husky_x/y/yaw)
- Extend `basic_autonomy_demo.py` with `single_goal` mode
- Tune goal tolerances
- Document run workflow

### P1 — Required for robust navigation
- Tune footprint and inflation
- Increase lidar rate (if CPU OK)
- Add collision monitor
- Increase controller frequency
- package.xml deps

### P2 — Significant improvement
- Depth camera in local costmap
- Smac planner plugin
- Unit tests for demo helpers
- Unified mission launch file
- Legacy config cleanup

### P3 — Optional advanced
- Custom behavior executive / exploration
- Saved map localization mode
- MPPI controller
- ros2_control for real Husky migration

---

## 59. Trade-offs

### Nav2 vs custom stack

| | Nav2 | Custom |
|---|------|--------|
| **Advantages** | Already integrated; industry standard; replanning BT built-in | Full control; course algorithm showcase |
| **Disadvantages** | Less custom algorithm visibility | High effort; duplicates tested software |
| **Recommended** | **Nav2** — extend demo layer only |

### SLAM Toolbox vs Cartographer

| | SLAM Toolbox | Cartographer |
|---|--------------|--------------|
| **Advantages** | Already wired; async mapping | Strong loop closure |
| **Disadvantages** | 2D only | Re-integration effort |
| **Recommended** | **Keep SLAM Toolbox** |

### DWB vs Regulated Pure Pursuit

| | DWB | RPP |
|---|-----|-----|
| **Advantages** | Already configured; multi-critic avoidance | Simpler; smoother paths |
| **Disadvantages** | Tuning complexity | Less reactive in clutter |
| **Recommended** | **Keep DWB P0**; try RPP P2 if tuning painful |

### Online SLAM vs static map + AMCL

| | Online SLAM | Static + AMCL |
|---|-------------|---------------|
| **Advantages** | No map file needed; explores unknown | Repeatable runs |
| **Disadvantages** | AMCL conflicts; map drift early | Requires map save step |
| **Recommended** | **Online SLAM** for course demo |

---

## 60. Codebase Preservation

| Old | Why keep/replace | New | Migration |
|-----|------------------|-----|-----------|
| VelocityControl | Works in sim | Keep | None |
| ros_ign_bridge pattern | Proven | Keep | Add sensors same way |
| SLAM + Nav2 launch | Complete stack | Keep | Fix AMCL only |
| basic_autonomy_demo.py | Good template | Extend | Add modes, don't delete random_walk |
| Random walk demo | Teaching value | Preserve as `mission_mode:=random_walk` | Default parameter |

**Do not break:** namespacing, TF remaps in navigation launch, world-level Sensors plugin, `gz_model_name` matching spawn name.

---

## 61. Custom vs Nav2 Functionality

| Custom in repo | Nav2 equivalent | Recommendation |
|----------------|-----------------|----------------|
| Random goal sampling (`basic_autonomy_demo.py`) | Waypoint follower / external BT | **Keep custom** as decision layer |
| Brightness heuristic | Perception pipeline | **Replace** with real perception later |
| None — no custom planner | NavFn + DWB | **Use Nav2** |
| None — no custom controller | velocity_smoother + DWB | **Use Nav2** |
| dynamic_world_demo | N/A (Gazebo only) | **Keep** for replan testing |

---

## 62. Final Recommendation

**Implement autonomous start→goal navigation by fixing and extending the existing Nav2 + SLAM + Gazebo stack**, not by replacing it.

Minimum viable path for the implementation agent:

1. Set `amcl.tf_broadcast: false`
2. Add `husky_x/y/yaw` launch arguments
3. Add `mission_mode:=single_goal` with `goal_x/y/yaw` to autonomy demo
4. Tune footprint, inflation, goal tolerances
5. Verify replan with `dynamic_world_demo` + moving obstacle
6. Document commands in README

The repository already implements ~80% of the target architecture. The remaining work is **integration hardening, configuration, mission orchestration, and demonstration workflow** — not greenfield navigation development.

---

## 63. Implementation Checklist

### Simulation
- [x] Workspace builds (`colcon build --packages-select 41068_ignition_bringup`)
- [x] Gazebo launches (`simple_trees`, headless)
- [x] Husky spawns; DiffDrive yaw works

### Sensors
- [x] `/husky1/scan` ~10 Hz; `/husky1/odom` ~30 Hz; `/clock` 100 Hz
- [x] Camera off by default (WSL stall)

### TF / Localisation
- [x] map→odom from SLAM only
- [x] odom→base_link from EKF
- [x] AMCL not launched; `tf_broadcast: false`

### Mapping
- [x] `/husky1/map` grows during motion; trees appear occupied

### Costmaps
- [x] Footprint matches Husky geometry
- [x] Inflation 0.75 m; rolling global 40×40 m

### Global Planning
- [x] NavFn produces valid `/husky1/plan` to off-map goals

### Local Planning
- [x] Regulated Pure Pursuit follows path (DWB failed dynamic detour)
- [x] Goal tolerances 0.25 m / 0.35 rad

### Movement
- [x] cmd_vel → smoother → DiffDrive verified (movement_test.py)

### Dynamic Replanning
- [x] Live lidar → costmap 254 → diverging plan → goal
- [x] collision_monitor not added (WSL load; RPP collision detection used)

### Start / Goal
- [x] Launch args `husky_x/y/yaw`
- [x] Demo `--start` / `--goal`; RViz Nav2 Goal

### Demo
- [x] `python3 scripts/basic_autonomy_demo.py` one-command
- [x] `--replan` inserts a real Gazebo barrier

### RViz
- [x] Map, scan, plan, costmap (when `--rviz`)

### Testing
- [x] fast, movement, obstacle injection, TF/SLAM, 3-goal nav (two starts), replan
- [ ] large_demo Nav2 lifecycle under software GL (known limitation)

### Presentation
- [x] 30s and 2min explanations updated to the implemented stack

---

## 64. Quality Audit (Self-Check)

### Repository understanding
- [x] Entire codebase inspected (launch, config, scripts, URDF, worlds)
- [x] All relevant packages identified
- [x] Launch files inspected
- [x] Robot model and Gazebo plugins inspected

### Sensor understanding
- [x] Every navigation sensor identified with topics/frames
- [x] Consumers documented

### Movement understanding
- [x] cmd_vel chain traced to DiffDrive (VelocityControl retained as option; cannot yaw)

### Navigation understanding
- [x] Localisation: EKF + SLAM (AMCL not launched)
- [x] Mapping: SLAM Toolbox
- [x] Planning: Nav2 NavFn + Regulated Pure Pursuit
- [x] Replanning: BT `is_path_valid` + live lidar, verified by `replan_test.py`

### Target architecture
- [x] Single recommended architecture (Nav2 extension)
- [x] File changes documented
- [x] Testing documented

### User usability
- [x] Start/goal configuration specified
- [x] Run instructions
- [x] Presentation section included

---

## 65. Implementation Log

### Change 01 — Configurable spawn
**Date:** 2026-08-28  
**Change:** `husky_x/y/z/yaw` launch arguments on `41068_ignition.launch.py` and the husky wrapper.  
**Reason:** Start pose must be user-facing, not a hard-coded spawn.  
**Files:** `launch/41068_ignition.launch.py`, `launch/41068_ignition_husky.launch.py`  
**Test:** `navigation_test.py --start 0 0 0` and `--start 1.0 0.5 0.0` (3/3 goals each).  
**Status:** PASS

### Change 02 — DiffDrive default, not VelocityControl
**Date:** 2026-08-28  
**Original plan:** Keep VelocityControl if it was reliable.  
**Problem:** VelocityControl achieved only ~0.045 of a 0.5 rad/s yaw command, so Nav2 could not rotate in place.  
**Solution:** Gazebo DiffDrive with `effective_wheel_separation:=0.94` (skid-steer slip). VelocityControl remains `drive_plugin:=velocity_control`.  
**Files:** `urdf_husky/husky.gazebo.xacro`, launch args  
**Test:** `movement_test.py` — 0.4 m/s → 1.60 m; yaw tracks command.  
**Status:** PASS

### Change 03 — Camera off, lidar 10 Hz
**Date:** 2026-08-28  
**Problem:** RGB-D camera stalled Ignition on WSL2 software GL (no `/clock`).  
**Solution:** `enable_camera:=false` by default; lidar 10 Hz. Nav2/SLAM do not use the camera.  
**Files:** `launch/41068_ignition.launch.py`, `urdf_husky/husky.gazebo.xacro`  
**Test:** `fast_test.py` — scan 10.01 Hz, clock 100 Hz.  
**Status:** PASS

### Change 04 — SLAM owns map→odom
**Date:** 2026-08-28  
**Original plan:** Set AMCL `tf_broadcast: false` to stop a fight with SLAM.  
**Finding:** `navigation_launch.py` never starts AMCL. Setting `tf_broadcast: false` is a safety net only.  
**Files:** `config/nav2_params_husky1.yaml`, `config/nav2_params_parrot1.yaml`  
**Test:** `tf_slam_test.py` — publishers are slam_toolbox, robot_localization, robot_state_publisher; no amcl.  
**Status:** PASS

### Change 05 — Rolling global costmap 40×40 m
**Date:** 2026-08-28  
**Original plan:** Static global costmap sized by the SLAM map; `allow_unknown: true`.  
**Problem:** SLAM only grows the grid to lidar *hits*. Open-space goals (e.g. `(0, -5)`) were off the map; NavFn aborted with "goal sent to the planner is off the global costmap". `allow_unknown` cannot invent cells.  
**Solution:** `rolling_window: true`, width/height 40 m, resolution 0.1 m. Lidar obstacle layer fills what the robot sees.  
**Files:** `config/nav2_params_husky1.yaml`  
**Test:** `navigation_test.py` 3/3 goals after the change (was ABORTED at 16 s before).  
**Status:** PASS

### Change 06 — Controller DWB → Regulated Pure Pursuit
**Date:** 2026-08-28  
**Original plan:** DWB local planner.  
**Problem:** With a real wall on the path, DWB sat in the inflation and the progress checker reported "Failed to make progress". Lidar and a diverging NavFn plan were already present.  
**Solution:** `nav2_regulated_pure_pursuit_controller` follows the global plan; costmap used to slow/stop.  
**Files:** `config/nav2_params_husky1.yaml`  
**Test:** `replan_test.py` — wall inserted, lidar  inf→1.43 m, costmap 254, 1 replan (1.72 m divergence), 1.60 m clearance, goal reached 32.6 s. Then `navigation_test.py` 3/3 still passed.  
**Status:** PASS

### Change 07 — Nav2 start delay 15 s
**Date:** 2026-08-28  
**Problem:** On `large_demo`, lifecycle `change_state` on `controller_server` timed out; `navigate_to_pose` never appeared.  
**Solution:** `nav_start_delay` default 15 s (was 7). Tests use `simple_trees`.  
**Files:** `launch/41068_ignition.launch.py`, husky wrapper  
**Test:** `fast_test.py` / `tf_slam_test.py` Nav2 action up in <10 s after the delay on `simple_trees`.  
**Status:** PASS on `simple_trees`. `large_demo` Nav2 still heavy on software GL.

### Change 08 — Demo, tests, supervision
**Date:** 2026-08-28  
**Change:** Rewrote `scripts/basic_autonomy_demo.py` as a bounded start→goal (and `--replan`) runner. Shared `rs1_nav/` (`SimSupervisor`, `MissionRunner`, `NavObserver`, `PathBlocker`). Tests never wait forever; processes are swept on exit.  
**Files:** `scripts/basic_autonomy_demo.py`, `rs1_nav/*`, `test/*.py`, `package.xml` exec_depend nav2/slam  
**Test:** geometry unit; replan with occupied-cost 254 (not unknown 255).  
**Status:** PASS

### Change 09 — collision_monitor skipped
**Date:** 2026-08-28  
**Original plan:** Evaluate `nav2_collision_monitor`.  
**Decision:** Not launched. Extra node on WSL software GL; RPP already has `use_collision_detection`.  
**Status:** Won't add unless a GPU machine shows a safety gap.

---

*End of implementation record. Repository is source of truth.*
