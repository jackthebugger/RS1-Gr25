# MASTER CONTEXT

> **Purpose:** Consolidated knowledge base for future AI-assisted work on **41068 Robotics Studio 1** (UTS). Synthesised from four seminar extraction documents covering simulation, perception/mapping, decision making, and path planning.
>
> **Status:** REFERENCE — course seminar material; not a live project status report.
>
> **Last synthesised:** 2026-08-28

---

## 0. Context Usage Instructions

### How to use this document

- Treat this file as the **primary context layer** for 41068 autonomy, simulation, and robotics questions.
- Original per-seminar files remain source material; this document deduplicates and reorganises them.
- Information is grouped by **topic and system layer**, not by seminar slide order.
- Status labels (`CURRENT`, `CONFIRMED`, `REFERENCE`, etc.) appear only where they aid reasoning.

### What this document covers

| Domain | Coverage |
|---|---|
| Course & project requirements | CONFIRMED from all four seminars |
| Simulation starter package (`41068_ignition_bringup`) | CONFIRMED from simulation seminar |
| Autonomy stack architecture | CONFIRMED — repeated across seminars |
| Perception, mapping, SLAM | CONFIRMED from perception seminar |
| Decision making algorithms | CONFIRMED from decision making seminar |
| Path planning algorithms & Nav2 | CONFIRMED from path planning seminar |

### What this document does NOT cover

- Individual student project specifications (unknown — each team defines their own)
- Workspace install paths on a specific machine (not in source material; README may differ per student)
- Low-level motor control (deferred to "future seminar" in course hierarchy)

**Related source files:**
- `knowldge/simulation_package_llm_optimised.md`
- `knowldge/perception_mapping_llm_optimised.md`
- `knowldge/decision_making_llm_optimised.md`
- `knowldge/path_planning_llm_optimised.md`

---

## 1. Executive Context

### Identity

| Field | Value |
|---|---|
| **Course** | 41068 Robotics Studio 1 |
| **Institution** | University of Technology Sydney (UTS) — Faculty of Engineering and Information Technology |
| **Instructor** | Graeme Best |
| **Package** | `41068_ignition_bringup` (ROS 2 simulation starter) |
| **Simulator** | Ignition Gazebo |
| **Visualisation** | RViz |
| **Navigation stack** | Nav2 (default ROS 2 navigation) |
| **ROS distribution referenced** | ROS 2 Humble (`docs.ros.org/en/humble/`) |

### One-paragraph summary

41068 is a robotics studio course where student teams build autonomous robot systems in simulation. The supplied `41068_ignition_bringup` package provides a forest Gazebo world, two namespaced robots (Husky UGV and Parrot UAV), SLAM, Nav2, and example autonomy scripts. Projects must implement **perception/mapping**, **decision making**, **path planning**, and a **user interface**. The autonomy stack follows a five-layer hierarchy: Map Processor → Behavior Executive → Global Planner → Local Planner → Controller. Seminars teach the theory and tools; students adapt the starter package to their application.

### Seminar sequence (by presentation date)

| Date | Seminar | Focus |
|---|---|---|
| 2025-08-20 | Decision Making | TSP, exploration, informative path planning, MCTS, behaviour trees |
| 2026-08-06 | Simulation Package | ROS 2 / Gazebo / RViz, package structure, launch files |
| 2026-08-13 | Perception & Mapping | Sensors, map types, occupancy grids, Bayesian filtering, SLAM |
| 2026-08-27 | Path Planning | C-space, potential fields, graph search, PRM, RRT, Nav2 |

### Core autonomy equation

$$\text{action} = f(\text{state})$$

- **State** examples: location, map
- **Action** examples: "move over there"
- **Planner**: function mapping state → action

**Source:** All four seminars (intro slides)

---

## 2. Current State

> This section describes what is **established by course material** as the working baseline — not a specific student's project progress.

### Active baseline (CONFIRMED)

| Component | State |
|---|---|
| Simulation package | `41068_ignition_bringup` — working starter, must be adapted per project |
| Robots | Husky (`/husky1`) and Parrot (`/parrot1`) — always namespaced |
| Worlds | `simple_trees.sdf` (basic), `large_demo.sdf` (richer forest) |
| SLAM | SLAM Toolbox — namespaced per robot |
| Navigation | Nav2 — namespaced per robot; plugins swappable |
| Example scripts | `dynamic_world_demo.py`, `basic_autonomy_demo.py` |
| RViz configs | e.g. `41068_husky1.rviz`, `41068_parrot1.rviz` |

### Project requirements (CONFIRMED — mandatory for all 41068 projects)

Every project **must** include:

1. **Perception / mapping** — robot builds or uses maps from sensors
2. **Decision making** — robot decides where to go next (objective + algorithm)
3. **Path planning** — robot navigates the environment (likely Nav2; understanding/modification expected)
4. **User interface** — display maps and relevant information to users

### Expected student workflow (CONFIRMED)

1. Follow `README.md` — build workspace, launch simple config first
2. Confirm simulation runs without error; drive robot
3. Try more complex configurations
4. Preserve working state in Git
5. Adapt environment, robots, sensors, autonomy to project needs
6. Start with **one robot** before multi-robot

### Pending / student-defined (not in source material)

- Specific project objective and algorithms per team
- Custom map types beyond occupancy grids
- Custom decision-making implementation details
- User interface design and technology choices

---

## 3. Autonomy System Architecture

> **Canonical reference** — consolidated from decision making and path planning seminars. Duplicated hierarchy slides merged into one authoritative description.

### Five-layer stack

```
SLAM output ──┐
              ├──► Map Processor ──► maps ──┬──► Global Planner ──► path ──┐
other robots' │         │ stuck             │         ▲ conditions          │
maps ─────────┘         │                     │         │                     ├──► Local Planner ──► trajectory ──► Controller ──► motor commands
                        │                     │         │                     │         ▲ conditions
                        └── communicate ──────┘         │                     │         │
                              to other robots           │                     │         │
                                                        ▼                     │         │
                                              Behavior Executive ── actions ──┴─────────┘
                                                        ▲
                                                   conditions
```

### Component reference

| Component | Role | Inputs | Outputs |
|---|---|---|---|
| **Map Processor** | Fuse SLAM and multi-robot maps | SLAM output, other robots' maps | `maps`, `stuck` signal; communicates maps to other robots |
| **Behavior Executive** | High-level task switching — "Where do I go next?" | `stuck`, conditions from planners | `actions` to Global and Local Planners |
| **Global Planner** | Coarse path over map | `maps`, `actions` | `path`; `conditions` feedback |
| **Local Planner** | Feasible short-range trajectory | `maps`, `actions`, `path` | `trajectory`; `conditions` feedback |
| **Controller** | Execute motion | `trajectory` | `motor commands` |

### Seminar coverage mapping (HISTORICAL progression)

| Layer | Seminar status |
|---|---|
| Map Processor | Previous seminar (perception/mapping) |
| Behavior Executive | Decision making seminar — "Where do I go next?" |
| Global Planner + Local Planner | Path planning seminar — "How do I get there?" |
| Controller | Future seminar (not covered in supplied material) |

### Coupled sensing and planning

Robot alternates:
1. **Sensing** — make observation at current location
2. **Planning** — move to new location
3. Repeat

**Source:** `decision_making_llm_optimised.md`

### ROS node-level architecture (simplified)

From simulation seminar — typical data flow:

| Module | Nodes / topics (examples) |
|---|---|
| Sensing | Lidar driver → `scan`; Camera driver → `Image`; GPS driver |
| Perception | Object detection → object location |
| Mapping | Mapping node → `map` |
| Planning | Obstacle planning → `Path`, `Waypoint`, `Goal` |
| Control | Controller → motor commands |
| Decision making | Selects goals/tasks |
| User interface | Displays maps, goals, status |

**Source:** `simulation_package_llm_optimised.md`

---

## 4. Education & Academic Context

### Course structure

- **Subject code:** 41068 Robotics Studio 1
- **Prerequisite reference:** 41012 PFMS and related subjects (ROS familiarity expected)
- **Learning mode:** Seminar slides + hands-on package; ROS tutorials are **not** replaced by seminars

### Assessment-relevant project questions (consolidated)

**Simulation (`simulation_package`):**
- How will you adapt the simulation environment?
- How will you adapt robots and sensors?
- How will you adapt and develop autonomy?
- How will you test and evaluate the system?

**Perception / mapping (`perception_mapping`):**
- What types of maps will the robot produce? (primary task vs autonomy)
- What information will maps contain?
- How will maps be shown in the user interface?

**Decision making (`decision_making`):**
- What is the decision-making objective?
- What algorithms will solve the problem?
- How will algorithms connect to the rest of the system?

**Path planning (`path_planning`):**
- How will the robot navigate? (likely Nav2)
- How does Nav2 work in your system?
- How can you modify it — or implement planning yourself?

### Expectations (CONFIRMED)

- Starter package is a **starting point only** — must be changed for project relevance
- Not required to use every supplied component
- Multi-robot is optional; **one robot first**
- More complexity ≠ better; slows simulation
- May use completely different simulation setup if justified

---

## 5. Simulation Package — `41068_ignition_bringup`

> **Source:** `simulation_package_llm_optimised.md`

### Package contents

```
41068_ignition_bringup/
├── config/          # configuration and parameter files
├── launch/          # launch file combinations
├── models/          # custom reusable Gazebo environment models
├── scripts/         # two ROS Python examples
├── urdf_husky/      # Husky UGV robot, sensors, Gazebo plugins
├── urdf_parrot/     # Parrot UAV robot, sensors, Gazebo plugins
├── worlds/          # Gazebo world files (SDF)
├── CMakeLists.txt
├── package.xml
└── README.md        # installation and launch instructions
```

### Launch files

| File | Purpose |
|---|---|
| `41068_ignition.launch.py` | **Canonical** main launch file |
| `41068_ignition_husky.launch.py` | Husky only (wraps main) |
| `41068_ignition_parrot.launch.py` | Parrot only (wraps main) |
| `41068_navigation.launch.py` | Namespaced SLAM + Nav2 helper (don't call directly) |
| `41068_dynamic_world_demo.launch.py` | Dynamic world node only (main must be running) |
| `41068_autonomy_demo.launch.py` | Example autonomy node only (main must be running) |

### Main launch options (`41068_ignition.launch.py`)

| Argument | Values |
|---|---|
| `husky` | `true` / `false` |
| `parrot` | `true` / `false` |
| `slam` | `true` / `false` |
| `nav2` | `true` / `false` |
| `rviz` | `true` / `false` |
| `world` | e.g. `simple_trees` / `large_demo` |

### Main launch sequence

1. Start Gazebo world; bridge simulation clock
2. Generate and spawn each enabled robot
3. Start robot state publishing, localisation, robot-specific Gazebo bridge
4. Optionally start namespaced SLAM and Nav2 per robot
5. Optionally start one namespaced RViz per robot

### Namespaces and TF (CONFIRMED — always namespaced)

| | Husky | Parrot |
|---|---|---|
| ROS namespace | `/husky1` | `/parrot1` |
| Example topics | `/husky1/scan`, `/husky1/camera/image`, `/husky1/cmd_vel` | `/parrot1/scan`, `/parrot1/camera/image`, `/parrot1/cmd_vel` |
| TF frame prefix | `husky1_` | `parrot1_` |
| TF frames | `husky1_map`, `husky1_odom`, `husky1_camera_link` | `parrot1_map`, `parrot1_odom`, `parrot1_camera_link` |
| Gazebo model | `husky1` | `parrot1` |

### Robots

| Property | Husky (UGV) | Parrot (UAV) |
|---|---|---|
| Type | Four-wheeled ground robot | Visually quadrotor; simplified |
| Control | Differential drive | Husky-like planar movement |
| Odometry | Simulated | Simulated |
| IMU | Yes | Yes |
| Lidar | GPU lidar | GPU lidar |
| Camera | RGB-D | RGB-D |
| Gravity | Normal | **Disabled** |
| Altitude | Ground | **Fixed altitude** |
| Collisions | Normal | **Disabled** (navigate through leaves) |

### World files

| File | Description |
|---|---|
| `simple_trees.sdf` | Simple grass plane, few trees — verify basic sim works |
| `large_demo.sdf` | Forest texture, boundary walls, trees/rocks/objects, dynamic demo objects |

### Config file categories

| Config area | Contents |
|---|---|
| Gazebo-ROS bridges | Global clock, Husky/Parrot topics |
| Robot localisation | Localisation node parameters |
| SLAM toolbox | SLAM node parameters |
| Nav2 | Navigation, planning, control parameters — see `config/nav_params.yaml` |
| RViz | Visualisation config — best edited in RViz then saved |

### Robot model files

- **`URDF.xacro`**: links, joints, visual/collision geometry, inertial properties, sensor frames, macros
- **`Gazebo.xacro`**: simulated sensors, Gazebo topic/model names, drive/controller plugins

### Example scripts

**`dynamic_world_demo.py`**
- Changes Gazebo world parameters from Python (position, colour, visibility, lighting)
- Moves animal marker randomly in forest
- Cycles tree: healthy → fire → burnt

**`basic_autonomy_demo.py`**
- Skeleton autonomy connecting perception, mapping, navigation
- **Not** intended as a good algorithm — demonstrates ROS interface wiring
- Subscribes to map + RGB image
- Estimates image brightness; uses TF for robot pose in map
- Samples waypoint in free space: brighter image → distant goal; darker → closer goal
- Sends waypoint to Nav2; picks new goal when reached

### Expected running simulation

When launched correctly:
- **Gazebo:** forest world, 2 robots, dynamic objects
- **RViz UGV:** occupancy map (explored=white, obstacles=black/red), camera view
- **RViz UAV:** map + camera showing UGV and dynamic objects from above

### Dimensions students may change

Environment, dynamic features, robot, sensors, autonomy (perception, decision making, UI), multi-robot coordination — choose what supports the project.

### SDF / procedural worlds

- Format: SDF (Simulation Description Format) — XML-based — https://sdformat.org/spec/
- Tags: `world`, `model`, `link`, `joint`, `pose`, `visual`, `collision`, `material`, `geometry`, `sensor`, `plugin`, `light`, `physics`
- Can generate/modify worlds programmatically via XML parser
- Models from Gazebo Fuel: https://app.gazebosim.org/fuel/models
- Custom models in package `models/` directory

---

## 6. ROS 2, Gazebo, and RViz

> **Source:** `simulation_package_llm_optimised.md`

### ROS 2 definition

ROS 2 is a collection of software libraries, communication tools, and conventions. Systems are composed of many separate components exchanging data via interfaces — not one monolithic program.

### Key ROS concepts

| Concept | Description |
|---|---|
| **Nodes** | Running software components |
| **Topics** | Continuous data streams |
| **Messages** | Data structure on topics |
| **Parameters** | Node configuration values |
| **Services** | Request-response interactions |
| **Actions** | Long-running goals with feedback and results |
| **TF** | Coordinate frames and transformations |
| **Launch files** | Start/configure node collections |

### Tool roles

| Tool | Role |
|---|---|
| **Ignition Gazebo** | Simulates environment, robot models, collision/physics, joints/actuators, cameras/lidar/IMUs |
| **ROS** | Communication: topics, actions, services, parameters; autonomy stack |
| **RViz** | Visualises robot geometry, sensor data, maps, paths, TF frames, navigation goals |

### Learning resources

| Resource | URL |
|---|---|
| ROS 2 Humble tutorials | https://docs.ros.org/en/humble/Tutorials.html |
| Gazebo | https://gazebosim.org/home |
| Nav2 | https://docs.nav2.org/ |
| Turtlebot4 manual | https://turtlebot.github.io/turtlebot4-user-manual/ |
| Gazebo Fuel models | https://app.gazebosim.org/fuel/models |
| SDF spec | https://sdformat.org/spec/ |
| Python XML parser | https://docs.python.org/3/library/xml.etree.elementtree.html |

---

## 7. Perception and Mapping

> **Source:** `perception_mapping_llm_optimised.md`

### Definition

**Robot mapping** is the process by which a robot builds a representation of its environment using sensor data (and prior information). The map captures spatial information about surroundings.

### Why mapping is challenging

- Noisy sensors
- Local ↔ global coordinate transforms
- Robot motion during mapping
- Environment may change over time
- Many representations with different trade-offs

### Sensor fundamentals

A **sensor** measures a physical quantity and converts it to an electrical signal.

| Measurement domain | Examples |
|---|---|
| Temperature, distance, force, speed, sound, light | Various transducers |

**LIDAR** (Light Detection and Ranging):
- Measures time of flight of laser pulses
- Beam deflected by internal rotating mirror
- Real-time data via interface

### Application examples

- Fruit tree modelling (agriculture)
- Ocean monitoring
- Subterranean mapping (caves/tunnels)

### Map category taxonomy

| Category | Property | Examples |
|---|---|---|
| **Metric** | Accurately represents distance | Occupancy grids, terrain maps, distance function maps |
| **Topological** | Accurately represents connectivity | Train networks, navigation graphs, PRM graphs |
| **Semantic** | Associates descriptive meaning with locations | "Go to office", labelled park maps |

Maps may combine categories.

### Map representation types

| Type | Purpose | Key properties |
|---|---|---|
| **Occupancy grid** | Navigation | 2D cells / 3D voxels; occupied/free; rigid grid; memory-intensive; LIDAR ray casting; standard ROS packages |
| **Terrain map** | Complex terrain navigation | Elevation, surface type; contour maps |
| **Distance function map** | Safe navigation, fast ray tracing | Distance to occupied space; dense grid/voxels; expensive to compute |
| **Feature map** | Localisation (SLAM) | Landmarks; compact; multiple observations improve pose |
| **Trajectory tracking** | Moving objects | Landmarks change over time; whale GPS tracks, pedestrian tracking |
| **Topological map** | Connectivity | Graph of vertices/edges; may not preserve metric distances |
| **Semantic map** | HRI, high-level commands | Meaningful labels per location |
| **Environmental phenomena** | Continuous fields | Temperature, elevation, soil pH, wireless signal strength; often modelled with Gaussian Processes |

### Subterranean exploration map types (REFERENCE — earlier seminar example)

| Map type | Features |
|---|---|
| Exploration maps | OpenVDB structure; LiDAR/camera labels; dust filtering; repairs for changes; shared across team; GPU |
| Navigation roadmap | Connect visited locations; shared across team |
| Local planner map | Distance transform; computed on GPU |

### Occupancy grid — Bayesian approach

**Cell model:** Each cell is a binary random variable (occupied / free).

| State | Probability |
|---|---|
| Occupied | $p(m) = 1$ |
| Free | $p(m) = 0$ |
| Unknown | $p(m) = 0.5$ |

**Grid structure:** Discrete array $m_{x,y} \in \{0, 1\}$ (free/occupied).

**Estimation problem:**

Given sensor data $z_{1:t}$ and poses $x_{1:t}$:

$$p(m \mid z_{1:t}, x_{1:t}) = \prod_i p(m_i \mid z_{1:t}, x_{1:t})$$

→ **Binary Bayes filter** (static state per cell).

**Bayes update:**

$$p(m \mid z) = \frac{p(z \mid m)\, p(m)}{p(z)}$$

**Range observation (5 m example):**
- Cells along ray → more likely **free**
- Cell at measured distance → more likely **occupied**
- Uses measurement model for noisy sensors (e.g. sonar)

**Recursive filter:** Prior belief → measurement update → posterior belief → repeat as robot moves.

### SLAM

| Statement | Reality |
|---|---|
| "If we have a map, we can localise" | True in principle |
| "If we can localise, we can make a map" | True in principle |
| Combined | **NOT THAT SIMPLE** — chicken-and-egg problem; requires SLAM |

### ROS packages for mapping

- Nav2 mapping/localisation guide: https://docs.nav2.org/setup_guides/sensors/mapping_localization.html
- Used in 41068 starter package for **basic occupancy grid mapping only**
- Other map types (terrain, semantic, etc.) — must implement separately or find other packages
- **OctoMap**: 3D hierarchical mapping; tree structure; multi-resolution queries

---

## 8. Decision Making

> **Source:** `decision_making_llm_optimised.md`

### Definition

**Robot decision making** is choosing what the robot should do next based on current knowledge, goals, and environment. It selects the next **task, goal, or behaviour** — not just how to execute a specific movement.

### Why decision making is challenging

- Defining the right objective
- Incomplete/uncertain information (noisy sensors)
- Interaction with lower-level planners and controllers
- Computational difficulty (many options)
- Algorithm selection and parameter tuning
- Multi-robot coordination

### Topic areas covered

1. Travelling Salesman Problem (TSP) and variants
2. Exploration
3. Informative path planning
4. Monte Carlo Tree Search (MCTS)
5. Behaviour trees

---

### 8.1 Travelling Salesman Problem (TSP)

**Problem:** Given locations and distances, find shortest route visiting each exactly once and returning to origin.

| Property | Detail |
|---|---|
| Use in robotics | Useful approximation for routing problems |
| Complexity | NP-hard |

**Variants / related:**
- **Vehicle routing problems** — Mars rover missions, oil rig inspection, delivery routing
- **Orienteering problem** — Given max route length, visit as many locations as possible

**Algorithms:**

| Algorithm | Type | Complexity / quality |
|---|---|---|
| Brute force | Optimal | $O(n!)$ — try all permutations |
| Nearest neighbour | Greedy | Fast; locally optimal at each step |
| 2-Opt | Improvement | Start from NN solution; iteratively swap edge pairs |

**Packages:**
- `python_tsp`: https://pypi.org/project/python_tsp/
- Coverage planning: https://fields2cover.github.io/

---

### 8.2 Exploration

**Aim:** Construct a map of the world.

**Problem:** Given current map and sensor model, where should the robot move to quickly build a full map?

**Frontier-based exploration:**
- Move to boundary between **free** and **unknown** space ("frontiers")
- Gain maximum new information per move

**Frontier selection criteria:**
- Closest first (baseline)
- Also consider: path distance, safety, frontier size, information beyond frontier, coordination value, combinations
- Then use **path planning** to reach selected frontier

**Advanced exploration pipeline:**
1. Frontier generation — (a) vision/coverage, (b) range/exploration
2. Viewpoint extraction
3. Viewpoint ranking — distance, momentum change, coordination value
4. RRT-Connect planner

**Packages (experimental — significant integration work):**
- Recommended first: https://github.com/robo-friends/m-explore-ros2
- Tutorial: https://husarion.com/tutorials/ros2-tutorials/10-exploration/
- Alternatives: `Autonomous-Explorer-and-Mapper-ros2-nav2`, `nav2_wavefront_frontier_exploration`

---

### 8.3 Informative Path Planning

**Concept:** Observe/estimate a **quantity of interest** while reducing uncertainty.

| Quantity examples |
|---|
| Map, object location/type, temperature field, soil pH, wireless signal strength |

**Problem:** Given current belief, where should the robot move to quickly **reduce uncertainty**?

**Uncertainty quantification:**
- Random variable $X$; $p(x_i)$ = probability $X = x_i$
- **Entropy** measures uncertainty (Bernoulli: highest at $p = 0.5$)

**Information-based exploration:**
- Alternative to frontier-based; mathematically principled but slower
- Choose action that maximally reduces uncertainty (often entropy of occupancy grid)

**Field estimation:**
- Gaussian Processes — probability distributions over functions
- Package: https://scikit-learn.org/stable/modules/gaussian_process.html

**Algorithms for informative planning:**

| Algorithm | Strategy |
|---|---|
| Greedy | Move to highest-uncertainty location |
| Branch and bound | Tree search with quality bounds |
| Sampling-based | RRT over information field |
| MCTS | See below |

---

### 8.4 Monte Carlo Tree Search (MCTS)

**Properties:**
- Biased random sampling; exploits search space smoothness
- Any-time algorithm
- Evaluates full paths; supports heuristics
- Theoretical convergence guarantees
- Popular in robotics and games (e.g. AlphaGo — Silver, 2017)

**Tree search goal:** From current state, find best action sequence (green = high reward).

**Algorithm cycle:**
1. **Selection** — traverse tree (tree policy)
2. **Expansion** — add child node
3. **Simulation** — rollout to terminal (default policy)
4. **Backpropagation** — update visit counts and rewards

**UCB1 (Upper Confidence Bound):** Balances exploitation (high average reward) vs exploration (less-visited children). Uses parent/child visit counts.

**Implementation checklist:**
| Question | Define for your problem |
|---|---|
| State | What is a state? |
| Actions | What actions are available per state? |
| Transition | What is the next state per action? |
| Reward | Reward for complete action sequences (scale ~0–1) |

**Tuning parameters:**
- Planning horizon (tree size)
- Rollout policy: random or greedy
- Exploration-exploitation parameter (example value: 2)

**Debugging:**
- Observe solutions; plot tree; verify convergence
- Reward evaluation is slowest — profile and cache

**Dec-MCTS (multi-robot):**
- (a) Each robot grows search tree for own actions asynchronously
- (b) Decentralised optimisation of probability distributions
- (c) Communicate distributions between robots

---

### 8.5 Behaviour Trees

**Use case:** Multi-task missions (e.g. DARPA Subterranean Challenge: explore, avoid hazard, drill, scan rocks).

**Node types:**

| Node | Behaviour |
|---|---|
| **Sequence** | Activate children left-to-right; fail on first failure |
| **Fallback** | Activate children left-to-right; succeed on first success |
| **Conditions** | Return success or failure |
| **Actions** | Execute behaviours; return success or failure |

**Example mission tree (simplified):**
- Root Fallback → Sequence: Diagnostics → Takeoff → Exploration → Landing
- Branches: Return Home, Emergency Land, Rewind
- Conditions: Has Unvisited Frontiers, Stuck, Critical Battery, Unreachable Goal
- Actions: Coordinated Explore, Roadmap Explore, Independent Explore

**Packages:**
- Nav2 behaviour tree: https://docs.nav2.org/behavior_trees/overview/detailed_behavior_tree_walkthrough.html
- BehaviorTree.CPP ROS 2: https://www.behaviortree.dev/docs/ros2_integration/
- https://github.com/BehaviorTree/BehaviorTree.ROS2
- Also consider: Finite State Machines

---

## 9. Path Planning

> **Source:** `path_planning_llm_optimised.md`

### Definition

**Path planning:** Given map, current robot location, and goal — find a trajectory that reaches the goal when executed.

**Scope:** How the robot moves from current state to target given map, constraints, uncertainties. Includes feasible/safe paths, obstacle avoidance, kinematic/dynamic limits, non-holonomic constraints, replanning.

### Why path planning is challenging

- Objective trade-offs: time, speed, safety, comfort
- Incomplete/uncertain sensor information
- Coupling with high-level decision making and low-level control
- Computational difficulty
- Algorithm selection and tuning
- Multi-robot collision avoidance

---

### 9.1 Configuration space (C-space)

- Motion planning defined in **workspace** but solved in **configuration space**
- Configuration $q$ = position of all robot points relative to fixed frame (vector of positions/orientations)
- C-space topology is usually **not** Cartesian
- **C-free** = collision-free configurations

**Motion models:** industrial arm, omnidirectional, Dubins car, unicycle

### Method hierarchy

Combining three approaches:

| Method | Role | Limitation |
|---|---|---|
| **Feedback control** | Reactive motion toward goal | Fails on global obstacles (local minima) |
| **Trajectory optimization** | Smoothing | Locally optimal; may not find global path |
| **Path finding** | Global search | Needed for tricky environments |

---

### 9.2 Artificial Potential Fields

- Guide robot from start to goal while staying in C-free
- Analogous to electric potential field
- **Attractive** potential at goal + **repulsive** potential at obstacles
- Robot follows gradient: $U(q) = U_{att}(q) + U_{rep}(q)$

**Failure mode:** Local minima — robot stuck between obstacles.

**Mitigations:**
- Random walk when stuck
- Potential fields without local minima (difficult)
- Combine with global path planning

---

### 9.3 Graph search methods

**Pipeline:**
1. Discretise C-space → graph (grid cells, visibility graph, probabilistic roadmap)
2. Graph search
3. Extract shortest path

**Graph theory essentials:**
- $G = (V, E)$; edge $e = (v, w)$; directed/undirected; weighted
- Path: vertex sequence; connected graph has path between any two vertices
- Representations: adjacency matrix (dense), adjacency list (sparse)

**Cell decomposition:**
1. Create grid
2. Remove cells intersecting obstacles
3. Connect graph over cells
4. Plan shortest path
5. If no plan: double resolution and retry

**Wavefront algorithm:**
1. Discretise map (cell decomposition)
2. Add start (S) and goal (G)
3. Fill wavefront table from G outward
4. Trace path from S following decreasing values

**Shortest path problem:** Given weighted graph and vertices $s$, $g$ — find minimum-cost path.

**Dijkstra's algorithm:**
- Visit lowest-cost unvisited vertex
- Maintain visited (red) and unvisited (blue) sets
- Each node: path cost + parent pointer
- Repeat until goal found or unvisited empty

**A* algorithm:**
- Faster than Dijkstra in typical cases with good heuristic
- $f(x) = g(x) + h(x)$
  - $g(x)$ = cost from start to $x$
  - $h(x)$ = heuristic estimate from $x$ to goal
- Heuristic must be **admissible** (never overestimate) and **consistent**
- Equivalent to Dijkstra when $h(x) = 0$

---

### 9.4 Sampling-based motion planning

**Monte Carlo / randomised algorithms:** Replace exhaustive search with random sampling.

| Application | Algorithm |
|---|---|
| Localisation | Particle filter |
| Path planning | PRM, RRT |
| Decision making | MCTS (previous seminar) |

**Probabilistic Roadmap (PRM):**
1. Sample C-space randomly
2. Create nodes at collision-free samples
3. Connect nearby nodes with local paths
4. Search with Dijkstra or A*

| PRM advantages | PRM disadvantages |
|---|---|
| Probabilistically complete | Not optimal, not complete |
| No explicit C-space construction | Doesn't work for all problems |
| Scales to high dimensions | |

**RRT (Rapidly-exploring Random Tree):**
1. Initialise tree at start
2. Draw random sample in C-space
3. Find nearest tree node
4. Steer toward sample (step size $\Delta q$)
5. Insert new node if collision-free
6. Repeat until goal reached

**RRT*:** Adds **rewiring** step → asymptotically optimal paths.

**Path smoothing (post-processing):**
- Randomised planners produce jagged paths
- **Short-cutting:** connect non-adjacent nodes if collision-free
- **Non-linear optimisation:** smoothness objective

---

### 9.5 Nav2 Navigation Stack

**Status:** Default ROS 2 navigation — extensively used in research and industry.

**Capabilities:**
- Point-to-point, waypoint sequences, object following, coverage
- Integrates perception, planning, control, localisation, visualisation
- Environmental model from sensor data; dynamic path planning; velocity commands; obstacle avoidance; high-level behaviours via behaviour trees
- **Highly customisable** — all plugins swappable

**Architecture (plugins swappable):**

| Server / component | Role |
|---|---|
| BT Navigator Server | Behaviour tree navigation |
| Controller Server | Local control |
| Planner Server | Global planning |
| Behavior Server | Recovery behaviours |
| Smoother Server | Path smoothing |
| Waypoint Follower | Multi-waypoint missions |
| Global / Local Costmap | Obstacle representation |
| Route Server | Route planning |
| Velocity Smoother | Output smoothing |
| Collision Monitor | Safety layer |

**Configuration:** `config/nav_params.yaml` in 41068 package.

**Commander API:** Interact with Nav2 via **actions** (request/response, goals, results).
- Overview: https://docs.nav2.org/commander_api/index.html
- Examples: https://github.com/ros-navigation/navigation2/tree/main/nav2_simple_commander
- ROS 2 actions tutorial: https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html

**Documentation:**
- https://docs.nav2.org/
- Video overview: https://www.youtube.com/watch?v=QB7lOKp3ZDQ
- Config guide: https://docs.nav2.org/configuration/index.html
- Paper: https://arxiv.org/pdf/2003.00368
- Getting started: https://docs.nav2.org/getting_started/index.html

---

## 10. Algorithm Quick Reference

| Problem | Algorithms | Packages / tools |
|---|---|---|
| Route planning (visit all points) | Brute force, nearest neighbour, 2-opt | `python_tsp`, Fields2Cover |
| Orienteering (max visits, length limit) | Variants of TSP | — |
| Map exploration | Frontier-based, information-based | `m-explore-ros2`, wavefront frontier |
| Informative sensing | Greedy, branch-and-bound, RRT, MCTS | scikit-learn GP |
| High-level decisions | MCTS, Dec-MCTS, behaviour trees | Nav2 BT, BehaviorTree.ROS2 |
| Reactive navigation | Potential fields | — |
| Grid planning | Wavefront, Dijkstra, A* | — |
| High-D planning | PRM, RRT, RRT* | — |
| ROS navigation | Nav2 stack | `nav2_simple_commander` |
| Occupancy mapping | Binary Bayes filter | SLAM Toolbox, Nav2 |
| 3D mapping | OctoMap | OctoMap library |

---

## 11. Requirements & Constraints

### Mandatory project components (CONFIRMED)

| Component | Requirement |
|---|---|
| Perception / mapping | Required |
| Decision making | Required — define objective and algorithm |
| Path planning | Required — likely Nav2; understand and optionally modify |
| User interface | Required — display maps and project-relevant information |

### Technical constraints (CONFIRMED)

| Constraint | Detail |
|---|---|
| Namespacing | Robots always namespaced even for single-robot runs |
| Starter package scope | Basic occupancy grid only via supplied SLAM; other map types self-implemented |
| Simulation performance | Complexity increases slow simulation — avoid unnecessary complexity |
| Multi-robot | Optional; start with one robot |
| Example autonomy | `basic_autonomy_demo.py` is intentionally naive — replace for real projects |
| Parrot UAV | Fixed altitude, no gravity, no collisions — simplified model |

### Design recommendations (REFERENCE — not mandatory)

- Incrementally add world/model complexity
- Preserve working Git state before modifications
- Launch simple config before `large_demo`
- Edit RViz config interactively then save
- Frontier exploration: try `m-explore-ros2` first

---

## 12. Terminology

| Canonical term | Alternatives | Meaning |
|---|---|---|
| ROS 2 Humble | ROS2 Humble | ROS 2 distribution referenced in course materials |
| Ignition Gazebo | Gazebo, Gazebo Sim | Physics simulator used in 41068 package |
| Nav2 | Navigation2 | Default ROS 2 navigation stack |
| C-space | Configuration space | Space of robot configurations $q$ |
| C-free | Free configuration space | Collision-free configurations |
| SLAM | Simultaneous Localisation and Mapping | Jointly estimate map and pose |
| Occupancy grid | — | Metric map; cells occupied/free/unknown |
| Frontier | — | Boundary between known free and unknown space |
| PRM | Probabilistic Roadmap | Sampling-based roadmap planner |
| RRT | Rapidly-exploring Random Tree | Sampling-based tree planner |
| RRT* | RRT star | RRT with rewiring; asymptotically optimal |
| MCTS | Monte Carlo Tree Search | Tree search with random rollouts |
| Dec-MCTS | — | Decentralised MCTS for multi-robot |
| BT | Behaviour Tree | Hierarchical task switching structure |
| TSP | Travelling Salesman Problem | Visit all locations minimum cost |
| UCB | Upper Confidence Bound | Exploration-exploitation in MCTS |
| SDF | Simulation Description Format | Gazebo world file format (XML) |
| UGV | Husky | Ground robot in starter package |
| UAV | Parrot | Aerial robot in starter package (simplified) |
| TF | Transform | ROS coordinate frame tree |

---

## 13. Important URLs

### Course package

| Resource | URL |
|---|---|
| Gazebo Fuel | https://app.gazebosim.org/fuel/models |
| SDF format | https://sdformat.org/spec/ |

### ROS / navigation

| Resource | URL |
|---|---|
| ROS 2 Humble tutorials | https://docs.ros.org/en/humble/Tutorials.html |
| Nav2 docs | https://docs.nav2.org/ |
| Nav2 mapping/localisation | https://docs.nav2.org/setup_guides/sensors/mapping_localization.html |
| Nav2 behaviour trees | https://docs.nav2.org/behavior_trees/overview/detailed_behavior_tree_walkthrough.html |
| Nav2 Commander API | https://docs.nav2.org/commander_api/index.html |
| nav2_simple_commander | https://github.com/ros-navigation/navigation2/tree/main/nav2_simple_commander |
| ROS 2 actions tutorial | https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html |

### Decision making / exploration

| Resource | URL |
|---|---|
| python_tsp | https://pypi.org/project/python_tsp/ |
| Fields2Cover | https://fields2cover.github.io/ |
| m-explore-ros2 | https://github.com/robo-friends/m-explore-ros2 |
| Exploration tutorial | https://husarion.com/tutorials/ros2-tutorials/10-exploration/ |
| scikit-learn GP | https://scikit-learn.org/stable/modules/gaussian_process.html |
| BehaviorTree.ROS2 | https://github.com/BehaviorTree/BehaviorTree.ROS2 |

### External references

| Resource | URL |
|---|---|
| Mars Perseverance instruments | https://science.nasa.gov/mission/mars-2020-perseverance/science-instruments/ |
| Turtlebot4 manual | https://turtlebot.github.io/turtlebot4-user-manual/ |
| Nav2 paper | https://arxiv.org/pdf/2003.00368 |

---

## 14. Decisions & Rationale

> Course-level design decisions embedded in starter package and seminar structure.

| Decision | Rationale | Source |
|---|---|---|
| Always use namespaces | Enable multi-robot without topic collision | simulation_package |
| Start with one robot | Reduce debugging complexity | simulation_package |
| Use Nav2 as default navigation | Industry/research standard for ROS 2 mobile robots | path_planning |
| Provide naive autonomy demo | Teach ROS interface wiring, not good algorithms | simulation_package |
| Simplify Parrot UAV | Focus on autonomy not flight dynamics | simulation_package |
| SLAM Toolbox for mapping | Standard ROS 2 SLAM; basic occupancy grid | perception_mapping, simulation_package |
| Behaviour trees in Nav2 | Structured high-level task switching | decision_making, path_planning |

---

## 15. Open Questions & Pending Items

| Item | Status |
|---|---|
| Student project specific objectives | UNKNOWN — per team |
| Controller layer implementation | PENDING — future seminar |
| Which map types each project needs | PENDING — student design choice |
| Custom decision-making algorithm per project | PENDING — student design choice |
| Nav2 plugin customisation per project | PENDING — optional |

---

## 16. Conflicts / Ambiguities

| Topic | Notes | Confidence |
|---|---|---|
| Seminar date for decision making (2025 vs 2026) | Decision making PDF dated 2025-08-20; other seminars 2026. Likely same course offering, different years. Content is consistent. | Medium |
| ROS 2 actions tutorial URL | Uses Foxy path; course references Humble. API concept identical; verify Humble equivalent if needed. | Low impact |
| `world` launch argument | Documented as `world:=simple_trees/large_demo` — verify exact argument format in launch file before use. | Verify in `launch/41068_ignition.launch.py` |

No substantive contradictions found between the four seminar documents on technical content.

---

## 17. Reusable Knowledge Patterns

### Autonomy development pattern (INFERRED from course structure)

1. **Simulate** — get `41068_ignition_bringup` running (Gazebo + RViz)
2. **Perceive** — sensors → maps (occupancy grid via SLAM)
3. **Decide** — select goals/tasks (exploration, TSP, informative planning, BT, MCTS)
4. **Plan** — global + local path (Nav2 or custom)
5. **Act** — controller executes trajectory
6. **Interface** — present maps/status to user

### Integrating custom autonomy with Nav2

1. Subscribe to map, sensor topics, TF
2. Implement decision logic in ROS node
3. Send goals via Nav2 **actions** (Commander API or direct action client)
4. Monitor feedback; replan on completion or failure

### Occupancy grid update pattern

1. Initialise cells at $p(m) = 0.5$ (unknown)
2. For each sensor reading: compute $p(z \mid m)$ measurement model
3. Apply Bayes: $p(m \mid z) \propto p(z \mid m)\, p(m)$
4. Repeat per cell; iterate as robot moves

---

## 18. Source & Provenance Register

| Source file | Origin | Slides | Date | Primary topics |
|---|---|---:|---|---|
| `knowldge/simulation_package_llm_optimised.md` | `simulation_package.pdf` | 33 | 2026-08-06 | ROS 2, Gazebo, package structure, launch, robots, worlds |
| `knowldge/perception_mapping_llm_optimised.md` | `perception_mapping.pdf` | 43 | 2026-08-13 | Sensors, map types, occupancy grids, Bayes, SLAM |
| `knowldge/decision_making_llm_optimised.md` | `decision_making.pdf` | 57 | 2025-08-20 | TSP, exploration, informative planning, MCTS, BT |
| `knowldge/path_planning_llm_optimised.md` | `path_planning.pdf` | 69 | 2026-08-27 | C-space, potential fields, graph search, PRM, RRT, Nav2 |

**Author (all):** Graeme Best, UTS FEIT  
**Extraction method:** Multi-pass PDF extraction (PyMuPDF + Tesseract OCR) → per-file LLM-optimised Markdown → this synthesis  
**Extraction date (source files):** 2026-08-28

---

## 19. Change Log

| Date | Change |
|---|---|
| 2026-08-28 | Initial `MASTER_CONTEXT.md` synthesised from four seminar Markdown extractions |
